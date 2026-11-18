"""Inject simulated image coordinates into real visual control, then return with odometry.

Offline by default. Coordinates are derived from wheel/IMU odometry with an explicit
synthetic projection, delay and noise: this is a motor/controller test, NOT camera
or absolute-position validation. No synthetic values replace sensor/odometry RAM.
"""
from __future__ import annotations
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time
import swd_session as swd
from continuous_return_session import ContinuousSession, wrap
from uart_telemetry import UartTelemetry, visual_snapshot

VISUAL_SIZES = {"tuning_visual_center":8, "tuning_visual_sample":40,
                "tuning_visual_seq":4, "tuning_visual_time_ms":4, "tuning_visual_valid":4}

def image_center(pose, goal, frame, response=10000.0):
    dx,dy = goal[0]-pose[0], goal[1]-pose[1]
    c,s = math.cos(pose[2]), math.sin(pose[2])
    forward,left = c*dx+s*dy, -s*dx+c*dy
    # Matches production X_TO_LEFT=-1, Y_TO_FORWARD=+1. q is not metres.
    noise = -1.0 if frame % 2 else 1.0
    return [5000.0+left*response+15.0*noise, 7500.0-forward*response-25.0*noise]

def pose_of(state):
    sample=state["sample"]
    return [sample[k] for k in ("odom_x_m","odom_y_m","odom_yaw_rad")]

def visual_leg(session, goal, telemetry, speed=.2, spike=True, response=10000.):
    args=[*goal,speed,.5,16000.,0.,0.]
    swd.validate_request(7,args)  # Same bounded geometry, rates and duration.
    initial=session.validate_initial()
    if initial["phase"] not in (0,4) or initial["error"]:
        raise swd.SessionError("Visual leg requires idle or completed HOLD")
    telemetry.discard()
    seq=session.publish(8,args)
    start=time.monotonic()
    history=deque(maxlen=32)
    frame=0
    last_tick=None
    terminal=None
    while time.monotonic()-start < 19.0:
        session.beat()
        state=visual_snapshot(session,telemetry)
        session.log.emit("sample",**state)
        if state["command"]!=8:
            raise swd.SessionError("Visual command replaced")
        if state["error"] or state["phase"]==5:
            raise swd.SessionError(f"Visual firmware fault {state['error']}")
        sample=state["sample"]
        if state["ack_seq"]==seq and state["phase"]==4 and sample["phase"]==4:
            if terminal is not None and sample["tick_ms"]!=terminal:
                swd.validate_endpoint(8,args,state)
                session.log.emit("visual_complete_enabled",**state)
                return state
            terminal=sample["tick_ms"]
        if state["phase"] in (1,2) and sample["phase"] in (1,2) and sample["odom_valid"]==1:
            pose=pose_of(state)
            if math.hypot(pose[0]-session.start_pose[0],pose[1]-session.start_pose[1])>1.65:
                raise swd.SessionError("Visual test exceeded origin fence")
            tick=int(sample["tick_ms"])
            if tick!=last_tick:
                last_tick=tick
                history.append((time.monotonic(),pose,tick))
                old=history[0]
                for item in history:
                    if item[0] <= time.monotonic()-.060:
                        old=item
                raw=image_center(old[1],goal,frame,response)
                outlier=bool(spike and frame==25)
                if outlier: raw[1]+=1800.
                if not session._firmware_verified or not session._initial_validated:
                    raise swd.SessionError("Verified firmware required before frame injection")
                # One Tcl transaction avoids per-field TCP round trips; sequence is committed last.
                values=" ".join(str(swd.float_word(x)) for x in raw)
                sequence=(frame+1)&0xffffffff
                client=session.client
                addresses=session.symbols
                client.evaluate(f"write_memory {addresses['tuning_visual_center']:#x} 32 {{{values}}}; "
                                f"write_memory {addresses['tuning_visual_time_ms']:#x} 32 {{{tick & 0xffffffff}}}; "
                                f"write_memory {addresses['tuning_visual_valid']:#x} 32 {{1}}; "
                                f"write_memory {addresses['tuning_visual_seq']:#x} 32 {{{sequence}}}")
                session.log.emit("visual_frame",source_ms=tick,raw=raw,synthetic_pose=old[1],
                                 projection_q_per_m=response,noise_q=[15,25],outlier=outlier)
                frame+=1
        # USART6 waits for the next complete 40 ms frame. Extra SWD diagnostics
        # here can starve coordinate freshness on low-throughput DAP probes.
    raise swd.SessionError("Visual host deadline")

def run(session, telemetry, forward=.6,left=.1,turn_deg=0.):
    state=session.validate_initial()
    if state["phase"] not in (0,4):
        raise swd.SessionError("Board must be idle/HOLD")
    if state["sample"]["enabled"]==0:
        yaw=wrap(math.radians(state["sample"]["yaw_deg"]))
        session.execute(7,[state["sample"]["odom_x_m"],state["sample"]["odom_y_m"],yaw,.2,.5,5000.,0.,0.],8.)
        state=session.snapshot()
    if state["sample"]["odom_valid"] != 1 or any(state["sample"][f"target_{i}"] != 0 for i in (1,2,3)):
        raise swd.SessionError("Origin requires valid zero-target HOLD")
    origin=pose_of(state)
    session.start_pose=origin
    if turn_deg:
        session.execute(7,[*origin[:2],wrap(origin[2]+math.radians(turn_deg)),.2,.5,10000.,0.,0.],13.)
    heading=pose_of(session.snapshot())[2]
    c,s=math.cos(origin[2]),math.sin(origin[2])
    goal=[origin[0]+c*forward-s*left,origin[1]+s*forward+c*left,heading]
    session.log.emit("visual_return_plan",origin=origin,goal=goal,body_displacement=[forward,left],
                     synthetic_visual=True,real_camera=False,estimated_only=True,initial_turn_deg=turn_deg)
    projection=6000. if math.hypot(forward,left)>.8 else 10000.
    outbound=visual_leg(session,goal,telemetry,response=projection)
    returned=session.execute(7,[*origin,.2,.5,16000.,0.,0.],19.)
    time.sleep(.3)
    state=session.snapshot()
    swd.validate_endpoint(7,[*origin,.2,.5,16000.,0.,0.],state)
    end=pose_of(state)
    report=dict(origin=origin,goal=goal,final=end,
                outbound_error_m=math.hypot(outbound["sample"]["end_x_m"]-goal[0],outbound["sample"]["end_y_m"]-goal[1]),
                return_error_m=math.hypot(end[0]-origin[0],end[1]-origin[1]),
                return_yaw_error_deg=math.degrees(wrap(end[2]-origin[2])),
                shortest_return_m=math.hypot(outbound["sample"]["end_x_m"]-origin[0],outbound["sample"]["end_y_m"]-origin[1]),
                initial_turn_deg=turn_deg,outbound_ms=outbound["sample"]["end_elapsed_ms"],return_ms=returned["sample"]["end_elapsed_ms"],
                enabled=True,zero_targets=True,estimated_only=True,real_camera=False)
    session.log.emit("visual_return_complete",**report)
    return report

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute",action="store_true")
    parser.add_argument("--elf",type=Path)
    parser.add_argument("--log",type=Path)
    parser.add_argument("--stop-file",type=Path)
    parser.add_argument("--serial-port",help="PC port wired to STM32 USART6, e.g. COM7")
    parser.add_argument("--forward",type=float,default=.6)
    parser.add_argument("--left",type=float,default=.1)
    parser.add_argument("--turn-deg",type=float,default=0.)
    args=parser.parse_args()
    if not math.isfinite(args.forward) or not math.isfinite(args.left) or not .1<=args.forward<=1.2 or abs(args.left)>.6 or not math.isfinite(args.turn_deg) or abs(args.turn_deg)>60:
        parser.error("Forward .1..1.2m, left +/- .6m, initial turn +/-60deg required")
    if not args.execute:
        print(json.dumps(dict(offline=True,forward=args.forward,left=args.left,synthetic_visual=True,
                             outbound_command=8,return_command=7,initial_turn_deg=args.turn_deg,enabled_hold=True)))
        return 0
    if not args.elf or not args.log or not args.stop_file or not args.serial_port:
        parser.error("Execution requires exact ELF, new log, STOP path and USART6 serial port")
    if args.stop_file.exists(): parser.error("STOP file exists")
    with swd.writer_lock("127.0.0.1",6666):
        client=log=session=telemetry=None
        complete=False
        try:
            symbols = swd.load_symbols(args.elf, extra_sizes=VISUAL_SIZES)
            log=swd.JsonLog(args.log)
            client=swd.TclClient("127.0.0.1",6666)
            if client.evaluate("stm32f4x.cpu curstate")!="running":
                raise swd.SessionError("MCU not running")
            session=ContinuousSession(client,symbols,log,stop_file=args.stop_file)
            session.verify_firmware(args.elf)
            session.fence_m=1.8
            telemetry=UartTelemetry(args.serial_port,args.log.with_suffix(".uart.bin"),log)
            session.snapshot_reader=lambda current: visual_snapshot(current,telemetry)
            result=run(session,telemetry,args.forward,args.left,args.turn_deg)
            complete=True
            print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
        finally:
            try:
                if not complete and session and session._firmware_verified and session._initial_validated:
                    session.execute(1,[0.]*8,2.5)
            finally:
                if telemetry: telemetry.close()
                if client: client.close()
                if log: log.close()
    return 0

if __name__=="__main__":
    raise SystemExit(main())
