"""Run a 30- or 35-turn YAW-counted experiment. Offline unless --execute."""
import argparse
import json
import math
from pathlib import Path
import time
import swd_session as swd
from continuous_return_session import ContinuousSession
from uart_telemetry import UartTelemetry, visual_snapshot

SPIN_NAMES = swd.SAMPLE_NAMES + (
    'spin_encoder_rad','spin_yaw_rad','spin_gyro_rad','spin_requested_wz',
    'spin_stage','spin_target_rad','spin_yaw_minus_encoder_rad','spin_wheel_wz')
TURN_OPTIONS = (30, 35)


def make_plan(turns):
    if turns not in TURN_OPTIONS:
        raise ValueError('Supported spin totals are 30 and 35 turns')
    return dict(turns=turns, direction='CCW', stages_rad_s=[2,3,4,3,2], turns_per_stage=turns//5,
                ramp_rad_s2=1.6, counter='AHRS_YAW', rotation_radius_m=.0982,
                real_ground_angle_verified=False)


def check_sample(sample, start_xy, turns=30):
    if sample['error'] or sample['phase']==5:
        raise swd.SessionError(f"Spin firmware fault {sample['error']}")
    if sample['phase'] in (2,3,4) and sample['enabled']==1:
        if sample['odom_valid']!=1 or not 0<=sample['odom_age_ms']<=50:
            raise swd.SessionError('Invalid spin odometry')
        if math.hypot(sample['odom_x_m']-start_xy[0],sample['odom_y_m']-start_xy[1])>.35:
            raise swd.SessionError('Spin left the 0.35 m drift fence')
        if any(sample[f'state_{i}']!=1 for i in (1,2,3)):
            raise swd.SessionError('Spin motor state invalid')
    if (not 0<=sample['spin_stage']<=4 or
        not math.isclose(sample['spin_target_rad'],turns*math.tau,abs_tol=2e-5) or
        not -.2<=sample['spin_yaw_rad']<=turns*math.tau+.2):
        raise swd.SessionError('Spin progress invalid')


def check_complete(sample, start_xy, turns=30):
    check_sample(sample,start_xy,turns)
    if (sample['phase']!=4 or sample['enabled']!=1 or sample['end_stopped']!=1 or
        sample['end_valid']!=1 or abs(sample['spin_yaw_rad']-turns*math.tau)>.2 or
        any(sample[f'target_{i}']!=0 for i in (1,2,3))):
        raise swd.SessionError(f'Spin completion lacks stopped/enabled/{turns}-turn evidence')


def new_spin_running(sample):
    # The serial bridge may deliver the previous STOP's fresh DONE frame
    # immediately after publishing a new request. Require the new RUN baseline.
    if sample['phase'] != 2:
        return False
    if not -.02 <= sample['spin_yaw_rad'] <= .2:
        raise swd.SessionError('New spin did not start at its own YAW baseline')
    return True


def run(session, telemetry, turns=30):
    plan = make_plan(turns)
    session.snapshot_reader=lambda current:visual_snapshot(current,telemetry)
    state=session.validate_initial()
    if state['phase'] not in (0,4) or state['error']:
        raise swd.SessionError('Spin requires idle or a previous completed HOLD')
    for i in range(20):
        state=session.snapshot()
        if any(state['sample'][f'target_{wheel}']!=0 for wheel in (1,2,3)):
            raise swd.SessionError('Spin preflight requires zero targets')
        session.log.emit('spin_preflight',**state)
    initial=state['sample'];start_xy=[initial['odom_x_m'],initial['odom_y_m']]
    reference_tick=int(initial['tick_ms']);reference_host=time.monotonic()
    session.log.emit('spin_plan',**plan,start_xy=start_xy,initial_yaw_deg=initial['yaw_deg'])
    session.publish(9,[turns,*plan['stages_rad_s'],120000,0])
    telemetry.set_names(SPIN_NAMES)
    published=time.monotonic();deadline=published+123
    terminal=None;last_stage=None;running_seen=False
    while time.monotonic()<deadline:
        session.guard();session.beat(force=True)
        sample=telemetry.latest(timeout=.15)
        # Do not read SWD diagnostics in the moving loop. The previously
        # observed DAP read stalls must not block the heartbeat. UART is the
        # motion evidence; its source clock is compared with a preflight anchor.
        elapsed=((int(sample['tick_ms'])-reference_tick)&0xffffffff)*.001
        if abs(elapsed-(time.monotonic()-reference_host))>.20:
            raise swd.SessionError('Spin serial source is stale or clock changed')
        session.log.emit('spin_sample',sample=sample)
        if not running_seen:
            if sample['error'] or sample['phase']==5:
                raise swd.SessionError(f"Spin firmware fault {sample['error']}")
            running_seen=new_spin_running(sample)
            if not running_seen:
                if time.monotonic()-published>2:
                    raise swd.SessionError('New spin did not enter RUN')
                continue
        check_sample(sample,start_xy,turns)
        stage=int(sample['spin_stage'])
        if stage!=last_stage:
            print(json.dumps(dict(stage=stage+1,target_rad_s=plan['stages_rad_s'][stage],
                                  yaw_turns=sample['spin_yaw_rad']/math.tau,
                                  encoder_turns=sample['spin_encoder_rad']/math.tau),ensure_ascii=False),flush=True)
            last_stage=stage
        if sample['phase']==4:
            check_complete(sample,start_xy,turns)
            if terminal is not None and terminal!=sample['tick_ms']:break
            terminal=sample['tick_ms']
    else:raise swd.SessionError('Spin host deadline')
    # Read-only HOLD observation: no heartbeat or restart after successful stop.
    session.heartbeat=None
    last=sample
    for i in range(75):
        last=telemetry.latest(timeout=.15)
        check_complete(last,start_xy,turns)
        session.log.emit('spin_hold',sample=last)
    report=dict(turns=turns,counter=plan['counter'],rotation_radius_m=plan['rotation_radius_m'],
        encoder_turns=sample['spin_encoder_rad']/math.tau,
        yaw_turns=sample['spin_yaw_rad']/math.tau,gyro_turns=sample['spin_gyro_rad']/math.tau,
        yaw_minus_encoder_deg=math.degrees(sample['spin_yaw_rad']-sample['spin_encoder_rad']),
        yaw_minus_gyro_deg=math.degrees(sample['spin_yaw_rad']-sample['spin_gyro_rad']),
        final_wrapped_yaw_deg=sample['yaw_deg'],duration_s=sample['end_elapsed_ms']/1000.,
        drift_m=math.hypot(sample['odom_x_m']-start_xy[0],sample['odom_y_m']-start_xy[1]),
        hold_yaw_change_deg=math.degrees(last['spin_yaw_rad']-sample['spin_yaw_rad']),
        enabled=True,zero_targets=True,real_ground_angle_verified=False)
    session.log.emit('spin_complete',**report)
    return report


def stop(session,telemetry):
    session.stopping=True
    # If publication failed partway, the board may still be on the previous 56-field layout.
    telemetry.set_names(swd.SAMPLE_NAMES, SPIN_NAMES)
    if not session.client.usable:
        session.client.reconnect();session.verify_firmware(session.elf_path)
        session.snapshot_reader=lambda current:visual_snapshot(current,telemetry)
        session.validate_initial()
    sequence = session.publish(1,[0.]*8)
    end=time.monotonic()+2.5
    terminal_tick=None
    retried_fault=False
    while time.monotonic()<end:
        session.beat(force=True)
        sample=telemetry.latest(timeout=.15)
        if sample['phase']==5 and not retried_fault and session.read('ack_seq')==sequence:
            # A fault can finish braking while STOP is in progress. Request a fresh disable receipt.
            sequence=session.publish(1,[0.]*8)
            retried_fault=True
        if (sample['phase']==4 and sample['enabled']==0 and not sample['error'] and
            all(sample[f'state_{i}']==0 and sample[f'target_{i}']==0 for i in (1,2,3)) and
            session.read('ack_seq')==sequence):
            if terminal_tick is not None and terminal_tick!=sample['tick_ms']:
                session.log.emit('spin_stop_confirmed',sample=sample)
                return
            terminal_tick=sample['tick_ms']
    raise swd.SessionError('Spin STOP not confirmed')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--turns', type=int, choices=TURN_OPTIONS, default=30)
    p.add_argument('--execute',action='store_true');p.add_argument('--elf',type=Path)
    p.add_argument('--serial-port');p.add_argument('--log',type=Path);p.add_argument('--stop-file',type=Path)
    p.add_argument('--previous-spin',action='store_true',help='Current firmware serial layout is already 64 channels after command 9')
    args=p.parse_args()
    if not args.execute:print(json.dumps(dict(offline=True,**make_plan(args.turns))));return 0
    if not all((args.elf,args.serial_port,args.log,args.stop_file)):p.error('Exact ELF, serial port, new log and STOP path required')
    if args.stop_file.exists():p.error('STOP file exists')
    with swd.writer_lock('127.0.0.1',6666):
        client=log=session=telemetry=None;complete=False
        try:
            symbols = swd.load_symbols(args.elf, extra_sizes={"tuning_spin_sample": 32})
            log = swd.JsonLog(args.log)
            client=swd.TclClient('127.0.0.1',6666)
            if client.evaluate('stm32f4x.cpu curstate')!='running':raise swd.SessionError('MCU not running')
            session=ContinuousSession(client,symbols,log,stop_file=args.stop_file)
            session.verify_firmware(args.elf)
            telemetry=UartTelemetry(args.serial_port,args.log.with_suffix('.uart.bin'),log)
            if args.previous_spin:telemetry.set_names(SPIN_NAMES)
            result=run(session,telemetry,args.turns);complete=True
            print(json.dumps(result,indent=2,ensure_ascii=False),flush=True)
        finally:
            try:
                if not complete and session and telemetry and session._firmware_verified and session._initial_validated:
                    stop(session,telemetry)
            finally:
                if telemetry:telemetry.close()
                if client:client.close()
                if log:log.close()
    return 0

if __name__=='__main__':raise SystemExit(main())
