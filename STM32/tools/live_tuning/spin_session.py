"""Explicit 30-turn YAW-counted experiment; wheel/gyro remain diagnostics. Offline unless --execute."""
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import time
import swd_session as swd
from continuous_return_session import ContinuousSession
from uart_telemetry import UartTelemetry, visual_snapshot

SPIN_NAMES = swd.SAMPLE_NAMES + (
    'spin_encoder_rad','spin_yaw_rad','spin_gyro_rad','spin_requested_wz',
    'spin_stage','spin_target_rad','spin_yaw_minus_encoder_rad','spin_wheel_wz')
ARGS = [30.,2.,3.,4.,3.,2.,120000.,0.]
PLAN = dict(turns=30, direction='CCW', stages_rad_s=[2,3,4,3,2], turns_per_stage=6,
            ramp_rad_s2=1.6, counter='AHRS_YAW', rotation_radius_m=.0982,
            real_ground_angle_verified=False)


def load_symbols(elf):
    symbols=swd.load_symbols(elf)
    lines=subprocess.check_output([shutil.which('arm-none-eabi-nm'),'-S','--defined-only',str(elf)],text=True)
    for line in lines.splitlines():
        fields=line.split()
        if len(fields)==4 and fields[3]=='tuning_spin_sample':
            address,size=int(fields[0],16),int(fields[1],16)
            if size!=32 or address%4 or not 0x20000000<=address<address+size<=0x20020000:
                raise swd.SessionError('Spin ABI mismatch')
            symbols['tuning_spin_sample']=address
    if 'tuning_spin_sample' not in symbols:raise swd.SessionError('Missing spin test firmware')
    return symbols


def check_sample(sample, start_xy):
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
        not math.isclose(sample['spin_target_rad'],30*math.tau,abs_tol=2e-5) or
        not -.2<=sample['spin_yaw_rad']<=30*math.tau+.2):
        raise swd.SessionError('Spin progress invalid')


def check_complete(sample, start_xy):
    check_sample(sample,start_xy)
    if (sample['phase']!=4 or sample['enabled']!=1 or sample['end_stopped']!=1 or
        sample['end_valid']!=1 or abs(sample['spin_yaw_rad']-30*math.tau)>.2 or
        any(sample[f'target_{i}']!=0 for i in (1,2,3))):
        raise swd.SessionError('Spin completion lacks stopped/enabled/30-turn evidence')


def new_spin_running(sample):
    # The serial bridge may deliver the previous STOP's fresh DONE frame
    # immediately after publishing a new request. Require the new RUN baseline.
    if sample['phase'] != 2:
        return False
    if not -.02 <= sample['spin_yaw_rad'] <= .2:
        raise swd.SessionError('New spin did not start at its own YAW baseline')
    return True


def run(session, telemetry):
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
    session.log.emit('spin_plan',**PLAN,start_xy=start_xy,initial_yaw_deg=initial['yaw_deg'])
    session.publish(9,ARGS)
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
        check_sample(sample,start_xy)
        stage=int(sample['spin_stage'])
        if stage!=last_stage:
            print(json.dumps(dict(stage=stage+1,target_rad_s=PLAN['stages_rad_s'][stage],
                                  yaw_turns=sample['spin_yaw_rad']/math.tau,
                                  encoder_turns=sample['spin_encoder_rad']/math.tau),ensure_ascii=False),flush=True)
            last_stage=stage
        if sample['phase']==4:
            check_complete(sample,start_xy)
            if terminal is not None and terminal!=sample['tick_ms']:break
            terminal=sample['tick_ms']
    else:raise swd.SessionError('Spin host deadline')
    # Read-only HOLD observation: no heartbeat or restart after successful stop.
    session.heartbeat=None
    last=sample
    for i in range(75):
        last=telemetry.latest(timeout=.15)
        check_complete(last,start_xy)
        session.log.emit('spin_hold',sample=last)
    report=dict(counter=PLAN['counter'],rotation_radius_m=PLAN['rotation_radius_m'],
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
    if not session.client.usable:
        session.client.reconnect();session.verify_firmware(session.elf_path)
        session.snapshot_reader=lambda current:visual_snapshot(current,telemetry)
        session.validate_initial()
    session.publish(1,[0.]*8)
    end=time.monotonic()+2.5
    while time.monotonic()<end:
        session.beat(force=True)
        sample=telemetry.latest(timeout=.15)
        if sample['phase']==4 and sample['enabled']==0 and all(sample[f'state_{i}']==0 for i in (1,2,3)):
            session.log.emit('spin_stop_confirmed',sample=sample)
            return
    raise swd.SessionError('Spin STOP not confirmed')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true');p.add_argument('--elf',type=Path)
    p.add_argument('--serial-port');p.add_argument('--log',type=Path);p.add_argument('--stop-file',type=Path)
    p.add_argument('--previous-spin',action='store_true',help='Current firmware serial layout is already 64 channels after command 9')
    args=p.parse_args()
    if not args.execute:print(json.dumps(dict(offline=True,**PLAN)));return 0
    if not all((args.elf,args.serial_port,args.log,args.stop_file)):p.error('Exact ELF, serial port, new log and STOP path required')
    if args.stop_file.exists():p.error('STOP file exists')
    with swd.writer_lock('127.0.0.1',6666):
        client=log=session=telemetry=None;complete=False
        try:
            symbols=load_symbols(args.elf);log=swd.JsonLog(args.log)
            client=swd.TclClient('127.0.0.1',6666)
            if client.evaluate('stm32f4x.cpu curstate')!='running':raise swd.SessionError('MCU not running')
            session=ContinuousSession(client,symbols,log,stop_file=args.stop_file)
            session.verify_firmware(args.elf)
            telemetry=UartTelemetry(args.serial_port,args.log.with_suffix('.uart.bin'),log)
            if args.previous_spin:telemetry.set_names(SPIN_NAMES)
            result=run(session,telemetry);complete=True
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
