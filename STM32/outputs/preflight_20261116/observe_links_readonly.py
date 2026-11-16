"""Read-only RAM diagnostics for the manual firmware; never sends a robot command.

--describe verifies ELF symbol sizes and DWARF layouts offline, without TCP.
Sampling verifies every Flash PT_LOAD byte first. Only read_memory is admitted;
no halt/reset, peripheral-register access, RAM writes, or final stop command.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'STM32/tools/live_tuning'))
from swd_session import TclClient, verify_flash

TOOL_BIN = Path('C:/Program Files/Arm/GNU Toolchain mingw-w64-x86_64-arm-none-eabi/bin')
ELF = ROOT / 'STM32/project/build/Debug/project.elf'
# Size validation catches stale assumptions; DWARF checks below validate offsets.
SIZES = {
    'xTickCount': 4, 'manual_velocity': 24, 'manual_control_state': 1,
    'blocked_sequence': 4, 'state_started_ms': 4,
    'chassis_motor_target': 12, 'chassis_motor_ff': 12, 'chassis_speed': 12,
    'enabled': 1, 'pending_disable': 1, 'pending_enable': 1, 'mode': 1,
    'monitoring_stop': 1, 'stop_groups': 1, 'disable_after_stop': 1,
    'dm_motor': 84, 'INS_control_data': 12, 'INS_control_ready': 1,
    'BMI088_error': 1, 'battery_voltage': 4, 'hcan1': 40,
    'configured': 1, 'tx_busy': 1, 'link_state': 8, 'rx_buffer': 1036,
    'parser': 15, 'usb_dbg_reset': 4, 'usb_dbg_setup': 4,
}
LAYOUTS = {
    '(unsigned)&((CAN_HandleTypeDef*)0)->State': 32,
    '(unsigned)&((CAN_HandleTypeDef*)0)->ErrorCode': 36,
    '(unsigned)&((UsbRxBuffer*)0)->head': 1024,
    '(unsigned)&((UsbRxBuffer*)0)->tail': 1026,
    '(unsigned)&((UsbRxBuffer*)0)->generation': 1028,
    '(unsigned)&((UsbRxBuffer*)0)->overflow_count': 1032,
    '(unsigned)&((LinkParser*)0)->count': 14,
    '(unsigned)&((Manual_CAN_Velocity_t*)0)->received_ms': 8,
    '(unsigned)&((Manual_CAN_Velocity_t*)0)->sequence': 12,
    '(unsigned)&((Manual_CAN_Velocity_t*)0)->invalid_sequence': 16,
    '(unsigned)&((Manual_CAN_Velocity_t*)0)->zero_sequence': 20,
    '(unsigned)&((INS_control_data_t*)0)->sample_time_ms': 8,
}


class ReadOnlyClient(TclClient):
    def evaluate(self, command):
        read = r'read_memory (?:0x[0-9a-fA-F]+|[0-9]+) 32 [0-9]+'
        if not (re.fullmatch(read, command) or
                re.fullmatch(r'concat (?:\[' + read + r'\] ?)+', command)):
            raise RuntimeError('Read-only client rejected command')
        for address, count in re.findall(r'read_memory (\S+) 32 (\d+)', command):
            start, length = int(address, 0), int(count) * 4
            in_flash = 0x08000000 <= start and start + length <= 0x08100000
            in_ram = 0x20000000 <= start and start + length <= 0x20020000
            if not 1 <= int(count) <= 256 or not (in_flash or in_ram):
                raise RuntimeError('Read-only client rejected range')
        return super().evaluate(command)

    def write_words(self, *args, **kwargs):
        raise RuntimeError('RAM writes are forbidden')


def describe(elf, nm, gdb):
    output = subprocess.check_output([str(nm), '-S', '--defined-only', str(elf)], text=True)
    found = {}
    for row in output.splitlines():
        parts = row.split()
        if len(parts) == 4 and parts[3] in SIZES:
            found.setdefault(parts[3], []).append((int(parts[0], 16), int(parts[1], 16)))
    symbols = {}
    for name, expected in SIZES.items():
        entries = found.get(name, [])
        if len(entries) != 1 or entries[0][1] != expected:
            raise RuntimeError(f'ELF symbol size/uniqueness mismatch: {name}: {entries}')
        address, size = entries[0]
        if not 0x20000000 <= address < address + size <= 0x20020000:
            raise RuntimeError(f'ELF symbol outside SRAM: {name}')
        symbols[name] = {'address': address, 'size': size}
    command = [str(gdb), '-nx', '-batch', str(elf)]
    for expression in LAYOUTS:
        command += ['-ex', 'p ' + expression]
    process = subprocess.run(command, capture_output=True, text=True, timeout=30)
    actual = [int(v) for v in re.findall(r'^\$\d+ = (\d+)\s*$', process.stdout, re.M)]
    if process.returncode or actual != list(LAYOUTS.values()):
        raise RuntimeError(f'DWARF layout mismatch: {process.stdout} {process.stderr}')
    return {'elf': str(elf), 'elf_sha256': hashlib.sha256(elf.read_bytes()).hexdigest(),
            'symbols': symbols, 'dwarf_offsets': LAYOUTS}


def blocks_for(symbols):
    spans = []
    for name, item in symbols.items():
        start, size = item['address'], item['size']
        if name == 'rx_buffer':  # Skip the actual USB byte buffer.
            start += 1024
            size = 12
        if name == 'hcan1':  # Only HAL state/error; no hardware registers.
            start += 32
            size = 8
        if name == 'xTickCount':
            continue
        spans.append((start & ~3, (start + size + 3) & ~3))
    blocks = []
    for start, end in sorted(spans):
        if blocks and start - blocks[-1][1] <= 16 and end - blocks[-1][0] <= 1024:
            blocks[-1][1] = max(end, blocks[-1][1])
        else:
            blocks.append([start, end])
    return blocks


def snapshot(client, symbols):
    blocks = blocks_for(symbols)
    tick = symbols['xTickCount']['address']
    reads = [[tick, tick+4]] + blocks + [[tick, tick+4]]
    command = 'concat ' + ' '.join(f'[read_memory {a:#x} 32 {(b-a)//4}]' for a, b in reads)
    start = time.monotonic()
    words = client.parse_words(client.evaluate(command), sum((b-a)//4 for a, b in reads))
    tick0, tick1 = words[0], words[-1]
    chunks, cursor = [], 1
    for a, b in blocks:
        count = (b-a)//4
        chunks.append((a, struct.pack('<'+'I'*count, *words[cursor:cursor+count])))
        cursor += count

    def get(name, fmt, offset=0):
        address = symbols[name]['address'] + offset
        for base, data in chunks:
            if base <= address and address+struct.calcsize(fmt) <= base+len(data):
                values = struct.unpack_from(fmt, data, address-base)
                return values[0] if len(values) == 1 else list(values)
        raise RuntimeError('Missing sampled bytes: ' + name)

    valid, vx, vy, wz, rx_ms, seq, invalid_seq, zero_seq = get('manual_velocity', '<BxhhhIIII')
    motors = []
    for i in range(3):
        mid, state, pos, vel, torque, mos, rotor, ms, mseq = get('dm_motor', '<BB2xfffBB2xII', i*28)
        motors.append(dict(id=mid, state=state, position=pos, velocity=vel, torque=torque,
                           mos=mos, rotor=rotor, feedback_ms=ms, sequence=mseq,
                           age_at_sample_end_ms=(tick1-ms)&0xffffffff))
    yaw, wz_imu, imu_ms = get('INS_control_data', '<ffI')
    head, tail, generation, overflow = get('rx_buffer', '<HHII', 1024)
    return {
        'host_monotonic': start, 'read_duration_ms': (time.monotonic()-start)*1000,
        'tick_before': tick0, 'tick_after': tick1, 'snapshot_span_ms': (tick1-tick0)&0xffffffff,
        'command': dict(valid=valid, vx=vx, vy=vy, wz=wz, received_ms=rx_ms, sequence=seq,
                        invalid_sequence=invalid_seq, zero_sequence=zero_seq,
                        age_at_sample_end_ms=(tick1-rx_ms)&0xffffffff),
        'manual_state': get('manual_control_state', '<B'),
        'blocked_sequence': get('blocked_sequence', '<I'),
        'state_started_ms': get('state_started_ms', '<I'),
        'chassis': {name: get(name, '<B') for name in
                    ['enabled', 'pending_enable', 'pending_disable', 'mode',
                     'monitoring_stop', 'stop_groups', 'disable_after_stop']},
        'targets': get('chassis_motor_target', '<3f'),
        'feedforward': get('chassis_motor_ff', '<3f'),
        'body_velocity': get('chassis_speed', '<3f'),
        'motors': motors,
        'imu': dict(ready=get('INS_control_ready', '<B'), yaw_deg=yaw, world_z_rad_s=wz_imu,
                    sample_ms=imu_ms, age_at_sample_end_ms=(tick1-imu_ms)&0xffffffff,
                    bmi088_init_error=get('BMI088_error', '<B')),
        'battery_voltage': get('battery_voltage', '<f'),
        'can_hal': dict(state=get('hcan1', '<I', 32), error_code=get('hcan1', '<I', 36)),
        'usb': dict(configured=get('configured', '<B'), connected=get('link_state', '<B'),
                    generation=get('link_state', '<I', 4), tx_busy=get('tx_busy', '<B'),
                    reset_count=get('usb_dbg_reset', '<I'), setup_count=get('usb_dbg_setup', '<I'),
                    rx_head=head, rx_tail=tail, rx_pending_bytes=(head-tail)%1024,
                    rx_generation=generation, overflow_count=overflow,
                    parser_partial_bytes=get('parser', '<B', 14)),
    }


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, list):
        return [finite_json(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', type=Path, default=ELF)
    parser.add_argument('--nm', type=Path, default=TOOL_BIN/'arm-none-eabi-nm.exe')
    parser.add_argument('--gdb', type=Path, default=TOOL_BIN/'arm-none-eabi-gdb.exe')
    parser.add_argument('--describe', action='store_true')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=6666)
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--period', type=float, default=0.1)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 30:
        parser.error('duration must be in (0,30]')
    if not math.isfinite(args.period) or not 0.05 <= args.period <= 2:
        parser.error('period must be in [0.05,2]')
    metadata = describe(args.elf, args.nm, args.gdb)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.describe:
        with args.output.open('x', encoding='utf-8') as output:
            json.dump(metadata, output, indent=2)
        print('Offline ELF description saved: ' + str(args.output))
        return
    # Do not construct a TCP client until offline validation has succeeded.
    client = None
    with args.output.open('x', encoding='utf-8') as output:
        def emit(event, **data):
            output.write(json.dumps(finite_json(dict(event=event, **data)), allow_nan=False)+'\n')
            output.flush()
        try:
            client = ReadOnlyClient(args.host, args.port, timeout=3)
            segments = verify_flash(client, args.elf.read_bytes())
            emit('firmware_verified', segments=segments, **metadata)
            emit('limitations', notes=[
                'RAM samples are not atomic; ages use end tick and include read-window skew.',
                'Disarmed motors may stop reporting; no motion or enable request is sent.',
                'command.sequence counts CRC-passed type2 frames, including valid0 revocations.',
                'No CRC failure, USB byte total, CAN TX completion or sensor-read failure counters exist here.',
                'HAL CAN error_code is not a complete live hardware bus-error monitor.',
                'No USB upstream proof: manual mode sends no protocol acknowledgement.',
                'Nonfinite floating values are logged as null, never silently treated as zero.',
            ])
            deadline = time.monotonic()+args.duration
            while time.monotonic() < deadline:
                started = time.monotonic()
                emit('sample', **snapshot(client, metadata['symbols']))
                time.sleep(max(0, min(args.period-(time.monotonic()-started), deadline-time.monotonic())))
            emit('complete', ok=True)
        except Exception as error:
            emit('error', error=str(error))
            raise
        finally:
            if client is not None:
                client.close()


if __name__ == '__main__':
    main()
