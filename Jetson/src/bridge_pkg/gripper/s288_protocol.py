"""宇树 J288/S288 单总线协议；角度、速度及力矩均为输出端单位。"""

import math
import struct

RATIO = 70070.0 / 243.0


def crc32(data):
    """每四字节按小端字序计算手册中的 CRC32。"""
    if len(data) % 4:
        raise ValueError('CRC 数据长度必须是 4 的倍数')
    crc = 0xFFFFFFFF
    for offset in range(0, len(data), 4):
        word = int.from_bytes(data[offset:offset + 4], 'little')
        crc ^= word
        for _ in range(32):
            crc = ((crc << 1) ^ (0x04C11DB7 if crc & 0x80000000 else 0)) & 0xFFFFFFFF
    return crc


def control_packet(motor_id, mode, timeout, torque, speed, position, kp, kd):
    """生成 20 字节控制包；拒绝无法编码的数值。"""
    if not 0 <= motor_id <= 14 or mode not in (0, 1) or timeout not in (0, 1):
        raise ValueError('舵机 ID、模式或超时位不合法')
    if not all(math.isfinite(v) for v in (torque, speed, position, kp, kd)) or kp < 0 or kd < 0:
        raise ValueError('控制量必须有限，增益不能为负')
    raw = (
        round(torque / RATIO * 256000),
        round(speed * RATIO * 2.56 / (2 * math.pi)),
        round(position * RATIO * 32768 / (2 * math.pi)),
        round(kp / (RATIO * RATIO) * 1280000),
        round(kd / (RATIO * RATIO) * 128000000),
    )
    try:
        payload = struct.pack('<2sBBhhihh', b'\xfe\xee', motor_id | mode << 4 | timeout << 7, 0, *raw)
    except struct.error as error:
        raise ValueError('控制量超出 S288 协议范围') from error
    return payload + struct.pack('<I', crc32(payload))


def parse_feedback(packet, expected_id):
    """校验并解析 26 字节反馈，返回输出端物理量。"""
    if len(packet) != 26 or packet[:2] != b'\xfc\xee':
        raise ValueError('反馈长度或帧头不正确')
    if crc32(packet[2:22]) != struct.unpack_from('<I', packet, 22)[0]:
        raise ValueError('反馈 CRC 不正确')
    mode, temperature, winding, voltage, torque, speed, position, error, output = struct.unpack_from(
        '<BbBBhhiIH', packet, 2)
    if mode & 15 != expected_id:
        raise ValueError('反馈舵机 ID 不匹配')
    return {
        'id': mode & 15, 'mode': (mode >> 4) & 7, 'timeout': bool(mode & 128),
        'temperature': temperature, 'winding_temperature': winding, 'voltage': voltage / 2.0,
        'torque': torque * RATIO / 256000, 'speed': speed * (2 * math.pi) / (2.56 * RATIO),
        'position': position * (2 * math.pi) / (32768 * RATIO),
        'output_encoder': output & 0x1FFF, 'warning': output >> 13, 'error': error,
    }
