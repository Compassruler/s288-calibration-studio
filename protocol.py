"""独立 S288 协议：公开角度、速度和增益均使用输出端单位，角度为 rad。

只有 Bus 构造时才打开真实串口；导入模块和使用 SimBus 不会访问硬件。
"""

from collections import deque
import math
import struct
import time


RATIO = 70070.0 / 243.0
POSITION_LIMIT = 1428.018898
DEFAULT_BAUDRATE = 6_000_000
SUPPORTED_BAUDRATES = (6_000_000, 4_000_000)


class ProtocolError(RuntimeError):
    """A reply is missing, incomplete, corrupt, or from another motor."""


def validate_id(motor_id):
    if isinstance(motor_id, bool) or not isinstance(motor_id, int) or not 0 <= motor_id <= 14:
        raise ValueError("舵机 ID 必须是 0–14 的整数")
    return motor_id


def validate_baudrate(baudrate):
    """Only select the host serial rate; this does not reconfigure a motor."""
    if type(baudrate) is not int or baudrate not in SUPPORTED_BAUDRATES:
        raise ValueError("通信波特率必须是 4000000 或 6000000")
    return baudrate


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} 必须是有限数值")
    return float(value)


def crc32(data):
    """S288 CRC: little-endian words, most-significant byte processed first."""
    if len(data) % 4:
        raise ValueError("CRC 输入长度必须是 4 的倍数")
    value = 0xFFFFFFFF
    for offset in range(0, len(data), 4):
        for byte in reversed(data[offset:offset + 4]):
            value ^= byte << 24
            for _ in range(8):
                value = ((value << 1) ^ (0x04C11DB7 if value & 0x80000000 else 0)) & 0xFFFFFFFF
    return value


def build_packet(motor_id, raw_target=0.0, kp=0.0, kd=0.0, timeout_flag=1):
    """Build a mode-1 request with zero feedforward and zero target speed.

    Reject invalid inputs instead of silently turning them into an extreme
    command. Timeout protection may only be cleared with zero control gains.
    """
    validate_id(motor_id)
    raw_target = _finite(raw_target, "位置")
    kp, kd = _finite(kp, "Kp"), _finite(kd, "Kd")
    if abs(raw_target) > POSITION_LIMIT:
        raise ValueError("位置超出 S288 协议范围")
    if not 0 <= kp <= 2128.523254 or not 0 <= kd <= 21.285233:
        raise ValueError("Kp/Kd 超出 S288 协议范围")
    if timeout_flag not in (0, 1) or isinstance(timeout_flag, float):
        raise ValueError("超时保护标志必须是 0 或 1")
    if timeout_flag == 0 and (kp != 0 or kd != 0):
        raise ValueError("仅允许在零增益状态清除超时保护")
    # 协议使用转子端数据：位置乘减速比，输出端位置/速度增益除减速比平方。
    pos = round(raw_target * RATIO * 32768.0 / math.tau)
    p_gain = round(kp / RATIO**2 * 1280000.0)
    d_gain = round(kd / RATIO**2 * 128000000.0)
    if not -2147483648 <= pos <= 2147483647 or not 0 <= p_gain <= 32767 or not 0 <= d_gain <= 32767:
        raise ValueError("位置或增益无法编码到协议整数范围")
    header = b"\xfe\xee" + bytes([motor_id | 0x10 | (int(timeout_flag) << 7), 0])
    payload = header + struct.pack("<hhihh", 0, 0, pos, p_gain, d_gain)  # 前馈力矩、期望角速度固定为 0。
    return payload + struct.pack("<I", crc32(payload))


def parse_feedback(packet, expected_id=None):
    if len(packet) != 26 or packet[:2] != b"\xfc\xee":
        raise ProtocolError("S288 反馈帧头或长度错误")
    if struct.unpack("<I", packet[22:])[0] != crc32(packet[2:22]):
        raise ProtocolError("S288 反馈 CRC 错误")
    motor_id = packet[2] & 0x0F
    if expected_id is not None and motor_id != expected_id:
        raise ProtocolError(f"反馈 ID={motor_id}，期望 ID={expected_id}")
    temp, _sensor, volts, torque, speed, position, error, output_status = struct.unpack(
        "<bBBhhiIH2x", packet[3:22]
    )
    # raw_pos 来自转子累计位置换算，expos 是输出端单圈编码器；两者不自动取模或互换。
    return {
        "id": motor_id,
        "raw_pos": math.tau * position / 32768.0 / RATIO,
        "expos": math.tau * (output_status & 0x1FFF) / 8192.0,
        "speed": speed / 2.56 * math.tau / RATIO,
        "torque": torque / 256000.0 * RATIO,
        "voltage": volts / 2.0,
        "temp": temp,
        "mode": (packet[2] >> 4) & 0x07,
        "fault": error,
        "warning": (output_status >> 13) & 0x07,
        "timeout": (packet[2] >> 7) & 1,
        "timestamp": time.monotonic(),
    }


class Bus:
    """One synchronous serial connection owned by the controller thread."""

    def __init__(self, port, timeout=0.03, *, baudrate=DEFAULT_BAUDRATE):
        self.baudrate = validate_baudrate(baudrate)
        import serial

        if not isinstance(port, str) or not port.strip():
            raise ValueError("请选择串口")
        self.timeout = timeout
        self.serial = serial.Serial(port.strip(), self.baudrate, timeout=0.005, write_timeout=timeout)

    def exchange(self, motor_id, raw_target=0.0, kp=0.0, kd=0.0, timeout_flag=1):
        packet = build_packet(motor_id, raw_target, kp, kd, timeout_flag)
        self.serial.reset_input_buffer()
        if self.serial.write(packet) != len(packet):
            raise ProtocolError("S288 指令未完整发送")
        deadline = time.monotonic() + self.timeout
        data = bytearray()
        last_error = "反馈超时或收包不完整"
        while time.monotonic() < deadline:
            data.extend(self.serial.read(max(1, 26 - len(data))))
            while len(data) >= 2:
                start = data.find(b"\xfc\xee")
                if start < 0:
                    data[:] = data[-1:]
                    break
                del data[:start]
                if len(data) < 26:
                    break
                candidate = bytes(data[:26])
                try:
                    return parse_feedback(candidate, motor_id)
                except ProtocolError as exc:
                    last_error = str(exc)
                    del data[0]
        raise ProtocolError(f"ID {motor_id}：{last_error}")

    def close(self):
        self.serial.close()


class SimBus:
    """Simple virtual motors, deliberately not a physical S288/BAM model."""

    def __init__(self, port="SIM", timeout=0.03, *, baudrate=DEFAULT_BAUDRATE):
        self.baudrate = validate_baudrate(baudrate)
        self.closed = False
        self.positions = {motor_id: 0.3 + motor_id * 0.035 for motor_id in range(15)}
        self.last_update = {motor_id: time.monotonic() for motor_id in range(15)}
        self.fail_ids = set()
        self.history = deque(maxlen=10000)

    def exchange(self, motor_id, raw_target=0.0, kp=0.0, kd=0.0, timeout_flag=1):
        build_packet(motor_id, raw_target, kp, kd, timeout_flag)
        if self.closed:
            raise ProtocolError("模拟总线已关闭")
        if motor_id in self.fail_ids:
            raise ProtocolError(f"模拟 ID {motor_id} 失联")
        now = time.monotonic()
        dt = min(0.1, max(0.001, now - self.last_update[motor_id]))
        previous = self.positions[motor_id]
        delta = max(-0.8 * dt, min(0.8 * dt, (raw_target - previous) * min(1, dt / 0.12))) if kp else 0.0
        position = previous + delta
        self.positions[motor_id] = position
        self.last_update[motor_id] = now
        speed = delta / dt
        self.history.append({"id": motor_id, "raw_target": raw_target, "kp": kp, "kd": kd, "timeout_flag": timeout_flag})
        return {
            "id": motor_id, "raw_pos": position, "expos": position % math.tau,
            "speed": speed, "torque": kp * (raw_target - position) - kd * speed,
            "voltage": 12.0, "temp": 34 + motor_id % 3,
            "mode": 1, "fault": 0, "warning": 0, "timeout": 0, "timestamp": now,
        }

    def close(self):
        self.closed = True
