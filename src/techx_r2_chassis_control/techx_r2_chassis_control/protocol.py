"""Simplified R2 chassis serial protocol.

Frame layout, fixed 18 bytes:
  Byte0      0xAA
  Byte1      0x55
  Byte2      frame_type
  Byte3      seq
  Byte4      cmd_type
  Byte5-12   data[8]
  Byte13-15  reserved[3]
  Byte16-17  CRC16_MODBUS little endian

First-version command set:
  0x30 = CHASSIS_VEL_CMD
  0x31 = LIFT_CONTROL
  0x32 = STEP_COMMAND
  0x41 = LIFT_STATUS
  0x42 = STEP_STATUS
  0xE0 = ESTOP

LIFT_CONTROL v1, Host -> STM32:
  data0~1: target_h_mm, int16 little endian
  data2:   mask, bit0=lift1, bit1=lift2, bit2=lift3
  data3~7: reserved, zero

LIFT_STATUS v1, STM32 -> Host:
  data0~1: lift1_mm, int16 little endian
  data2~3: lift2_mm, int16 little endian
  data4~5: lift3_mm, int16 little endian
  data6:   state
  data7:   error_code

LIFT_STATUS seq rule:
  seq = original LIFT_CONTROL seq: reliable task feedback.
  seq = 0: STM32 periodic status report.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Tuple
import struct

FRAME_LEN = 18
HEADER = b"\xAA\x55"

FRAME_TYPE_HOST_TO_STM32 = 0x01
FRAME_TYPE_STM32_TO_HOST = 0x02

CMD_CHASSIS_VEL = 0x30
CMD_LIFT_CONTROL = 0x31
CMD_STEP_COMMAND = 0x32

CMD_LIFT_STATUS = 0x41
CMD_STEP_STATUS = 0x42

CMD_ESTOP = 0xE0

STEP_MOVE_FLAT = 0
STEP_CLIMB_200 = 1
STEP_CLIMB_400 = 2
STEP_DESCEND_200 = 3
STEP_DESCEND_400 = 4

STEP_CMD_NAME_TO_ID = {
    "MOVE_FLAT": STEP_MOVE_FLAT,
    "CLIMB_200": STEP_CLIMB_200,
    "CLIMB_400": STEP_CLIMB_400,
    "DESCEND_200": STEP_DESCEND_200,
    "DESCEND_400": STEP_DESCEND_400,
}

STEP_CMD_ID_TO_NAME = {value: key for key, value in STEP_CMD_NAME_TO_ID.items()}

STATE_ACK = 1
STATE_RUNNING = 2
STATE_DONE = 3
STATE_ERROR = 4

STATE_NAME = {
    STATE_ACK: "ACK",
    STATE_RUNNING: "RUNNING",
    STATE_DONE: "DONE",
    STATE_ERROR: "ERROR",
}

ERROR_OK = 0x00
ERROR_MOTOR_ERROR = 0x08


class ProtocolError(ValueError):
    """Raised when a serial frame is malformed."""


@dataclass(frozen=True)
class Frame:
    frame_type: int
    seq: int
    cmd_type: int
    data: bytes
    raw: bytes


@dataclass(frozen=True)
class StepStatus:
    seq: int
    step_cmd: int
    state: int
    error_code: int
    progress: int

    @property
    def state_name(self) -> str:
        return STATE_NAME.get(self.state, f"UNKNOWN({self.state})")

    @property
    def step_cmd_name(self) -> str:
        return STEP_CMD_ID_TO_NAME.get(self.step_cmd, f"UNKNOWN({self.step_cmd})")


@dataclass(frozen=True)
class LiftStatus:
    """Parsed LIFT_STATUS frame.

    seq == 0 means STM32 periodic status report.
    seq != 0 usually means feedback for a LIFT_CONTROL reliable task.
    """

    seq: int
    lift1_mm: int
    lift2_mm: int
    lift3_mm: int
    state: int
    error_code: int

    @property
    def state_name(self) -> str:
        return STATE_NAME.get(self.state, f"UNKNOWN({self.state})")

    @property
    def avg_height_mm(self) -> int:
        return int(round((self.lift1_mm + self.lift2_mm + self.lift3_mm) / 3.0))

    @property
    def max_diff_mm(self) -> int:
        values = [self.lift1_mm, self.lift2_mm, self.lift3_mm]
        return max(values) - min(values)


def crc16_modbus(data: bytes) -> int:
    """Compute CRC16-MODBUS, polynomial 0xA001, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def i16_to_le(value: int) -> bytes:
    if value < -32768 or value > 32767:
        raise ValueError(f"int16 out of range: {value}")
    return struct.pack("<h", int(value))


def u16_to_le(value: int) -> bytes:
    if value < 0 or value > 0xFFFF:
        raise ValueError(f"uint16 out of range: {value}")
    return struct.pack("<H", int(value))


def le_to_i16(data: bytes, offset: int = 0) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def le_to_u16(data: bytes, offset: int = 0) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def clamp_i16(value: float | int) -> int:
    value = int(round(value))
    return max(-32768, min(32767, value))


def pack_frame(frame_type: int, seq: int, cmd_type: int, data: bytes | bytearray | Iterable[int]) -> bytes:
    data = bytes(data)
    if len(data) != 8:
        raise ValueError(f"data must be exactly 8 bytes, got {len(data)}")
    if not (0 <= frame_type <= 0xFF and 0 <= seq <= 0xFF and 0 <= cmd_type <= 0xFF):
        raise ValueError("frame_type, seq and cmd_type must be uint8")

    frame_wo_crc = bytearray()
    frame_wo_crc.extend(HEADER)
    frame_wo_crc.append(frame_type)
    frame_wo_crc.append(seq & 0xFF)
    frame_wo_crc.append(cmd_type & 0xFF)
    frame_wo_crc.extend(data)
    frame_wo_crc.extend(b"\x00\x00\x00")

    crc = crc16_modbus(bytes(frame_wo_crc))
    frame_wo_crc.extend(struct.pack("<H", crc))
    return bytes(frame_wo_crc)


def parse_frame(raw: bytes | bytearray) -> Frame:
    raw = bytes(raw)
    if len(raw) != FRAME_LEN:
        raise ProtocolError(f"frame length must be {FRAME_LEN}, got {len(raw)}")
    if raw[0:2] != HEADER:
        raise ProtocolError("invalid frame header")
    expected_crc = le_to_u16(raw, 16)
    actual_crc = crc16_modbus(raw[:16])
    if expected_crc != actual_crc:
        raise ProtocolError(f"crc mismatch expected=0x{expected_crc:04X}, actual=0x{actual_crc:04X}")
    return Frame(frame_type=raw[2], seq=raw[3], cmd_type=raw[4], data=raw[5:13], raw=raw)


def extract_frames(buffer: bytearray) -> List[Frame]:
    """Extract all valid frames from a mutable byte buffer.

    Invalid bytes before the next AA 55 header are discarded. CRC-bad frames are
    discarded one byte at a time so the parser can resynchronize.
    """
    frames: List[Frame] = []
    while True:
        header_index = buffer.find(HEADER)
        if header_index < 0:
            buffer.clear()
            break
        if header_index > 0:
            del buffer[:header_index]
        if len(buffer) < FRAME_LEN:
            break
        candidate = bytes(buffer[:FRAME_LEN])
        try:
            frames.append(parse_frame(candidate))
            del buffer[:FRAME_LEN]
        except ProtocolError:
            del buffer[0]
    return frames


def build_chassis_vel_cmd(seq: int, vx_mm_s: int, vy_mm_s: int, omega_mrad_s: int, mode: int = 0) -> bytes:
    data = bytearray(8)
    data[0:2] = i16_to_le(clamp_i16(vx_mm_s))
    data[2:4] = i16_to_le(clamp_i16(vy_mm_s))
    data[4:6] = i16_to_le(clamp_i16(omega_mrad_s))
    data[6] = mode & 0xFF
    data[7] = 0
    return pack_frame(FRAME_TYPE_HOST_TO_STM32, seq, CMD_CHASSIS_VEL, data)


def decode_chassis_vel_data(data: bytes) -> Tuple[int, int, int, int]:
    if len(data) != 8:
        raise ValueError("velocity data must be 8 bytes")
    return le_to_i16(data, 0), le_to_i16(data, 2), le_to_i16(data, 4), data[6]


def build_lift_control(seq: int, target_h_mm: int, mask: int = 0x07) -> bytes:
    """Build first-version LIFT_CONTROL frame.

    This command means: move the lifts selected by mask to target_h_mm.

    Args:
        seq: reliable-task sequence id. Host should use non-zero seq.
        target_h_mm: target lift height in millimeters.
        mask: bit0=lift1, bit1=lift2, bit2=lift3.
              0x07 means all three lifts, i.e. whole chassis height.
    """
    data = bytearray(8)
    data[0:2] = i16_to_le(clamp_i16(target_h_mm))
    data[2] = mask & 0x07
    data[3:8] = b"\x00\x00\x00\x00\x00"
    return pack_frame(FRAME_TYPE_HOST_TO_STM32, seq, CMD_LIFT_CONTROL, data)


def decode_lift_control_data(data: bytes) -> Tuple[int, int]:
    """Decode first-version LIFT_CONTROL data area.

    Returns:
        (target_h_mm, mask)
    """
    if len(data) != 8:
        raise ValueError("lift control data must be 8 bytes")
    target_h_mm = le_to_i16(data, 0)
    mask = data[2]
    return target_h_mm, mask


def build_lift_status(
    seq: int,
    lift1_mm: int,
    lift2_mm: int,
    lift3_mm: int,
    state: int,
    error_code: int = ERROR_OK,
) -> bytes:
    """Build first-version LIFT_STATUS frame.

    Args:
        seq: original LIFT_CONTROL seq for task feedback; 0 for periodic report.
        lift1_mm: lift 1 measured height in millimeters.
        lift2_mm: lift 2 measured height in millimeters.
        lift3_mm: lift 3 measured height in millimeters.
        state: STATE_ACK / STATE_RUNNING / STATE_DONE / STATE_ERROR.
        error_code: protocol error code.
    """
    data = bytearray(8)
    data[0:2] = i16_to_le(clamp_i16(lift1_mm))
    data[2:4] = i16_to_le(clamp_i16(lift2_mm))
    data[4:6] = i16_to_le(clamp_i16(lift3_mm))
    data[6] = state & 0xFF
    data[7] = error_code & 0xFF
    return pack_frame(FRAME_TYPE_STM32_TO_HOST, seq, CMD_LIFT_STATUS, data)


def parse_lift_status(frame: Frame | bytes | bytearray) -> LiftStatus:
    if isinstance(frame, (bytes, bytearray)):
        frame = parse_frame(frame)
    if frame.frame_type != FRAME_TYPE_STM32_TO_HOST:
        raise ProtocolError("LIFT_STATUS must be STM32->Host frame")
    if frame.cmd_type != CMD_LIFT_STATUS:
        raise ProtocolError(f"not LIFT_STATUS cmd_type=0x{frame.cmd_type:02X}")

    data = frame.data
    return LiftStatus(
        seq=frame.seq,
        lift1_mm=le_to_i16(data, 0),
        lift2_mm=le_to_i16(data, 2),
        lift3_mm=le_to_i16(data, 4),
        state=data[6],
        error_code=data[7],
    )


def build_step_command(seq: int, step_cmd: int, delta_h_mm: int, edge_id: int = 0, flags: int = 0) -> bytes:
    data = bytearray(8)
    data[0] = step_cmd & 0xFF
    data[1:3] = i16_to_le(clamp_i16(delta_h_mm))
    data[3] = edge_id & 0xFF
    data[4] = flags & 0xFF
    data[5:8] = b"\x00\x00\x00"
    return pack_frame(FRAME_TYPE_HOST_TO_STM32, seq, CMD_STEP_COMMAND, data)


def decode_step_command_data(data: bytes) -> Tuple[int, int, int, int]:
    if len(data) != 8:
        raise ValueError("step command data must be 8 bytes")
    step_cmd = data[0]
    delta_h_mm = le_to_i16(data, 1)
    edge_id = data[3]
    flags = data[4]
    return step_cmd, delta_h_mm, edge_id, flags


def build_step_status(seq: int, step_cmd: int, state: int, error_code: int = ERROR_OK, progress: int = 0) -> bytes:
    data = bytearray(8)
    data[0] = step_cmd & 0xFF
    data[1] = state & 0xFF
    data[2] = error_code & 0xFF
    data[3] = max(0, min(100, int(progress))) & 0xFF
    data[4:8] = b"\x00\x00\x00\x00"
    return pack_frame(FRAME_TYPE_STM32_TO_HOST, seq, CMD_STEP_STATUS, data)


def parse_step_status(frame: Frame | bytes | bytearray) -> StepStatus:
    if isinstance(frame, (bytes, bytearray)):
        frame = parse_frame(frame)
    if frame.frame_type != FRAME_TYPE_STM32_TO_HOST:
        raise ProtocolError("STEP_STATUS must be STM32->Host frame")
    if frame.cmd_type != CMD_STEP_STATUS:
        raise ProtocolError(f"not STEP_STATUS cmd_type=0x{frame.cmd_type:02X}")
    data = frame.data
    return StepStatus(
        seq=frame.seq,
        step_cmd=data[0],
        state=data[1],
        error_code=data[2],
        progress=data[3],
    )


def build_estop(seq: int = 0) -> bytes:
    return pack_frame(FRAME_TYPE_HOST_TO_STM32, seq, CMD_ESTOP, b"\x00" * 8)


def step_cmd_from_string(cmd_type: str) -> int:
    key = cmd_type.strip().upper()
    if key not in STEP_CMD_NAME_TO_ID:
        valid = ", ".join(STEP_CMD_NAME_TO_ID.keys())
        raise ValueError(f"unknown cmd_type '{cmd_type}', valid: {valid}")
    return STEP_CMD_NAME_TO_ID[key]


def parse_edge_id(edge_id: str) -> int:
    """Convert behavior-tree edge_id string to uint8 for the first protocol version."""
    text = str(edge_id).strip()
    if not text:
        return 0
    try:
        return int(text, 0) & 0xFF
    except ValueError:
        # Deterministic compact id for symbolic names like A_B.
        return sum(text.encode("utf-8")) & 0xFF


def frame_to_hex(frame: bytes | bytearray) -> str:
    return " ".join(f"{b:02X}" for b in bytes(frame))