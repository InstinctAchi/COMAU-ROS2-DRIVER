"""
protocol.py - Binary telegram codec for the Comau PDL2 TCP/IP driver simulator.

Telegram frame layout (little-endian, C-packed):
  [STX 1B][VERSION 1B][MSG_TYPE 1B][SEQ_NUM 2B][PAYLOAD_LEN 2B][PAYLOAD NB][CHECKSUM 1B]

  STX      = 0x02
  VERSION  = 0x01
  CHECKSUM = XOR of all bytes from VERSION through end of PAYLOAD

All multi-byte integers are little-endian.
Floats are IEEE 754 single precision (4 bytes).
"""

import struct
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# MSG_TYPE constants
# ---------------------------------------------------------------------------

# Client → Simulator (requests)
MSG_GET_STATE    = 0x01
MSG_MOVE_JNT     = 0x02
MSG_MOVE_LIN     = 0x03
MSG_SET_SPEED    = 0x04
MSG_SET_IO       = 0x05
MSG_GET_IO       = 0x06
MSG_RESET_ALARM  = 0x07
MSG_SET_MODE     = 0x08
MSG_ESTOP        = 0x09
MSG_HEARTBEAT    = 0x0A
MSG_GRIPPER_CMD  = 0x0B
MSG_GET_ALARMS   = 0x0C
MSG_HOME         = 0x0D

# Simulator → Client (responses)
MSG_STATE_RESP    = 0x80
MSG_ACK           = 0x81
MSG_NACK          = 0x82
MSG_IO_RESP       = 0x83
MSG_ALARM_RESP    = 0x84
MSG_HEARTBEAT_ACK = 0x85
MSG_MOTION_DONE   = 0x86

# ---------------------------------------------------------------------------
# NACK error codes (Comau C5G style: XXXXX-LL)
# ---------------------------------------------------------------------------
ERR_JOINT_LIMIT      = (20012, 10)
ERR_ESTOP            = (20001, 12)
ERR_DRIVE_FAULT      = (20003, 10)
ERR_COMM_TIMEOUT     = (10005,  8)
ERR_PDL2_PROG        = (30001,  4)
ERR_SPEED_RANGE      = (40002,  2)
ERR_GRIPPER_MISMATCH = (50010,  0)
ERR_UNKNOWN_MSG      = (60001,  4)
ERR_INVALID_PAYLOAD  = (60002,  4)
ERR_BLOCKED          = (20005, 10)   # motion blocked (estop / alarm)

STX = 0x02
VERSION = 0x01
HEADER_FMT = "<BBBHH"   # STX, VERSION, MSG_TYPE, SEQ_NUM, PAYLOAD_LEN
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # = 7 bytes
MAX_PAYLOAD = 512


def _checksum(data: bytes) -> int:
    """XOR checksum over provided bytes."""
    result = 0
    for b in data:
        result ^= b
    return result


def encode(msg_type: int, seq_num: int, payload: bytes = b"") -> bytes:
    """
    Encode a telegram frame.

    Args:
        msg_type: one of MSG_* constants
        seq_num:  sequence number (0-65535)
        payload:  raw payload bytes

    Returns:
        Complete framed telegram ready to send.
    """
    header = struct.pack(
        HEADER_FMT,
        STX,
        VERSION,
        msg_type,
        seq_num & 0xFFFF,
        len(payload),
    )
    body = header[1:] + payload   # checksum covers VERSION through payload
    cs = _checksum(body)
    return header + payload + bytes([cs])


def decode_header(data: bytes) -> Optional[Tuple[int, int, int]]:
    """
    Parse the fixed header from a buffer.

    Returns:
        (msg_type, seq_num, payload_len) or None if too short / bad STX.
    """
    if len(data) < HEADER_SIZE:
        return None
    stx, ver, msg_type, seq_num, payload_len = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
    if stx != STX:
        return None
    return msg_type, seq_num, payload_len


def decode_frame(data: bytes) -> Optional[Tuple[int, int, bytes, bytes]]:
    """
    Parse one complete telegram from buffer.

    Returns:
        (msg_type, seq_num, payload, remaining_data) or None if incomplete/invalid.
    """
    if len(data) < HEADER_SIZE:
        return None

    header_parsed = decode_header(data)
    if header_parsed is None:
        # Try to resync: find next STX
        idx = data.find(STX, 1)
        if idx == -1:
            return None
        return decode_frame(data[idx:])

    msg_type, seq_num, payload_len = header_parsed
    total_len = HEADER_SIZE + payload_len + 1  # +1 for checksum
    if len(data) < total_len:
        return None

    payload = data[HEADER_SIZE: HEADER_SIZE + payload_len]
    cs_received = data[HEADER_SIZE + payload_len]
    body = data[1: HEADER_SIZE + payload_len]  # VERSION through payload
    cs_computed = _checksum(body)

    if cs_received != cs_computed:
        # Bad checksum: discard this frame, resync
        remaining = data[total_len:]
        return None

    remaining = data[total_len:]
    return msg_type, seq_num, payload, remaining


# ---------------------------------------------------------------------------
# Payload encoders (simulator → client)
# ---------------------------------------------------------------------------

def encode_ack(seq_num: int) -> bytes:
    return encode(MSG_ACK, seq_num, b"")


def encode_nack(seq_num: int, error_code: int, error_level: int, message: str = "") -> bytes:
    msg_bytes = message.encode("ascii")[:64]
    payload = struct.pack("<IH", error_code, error_level) + msg_bytes
    return encode(MSG_NACK, seq_num, payload)


def encode_heartbeat_ack(seq_num: int) -> bytes:
    return encode(MSG_HEARTBEAT_ACK, seq_num, b"")


def encode_motion_done(seq_num: int, joint_pos: List[float]) -> bytes:
    payload = struct.pack("<6f", *joint_pos)
    return encode(MSG_MOTION_DONE, seq_num, payload)


def encode_state(
    seq_num: int,
    joint_pos: List[float],
    joint_vel: List[float],
    cartesian: List[float],
    digital_out: List[bool],
    digital_in:  List[bool],
    analog_out:  List[float],
    analog_in:   List[float],
    status: int,
    mode: int,
    drives_on: bool,
    estop: bool,
    speed_override: int,
    alarm_count: int,
    controller_model: str,
    robot_model: str,
) -> bytes:
    """
    STATE_RESP payload layout:
      6×f  joint_pos   (24 bytes)
      6×f  joint_vel   (24 bytes)
      6×f  cartesian   (24 bytes)
      4×B  DO[0..31]   packed bitmask (4 bytes)
      4×B  DI[0..31]   packed bitmask (4 bytes)
      4×f  AO          (16 bytes)
      8×f  AI          (32 bytes)
      B    status       (1 byte)  0=IDLE,1=MOVING,2=HOLD,3=ERROR,4=ESTOP
      B    mode         (1 byte)  0=MAN,1=AUTO,2=PROG
      B    drives_on    (1 byte)
      B    estop        (1 byte)
      B    speed_ovr    (1 byte, 0-100)
      B    alarm_count  (1 byte)
      16s  controller   (16 bytes ASCII, null-padded)
      16s  robot_model  (16 bytes ASCII, null-padded)
    """

    def bitmask(flags: List[bool]) -> bytes:
        out = 0
        for i, f in enumerate(flags[:32]):
            if f:
                out |= (1 << i)
        return struct.pack("<I", out)

    payload = (
        struct.pack("<6f", *joint_pos[:6]) +
        struct.pack("<6f", *joint_vel[:6]) +
        struct.pack("<6f", *cartesian[:6]) +
        bitmask(digital_out) +
        bitmask(digital_in) +
        struct.pack("<4f", *analog_out[:4]) +
        struct.pack("<8f", *analog_in[:8]) +
        struct.pack("<BBBBB", status, mode, int(drives_on), int(estop), speed_override & 0xFF) +
        struct.pack("<B", alarm_count & 0xFF) +
        controller_model.encode("ascii").ljust(16, b"\x00")[:16] +
        robot_model.encode("ascii").ljust(16, b"\x00")[:16]
    )
    return encode(MSG_STATE_RESP, seq_num, payload)


def encode_io_resp(
    seq_num: int,
    digital_out: List[bool],
    digital_in:  List[bool],
    analog_out:  List[float],
    analog_in:   List[float],
) -> bytes:
    def bitmask(flags: List[bool]) -> bytes:
        v = 0
        for i, f in enumerate(flags[:32]):
            if f:
                v |= (1 << i)
        return struct.pack("<I", v)

    payload = (
        bitmask(digital_out) +
        bitmask(digital_in) +
        struct.pack("<4f", *analog_out[:4]) +
        struct.pack("<8f", *analog_in[:8])
    )
    return encode(MSG_IO_RESP, seq_num, payload)


def encode_alarm_resp(seq_num: int, alarms: list) -> bytes:
    """
    ALARM_RESP payload:
      B          count   (1 byte)
      count × (IHH32s)  per alarm: code(4), level(2), reserved(2), desc(32)
    """
    count = min(len(alarms), 16)
    payload = struct.pack("<B", count)
    for a in alarms[:count]:
        desc = a.description.encode("ascii").ljust(32, b"\x00")[:32]
        payload += struct.pack("<IHH", a.code, a.level, 0) + desc
    return encode(MSG_ALARM_RESP, seq_num, payload)


# ---------------------------------------------------------------------------
# Payload decoders (client → simulator)
# ---------------------------------------------------------------------------

def decode_move_jnt(payload: bytes) -> Optional[List[float]]:
    """Returns list of 6 target joint angles [degrees]."""
    if len(payload) < 24:
        return None
    return list(struct.unpack("<6f", payload[:24]))


def decode_move_lin(payload: bytes) -> Optional[List[float]]:
    """Returns [x, y, z, rx, ry, rz] target pose [mm, degrees]."""
    if len(payload) < 24:
        return None
    return list(struct.unpack("<6f", payload[:24]))


def decode_set_speed(payload: bytes) -> Optional[int]:
    """Returns speed override percentage (0-100)."""
    if len(payload) < 1:
        return None
    return payload[0]


def decode_set_io(payload: bytes) -> Optional[Tuple[int, int, float]]:
    """
    Returns (channel_index, io_type, value).
    io_type: 0=digital_out, 1=analog_out
    value:   0/1 for digital, float for analog
    """
    if len(payload) < 6:
        return None
    channel, io_type = struct.unpack("<BB", payload[:2])
    value = struct.unpack("<f", payload[2:6])[0]
    return channel, io_type, value


def decode_set_mode(payload: bytes) -> Optional[int]:
    """Returns mode byte: 0=MAN, 1=AUTO, 2=PROG."""
    if len(payload) < 1:
        return None
    return payload[0]


def decode_estop(payload: bytes) -> Optional[bool]:
    """Returns True = engage estop, False = release."""
    if len(payload) < 1:
        return None
    return bool(payload[0])


def decode_gripper_cmd(payload: bytes) -> Optional[int]:
    """Returns 0 = open, 1 = close."""
    if len(payload) < 1:
        return None
    return payload[0]
