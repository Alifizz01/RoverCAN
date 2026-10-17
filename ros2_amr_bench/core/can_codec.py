"""CAN frame layout for the (simulated) wheel motor controller.

0x101 WHEEL_CMD (bridge -> motor ECU), 8 bytes little endian:
    int16 left mrad/s, int16 right mrad/s, uint8 counter, uint8 0, uint8 0, uint8 xor checksum
0x181 ENCODER (motor ECU -> bridge), 8 bytes little endian:
    int16 left mrad/s, int16 right mrad/s, uint8 counter, uint8 status, uint8 0, uint8 xor checksum
status bits: 0x01 left stalled, 0x02 right stalled (as reported by the ECU)
"""
import struct

WHEEL_CMD_ID = 0x101
ENCODER_ID = 0x181
_FMT = "<hhBBB"  # 7 bytes, checksum appended


class FrameError(ValueError):
    pass


def _xor(data):
    c = 0
    for b in data:
        c ^= b
    return c


def _to_mrad(x):
    return max(-32767, min(32767, round(x * 1000)))


def _pack(left, right, counter, status):
    body = struct.pack(_FMT, _to_mrad(left), _to_mrad(right), counter & 0xFF, status & 0xFF, 0)
    return body + bytes([_xor(body)])


def _unpack(data):
    data = bytes(data)
    if len(data) != 8:
        raise FrameError(f"expected 8 bytes, got {len(data)}")
    if _xor(data[:7]) != data[7]:
        raise FrameError("checksum mismatch")
    left, right, cnt, status, _ = struct.unpack(_FMT, data[:7])
    return left / 1000, right / 1000, cnt, status


def encode_cmd(left, right, counter):
    return _pack(left, right, counter, 0)


def decode_cmd(data):
    left, right, cnt, _ = _unpack(data)
    return left, right, cnt


def encode_encoder(left, right, counter, status=0):
    return _pack(left, right, counter, status)


def decode_encoder(data):
    """Returns (left, right, counter, status)."""
    return _unpack(data)
