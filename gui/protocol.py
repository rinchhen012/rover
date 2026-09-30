"""Python mirror of rover/shared/protocol.h.

Keep this file in sync with the C header. Same framing, same CRC8,
same payload layouts (big-endian).

Frame layout:
  [0] SYNC 0xAA
  [1] TYPE
  [2] LEN  (payload length, max 16)
  [3..3+LEN-1] PAYLOAD
  [3+LEN] CRC8 over bytes [0 .. 3+LEN-1]  (poly 0x07, init 0x00)
"""

SYNC = 0xAA
MAX_PAYLOAD = 16

CMD_DRIVE = 0x01
CMD_GIMBAL = 0x02
TELEM = 0x03
PING = 0x04
ACK = 0x05

TYPE_NAMES = {
    CMD_DRIVE: "CMD_DRIVE",
    CMD_GIMBAL: "CMD_GIMBAL",
    TELEM: "TELEM",
    PING: "PING",
    ACK: "ACK",
}

MAX_THROTTLE = 255
MAX_STEER = 255


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def encode(frame_type: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    body = bytes([SYNC, frame_type, len(payload)]) + payload
    return body + bytes([crc8(body)])


# ---- frame builders -------------------------------------------------------
def make_drive(throttle: int, steer: int) -> bytes:
    """throttle/steer in -255..255."""
    throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, int(throttle)))
    steer = max(-MAX_STEER, min(MAX_STEER, int(steer)))
    return encode(CMD_DRIVE, int(throttle).to_bytes(2, "big", signed=True)
                             + int(steer).to_bytes(2, "big", signed=True))


def make_gimbal(pan: int, tilt: int) -> bytes:
    return encode(CMD_GIMBAL, bytes([int(pan) & 0xFF, int(tilt) & 0xFF]))


def make_ping() -> bytes:
    return encode(PING, b"")


def make_ack() -> bytes:
    return encode(ACK, b"")


def parse_drive(payload: bytes) -> tuple:
    return int.from_bytes(payload[0:2], "big", signed=True), int.from_bytes(payload[2:4], "big", signed=True)


def parse_gimbal(payload: bytes) -> tuple:
    return payload[0], payload[1]


def parse_telem(payload: bytes) -> dict:
    return {
        "dist_cm": int.from_bytes(payload[0:2], "big"),
        "batt_mv": int.from_bytes(payload[2:4], "big"),
        "loop_ms": payload[4],
        "radio_ok": payload[5],  # 1 = carrier detected (RPD), 0 = none
    }


class Parser:
    """Streaming parser. Feed bytes, get complete frames back."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._state = 0  # 0=sync, 1=type, 2=len, 3=payload, 4=crc
        self._frame = bytearray()
        self.frames_ok = 0
        self.frames_bad = 0

    def feed(self, data) -> list:
        frames = []
        for b in data if isinstance(data, bytes) else bytes([data]):
            if self._state == 0:
                if b == SYNC:
                    self._frame = bytearray([b])
                    self._state = 1
            elif self._state == 1:
                self._frame.append(b)
                self._state = 2
            elif self._state == 2:
                self._frame.append(b)
                if b > MAX_PAYLOAD:
                    self.frames_bad += 1
                    self._state = 0
                else:
                    self._state = 3
            elif self._state == 3:
                self._frame.append(b)
                if len(self._frame) == 3 + self._frame[2]:
                    self._state = 4
            elif self._state == 4:
                if crc8(self._frame) == b:
                    self.frames_ok += 1
                    self._state = 0
                    frames.append((self._frame[1], bytes(self._frame[3:])))
                else:
                    self.frames_bad += 1
                    self._state = 0
                    if b == SYNC:
                        self._frame = bytearray([b])
                        self._state = 1
        return frames
