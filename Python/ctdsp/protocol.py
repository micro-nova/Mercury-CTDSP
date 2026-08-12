"""CT-DSP link protocol v2 -- pure codec, no I/O.

Wire format
-----------
Every frame is COBS-encoded and terminated by a single 0x00 byte::

    <COBS(payload)> 0x00

COBS guarantees 0x00 never occurs inside an encoded frame, so the delimiter is
unambiguous: resynchronising is "scan to the next 0x00", and a payload byte can
never counterfeit a frame boundary. This replaces the v1 [0xF0][0x0F] sync
flags, which were ordinary data values and therefore forgeable.

Every payload is::

    [type:1] [body...] [crc16:2 LE]

where crc16 is CRC-16/CCITT-FALSE over [type..end of body].

Device -> host payloads
~~~~~~~~~~~~~~~~~~~~~~~
EVENT_BATCH (0x01)  [seq:1][abs_ts:6][n:1][ev_0..ev_(n-1)]
                    ev = [varint dt][bits:1]

    abs_ts is a 48-bit free-running tick count at the device's clock rate
    (reported by HELLO). Each event's absolute timestamp is the running sum of
    the deltas: ts_0 = abs_ts (dt_0 is always 0), ts_i = ts_(i-1) + dt_i.

    Every batch carries its own absolute anchor, so a lost frame costs exactly
    that frame -- the next one re-anchors on its own with no host intervention.
    `seq` increments per frame and wraps at 256; a gap in seq is the host's
    definitive loss detector.

STATUS (0x02)  [flags:1][fifo_level:2][overflow_events:4][frames_sent:4]
HELLO  (0x03)  [proto_ver:1][fw_id:4][clk_freq_hz:4][n_bits:1]
ACK    (0x04)  [echoed_cmd:1][status:1]

Host -> device payloads
~~~~~~~~~~~~~~~~~~~~~~~
PING (0x80), START (0x81), STOP (0x82), RESET (0x83), GET_STATUS (0x84)
    no body
SET_SETTLING (0x85)  [cycles:2]      settling time in device clock cycles
SET_BATCH    (0x86)  [max_events:1][timeout_us:2]

All multi-byte integers are little-endian.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Iterator, List, Sequence, Tuple

PROTO_VERSION = 2

#: Largest payload the device will ever emit, and the size of its staging
#: buffer. Mirrored by BUF_DEPTH in frame_tx.vhd.
MAX_PAYLOAD = 128

#: Largest number of events the device packs into one EVENT_BATCH. Mirrored by
#: the BATCH_MAX_EVENTS generic in frame_tx.vhd.
BATCH_MAX_EVENTS = 16


class FrameType(enum.IntEnum):
    EVENT_BATCH = 0x01
    STATUS = 0x02
    HELLO = 0x03
    ACK = 0x04


class Command(enum.IntEnum):
    PING = 0x80
    START = 0x81
    STOP = 0x82
    RESET = 0x83
    GET_STATUS = 0x84
    SET_SETTLING = 0x85
    SET_BATCH = 0x86


class AckStatus(enum.IntEnum):
    OK = 0
    BAD_CRC = 1
    UNKNOWN_CMD = 2
    BAD_ARGS = 3


class StatusFlag(enum.IntFlag):
    STREAMING = 0x01
    OVERFLOW_LATCHED = 0x02
    RX_FRAMING_ERROR = 0x04
    RX_CRC_ERROR = 0x08


class ProtocolError(Exception):
    """Raised when a frame cannot be decoded."""


class CobsError(ProtocolError):
    pass


class CrcError(ProtocolError):
    pass


# ---------------------------------------------------------------------------
# CRC-16/CCITT-FALSE  (poly 0x1021, init 0xFFFF, no reflection, no final xor)
# ---------------------------------------------------------------------------

def _build_crc_table() -> Tuple[int, ...]:
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return tuple(table)


_CRC_TABLE = _build_crc_table()


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


# ---------------------------------------------------------------------------
# LEB128 unsigned varint
# ---------------------------------------------------------------------------

def varint_encode(value: int) -> bytes:
    if value < 0:
        raise ValueError("varint values must be non-negative")
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            return bytes(out)


def varint_decode(buf: bytes, pos: int = 0) -> Tuple[int, int]:
    """Return (value, index just past the varint)."""
    value = 0
    shift = 0
    while True:
        if pos >= len(buf):
            raise ProtocolError("truncated varint")
        byte = buf[pos]
        pos += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, pos
        shift += 7
        if shift > 63:
            raise ProtocolError("varint too long")


# ---------------------------------------------------------------------------
# COBS
# ---------------------------------------------------------------------------

def cobs_encode(data: bytes) -> bytes:
    """Encode so the result contains no 0x00. Does not append the delimiter."""
    out = bytearray()
    code_index = 0
    out.append(0)  # placeholder for the first code byte
    code = 1
    for byte in data:
        if byte:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1
        else:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Decode a single COBS block. `data` must not include the delimiter."""
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise CobsError("zero code byte inside COBS block")
        i += 1
        end = i + code - 1
        if end > n:
            raise CobsError("COBS block truncated")
        out += data[i:end]
        i = end
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


# ---------------------------------------------------------------------------
# Payload framing
# ---------------------------------------------------------------------------

def build_payload(frame_type: int, body: bytes = b"") -> bytes:
    """Prepend the type byte and append the CRC."""
    payload = bytes([frame_type]) + body
    return payload + crc16(payload).to_bytes(2, "little")


def frame(payload: bytes) -> bytes:
    """COBS-encode a complete payload and append the 0x00 delimiter."""
    return cobs_encode(payload) + b"\x00"


def build_frame(frame_type: int, body: bytes = b"") -> bytes:
    return frame(build_payload(frame_type, body))


def check_payload(payload: bytes) -> Tuple[int, bytes]:
    """Validate the CRC and split a payload into (type, body)."""
    if len(payload) < 3:
        raise ProtocolError(f"payload too short ({len(payload)} bytes)")
    expected = int.from_bytes(payload[-2:], "little")
    actual = crc16(payload[:-2])
    if expected != actual:
        raise CrcError(f"CRC mismatch: got 0x{expected:04X}, computed 0x{actual:04X}")
    return payload[0], payload[1:-2]


# ---------------------------------------------------------------------------
# Decoded frame objects
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Event:
    #: Absolute timestamp in device clock ticks (48-bit).
    ticks: int
    #: Raw 8-bit comparator snapshot.
    bits: int


@dataclass(frozen=True)
class EventBatch:
    seq: int
    abs_ts: int
    events: List[Event] = field(default_factory=list)


@dataclass(frozen=True)
class Status:
    flags: StatusFlag
    fifo_level: int
    overflow_events: int
    frames_sent: int

    @property
    def streaming(self) -> bool:
        return bool(self.flags & StatusFlag.STREAMING)

    @property
    def overflowed(self) -> bool:
        return bool(self.flags & StatusFlag.OVERFLOW_LATCHED)


@dataclass(frozen=True)
class Hello:
    proto_ver: int
    fw_id: int
    clk_freq_hz: int
    n_bits: int

    def ticks_to_seconds(self, ticks: int) -> float:
        """Convert a device tick count to seconds.

        This is the single place the tick rate is applied. v1 hard-coded a
        divisor of 1e6 against a counter that actually ran at the system clock,
        which made every plotted time axis wrong by the clock-rate ratio.
        """
        return ticks / float(self.clk_freq_hz)


@dataclass(frozen=True)
class Ack:
    command: int
    status: AckStatus


def _decode_event_batch(body: bytes) -> EventBatch:
    if len(body) < 8:
        raise ProtocolError("EVENT_BATCH header truncated")
    seq = body[0]
    abs_ts = int.from_bytes(body[1:7], "little")
    count = body[7]
    pos = 8
    events: List[Event] = []
    ticks = abs_ts
    for _ in range(count):
        dt, pos = varint_decode(body, pos)
        if pos >= len(body):
            raise ProtocolError("EVENT_BATCH truncated: missing bits byte")
        ticks += dt  # dt_0 is always 0, so the first event sits exactly on abs_ts
        events.append(Event(ticks=ticks, bits=body[pos]))
        pos += 1
    if pos != len(body):
        raise ProtocolError(f"EVENT_BATCH has {len(body) - pos} trailing bytes")
    return EventBatch(seq=seq, abs_ts=abs_ts, events=events)


def _decode_status(body: bytes) -> Status:
    if len(body) != 11:
        raise ProtocolError(f"STATUS body must be 11 bytes, got {len(body)}")
    return Status(
        flags=StatusFlag(body[0]),
        fifo_level=int.from_bytes(body[1:3], "little"),
        overflow_events=int.from_bytes(body[3:7], "little"),
        frames_sent=int.from_bytes(body[7:11], "little"),
    )


def _decode_hello(body: bytes) -> Hello:
    if len(body) != 10:
        raise ProtocolError(f"HELLO body must be 10 bytes, got {len(body)}")
    return Hello(
        proto_ver=body[0],
        fw_id=int.from_bytes(body[1:5], "little"),
        clk_freq_hz=int.from_bytes(body[5:9], "little"),
        n_bits=body[9],
    )


def _decode_ack(body: bytes) -> Ack:
    if len(body) != 2:
        raise ProtocolError(f"ACK body must be 2 bytes, got {len(body)}")
    return Ack(command=body[0], status=AckStatus(body[1]))


_DECODERS = {
    FrameType.EVENT_BATCH: _decode_event_batch,
    FrameType.STATUS: _decode_status,
    FrameType.HELLO: _decode_hello,
    FrameType.ACK: _decode_ack,
}


def decode_payload(payload: bytes):
    """Validate and decode one complete payload into its dataclass."""
    frame_type, body = check_payload(payload)
    try:
        kind = FrameType(frame_type)
    except ValueError:
        raise ProtocolError(f"unknown frame type 0x{frame_type:02X}") from None
    return _DECODERS[kind](body)


# ---------------------------------------------------------------------------
# Encoders (device -> host; used by the selftest and by any simulator)
# ---------------------------------------------------------------------------

def encode_event_batch(seq: int, events: Sequence[Tuple[int, int]]) -> bytes:
    """Build an EVENT_BATCH frame from (absolute_ticks, bits) pairs."""
    if not events:
        raise ValueError("an EVENT_BATCH needs at least one event")
    if len(events) > 255:
        raise ValueError("at most 255 events per batch")
    abs_ts = events[0][0]
    body = bytearray()
    body.append(seq & 0xFF)
    body += abs_ts.to_bytes(6, "little")
    body.append(len(events))
    prev = abs_ts
    for i, (ticks, bits) in enumerate(events):
        dt = 0 if i == 0 else ticks - prev
        if dt < 0:
            raise ValueError("event timestamps must be non-decreasing")
        body += varint_encode(dt)
        body.append(bits & 0xFF)
        prev = ticks
    return build_frame(FrameType.EVENT_BATCH, bytes(body))


def encode_status(flags: int, fifo_level: int, overflow_events: int, frames_sent: int) -> bytes:
    body = (
        bytes([int(flags) & 0xFF])
        + int(fifo_level).to_bytes(2, "little")
        + int(overflow_events).to_bytes(4, "little")
        + int(frames_sent).to_bytes(4, "little")
    )
    return build_frame(FrameType.STATUS, body)


def encode_hello(fw_id: int, clk_freq_hz: int, n_bits: int = 8,
                 proto_ver: int = PROTO_VERSION) -> bytes:
    body = (
        bytes([proto_ver])
        + int(fw_id).to_bytes(4, "little")
        + int(clk_freq_hz).to_bytes(4, "little")
        + bytes([n_bits])
    )
    return build_frame(FrameType.HELLO, body)


def encode_ack(command: int, status: int = AckStatus.OK) -> bytes:
    return build_frame(FrameType.ACK, bytes([command & 0xFF, int(status) & 0xFF]))


# ---------------------------------------------------------------------------
# Encoders (host -> device)
# ---------------------------------------------------------------------------

def cmd_ping() -> bytes:
    return build_frame(Command.PING)


def cmd_start() -> bytes:
    return build_frame(Command.START)


def cmd_stop() -> bytes:
    return build_frame(Command.STOP)


def cmd_reset() -> bytes:
    return build_frame(Command.RESET)


def cmd_get_status() -> bytes:
    return build_frame(Command.GET_STATUS)


def cmd_set_settling(cycles: int) -> bytes:
    """Set the comparator settling time, in device clock cycles.

    Cycles rather than nanoseconds: converting ns to cycles needs a divide by
    1000, which is expensive in fabric and free here. The device reports its
    clock rate in HELLO, so `CtdspDevice.set_settling_ns` does the conversion.
    """
    if not 0 <= cycles <= 0xFFFF:
        raise ValueError("settling time must fit in 16 bits (cycles)")
    return build_frame(Command.SET_SETTLING, int(cycles).to_bytes(2, "little"))


def cmd_set_batch(max_events: int, timeout_us: int) -> bytes:
    if not 1 <= max_events <= 255:
        raise ValueError("max_events must be 1..255")
    if not 0 <= timeout_us <= 0xFFFF:
        raise ValueError("timeout_us must fit in 16 bits")
    return build_frame(
        Command.SET_BATCH,
        bytes([max_events]) + int(timeout_us).to_bytes(2, "little"),
    )


# ---------------------------------------------------------------------------
# Incremental byte-stream splitter
# ---------------------------------------------------------------------------

class FrameSplitter:
    """Feed raw serial bytes in, get validated payloads out.

    Recovery is trivial by construction: 0x00 delimits frames and cannot occur
    inside one, so a partial or corrupt frame is discarded at the next
    delimiter and the following frame decodes cleanly. No heuristics needed.
    """

    def __init__(self, max_payload: int = MAX_PAYLOAD) -> None:
        self._buf = bytearray()
        self._max_encoded = max_payload + max_payload // 254 + 2
        self.cobs_errors = 0
        self.crc_errors = 0
        self.oversize_drops = 0

    def feed(self, data: bytes) -> Iterator[bytes]:
        """Yield each complete, CRC-valid payload found in the stream."""
        self._buf += data
        while True:
            idx = self._buf.find(0)
            if idx < 0:
                # No complete frame yet. Guard against a stuck stream with no
                # delimiter ever arriving (e.g. line noise at the wrong baud).
                if len(self._buf) > self._max_encoded:
                    del self._buf[:-self._max_encoded]
                    self.oversize_drops += 1
                return
            block = bytes(self._buf[:idx])
            del self._buf[: idx + 1]
            if not block:
                continue  # back-to-back delimiters, or leading garbage
            try:
                payload = cobs_decode(block)
            except CobsError:
                self.cobs_errors += 1
                continue
            try:
                check_payload(payload)
            except ProtocolError:
                self.crc_errors += 1
                continue
            yield payload

    def reset(self) -> None:
        self._buf.clear()
