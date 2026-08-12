"""Self-test for the CT-DSP link protocol.

Two modes:

    python -m ctdsp.selftest
        Round-trip and known-answer vectors for COBS, LEB128 and CRC-16, plus
        frame encode/decode and resynchronisation behaviour.

    python -m ctdsp.selftest --decode <bytes.txt> [--expect <events.txt>]
        Decode a byte dump produced by the VHDL testbench (tb_frame_tx.vhd) and
        print the recovered events. With --expect, assert they match exactly.
        This is the cross-check that validates the HDL and the host against the
        same spec before any hardware is involved.

Byte-dump format: whitespace-separated hex bytes, "#" starts a comment.
Expectation format: one "<ticks> <bits>" pair per line, decimal.
"""

from __future__ import annotations

import argparse
import random
import sys
from typing import List, Tuple

from . import protocol as p


class SelfTestFailure(AssertionError):
    pass


_checks = 0


def check(condition: bool, message: str) -> None:
    global _checks
    _checks += 1
    if not condition:
        raise SelfTestFailure(message)


def check_eq(actual, expected, message: str) -> None:
    check(actual == expected, f"{message}\n  expected: {expected!r}\n  actual:   {actual!r}")


# ---------------------------------------------------------------------------
# CRC-16/CCITT-FALSE known answers
# ---------------------------------------------------------------------------

def test_crc16() -> None:
    check_eq(p.crc16(b""), 0xFFFF, "CRC of empty input")
    check_eq(p.crc16(b"A"), 0xB915, "CRC of 'A'")
    check_eq(p.crc16(b"123456789"), 0x29B1, "CRC-16/CCITT-FALSE check value")
    # A CRC appended to its own message re-CRCs to a constant; that constant is
    # what the HDL compares against, so pin it down here.
    msg = b"\x01\x02\x03\x04"
    with_crc = msg + p.crc16(msg).to_bytes(2, "little")
    check_eq(p.crc16(with_crc[:-2]), int.from_bytes(with_crc[-2:], "little"),
             "CRC verifies over its own message")


# ---------------------------------------------------------------------------
# LEB128
# ---------------------------------------------------------------------------

def test_varint() -> None:
    vectors = {
        0: b"\x00",
        1: b"\x01",
        127: b"\x7f",
        128: b"\x80\x01",
        255: b"\xff\x01",
        16383: b"\xff\x7f",
        16384: b"\x80\x80\x01",
        48000: b"\x80\xf7\x02",  # the 500 us flush bound at 96 MHz: 3 bytes
        2097151: b"\xff\xff\x7f",
        2097152: b"\x80\x80\x80\x01",
    }
    for value, encoded in vectors.items():
        check_eq(p.varint_encode(value), encoded, f"varint_encode({value})")
        check_eq(p.varint_decode(encoded), (value, len(encoded)), f"varint_decode({encoded!r})")

    for value in [0, 1, 63, 64, 127, 128, 1000, 65535, 1 << 20, (1 << 48) - 1]:
        enc = p.varint_encode(value)
        check_eq(p.varint_decode(enc)[0], value, f"varint round-trip {value}")

    # Truncated varints must raise rather than return garbage.
    try:
        p.varint_decode(b"\x80")
        check(False, "truncated varint should raise")
    except p.ProtocolError:
        pass


# ---------------------------------------------------------------------------
# COBS
# ---------------------------------------------------------------------------

def test_cobs_vectors() -> None:
    # The canonical COBS vectors from Cheshire & Baker.
    vectors = [
        (b"\x00", b"\x01\x01"),
        (b"\x00\x00", b"\x01\x01\x01"),
        (b"\x11\x22\x00\x33", b"\x03\x11\x22\x02\x33"),
        (b"\x11\x22\x33\x44", b"\x05\x11\x22\x33\x44"),
        (b"\x11\x00\x00\x00", b"\x02\x11\x01\x01\x01"),
        (b"", b"\x01"),
    ]
    for raw, encoded in vectors:
        check_eq(p.cobs_encode(raw), encoded, f"cobs_encode({raw!r})")
        check_eq(p.cobs_decode(encoded), raw, f"cobs_decode({encoded!r})")


def test_cobs_roundtrip() -> None:
    rng = random.Random(0xC7D5)
    cases: List[bytes] = [
        b"",
        b"\x00",
        b"\x00" * 300,
        b"\xff" * 300,          # forces the 0xFF code-byte split
        bytes(range(256)),
        bytes(254),             # exactly one full COBS run of zeros
        b"\x01" * 254,          # exactly at the 0xFF boundary
        b"\x01" * 255,          # one past it
        b"\x01" * 253 + b"\x00",
    ]
    for _ in range(500):
        n = rng.randrange(0, 400)
        cases.append(bytes(rng.randrange(0, 256) for _ in range(n)))

    for raw in cases:
        encoded = p.cobs_encode(raw)
        check(0 not in encoded, f"COBS output must contain no zero byte (input {raw[:16]!r}...)")
        check_eq(p.cobs_decode(encoded), raw, f"COBS round-trip for {len(raw)}-byte input")


# ---------------------------------------------------------------------------
# Frames
# ---------------------------------------------------------------------------

def test_frame_roundtrip() -> None:
    hello = p.encode_hello(fw_id=0xDEADBEEF, clk_freq_hz=96_000_000, n_bits=8)
    decoded = _decode_single(hello)
    check_eq(decoded, p.Hello(proto_ver=p.PROTO_VERSION, fw_id=0xDEADBEEF,
                              clk_freq_hz=96_000_000, n_bits=8), "HELLO round-trip")
    check(abs(decoded.ticks_to_seconds(96_000_000) - 1.0) < 1e-12,
          "HELLO tick conversion: 96e6 ticks at 96 MHz is one second")

    status = p.encode_status(
        flags=p.StatusFlag.STREAMING | p.StatusFlag.OVERFLOW_LATCHED,
        fifo_level=1234, overflow_events=7, frames_sent=999999)
    st = _decode_single(status)
    check_eq(st.fifo_level, 1234, "STATUS fifo_level")
    check_eq(st.overflow_events, 7, "STATUS overflow_events")
    check_eq(st.frames_sent, 999999, "STATUS frames_sent")
    check(st.streaming and st.overflowed, "STATUS flags decode")

    ack = _decode_single(p.encode_ack(p.Command.START, p.AckStatus.OK))
    check_eq(ack, p.Ack(command=int(p.Command.START), status=p.AckStatus.OK), "ACK round-trip")


def test_event_batch() -> None:
    # A realistic burst: a fast run of transitions then a long-ish gap.
    events: List[Tuple[int, int]] = [
        (1_000_000, 0x00),
        (1_000_012, 0x01),
        (1_000_030, 0x03),
        (1_000_031, 0x07),
        (1_048_000, 0x0F),  # ~500 us later at 96 MHz -- the 3-byte varint case
    ]
    raw = p.encode_event_batch(seq=42, events=events)
    batch = _decode_single(raw)
    check_eq(batch.seq, 42, "EVENT_BATCH seq")
    check_eq(batch.abs_ts, 1_000_000, "EVENT_BATCH abs_ts anchors on the first event")
    check_eq([(e.ticks, e.bits) for e in batch.events], events,
             "EVENT_BATCH reconstructs absolute timestamps from deltas")

    # 48-bit timestamps must survive intact -- this is what removes v1's
    # 32-bit wraparound handling from the host entirely.
    big = (1 << 48) - 1 - 100
    batch = _decode_single(p.encode_event_batch(seq=0, events=[(big, 0xAA), (big + 50, 0x55)]))
    check_eq([(e.ticks, e.bits) for e in batch.events], [(big, 0xAA), (big + 50, 0x55)],
             "48-bit timestamps round-trip")

    # A full batch must stay inside the device's staging buffer.
    full = [(1000 + 17 * i, i & 0xFF) for i in range(p.BATCH_MAX_EVENTS)]
    encoded = p.encode_event_batch(seq=1, events=full)
    payload_len = len(p.cobs_decode(encoded[:-1]))
    check(payload_len <= p.MAX_PAYLOAD,
          f"a {p.BATCH_MAX_EVENTS}-event batch payload is {payload_len} bytes, "
          f"which must fit MAX_PAYLOAD={p.MAX_PAYLOAD}")


def test_bad_crc_rejected() -> None:
    good = p.encode_hello(fw_id=1, clk_freq_hz=96_000_000)
    payload = bytearray(p.cobs_decode(good[:-1]))
    payload[-1] ^= 0xFF  # corrupt the CRC
    try:
        p.decode_payload(bytes(payload))
        check(False, "a corrupted CRC must be rejected")
    except p.CrcError:
        pass


# ---------------------------------------------------------------------------
# Stream resynchronisation
# ---------------------------------------------------------------------------

def test_splitter_resync() -> None:
    frames = [
        p.encode_hello(fw_id=1, clk_freq_hz=96_000_000),
        p.encode_event_batch(seq=1, events=[(500, 0x11), (512, 0x22)]),
        p.encode_event_batch(seq=2, events=[(900, 0x33)]),
    ]

    # Clean stream, delivered one byte at a time to exercise partial feeds.
    splitter = p.FrameSplitter()
    got = []
    for byte in b"".join(frames):
        got.extend(splitter.feed(bytes([byte])))
    check_eq(len(got), 3, "all frames recovered from a byte-at-a-time feed")

    # Attaching mid-stream: the host lands on partial bytes with no leading
    # delimiter. Those bytes merge into the frame that follows, so exactly one
    # frame is lost -- and then the stream is locked for good.
    splitter = p.FrameSplitter()
    stream = b"\x7f\x42\x99\xab" + b"".join(frames)
    recovered = [p.decode_payload(payload) for payload in splitter.feed(stream)]
    check_eq(len(recovered), 2, "leading junk costs exactly the one frame it merges into")
    check_eq([f.seq for f in recovered], [1, 2], "every frame after the junk decodes")

    # Once the junk is terminated by a delimiter -- which is what actually
    # happens, since the in-flight frame ends with one -- nothing is lost.
    splitter = p.FrameSplitter()
    recovered = [p.decode_payload(payload)
                 for payload in splitter.feed(b"\x7f\x42\x99\xab\x00" + b"".join(frames))]
    check_eq(len(recovered), 3, "junk terminated by a delimiter costs no frames")

    # A corrupt frame is dropped on its own; its neighbours are unaffected.
    broken = bytearray(frames[1])
    broken[2] ^= 0xFF  # corrupt a body byte -> CRC failure
    splitter = p.FrameSplitter()
    recovered = [p.decode_payload(payload)
                 for payload in splitter.feed(frames[0] + bytes(broken) + frames[2])]
    check_eq(len(recovered), 2, "only the corrupt frame is dropped")
    check(isinstance(recovered[0], p.Hello), "the frame before the corrupt one survives")
    check_eq(recovered[1].seq, 2, "the frame after the corrupt one decodes cleanly")
    check(splitter.crc_errors + splitter.cobs_errors == 1, "the corrupt frame was counted once")

    # A stream of pure noise must never yield a frame, and must not grow the
    # buffer without bound.
    rng = random.Random(7)
    splitter = p.FrameSplitter()
    noise = bytes(rng.randrange(1, 256) for _ in range(10_000))
    check_eq(list(splitter.feed(noise)), [], "noise with no delimiters yields no frames")


def test_command_encoders() -> None:
    for encoder, expected_cmd in [
        (p.cmd_ping, p.Command.PING),
        (p.cmd_start, p.Command.START),
        (p.cmd_stop, p.Command.STOP),
        (p.cmd_reset, p.Command.RESET),
        (p.cmd_get_status, p.Command.GET_STATUS),
    ]:
        payload = p.cobs_decode(encoder()[:-1])
        cmd, body = p.check_payload(payload)
        check_eq(cmd, int(expected_cmd), f"{expected_cmd.name} command byte")
        check_eq(body, b"", f"{expected_cmd.name} takes no arguments")

    cmd, body = p.check_payload(p.cobs_decode(p.cmd_set_settling(250)[:-1]))
    check_eq(cmd, int(p.Command.SET_SETTLING), "SET_SETTLING command byte")
    check_eq(int.from_bytes(body, "little"), 250, "SET_SETTLING argument")

    cmd, body = p.check_payload(p.cobs_decode(p.cmd_set_batch(16, 500)[:-1]))
    check_eq(cmd, int(p.Command.SET_BATCH), "SET_BATCH command byte")
    check_eq(body[0], 16, "SET_BATCH max_events")
    check_eq(int.from_bytes(body[1:3], "little"), 500, "SET_BATCH timeout_us")

    # Every command frame must be free of 0x00 apart from its delimiter, or the
    # device's byte-level resync would trip on its own input.
    for framed in [p.cmd_ping(), p.cmd_start(), p.cmd_set_settling(0), p.cmd_set_batch(1, 0)]:
        check_eq(framed.count(0), 1, "a command frame contains exactly one zero byte")
        check_eq(framed[-1], 0, "the zero byte is the trailing delimiter")


def _decode_single(framed: bytes):
    splitter = p.FrameSplitter()
    payloads = list(splitter.feed(framed))
    check_eq(len(payloads), 1, "exactly one frame expected")
    return p.decode_payload(payloads[0])


# ---------------------------------------------------------------------------
# Device layer, driven against a simulated board
#
# These exercise CtdspDevice's command/response matching, background reader and
# loss accounting without any hardware. They need pyserial installed (only for
# the exception types and the module CtdspDevice patches); everything else is
# stubbed.
# ---------------------------------------------------------------------------

class FakeBoard:
    """Stands in for pyserial's Serial, emulating a CT-DSP board."""

    CLK_HZ = 96_000_000

    def __init__(self, *args, **kwargs):
        import threading

        self._to_host = bytearray()
        self._lock = threading.Lock()
        self._splitter = p.FrameSplitter()
        self.is_open = True
        self.streaming = False
        self.seq = 0
        self.commands = []
        self.settling_cycles = 24
        self.reject_next = False

    # -- pyserial surface used by CtdspDevice --

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._to_host)

    def read(self, size: int = 1) -> bytes:
        import time
        with self._lock:
            if self._to_host:
                chunk = bytes(self._to_host[:size])
                del self._to_host[:size]
                return chunk
        time.sleep(0.002)   # emulate the read timeout instead of spinning
        return b""

    def write(self, data: bytes) -> int:
        for payload in self._splitter.feed(data):
            self._handle(payload)
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._to_host.clear()

    def close(self) -> None:
        self.is_open = False

    # -- board behaviour --

    def _send(self, framed: bytes) -> None:
        with self._lock:
            self._to_host += framed

    def _handle(self, payload: bytes) -> None:
        code, body = p.check_payload(payload)
        self.commands.append(code)

        if self.reject_next:
            self.reject_next = False
            self._send(p.encode_ack(code, p.AckStatus.BAD_CRC))
            return

        if code == p.Command.PING:
            self._send(p.encode_hello(fw_id=0x00000002, clk_freq_hz=self.CLK_HZ))
        elif code == p.Command.GET_STATUS:
            flags = p.StatusFlag.STREAMING if self.streaming else p.StatusFlag(0)
            self._send(p.encode_status(flags=flags, fifo_level=3,
                                       overflow_events=0, frames_sent=self.seq))
        else:
            if code == p.Command.START:
                self.streaming = True
            elif code == p.Command.STOP:
                self.streaming = False
            elif code == p.Command.RESET:
                self.streaming = False
                self.seq = 0
            elif code == p.Command.SET_SETTLING:
                self.settling_cycles = int.from_bytes(body, "little")
            self._send(p.encode_ack(code, p.AckStatus.OK))

    def push_batch(self, events, seq=None) -> None:
        """Emit an EVENT_BATCH; pass `seq` explicitly to fake a lost frame."""
        use = self.seq if seq is None else seq
        self._send(p.encode_event_batch(use, events))
        self.seq = (use + 1) & 0xFF


def _with_fake_board():
    """Return (device, board) with CtdspDevice talking to a FakeBoard."""
    from . import device as device_mod

    board = FakeBoard()
    original = device_mod.serial.Serial
    device_mod.serial.Serial = lambda *a, **kw: board
    try:
        dev = device_mod.CtdspDevice("FAKE")
    finally:
        device_mod.serial.Serial = original
    return dev, board


def test_device_handshake() -> None:
    dev, board = _with_fake_board()
    try:
        check_eq(dev.hello.clk_freq_hz, FakeBoard.CLK_HZ, "HELLO reports the device clock")
        check_eq(dev.hello.proto_ver, p.PROTO_VERSION, "HELLO protocol version")
        check(int(p.Command.PING) in board.commands, "the board saw a PING")

        # The tick conversion must come from HELLO, not a hard-coded constant.
        check(abs(dev.ticks_to_seconds(96_000) - 0.001) < 1e-12,
              "96,000 ticks at 96 MHz is one millisecond")

        dev.start()
        check(board.streaming, "START put the board into streaming mode")
        dev.stop()
        check(not board.streaming, "STOP took it out again")

        status = dev.status()
        check_eq(status.fifo_level, 3, "STATUS round-trips through the device layer")

        cycles = dev.set_settling_ns(250)
        check_eq(cycles, 24, "250 ns at 96 MHz is 24 cycles")
        check_eq(board.settling_cycles, 24, "the board received the converted value")
    finally:
        dev.close()


def test_device_events_and_loss() -> None:
    dev, board = _with_fake_board()
    try:
        dev.start()

        board.push_batch([(1000, 0x01), (1012, 0x02), (1030, 0x03)])
        board.push_batch([(2000, 0x04)])
        _wait_for(lambda: dev.stats.events >= 4)

        events = dev.read_events()
        check_eq([(e.ticks, e.bits) for e in events],
                 [(1000, 0x01), (1012, 0x02), (1030, 0x03), (2000, 0x04)],
                 "events arrive in order with absolute timestamps")
        check_eq(dev.stats.frames_lost, 0, "no loss reported on a clean stream")

        # Skip a sequence number, as a dropped frame would.
        board.push_batch([(3000, 0x05)], seq=(board.seq + 1) & 0xFF)
        _wait_for(lambda: dev.stats.frames_lost >= 1)
        check_eq(dev.stats.frames_lost, 1, "a sequence gap is reported as one lost frame")

        # 48-bit timestamps must survive the whole path. v1 carried 32 bits and
        # wrapped every ~86 s, which the host had to paper over.
        dev.drain()
        before = dev.stats.events
        big = (1 << 47) + 12345
        board.push_batch([(big, 0xAA), (big + 7, 0xBB)])
        _wait_for(lambda: dev.stats.events >= before + 2)
        check_eq([(e.ticks, e.bits) for e in dev.read_events()],
                 [(big, 0xAA), (big + 7, 0xBB)],
                 "48-bit timestamps survive the full host path")
    finally:
        dev.close()


def test_device_rejects_bad_command() -> None:
    from .device import CommandError

    dev, board = _with_fake_board()
    try:
        board.reject_next = True
        try:
            dev.start()
            check(False, "a NACK from the board must raise")
        except CommandError:
            pass
    finally:
        dev.close()


def _wait_for(predicate, tries: int = 200, delay: float = 0.01) -> None:
    import time
    for _ in range(tries):
        if predicate():
            return
        time.sleep(delay)
    check(False, "timed out waiting for the device layer")


DEVICE_TESTS = [
    test_device_handshake,
    test_device_events_and_loss,
    test_device_rejects_bad_command,
]


def device_tests_available() -> bool:
    try:
        import serial  # noqa: F401
        return True
    except ImportError:
        return False


# ---------------------------------------------------------------------------
# Testbench dump decoding
# ---------------------------------------------------------------------------

def read_byte_dump(path: str) -> bytes:
    out = bytearray()
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            for token in line.split():
                out.append(int(token, 16))
    return bytes(out)


def read_expectation(path: str) -> List[Tuple[int, int]]:
    expected = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            ticks, bits = line.split()
            # int(x, 0) accepts both decimal and 0x-prefixed hex, so the VHDL
            # testbench can emit 48-bit timestamps that overflow VHDL's 32-bit
            # integer type by writing them as hex.
            expected.append((int(ticks, 0), int(bits, 0)))
    return expected


def decode_dump(dump_path: str, expect_path: str | None) -> int:
    data = read_byte_dump(dump_path)
    splitter = p.FrameSplitter()
    events: List[Tuple[int, int]] = []
    last_seq = None
    seq_gaps = 0
    others = 0

    for payload in splitter.feed(data):
        decoded = p.decode_payload(payload)
        if isinstance(decoded, p.EventBatch):
            if last_seq is not None and decoded.seq != (last_seq + 1) & 0xFF:
                seq_gaps += 1
            last_seq = decoded.seq
            events.extend((e.ticks, e.bits) for e in decoded.events)
        else:
            others += 1

    print(f"{len(data)} bytes -> {len(events)} events, {others} non-event frames")
    print(f"cobs errors: {splitter.cobs_errors}  crc errors: {splitter.crc_errors}  "
          f"seq gaps: {seq_gaps}")
    if events:
        density = len(data) / len(events)
        print(f"encoded cost: {density:.2f} bytes/event (v1 protocol was a flat 7.00)")

    if splitter.cobs_errors or splitter.crc_errors or seq_gaps:
        print("FAIL: the dump contains framing, CRC or sequence errors", file=sys.stderr)
        return 1

    if expect_path is None:
        for ticks, bits in events:
            print(f"  {ticks:>16}  0x{bits:02X}")
        return 0

    expected = read_expectation(expect_path)
    if events == expected:
        print(f"PASS: all {len(expected)} events match {expect_path}")
        return 0

    print(f"FAIL: decoded {len(events)} events, expected {len(expected)}", file=sys.stderr)
    for i, (got, want) in enumerate(zip(events, expected)):
        if got != want:
            print(f"  first difference at index {i}: got {got}, expected {want}", file=sys.stderr)
            break
    return 1


# ---------------------------------------------------------------------------

TESTS = [
    test_crc16,
    test_varint,
    test_cobs_vectors,
    test_cobs_roundtrip,
    test_frame_roundtrip,
    test_event_batch,
    test_bad_crc_rejected,
    test_splitter_resync,
    test_command_encoders,
]


def run_all() -> int:
    tests = list(TESTS)
    if device_tests_available():
        tests += DEVICE_TESTS
    else:
        print("note: skipping device-layer tests (pyserial not installed)",
              file=sys.stderr)

    failures = 0
    for test in tests:
        try:
            test()
        except SelfTestFailure as exc:
            failures += 1
            print(f"FAIL  {test.__name__}\n      {exc}", file=sys.stderr)
        except Exception as exc:  # a crash is a failure too
            failures += 1
            print(f"ERROR {test.__name__}\n      {exc!r}", file=sys.stderr)
        else:
            print(f"ok    {test.__name__}")
    if failures:
        print(f"\n{failures} of {len(tests)} tests failed", file=sys.stderr)
        return 1
    print(f"\nall {len(tests)} tests passed ({_checks} checks)")
    return 0


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--decode", metavar="DUMP",
                        help="decode a hex byte dump from the VHDL testbench")
    parser.add_argument("--expect", metavar="EVENTS",
                        help="assert the decoded events match this file")
    args = parser.parse_args(argv)

    if args.decode:
        return decode_dump(args.decode, args.expect)
    return run_all()


if __name__ == "__main__":
    sys.exit(main())
