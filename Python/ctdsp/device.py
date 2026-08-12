"""Session-oriented interface to a CT-DSP board over the FT2232H channel-B UART.

Typical use::

    with CtdspDevice("COM8") as dev:
        print(dev.hello)              # protocol version, firmware id, clock rate
        dev.start()
        for event in dev.events(timeout=5.0):
            print(event.ticks, event.bits)

The reader runs on a background thread and pushes decoded events into a queue.
Framing recovery is handled entirely by `FrameSplitter`: a corrupt or partial
frame is dropped at the next 0x00 delimiter and the following frame decodes
cleanly, so there is no resynchronisation logic here and no need for the
timestamp-monotonicity and outlier heuristics the v1 reader relied on.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Iterator, List, Optional

import serial

from . import protocol as p


DEFAULT_BAUD = 3_000_000


class DeviceError(Exception):
    pass


class CommandError(DeviceError):
    """The device rejected a command."""


class TimeoutError_(DeviceError):
    """The device did not reply in time."""


@dataclass
class LinkStats:
    """Everything the host knows about how the link is behaving."""

    events: int = 0
    frames: int = 0
    frames_lost: int = 0     # inferred from gaps in the frame sequence number
    crc_errors: int = 0
    cobs_errors: int = 0
    queue_drops: int = 0     # events discarded because the consumer fell behind

    def __str__(self) -> str:
        return (
            f"events={self.events:,} frames={self.frames:,} "
            f"lost={self.frames_lost:,} crc_err={self.crc_errors:,} "
            f"cobs_err={self.cobs_errors:,} qdrop={self.queue_drops:,}"
        )


class CtdspDevice:
    def __init__(self, port: str, baudrate: int = DEFAULT_BAUD,
                 rtscts: bool = True, queue_size: int = 200_000,
                 auto_hello: bool = True) -> None:
        self.port = port
        self._serial = serial.Serial(
            port,
            baudrate=baudrate,
            timeout=0.05,
            rtscts=rtscts,
        )
        self._splitter = p.FrameSplitter()
        self._events: "queue.Queue[p.Event]" = queue.Queue(maxsize=queue_size)
        self._replies: "queue.Queue[object]" = queue.Queue()
        self._stats = LinkStats()
        self._last_seq: Optional[int] = None
        self._running = True
        self._lock = threading.Lock()

        self.hello: Optional[p.Hello] = None

        self._serial.reset_input_buffer()
        self._reader = threading.Thread(target=self._read_loop, daemon=True,
                                        name="ctdsp-reader")
        self._reader.start()

        if auto_hello:
            self.hello = self.ping()

    # -- context manager ---------------------------------------------------

    def __enter__(self) -> "CtdspDevice":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        if not self._running:
            return
        self._running = False
        try:
            self.stop()
        except DeviceError:
            pass  # closing anyway
        self._reader.join(timeout=1.0)
        self._serial.close()

    # -- reader thread -----------------------------------------------------

    def _read_loop(self) -> None:
        while self._running:
            try:
                pending = self._serial.in_waiting
                data = self._serial.read(pending if pending else 1)
            except (serial.SerialException, OSError):
                if self._running:
                    self._running = False
                return

            if not data:
                continue

            for payload in self._splitter.feed(data):
                try:
                    decoded = p.decode_payload(payload)
                except p.ProtocolError:
                    continue
                self._dispatch(decoded)

            with self._lock:
                self._stats.crc_errors = self._splitter.crc_errors
                self._stats.cobs_errors = self._splitter.cobs_errors

    def _dispatch(self, decoded) -> None:
        if isinstance(decoded, p.EventBatch):
            with self._lock:
                self._stats.frames += 1
                if self._last_seq is not None:
                    gap = (decoded.seq - self._last_seq - 1) & 0xFF
                    if gap:
                        # A sequence gap is unambiguous evidence of loss --
                        # far better than v1's guess-from-the-timestamp.
                        self._stats.frames_lost += gap
                self._last_seq = decoded.seq
                self._stats.events += len(decoded.events)

            for event in decoded.events:
                try:
                    self._events.put_nowait(event)
                except queue.Full:
                    with self._lock:
                        self._stats.queue_drops += 1
        else:
            self._replies.put(decoded)

    # -- commands ----------------------------------------------------------

    def _command(self, frame: bytes, expect, timeout: float = 1.0,
                 retries: int = 2):
        """Send a command and wait for its reply, retrying on timeout."""
        last_error: Optional[Exception] = None
        for _ in range(retries + 1):
            # Drop any stale reply so a late arrival is not mistaken for ours.
            while True:
                try:
                    self._replies.get_nowait()
                except queue.Empty:
                    break

            self._serial.write(frame)
            self._serial.flush()

            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    last_error = TimeoutError_(
                        f"no reply from {self.port} within {timeout}s")
                    break
                try:
                    reply = self._replies.get(timeout=remaining)
                except queue.Empty:
                    continue

                if isinstance(reply, p.Ack) and reply.status != p.AckStatus.OK:
                    raise CommandError(
                        f"device rejected command 0x{reply.command:02X}: "
                        f"{reply.status.name}")
                if isinstance(reply, expect):
                    return reply
                # Some other frame type; keep waiting within the deadline.

        raise last_error if last_error else TimeoutError_("no reply")

    def ping(self, timeout: float = 1.0) -> p.Hello:
        hello = self._command(p.cmd_ping(), p.Hello, timeout=timeout)
        self.hello = hello
        if hello.proto_ver != p.PROTO_VERSION:
            raise DeviceError(
                f"device speaks protocol v{hello.proto_ver}, host expects "
                f"v{p.PROTO_VERSION} -- the bitstream and this host are out of "
                f"step (v1 firmware will not respond to PING at all)")
        return hello

    def start(self) -> None:
        self._command(p.cmd_start(), p.Ack)

    def stop(self) -> None:
        self._command(p.cmd_stop(), p.Ack)

    def reset(self) -> None:
        self._command(p.cmd_reset(), p.Ack)
        with self._lock:
            self._last_seq = None
            self._stats = LinkStats()
        self.drain()

    def status(self) -> p.Status:
        return self._command(p.cmd_get_status(), p.Status)

    def set_settling_ns(self, nanoseconds: float) -> int:
        """Set the comparator settling time, given in nanoseconds.

        The wire field is in clock cycles; the conversion happens here because
        the host knows the device clock rate from HELLO and the fabric would
        need a divider to do it.
        """
        if self.hello is None:
            raise DeviceError("call ping() before set_settling_ns()")
        cycles = round(nanoseconds * self.hello.clk_freq_hz / 1e9)
        cycles = max(1, min(0xFFFF, cycles))
        self._command(p.cmd_set_settling(cycles), p.Ack)
        return cycles

    def set_batching(self, max_events: int, timeout_us: int) -> None:
        """Trade latency against per-event overhead.

        Larger batches amortise the frame header over more events; a shorter
        timeout bounds how long a partial batch waits before being flushed.
        """
        self._command(p.cmd_set_batch(max_events, timeout_us), p.Ack)

    # -- data --------------------------------------------------------------

    def events(self, timeout: Optional[float] = None) -> Iterator[p.Event]:
        """Yield events as they arrive.

        Stops when `timeout` seconds pass with no new event, or never if
        `timeout` is None.
        """
        while self._running:
            try:
                yield self._events.get(timeout=timeout if timeout else 0.5)
            except queue.Empty:
                if timeout is not None:
                    return

    def read_events(self, max_events: int = 4096) -> List[p.Event]:
        """Drain up to `max_events` without blocking."""
        out: List[p.Event] = []
        for _ in range(max_events):
            try:
                out.append(self._events.get_nowait())
            except queue.Empty:
                break
        return out

    def drain(self) -> None:
        while True:
            try:
                self._events.get_nowait()
            except queue.Empty:
                return

    @property
    def stats(self) -> LinkStats:
        with self._lock:
            return LinkStats(**vars(self._stats))

    def ticks_to_seconds(self, ticks: int) -> float:
        if self.hello is None:
            raise DeviceError("call ping() first so the clock rate is known")
        return self.hello.ticks_to_seconds(ticks)


def find_ports() -> List[str]:
    """List candidate serial ports, FTDI devices first."""
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    ports.sort(key=lambda info: (0 if (info.manufacturer or "").upper().startswith("FTDI")
                                 or "0403" in (info.hwid or "") else 1, info.device))
    return [info.device for info in ports]
