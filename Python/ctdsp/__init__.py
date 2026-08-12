"""CT-DSP host-side support package.

Three lines is the whole program::

    import ctdsp
    dev = ctdsp.receive("COM11")
    ctdsp.plot(dev, window=5, ticks=True, time_span=2)

`protocol` holds the pure wire-format codec (no I/O); `device` wraps a serial
port in a session-oriented API; `view` holds the filters and the live plot. The
wire format is documented in protocol.py and is mirrored bit-for-bit by the VHDL
in HDL/.../frame_tx.vhd and cmd_rx.vhd.

`plot` and `ViewOptions` are resolved lazily, because importing them pulls in
pyqtgraph and Qt. Headless users -- the CLI, capture scripts, the selftest, and
anything running over SSH or in CI -- get `import ctdsp` without a GUI stack.
"""

from .protocol import (
    PROTO_VERSION,
    FrameType,
    Command,
    AckStatus,
    Event,
    EventBatch,
    Status,
    Hello,
    Ack,
    FrameSplitter,
    ProtocolError,
    decode_payload,
)
from .device import CtdspDevice, DeviceError, LinkStats, find_ports

DEFAULT_BAUD = 3_000_000


def receive(port=None, *, baud=DEFAULT_BAUD, rtscts=True, settling_ns=None,
            start=True):
    """Connect to a board and start streaming events.

        dev = ctdsp.receive()            # first FTDI port
        dev = ctdsp.receive("COM11")     # explicit

    `port=None` picks the first port `find_ports()` offers, FTDI first. Beware
    that this is a guess: on a bench with more than one FTDI device it can pick
    the wrong one, and the CT-DSP is the channel-B interface specifically. Pass
    the port explicitly whenever you know it.

    Opening the device performs the HELLO handshake, so a bad baud rate, a
    stale bitstream or a dead board fails here rather than as silence later.
    `settling_ns` retunes the comparator settling window before streaming
    begins. `start=False` connects without streaming, for callers that only want
    to issue commands.

    Returns a `CtdspDevice`, which is a context manager::

        with ctdsp.receive("COM11") as dev:
            for event in dev.events(timeout=5):
                ...
    """
    if port is None:
        ports = find_ports()
        if not ports:
            raise DeviceError("no serial ports found")
        port = ports[0]

    device = CtdspDevice(port, baudrate=baud, rtscts=rtscts)
    try:
        if settling_ns is not None:
            device.set_settling_ns(settling_ns)
        if start:
            device.reset()
            device.start()
    except Exception:
        # Do not strand the port on a half-open session.
        device.close()
        raise
    return device


def __getattr__(name):
    """Resolve the Qt-dependent names on first use (PEP 562)."""
    if name in ("plot", "ViewOptions", "DuplicateFilter", "GlitchFilter"):
        from . import view
        return getattr(view, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "PROTO_VERSION",
    "FrameType",
    "Command",
    "AckStatus",
    "Event",
    "EventBatch",
    "Status",
    "Hello",
    "Ack",
    "FrameSplitter",
    "ProtocolError",
    "decode_payload",
    "CtdspDevice",
    "DeviceError",
    "LinkStats",
    "find_ports",
    "receive",
    "plot",
    "ViewOptions",
]
