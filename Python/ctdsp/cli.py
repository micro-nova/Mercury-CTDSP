"""Command-line bring-up tool for the CT-DSP board.

    python -m ctdsp.cli ports
    python -m ctdsp.cli ping [PORT]
    python -m ctdsp.cli status [PORT]
    python -m ctdsp.cli reset [PORT]
    python -m ctdsp.cli capture [PORT] --count 1000 [--csv out.csv]
    python -m ctdsp.cli rate [PORT] --seconds 5

`ping` is the single most useful check after flashing: a successful HELLO
confirms the baud rate, framing, CRC and the device's reported clock all at
once, before any streaming is involved.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from typing import Optional

from .device import CtdspDevice, DeviceError, find_ports


def _open(port: Optional[str], baud: int, rtscts: bool) -> CtdspDevice:
    if port is None:
        ports = find_ports()
        if not ports:
            raise DeviceError("no serial ports found")
        port = ports[0]
    return CtdspDevice(port, baudrate=baud, rtscts=rtscts)


def cmd_ports(_args) -> int:
    ports = find_ports()
    if not ports:
        print("no serial ports found")
        return 1
    for name in ports:
        print(name)
    return 0


def cmd_ping(args) -> int:
    with _open(args.port, args.baud, not args.no_rtscts) as dev:
        h = dev.hello
        print(f"port          {dev.port}")
        print(f"protocol      v{h.proto_ver}")
        print(f"firmware id   0x{h.fw_id:08X}")
        print(f"device clock  {h.clk_freq_hz:,} Hz  "
              f"({1e9 / h.clk_freq_hz:.4f} ns per tick)")
        print(f"trigger bits  {h.n_bits}")
    return 0


def cmd_status(args) -> int:
    with _open(args.port, args.baud, not args.no_rtscts) as dev:
        s = dev.status()
        print(f"streaming        {s.streaming}")
        print(f"fifo level       {s.fifo_level}")
        print(f"overflow events  {s.overflow_events:,}")
        print(f"frames sent      {s.frames_sent:,}")
        print(f"flags            {s.flags!r}")
        if s.overflowed:
            print("\nThe FIFO has overflowed since the last reset: events were "
                  "dropped on the board itself. Either the event rate exceeds "
                  "what the link can carry, or the host stopped reading.")
    return 0


def cmd_reset(args) -> int:
    with _open(args.port, args.baud, not args.no_rtscts) as dev:
        dev.reset()
        print("device reset: FIFO cleared, counters zeroed, streaming stopped")
    return 0


def cmd_capture(args) -> int:
    with _open(args.port, args.baud, not args.no_rtscts) as dev:
        dev.reset()
        dev.start()
        collected = []
        deadline = time.monotonic() + args.timeout
        try:
            for event in dev.events(timeout=args.timeout):
                collected.append(event)
                if len(collected) >= args.count or time.monotonic() > deadline:
                    break
        except KeyboardInterrupt:
            pass
        dev.stop()

        if not collected:
            print("no events captured -- is anything driving the comparator inputs?",
                  file=sys.stderr)
            return 1

        if args.csv:
            with open(args.csv, "w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(["ticks", "seconds", "bits"])
                base = collected[0].ticks
                for e in collected:
                    writer.writerow([e.ticks,
                                     f"{dev.ticks_to_seconds(e.ticks - base):.9f}",
                                     e.bits])
            print(f"wrote {len(collected):,} events to {args.csv}")
        else:
            base = collected[0].ticks
            for e in collected:
                print(f"{dev.ticks_to_seconds(e.ticks - base):.9f}  "
                      f"{e.ticks:>14}  0x{e.bits:02X}")

        print(f"\n{dev.stats}", file=sys.stderr)
    return 0


def cmd_rate(args) -> int:
    """Measure sustained event throughput -- useful for checking headroom."""
    with _open(args.port, args.baud, not args.no_rtscts) as dev:
        dev.reset()
        dev.start()
        t0 = time.monotonic()
        count = 0
        try:
            while time.monotonic() - t0 < args.seconds:
                count += len(dev.read_events())
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        elapsed = time.monotonic() - t0
        dev.stop()
        status = dev.status()

        print(f"{count:,} events in {elapsed:.2f} s = {count / elapsed:,.0f} events/s")
        print(f"device: {status.frames_sent:,} frames sent, "
              f"{status.overflow_events:,} overflow events")
        print(f"host:   {dev.stats}")
        if status.overflow_events:
            print("\nThe board dropped events: the source is faster than the "
                  "link. Raise the baud rate or reduce the event rate.")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--baud", type=int, default=3_000_000)
    parser.add_argument("--no-rtscts", action="store_true",
                        help="disable hardware flow control")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("ports", help="list serial ports").set_defaults(func=cmd_ports)

    for name, func, helptext in [
        ("ping", cmd_ping, "identify the device"),
        ("status", cmd_status, "read device counters"),
        ("reset", cmd_reset, "clear FIFO and counters"),
    ]:
        sp = sub.add_parser(name, help=helptext)
        sp.add_argument("port", nargs="?")
        sp.set_defaults(func=func)

    sp = sub.add_parser("capture", help="capture events")
    sp.add_argument("port", nargs="?")
    sp.add_argument("--count", type=int, default=100)
    sp.add_argument("--timeout", type=float, default=10.0)
    sp.add_argument("--csv")
    sp.set_defaults(func=cmd_capture)

    sp = sub.add_parser("rate", help="measure sustained event throughput")
    sp.add_argument("port", nargs="?")
    sp.add_argument("--seconds", type=float, default=5.0)
    sp.set_defaults(func=cmd_rate)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except DeviceError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
