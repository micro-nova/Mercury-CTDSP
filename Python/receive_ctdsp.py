"""CT-DSP live trigger monitor -- command line front end.

    python receive_ctdsp.py                       # full 8-bit code, auto-detect port
    python receive_ctdsp.py COM11                 # explicit port
    python receive_ctdsp.py COM11 --raw           # no smoothing
    python receive_ctdsp.py COM11 --bits 1,2,3,4  # per-comparator logic view
    python receive_ctdsp.py COM11 --autoscale     # reconstruction, Y fit to data
    python receive_ctdsp.py COM11 --window 25     # smooth rolling average
    python receive_ctdsp.py COM11 --time-span 2   # 2-second scrolling window
    python receive_ctdsp.py COM11 --ticks         # polarity tick per event
    python receive_ctdsp.py --list                # show available ports

This file is only argument parsing. Everything it does is available directly::

    import ctdsp
    dev = ctdsp.receive("COM11")
    ctdsp.plot(dev, window=5, ticks=True, time_span=2)

The filters, the views and the reasoning behind them live in `ctdsp/view.py`;
the argparse dests below are named to match `ViewOptions` field-for-field, so
the namespace this builds is handed straight to `ctdsp.plot`.
"""

from __future__ import annotations

import argparse
import sys

import ctdsp
from ctdsp.device import DeviceError, find_ports


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", nargs="?", help="serial port (default: first FTDI port)")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--baud", type=int, default=ctdsp.DEFAULT_BAUD)
    parser.add_argument("--raw", action="store_true", help="disable smoothing")
    parser.add_argument("--bits", help="comparators to show, MSB first, e.g. 1,2,3,4")
    parser.add_argument("--no-rtscts", action="store_true",
                        help="disable hardware flow control")
    parser.add_argument("--autoscale", action="store_true",
                        help="scale the amplitude axis to the data instead of "
                             "the full 0-255 range (reconstruction view)")
    parser.add_argument("--levels", type=int, default=0, metavar="N",
                        help="amplitude resolution in bits (1-8, default 8): "
                             "8 = 256 levels on a 0-255 axis, 7 = 128 levels "
                             "on 0-127. Value view only")
    parser.add_argument("--time-span", type=float, default=0.0, metavar="S",
                        help="show a scrolling window of the last S seconds")
    parser.add_argument("--ticks", action="store_true",
                        help="draw a polarity tick per event under the trace: "
                             "green above the baseline where the code stepped "
                             "up, red below where it stepped down")
    parser.add_argument("--no-stats", action="store_true",
                        help="hide the counters and title drawn on the plot. "
                             "The same numbers still print on exit")
    parser.add_argument("--allow-duplicates", action="store_true",
                        help="plot every event, including ones whose value "
                             "repeats the previous point")
    parser.add_argument("--keep-spikes", action="store_true",
                        help="keep smoothed points that deviate for one sample "
                             "and return to the previous level (no effect with "
                             "--raw, which has no smoothed value)")
    parser.add_argument("--keep-glitches", action="store_true",
                        help="keep excursions that step one way and immediately "
                             "step back")
    parser.add_argument("--cancel-ns", type=float, default=1000.0, metavar="NS",
                        help="cancel a +/- pair only if the excursion lasted "
                             "less than NS nanoseconds (default 1000). 0 "
                             "cancels regardless of how long it lasted")
    parser.add_argument("--window", type=int, default=0, metavar="N",
                        help="rolling average over N samples, unquantised. "
                             "Bigger looks smoother; mutually exclusive with "
                             "--smooth-bits")
    parser.add_argument("--smooth-bits", type=int, default=0, metavar="N",
                        help="buy N extra bits of amplitude resolution by "
                             "oversampling: averages 4**N samples and keeps "
                             "1/2**N-code steps. Ignored with --raw")
    parser.add_argument("--settling-ns", type=float,
                        help="set the comparator settling time before streaming")
    args = parser.parse_args(argv)

    if args.list:
        ports = find_ports()
        if not ports:
            print("no serial ports found")
        for name in ports:
            print(name)
        return 0

    try:
        device = ctdsp.receive(args.port, baud=args.baud,
                               rtscts=not args.no_rtscts,
                               settling_ns=args.settling_ns)
    except DeviceError as exc:
        print(f"Could not talk to the board: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:  # pyserial raises its own types
        print(f"Could not open {args.port or 'the port'}: {exc}", file=sys.stderr)
        return 1

    if args.settling_ns is not None:
        print(f"Settling time set to {args.settling_ns:.0f} ns")

    try:
        ctdsp.plot(device, args)
    except ValueError as exc:       # contradictory options
        device.close()
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
