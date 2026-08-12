"""CT-DSP live plotting: filters, views, and the `plot()` entry point.

Three lines is the whole program::

    import ctdsp
    dev = ctdsp.receive("COM11")
    ctdsp.plot(dev, window=5, ticks=True, time_span=2)

`ViewOptions` carries every display decision. Its field names match the CLI
flags exactly (`--smooth-bits` is `smooth_bits`, and so on), which is what lets
`receive_ctdsp.py` hand an argparse namespace straight to the same functions
that accept a `ViewOptions` -- there is one set of display code, not two.

Importing this module pulls in pyqtgraph and Qt. `ctdsp/__init__.py` therefore
resolves `plot` lazily, so `import ctdsp` stays cheap for headless work: the CLI,
capture scripts and the selftest never load a GUI stack they do not use.

Bit numbering follows the schematic nets B1..B8, where **B1 is the most
significant bit**. `--bits` takes them most-significant first, so `1,2,3,4`
means "show B1..B4 and build a 4-bit code with B1 on top".

Because the order is a runtime argument you can test an ordering hypothesis
without rebuilding the bitstream -- `--bits 4,3,2,1` composes the same four
comparators the other way round. Only the composed-value trace changes; the
individual bit traces are always labelled by their true net name.

The v1 monitor hunted for a 0xF0 0x0F sync pair byte by byte and then tried to
tell good packets from bad using timestamp monotonicity and an outlier filter.
Those heuristics discarded real samples, because they were compensating for a
framing scheme that could not distinguish a header from data. Framing is now
unambiguous (see ctdsp/protocol.py), so the filters are gone: what you see is
what the board sent, and anything actually lost is reported as a count rather
than smoothed over.

One filter does survive, and it is deliberately not one of those heuristics.
`diff.vhd` raises an event only when the comparator byte changes, but the value
written to the FIFO is sampled *after* the settling window -- `capture_fsm.vhd`
holds `capture` high across it and `delta.vhd` latches on every cycle it is
high -- so an input that glitches and settles back to where it started yields an
event whose value repeats the previous one. Such a point plots as a zero-length
step and carries nothing the preceding point did not already state, so it is
dropped. Unlike v1's outlier filter this rejects no information: the test is
exact equality, not a guess about plausibility. Rejections are counted and
shown, never silently absorbed, and `--allow-duplicates` disables the filter
entirely.

Equality is tested against the quantity actually drawn, not the raw byte: the
masked value in the default view, and only the on-screen comparators under
`--bits`. Two events differing solely in a bit that is not displayed are
therefore treated as a repeat, because on screen they are one.

A second filter, `GlitchFilter`, is a policy choice rather than a lossless one
and is called out as such. It removes single-event excursions of the form
`A -> A^bit -> A`: one bit flips and reverses at the very next event. That middle
event is a real measurement the board took, so this is the kind of filter the
paragraph above criticises v1 for -- the difference is that it is exact rather
than statistical, it is counted, and `--keep-glitches` turns it off. Removing
the excursion leaves `A ... A`, which `DuplicateFilter` then collapses, so the
whole triple becomes a single plotted point.

Both filters run before smoothing, which matters: an excursion left in place
would pull a 4-sample average by a quarter of a code and show up as a
sub-LSB feature that the analog signal never had.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

from .device import CtdspDevice, DeviceError

# Display configuration
MAX_POINTS = 2000
SMOOTHING_WINDOW = 10

#: Mask applied to each trigger snapshot before plotting. The v1 script hard
#: coded `& 0xFE` to hide a comparator bit that was misbehaving on that board
#: revision. It is a per-board decision, not a property of the protocol, so it
#: lives here as a named constant -- set it to 0xFF to plot all eight bits.
TRIGGER_MASK = 0xFF

#: Distinct colours for the per-bit traces.
BIT_COLOURS = ["#4FC3F7", "#81C784", "#FFB74D", "#E57373",
               "#BA68C8", "#4DD0E1", "#FFF176", "#A1887F"]


#: Where each schematic net B<n> lands in the transmitted byte.
#:
#: The comparator nets are not in binary weight order on the connector -- they
#: are transposed within each pair. Measured significance, MSB first, is
#: B2 B1 B4 B3 B6 B5 B8 B7, and mercury.xdc maps the bus to match. So the byte
#: is a plain binary code, but recovering an individual *net* means looking up
#: its position here rather than assuming B1 is bit 7.
#:
#: Requires firmware 0x00000003 or later; earlier bitstreams used the naive
#: B1..B8 order and this table will not match them.
NET_TO_BIT = {1: 6, 2: 7, 3: 4, 4: 5, 5: 2, 6: 3, 7: 0, 8: 1}


@dataclass
class ViewOptions:
    """Every display decision, with the defaults the CLI uses.

    Field names match the argparse dests one-for-one, so an `argparse.Namespace`
    and a `ViewOptions` are interchangeable everywhere below. That is the whole
    trick behind having a library and a CLI without maintaining two code paths.
    """

    # what to draw
    bits: str | None = None        # e.g. "2,1,8,7" -> per-comparator logic view
    levels: int = 0                # resolution in bits: 8=256 levels, 7=128; 0=8
    autoscale: bool = False        # fit Y to data instead of 0-255
    time_span: float = 0.0         # seconds on screen; 0 = MAX_POINTS worth
    ticks: bool = False            # per-event polarity marks
    no_stats: bool = False         # hide counters and title

    # conditioning
    raw: bool = False              # no smoothing at all
    window: int = 0                # explicit rolling window, unquantised
    smooth_bits: int = 0           # oversample for N extra bits
    allow_duplicates: bool = False
    keep_glitches: bool = False
    cancel_ns: float = 1000.0      # +/- pair cancels only if shorter than this
    keep_spikes: bool = False

    # connection facts, filled in by plot() for the banner
    port_used: str = ""
    baud: int = 0
    no_rtscts: bool = False
    settling_ns: float | None = None


def level_shift(n_bits: int) -> int:
    """Bits to discard to render an 8-bit code at `n_bits` of resolution.

    Resolution, not masking: 8 bits is 256 levels spanning 0-255, 7 bits is 128
    levels spanning 0-127, 6 bits is 64 levels spanning 0-63. A code of 181 at
    7 bits becomes 181 >> 1 = 90, and the axis it is drawn on shrinks to match.

    Masking (181 & 0xFE = 180, still on 0-255) would keep the number large while
    coarsening the step. That is a different operation and not what a bit depth
    means -- dropping a bit halves the range as well as the count of levels.
    """
    if not 1 <= n_bits <= 8:
        raise ValueError(f"levels must be 1..8, got {n_bits}")
    return 8 - n_bits


def full_scale(n_bits: int) -> int:
    """Largest code at `n_bits` of resolution: 255 at 8 bits, 127 at 7."""
    return (1 << n_bits) - 1


def bit_position(bnum: int) -> int:
    """Map schematic net B<n> to its position in the transmitted byte."""
    if bnum not in NET_TO_BIT:
        raise ValueError(f"comparator number must be 1..8, got {bnum}")
    return NET_TO_BIT[bnum]


class DuplicateFilter:
    """Reject consecutive events whose plotted value is unchanged.

    The caller decides what "value" means -- it passes in the quantity that
    reaches the screen -- so the same filter serves both views without knowing
    anything about either. See the module docstring for why these repeats occur
    on a board that only reports transitions.
    """

    def __init__(self, enabled: bool = True) -> None:
        self.enabled = enabled
        self.rejected = 0
        self._last: int | None = None

    def accept(self, value: int) -> bool:
        """True if `value` should be plotted, False if it repeats the last one."""
        if not self.enabled:
            return True
        if value == self._last:
            self.rejected += 1
            return False
        self._last = value
        return True


class GlitchFilter:
    """Drop single-event excursions that flip one bit and immediately return.

    A comparator sitting on its threshold can produce a transition that reverses
    at the very next event: `A -> A^bit -> A`. The middle event is a real
    measurement, not a duplicate, so unlike `DuplicateFilter` this **does**
    discard information -- see the module docstring.

    Detection needs one event of lookahead, because an event is only known to be
    an excursion once its successor returns to the preceding value. So one event
    is always held back, and callers get it on the following `feed()`. During
    activity that lag is imperceptible; across an idle gap the last transition
    waits for the next one.

    Downstream, the `A ... A` that remains after removing the middle event is a
    duplicate, so pairing this with `DuplicateFilter` collapses the whole triple
    to a single point -- which is what "don't log it" means in practice.
    """

    def __init__(self, enabled: bool = True, max_ticks: int = 0) -> None:
        self.enabled = enabled
        self.max_ticks = max_ticks            # 0 = no time limit
        self.rejected = 0
        self._prev_value: int | None = None   # last value actually emitted
        self._pending = None                  # (value, event) awaiting lookahead

    def feed(self, value: int, event):
        """Yield the (value, event) pairs cleared for plotting."""
        if not self.enabled:
            yield value, event
            return

        if self._pending is None:
            self._pending = (value, event)
            return

        pending_value, pending_event = self._pending

        # An excursion cancels when it returns to the previous value AND did not
        # last long. The dwell test is what makes this a glitch filter rather
        # than a signal filter: without it, a slow, real alternation is
        # indistinguishable from a spurious blip and gets eaten too.
        #
        # This deliberately does NOT require a single-bit change. That test
        # excluded the case most worth catching -- a mid-scale crossing flips
        # every bit at once (0x7F <-> 0x80), so a major-carry glitch has a
        # popcount of 8 and would have sailed straight through.
        if (self._prev_value is not None
                and value == self._prev_value
                # The held value must actually differ, or it is a repeat rather
                # than an excursion. The old popcount==1 test enforced this as a
                # side effect; with that gone it has to be explicit.
                and pending_value != self._prev_value
                and (not self.max_ticks
                     or event.ticks - pending_event.ticks <= self.max_ticks)):
            self.rejected += 1
            self._pending = (value, event)
            return

        self._prev_value = pending_value
        self._pending = (value, event)
        yield pending_value, pending_event


def smooth(data, window, step=1.0):
    """Moving average, quantised to `step` rather than to whole codes.

    `step` is the whole point when oversampling. Averaging N samples of a
    dithering LSB recovers amplitude information *below* one code, and rounding
    the result back to an integer -- as this function used to do
    unconditionally -- discards exactly the bits the averaging just bought.
    Leaving step at 1.0 reproduces the original behaviour.
    """
    if len(data) < window:
        return data
    kernel = np.ones(window) / window
    avg = np.convolve(data, kernel, mode="valid")
    if not step:
        # No quantisation: keep full precision. This is what actually makes a
        # trace look smooth -- snapping the average back onto a grid reimposes
        # the staircase the averaging just removed, however wide the window.
        return avg
    return np.round(avg / step) * step


def despike(t, y):
    """Drop smoothed points that deviate for one sample and return.

    Where `GlitchFilter` works on raw codes before averaging, this works on the
    smoothed trace, so it catches excursions that only exist after smoothing --
    a run of codes whose average momentarily departs and comes straight back.

    A point qualifies when its neighbours agree exactly and it differs from
    them: `y[i-1] == y[i+1] != y[i]`. Exact equality is safe because `smooth`
    quantises to multiples of `step`, and every step used here (1, 1/2, 1/4,
    1/8, 1/16) is a negative power of two and therefore exact in binary floating
    point. No magnitude limit is applied: a large excursion that returns is more
    obviously spurious than a small one, not less.

    Returns (t, y, removed). First and last points are never candidates, since
    judging them would need a neighbour that does not exist -- so the newest
    point is always kept and the trace never lags.

    Single pass by design: removing a spike can expose another, but iterating
    would let one filter pass reshape the trace arbitrarily far from the data.

    Known limitation, inherent to any local "deviates and returns" test: a
    sustained alternation is indistinguishable from a train of spikes, because
    every interior point of `A B A B A` has matching neighbours. Such a trace is
    flattened to its endpoints. Smoothing makes this unlikely -- an alternation
    faster than the averaging window averages away before reaching here -- but
    if a genuine square wave ever survives smoothing, `--keep-spikes` is the
    escape hatch.
    """
    if len(y) < 3:
        return t, y, 0
    spike = (y[:-2] == y[2:]) & (y[1:-1] != y[:-2])
    if not spike.any():
        return t, y, 0
    keep = np.ones(len(y), dtype=bool)
    keep[1:-1] = ~spike
    return t[keep], y[keep], int(spike.sum())


def smoothing_plan(args):
    """Return (window, step, label) for the requested smoothing.

    Averaging N samples of white quantisation noise buys 10*log10(N) dB of SNR,
    and one bit is 6.02 dB, so each extra bit costs a 4x window. The step is the
    resolution that purchase makes meaningful: 1/2**bits of a code.

    Caveat worth stating plainly: this averages over *samples*, not over time.
    The board reports level crossings, so samples are not uniformly spaced and
    each one is weighted equally regardless of how long its value actually held.
    The result is a valid noise-reduction filter but not a true time-domain low
    pass, and the duplicate filter skews it further by removing repeats.
    """
    if args.window:
        # Unquantised: an explicit window is a request for a smooth-looking
        # trace, not for a defensible resolution claim, so nothing is gained by
        # rounding the result onto a grid.
        return args.window, 0.0, (f"{args.window}-sample rolling average "
                                  f"(unquantised)")
    if args.smooth_bits:
        window = 4 ** args.smooth_bits
        step = 1.0 / (1 << args.smooth_bits)
        return window, step, (f"{args.smooth_bits}-bit oversampling "
                              f"({window}-sample average, {step:g}-code steps)")
    return SMOOTHING_WINDOW, 1.0, f"{SMOOTHING_WINDOW}-sample moving average"


def trim_window(times, *parallel, span=0.0, max_points=MAX_POINTS):
    """Drop samples older than `span` seconds, then cap the count.

    `times` is ascending, so the cut point is a bisect rather than a scan.
    `parallel` holds lists indexed in lockstep with `times`; they are trimmed
    identically so a sample and its timestamp can never drift apart.

    `max_points` still applies on top of `span`. It bounds redraw cost, which is
    proportional to retained points: without it, asking for a long span during a
    burst would quietly stall the UI. When the cap bites, the visible window is
    shorter than requested -- the caller reports that rather than pretending.
    """
    if span and times:
        drop = bisect.bisect_left(times, times[-1] - span)
        if drop:
            del times[:drop]
            for seq in parallel:
                del seq[:drop]
    if len(times) > max_points:
        del times[:-max_points]
        for seq in parallel:
            del seq[:-max_points]


def make_step(x, y):
    """Zero-order hold: hold each value until the next sample.

    Events are non-uniformly spaced, so a straight line between them would
    imply intermediate values the board never reported.
    """
    sx, sy = [], []
    for i in range(len(x)):
        if i > 0:
            sx.append(x[i])
            sy.append(y[i - 1])
        sx.append(x[i])
        sy.append(y[i])
    return np.array(sx), np.array(sy)


def parse_bits(spec: str) -> list[int]:
    nums = [int(tok) for tok in spec.replace(" ", "").split(",") if tok]
    if not nums:
        raise ValueError("--bits needs at least one comparator number")
    for n in nums:
        bit_position(n)  # validates range
    if len(set(nums)) != len(nums):
        raise ValueError("--bits must not repeat a comparator")
    return nums


def build_banner(device, args, bnums) -> None:
    hello = device.hello
    print("=" * 62)
    print("  CT-DSP Trigger Monitor")
    print("=" * 62)
    print(f"Port:        {args.port_used} @ {args.baud:,} baud"
          f"{'' if args.no_rtscts else ' (RTS/CTS)'}")
    print(f"Protocol:    v{hello.proto_ver}   firmware id 0x{hello.fw_id:08X}")
    print(f"Device clock:{hello.clk_freq_hz / 1e6:.3f} MHz "
          f"({1e9 / hello.clk_freq_hz:.3f} ns per timestamp tick)")
    if bnums:
        mapping = "  ".join(f"B{n}=bit{bit_position(n)}" for n in bnums)
        print(f"Showing:     {len(bnums)} comparators, MSB first -> {mapping}")
        weights = "  ".join(
            f"B{n}x{1 << (len(bnums) - 1 - i)}" for i, n in enumerate(bnums))
        print(f"Composed as: {weights}")
    else:
        n_bits = args.levels or 8
        print(f"Resolution:  {n_bits} bits = {1 << n_bits:,} levels, "
              f"0-{full_scale(n_bits)}"
              f"{'' if n_bits == 8 else f'  (from {hello.n_bits}-bit codes)'}")
        print(f"Event ticks: "
              f"{'green = step up, red = step down' if args.ticks else 'off (--ticks to enable)'}")
        print(f"Amplitude:   "
              f"{'auto-scaled to the data' if args.autoscale else f'fixed 0-{full_scale(args.levels or 8)} full scale'}")
        print(f"Smoothing:   "
              f"{'off' if args.raw else smoothing_plan(args)[2]}")
    print(f"Duplicates:  "
          f"{'kept (--allow-duplicates)' if args.allow_duplicates else 'rejected when the plotted value repeats'}")
    if args.keep_glitches:
        glitch_mode = "kept (--keep-glitches)"
    elif args.cancel_ns > 0:
        glitch_mode = (f"+/- pairs cancelled if the excursion lasted "
                       f"< {args.cancel_ns:,.0f} ns")
    else:
        glitch_mode = "+/- pairs cancelled regardless of duration"
    print(f"Glitches:    {glitch_mode}")
    if not bnums:
        if args.raw:
            spike_mode = "n/a (--raw: no smoothed value to despike)"
        elif args.keep_spikes:
            spike_mode = "kept (--keep-spikes)"
        else:
            spike_mode = "rejected: smoothed points that deviate and return"
        print(f"Spikes:      {spike_mode}")
    print(f"Time axis:   "
          f"{f'last {args.time_span:g} s (scrolling)' if args.time_span else f'whatever {MAX_POINTS:,} events span'}")
    print("Close the window to exit")
    print("=" * 62)


def run_bit_view(device, args, bnums, dedup, glitch):
    """Logic-analyser style view: one row per comparator, plus a composed code."""
    positions = [bit_position(n) for n in bnums]
    nbits = len(bnums)

    # The bits that reach the screen. Equality on this mask is exactly "nothing
    # the viewer can see has changed", which is the right test for the filter --
    # and it is far cheaper than recomposing the weighted code per event.
    sel_mask = 0
    for pos in positions:
        sel_mask |= 1 << pos

    times: list[float] = []
    raw: list[int] = []
    start_ticks: int | None = None

    app = pg.mkQApp()
    win = pg.GraphicsLayoutWidget(show=True, title="CT-DSP Comparator Bits")
    win.resize(1400, 180 + 110 * nbits)

    bit_plots, bit_curves = [], []
    for row, (n, colour) in enumerate(zip(bnums, BIT_COLOURS)):
        p = win.addPlot(row=row, col=0)
        p.setLabel("left", f"B{n}")
        p.setYRange(-0.15, 1.15)
        p.getAxis("left").setTicks([[(0, "0"), (1, "1")]])
        p.showGrid(x=True, y=True, alpha=0.25)
        p.setMouseEnabled(y=False)
        if row:
            p.setXLink(bit_plots[0])
        p.hideAxis("bottom")
        bit_plots.append(p)
        bit_curves.append(p.plot(pen=pg.mkPen(colour, width=1.6)))

    code_plot = win.addPlot(row=nbits, col=0)
    code_plot.setLabel("left", f"code ({nbits}b)")
    code_plot.setLabel("bottom", "Time", units="s")
    code_plot.setYRange(-0.5, (1 << nbits) - 0.5)
    code_plot.showGrid(x=True, y=True, alpha=0.25)
    code_plot.setXLink(bit_plots[0])
    code_curve = code_plot.plot(pen=pg.mkPen("w", width=1.6))

    stats_label = None
    if not args.no_stats:
        stats_label = pg.LabelItem(justify="left")
        stats_label.setParentItem(bit_plots[0].getViewBox())
        stats_label.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 6))

    flips = [0] * nbits

    def update():
        nonlocal start_ticks

        for e in device.read_events():
            for value, ev in glitch.feed(e.bits & sel_mask, e):
                if not dedup.accept(value):
                    continue
                if start_ticks is None:
                    start_ticks = ev.ticks
                times.append(device.ticks_to_seconds(ev.ticks - start_ticks))
                raw.append(ev.bits)

        trim_window(times, raw, span=args.time_span)
        if len(raw) < 2:
            return

        t = np.array(times)
        arr = np.array(raw, dtype=np.uint16)

        code = np.zeros(len(arr), dtype=np.int32)
        for i, pos in enumerate(positions):
            b = (arr >> pos) & 1
            sx, sy = make_step(t, b)
            bit_curves[i].setData(sx, sy)
            flips[i] = int(np.count_nonzero(np.diff(b)))
            code |= b.astype(np.int32) << (nbits - 1 - i)

        sx, sy = make_step(t, code)
        code_curve.setData(sx, sy)

        if args.time_span:
            bit_plots[0].setXRange(times[-1] - args.time_span, times[-1],
                                   padding=0)
        elif len(times) >= MAX_POINTS:
            bit_plots[0].setXRange(sx[0], sx[-1], padding=0.02)

        if not args.no_stats:
            for i, n in enumerate(bnums):
                bit_plots[i].setLabel("left", f"B{n}  ({flips[i]})")

        if stats_label is not None:
            s = device.stats
            bad = s.frames_lost or s.crc_errors or s.queue_drops
            stats_label.setText(
                f"<span style='font-size:10pt;color:#FFFFFF'>"
                f"Events {s.events:,} &nbsp; Frames {s.frames:,} &nbsp; "
                f"code {int(code[-1])}/{(1 << nbits) - 1}</span>"
                f"<span style='font-size:10pt;color:#9E9E9E'>"
                f" &nbsp;|&nbsp; dup {dedup.rejected:,} "
                f"glitch {glitch.rejected:,}</span>"
                f"<span style='font-size:10pt;color:{'#FF6B6B' if bad else '#FFFFFF'}'>"
                f" &nbsp;|&nbsp; lost {s.frames_lost:,} crc {s.crc_errors:,} "
                f"drop {s.queue_drops:,}</span>")
        if not args.no_stats:
            code_plot.setTitle(
                "transitions per comparator: "
                + "   ".join(f"B{n}={flips[i]:,}" for i, n in enumerate(bnums)))

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(33)
    # The caller must hold on to these. They are the only Python references to
    # the window and timer, and letting them fall out of scope lets the garbage
    # collector destroy the Qt widget before it is ever shown.
    return app, (win, timer, bit_plots, bit_curves, code_plot, code_curve, stats_label)


def run_value_view(device, args, dedup, glitch):
    """Original view: the full 8-bit code as one trace."""
    times: list[float] = []
    values: list[int] = []
    start_ticks: int | None = None
    window, step, _ = smoothing_plan(args)
    n_bits = args.levels or 8
    shift = level_shift(n_bits)
    top = full_scale(n_bits)

    app = pg.mkQApp()
    win = pg.GraphicsLayoutWidget(show=True, title="CT-DSP Trigger Monitor")
    win.resize(1400, 600)

    plot = win.addPlot(title=None if args.no_stats else "CT-DSP Triggers")
    plot.setLabel("bottom", "Time", units="s")
    plot.showGrid(x=True, y=True, alpha=0.3)
    if args.autoscale:
        # The full-scale window is the honest default, but it buries a signal
        # that only moves by an LSB or two: a +-1 dither inside a 255-wide axis
        # is one pixel. Auto-ranging trades absolute context for the ability to
        # see the shape at all, so the title reports the span to put it back.
        plot.setLabel("left", "Value (auto-scaled)")
        plot.enableAutoRange(axis="y")
    else:
        plot.setLabel("left", f"Value (0-{top})")
        # padding=0 matters: pyqtgraph pads a requested range by ~2% unless told
        # otherwise, so without it the axis would run to roughly -5..260.
        plot.setYRange(0, top, padding=0)
    curve = plot.plot(pen=pg.mkPen("b", width=1.5))

    # Event ticks, drawn under the trace: one short vertical mark per event,
    # green where the code stepped up, red where it stepped down. Two items
    # rather than per-tick objects -- `connect="pairs"` draws each consecutive
    # point pair as its own segment, so every tick costs two array entries
    # instead of a QGraphicsItem.
    up_ticks = dn_ticks = None
    if args.ticks:
        up_ticks = plot.plot(pen=pg.mkPen("#4CAF50", width=1.4))
        dn_ticks = plot.plot(pen=pg.mkPen("#E53935", width=1.4))

    # Parent the label straight to the view box. The v1 code also added it to
    # the window grid at (0,0), where the plot already sits -- Qt warned
    # "Cell (0, 0) already taken" and the grid placement was discarded anyway.
    stats_label = None
    if not args.no_stats:
        stats_label = pg.LabelItem(justify="left")
        stats_label.setParentItem(plot.getViewBox())
        stats_label.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 10))

    def update():
        nonlocal start_ticks

        for event in device.read_events():
            # Filter on the value that is plotted, at the resolution it is
            # plotted at: below 8 bits, two codes that differ only in a
            # discarded low bit are the same point on screen, so they must
            # count as a repeat rather than a transition.
            for value, ev in glitch.feed((event.bits & TRIGGER_MASK) >> shift,
                                         event):
                if not dedup.accept(value):
                    continue
                if start_ticks is None:
                    start_ticks = ev.ticks
                times.append(device.ticks_to_seconds(ev.ticks - start_ticks))
                values.append(value)

        trim_window(times, values, span=args.time_span)
        if not values:
            return

        t = np.array(times)
        v = np.array(values, dtype=np.float64)

        spikes = 0
        if args.raw or len(v) <= window:
            st, sv = t, v
            latest_smoothed = v[-1]
        else:
            sv = smooth(v, window, step)
            st = t[window - 1:]
            if not args.keep_spikes:
                st, sv, spikes = despike(st, sv)
            latest_smoothed = sv[-1]
        sx, sy = make_step(st, sv)

        curve.setData(sx, sy)
        if args.time_span:
            plot.setXRange(times[-1] - args.time_span, times[-1], padding=0)
        elif len(times) >= MAX_POINTS:
            plot.setXRange(sx[0], sx[-1], padding=0.02)

        if up_ticks is not None and len(v) > 1:
            # Ticks mark the raw events, not the smoothed trace: they are a
            # record of when the board reported a crossing and which way it
            # went, so they must not inherit the averaging window's lag.
            delta = np.diff(v)
            edge_t = t[1:]
            lo_y, hi_y = plot.getViewBox().viewRange()[1]
            span = hi_y - lo_y
            # Ups above a baseline, downs below it. Drawing both at the same y
            # made them overwrite each other -- whichever item drew last won the
            # pixel -- so their relative counts could not be judged by eye. That
            # defeats the point of a display meant for exactly that comparison.
            base = lo_y + 0.05 * span
            for item, sel, tip in ((up_ticks, delta > 0, base + 0.04 * span),
                                   (dn_ticks, delta < 0, base - 0.04 * span)):
                xs = edge_t[sel]
                item.setData(np.repeat(xs, 2),
                             np.tile([base, tip], len(xs)),
                             connect="pairs")

        if stats_label is not None:
            s = device.stats
            loss_colour = ("#FF6B6B"
                           if (s.frames_lost or s.crc_errors or s.queue_drops)
                           else "#FFFFFF")
            stats_label.setText(
                f"<span style='font-size:11pt; color:#FFFFFF;'>"
                f"Events: {s.events:,}&nbsp; Frames: {s.frames:,}&nbsp; "
                f"Latest: {int(v[-1])}</span>"
                f"<span style='font-size:11pt; color:#9E9E9E;'>"
                f"&nbsp;|&nbsp; Dup: {dedup.rejected:,}&nbsp; "
                f"Glitch: {glitch.rejected:,}&nbsp; "
                f"Spike: {spikes:,}/win</span>"
                f"<span style='font-size:11pt; color:{loss_colour};'>"
                f"&nbsp;|&nbsp; Lost frames: {s.frames_lost:,}&nbsp; "
                f"CRC: {s.crc_errors:,}&nbsp; Dropped: {s.queue_drops:,}</span>"
            )
        if args.no_stats:
            pass          # waveform only: no counters, no title
        elif args.autoscale:
            lo, hi = int(v.min()), int(v.max())
            plot.setTitle(
                f"CT-DSP reconstruction - latest {int(v[-1])} "
                f"&nbsp;|&nbsp; window spans {lo}-{hi} of 0-{top} "
                f"({hi - lo + 1} distinct code{'' if hi == lo else 's'})")
        else:
            plot.setTitle(f"CT-DSP Triggers - latest {int(v[-1])}")

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(33)
    return app, (win, timer, plot, curve, stats_label)


def validate(options) -> None:
    """Raise ValueError if the options contradict each other.

    Shared by the library and the CLI so both reject the same things. The CLI
    turns the exception into a message and a non-zero exit; a library caller
    gets the exception.
    """
    if options.time_span < 0:
        raise ValueError("time_span must be positive")
    if options.levels and not 1 <= options.levels <= 8:
        raise ValueError(f"levels must be 1..8, got {options.levels}")
    if options.levels and options.bits:
        raise ValueError("levels applies to the value view; bits already "
                         "selects comparators explicitly")
    if options.window and options.smooth_bits:
        raise ValueError("window and smooth_bits both set the averaging "
                         "window; use one or the other")
    if options.window and not 2 <= options.window <= MAX_POINTS // 4:
        raise ValueError(f"window must be between 2 and {MAX_POINTS // 4} "
                         f"(the plot only retains {MAX_POINTS:,} points)")
    if options.smooth_bits < 0:
        raise ValueError("smooth_bits must be 0 or more")
    if options.smooth_bits > 4:
        # 4 bits is already a 256-sample window, well past MAX_POINTS/8.
        raise ValueError("smooth_bits above 4 averages more samples than the "
                         "plot retains")


def plot(device, options=None, *, quiet=False, **kwargs):
    """Open the live plot for `device` and run until the window is closed.

    `options` may be a ViewOptions, an argparse.Namespace, or omitted -- any
    keyword arguments are applied on top, so the common case is a single call::

        ctdsp.plot(dev, window=5, ticks=True, time_span=2)

    The device should already be streaming (`receive()` does that). Returns a
    dict of closing statistics; `quiet` suppresses printing them.
    """
    if options is None:
        options = ViewOptions()
    for key, value in kwargs.items():
        if not hasattr(options, key):
            raise TypeError(f"unknown display option {key!r}")
        setattr(options, key, value)
    validate(options)

    bnums = parse_bits(options.bits) if options.bits else None

    if not options.port_used:
        options.port_used = device.port
    if not options.baud:
        options.baud = device._serial.baudrate

    if not quiet:
        build_banner(device, options, bnums)

    dedup = DuplicateFilter(enabled=not options.allow_duplicates)
    # The wire carries ticks, not nanoseconds, and only the host knows the
    # device clock (from HELLO) -- same division of labour as set_settling_ns.
    cancel_ticks = 0
    if options.cancel_ns > 0:
        cancel_ticks = max(1, round(options.cancel_ns * device.hello.clk_freq_hz / 1e9))
    glitch = GlitchFilter(enabled=not options.keep_glitches, max_ticks=cancel_ticks)

    # `keepalive` must stay referenced for the lifetime of the event loop; see
    # the note in run_bit_view.
    if bnums:
        app, keepalive = run_bit_view(device, options, bnums, dedup, glitch)
    else:
        app, keepalive = run_value_view(device, options, dedup, glitch)
    assert keepalive is not None

    try:
        app.exec_() if hasattr(app, "exec_") else app.exec()
    except KeyboardInterrupt:
        pass

    return _finish(device, dedup, glitch, quiet)


def _finish(device, dedup, glitch, quiet):
    """Read closing status and shut the port down.

    Every step is guarded, because the failure this most often meets is the USB
    device disappearing mid-session -- and pyserial raises SerialException, not
    DeviceError, so a narrow except here loses the statistics AND leaks the
    port. Whatever could be read is still returned.
    """
    result = {"host": device.stats, "duplicates": dedup.rejected,
              "glitches": glitch.rejected, "device": None, "error": None}
    try:
        result["device"] = device.status()
    except Exception as exc:          # DeviceError, SerialException, OSError
        result["error"] = exc

    if not quiet:
        final = result["device"]
        if final is not None:
            print(f"\nDevice: fifo_level={final.fifo_level} "
                  f"overflow_events={final.overflow_events:,} "
                  f"frames_sent={final.frames_sent:,}")
            if final.overflowed:
                print("  NOTE: the device FIFO overflowed -- events were dropped "
                      "on the board, before they ever reached the link.")
        else:
            print(f"\nCould not read final status: {result['error']}")
        print(f"Host:   {result['host']}")
        if dedup.enabled:
            print(f"Filter: {dedup.rejected:,} duplicate events rejected "
                  f"(plotted value unchanged)")
        if glitch.enabled:
            print(f"        {glitch.rejected:,} glitch events rejected "
                  f"(+/- pair, returned next event)")

    try:
        device.close()
        if not quiet:
            print("Serial port closed")
    except Exception as exc:
        if not quiet:
            print(f"Serial port did not close cleanly: {exc}")
    return result
