"""Show the CT-DSP live plot.

    py -3.13 example.py                # first FTDI port, 8 bits
    py -3.13 example.py COM11          # pin the port
    py -3.13 example.py COM11 7        # 7 bits = 128 levels, 0-127 axis

Three lines is the whole thing. Everything below the imports is optional taste:
swap the keywords for the view you want.

    ctdsp.plot(dev)                                   # 8-bit code, 0-255
    ctdsp.plot(dev, levels=7)                         # 128 levels, 0-127
    ctdsp.plot(dev, autoscale=True, smooth_bits=2)    # zoom in, quarter-code
    ctdsp.plot(dev, bits="2,1,8,7")                   # per-comparator logic view
    ctdsp.plot(dev, keep_glitches=True, allow_duplicates=True)   # unfiltered

`levels` is resolution in bits, not a comparator count: 8 bits is 256 levels on
a 0-255 axis, 7 bits is 128 levels on 0-127. Dropping a bit halves the range as
well as the number of levels, and the axis follows, so the trace keeps filling
the frame instead of collapsing into the bottom half.

`receive()` with no port takes the first FTDI device it finds, which is a guess:
the CT-DSP is the channel-B interface, and a bench with a second FTDI device can
enumerate it first. Pass the port explicitly when you know it.
"""
import sys
import ctdsp

dev = ctdsp.receive(sys.argv[1] if len(sys.argv) > 1 else None)

ctdsp.plot(dev, levels=int(sys.argv[2]) if len(sys.argv) > 2 else 8,
           window=5, ticks=True, time_span=2, no_stats=True)
