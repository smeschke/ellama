#!/usr/bin/env python3
"""Two single-encoder boards sharing one ENC stream. No hardware needed:

    ELLAMA_SERIAL_PORT=/dev/none mcp_server/.venv/bin/python mcp_server/test_two_board_enc.py

Each board sends left-only or right-only ENC packets on its own sender clock.
The merge must hold per-wheel distances, keep availability stable (no flicker),
difference each wheel against its own clock, and still report a board that
actually dies as unavailable.
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("ELLAMA_SERIAL_PORT", "/dev/none_for_tests")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # for bridge/
import bridge  # noqa: E402

STEP = 82  # counts per 20 ms packet: 4100 counts/s per wheel


def fresh():
    return bridge.EncoderState(dict(bridge.DEFAULT_CONFIG))


def expected_speed_in_s():
    return STEP / 0.02 * bridge.in_per_tick(dict(bridge.DEFAULT_CONFIG))


def test_interleaved_boards_hold_both_wheels():
    enc = fresh()
    now = 1000.0
    ms_a, ms_b = 5000, 9000  # unrelated sender clocks
    cl = cr = 0
    for n in range(200):  # 100 packets per board, alternating
        now += 0.01
        if n % 2 == 0:
            ms_a += 20
            cl += STEP
            sl, sr = enc.update(cl, None, ms_a, now=now)
            if n >= 2:  # packet 0 predates the right board's first word: [True, False] is right
                assert enc.available == [True, True], (n, enc.available)  # no flicker
            assert sr is None, (n, sr)  # silent wheel reports no speed, not zero
            if n >= 4:
                assert sl is not None, (n, sl)
                assert abs(sl - expected_speed_in_s()) < 0.1 * expected_speed_in_s(), (n, sl)
        else:
            ms_b += 20
            cr += STEP
            sl, sr = enc.update(None, cr, ms_b, now=now)
            assert enc.available == [True, True], (n, enc.available)
            assert sl is None, (n, sl)
            if n >= 5:
                assert sr is not None, (n, sr)
                assert abs(sr - expected_speed_in_s()) < 0.1 * expected_speed_in_s(), (n, sr)
    k = bridge.in_per_tick(dict(bridge.DEFAULT_CONFIG))
    assert abs(enc.dist_l - (cl - STEP) * k) < 1e-9, (enc.dist_l, cl)
    assert abs(enc.dist_r - (cr - STEP) * k) < 1e-9, (enc.dist_r, cr)
    assert enc.heading_deg is not None  # both wheels live: heading defined


def test_dead_board_reads_unavailable():
    enc = fresh()
    now = 2000.0
    ms_a, ms_b = 5000, 9000
    cl = cr = 0
    for n in range(100):  # 1 s of both boards
        now += 0.01
        if n % 2 == 0:
            ms_a += 20
            cl += STEP
            enc.update(cl, None, ms_a, now=now)
        else:
            ms_b += 20
            cr += STEP
            enc.update(None, cr, ms_b, now=now)
    assert enc.available == [True, True]
    frozen_r = enc.dist_r
    for _ in range(120):  # board B dies; board A alone for 1.2 s
        now += 0.01
        ms_a += 20
        cl += STEP
        enc.update(cl, None, ms_a, now=now)
    assert enc.available == [True, False], enc.available
    assert enc.dist_r == frozen_r  # holds, never a fabricated zero
    assert enc.heading_deg is None  # one wheel live: no plausible heading
    k = bridge.in_per_tick(dict(bridge.DEFAULT_CONFIG))
    assert abs(enc.dist_l - (cl - STEP) * k) < 1e-9


def test_single_dual_board_unchanged():
    enc = fresh()
    now, ms, cl, cr = 3000.0, 1000, 0, 0
    for _ in range(50):  # one board, both channels, one clock
        now += 0.02
        ms += 20
        cl += STEP
        cr += STEP
        sl, sr = enc.update(cl, cr, ms, now=now)
        assert enc.available == [True, True]
    k = bridge.in_per_tick(dict(bridge.DEFAULT_CONFIG))
    assert abs(enc.dist_avg - (cl - STEP) * k) < 1e-9
    assert sl is not None and sr is not None
    # ...and a persistently silent channel still goes unavailable (after the
    # latch timeout, not instantly as before).
    for _ in range(70):
        now += 0.02
        ms += 20
        cr += STEP
        enc.update(None, cr, ms, now=now)
    assert enc.available == [False, True], enc.available


def main():
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print("PASS", fn.__name__)
        except AssertionError as e:
            failed += 1
            print("FAIL", fn.__name__, "-", e)
    print("%d failed, %d passed" % (failed, len(fns) - failed))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
