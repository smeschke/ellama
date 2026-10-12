#!/usr/bin/env python3
"""Tests for tilt_guard.py and the RobotLink choke point. No hardware needed:

    ELLAMA_SERIAL_PORT=/dev/none python3 mcp_server/test_tilt_guard.py

Also replays every recordings/*/*.csv that has pitch/roll columns through the guard and
prints the worst tilt seen -- real driving must not trip it.
"""
import csv
import glob
import math
import os
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("ELLAMA_SERIAL_PORT", "/dev/none_for_tests")
sys.path.insert(0, str(Path(__file__).resolve().parent))
import tilt_guard as tg  # noqa: E402
import motion_guard as mg  # noqa: E402

MOUNT = (1.0, -7.5)  # the IMU rests like this on a flat floor (pitch, roll), as measured


def fresh(**kw):
    g = tg.TiltGuard(**kw)
    t = 0
    for _ in range(tg.REF_SAMPLES + 2):  # sit still -> auto reference
        t += 20
        g.update(MOUNT[0], MOUNT[1], t, gyro_dps=2.6)
    assert g.ready and g.snapshot()["reference_source"] == "auto"
    return g, t


def lean(g, t, roll_rate_dps, to_deg, pitch_rate_dps=0.0):
    """Lean the robot at a constant rate until it trips or reaches to_deg of extra roll;
    returns (fault or None, tilt at the end, t)."""
    extra = 0.0
    while extra < to_deg:
        t += 20
        extra += roll_rate_dps * 0.02
        f = g.update(MOUNT[0] + pitch_rate_dps * 0.02 * ((t // 20) % 10000), MOUNT[1] + extra, t)
        if f:
            return f, g.tilt_deg, t
    return None, g.tilt_deg, t


def test_mount_offset_is_ignored():
    g, t = fresh()
    assert g.tilt_deg < 0.01, g.tilt_deg  # reads ~0, not 7.6
    # ...and a 5 deg lean reads 5, not 5 + the mounting offset
    for i in range(10):
        t += 20
        g.update(MOUNT[0], MOUNT[1] + 5.0, t)
    assert abs(g.tilt_deg - 5.0) < 0.05, g.tilt_deg


def test_slow_bank_trips_at_the_limit_not_before():
    g, t = fresh()
    f, tilt, _ = lean(g, t, roll_rate_dps=3.0, to_deg=40)  # 3 deg/s: an ordinary bank
    assert f is not None and g.tripped
    assert 24.0 <= f["tilt_deg"] <= 26.0, f  # lets it drive right up to ~25, stops there
    assert f["limit_deg"] == 25.0


def test_fast_lean_trips_early_but_never_below_floor():
    g, t = fresh()
    f, tilt, _ = lean(g, t, roll_rate_dps=120.0, to_deg=40)  # a real tip: 120 deg/s
    assert f is not None
    # caught before 25 (the rate estimate + 2-sample confirm cost a few degrees), not at ~0
    assert 14.0 <= f["tilt_deg"] <= 23.0, f
    assert f["predicted_deg"] >= 25.0


def test_bump_at_small_tilt_does_not_trip():
    g, t = fresh()
    # curb strike: 100 deg/s for 60 ms (6 deg), then back
    for r in (50, 100, 100, 100, 50, -50, -100, -100, -100, -50):
        t += 20
        MOUNT_ROLL = MOUNT[1]
        f = g.update(MOUNT[0], MOUNT_ROLL + (6.0 if r > 0 else 0.0), t)
        assert f is None and not g.tripped


def test_single_glitch_sample_does_not_trip():
    g, t = fresh()
    t += 20
    assert g.update(MOUNT[0], MOUNT[1] + 60.0, t) is None  # I2C garbage, one sample
    t += 20
    assert g.update(MOUNT[0], MOUNT[1], t) is None
    assert not g.tripped


def test_diagonal_bank_counts_in_full():
    g, t = fresh()
    # 18 deg of roll AND 18 deg of pitch: neither alone is near 25, together they are ~25.5
    for i in range(6):
        t += 20
        f = g.update(MOUNT[0] + 18.0, MOUNT[1] + 18.0, t)
    assert g.tilt_deg > 24.0, g.tilt_deg
    assert g.tripped, "combined lean must trip"


def test_latch_and_clear():
    g, t = fresh()
    lean(g, t, roll_rate_dps=3.0, to_deg=40)
    assert g.tripped
    ok, msg = g.clear()
    assert not ok and "still leaning" in msg
    # robot righted: back to level, still
    for i in range(10):
        t += 5000
        g.update(MOUNT[0], MOUNT[1], t)
    assert g.tripped, "must stay latched until cleared, even once upright"
    ok, msg = g.clear()
    assert ok, msg
    assert not g.tripped


def test_peak_tilt_is_tracked_and_reset_on_clear():
    g, t = fresh()
    assert g.snapshot()["max_tilt_deg"] == 0.0
    lean(g, t, roll_rate_dps=3.0, to_deg=40)
    peak = g.snapshot()["max_tilt_deg"]
    assert peak >= 17, peak                    # at least where it tripped
    for i in range(10):                        # righted
        t += 5000
        g.update(MOUNT[0], MOUNT[1], t)
    assert g.snapshot()["max_tilt_deg"] == peak, "peak must survive the robot righting itself"
    ok, msg = g.clear()
    assert ok, msg
    assert g.snapshot()["max_tilt_deg"] < 2.0, "clearing a fault resets the peak"


def test_disabled_never_trips_but_reports():
    g, t = fresh(enabled=False)
    f, tilt, _ = lean(g, t, roll_rate_dps=3.0, to_deg=40)
    assert f is None and not g.tripped and g.tilt_deg > 30


def test_reference_waits_for_stillness():
    g = tg.TiltGuard()
    t = 0
    for i in range(100):  # being carried: gyro high
        t += 20
        g.update(10.0, 10.0, t, gyro_dps=60.0)
    assert not g.ready
    for i in range(tg.REF_SAMPLES):
        t += 20
        g.update(2.0, 3.0, t, gyro_dps=2.0)
    assert g.ready


# ---- RobotLink choke point ------------------------------------------------------------

class FakeSer:
    def __init__(self):
        self.lines = []
    def write(self, b):
        self.lines.append(b.decode().strip())
    def flush(self):
        pass


def make_link(rs, **guard_kw):
    link = rs.RobotLink.__new__(rs.RobotLink)
    link._wlock = threading.RLock()
    link.ser = FakeSer()
    link.tilt = tg.TiltGuard(**guard_kw)
    link.motion = mg.MotionGuard()
    link.last_imu_wall = time.monotonic()
    return link


def test_send_drive_is_the_choke_point():
    import robot_server as rs
    # no reference yet -> refuses (fail closed), but stop/zero always go out
    link = make_link(rs)
    try:
        link.send_drive(60, 60)
        assert False, "must refuse without a level reference"
    except rs.TiltFault as e:
        assert "no level reference" in str(e)
    assert link.ser.lines[-1] == "stop"
    link.send_drive(0, 0)
    assert link.ser.lines[-1] == "0 0"

    # reference ready, IMU live -> drives
    t = 0
    for _ in range(tg.REF_SAMPLES + 1):
        t += 20
        link.tilt.update(0.0, 0.0, t, gyro_dps=1.0)
    link.last_imu_wall = time.monotonic()
    link.send_drive(60, -60)
    assert link.ser.lines[-1] == "60 -60"

    # IMU goes quiet mid-move -> refuses
    link.last_imu_wall = time.monotonic() - 1.0
    try:
        link.send_drive(60, 60)
        assert False
    except rs.TiltFault as e:
        assert "no live IMU" in str(e)

    # tripped -> refuses, and the line that goes out is "stop", never a drive
    link.last_imu_wall = time.monotonic()
    link.tilt.tripped = True
    link.tilt.fault = dict(reason="test")
    n = len(link.ser.lines)
    try:
        link.send_drive(100, 100)
        assert False
    except rs.TiltFault as e:
        assert "tripped" in str(e)
    assert link.ser.lines[n:] == ["stop"], link.ser.lines[n:]


def test_tool_returns_fault_instead_of_raising():
    import robot_server as rs
    link = make_link(rs)
    link.tilt.tripped = True
    link.tilt.fault = dict(reason="leaned 30 deg")
    link.telemetry_snapshot = lambda: {"fake": True}
    rs.link = link
    try:
        out = rs.jog(50, 50, 1.0, confirmed_safe=True)  # the gate refuses before any wheel moves
        assert out["ok"] is False and "tripped" in out["error"], out
        assert not [l for l in link.ser.lines if l not in ("stop",)], link.ser.lines
        # and a fault raised mid-move comes back as a result via the decorator
        @rs._tilt_guarded
        def boom():
            raise rs.TiltFault("mid-move", {"x": 1})
        assert boom()["ok"] is False
    finally:
        rs.link = None


def test_motion_guard_through_the_link():
    import robot_server as rs
    link = make_link(rs, enabled=False)   # tilt guard off: isolate the motion guard
    link.motion = mg.MotionGuard(forward_pwm_sign=rs.FORWARD_PWM_SIGN)
    # commands reach the guard, and a stop resets it
    link.send_drive(-60, -60)
    assert link.motion.cmd == (-60, -60)
    link.send_stop()
    assert link.motion.cmd == (0, 0)
    # a stall latched in the motion guard blocks the next drive with a "motion guard" message,
    # and what goes on the wire is "stop"
    link.motion.on_command(-70, -70, 0.0)
    for i in range(1, 120):
        link.motion.on_enc(0.0, 0.0, i * 0.02)
    assert link.motion.tripped and link.motion.fault["kind"] == "stall", link.motion.fault
    n = len(link.ser.lines)
    try:
        link.send_drive(-60, -60)
        assert False, "must refuse"
    except rs.GuardFault as e:
        assert "motion guard tripped" in str(e) and "stall" in str(e), str(e)
        assert set(e.info) == {"tilt", "motion"}
    assert link.ser.lines[n:] == ["stop"]
    link.send_drive(0, 0)  # zero always goes out
    ok, _ = link.motion.clear()
    assert ok
    link.send_drive(-60, -60)  # and drives again after the operator clears it


def test_server_registers_the_guard_tools():
    import asyncio
    import robot_server as rs
    names = {t.name for t in asyncio.run(rs.mcp.list_tools())}
    assert {"clear_tilt_fault", "clear_motion_fault", "set_level_reference"} <= names, names


# ---- replay recorded runs -----------------------------------------------------------

def replay_recordings():
    root = Path(__file__).resolve().parent.parent / "recordings"
    worst = []
    for path in sorted(glob.glob(str(root / "*" / "*.csv"))):
        try:
            with open(path, newline="") as f:
                rows = [r for r in csv.DictReader(f)]
        except Exception:
            continue
        if not rows or "pitch_deg" not in rows[0] or "source" not in rows[0]:
            continue
        g = tg.TiltGuard()
        tripped = None
        n = 0
        mx = 0.0
        for r in rows:
            if r["source"] != "IMU":
                continue
            try:
                ms = int(float(r["time_s"]) * 1000)
                f = g.update(float(r["pitch_deg"]), float(r["roll_deg"]), ms, gyro_dps=0.0)
            except ValueError:
                continue
            n += 1
            mx = max(mx, g.tilt_deg)
            if f and tripped is None:
                tripped = (f, float(r["time_s"]))
        if n:
            worst.append((mx, path, n, tripped, g))
    return worst


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in tests:
        try:
            fn()
            print("PASS", fn.__name__)
        except Exception as e:
            failed += 1
            import traceback
            print("FAIL", fn.__name__, "-", repr(e))
            traceback.print_exc()
    res = replay_recordings()
    # recordings of deliberate tips by hand MUST trip (real-data regression); everything else
    # (ordinary driving) must not.
    expected = [r for r in res if "tilt_test" in r[1]]
    normal = [r for r in res if "tilt_test" not in r[1]]
    print("\nreplay of %d recorded runs (limit %.0f deg):" % (len(res), tg.DEFAULT_LIMIT_DEG))
    bad = 0
    for _, path, n, tripped, g in normal:
        if tripped:
            bad += 1
            print("  UNEXPECTED TRIP", os.path.relpath(path), tripped[0]["reason"], "at %.1fs" % tripped[1])
    print("  %d/%d ordinary runs tripped the guard" % (bad, len(normal)))
    for mx, path, n, tripped, g in expected:
        ok = tripped is not None
        bad += 0 if ok else 1
        print("  %s deliberate tip %s: %s" % ("OK  " if ok else "MISS", os.path.relpath(path),
              tripped[0]["reason"] + " at %.1fs" % tripped[1] if ok else "never tripped (peak %.1f deg)" % mx))
    for mx, path, n, _, _ in sorted(normal, reverse=True)[:3]:
        print("  worst ordinary tilt %5.1f deg over %5d IMU samples  %s" % (mx, n, os.path.relpath(path)))
    sys.exit(1 if failed or bad else 0)


if __name__ == "__main__":
    main()
