#!/usr/bin/env python3
"""Tests for motion_guard.py -- synthetic scenarios.

    python3 mcp_server/test_motion_guard.py
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import motion_guard as mg  # noqa: E402

FWD = -1  # FORWARD_PWM_SIGN: raw PWM sign that drives the robot forward


def guard(**kw):
    kw.setdefault("forward_pwm_sign", FWD)
    kw.setdefault("track_width_in", 20.0)
    return mg.MotionGuard(**kw)


def run(g, cmd, seconds, speeds, gz=0.0, acc=(0.0, 0.0), t0=0.0, hz=50, send_cmd=True):
    """Feed `seconds` of samples. speeds/gz/acc may be constants or functions of elapsed s."""
    fault = None
    n = int(seconds * hz)
    f = lambda v, e: v(e) if callable(v) else v
    for i in range(n):
        t = t0 + i / hz
        e = i / hz
        if send_cmd and i % 5 == 0:
            g.on_command(cmd[0], cmd[1], t)
        sl, sr = f(speeds, e)
        fault = fault or g.on_imu(f(gz, e), *f(acc, e), t)
        fault = fault or g.on_enc(sl, sr, t)
        if fault:
            return fault, t
    return None, t0 + seconds


def ramp(v, tau=0.25):
    return lambda e: (v * (1 - math.exp(-e / tau)),) * 2


def test_normal_straight_drive_is_quiet():
    g = guard()
    f, _ = run(g, (-60, -60), 6.0, ramp(6.0))
    assert f is None and not g.tripped, f


def test_one_wheel_snagged_is_a_stall():
    g = guard()
    # right wheel drives 6 in/s, left is held; chassis pivots about the held wheel
    right = 6.0
    gz = math.degrees(right / 20.0)
    f, t = run(g, (-60, -60), 4.0, lambda e: (0.0, right) if e > 0.5 else (0.0, 0.0), gz=gz)
    assert f and f["kind"] == "stall" and f["wheel"] == "left", f
    assert t < 2.2, t  # grace 1.0 + hold 0.4 after the command began


def test_dead_encoder_with_straight_driving_is_encoder_fault_not_stall():
    g = guard()
    f, _ = run(g, (-60, -60), 4.0, lambda e: (0.0, 6.0), gz=0.0)  # robot going straight
    assert f and f["kind"] == "encoder_fault" and f["wheel"] == "left", f


def test_encoder_fault_only_logged_when_encoders_not_required():
    g = guard()
    g.encoders_required = False  # camera-only run: encoders not required
    f, _ = run(g, (-60, -60), 4.0, lambda e: (0.0, 6.0), gz=0.0)
    assert f is None and not g.tripped
    assert any(ev["kind"] == "encoder_fault" for ev in g.events)
    # ...but a real snag still stops it
    g2 = guard()
    g2.encoders_required = False
    f, _ = run(g2, (-60, -60), 4.0, lambda e: (0.0, 6.0), gz=math.degrees(6.0 / 20.0))
    assert f and f["kind"] == "stall"


def test_spin_with_one_dead_encoder_is_encoder_fault():
    g = guard()
    # spinning in place; right encoder reads 0 but the wheel turns (the 10-03 runs): left reads
    # 4.5 in/s, the real rotation is ~20 deg/s -- more than the left wheel alone implies (13)
    f, _ = run(g, (-70, 70), 4.0, (4.5, 0.0), gz=-20.0)
    assert f and f["kind"] == "encoder_fault" and f["wheel"] == "right", f


def test_spin_with_a_held_wheel_is_reported_as_ambiguous_not_guessed():
    g = guard()
    g.encoders_required = False
    # right wheel truly held, left pivots the robot at 6.7 in/s -> gyro -19: same as dead-encoder+scrub
    f, _ = run(g, (-80, 80), 4.0, (6.7, 0.0), gz=-19.0)
    assert f is None
    ev = [e for e in g.events if e["kind"] == "encoder_fault"]
    assert ev and ev[0].get("ambiguous"), list(g.events)


def test_dead_encoder_does_not_fake_a_slip():
    g = guard()
    g.mode = "warn"
    f, _ = run(g, (-80, -40), 5.0, (6.0, 0.0), gz=0.0)  # curving drive, right encoder dead
    assert not any(e["kind"] == "slip" for e in g.events), list(g.events)


def test_both_wheels_stuck():
    g = guard()
    f, t = run(g, (-70, -70), 4.0, (0.0, 0.0))
    assert f and f["kind"] == "stall", f


def test_below_deadband_command_is_not_a_stall():
    g = guard()
    f, _ = run(g, (-35, -35), 5.0, (0.0, 0.0))
    assert f is None


def test_turn_in_place_normal_scrub_is_quiet():
    g = guard()
    # spin: wheels +-4.5 in/s -> encoder rate 27 deg/s; gyro sees 16 (1.7x scrub, as measured)
    f, _ = run(g, (-70, 70), 5.0, (4.5, -4.5), gz=-16.0)
    assert f is None, f


def test_foot_held_spin_is_slip():
    g = guard()
    # the real incident: 3x the wheel travel, gyro saw a fifth of it
    f, t = run(g, (-70, 70), 6.0, (4.5, -4.5), gz=-5.0)
    assert f and f["kind"] == "slip", f
    assert abs(f["enc_deg"]) > 20 and abs(f["gyro_deg"]) < abs(f["enc_deg"]) / 3.5


def test_direction_mismatch():
    g = guard()
    # right wheel runs the wrong way; the chassis really does rotate (gyro agrees with the
    # encoders) so only the direction check is left to fire
    f, _ = run(g, (-60, -60), 4.0, (6.0, -4.0), gz=math.degrees((-4.0 - 6.0) / 20.0))
    assert f and f["kind"] == "encoder_fault" and f["wheel"] == "right", f


def test_hard_bump():
    g = guard()
    g.on_command(-60, -60, 0.0)
    assert g.on_imu(0.0, 0.1, 0.0, 0.0) is None
    f = g.on_imu(0.0, 0.1 + 0.9, 0.0, 0.02)
    assert f and f["kind"] == "bump" and f["peak_g"] >= 0.8, f


def test_corroborated_bump_needs_both_spike_and_collapse():
    # spike + speed collapse with command held -> bump
    g = guard()
    f, _ = run(g, (-60, -60), 3.0, lambda e: (6.5, 6.5) if e < 1.5 else (0.5, 0.5),
               acc=lambda e: (0.45, 0.0) if 1.50 <= e < 1.54 else (0.0, 0.0))
    assert f and f["kind"] == "bump", f
    # same speed collapse with NO accel spike (a slow-moving run does this) -> nothing... until
    # the stall detector's own rules (grace/hold) would apply, so keep it short
    g = guard()
    f, _ = run(g, (-60, -60), 1.8, lambda e: (6.5, 6.5) if e < 1.2 else (0.5, 0.5))
    assert f is None, f
    # spike alone (vibration over a seam) with speed unchanged -> nothing
    g = guard()
    f, _ = run(g, (-60, -60), 3.0, (6.5, 6.5),
               acc=lambda e: (0.45, 0.0) if 1.5 <= e < 1.54 else (0.0, 0.0))
    assert f is None, f


def test_warn_mode_logs_but_never_trips():
    g = guard(mode="warn")
    f, _ = run(g, (-70, -70), 4.0, (0.0, 0.0))
    assert f is None and not g.tripped and any(e["kind"] == "stall" for e in g.events)


def test_latch_and_clear():
    g = guard()
    f, _ = run(g, (-70, -70), 4.0, (0.0, 0.0))
    assert g.tripped
    ok, _ = g.clear()
    assert ok and not g.tripped and g.fault is None


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
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
