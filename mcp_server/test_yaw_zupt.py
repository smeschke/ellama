#!/usr/bin/env python3
"""Tests for the yaw zero-point update in bridge.OrientationState (no hardware).

    ELLAMA_SERIAL_PORT=/dev/none mcp_server/.venv/bin/python mcp_server/test_yaw_zupt.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import bridge  # noqa: E402

LSB = bridge.GYRO_LSB_PER_DPS
G = int(bridge.ACCEL_LSB_PER_G)
HZ = 50


def run(o, seconds, gz_dps, t0, wheels=None, enc=True):
    """Feed `seconds` of IMU at HZ with a constant z rate. wheels(t) -> (dl, dr) inches.
    Returns the end time."""
    n = int(seconds * HZ)
    t = t0
    for _ in range(n):
        t += 1.0 / HZ
        if enc:
            dl, dr = wheels(t) if wheels else (0.0, 0.0)
            o.note_wheels(dl, dr, now=t)
        o.update(0, 0, G, 0, 0, int(round(gz_dps * LSB)), int(t * 1000), now=t)
    return t


def test_stopped_robot_with_a_biased_gyro_holds_yaw_and_learns_the_bias():
    o = bridge.OrientationState()
    t = run(o, 20.0, 0.4, 0.0)          # 0.4 deg/s bias, wheels still the whole time
    # only the first STILL_AFTER_S integrates
    assert abs(o.yaw) < 0.4 * (bridge.STILL_AFTER_S + 0.1), o.yaw
    assert abs(o.gz_bias_dps - 0.4) < 0.02, o.gz_bias_dps
    assert o.still
    # after learning, a real turn with the same bias reads true
    o.zero_yaw()
    t = run(o, 3.0, 30.0 + 0.4, t, wheels=lambda tt: (-(tt - t) * 3, (tt - t) * 3))
    assert abs(abs(o.yaw) - 90.0) < 0.5, o.yaw


def test_without_encoders_nothing_changes():
    o = bridge.OrientationState()
    run(o, 10.0, 0.4, 0.0, enc=False)
    assert abs(abs(o.yaw) - 4.0) < 0.1, o.yaw
    assert o.gz_bias_dps == 0.0


def test_stale_encoders_stop_holding_yaw():
    o = bridge.OrientationState()
    t = run(o, 5.0, 0.0, 0.0)
    assert o.still
    run(o, 2.0, 0.4, t, enc=False)       # encoder packets stopped: could be moving
    assert not o.still


def test_rotation_with_still_wheels_still_integrates():
    # picked up and turned by hand, or skidding: the gyro rate is far above the gate
    o = bridge.OrientationState()
    t = run(o, 3.0, 0.0, 0.0)
    o.zero_yaw()
    run(o, 2.0, 45.0, t)
    assert abs(abs(o.yaw) - 90.0) < 1.0, o.yaw
    assert abs(o.gz_bias_dps) < 0.01


def test_encoder_jitter_does_not_count_as_motion():
    o = bridge.OrientationState()
    run(o, 12.0, 0.4, 0.0, wheels=lambda t: (0.02 * (int(t * 10) % 2), 0.0))
    assert o.still and abs(o.gz_bias_dps - 0.4) < 0.05


if __name__ == "__main__":
    n = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            n += 1
    print("PASS mcp_server/test_yaw_zupt.py (%d tests)" % n)
