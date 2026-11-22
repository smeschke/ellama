#!/usr/bin/env python3
"""Focused tests for v2 bridge framing: "@ <seq> <NAME> ... *CC".

Run: ELLAMA_SERIAL_PORT=/dev/none python3 mcp_server/test_bridge_v2.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import bridge


def frame(seq, name, *fields):
    payload = " ".join([str(seq), name] + [str(f) for f in fields])
    cs = bridge._v2_checksum(payload)
    return ("@ %s *%02X" % (payload, cs)).encode()


def test_v2_tof_parses():
    raw = frame(123, "TOF_FRONT", 812, -1, 48213)
    kind, payload = bridge.parse_line(raw)
    assert kind == "SENSOR", (raw, kind, payload)
    assert payload == ("TOF_FRONT", (812, -1), 48213), payload


def test_v2_enc_parses():
    raw = frame(7, "ENC", 100, 200, 5000)
    kind, payload = bridge.parse_line(raw)
    assert kind == "ENC", (raw, kind, payload)
    # encoder_sign from calibration/config.json ([-1, -1]: forward counts up)
    # is applied at parse time, so the wire counts come back negated.
    assert payload == (-100, -200, 5000), payload


def test_v2_bad_checksum_drops():
    raw = frame(8, "TOF_FRONT", 812, -1, 48213)
    bad = raw[:-2] + (b"00" if not raw.endswith(b"00") else b"FF")
    kind, _ = bridge.parse_line(bad)
    assert kind is None, bad


def test_v2_torn_line_drops():
    # "MU" from a split "IMU" line must not parse, even with framing.
    kind, _ = bridge.parse_line(b"@ 9 MU 150 -128 *00")
    assert kind is None


def test_v1_legacy_still_parses():
    kind, payload = bridge.parse_line(b"TOF_FRONT 812 -1 48213")
    assert kind == "SENSOR", (kind, payload)


def test_v2_status_is_chatter():
    kind, _ = bridge.parse_line(b"@ status bridge-v2 *00")
    assert kind is None


def _v2_dropped(n):
    payload = "9 DROPPED %d" % n
    return ("@ %s *%02X" % (payload, bridge._v2_checksum(payload))).encode()


def test_v2_dropped_reaches_pump_shape():
    kind, payload = bridge.parse_line(_v2_dropped(3))
    assert kind is None and payload == "DROPPED 3", (kind, payload)


def test_yaw_follows_imu_yaw_sign():
    # Positive gz must move yaw in the direction IMU_YAW_SIGN says is
    # nose-left (ground-verified operator convention, see bridge config).
    ori = bridge.OrientationState()
    lsb = bridge.GYRO_LSB_PER_DPS * 90.0  # +90 dps for 1 s
    ori.update(0, 0, 16384, 0, 0, int(lsb), 1000)
    ori.update(0, 0, 16384, 0, 0, int(lsb), 2000)
    assert ori.yaw == bridge.IMU_YAW_SIGN * 90.0, (ori.yaw, bridge.IMU_YAW_SIGN)


_FUNCS = [v for k, v in sorted(globals().items())
          if k.startswith("test_") and callable(v)]


def main():
    failed = 0
    for fn in _FUNCS:
        try:
            fn()
            print("ok %s" % fn.__name__)
        except AssertionError as e:
            failed += 1
            print("FAIL %s: %r" % (fn.__name__, e))
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
