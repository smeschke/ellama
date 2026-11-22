#!/usr/bin/env python3
"""Focused tests for dashboard hold-to-drive (mcp_server/daemon_http.py).

No hardware needed:

    ELLAMA_SERIAL_PORT=/dev/none python3 mcp_server/test_dashboard_drive.py
"""

import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import daemon_http as dh  # noqa: E402
from drive_lease import DriveLease  # noqa: E402
from operator_stop import OperatorStop  # noqa: E402
from robot_link import GuardFault  # noqa: E402


class FakeLink:
    def __init__(self, refuse=None):
        self.drives = []
        self.stops = 0
        self.refuse = refuse

    def send_drive(self, left, right):
        if self.refuse is not None:
            raise self.refuse
        self.drives.append((left, right))

    def send_stop(self):
        self.stops += 1


def ctx(link=None):
    return types.SimpleNamespace(
        link=FakeLink() if link is None else link,
        LEASE=DriveLease(),
        OPSTOP=OperatorStop(),
    )


def reset():
    dh._dash_reset_for_tests()


# ---- vector mapping ------------------------------------------------------------

def test_vectors_match_turn_mapping():
    assert dh.dash_vector("forward", 70) == (70, 70)
    assert dh.dash_vector("back", 70) == (-70, -70)
    assert dh.dash_vector("spin_left", 70) == (-70, 70)
    assert dh.dash_vector("spin_right", 70) == (70, -70)


def test_unknown_button_is_rejected():
    for bad in ("forward-left", "", None, 3):
        code, out = dh.dash_hold(ctx(), "t", bad, 70, now=100.0)
        assert code == 400 and out["ok"] is False, (bad, code, out)


def test_pwm_clamps_to_dashboard_range():
    c = ctx()
    code, out = dh.dash_hold(c, "t", "forward", 500, now=100.0)
    assert code == 200 and out["pwm"] == 125, out
    assert c.link.drives[-1] == (125, 125)
    code, out = dh.dash_hold(c, "t", "forward", 1, now=100.1)
    assert code == 200 and out["pwm"] == 20, out


# ---- hold / stop -----------------------------------------------------------------

def test_hold_drives_and_stop_brakes_and_frees_lease():
    reset()
    c = ctx()
    code, out = dh.dash_hold(c, "t", "forward", 70, now=100.0)
    assert code == 200 and out["ok"] is True, out
    assert c.link.drives == [(70, 70)]
    assert c.LEASE.snapshot()["holder"] == "dashboard"
    code, out = dh.dash_stop(c, "t", now=100.5)
    assert code == 200 and out["was_driving"] is True
    assert c.link.stops == 1
    assert c.LEASE.snapshot()["held"] is False


def test_hold_refused_when_operator_stop_latched():
    reset()
    c = ctx()
    c.OPSTOP.latch("tester", "testing")
    try:
        code, out = dh.dash_hold(c, "t", "forward", 70, now=100.0)
        assert code == 409 and out["ok"] is False, out
        assert c.link.drives == []
    finally:
        c.OPSTOP.clear("tester")


def test_hold_refused_when_another_client_holds_lease():
    reset()
    c = ctx()
    ok, _ = c.LEASE.take("claude", "human")
    assert ok
    code, out = dh.dash_hold(c, "t", "forward", 70, now=100.0)
    assert code == 409 and out["ok"] is False, out
    assert c.link.drives == []


def test_guard_trip_refuses_and_brakes():
    reset()
    c = ctx(FakeLink(refuse=GuardFault("tilt guard tripped")))
    code, out = dh.dash_hold(c, "t", "forward", 70, now=100.0)
    assert code == 409 and "tilt" in out["error"], out
    assert c.link.stops == 1  # braked even though the drive was refused


def test_watchdog_brakes_stale_hold():
    reset()
    c = ctx()
    dh.dash_hold(c, "t", "forward", 70, now=100.0)
    assert dh.dash_watchdog(c, now=100.5) is False   # holds still arriving
    assert dh.dash_watchdog(c, now=100.0 + 5.0) is True
    assert c.link.stops == 1
    assert c.LEASE.snapshot()["held"] is False      # turn freed


def test_stop_is_safe_when_idle():
    reset()
    c = ctx()
    code, out = dh.dash_stop(c, "t", now=100.0)
    assert code == 200 and out["was_driving"] is False


def test_frame_conventions_reports_live_settings():
    import types
    assert dh.frame_conventions(None) == {
        "drive_swap_sides": True, "imu_yaw_sign": -1,
        "encoder_sign": [1, 1], "gear_ratio_used": 4.5}
    stub = types.SimpleNamespace(cfg={"encoder_sign": [1, -1], "gear_ratio": 4.5})
    assert dh.frame_conventions(stub)["encoder_sign"] == [1, -1]


_FUNCS = [v for k, v in sorted(globals().items())
          if k.startswith("test_") and callable(v)]


def main():
    failed = 0
    for fn in _FUNCS:
        reset()
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
