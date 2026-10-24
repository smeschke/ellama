#!/usr/bin/env python3
"""Tests for phone_status and torch (no hardware, no phone): the phone's HTTP endpoint is faked.

    ELLAMA_SERIAL_PORT=/dev/none .venv/bin/python test_phone_status.py
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import robot_server as rs  # noqa: E402


class FakePhone:
    """Answers /info.json and /?torch=on|off like the camera app; records every path asked."""
    def __init__(self, battery=60, torch="off", follow_torch=True):
        self.battery, self.torch, self.follow_torch = battery, torch, follow_torch
        self.paths = []

    def __call__(self, path, timeout=6.0):
        self.paths.append(path)
        if path == "/info.json":
            return json.dumps({"batteryPercent": self.battery, "settings": {"torch": self.torch}}).encode()
        if path.startswith("/?torch="):
            if self.follow_torch:
                self.torch = path.split("=")[1]
            return b"ok"
        raise AssertionError(path)


class Patched:
    def __init__(self, fake):
        self.fake = fake

    def __enter__(self):
        self.saved = rs._phone_get
        rs._phone_get = self.fake
        return self.fake

    def __exit__(self, *exc):
        rs._phone_get = self.saved


def test_status_reports_battery_and_torch():
    with Patched(FakePhone(battery=60, torch="off")):
        r = rs.phone_status()
    assert r == {"ok": True, "battery_percent": 60, "torch_on": False, "low_battery": False}, r


def test_low_battery_flag():
    with Patched(FakePhone(battery=20, torch="on")):
        r = rs.phone_status()
    assert r["low_battery"] is True and r["torch_on"] is True, r
    with Patched(FakePhone(battery=21)):
        assert rs.phone_status()["low_battery"] is False


def test_torch_sets_then_reads_back():
    with Patched(FakePhone(torch="off")) as ph:
        r = rs.torch(True)
        assert r["ok"] and r["torch_on"] is True and r["battery_percent"] == 60, r
        assert ph.paths == ["/?torch=on", "/info.json"], ph.paths
        r = rs.torch(False)
        assert r["ok"] and r["torch_on"] is False, r


def test_torch_reports_when_the_phone_did_not_follow():
    with Patched(FakePhone(torch="off", follow_torch=False)):
        r = rs.torch(True)
    assert r["ok"] is False and "reports it off" in r["error"], r


def test_unknown_torch_state_is_null_not_an_error():
    def no_settings(path, timeout=6.0):
        return json.dumps({"batteryPercent": 55}).encode() if path == "/info.json" else b"ok"
    with Patched(no_settings):
        r = rs.torch(True)
    assert r["ok"] and r["torch_on"] is None and r["battery_percent"] == 55, r


def test_unreachable_phone_is_an_error():
    def boom(path, timeout=6.0):
        raise RuntimeError("phone camera app did not answer")
    with Patched(boom):
        assert rs.phone_status()["ok"] is False
        assert rs.torch(True)["ok"] is False


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
