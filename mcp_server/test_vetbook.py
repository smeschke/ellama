#!/usr/bin/env python3
"""Hardware-free tests for the lifetime logbook (vetbook) and startup config checks.

Run: mcp_server/.venv/bin/python mcp_server/test_vetbook.py   (or `make test`)
"""

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import recording  # noqa: E402


def test_check_config_accepts_repo_config():
    cfg = {"wheel_diameter_in": 10.0, "ticks_per_rev": 4096, "gear_ratio": 6.33,
           "track_width_in": 19.0, "encoder_sign": [1, 1]}
    assert recording.check_config(cfg) == [], recording.check_config(cfg)


def test_check_config_rejects_nonsense():
    base = {"wheel_diameter_in": 10.0, "ticks_per_rev": 4096, "gear_ratio": 6.33,
            "track_width_in": 19.0, "encoder_sign": [1, 1]}
    for key, bad in [("wheel_diameter_in", 200), ("ticks_per_rev", 0),
                     ("gear_ratio", -1), ("track_width_in", 200)]:
        cfg = dict(base, **{key: bad})
        assert recording.check_config(cfg), (key, bad)
    cfg = dict(base)
    del cfg["track_width_in"]
    assert recording.check_config(cfg), "a missing key must be reported"
    cfg = dict(base, encoder_sign=[1, 2])
    assert recording.check_config(cfg), "encoder_sign must be +-1 per wheel"


def test_vetlog_appends_and_reads_back():
    with tempfile.TemporaryDirectory() as tmp:
        vet = recording.VetLog(Path(tmp))
        vet.log("server_start", {"mode": "test"})
        vet.log("run_start", {"name": "test_run"})
        recent = vet.recent(5)
        assert [r["event"] for r in recent] == ["server_start", "run_start"], recent
        assert all("ts" in r for r in recent)


def test_recorder_run_writes_vetbook_entries():
    with tempfile.TemporaryDirectory() as tmp:
        rec = recording.Recorder(Path(tmp))
        rec.start("vettest")
        rec.on_config("tilt guard: test change")
        summary = rec.stop()
        assert summary["csv"].endswith("vettest.csv")
        events = [r["event"] for r in rec.vet.recent(10)]
        assert "run_start" in events and "run_stop" in events, events
        assert "config" in events, events


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
    print("\n%d/%d passed" % (len(tests) - failed, len(tests)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
