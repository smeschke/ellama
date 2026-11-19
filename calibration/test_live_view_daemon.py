#!/usr/bin/env python3
"""Tests for calibration/live_view.py --daemon mode (dashboard via the HTTP daemon).

No hardware, no window needed:

    mcp_server/.venv/bin/python calibration/test_live_view_daemon.py

The bug: live_view.py opened the serial port directly with exclusive=True, so
once the robot daemon (mcp_server/robot_server.py) owns /dev/ttyUSB0 it dies
with BlockingIOError [Errno 11]. --daemon mode polls the daemon's GET /state
instead and renders the same dashboard from that snapshot.
"""

import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import live_view as lv  # noqa: E402


def canned_state(**over):
    tel = {
        "left_ticks": 1000, "right_ticks": 1010,
        "encoders_available": [True, True],
        "dist_l_in": 5.0, "dist_r_in": 5.2, "dist_avg_in": 5.1,
        "heading_deg": 1.5,
        "imu_last": {"ax": 0, "ay": 0, "az": 16384, "gx": 0, "gy": 0, "gz": 0},
        "pitch_deg": -0.8, "roll_deg": 0.4, "yaw_deg": 33.0,
        "tilt": {"tripped": False, "tilt_deg": 1.0, "limit_deg": 25.0},
        "telemetry_age_s": 0.01, "telemetry_live": True,
        "imu_age_s": 0.01, "imu_live": True,
        "range": {
            "channels": {
                "front_left": {"sensor": "TOF_FRONT", "channel": 0, "mm": 150,
                               "inches": 5.9, "latest_mm": 152,
                               "in_range": True, "latest_in_range": True, "live": True},
                "front_right": {"sensor": "TOF_FRONT", "channel": 1, "mm": 42,
                                "inches": 1.7, "latest_mm": 40,
                                "in_range": True, "latest_in_range": True, "live": True},
            },
            "closest_mm": 42, "closest_in": 1.7, "live": True,
            "note": "closest obstacle 42 mm (1.7 in)",
        },
        "sensors": {},
        "bridge": {"dropped_frames": 0, "foreign_frames": 0},
        "required_sensors": ["enc", "imu"],
        "missing_sensors": [],
    }
    tel.update(over.get("telemetry", {}))
    state = {
        "connected": True,
        "connect_error": None,
        "operator_stop": {"latched": False, "who": None, "reason": None},
        "drive_lease": {"held": False, "holder": None, "move_in_progress": False},
        "telemetry": tel,
    }
    state.update({k: v for k, v in over.items() if k != "telemetry"})
    return state


def reset_display():
    lv.enc.__init__(lv.cfg)
    lv.ori.__init__()
    lv.tof.__init__()
    lv.tof_hist.clear()
    for h in lv.hist.values():
        h.clear()
    for s in lv.streams.values():
        s["stamps"].clear()
        s["last_rx"] = 0.0
        s["count"] = 0
    lv.accel_g = (0.0, 0.0, 0.0)
    lv.cmd["left"] = lv.cmd["right"] = 0
    lv.daemon_zero_offsets()


def test_apply_daemon_state_encoders():
    reset_display()
    lv.apply_daemon_state(canned_state())
    assert lv.enc.dist_l == 5.0 and lv.enc.dist_r == 5.2, (lv.enc.dist_l, lv.enc.dist_r)
    assert lv.enc.available == [True, True]
    assert lv.enc.heading_deg is not None
    assert lv.hist["dist_l"] and lv.hist["dist_r"]


def test_apply_daemon_state_one_wheel_out_holds_heading_none():
    reset_display()
    tel = {"encoders_available": [True, False], "dist_l_in": 5.0, "dist_r_in": 5.2}
    lv.apply_daemon_state(canned_state(telemetry=tel))
    assert lv.enc.available == [True, False]
    assert lv.enc.heading_deg is None  # never a number off one live wheel


def test_apply_daemon_state_imu_and_accel():
    reset_display()
    tel = {"pitch_deg": -0.8, "roll_deg": 0.4, "yaw_deg": 33.0,
           "imu_last": {"ax": 8192, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0}}
    lv.apply_daemon_state(canned_state(telemetry=tel))
    assert (lv.ori.pitch, lv.ori.roll, lv.ori.yaw) == (-0.8, 0.4, 33.0)
    assert lv.accel_g == (0.5, 0.0, 0.0), lv.accel_g  # raw / 16384 -> g
    assert lv.hist["yaw"]


def test_apply_daemon_state_range_by_label():
    reset_display()
    lv.apply_daemon_state(canned_state())
    by_label = {e["label"]: e for e in lv.daemon_range}
    assert by_label["front_right"]["mm"] == 42, by_label
    assert by_label["front_left"]["mm"] == 150, by_label
    assert lv.daemon_closest == 42
    assert lv.hist  # sparklines keep feeding


def test_apply_daemon_state_range_nothing_in_range_is_not_dead():
    reset_display()
    ch = {"front_left": {"sensor": "TOF_FRONT", "channel": 0, "mm": None,
                         "inches": None, "latest_mm": None,
                         "in_range": False, "latest_in_range": False, "live": True}}
    rng = {"channels": ch, "closest_mm": None, "closest_in": None, "live": True,
           "note": "nothing within range of any sensor"}
    lv.apply_daemon_state(canned_state(telemetry={"range": rng}))
    assert lv.daemon_range[0]["mm"] is None
    assert lv.daemon_closest is None
    assert lv.is_live("RANGE")  # reporting, just nothing in range


def test_apply_daemon_state_dead_streams_are_not_marked_live():
    reset_display()
    n_enc, n_imu, n_rng = (lv.streams[s]["count"] for s in ("ENC", "IMU", "RANGE"))
    tel = {"telemetry_live": False, "imu_live": False,
           "range": {"channels": {}, "closest_mm": None, "closest_in": None,
                     "live": False, "note": "blind"}}
    lv.apply_daemon_state(canned_state(telemetry=tel))
    assert lv.streams["ENC"]["count"] == n_enc
    assert lv.streams["IMU"]["count"] == n_imu
    assert lv.streams["RANGE"]["count"] == n_rng


def test_apply_daemon_state_drive_info_for_panel():
    reset_display()
    lv.apply_daemon_state(canned_state())
    assert lv.daemon_info["connected"] is True
    assert lv.daemon_info["stop_latched"] is False
    lv.apply_daemon_state(canned_state(
        operator_stop={"latched": True, "who": "op", "reason": "x"},
        drive_lease={"held": True, "holder": "claude", "move_in_progress": True}))
    assert lv.daemon_info["stop_latched"] is True
    assert lv.daemon_info["lease_holder"] == "claude"
    assert lv.daemon_info["move_in_progress"] is True


def test_daemon_zero_offsets_display():
    reset_display()
    lv.apply_daemon_state(canned_state())
    lv.daemon_zero()  # daemon owns true odometry; this only re-zeros the display
    assert lv.enc.dist_l == 0.0 and lv.enc.dist_r == 0.0
    assert lv.ori.yaw == 0.0
    lv.apply_daemon_state(canned_state(
        telemetry={"dist_l_in": 6.0, "dist_r_in": 6.2, "yaw_deg": 40.0}))
    assert lv.enc.dist_l == 1.0 and lv.enc.dist_r == 1.0, (lv.enc.dist_l, lv.enc.dist_r)
    assert lv.ori.yaw == 7.0


def _stub_server(payload, status=200):
    body = json.dumps(payload).encode()

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            assert self.path == "/state", self.path
            self.send_response(status)
            self.send_header("content-type", "application/json")
            self.send_header("content-length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def test_fetch_daemon_state_round_trip():
    payload = canned_state()
    srv = _stub_server(payload)
    try:
        got = lv.fetch_daemon_state("http://127.0.0.1:%d" % srv.server_port)
    finally:
        srv.shutdown()
    assert got["connected"] is True
    assert got["telemetry"]["dist_avg_in"] == 5.1


def test_draw_frame_daemon_mode_renders():
    import numpy as np
    reset_display()
    lv.DAEMON_MODE = True
    try:
        lv.apply_daemon_state(canned_state())
        img = lv.draw_frame("sub", "keys", [])
        assert isinstance(img, np.ndarray) and img.shape == (lv.H, lv.W, 3), img.shape
        img = lv.draw_frame("sub", "keys", ["boom"])
        assert img.shape == (lv.H, lv.W, 3)
    finally:
        lv.DAEMON_MODE = False


def test_fetch_daemon_state_connection_refused_is_helpful():
    try:
        lv.fetch_daemon_state("http://127.0.0.1:1")
    except RuntimeError as e:
        assert "daemon" in str(e).lower(), e
    else:
        raise AssertionError("expected RuntimeError")


# ---- runner (same shape as mcp_server/test_*.py) --------------------------------------

def main():
    import traceback
    failures = 0
    for name, fn in sorted(globals().items()):
        if not (name.startswith("test_") and callable(fn)):
            continue
        try:
            fn()
            print("PASS", name)
        except Exception as e:
            failures += 1
            print("FAIL", name, "-", repr(e))
            traceback.print_exc()
    print("\n%s" % ("all passed" if not failures else "%d FAILED" % failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
