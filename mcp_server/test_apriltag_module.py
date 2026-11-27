#!/usr/bin/env python3
"""Hardware-free tests for apriltag_module.py: a fake MJPEG camera on localhost.

    ELLAMA_SERIAL_PORT=/dev/none mcp_server/.venv/bin/python mcp_server/test_apriltag_module.py
"""

import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import apriltag_module as am  # noqa: E402


def frame_with_tags():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    img = np.full((480, 640), 255, np.uint8)
    img[100:200, 100:200] = cv2.aruco.generateImageMarker(d, 3, 100)
    img[250:400, 400:550] = cv2.aruco.generateImageMarker(d, 12, 150)
    return cv2.imencode(".jpg", img)[1].tobytes()


JPG = frame_with_tags()


class FakeCamera(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/video/mjpeg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
        self.end_headers()
        try:
            while True:
                self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + JPG + b"\r\n")
                time.sleep(0.1)
        except OSError:
            pass

    def log_message(self, *a):
        pass


def test_normalize_address():
    assert am.normalize_address("192 168 0 52 4444") == "http://192.168.0.52:4444"
    assert am.normalize_address("192.168.0.52:4444") == "http://192.168.0.52:4444"
    assert am.normalize_address("192 168 0 52") == "http://192.168.0.52:8080"
    assert am.normalize_address("https://cam.local/video") == "https://cam.local/video"
    assert am.normalize_address("") == ""


def test_sees_tags_from_a_stream():
    srv = ThreadingHTTPServer(("127.0.0.1", 0), FakeCamera)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        st = am.apriltags_in_view("127.0.0.1:%d" % srv.server_address[1])
        assert sorted(t["id"] for t in st["tags"]) == [3, 12], st
        centres = {t["id"]: t["center_px"] for t in st["tags"]}
        assert abs(centres[3][0] - 150) <= 2 and abs(centres[12][1] - 325) <= 2, st
        assert am.VIEW.result["jpg"][:3] == b"\xff\xd8\xff"   # annotated frame for the page
    finally:
        srv.shutdown()


def test_detects_continuously_then_hangs_up_when_idle():
    srv = ThreadingHTTPServer(("127.0.0.1", 0), FakeCamera)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    old_idle, am.IDLE_S = am.IDLE_S, 0.6
    try:
        v = am.TagView()
        v.use("127.0.0.1:%d" % srv.server_address[1])
        first = v.wait_new(0, 8.0)
        assert first is not None and sorted(t[0] for t in first["tags"]) == [3, 12]
        second = v.wait_new(first["seq"], 2.0)          # keeps detecting new frames
        assert second is not None and second["seq"] > first["seq"]
        assert v.status()["detecting"]
        time.sleep(1.5)                                  # nobody touched it
        st = v.status()
        assert not st["detecting"] and st["stream_url"] == "", st
        v.touch()                                        # a viewer comes back
        assert v.wait_new(v.seq, 8.0) is not None
    finally:
        am.IDLE_S = old_idle
        srv.shutdown()


def test_no_camera_says_so():
    v = am.TagView()
    v.touch()
    assert v.status()["note"] == "enter the camera address" and not v.status()["detecting"]


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok   ", name)
            except Exception as e:
                fails += 1
                print("FAIL ", name, repr(e))
    sys.exit(1 if fails else 0)
