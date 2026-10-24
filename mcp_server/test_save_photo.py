#!/usr/bin/env python3
"""Tests for save_photo (no hardware, no phone): the camera fetch and the robot link are faked.

    ELLAMA_SERIAL_PORT=/dev/none .venv/bin/python test_save_photo.py
"""
import json
import sys
import tempfile
import threading
from io import BytesIO
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import robot_server as rs  # noqa: E402
from PIL import Image as PILImage  # noqa: E402


def jpeg(color=(120, 120, 120), size=(320, 240)):
    buf = BytesIO()
    PILImage.new("RGB", size, color).save(buf, "JPEG")
    return buf.getvalue()


class _Obj:
    pass


class FakeLink:
    """Just what save_photo reads: lock, enc.dist_l/dist_r, orient.yaw, telemetry_snapshot()."""
    def __init__(self):
        self.lock = threading.Lock()
        self.enc, self.orient = _Obj(), _Obj()
        self.enc.dist_l = self.enc.dist_r = 0.0
        self.orient.yaw = 0.0

    def telemetry_snapshot(self):
        return dict(dist_avg_in=round((self.enc.dist_l + self.enc.dist_r) / 2, 3), heading_deg=0.4,
                    yaw_deg=self.orient.yaw, pitch_deg=2.0, roll_deg=-1.0,
                    telemetry_live=True, imu_live=True, other="not copied")


class Patched:
    """Point PHOTO_DIR at a temp dir, fake the fetch and the link; restore on exit."""
    def __init__(self, fetch, link=None):
        self.fetch, self.link = fetch, link

    def __enter__(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.saved = (rs.PHOTO_DIR, rs._fetch_snapshot, rs.link)
        rs.PHOTO_DIR, rs._fetch_snapshot, rs.link = Path(self.tmp.name) / "photos", self.fetch, self.link
        return Path(self.tmp.name) / "photos"

    def __exit__(self, *exc):
        rs.PHOTO_DIR, rs._fetch_snapshot, rs.link = self.saved
        self.tmp.cleanup()


def test_saves_file_sidecar_and_gitignore_without_a_robot():
    with Patched(lambda: jpeg()) as d:
        r = rs.save_photo(name="front door!")
        assert r["ok"] and r["robot"] is None and r["moved_during_capture"] is None, r
        p = Path(r["path"])
        assert p.parent == d and p.suffix == ".jpg" and p.name.endswith("_front_door.jpg"), p.name
        assert (r["width"], r["height"]) == (320, 240) and r["bytes"] == p.stat().st_size
        meta = json.loads(p.with_suffix(".json").read_text())
        assert meta["file"] == p.name and meta["name"] == "front door!" and meta["brightness"] == 120
        assert (d / ".gitignore").read_text().startswith("*")


def test_two_photos_in_one_second_do_not_overwrite():
    with Patched(lambda: jpeg()):
        a, b = rs.save_photo()["path"], rs.save_photo()["path"]
        assert a != b and Path(a).exists() and Path(b).exists()


def test_brightness_reports_a_dark_scene():
    with Patched(lambda: jpeg((8, 8, 8))):
        assert rs.save_photo()["brightness"] < 20


def test_records_pose_and_detects_motion_during_the_fetch():
    link = FakeLink()
    link.enc.dist_l = link.enc.dist_r = 10.0

    def still():
        return jpeg()

    def moving():
        link.enc.dist_l += 1.0
        link.enc.dist_r += 1.0
        return jpeg()

    with Patched(still, link):
        r = rs.save_photo()
        assert r["moved_during_capture"] is False, r
        assert r["robot"]["dist_avg_in"] == 10.0 and "other" not in r["robot"]
    with Patched(moving, link):
        assert rs.save_photo()["moved_during_capture"] is True
    link.orient.yaw = 0.0

    def turning():
        link.orient.yaw += 5.0
        return jpeg()

    with Patched(turning, link):
        assert rs.save_photo()["moved_during_capture"] is True


def test_camera_failure_is_an_error_and_writes_nothing():
    def boom():
        raise RuntimeError("the camera app is up at X but its photo endpoint returned no image")

    with Patched(boom) as d:
        r = rs.save_photo()
        assert r["ok"] is False and "photo endpoint" in r["error"]
        assert not d.exists()


def test_preview_returns_the_picture_and_the_json():
    with Patched(lambda: jpeg(size=(1000, 500))):
        out = rs.save_photo(preview_width=400)
        assert isinstance(out, list) and len(out) == 2
        meta = json.loads(out[0])
        assert meta["ok"] and meta["width"] == 1000    # the saved file is full size
        prev = PILImage.open(BytesIO(out[1].data))
        assert prev.size == (400, 200), prev.size
        assert isinstance(rs.save_photo(), dict)       # default: no image in the reply


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
