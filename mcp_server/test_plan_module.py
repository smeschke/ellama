#!/usr/bin/env python3
"""Hardware-free tests for plan_module.py (and vision_drive's toward/find) against a
simulated robot on a 2D floor with two AprilTags, a camera that runs LATENCY late, and
an optional front ToF.

    ELLAMA_SERIAL_PORT=/dev/none mcp_server/.venv/bin/python mcp_server/test_plan_module.py
"""

import csv
import json
import math
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plan_module as pm  # noqa: E402
import vision_drive as vd  # noqa: E402
from drive_lease import DriveLease  # noqa: E402
from operator_stop import OperatorStop  # noqa: E402
import recording  # noqa: E402

W, H, HFOV = 640, 480, 60.0
F_PX = (W / 2) / math.tan(math.radians(HFOV) / 2)
TAG_IN = 186.0 * 12.0 / F_PX      # a tag that fills 29% of the width at 12 in, like tag 5
IN_PER_S_PER_PWM = 0.2            # pwm 60 forward = 12 in/s
DEG_PER_S_PER_PWM_DIFF = 0.15     # spin at pwm 50 (diff 100) = 15 deg/s
FAILSAFE_S = 0.3
LATENCY = 0.3
vd.hfov_deg = lambda: (HFOV, "test")


class Sim:
    """x forward, y right (inches); heading in degrees, + = clockwise (right)."""

    def __init__(self, tags, tof=False):
        self.lock = threading.Lock()
        self.x = self.y = self.h = 0.0
        self.l = self.r = 0.0
        self.last_cmd = 0.0
        self.tags = dict(tags)            # id -> (x, y)
        self.tof = tof
        self.hist = [(time.monotonic(), 0.0, 0.0, 0.0)]
        self.drives = []                  # (left, right) from send_drive
        self.opstop = OperatorStop()
        # what RunRecording reads off the link: recorder, lock, enc, orient
        self.rec_dir = tempfile.mkdtemp(prefix="plan_rec_")
        self.recorder = recording.Recorder(self.rec_dir)
        self.enc = types.SimpleNamespace(dist_l=0.0, dist_r=0.0, dist_avg=0.0, heading_deg=None)
        self.orient = types.SimpleNamespace(pitch=0.0, roll=0.0, yaw=0.0)
        self.alive = True
        threading.Thread(target=self._integrate, daemon=True).start()

    # -- the link the modules drive through
    def send_drive(self, left, right):
        if self.opstop.latched:
            raise RuntimeError("operator stop is latched")
        with self.lock:
            self.l, self.r, self.last_cmd = left, right, time.monotonic()
            self.drives.append((left, right))

    def send_stop(self):
        with self.lock:
            self.l = self.r = 0.0

    def emergency_stop(self):
        self.send_stop()

    def range_snapshot(self):
        if not self.tof:
            return None
        d = self.dist_to(2) * 25.4 - 60.0      # board sits 60 mm behind the camera's view
        return {"channels": {"TOF_FRONT:0": {"sensor": "TOF_FRONT", "mm": d, "live": True}}}

    def _integrate(self):
        last = time.monotonic()
        while self.alive:
            time.sleep(0.005)
            now = time.monotonic()
            dt = now - last
            last = now
            with self.lock:
                if now - self.last_cmd > FAILSAFE_S:
                    self.l = self.r = 0.0
                v = IN_PER_S_PER_PWM * (self.l + self.r) / 2.0
                self.h += DEG_PER_S_PER_PWM_DIFF * (self.l - self.r) * dt
                self.x += v * math.cos(math.radians(self.h)) * dt
                self.y += v * math.sin(math.radians(self.h)) * dt
                self.hist.append((now, self.x, self.y, self.h))
                self.hist = self.hist[-3000:]

    def pose_at(self, t):
        with self.lock:
            p = self.hist[0]
            for e in self.hist:
                if e[0] > t:
                    break
                p = e
            return p[1:]

    def dist_to(self, tid):
        with self.lock:
            tx, ty = self.tags[tid]
            return math.hypot(tx - self.x, ty - self.y)

    def bearing_to(self, tid, pose=None):
        x, y, h = pose or (self.x, self.y, self.h)
        tx, ty = self.tags[tid]
        a = math.degrees(math.atan2(ty - y, tx - x)) - h
        return (a + 180) % 360 - 180

    def frame(self, now):
        pose = self.pose_at(now - LATENCY)
        out = []
        for tid, (tx, ty) in self.tags.items():
            rel = self.bearing_to(tid, pose)
            d = math.hypot(tx - pose[0], ty - pose[1])
            if abs(rel) >= HFOV / 2 or d < 1:
                continue
            u = W / 2 + F_PX * math.tan(math.radians(rel))
            s = F_PX * TAG_IN / d / 2
            out.append((tid, (u, H / 2), [(u - s, 240 + s), (u + s, 240 + s),
                                          (u + s, 240 - s), (u - s, 240 - s)]))
        return out

    # -- the closed-loop tools, simulated (they move the sim the way the real ones do)
    def drive_distance(self, inches, direction, pwm, confirmed_safe, lease=None):
        assert confirmed_safe
        sign = 1 if direction == "forward" else -1
        start = (self.x, self.y)
        while math.dist(start, (self.x, self.y)) < inches:
            try:
                self.send_drive(sign * pwm, sign * pwm)
            except RuntimeError as e:
                self.send_stop()
                return {"ok": False, "error": str(e)}
            time.sleep(0.02)
        self.send_stop()
        return {"ok": True, "actual_inches_traveled": round(sign * math.dist(start, (self.x, self.y)), 1)}

    def turn_degrees(self, degrees, direction, pwm, confirmed_safe, lease=None):
        assert confirmed_safe
        start = self.h
        while abs(self.h - start) < degrees:
            try:
                self.send_drive(pwm, -pwm) if direction == "right" else self.send_drive(-pwm, pwm)
            except RuntimeError as e:
                self.send_stop()
                return {"ok": False, "error": str(e)}
            time.sleep(0.01)
        self.send_stop()
        return {"ok": True, "actual_degrees_turned": round(-(self.h - start), 1)}


class FakeVision:
    def __init__(self, sim, address="sim"):
        import cv2
        import numpy as np
        self.sim = sim
        self.jpg = cv2.imencode(".jpg", np.zeros((H, W, 3), np.uint8))[1].tobytes()
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.seq, self.result, self.address = 0, None, address
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.sim.alive:
            now = time.monotonic()
            tags = self.sim.frame(now)
            with self.cond:
                self.seq += 1
                self.result = {"seq": self.seq, "tags": tags, "size": (W, H), "frame_t": now,
                               "jpg": self.jpg}
                self.cond.notify_all()
            time.sleep(1 / 30)

    def use(self, address=None, family=None):
        pass

    def touch(self):
        pass

    def status(self):
        return {"address": self.address,
                "tags": [{"id": t[0]} for t in (self.result or {}).get("tags", [])]}

    def wait_new(self, after_seq, timeout_s):
        with self.cond:
            self.cond.wait_for(lambda: self.seq > after_seq, timeout_s)
            return self.result if self.seq > after_seq else None


def setup(tags, tof=False, camera="sim"):
    sim = Sim(tags, tof=tof)

    def operator_stop(who="operator", reason=""):
        snap = sim.opstop.latch(who, reason)
        sim.send_stop()
        return snap

    ctx = types.SimpleNamespace(link=sim, LEASE=DriveLease(required=True), OPSTOP=sim.opstop,
                                drive_distance=sim.drive_distance, turn_degrees=sim.turn_degrees,
                                operator_stop=operator_stop)
    p = pm.Plan(ctx, vision=FakeVision(sim, camera))
    p.ping("test page")
    return sim, ctx, p


def run_plan(p, steps, during=None, timeout=90, repeat=1, record=False, video=False):
    code, out = p.run(steps, "test", repeat, record, video)
    assert code == 200, out
    if during:
        during()
    p.thread.join(timeout)
    assert not p.thread.is_alive(), "plan did not finish"
    return p.status()


def test_check_plan():
    clean, problems = pm.check_plan([{"cmd": "forward", "inches": 12}])
    assert not problems and clean[0]["pwm"] == 50
    assert pm.check_plan([{"cmd": "turn", "direction": "right", "degrees": 90}])[0][0]["pwm"] == 60
    for bad, why in [({"cmd": "forward", "inches": 12, "pwm": 30}, "pwm"),
                     ({"cmd": "forward", "inches": 12, "pwm": 120}, "pwm"),
                     ({"cmd": "toward", "tag": 2}, "frame width"),
                     ({"cmd": "turn", "direction": "up", "degrees": 90}, "direction"),
                     ({"cmd": "fly"}, "unknown"),
                     ({"cmd": "back"}, "inches")]:
        _, problems = pm.check_plan([bad])
        assert problems and why in problems[0], (bad, problems)
    assert pm.check_plan([])[1] == ["the plan is empty"]


def test_the_garage_plan():
    # Tag 1 straight ahead 48 in; tag 2 off to the right, 60 in away.
    sim, ctx, p = setup({1: (48.0, 3.0), 2: (10.0, 60.0)})
    try:
        steps = [{"cmd": "aim", "tag": 1},
                 {"cmd": "back", "inches": 12},
                 {"cmd": "aim", "tag": 1},
                 {"cmd": "turn", "direction": "right", "degrees": 90},
                 {"cmd": "find", "tag": 2},
                 {"cmd": "toward", "tag": 2, "width_pct": 29}]
        n_before_toward = [None]
        orig_toward = vd.toward

        def spy(drv, *a, **k):
            n_before_toward[0] = len(sim.drives)
            return orig_toward(drv, *a, **k)
        vd.toward = spy
        try:
            st = run_plan(p, steps)
        finally:
            vd.toward = orig_toward
        assert st["phase"] == "done", (st["note"], st["log"])
        assert all(s["state"] == "done" for s in st["steps"]), st["steps"]
        d = sim.dist_to(2)
        assert 10.0 <= d <= 15.0, d                              # ~12 in = 29% wide
        assert abs(sim.bearing_to(2)) < 8, sim.bearing_to(2)     # still pointed at it
        toward_cmds = sim.drives[n_before_toward[0]:]
        assert toward_cmds and all(l >= 0 and r >= 0 and l + r > 0 for l, r in toward_cmds), \
            "toward must arc forward (one wheel may stop to pivot), never spin"
        assert all(w == 0 or 50 <= w <= 100 for c in toward_cmds for w in c)
    finally:
        sim.alive = False


def test_toward_stops_on_tof():
    sim, ctx, p = setup({2: (60.0, 4.0)}, tof=True)
    try:
        st = run_plan(p, [{"cmd": "toward", "tag": 2, "width_pct": None, "tof_mm": 500}])
        assert st["phase"] == "done" and "ToF" in st["steps"][0]["result"], st
        mm = sim.dist_to(2) * 25.4 - 60
        assert 380 <= mm <= 520, mm
    finally:
        sim.alive = False


def test_tof_asked_for_without_a_sensor_fails_without_moving():
    sim, ctx, p = setup({2: (60.0, 0.0)}, tof=False)
    try:
        st = run_plan(p, [{"cmd": "toward", "tag": 2, "tof_mm": 400}])
        assert st["phase"] == "failed" and "ToF" in st["note"], st
        assert not sim.drives
    finally:
        sim.alive = False


def test_a_failed_step_stops_the_plan():
    sim, ctx, p = setup({1: (48.0, 0.0)})
    try:
        st = run_plan(p, [{"cmd": "aim", "tag": 9},
                          {"cmd": "forward", "inches": 12}])
        assert st["phase"] == "failed", st
        assert [s["state"] for s in st["steps"]] == ["failed", "skipped"], st["steps"]
        assert not sim.drives
    finally:
        sim.alive = False


def test_check_repeat():
    assert pm.check_repeat(None) == 1 and pm.check_repeat("") == 1
    assert pm.check_repeat(3) == 3 and pm.check_repeat("3") == 3 and pm.check_repeat(0) == 0
    for bad in (-1, pm.MAX_LAPS + 1, 2.5, "two", True):
        try:
            pm.check_repeat(bad)
        except ValueError:
            continue
        raise AssertionError("repeat=%r should be refused" % (bad,))


def test_repeat_runs_the_plan_n_times():
    sim, ctx, p = setup({1: (48.0, 0.0)})
    try:
        out_and_back = [{"cmd": "forward", "inches": 6, "pwm": 60},
                        {"cmd": "back", "inches": 6, "pwm": 60}]
        st = run_plan(p, out_and_back, repeat=3)
        assert st["phase"] == "done" and st["lap"] == 3 and st["laps"] == 3, st
        assert "3 laps" in st["note"], st["note"]
        assert sum("lap " in line for line in st["log"]) == 3, st["log"]
        assert sum("step 1:" in line for line in st["log"]) == 3, st["log"]
        assert all(s["state"] == "done" for s in st["steps"]), st["steps"]
        assert abs(sim.x) < 3, sim.x                          # back where it started
    finally:
        sim.alive = False


def test_repeat_until_stop_loops_until_stopped():
    sim, ctx, p = setup({1: (48.0, 0.0)})
    try:
        def press():
            while p.status()["lap"] < 3:
                time.sleep(0.05)
            p.stop("test")
        st = run_plan(p, [{"cmd": "wait", "seconds": 0.2}], during=press, repeat=0)
        assert st["phase"] == "stopped" and st["laps"] == 0 and st["lap"] >= 3, st
        assert "until Stop" in st["note"], st["note"]
    finally:
        sim.alive = False


def test_bad_repeat_refuses_the_run():
    sim, ctx, p = setup({1: (48.0, 0.0)})
    try:
        code, out = p.run([{"cmd": "wait", "seconds": 1}], "test", repeat=-2)
        assert code == 400 and "repeat" in out["error"], out
        assert p.thread is None
    finally:
        sim.alive = False


def test_not_ticked_saves_nothing():
    sim, ctx, p = setup({1: (48.0, 0.0)})
    try:
        st = run_plan(p, [{"cmd": "wait", "seconds": 0.2}])
        assert st["phase"] == "done" and st["recording"] is None
        assert not list(Path(sim.rec_dir).glob("plan_*"))
    finally:
        sim.alive = False


def test_record_saves_telemetry_with_step_marks():
    sim, ctx, p = setup({1: (48.0, 0.0)})
    try:
        st = run_plan(p, [{"cmd": "forward", "inches": 6, "pwm": 60},
                          {"cmd": "back", "inches": 6, "pwm": 60}], repeat=2, record=True)
        assert st["phase"] == "done", st
        rec = st["recording"]
        assert rec and not rec.get("recording") and "video" not in rec, rec
        d = Path(rec["dir"])
        assert d.name.startswith("plan_") and not (d / "video.avi").exists()
        with open(rec["csv"]) as f:
            marks = [r["note"] for r in csv.DictReader(f) if r["source"] == "MARK"]
        for want in ("lap 1 start", "lap 1 step 1 start: forward", "lap 2 step 2 done",
                     "lap 2 end", "plan end: done"):
            assert any(m.startswith(want) for m in marks), (want, marks)
        info = json.loads((d / "plan.json").read_text())
        assert info["phase"] == "done" and info["laps_run"] == 2 and info["repeat"] == 2
        assert len(info["steps"]) == 2 and info["who"] == "test"
        assert not sim.recorder.active                       # stopped with the plan
    finally:
        sim.alive = False


def test_record_video():
    import cv2
    sim, ctx, p = setup({1: (48.0, 0.0)})
    try:
        st = run_plan(p, [{"cmd": "wait", "seconds": 1.5}], record=True, video=True)
        rec = st["recording"]
        assert rec["video"] and not rec["video_error"], rec
        cap = cv2.VideoCapture(rec["video"])
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        ok, img = cap.read()
        cap.release()
        assert ok and img.shape[:2] == (H, W), (ok, None if img is None else img.shape)
        secs = n / pm.VIDEO_FPS
        assert abs(secs - rec["duration_s"]) < 0.8, (n, rec)   # real-time, in step with the CSV
        with open(Path(rec["dir"]) / "frames.csv") as f:
            rows = list(csv.DictReader(f))
        assert len(rows) == rec["camera_frames"] > 10
        assert any(r["tags"].startswith("1@") for r in rows), rows[:3]   # tag 1 dead ahead
        info = json.loads((Path(rec["dir"]) / "plan.json").read_text())
        for r in rows:                                         # wall clock = CSV t0 + time_s
            assert abs(float(r["unix_s"]) - (info["csv_t0_unix_s"] + float(r["time_s"]))) < 0.05, r
            assert r["wall_time"] == pm.wall_stamp(float(r["unix_s"])), r
        assert img[H - 5, 5].max() < 40 and img[H - 30:H, :200].max() > 200   # stamp burned in
    finally:
        sim.alive = False


def test_video_stays_real_time_with_fast_and_slow_cameras():
    for cam_fps in (21.0, 10.0, 60.0):
        written = dropped = 0
        for i in range(int(cam_fps * 60)):                 # one minute of camera frames
            n = pm.video_copies(i / cam_fps, written, fps=30)
            written += n
            dropped += n == 0
        assert abs(written / 30 - 60) < 0.1, (cam_fps, written)   # plays for 60 s
        assert (dropped > 0) == (cam_fps > 30), (cam_fps, dropped)
    assert pm.video_copies(0.0, 0, fps=30) == 1
    assert pm.video_copies(100.0, 0, fps=30, max_repeat_s=5) == 150   # long gap: capped


def test_record_video_is_saved_when_the_run_is_stopped():
    sim, ctx, p = setup({1: (48.0, 0.0)})
    try:
        def press():
            time.sleep(0.8)
            p.stop("test")
        st = run_plan(p, [{"cmd": "wait", "seconds": 1}], during=press, repeat=0,
                      record=True, video=True)
        assert st["phase"] == "stopped" and st["recording"]["video"], st
        info = json.loads((Path(st["recording"]["dir"]) / "plan.json").read_text())
        assert info["phase"] == "stopped" and info["repeat"] == 0
    finally:
        sim.alive = False


def test_record_refusals_move_nothing():
    sim, ctx, p = setup({1: (48.0, 0.0)}, camera="")
    try:
        code, out = p.run([{"cmd": "forward", "inches": 6}], "test", 1, True, True)
        assert code == 409 and "camera address" in out["error"], out
        sim.recorder.start("someone_else")
        code, out = p.run([{"cmd": "forward", "inches": 6}], "test", 1, True, False)
        assert code == 409 and "already recording" in out["error"], out
        assert p.thread is None and not sim.drives
        sim.recorder.stop()
    finally:
        sim.alive = False


def test_find_turns_until_the_tag_shows():
    sim, ctx, p = setup({3: (-30.0, -30.0)})          # behind and to the left
    try:
        st = run_plan(p, [{"cmd": "find", "tag": 3, "direction": "left"}])
        assert st["phase"] == "done", (st["note"], st["log"])
        assert abs(sim.bearing_to(3)) <= vd.DEADBAND_DEG + 0.5, sim.bearing_to(3)
    finally:
        sim.alive = False


def test_stop_latches_the_operator_stop_mid_drive():
    sim, ctx, p = setup({1: (48.0, 0.0)})
    try:
        def press():
            time.sleep(0.5)
            p.stop("test")
        st = run_plan(p, [{"cmd": "forward", "inches": 100}, {"cmd": "wait", "seconds": 1}],
                      during=press)
        assert st["phase"] == "stopped" and ctx.OPSTOP.latched, st
        assert sim.x < 30, sim.x                              # cut short, not 100 in
        assert [s["state"] for s in st["steps"]] == ["failed", "skipped"]
        code, out = p.run([{"cmd": "wait", "seconds": 1}], "test")
        assert code == 409 and "latched" in out["error"]      # Resume first
    finally:
        sim.alive = False


def test_page_closing_releases_the_lease():
    sim, ctx, p = setup({1: (48.0, 0.0)})
    old = vd.PAGE_TIMEOUT_S
    try:
        assert ctx.LEASE.snapshot()["holder"] == "plan"
        vd.PAGE_TIMEOUT_S = 0.3
        time.sleep(0.4)
        assert p.watchdog() and not ctx.LEASE.snapshot()["held"]
    finally:
        vd.PAGE_TIMEOUT_S = old
        sim.alive = False


def test_arc_pwms():
    for slow in (False, True):
        for b in (-40, -10, -5, -2, 0, 2, 5, 10, 40):
            l, r = vd.arc_pwms(b, 60, slow)
            assert all(w == 0 or 50 <= w <= 100 for w in (l, r)), (b, slow, l, r)
            if abs(b) <= vd.DEADBAND_DEG:
                assert l == r, (b, slow, l, r)              # centred: straight
            elif b > 0:
                assert l > r, (b, slow, l, r)               # tag right: curve right
            else:
                assert r > l, (b, slow, l, r)
    for slow in (False, True):                              # off centre: pivot, any distance
        assert vd.arc_pwms(10, 60, slow) == (70, 0)
        assert vd.arc_pwms(-5, 60, slow) == (0, 70)
    assert vd.arc_pwms(10, 85) == (85, 0)                   # a faster step pivots faster
    assert vd.arc_pwms(1, 60) == (60, 60)                   # centred: straight at the step pwm
    assert vd.arc_pwms(1, 60, slow=True) == (50, 50)        # centred and close: creep



def test_check_arc_steps():
    clean, problems = pm.check_plan([{"cmd": "arc", "direction": "left", "radius_in": 24,
                                      "degrees": 90, "stop_after": False, "warning": "x"}])
    assert not problems, problems
    assert clean[0] == {"cmd": "arc", "direction": "left", "radius_in": 24.0, "degrees": 90.0,
                        "travel": "forward", "pwm": 60}, clean
    assert "arc back left 90 deg, radius 24 in" in pm.describe(
        pm.check_step({"cmd": "arc", "direction": "left", "radius_in": 24, "degrees": 90,
                       "travel": "reverse"}))
    for bad, why in [({"cmd": "arc", "direction": "up", "radius_in": 24, "degrees": 90}, "direction"),
                     ({"cmd": "arc", "direction": "left", "radius_in": 3, "degrees": 90}, "radius_in"),
                     ({"cmd": "arc", "direction": "left", "radius_in": 24, "degrees": 400}, "degrees"),
                     ({"cmd": "arc", "direction": "left", "radius_in": 100, "degrees": 180}, "split"),
                     ({"cmd": "arc", "direction": "left", "radius_in": 24, "degrees": 90,
                       "travel": "sideways"}, "travel")]:
        _, problems = pm.check_plan([bad])
        assert problems and why in problems[0], (bad, problems)


def test_arc_step_calls_drive_arc():
    sim, ctx, p = setup({1: (48.0, 0.0)})
    calls = []

    def drive_arc(radius, degrees, direction, pwm, confirmed_safe, travel="forward", lease=None):
        calls.append((radius, degrees, direction, pwm, confirmed_safe, travel, lease))
        return {"ok": True, "actual_degrees_turned": -degrees}
    ctx.drive_arc = drive_arc
    try:
        st = run_plan(p, [{"cmd": "arc", "direction": "right", "radius_in": 30, "degrees": 45,
                           "travel": "reverse", "pwm": 70}])
        assert st["phase"] == "done", st
        assert calls == [(30.0, 45.0, "right", 70, True, "reverse", p.lease)], calls
        assert st["steps"][0]["result"].startswith("turned 45"), st["steps"]
        ctx.drive_arc = lambda *a, **k: {"ok": False, "abort_reason": "IMU went stale"}
        st = run_plan(p, [{"cmd": "arc", "direction": "left", "radius_in": 30, "degrees": 45}])
        assert st["phase"] == "failed" and "IMU went stale" in st["note"], st
    finally:
        sim.alive = False


def test_drawings_list_and_load():
    d = Path(tempfile.mkdtemp(prefix="drawings_"))
    steps = [{"cmd": "forward", "inches": 48, "pwm": 50, "stop_after": False},
             {"cmd": "arc", "direction": "left", "radius_in": 18, "degrees": 90,
              "travel": "forward", "pwm": 60, "stop_after": True, "warning": "leaves the map"}]
    (d / "garage_loop.json").write_text(json.dumps({"name": "garage loop", "steps": steps,
                                                    "saved": "2026-11-28 06:00:00"}))
    (d / "broken.json").write_text("{not json")
    lst = pm.list_drawings(d)
    assert {x["file"] for x in lst} == {"garage_loop.json", "broken.json"}, lst
    good = next(x for x in lst if x["file"] == "garage_loop.json")
    assert good["name"] == "garage loop" and good["steps"] == 2
    assert "error" in next(x for x in lst if x["file"] == "broken.json")
    code, out = pm.load_drawing("garage_loop.json", d)
    assert code == 200 and out["ok"] and not out["problems"] and out["steps"] == steps, out
    assert any("1 smooth link" in n for n in out["notes"]), out["notes"]
    assert any("leaves the map" in n for n in out["notes"]), out["notes"]
    assert pm.load_drawing("nope.json", d)[0] == 404
    for bad in ("../secret.json", "/etc/passwd", "a/b.json", "x.txt", None, ".json/.."):
        assert pm.load_drawing(bad, d)[0] == 400, bad


def test_drawer_launch():
    d = Path(tempfile.mkdtemp(prefix="drawer_")) / "plans"
    d.mkdir()
    (d / "lap.json").write_text("{}")
    script = d.parent / "plan_drawer.py"
    script.write_text("")
    started = []

    class Proc:
        def __init__(self, cmd, **kw):
            started.append((cmd, kw))
            self.alive = True

        def poll(self):
            return None if self.alive else 0
    dr = pm.Drawer(script, d, popen=Proc)
    code, out = dr.launch(env={})
    assert code == 409 and "no screen" in out["error"] and not started, out
    code, out = dr.launch("lap.json", env={"DISPLAY": ":0"})
    assert code == 200, out
    cmd, kw = started[0]
    assert cmd[-2:] == ["--load", str((d / "lap.json").resolve())], cmd
    assert kw["env"]["DISPLAY"] == ":0" and kw["start_new_session"]
    code, out = dr.launch(env={"DISPLAY": ":0"})
    assert code == 409 and "already open" in out["error"], out
    dr.proc.alive = False
    assert dr.launch(env={"DISPLAY": ":0"})[0] == 200 and "--load" not in started[1][0]
    dr.proc.alive = False
    assert dr.launch("../x.json", env={"DISPLAY": ":0"})[0] == 400
    assert dr.launch("missing.json", env={"DISPLAY": ":0"})[0] == 404
    assert len(started) == 2


def test_the_drawer_saves_plans_the_page_accepts():
    """A drawing made with the drawer's own geometry loads cleanly here."""
    sys.path.insert(0, str(pm.DRAWER.parent))
    import plan_drawer as pdraw
    app = pdraw.App(288, 240, Path(tempfile.mkdtemp()) / "x.json", name="check")
    app.start = {"x": 24.0, "y": 120.0, "heading": 0.0}
    app.tags = [{"id": 5, "x": 260.0, "y": 120.0, "face": 180.0}]
    app.intents = [
        {"kind": "line", "target": [72, 120], "stop": True},
        {"kind": "arc", "target": [120, 168], "stop": True},
        {"kind": "line", "target": [168, 60], "smooth": True, "R": 18, "stop": False},
        {"kind": "arc", "target": [140, 40], "rev": True, "stop": True},
        {"kind": "line", "target": [100, 100], "rev": True, "smooth": False, "R": 18, "stop": True},
        {"kind": "aim", "tag": 5},
        {"kind": "toward", "tag": 5, "stop_in": 18, "stop": True},
    ]
    app.save()
    code, out = pm.load_drawing(app.out.name, app.out.parent)
    assert code == 200 and not out["problems"], out
    assert {s["cmd"] for s in out["steps"]} >= {"forward", "arc", "turn", "back", "find", "toward"} \
        or {s["cmd"] for s in out["steps"]} >= {"forward", "arc", "turn", "back", "aim", "toward"}, out


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
