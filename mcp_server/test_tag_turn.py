#!/usr/bin/env python3
"""Hardware-free tests for tag_turn.py against a simulated robot and camera.

The robot spins at a rate set by the spin PWM it is sent (and stops if commands stop
arriving, like the motor board's fail-safe). The camera shows the world as it was
LATENCY seconds ago, so stopping on the first "centred" frame would overshoot.

    ELLAMA_SERIAL_PORT=/dev/none mcp_server/.venv/bin/python mcp_server/test_tag_turn.py
"""

import math
import random
import sys
import threading
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tag_turn as tt  # noqa: E402
import vision_drive as vd  # noqa: E402
from drive_lease import DriveLease  # noqa: E402
from operator_stop import OperatorStop  # noqa: E402

W, H = 640, 480
HFOV = 60.0
DEG_PER_S_PER_PWM = 0.3      # pwm 50 -> 15 deg/s, pwm 100 -> 30 deg/s
FAILSAFE_S = 0.3
vd.hfov_deg = lambda: (HFOV, "test")


class SimRobot:
    """Heading (+ = right) driven by send_drive, recorded over time for the camera."""

    def __init__(self, tag_deg, latency=0.4, drop=0.0):
        self.lock = threading.Lock()
        self.heading, self.rate, self.last_cmd = 0.0, 0.0, 0.0
        self.history = [(time.monotonic(), 0.0)]
        self.tag_deg, self.latency, self.drop = tag_deg, latency, drop
        self.tag_present = True
        self.commands = []           # (left, right) as sent
        self.alive = True
        threading.Thread(target=self._integrate, daemon=True).start()

    # the link the module drives through
    def send_drive(self, left, right):
        assert left == -right, (left, right)        # spins only
        with self.lock:
            self.commands.append((left, right))
            self.rate = DEG_PER_S_PER_PWM * left    # left wheel forward = nose right
            self.last_cmd = time.monotonic()

    def send_stop(self):
        with self.lock:
            self.rate = 0.0

    def _integrate(self):
        last = time.monotonic()
        while self.alive:
            time.sleep(0.005)
            now = time.monotonic()
            with self.lock:
                if now - self.last_cmd > FAILSAFE_S:
                    self.rate = 0.0
                self.heading += self.rate * (now - last)
                self.history.append((now, self.heading))
                self.history = self.history[-2000:]
            last = now

    def heading_at(self, t):
        with self.lock:
            h = self.history[0][1]
            for ts, hd in self.history:
                if ts > t:
                    break
                h = hd
            return h

    def error(self):
        with self.lock:
            return self.tag_deg - self.heading

    def tags_in_frame(self, now):
        rel = self.tag_deg - self.heading_at(now - self.latency)
        if not self.tag_present or abs(rel) >= HFOV / 2 or random.random() < self.drop:
            return []
        f = (W / 2) / math.tan(math.radians(HFOV) / 2)
        u = W / 2 + f * math.tan(math.radians(rel))
        return [(7, (u, H / 2), [(u - 20, 260), (u + 20, 260), (u + 20, 220), (u - 20, 220)])]


class FakeVision:
    """The parts of apriltag_module.VIEW tag turn uses, at 30 frames a second."""

    def __init__(self, robot):
        self.robot = robot
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.seq, self.result, self.address = 0, None, "sim"
        self.sending = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.robot.alive:
            if self.sending:
                now = time.monotonic()
                tags = self.robot.tags_in_frame(now)
                with self.cond:
                    self.seq += 1
                    self.result = {"seq": self.seq, "tags": tags, "size": (W, H), "frame_t": now}
                    self.cond.notify_all()
            time.sleep(1 / 30)

    def use(self, address=None, family=None):
        pass

    def touch(self):
        pass

    def status(self):
        return {}

    def wait_new(self, after_seq, timeout_s):
        with self.cond:
            self.cond.wait_for(lambda: self.seq > after_seq, timeout_s)
            return self.result if self.seq > after_seq else None


def setup(tag_deg, **kw):
    robot = SimRobot(tag_deg, **kw)
    ctx = types.SimpleNamespace(link=robot, LEASE=DriveLease(required=True), OPSTOP=OperatorStop())
    s = tt.TagTurn(ctx, vision=FakeVision(robot))
    s.ping("test page")
    return robot, ctx, s


def run_center(s, tag_id=7, during=None, timeout=30):
    code, out = s.center(tag_id, "test")
    assert code == 200, out
    if during:
        during()
    s.thread.join(timeout)
    assert not s.thread.is_alive()
    return s.status()


def direction_changes(robot):
    signs = [1 if l > 0 else -1 for l, _ in robot.commands]
    return sum(1 for a, b in zip(signs, signs[1:]) if a != b)


def test_pwm_steps():
    assert [tt.pwm_for(a) for a in (0, 5, -6, 8, 15, -25, 31, 90)] == \
        [50, 50, 50, 63, 75, 88, 100, 100]
    assert sorted({p for _, p in tt.PWM_STEPS}) == [50, 63, 75, 88, 100]


def test_centres_a_tag_on_the_right():
    robot, ctx, s = setup(25.0)
    try:
        st = run_center(s)
        assert st["phase"] == "done", st["log"]
        assert abs(robot.error()) <= tt.DEADBAND_DEG, robot.error()
        assert robot.commands[0][0] > 0                       # turned right first
        assert direction_changes(robot) <= 1, robot.commands
    finally:
        robot.alive = False


def test_centres_a_far_tag_on_the_left_within_pwm_limits():
    robot, ctx, s = setup(-28.0)
    try:
        st = run_center(s)
        assert st["phase"] == "done", st["log"]
        assert abs(robot.error()) <= tt.DEADBAND_DEG, robot.error()
        pwms = {abs(l) for l, _ in robot.commands}
        assert pwms <= {50, 63, 75, 88, 100} and 88 in pwms and 50 in pwms, pwms
    finally:
        robot.alive = False


def test_already_centred_does_not_move():
    robot, ctx, s = setup(1.5)
    try:
        st = run_center(s)
        assert st["phase"] == "done" and not robot.commands, (st["log"], robot.commands)
    finally:
        robot.alive = False


def test_dropped_detections_are_tolerated():
    random.seed(3)
    robot, ctx, s = setup(18.0, drop=0.25)
    try:
        st = run_center(s)
        assert st["phase"] == "done", st["log"]
        assert abs(robot.error()) <= tt.DEADBAND_DEG
    finally:
        robot.alive = False


def test_tag_leaving_the_frame_brakes_and_fails():
    robot, ctx, s = setup(25.0)
    try:
        def remove_tag():
            time.sleep(0.4)
            robot.tag_present = False
        st = run_center(s, during=remove_tag)
        assert st["phase"] == "failed" and "not in view" in st["note"], st
        assert robot.rate == 0.0
    finally:
        robot.alive = False


def test_wrong_tag_id_never_moves():
    robot, ctx, s = setup(10.0)
    try:
        st = run_center(s, tag_id=3)
        assert st["phase"] == "failed" and not robot.commands, st
    finally:
        robot.alive = False


def test_camera_stopping_brakes_and_fails():
    # A frame gap is a pause (brake and wait); only CAMERA_LOST_S of nothing fails.
    robot, ctx, s = setup(25.0)
    old = vd.CAMERA_LOST_S
    vd.CAMERA_LOST_S = 1.5
    try:
        def freeze():
            time.sleep(0.4)
            s.view.sending = False
        st = run_center(s, during=freeze)
        assert st["phase"] == "failed" and "camera" in st["note"], st
        assert robot.rate == 0.0
        assert any("camera paused" in line for line in st["log"]), st["log"]
    finally:
        vd.CAMERA_LOST_S = old
        robot.alive = False


def test_stop_button_and_hub_stop():
    for how in ("page", "hub"):
        robot, ctx, s = setup(30.0)
        try:
            def press():
                time.sleep(0.4)
                if how == "page":
                    s.stop("test")
                else:
                    ctx.OPSTOP.latch("test", "hub STOP")
            st = run_center(s, during=press)
            assert st["phase"] in ("stopped", "failed"), (how, st)
            assert robot.rate == 0.0, how
            assert abs(robot.error()) > 10, how                # it really stopped early
        finally:
            robot.alive = False


def test_lease_is_held_while_open_and_released_when_page_goes_quiet():
    robot, ctx, s = setup(10.0)
    old = vd.PAGE_TIMEOUT_S
    try:
        assert s.status()["holding_lease"]
        assert ctx.LEASE.snapshot()["holder"] == "tag-turn"
        vd.PAGE_TIMEOUT_S = 0.3
        assert not s.watchdog()             # just pinged
        time.sleep(0.4)
        assert s.watchdog()                 # page went quiet
        assert not ctx.LEASE.snapshot()["held"] and not s.status()["holding_lease"]
        code, out = s.center(7, "test")
        assert code == 409, out
        s.ping("test page")                 # page back: lease back
        assert s.status()["holding_lease"]
    finally:
        vd.PAGE_TIMEOUT_S = old
        robot.alive = False


def test_someone_else_holding_the_lease_blocks_center():
    robot = SimRobot(10.0)
    try:
        ctx = types.SimpleNamespace(link=robot, LEASE=DriveLease(required=True),
                                    OPSTOP=OperatorStop())
        ok, _ = ctx.LEASE.take("dashboard", "someone", ttl_s=60)
        assert ok
        s = tt.TagTurn(ctx, vision=FakeVision(robot))
        s.ping("test page")
        st = s.status()
        assert not st["holding_lease"] and "dashboard" in st["lease_error"], st
        code, out = s.center(7, "test")
        assert code == 409 and not robot.commands
    finally:
        robot.alive = False


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
