#!/usr/bin/env python3
"""Camera-guided moves shared by the tag turn and plan pages.

    center(drv, tag_id)                      spin onto a tag (tag turn's Center)
    toward(drv, tag_id, width_pct, tof_mm)   arc toward a tag until it fills width_pct of
                                             the frame width, or a ToF reads tof_mm
    find(drv, tag_id, direction, pwm)        turn in steps until the tag shows, then center

Each returns (ok, note) and leaves the robot braked. A Driver carries what a move needs:
ctx (the robot_server module), the shared detection (apriltag_module.VIEW), the drive
lease token, and a stop event. Spins and arcs are momentary send_drive commands under the
lease and guards, like the dashboard's hold-to-drive buttons; find's steps are
turn_degrees.

PageLease: a page that holds the drive lease while it is open (pinged every 0.5 s by the
page; brake and release after PAGE_TIMEOUT_S of silence).
"""

import math
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # for bridge/
from drive_math import FORWARD_PWM_SIGN  # noqa: E402

DEADBAND_DEG = 3.0
SPIN_STEPS = ((6.0, 50), (12.0, 63), (20.0, 75), (30.0, 88), (math.inf, 100))
PWM_MIN, PWM_MAX = 50, 100
RECHECK_S = 0.6          # after braking: camera latency + settle before a frame counts
FRAME_GAP_S = 0.5        # no new frame this long while moving: brake and wait for the camera
CAMERA_LOST_S = 30.0     # ...and give up only after this long with no frames at all
RESEND_S = 0.2           # re-send a drive this often (motor board fail-safe is 300 ms)
LOST_S = 5.0             # tag missing this long (braked at the first miss, waiting) = give up
CENTER_TIMEOUT_S = 60.0
TOWARD_TIMEOUT_S = 120.0
PIVOT_PWM = 70           # toward, tag outside the deadband: outer wheel (or the step's pwm if
                         # higher); the inner wheel stops
STALL_S = 1.5           # commanding a move but the tag has not moved in the picture this long = stalled
STALL_DEG = 1.0         # ...by this many degrees (or, for toward, STALL_WIDTH_PCT of width)
STALL_WIDTH_PCT = 0.5
STALL_BOOST_PWM = 10     # stalled: add this much PWM and keep going; give up only stalled at PWM_MAX
TOWARD_LEAD_S = 0.5      # toward: brake when the width predicted this far ahead reaches the target
TOWARD_SLOW_FRAC = 0.75  # toward: drop to PWM_MIN once the tag is this close to its target width
TOWARD_DONE_FRAC = 0.95  # toward: after braking, this close to the target width counts as there
PAGE_TIMEOUT_S = 3.0
LEASE_TTL_S = 10
DEFAULT_HFOV_DEG = 60.0
# Measured 2026-11-26: tag 5 filled 29% of a 320x240 frame width at 12 in from the
# camera. Distance is inversely proportional to that fraction; used only to show an
# estimate next to a width_pct, never to steer.
REF_WIDTH_PCT, REF_DISTANCE_IN = 29.0, 12.0


def est_distance_in(width_pct):
    return REF_DISTANCE_IN * REF_WIDTH_PCT / width_pct if width_pct > 0 else None


def spin_pwm(angle_deg):
    a = abs(angle_deg)
    for limit, pwm in SPIN_STEPS:
        if a <= limit:
            return pwm
    return PWM_MAX


def hfov_deg():
    """(hfov, where it came from) from the active robot profile's camera block."""
    try:
        import bridge
        cam = bridge.load_config().get("camera") or {}
        hfov = float(cam.get("hfov_deg", DEFAULT_HFOV_DEG))
        src = ("profile %s" % bridge.ROBOT_NAME if "hfov_deg" in cam
               else "default 60 (a guess; set camera.hfov_deg in the robot profile)")
    except Exception as e:
        hfov, src = DEFAULT_HFOV_DEG, "default 60 (profile unreadable: %s)" % e
    if not 10.0 <= hfov <= 170.0:
        hfov, src = DEFAULT_HFOV_DEG, "default 60 (profile hfov_deg %r out of range)" % hfov
    return hfov, src


def bearing_deg(center_x_px, width_px, hfov):
    """Angle from the camera's centre line to a pixel column; positive = right."""
    f = (width_px / 2.0) / math.tan(math.radians(hfov) / 2.0)
    return math.degrees(math.atan((center_x_px - width_px / 2.0) / f))


def _side_px(corners):
    return statistics.mean(math.dist(corners[i], corners[(i + 1) % 4]) for i in range(4))


def sight(result, tag_id, hfov):
    """(bearing_deg, width_pct, id) of tag_id (None = biggest) in a detection, or Nones."""
    tags = [t for t in (result or {}).get("tags") or [] if tag_id is None or t[0] == tag_id]
    if not tags:
        return None, None, None
    tid, (cx, _), corners = max(tags, key=lambda t: _side_px(t[2]))
    w = result["size"][0]
    return bearing_deg(cx, w, hfov), 100.0 * _side_px(corners) / w, tid


def front_tof_mm(link):
    """Closest live front-facing ToF median in mm; None if none is reporting a distance;
    raises LookupError if the robot has no front range sensor at all."""
    snap = link.range_snapshot() if link is not None else None
    chans = [c for c in ((snap or {}).get("channels") or {}).values()
             if "REAR" not in str(c.get("sensor", "")).upper()]
    if not chans:
        raise LookupError("no front-facing ToF sensor is reporting")
    mms = [c["mm"] for c in chans if c.get("live") and c.get("mm") is not None]
    return min(mms) if mms else None


def _vision():
    import apriltag_module
    return apriltag_module.VIEW


class Driver:
    def __init__(self, ctx, lease, stop_evt, log, view=None, status=None):
        self.ctx, self.lease, self.stop_evt, self.log = ctx, lease, stop_evt, log
        self.view = view if view is not None else _vision()
        self.status = status or (lambda note, pwm=0: None)
        self.hfov, self.hfov_source = hfov_deg()

    def halted(self):
        if self.stop_evt.is_set():
            return "stopped"
        if self.ctx.OPSTOP.latched:
            return "operator stop is latched"
        return None

    def send(self, left, right):
        """One momentary drive under the lease and guards. None, or why it was refused."""
        if self.ctx.link is None:
            return "no robot connected"
        if self.ctx.OPSTOP.latched:
            return "operator stop is latched"
        err = self.ctx.LEASE.check(self.lease)
        if err is not None:
            return "lease: " + err
        self.ctx.LEASE.begin()
        try:
            self.ctx.link.send_drive(left, right)
        except Exception as e:
            return "guard: %s" % e
        finally:
            self.ctx.LEASE.end()
        return None

    def spin(self, direction, pwm):
        return self.send(pwm, -pwm) if direction == "right" else self.send(-pwm, pwm)

    def brake(self):
        try:
            if self.ctx.link is not None:
                self.ctx.link.send_stop()
        except Exception:
            pass

    def frames(self):
        """Generator of new detection results; yields None when a RESEND_S tick passes
        without one. frame_gap() says how long since the last one: callers brake past
        FRAME_GAP_S and give up only past CAMERA_LOST_S, so dropped frames are a pause."""
        v = self.view
        with v.lock:
            seq = v.seq
        self.last_frame = time.monotonic()
        while True:
            v.touch()
            res = v.wait_new(seq, RESEND_S)
            if res is None:
                yield None
                continue
            seq, self.last_frame = res["seq"], time.monotonic()
            yield res

    def frame_gap(self):
        return time.monotonic() - getattr(self, "last_frame", time.monotonic())

    def camera_lost(self):
        return self.frame_gap() > CAMERA_LOST_S

    def finish(self, ok, note):
        self.brake()
        self.log(note)
        self.status(note, 0)
        return ok, note


def center(drv, tag_id, timeout_s=CENTER_TIMEOUT_S):
    """Spin toward the tag at a stepped PWM; inside the deadband brake, let the camera
    catch up, and look again."""
    want = "tag %d" % tag_id if tag_id is not None else "the biggest tag"
    t0 = time.monotonic()
    direction, pwm, missing_since, recheck_after = None, 0, None, None
    stall = None              # (time, bearing) when the current spin last saw the tag move
    boost = 0                 # PWM added after a stall
    try:
        for res in drv.frames():
            why = drv.halted()
            if why:
                return drv.finish(False, why)
            if time.monotonic() - t0 > timeout_s:
                return drv.finish(False, "not centered on %s after %.0f s" % (want, timeout_s))
            if res is None:
                if drv.camera_lost():
                    return drv.finish(False, "no camera frames for %.0f s" % CAMERA_LOST_S)
                if drv.frame_gap() > FRAME_GAP_S:
                    if direction is not None:          # camera hiccup: hold still, wait
                        drv.brake()
                        direction, stall = None, None
                        drv.log("camera paused: braked, waiting for frames")
                        drv.status("camera paused: waiting for frames", 0)
                elif direction is not None:
                    err = drv.spin(direction, pwm)
                    if err:
                        return drv.finish(False, "refused: " + err)
                continue
            if recheck_after is not None and (res.get("frame_t") or 0) < recheck_after:
                continue
            b, _, tid = sight(res, tag_id, drv.hfov)
            if b is None:
                if direction is not None:
                    drv.brake()
                    direction, stall = None, None
                missing_since = missing_since or time.monotonic()
                if time.monotonic() - missing_since > LOST_S:
                    return drv.finish(False, "%s is not in view" % want)
                continue
            missing_since = None
            if abs(b) <= DEADBAND_DEG:
                if recheck_after is not None:
                    return drv.finish(True, "centered on tag %d (%+.1f deg)" % (tid, b))
                drv.brake()
                direction, stall = None, None
                drv.log("tag %d at %+.1f deg: inside the deadband, braked, checking" % (tid, b))
                drv.status("tag %d at %+.1f deg: braked, checking" % (tid, b), 0)
                recheck_after = time.monotonic() + RECHECK_S
                continue
            recheck_after = None
            new_dir = "right" if b > 0 else "left"
            new_pwm = min(PWM_MAX, spin_pwm(b) + boost)
            if new_dir != direction or stall is None or abs(b - stall[1]) >= STALL_DEG:
                stall = (time.monotonic(), b)
            elif time.monotonic() - stall[0] > STALL_S:
                if new_pwm >= PWM_MAX:
                    return drv.finish(False, "not turning: tag %d stayed at %+.1f deg for %.1f s "
                                      "spinning %s even at pwm %d (stuck?)"
                                      % (tid, b, STALL_S, new_dir, new_pwm))
                boost += STALL_BOOST_PWM
                new_pwm = min(PWM_MAX, spin_pwm(b) + boost)
                drv.log("not turning at pwm %d: boosting to %d" % (pwm, new_pwm))
                stall = (time.monotonic(), b)
            if (new_dir, new_pwm) != (direction, pwm):
                drv.log("tag %d at %+.1f deg: spin %s at pwm %d" % (tid, b, new_dir, new_pwm))
            direction, pwm = new_dir, new_pwm
            drv.status("tag %d at %+.1f deg: spinning %s at %d" % (tid, b, direction, pwm), pwm)
            err = drv.spin(direction, pwm)
            if err:
                return drv.finish(False, "refused: " + err)
    except TimeoutError as e:
        return drv.finish(False, str(e))
    except Exception as e:  # a bug must still brake
        return drv.finish(False, "crashed: %r" % e)


def arc_pwms(bearing, pwm, slow=False):
    """(left, right) forward PWMs toward a tag at `bearing` (+ = right).

    Tag within DEADBAND_DEG: straight, at pwm (or PWM_MIN when slow, closing in on the
    target width). Outside it, at any distance: pivot on the inner wheel -- the outer
    wheel at max(PIVOT_PWM, pwm), the inner one stopped. Never a wheel backwards, so
    never a spin in place; never a wheel between 0 and PWM_MIN.
    """
    if abs(bearing) <= DEADBAND_DEG:
        straight = PWM_MIN if slow else pwm
        return straight, straight
    outer = min(PWM_MAX, max(PIVOT_PWM, pwm))
    return (outer, 0) if bearing > 0 else (0, outer)


def toward(drv, tag_id, width_pct=None, tof_mm=None, pwm=60, timeout_s=TOWARD_TIMEOUT_S):
    """Drive forward on an arc that keeps the tag centred, until the tag fills width_pct
    of the frame width or a front ToF reads tof_mm, whichever comes first."""
    want = "tag %d" % tag_id if tag_id is not None else "the biggest tag"
    if width_pct is None and tof_mm is None:
        return drv.finish(False, "toward needs a frame width % or a ToF distance to stop at")
    if tof_mm is not None:
        try:
            front_tof_mm(drv.ctx.link)
        except LookupError as e:
            return drv.finish(False, "ToF stop asked for, but %s" % e)
    t0 = time.monotonic()
    missing_since, driving, last_cmd = None, False, (0, 0)
    seen = []                 # (frame_t, width %) while driving, to predict ahead
    stall = None              # (time, bearing, width) when the arc last saw the tag move
    recheck_after = None      # braked at the predicted stop: frames before this don't count
    boost = 0                 # PWM added after a stall
    try:
        for res in drv.frames():
            why = drv.halted()
            if why:
                return drv.finish(False, why)
            if time.monotonic() - t0 > timeout_s:
                return drv.finish(False, "did not reach %s after %.0f s" % (want, timeout_s))
            if tof_mm is not None:
                d = front_tof_mm(drv.ctx.link)
                if d is not None and d <= tof_mm:
                    return drv.finish(True, "stopped by ToF at %.0f mm" % d)
            if res is None:
                if drv.camera_lost():
                    return drv.finish(False, "no camera frames for %.0f s" % CAMERA_LOST_S)
                if drv.frame_gap() > FRAME_GAP_S:
                    if driving:                        # camera hiccup: hold still, wait
                        drv.brake()
                        driving, stall, seen = False, None, []
                        drv.log("camera paused: braked, waiting for frames")
                        drv.status("camera paused: waiting for frames", 0)
                elif driving:
                    err = drv.send(*last_cmd)
                    if err:
                        return drv.finish(False, "refused: " + err)
                continue
            b, w, tid = sight(res, tag_id, drv.hfov)
            if b is None:
                if driving:
                    drv.brake()
                    driving, stall = False, None
                missing_since = missing_since or time.monotonic()
                if time.monotonic() - missing_since > LOST_S:
                    return drv.finish(False, "%s is not in view" % want)
                continue
            missing_since = None
            if width_pct is not None:
                if recheck_after is not None:
                    if (res.get("frame_t") or 0) < recheck_after:
                        continue                   # still the picture from before the brake
                    if w >= width_pct * TOWARD_DONE_FRAC:
                        return drv.finish(True, "tag %d fills %.0f%% of the frame (~%.0f in)"
                                          % (tid, w, est_distance_in(w)))
                    drv.log("tag %d at %.0f%% after braking: creeping closer" % (tid, w))
                    recheck_after, seen = None, []
                # The camera runs late: predict the width now from how fast it is growing.
                seen = [p for p in seen if p[0] > res["frame_t"] - 0.6] + [(res["frame_t"], w)]
                rate = ((seen[-1][1] - seen[0][1]) / (seen[-1][0] - seen[0][0])
                        if len(seen) > 2 and seen[-1][0] > seen[0][0] else 0.0)
                predicted = w + max(0.0, rate) * TOWARD_LEAD_S
                if predicted >= width_pct:
                    drv.brake()
                    driving, stall = False, None
                    drv.log("tag %d at %.0f%% (predicted %.0f%%): braked, checking"
                            % (tid, w, predicted))
                    recheck_after = time.monotonic() + RECHECK_S
                    continue
                slow = predicted >= width_pct * TOWARD_SLOW_FRAC   # close: creep and pivot
            else:
                slow = False
            left, right = (min(PWM_MAX, p + boost) if p else 0 for p in arc_pwms(b, pwm, slow))
            if (not driving or stall is None or abs(b - stall[1]) >= STALL_DEG
                    or abs(w - stall[2]) >= STALL_WIDTH_PCT):
                stall = (time.monotonic(), b, w)
            elif time.monotonic() - stall[0] > STALL_S:
                if max(left, right) >= PWM_MAX:
                    return drv.finish(False, "not moving: tag %d stayed at %+.1f deg, %.0f%% wide "
                                      "for %.1f s even at left %d right %d (stuck?)"
                                      % (tid, b, w, STALL_S, left, right))
                boost += STALL_BOOST_PWM
                left, right = (min(PWM_MAX, p + boost) if p else 0 for p in arc_pwms(b, pwm, slow))
                drv.log("not moving: boosting by %d to left %d right %d" % (boost, left, right))
                stall = (time.monotonic(), b, w)
            if not driving or abs(left - last_cmd[0]) >= 5 or abs(right - last_cmd[1]) >= 5:
                drv.log("tag %d at %+.1f deg, %.0f%% wide: left %d right %d" % (tid, b, w, left, right))
            driving, last_cmd = True, (FORWARD_PWM_SIGN * left, FORWARD_PWM_SIGN * right)
            drv.status("tag %d at %+.1f deg, %.0f%% wide (~%.0f in): left %d right %d"
                       % (tid, b, w, est_distance_in(w), left, right), max(left, right))
            err = drv.send(*last_cmd)
            if err:
                return drv.finish(False, "refused: " + err)
    except TimeoutError as e:
        return drv.finish(False, str(e))
    except Exception as e:
        return drv.finish(False, "crashed: %r" % e)


def find(drv, tag_id, direction="right", pwm=60, max_deg=360.0):
    """Look; if the tag is not in view, turn_degrees in steps a bit narrower than the
    camera's view and look again, up to max_deg. Then center on it."""
    want = "tag %d" % tag_id if tag_id is not None else "a tag"
    step = 0.6 * drv.hfov
    turned = 0.0
    while True:
        why = drv.halted()
        if why:
            return drv.finish(False, why)
        settle_until = time.monotonic() + RECHECK_S
        seen, settled = False, 0
        try:
            for res in drv.frames():            # look at 5 frames taken after the robot stopped
                if drv.halted():
                    break
                if res is None and drv.camera_lost():
                    return drv.finish(False, "no camera frames for %.0f s" % CAMERA_LOST_S)
                if res is None or (res.get("frame_t") or 0) < settle_until:
                    continue
                settled += 1
                if sight(res, tag_id, drv.hfov)[0] is not None:
                    seen = True
                    break
                if settled >= 5:
                    break
        except TimeoutError as e:
            return drv.finish(False, str(e))
        if drv.halted():
            return drv.finish(False, drv.halted())
        if seen:
            drv.log("found %s after turning %.0f deg" % (want, turned))
            return center(drv, tag_id)
        if turned + step > max_deg + 1e-6:
            return drv.finish(False, "%s not found in %.0f deg" % (want, turned))
        drv.log("%s not in view: turn %s %.0f deg" % (want, direction, step))
        drv.status("looking for %s: turned %.0f deg" % (want, turned), pwm)
        r = drv.ctx.turn_degrees(step, direction, pwm, True, lease=drv.lease)
        if not r.get("ok"):
            return drv.finish(False, "turn refused: %s" % (r.get("error") or r.get("abort_reason")
                                                         or "timeout"))
        turned += step


class PageLease:
    """The drive lease, held for as long as a page keeps pinging."""

    def __init__(self, ctx, holder, log, on_quiet):
        self.ctx, self.holder, self.log, self.on_quiet = ctx, holder, log, on_quiet
        self.lock = threading.Lock()
        self.token = None
        self.error = ""
        self.last_ping = 0.0

    def ping(self, who):
        with self.lock:
            self.last_ping = time.monotonic()
            tok = self.token
        if tok is not None and self.ctx.LEASE.renew(tok)[0]:
            return
        ok, info = self.ctx.LEASE.take(self.holder, who, purpose="%s page open" % self.holder,
                                       ttl_s=LEASE_TTL_S)
        with self.lock:
            self.token = info["lease"] if ok else None
            self.error = "" if ok else info.get("error", "lease refused")
        if ok:
            self.log("took the drive lease")

    def watchdog(self):
        """Brake (via on_quiet) and release once the page has gone quiet. True if fired."""
        with self.lock:
            quiet = self.token is not None and time.monotonic() - self.last_ping > PAGE_TIMEOUT_S
        if not quiet:
            return False
        self.on_quiet()
        with self.lock:
            tok, self.token = self.token, None
        self.ctx.LEASE.release(tok)
        self.log("page closed: braked and released the drive lease")
        return True

    def keep_alive(self):
        tok = self.token
        if tok is not None:
            self.ctx.LEASE.renew(tok)
