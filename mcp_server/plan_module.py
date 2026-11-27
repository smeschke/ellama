#!/usr/bin/env python3
"""plan module: stack commands on a page and run them top to bottom.

Registered by register(mcp, ctx) from robot_server. Page: GET /plan on the daemon,
linked from the operator page's Modules card.

Commands (each forward/back/turn/find/toward step has its own PWM):
  forward N in, back N in     drive_distance (encoders)
  turn left|right N deg       turn_degrees (IMU)
  aim TAG                     spin onto the tag (vision_drive.center, as on the tag turn page)
  find TAG [left|right]       turn in steps until the tag shows (up to 360 deg), then aim
  toward TAG                  arc toward the tag, never spinning, until it fills W% of the
                              frame width and/or a front ToF reads N mm (first one wins)
  wait N s
TAG blank = the biggest tag in view.

Repeat: Run can loop the whole list N times (1..100), or until Stop (repeat 0), for a
plan that brings the robot back to where it started. Each lap starts from step 1; any
failed step, Stop, or the page closing ends the loop. Nothing checks that the robot is
really back at the start, so encoder/IMU drift adds up lap by lap: aim/find steps on a
tag at the start of the list re-anchor each lap.

Record (tick "record this run"): Run also starts a telemetry recording,
recordings/plan_<time>/, the same CSV as start_recording() (encoders, IMU, ToF, drive
commands) with a MARK row at the start and end of every step and lap, plus plan.json (the
steps, repeat, who ran it, and how it ended). Tick "with camera video" too for video.avi:
the frames the plan drove by, tags outlined, at VIDEO_FPS in real time (a frame is repeated
while the camera is slower than that and dropped while it is faster, so the video plays at
real speed and stays in step with the CSV), and frames.csv
(time_s on the CSV's clock, the wall-clock time, and the tags seen in each frame). Every
frame has its capture time burned into the picture, wall clock to the millisecond plus
the CSV's time_s, so the video can be lined up with another camera's recording later;
plan.json has csv_t0_unix_s, the wall-clock time of the CSV's time_s = 0. The recording stops when the
plan ends, however it ends. Unticked, nothing is saved.

Drive lease: held while the page is open (vision_drive.PageLease, holder "plan").
Run is the go-ahead for the whole list; the plan is checked in full before anything
moves, and stops at the first step that fails. Stop latches the operator stop (the only
thing that interrupts a blocking drive_distance/turn_degrees), so Resume on the operator
page afterwards.
"""

import csv
import json
import sys
import threading
import time
from pathlib import Path

from starlette.requests import Request
from starlette.responses import HTMLResponse, JSONResponse

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vision_drive as vd  # noqa: E402

HOLDER = "plan"
MAX_STEPS = 50
MAX_LAPS = 100
VIDEO_FPS = 30             # above the ~21 fps the robot camera gives, so few frames drop
VIDEO_MAX_REPEAT_S = 5.0    # pad at most this much per frame; a longer gap is made up later


def video_copies(t, written, fps=VIDEO_FPS, max_repeat_s=VIDEO_MAX_REPEAT_S):
    """How many times to write a frame captured at time_s t into a constant-rate video
    that already holds `written` frames. Video frame k covers t = k/fps. 0 = drop it (the
    video is already past t: the camera is faster than fps)."""
    want = int(max(t, 0.0) * fps) + 1          # frames needed to reach t
    return max(0, min(want - written, int(max_repeat_s * fps)))
DEFAULT_PWM = {"forward": 50, "back": 50, "turn": 60, "find": 60, "toward": 60}


def wall_stamp(unix_s):
    """'2026-11-26 14:03:12.345' in local time."""
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(unix_s)) + ".%03d" % (
        int(unix_s * 1000) % 1000)


def burn_stamp(img, text):
    """Write text into the bottom-left corner of a BGR frame, white on a black box."""
    import cv2
    h, w = img.shape[:2]
    scale = max(0.4, w / 1100.0)
    thick = max(1, int(round(scale * 2)))
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    pad = int(6 * scale) + 2
    cv2.rectangle(img, (0, h - th - base - 2 * pad), (tw + 2 * pad, h), (0, 0, 0), -1)
    cv2.putText(img, text, (pad, h - base - pad), cv2.FONT_HERSHEY_SIMPLEX, scale,
                (255, 255, 255), thick, cv2.LINE_AA)
    return img


def _num(step, key, lo, hi, default=None, required=True, kind=float):
    v = step.get(key, default)
    if v is None or v == "":
        if required:
            raise ValueError("%s is required" % key)
        return None
    try:
        v = kind(v)
    except (TypeError, ValueError):
        raise ValueError("%s must be a number, got %r" % (key, v))
    if not lo <= v <= hi:
        raise ValueError("%s=%g is outside %g..%g" % (key, v, lo, hi))
    return v


def _tag(step):
    return _num(step, "tag", 0, 586, required=False, kind=int)   # tag36h11 has ids 0..586


def check_step(step):
    """A clean copy of one step, or ValueError saying what is wrong with it."""
    if not isinstance(step, dict):
        raise ValueError("a step must be an object")
    cmd = step.get("cmd")
    out = {"cmd": cmd}
    if cmd in ("forward", "back"):
        out["inches"] = _num(step, "inches", 0.5, 240)
    elif cmd == "turn":
        if step.get("direction") not in ("left", "right"):
            raise ValueError("direction must be left or right")
        out["direction"] = step["direction"]
        out["degrees"] = _num(step, "degrees", 1, 360)
    elif cmd == "aim":
        out["tag"] = _tag(step)
    elif cmd == "find":
        out["tag"] = _tag(step)
        out["direction"] = step.get("direction") or "right"
        if out["direction"] not in ("left", "right"):
            raise ValueError("direction must be left or right")
    elif cmd == "toward":
        out["tag"] = _tag(step)
        out["width_pct"] = _num(step, "width_pct", 2, 95, required=False)
        out["tof_mm"] = _num(step, "tof_mm", 30, 3000, required=False)
        if out["width_pct"] is None and out["tof_mm"] is None:
            raise ValueError("give a frame width %, a ToF mm, or both, to stop at")
    elif cmd == "wait":
        out["seconds"] = _num(step, "seconds", 0.1, 120)
    else:
        raise ValueError("unknown command %r" % (cmd,))
    if cmd in DEFAULT_PWM:
        out["pwm"] = _num(step, "pwm", vd.PWM_MIN, vd.PWM_MAX, default=DEFAULT_PWM[cmd], kind=int)
    return out


def check_plan(steps):
    """(clean steps, problems). Nothing runs unless problems is empty."""
    if not isinstance(steps, list) or not steps:
        return [], ["the plan is empty"]
    if len(steps) > MAX_STEPS:
        return [], ["at most %d steps" % MAX_STEPS]
    clean, problems = [], []
    for i, s in enumerate(steps, 1):
        try:
            clean.append(check_step(s))
        except ValueError as e:
            problems.append("step %d: %s" % (i, e))
    return clean, problems


def check_repeat(v):
    """Laps to run: 1..MAX_LAPS, or 0 = until Stop. Blank/missing = 1."""
    if v is None or v == "":
        return 1
    if isinstance(v, bool):
        raise ValueError("repeat must be a whole number")
    try:
        n = int(v)
    except (TypeError, ValueError):
        raise ValueError("repeat must be a whole number, got %r" % (v,))
    if n != v and str(n) != str(v).strip():
        raise ValueError("repeat must be a whole number, got %r" % (v,))
    if not 0 <= n <= MAX_LAPS:
        raise ValueError("repeat=%d is outside 0..%d (0 = until Stop)" % (n, MAX_LAPS))
    return n


def laps_text(lap, laps):
    return "lap %d of %s" % (lap, laps if laps else "until Stop")


def describe(s):
    tag = "tag %d" % s["tag"] if s.get("tag") is not None else "biggest tag"
    c = s["cmd"]
    if c in ("forward", "back"):
        return "%s %g in (pwm %d)" % (c, s["inches"], s["pwm"])
    if c == "turn":
        return "turn %s %g deg (pwm %d)" % (s["direction"], s["degrees"], s["pwm"])
    if c == "aim":
        return "aim at %s" % tag
    if c == "find":
        return "find %s, turning %s (pwm %d)" % (tag, s["direction"], s["pwm"])
    if c == "toward":
        stops = []
        if s["width_pct"] is not None:
            stops.append("%g%% wide (~%.0f in)" % (s["width_pct"], vd.est_distance_in(s["width_pct"])))
        if s["tof_mm"] is not None:
            stops.append("ToF %g mm" % s["tof_mm"])
        return "toward %s until %s (pwm %d)" % (tag, " or ".join(stops), s["pwm"])
    return "wait %g s" % s["seconds"]


class RunRecording:
    """One plan run's recording: the link's telemetry Recorder, plus video if asked."""

    def __init__(self, link, view, video):
        self.link, self.view, self.video = link, view, video
        self.rec = link.recorder
        self.stop_evt = threading.Event()
        self.thread = None
        self.frames = self.written = self.dropped = 0
        self.video_error = ""
        name = time.strftime("plan_%Y%m%d_%H%M%S")
        self.csv_path = self.rec.start(name)      # raises if already recording
        self.dir = Path(self.rec.dir)
        if video:
            self.thread = threading.Thread(target=self._video, name="plan-video", daemon=True)
            self.thread.start()

    def mark(self, text):
        try:
            with self.link.lock:
                self.rec.mark(self.link.enc, self.link.orient, "MARK", text)
        except Exception:
            pass    # recording must never break driving

    def write_json(self, info):
        try:
            (self.dir / "plan.json").write_text(json.dumps(info, indent=1))
        except Exception:
            pass

    def _video(self):
        import cv2
        import numpy as np
        out = None
        try:
            with open(self.dir / "frames.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["frame", "time_s", "unix_s", "wall_time", "video_frame", "tags"])
                seq = 0
                while not self.stop_evt.is_set():
                    self.view.touch()
                    res = self.view.wait_new(seq, 0.5)
                    if res is None or res.get("jpg") is None:
                        continue
                    seq = res["seq"]
                    img = cv2.imdecode(np.frombuffer(res["jpg"], np.uint8), cv2.IMREAD_COLOR)
                    if img is None:
                        continue
                    if out is None:
                        h, wd = img.shape[:2]
                        out = cv2.VideoWriter(str(self.dir / "video.avi"),
                                              cv2.VideoWriter_fourcc(*"MJPG"), VIDEO_FPS, (wd, h))
                        if not out.isOpened():
                            raise RuntimeError("could not open video.avi for writing")
                        size = (wd, h)
                    if img.shape[1::-1] != size:
                        img = cv2.resize(img, size)
                    ago = time.monotonic() - res.get("frame_t", time.monotonic())
                    t = self.rec.now() - ago
                    unix = time.time() - ago           # when the frame arrived, wall clock
                    burn_stamp(img, "%s  t=%.3f s" % (wall_stamp(unix), t))
                    # Real time: repeat this frame up to t, or drop it if the video is past t.
                    n = video_copies(t, self.written)
                    for _ in range(n):
                        out.write(img)
                    self.frames += 1
                    self.dropped += n == 0
                    tags = ";".join("%d@%.0f,%.0f" % (tid, c[0], c[1])
                                    for tid, c, *_ in res.get("tags") or [])
                    w.writerow([self.frames, "%.3f" % t, "%.3f" % unix, wall_stamp(unix),
                                self.written if n else "", tags])   # blank = not in the video
                    self.written += n
        except Exception as e:
            self.video_error = "video stopped: %s" % e
        finally:
            if out is not None:
                out.release()

    def finish(self):
        self.stop_evt.set()
        if self.thread is not None:
            self.thread.join(5.0)
        summary = self.rec.stop() or {}
        out = {"dir": str(self.dir), "csv": str(self.csv_path),
               "duration_s": summary.get("duration_s"), "markers": summary.get("markers")}
        if self.video:
            out.update(video=str(self.dir / "video.avi") if self.frames else None,
                       camera_frames=self.frames, video_frames=self.written,
                       dropped_frames=self.dropped, video_fps=VIDEO_FPS,
                       video_error=self.video_error or (None if self.frames else
                                                        "no camera frames arrived"))
        return out


class Plan:
    def __init__(self, ctx, vision=None):
        self.ctx = ctx
        self._vision = vision
        self.lock = threading.Lock()
        self.thread = None
        self.stop_evt = threading.Event()
        self.phase = "idle"
        self.note = "build a plan, then press Run"
        self.steps = []         # [{"text", "state", "result"}] for the run in progress / last run
        self.current = None
        self.lap, self.laps = 0, 1  # laps 0 = until Stop
        self.recording = None       # what the last recorded run saved (dict), else None
        self._rec = None
        self.log = []
        self.page = vd.PageLease(ctx, HOLDER, self._log, on_quiet=self._quiet)

    @property
    def view(self):
        return self._vision if self._vision is not None else vd._vision()

    @property
    def lease(self):
        return self.page.token

    def _log(self, text):
        with self.lock:
            self.log.append(time.strftime("%H:%M:%S ") + text)
            self.log = self.log[-100:]

    def _set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def ping(self, who, cam=None):
        self.view.use(cam or None)
        self.page.ping(who or "plan page")

    def watchdog(self):
        return self.page.watchdog()

    def _brake(self):
        try:
            if self.ctx.link is not None:
                self.ctx.link.send_stop()
        except Exception:
            pass

    def _quiet(self):
        if self.thread is not None and self.thread.is_alive():
            self.stop(who="page closed")
            self.thread.join(5.0)
        self._brake()

    def run(self, steps, who, repeat=1, record=False, video=False):
        clean, problems = check_plan(steps)
        try:
            laps = check_repeat(repeat)
        except ValueError as e:
            problems = problems + [str(e)]
        if problems:
            return 400, {"ok": False, "error": "; ".join(problems), "problems": problems}
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                return 409, {"ok": False, "error": "a plan is already running"}
        if self.lease is None:
            return 409, {"ok": False, "error": "this page does not hold the drive lease: "
                         + (self.page.error or "open the page first")}
        if self.ctx.link is None:
            return 409, {"ok": False, "error": "no robot connected"}
        if self.ctx.OPSTOP.latched:
            return 409, {"ok": False, "error": "operator stop is latched: Resume on the operator page"}
        rec = None
        if record or video:
            if video and not self.view.status().get("address"):
                return 409, {"ok": False, "error": "enter the camera address to record video"}
            try:
                rec = RunRecording(self.ctx.link, self.view, bool(video))
            except Exception as e:
                return 409, {"ok": False, "error": "could not start recording: %s" % e}
            rec.write_json({"who": who, "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                            "csv_t0_unix_s": round(time.time() - rec.rec.now(), 3),
                            "repeat": laps, "steps": clean, "video": bool(video)})
        self.stop_evt.clear()
        with self.lock:
            self._rec = rec
            self.recording = {"dir": str(rec.dir), "recording": True} if rec else None
            self.steps = [{"text": describe(s), "state": "pending", "result": ""} for s in clean]
            self.phase, self.note, self.current = "running", "starting", None
            self.lap, self.laps = 0, laps
            self.thread = threading.Thread(target=self._run, args=(clean, who, laps),
                                           name="plan", daemon=True)
            self.thread.start()
        return 200, {"ok": True, "steps": len(clean), "repeat": laps,
                     "recording": str(rec.dir) if rec else None}

    def stop(self, who):
        """Latch the operator stop: the only thing that cuts a blocking move short."""
        self.stop_evt.set()
        try:
            self.ctx.operator_stop(who, "plan Stop")
        except Exception:
            self._brake()
        self._log("Stop by %s (operator stop latched: Resume on the operator page)" % who)
        return 200, {"ok": True}

    def _mark(self, i, state, result=""):
        with self.lock:
            self.steps[i].update(state=state, result=result)

    def _mark_rec(self, text):
        if self._rec is not None:
            self._rec.mark(text)

    def _run(self, steps, who, laps=1):
        try:
            self._run_laps(steps, who, laps)
        finally:
            rec, self._rec = self._rec, None
            if rec is not None:
                self._brake()
                with self.lock:
                    phase, note, lap = self.phase, self.note, self.lap
                rec.mark("plan end: %s: %s" % (phase, note))
                summary = rec.finish()
                try:
                    info = json.loads((rec.dir / "plan.json").read_text())
                except Exception:
                    info = {}
                info.update(ended=time.strftime("%Y-%m-%d %H:%M:%S"), phase=phase, note=note,
                            laps_run=lap, steps_state=[dict(x) for x in self.steps], **summary)
                rec.write_json(info)
                self._set(recording=summary)
                self._log("recorded to %s%s" % (summary["dir"], (
                    " (%s)" % summary["video_error"]) if summary.get("video_error") else ""))

    def _run_laps(self, steps, who, laps):
        self._log("Run by %s: %d steps, %s" % (
            who, len(steps), "until Stop" if not laps else "%d lap%s" % (laps, "" if laps == 1 else "s")))
        drv = vd.Driver(self.ctx, self.lease, self.stop_evt, self._log, view=self.view,
                        status=lambda note, pwm=0: self._set(note=note))
        failed = None
        lap = 0
        while failed is None and (not laps or lap < laps):
            lap += 1
            why = drv.halted()
            if why:
                failed = (0, why)
                break
            with self.lock:
                self.lap = lap
                if lap > 1:
                    for st in self.steps:
                        st.update(state="pending", result="")
            if laps != 1:
                self._log(laps_text(lap, laps))
            self._mark_rec("lap %d start" % lap)
            failed = self._lap(drv, steps)
            self._mark_rec("lap %d %s" % (lap, "end" if failed is None else "failed"))
        self._brake()
        where = laps_text(lap, laps) + ", " if laps != 1 else ""
        if failed is None:
            self._set(phase="done", current=None,
                      note="plan finished" + (" (%d laps)" % lap if laps != 1 else ""))
        else:
            i, why = failed
            with self.lock:
                for st in self.steps[i:]:
                    if st["state"] == "pending":
                        st["state"] = "skipped"
            self._set(phase="stopped" if self.stop_evt.is_set() else "failed",
                      note="%sstep %d: %s" % (where, i + 1, why), current=None)

    def _lap(self, drv, steps):
        """Run the list once. None, or (index, why) for the step that failed."""
        for i, s in enumerate(steps):
            why = drv.halted()
            if why:
                return (i, why)
            self._set(current=i, note=describe(s))
            self._mark(i, "running")
            self._log("step %d: %s" % (i + 1, describe(s)))
            self._mark_rec("lap %d step %d start: %s" % (self.lap, i + 1, describe(s)))
            try:
                ok, result = self._step(drv, s)
            except Exception as e:  # a bug must still brake
                self._brake()
                ok, result = False, "crashed: %r" % e
            self._mark(i, "done" if ok else "failed", result)
            self._log("  -> %s" % result)
            self._mark_rec("lap %d step %d %s: %s" % (self.lap, i + 1, "done" if ok else "failed", result))
            if not ok:
                return (i, result)
        return None

    def _step(self, drv, s):
        ctx, lease, c = self.ctx, self.lease, s["cmd"]
        if c in ("forward", "back"):
            r = ctx.drive_distance(s["inches"], "forward" if c == "forward" else "reverse",
                                   s["pwm"], True, lease=lease)
            got = r.get("actual_inches_traveled")
            if not r.get("ok"):
                return False, "refused: %s" % (r.get("error") or r.get("abort_reason") or "timeout")
            return True, "went %s in" % (abs(got) if isinstance(got, (int, float)) else got)
        if c == "turn":
            r = ctx.turn_degrees(s["degrees"], s["direction"], s["pwm"], True, lease=lease)
            got = r.get("actual_degrees_turned")
            if not r.get("ok"):
                return False, "refused: %s" % (r.get("error") or r.get("abort_reason") or "timeout")
            return True, "turned %s deg" % (abs(got) if isinstance(got, (int, float)) else got)
        if c == "aim":
            return vd.center(drv, s["tag"])
        if c == "find":
            return vd.find(drv, s["tag"], s["direction"], s["pwm"])
        if c == "toward":
            return vd.toward(drv, s["tag"], s["width_pct"], s["tof_mm"], s["pwm"])
        end = time.monotonic() + s["seconds"]
        while time.monotonic() < end:
            why = drv.halted()
            if why:
                return False, why
            self.page.keep_alive()
            time.sleep(0.1)
        return True, "waited %g s" % s["seconds"]

    def status(self):
        hfov, src = vd.hfov_deg()
        with self.lock:
            out = {"phase": self.phase, "note": self.note, "current": self.current,
                   "lap": self.lap, "laps": self.laps, "recording": self.recording,
                   "steps": [dict(s) for s in self.steps], "log": list(self.log),
                   "hfov_deg": hfov, "hfov_source": src,
                   "ref": {"width_pct": vd.REF_WIDTH_PCT, "distance_in": vd.REF_DISTANCE_IN}}
        out["holding_lease"] = self.lease is not None
        out["lease_error"] = self.page.error
        out["operator_stop"] = self.ctx.OPSTOP.latched
        out["vision"] = self.view.status()
        return out


SESSION = None


def _watchdog_loop():
    while True:
        try:
            if SESSION is not None:
                SESSION.watchdog()
        except Exception:
            pass
        time.sleep(0.2)


def plan_status() -> dict:
    """Read-only: the plan page's run (each step's state and result, the lap it is on and
    laps asked for, 0 = until Stop), where a recorded run was saved, its lease and log.
    Never drives; plans are built and run from the /plan page, which holds the lease."""
    return SESSION.status()


def register(mcp, ctx):
    global SESSION
    SESSION = Plan(ctx)
    mcp.tool()(plan_status)
    threading.Thread(target=_watchdog_loop, name="plan-watchdog", daemon=True).start()

    def who_of(request):
        return "plan page (%s)" % (request.client.host if request.client else "?")

    async def body_of(request):
        import json
        try:
            b = json.loads((await request.body()).decode() or "{}")
            return b if isinstance(b, dict) else {}
        except Exception:
            return {}

    @mcp.custom_route("/plan", methods=["GET"], include_in_schema=False)
    async def page(request: Request):
        return HTMLResponse(PAGE)

    @mcp.custom_route("/plan/state", methods=["GET"], include_in_schema=False)
    async def state(request: Request):
        SESSION.ping(who_of(request), request.query_params.get("cam"))
        return JSONResponse(SESSION.status())

    @mcp.custom_route("/plan/check", methods=["POST"], include_in_schema=False)
    async def check(request: Request):
        b = await body_of(request)
        clean, problems = check_plan(b.get("steps"))
        try:
            check_repeat(b.get("repeat"))
        except ValueError as e:
            problems = problems + [str(e)]
        return JSONResponse({"ok": not problems, "problems": problems,
                             "steps": [describe(s) for s in clean] if not problems else []})

    @mcp.custom_route("/plan/run", methods=["POST"], include_in_schema=False)
    async def run(request: Request):
        b = await body_of(request)
        code, out = SESSION.run(b.get("steps"), who_of(request), b.get("repeat"),
                                bool(b.get("record")), bool(b.get("video")))
        return JSONResponse(out, status_code=code)

    @mcp.custom_route("/plan/stop", methods=["POST"], include_in_schema=False)
    async def stop(request: Request):
        code, out = SESSION.stop(who_of(request))
        return JSONResponse(out, status_code=code)


# Raw string so JavaScript escapes reach the browser unchanged.
PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>eLlama plan</title>
<style>body{font-family:system-ui,sans-serif;margin:12px;background:#161616;color:#e8e8e8}
h1{font-size:1.2em}h2{font-size:1em;margin:16px 0 6px}
.cols{display:flex;flex-wrap:wrap;gap:18px;align-items:flex-start}
.view{position:relative;width:min(92vw,480px)}
.view img{display:block;width:100%;min-height:100px;border:1px solid #3a3a3a;border-radius:8px;background:#1e1e1e}
.mid{position:absolute;top:0;bottom:0;left:50%;border-left:2px dashed #5aa0ff;opacity:.6}
.panel{flex:1;min-width:300px;max-width:640px}
input,select{font:inherit;padding:5px 8px;border-radius:7px;border:1px solid #3a3a3a;background:#1e1e1e;color:#e8e8e8}
input.n{width:5.5em}
button{font:inherit;padding:7px 14px;margin:3px;border-radius:7px;border:0;color:#fff;background:#3b4a5e;cursor:pointer}
button:disabled{background:#333;color:#777;cursor:default}
#run{background:#2e6b3a;padding:10px 26px}#stop{background:#c62828;padding:10px 26px}
#add{background:#2f6fd0}
ol{padding-left:1.6em;margin:6px 0}li{padding:5px 4px;border-bottom:1px solid #2c2c2c}
li .t{display:inline-block;min-width:16em}li button{padding:2px 9px;margin:0 2px;background:#2c2c2c}
li.running{background:#24364d}li.done .r{color:#7fd08a}li.failed .r,li.failed .t{color:#ff8a80}li.skipped{opacity:.5}
.r{color:#9a9a9a;margin-left:8px}.muted,#lease,#s{color:#9a9a9a}.err{color:#ff8a80}
.fields label{margin-right:10px;white-space:nowrap}
pre{white-space:pre-wrap;color:#c8c8c8;max-height:14em;overflow:auto}</style></head><body>
<h1>Plan: stack commands, then run them top to bottom</h1>
<p>Camera: <input id=cam size=22 placeholder="192.168.0.52:4444"> <span id=lease>checking the drive lease&hellip;</span></p>
<div class=cols>
 <div class=view><img id=f alt="camera (set the address above)"><div class=mid></div>
  <p id=b class=muted>&nbsp;</p></div>
 <div class=panel>
  <h2>Add a step</h2>
  <p><select id=cmd>
    <option value=forward>forward</option><option value=back>back</option>
    <option value=turn>turn</option><option value=aim>aim at tag</option>
    <option value=find>find tag</option><option value=toward>drive toward tag</option>
    <option value=wait>wait</option></select>
   <span id=fields class=fields></span> <button id=add>Add</button></p>
  <p id=hint class=muted></p>
  <h2>Plan</h2>
  <ol id=plan></ol>
  <p id=problems class=err></p>
  <p class=fields><label>repeat <input id=laps class=n value=1 inputmode=numeric> times</label>
   <label><input type=checkbox id=forever> until Stop</label>
   <span class=muted>(for a plan that ends where it started)</span></p>
  <p class=fields><label><input type=checkbox id=record> record this run</label>
   <label><input type=checkbox id=video> with camera video</label></p>
  <p id=rec class=muted></p>
  <p><button id=run>Run</button> <button id=stop>Stop</button> <button id=clear>Clear plan</button></p>
  <p id=s>&nbsp;</p>
 </div>
</div>
<h2>Log</h2>
<pre id=log>(nothing yet)</pre>
<script>
const $ = id => document.getElementById(id);
const PWM = {forward: 50, back: 50, turn: 60, find: 60, toward: 60};
// [key, label, default, kind]; kind: num | dir | tag
const SPEC = {
  forward: [['inches', 'inches', 12, 'num'], ['pwm', 'pwm', 50, 'num']],
  back:    [['inches', 'inches', 12, 'num'], ['pwm', 'pwm', 50, 'num']],
  turn:    [['direction', '', 'right', 'dir'], ['degrees', 'degrees', 90, 'num'], ['pwm', 'pwm', 60, 'num']],
  aim:     [['tag', 'tag id (blank = biggest)', '', 'tag']],
  find:    [['tag', 'tag id', '', 'tag'], ['direction', 'turning', 'right', 'dir'], ['pwm', 'pwm', 60, 'num']],
  toward:  [['tag', 'tag id', '', 'tag'], ['width_pct', 'stop at frame width %', 29, 'num'],
            ['tof_mm', 'or ToF mm (blank = off)', '', 'num'], ['pwm', 'pwm', 60, 'num']],
  wait:    [['seconds', 'seconds', 2, 'num']],
};
let REF = {width_pct: 29, distance_in: 12};
const HINT = {
  forward: 'Drives straight by the wheel encoders.',
  back: 'Backs up straight by the wheel encoders.',
  turn: 'Turns in place by the IMU.',
  aim: 'Spins onto the tag (same as the tag turn page). Fails if it is not in view.',
  find: 'Looks; if the tag is not in view, turns in steps (up to a full circle) until it is, then aims.',
  toward: 'Curves toward the tag without spinning, stopping when it fills that % of the frame width (and/or at the ToF distance).',
  wait: 'Pauses.',
};
let plan = [];
try { plan = JSON.parse(localStorage.getItem('ellama.plan') || '[]') || []; } catch (e) { plan = []; }
try { const c = localStorage.getItem('ellama.apriltag.cam'); if (c) $('cam').value = c; } catch (e) {}
$('cam').addEventListener('change', () => {
  try { localStorage.setItem('ellama.apriltag.cam', $('cam').value); } catch (e) {} });
const save = () => { try { localStorage.setItem('ellama.plan', JSON.stringify(plan)); } catch (e) {} };
try { const r = JSON.parse(localStorage.getItem('ellama.plan.repeat') || 'null');
      if (r) { $('laps').value = r.laps; $('forever').checked = !!r.forever; } } catch (e) {}
function repeat() { return $('forever').checked ? 0 : ($('laps').value.trim() === '' ? 1 : Number($('laps').value)); }
function repeatChanged() {
  $('laps').disabled = $('forever').checked;
  try { localStorage.setItem('ellama.plan.repeat', JSON.stringify({laps: $('laps').value, forever: $('forever').checked})); } catch (e) {}
  check();
}
$('laps').addEventListener('input', repeatChanged);
$('forever').addEventListener('change', repeatChanged);
$('laps').disabled = $('forever').checked;
try { const r = JSON.parse(localStorage.getItem('ellama.plan.record') || 'null');
      if (r) { $('record').checked = !!r.record; $('video').checked = !!r.video; } } catch (e) {}
function recordChanged() {
  if (!$('record').checked) $('video').checked = false;
  $('video').disabled = !$('record').checked;
  try { localStorage.setItem('ellama.plan.record', JSON.stringify({record: $('record').checked, video: $('video').checked})); } catch (e) {}
}
$('record').addEventListener('change', recordChanged);
$('video').addEventListener('change', recordChanged);
recordChanged();
function estIn(pct) { return pct > 0 ? REF.distance_in * REF.width_pct / pct : null; }

function renderFields() {
  const cmd = $('cmd').value;
  $('fields').innerHTML = '';
  for (const [key, label, def, kind] of SPEC[cmd]) {
    const lab = document.createElement('label');
    let el;
    if (kind === 'dir') {
      el = document.createElement('select');
      for (const d of ['right', 'left']) { const o = document.createElement('option'); o.value = o.textContent = d; el.appendChild(o); }
      el.value = def;
    } else {
      el = document.createElement('input'); el.className = 'n'; el.value = def; el.inputMode = 'decimal';
    }
    el.dataset.key = key;
    if (label) lab.append(label + ' ');
    lab.append(el);
    $('fields').append(lab);
    if (key === 'width_pct') {
      const est = document.createElement('span'); est.className = 'muted';
      const upd = () => { const v = parseFloat(el.value); est.textContent = v > 0 ? ' (~' + estIn(v).toFixed(0) + ' in)' : ''; };
      el.addEventListener('input', upd); upd(); lab.append(est);
    }
  }
  $('hint').textContent = HINT[cmd];
}
function readFields() {
  const step = {cmd: $('cmd').value};
  for (const el of $('fields').querySelectorAll('[data-key]')) {
    const v = el.value.trim();
    step[el.dataset.key] = el.tagName === 'SELECT' ? v : (v === '' ? null : Number(v));
  }
  return step;
}
function text(s) {
  const tag = (s.tag === null || s.tag === undefined) ? 'biggest tag' : 'tag ' + s.tag;
  switch (s.cmd) {
    case 'forward': case 'back': return s.cmd + ' ' + s.inches + ' in (pwm ' + s.pwm + ')';
    case 'turn': return 'turn ' + s.direction + ' ' + s.degrees + '° (pwm ' + s.pwm + ')';
    case 'aim': return 'aim at ' + tag;
    case 'find': return 'find ' + tag + ', turning ' + s.direction + ' (pwm ' + s.pwm + ')';
    case 'toward': {
      const stops = [];
      if (s.width_pct !== null) stops.push(s.width_pct + '% wide (~' + estIn(s.width_pct).toFixed(0) + ' in)');
      if (s.tof_mm !== null) stops.push('ToF ' + s.tof_mm + ' mm');
      return 'toward ' + tag + ' until ' + stops.join(' or ') + ' (pwm ' + s.pwm + ')';
    }
    case 'wait': return 'wait ' + s.seconds + ' s';
  }
  return JSON.stringify(s);
}
let lastRun = null;   // server's step states while/after running
function renderPlan() {
  const ol = $('plan'); ol.innerHTML = '';
  plan.forEach((s, i) => {
    const li = document.createElement('li');
    const run = lastRun && lastRun[i];
    if (run) li.className = run.state;
    const t = document.createElement('span'); t.className = 't'; t.textContent = text(s);
    li.append(t);
    const mk = (label, fn, title) => { const b = document.createElement('button'); b.textContent = label; b.title = title; b.onclick = fn; li.append(b); };
    mk('↑', () => { if (i > 0) { [plan[i - 1], plan[i]] = [plan[i], plan[i - 1]]; changed(); } }, 'move up');
    mk('↓', () => { if (i < plan.length - 1) { [plan[i + 1], plan[i]] = [plan[i], plan[i + 1]]; changed(); } }, 'move down');
    mk('✕', () => { plan.splice(i, 1); changed(); }, 'delete');
    if (run && run.result) { const r = document.createElement('span'); r.className = 'r'; r.textContent = '→ ' + run.result; li.append(r); }
    ol.append(li);
  });
  if (!plan.length) ol.innerHTML = '<span class=muted>(empty: add steps above)</span>';
}
async function changed() { lastRun = null; save(); renderPlan(); await check(); }
async function post(p, b) {
  const r = await fetch(p, {method: 'POST', headers: {'content-type': 'application/json'}, body: JSON.stringify(b)});
  try { return await r.json(); } catch (e) { return {}; }
}
async function check() {
  if (!plan.length) { $('problems').textContent = ''; return; }
  const r = await post('/plan/check', {steps: plan, repeat: repeat()});
  $('problems').textContent = (r.problems || []).join('\n');
}
$('cmd').addEventListener('change', renderFields);
$('add').onclick = () => { plan.push(readFields()); changed(); };
$('clear').onclick = () => { if (plan.length) { plan = []; changed(); } };
$('run').onclick = async () => {
  const r = await post('/plan/run', {steps: plan, repeat: repeat(),
                                     record: $('record').checked, video: $('video').checked});
  if (r.ok === false) $('s').textContent = 'Run refused: ' + (r.error || '?');
};
$('stop').onclick = () => post('/plan/stop', {});

let streaming = '', running = false;
async function tick() {
  const cam = $('cam').value.trim();
  try {
    const r = await fetch('/plan/state?cam=' + encodeURIComponent(cam), {cache: 'no-store'});
    const s = await r.json();
    if (s.ref) REF = s.ref;
    $('lease').textContent = s.holding_lease ? 'This page holds the drive lease.'
      : 'No drive lease: ' + (s.lease_error || 'not taken yet');
    running = s.phase === 'running';
    $('run').disabled = !s.holding_lease || running || !plan.length || s.operator_stop;
    $('add').disabled = $('clear').disabled = $('forever').disabled = running;
    $('laps').disabled = running || $('forever').checked;
    $('record').disabled = running;
    $('video').disabled = running || !$('record').checked;
    const rc = s.recording;
    $('rec').textContent = !rc ? '' : rc.recording ? 'Recording to ' + rc.dir + ' …'
      : 'Last recording: ' + rc.dir + (rc.video ? ' (video.avi, ' + rc.camera_frames + ' camera frames)' : '')
        + (rc.video_error ? '  ' + rc.video_error : '');
    const lap = running && s.laps !== 1 ? ' (lap ' + s.lap + ' of ' + (s.laps || 'until Stop') + ')' : '';
    $('s').textContent = s.phase + lap + ': ' + s.note + (s.operator_stop ? '  (operator stop latched: Resume on the operator page)' : '');
    if (s.steps && s.steps.length && (running || lastRun)) { lastRun = s.steps; renderPlan(); }
    if (running && !lastRun) { lastRun = s.steps; }
    if (s.log && s.log.length) $('log').textContent = s.log.join('\n');
    const v = s.vision || {};
    const tags = v.tags || [];
    $('b').textContent = v.note || (tags.length ? 'in view: ' + tags.map(t => 'tag ' + t.id).join(', ') : 'no tags in view');
    if (cam && streaming !== cam) { streaming = cam; $('f').src = '/apriltag/stream.mjpg?t=' + Date.now(); }
  } catch (e) { $('s').textContent = 'daemon not answering'; }
}
$('run').addEventListener('click', () => { lastRun = plan.map(() => ({state: 'pending', result: ''})); renderPlan(); });
$('f').addEventListener('error', () => { streaming = ''; });
renderFields(); renderPlan(); check();
setInterval(tick, 500); tick();
</script></body></html>"""
