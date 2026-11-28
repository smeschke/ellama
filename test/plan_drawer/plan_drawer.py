#!/usr/bin/env python3
"""Plan drawer: draw a robot plan with the mouse, get long moves the robot does well.

    mcp_server/.venv/bin/python test/plan_drawer/plan_drawer.py
        [--name NAME] [--width-in 288] [--height-in 240] [--out FILE] [--load FILE]
        [--selftest] [--snapshot PNG]

A standalone test utility: it never talks to the daemon or the robot. You draw the
final shape on a map of the garage, and every click becomes one or two long
primitives (spin, straight, arc, tag steps) instead of thousands of stick commands.

Tools (toolbar button or key):
  s  Start pose     click = where the robot starts, drag = which way it faces
  t  Place tag      click = tag position, drag = which way the printed face points
  a  Turn to tag    click a tag: spin onto it (aim; find if it starts out of view)
  g  Go to tag      click a tag: curve toward it without spinning, stop D in short
  l  Straight fwd   click a point. Stop mode: spin to face it, then drive.
                    Smooth mode: tangent fillet arc of radius R, then drive, no stop.
  b  Straight back  the same driving backward: spin so the rear faces it, then back up
                    (smooth: a backing fillet arc, then back up).
                    Shift-click (either): straight along the current heading only
  c  Arc fwd        click a point: the forward arc tangent to the heading through it
  v  Arc back       the backing-up arc through it (drive_arc travel="reverse")
  p  Spin           click a point: turn in place to face it
  m  Stop between / Smooth   (sets stop_after on new moves, and how Straight turns)
  n  Snap           points to a 6 in grid, start/tag headings to 15 deg
  r  Name the plan  type it, Enter to keep, Esc to cancel (or click the Name button)
  u / Backspace  Undo      w  Save      o  Load (plans/<name>.json)     q / Esc  Quit
  [ ]  go-to-tag stop distance     - =  smooth fillet radius
  mouse wheel over the step list scrolls it

Frame: inches, x right, y up on the map; heading counter-clockwise, so "left" = CCW,
the same meaning as the plan page's turn left.

Saved JSON (test/plan_drawer/plans/<name>.json, or --out): name, start, tags, intents (the
drawing, so it reloads) and steps (plan-page step dicts). NOTE: the plan page cannot run
these yet: it has no "arc" command (drive_arc exists in drive_tools.py) and ignores
"stop_after"; a step with stop_after false is meant to flow into the next one.
"""

import argparse
import copy
import json
import math
import re
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
PLANS_DIR = HERE / "plans"
DEFAULT_NAME = "drawing"
NAME_MAX = 40


def file_stem(name):
    """A plan name -> a safe file name: letters, digits, - _ . ; anything else is _."""
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", (name or "").strip()).strip("._")
    return stem[:NAME_MAX] or DEFAULT_NAME

# Plan-page defaults and drive tool limits (plan_module.DEFAULT_PWM, drive_tools.drive_arc).
PWM = {"forward": 50, "back": 50, "turn": 60, "arc": 60, "find": 60, "toward": 60}
MIN_ARC_RADIUS_IN = 6.0
MAX_ARC_DEG = 360.0
MAX_ARC_IN = 160.0
MIN_TURN_DEG = 1.0          # plan page turn minimum
MIN_DRIVE_IN = 0.5          # plan page forward/back minimum
HALF_FOV_DEG = 30.0         # vision_drive.DEFAULT_HFOV_DEG / 2
REF_WIDTH_PCT, REF_DISTANCE_IN = 29.0, 12.0   # vision_drive: 29% of the frame at 12 in
TAG_FACE_DEG = 75.0         # the camera sees a tag only within this angle of its face
TAG_RANGE_IN = 240.0
TOWARD_MIN_RADIUS_IN = 12.0
GRID_IN, SNAP_IN, SNAP_DEG = 12.0, 6.0, 15.0

TOOLBAR_W, PANEL_W, CANVAS_W, WIN_H = 200, 400, 1240, 1000
MARGIN = 24

# BGR colours
BG, CANVAS_BG, GRID, GRID_MAJOR = (24, 24, 24), (34, 34, 32), (52, 52, 50), (72, 72, 68)
FG, MUT, ACCENT = (232, 232, 228), (150, 150, 145), (210, 140, 60)
STOP_DOT, FLOW_DOT, ERR = (70, 70, 230), (110, 200, 120), (90, 110, 255)
COL = {"line": (120, 200, 120), "arc": (50, 150, 240), "spin": (230, 190, 80),
       "aim": (220, 120, 200), "find": (220, 120, 200), "toward": (80, 215, 235)}
FONT = cv2.FONT_HERSHEY_SIMPLEX


# ---- geometry -----------------------------------------------------------------

def wrap(a):
    """Radians into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def local(pose, p):
    """Point p in the robot frame: (forward, left)."""
    x, y, th = pose
    dx, dy = p[0] - x, p[1] - y
    c, s = math.cos(th), math.sin(th)
    return c * dx + s * dy, -s * dx + c * dy


def bearing(pose, p):
    """Radians from the heading to p, + = left (CCW)."""
    return wrap(math.atan2(p[1] - pose[1], p[0] - pose[0]) - pose[2])


def flip(pose):
    """The same spot facing backward: backing up is driving forward from here."""
    return pose[0], pose[1], pose[2] + math.pi


def arc_pose(pose, r, sweep, rev=False):
    """Pose after an arc of radius r, signed sweep (radians, + = heading turns left/CCW).
    rev: backing up (drive_arc travel="reverse"): the heading still turns by sweep."""
    if rev:
        e = arc_pose(flip(pose), r, sweep)
        return e[0], e[1], e[2] - math.pi
    x, y, th = pose
    sgn = 1.0 if sweep >= 0 else -1.0
    cx, cy = x - sgn * r * math.sin(th), y + sgn * r * math.cos(th)
    a = th - sgn * math.pi / 2 + sweep
    return cx + r * math.cos(a), cy + r * math.sin(a), th + sweep


def tag_visible(pose, tag):
    """None, or why the camera would not see this tag from pose."""
    d = math.hypot(tag["x"] - pose[0], tag["y"] - pose[1])
    if d > TAG_RANGE_IN:
        return "tag %d is %.0f in away" % (tag["id"], d)
    face = math.radians(tag["face"])
    to_robot = math.atan2(pose[1] - tag["y"], pose[0] - tag["x"])
    if abs(math.degrees(wrap(to_robot - face))) > TAG_FACE_DEG:
        return "robot is behind / beside tag %d (its face points away)" % tag["id"]
    return None


def width_pct_for(dist_in):
    return REF_DISTANCE_IN * REF_WIDTH_PCT / dist_in


# ---- primitives: what the robot actually does ----------------------------------
# {"type": "spin", "deg"}            turn in place, + = left
# {"type": "line", "in"}             straight, - = back
# {"type": "arc", "r", "deg", "rev"} arc, + = heading turns left; rev = backing up
# {"type": "aim"|"find", "tag", "deg", "dir"}
# {"type": "toward", "tag", "stop_in", "pts", "end"}
# each also carries "stop" (stop after it) and maybe "warn"

def sim_prim(pose, p):
    """(polyline of (x, y), end pose)."""
    t = p["type"]
    if t in ("spin", "aim", "find"):
        return [pose[:2]], (pose[0], pose[1], pose[2] + math.radians(p["deg"]))
    if t == "line":
        x, y, th = pose
        e = (x + p["in"] * math.cos(th), y + p["in"] * math.sin(th), th)
        return [(x, y), e[:2]], e
    if t == "arc":
        sweep = math.radians(p["deg"])
        rev = p.get("rev", False)
        n = max(2, int(abs(sweep) * p["r"] / 1.0) + 1)
        pts = [arc_pose(pose, p["r"], sweep * i / n, rev)[:2] for i in range(n + 1)]
        return pts, arc_pose(pose, p["r"], sweep, rev)
    if t == "toward":
        return p["pts"], p["end"]
    raise ValueError(t)


def sim_toward(pose, tag, stop_in, step=0.5):
    """Curve toward the tag without spinning (pure pursuit, curvature capped), until
    stop_in from it. (pts, end pose) or raises ValueError."""
    x, y, th = pose
    pts = [(x, y)]
    for _ in range(20000):
        d = math.hypot(tag["x"] - x, tag["y"] - y)
        if d <= stop_in:
            return pts, (x, y, th)
        b = bearing((x, y, th), (tag["x"], tag["y"]))
        k = 2.0 * math.sin(b) / max(d, 1e-6)
        k = max(-1.0 / TOWARD_MIN_RADIUS_IN, min(1.0 / TOWARD_MIN_RADIUS_IN, k))
        th += k * step
        x += step * math.cos(th)
        y += step * math.sin(th)
        pts.append((x, y))
    raise ValueError("never got within %g in of tag %d" % (stop_in, tag["id"]))


def backward(prims):
    """Primitives worked out from the flipped pose -> the real backing-up ones. Spins are
    the same rotation either way; straights change sign; arcs back up."""
    out = []
    for p in prims:
        p = dict(p)
        if p["type"] == "line":
            p["in"] = -p["in"]
        elif p["type"] == "arc":
            p["rev"] = True
        out.append(p)
    return out


def expand(intent, pose, tags):
    """One drawn intent -> list of primitives from pose. Raises ValueError if it can't."""
    k = intent["kind"]
    if k in ("line", "arc") and intent.get("rev"):
        fwd = dict(intent, rev=False)
        return backward(expand(fwd, flip(pose), tags))
    stop = intent.get("stop", True)
    out = []
    if k == "spin":
        d = math.degrees(wrap(math.radians(intent["heading"]) - pose[2]))
        if abs(d) < MIN_TURN_DEG:
            raise ValueError("already facing that way")
        return [{"type": "spin", "deg": d, "stop": True}]
    if k == "line_dist":
        if abs(intent["dist"]) < MIN_DRIVE_IN:
            raise ValueError("too short")
        return [{"type": "line", "in": intent["dist"], "stop": stop}]
    if k == "line":
        tgt = intent["target"]
        dist = math.hypot(tgt[0] - pose[0], tgt[1] - pose[1])
        if dist < MIN_DRIVE_IN:
            raise ValueError("that is where the robot already is")
        b = bearing(pose, tgt)
        if abs(math.degrees(b)) < 0.5:
            return [{"type": "line", "in": dist, "stop": stop}]
        if not intent.get("smooth"):
            return [{"type": "spin", "deg": math.degrees(b), "stop": True},
                    {"type": "line", "in": dist, "stop": stop}]
        r = intent["R"]
        sgn = 1.0 if b > 0 else -1.0
        x, y, th = pose
        cx, cy = x - sgn * r * math.sin(th), y + sgn * r * math.cos(th)
        dc = math.hypot(tgt[0] - cx, tgt[1] - cy)
        if dc <= r + 0.01:
            raise ValueError("too close to curve onto at R=%g in (smaller R, or Stop mode)" % r)
        beta = math.atan2(tgt[1] - cy, tgt[0] - cx)
        a0 = th - sgn * math.pi / 2
        aT = beta - sgn * math.acos(r / dc)
        sweep = ((aT - a0) * sgn) % (2 * math.pi)
        lin = math.sqrt(dc * dc - r * r)
        if sweep * 180 / math.pi >= 0.5:
            out.append({"type": "arc", "r": r, "deg": sgn * math.degrees(sweep), "stop": False})
        if lin >= MIN_DRIVE_IN:
            out.append({"type": "line", "in": lin, "stop": stop})
        elif out:
            out[-1]["stop"] = stop
        return out
    if k == "arc":
        px, py = local(pose, intent["target"])
        if math.hypot(px, py) < MIN_DRIVE_IN:
            raise ValueError("that is where the robot already is")
        if abs(py) < 1e-3 * math.hypot(px, py):
            if px <= 0:
                raise ValueError("straight behind: no forward arc gets there")
            return [{"type": "line", "in": px, "stop": stop}]
        r = (px * px + py * py) / (2 * abs(py))
        if r < MIN_ARC_RADIUS_IN:
            raise ValueError("radius %.1f in is under %g in: use Spin" % (r, MIN_ARC_RADIUS_IN))
        sweep = 2 * math.atan2(abs(py), px)     # (0, 2pi)
        return [{"type": "arc", "r": r, "deg": math.degrees(sweep) * (1 if py > 0 else -1),
                 "stop": stop}]
    if k in ("aim", "toward"):
        tag = next((t for t in tags if t["id"] == intent["tag"]), None)
        if tag is None:
            raise ValueError("tag %s is gone" % intent["tag"])
        warn = tag_visible(pose, tag)
        b = math.degrees(bearing(pose, (tag["x"], tag["y"])))
        if k == "aim":
            if abs(b) <= HALF_FOV_DEG:
                p = {"type": "aim", "tag": tag["id"], "deg": b, "dir": None, "stop": True}
            else:
                p = {"type": "find", "tag": tag["id"], "deg": b,
                     "dir": "left" if b > 0 else "right", "stop": True}
            # find/aim end facing the tag: check visibility from where it ends up
            if warn:
                p["warn"] = warn
            return [p]
        if abs(b) > HALF_FOV_DEG:
            raise ValueError("tag %d is %.0f deg off, out of view: Turn to tag first"
                             % (tag["id"], b))
        d = math.hypot(tag["x"] - pose[0], tag["y"] - pose[1])
        if d <= intent["stop_in"]:
            raise ValueError("already within %g in of tag %d" % (intent["stop_in"], tag["id"]))
        if not 2 <= width_pct_for(intent["stop_in"]) <= 95:
            raise ValueError("stop distance %g in is outside what width %% can express"
                             % intent["stop_in"])
        pts, end = sim_toward(pose, tag, intent["stop_in"])
        p = {"type": "toward", "tag": tag["id"], "stop_in": intent["stop_in"],
             "pts": pts, "end": end, "stop": stop}
        if warn:
            p["warn"] = warn
        return [p]
    raise ValueError("unknown intent %r" % k)


def off_map(pts, bounds):
    if bounds is None:
        return False
    w, h = bounds
    return any(not (0 <= x <= w and 0 <= y <= h) for x, y in pts)


def simulate(start, intents, tags, bounds=None):
    """[(intent index, prim, pts, pose_before, pose_after, error)], end pose.
    bounds (w, h): primitives whose path leaves the map get a warning."""
    pose = (start["x"], start["y"], math.radians(start["heading"]))
    rows = []
    for i, it in enumerate(intents):
        try:
            prims = expand(it, pose, tags)
        except ValueError as e:
            rows.append((i, None, [], pose, pose, str(e)))
            continue
        for p in prims:
            pts, end = sim_prim(pose, p)
            if off_map(pts, bounds) and not p.get("warn"):
                p["warn"] = "leaves the map"
            rows.append((i, p, pts, pose, end, None))
            pose = end
    return rows, pose


def to_steps(rows):
    """Primitives -> plan-page step dicts (arcs split to drive_arc's limits)."""
    steps = []
    for _, p, _, _, _, err in rows:
        if p is None:
            continue
        t, stop = p["type"], p["stop"]
        if t == "spin":
            if abs(p["deg"]) < MIN_TURN_DEG:
                continue
            steps.append({"cmd": "turn", "direction": "left" if p["deg"] > 0 else "right",
                          "degrees": round(abs(p["deg"]), 1), "pwm": PWM["turn"]})
        elif t == "line":
            if abs(p["in"]) < MIN_DRIVE_IN:
                continue
            c = "forward" if p["in"] > 0 else "back"
            steps.append({"cmd": c, "inches": round(abs(p["in"]), 2), "pwm": PWM[c]})
        elif t == "arc":
            deg, r = abs(p["deg"]), p["r"]
            length = r * math.radians(deg)
            n = max(1, math.ceil(max(deg / MAX_ARC_DEG, length / MAX_ARC_IN) - 1e-9))
            for j in range(n):
                steps.append({"cmd": "arc", "direction": "left" if p["deg"] > 0 else "right",
                              "radius_in": round(r, 2), "degrees": round(deg / n, 1),
                              "travel": "reverse" if p.get("rev") else "forward",
                              "pwm": PWM["arc"],
                              "stop_after": stop if j == n - 1 else False})
            continue
        elif t == "aim":
            steps.append({"cmd": "aim", "tag": p["tag"]})
        elif t == "find":
            steps.append({"cmd": "find", "tag": p["tag"], "direction": p["dir"],
                          "pwm": PWM["find"]})
        elif t == "toward":
            steps.append({"cmd": "toward", "tag": p["tag"],
                          "width_pct": round(width_pct_for(p["stop_in"]), 1),
                          "tof_mm": None, "pwm": PWM["toward"]})
        steps[-1]["stop_after"] = stop
        if p.get("warn"):
            steps[-1]["warning"] = p["warn"]
    return steps


def step_text(s):
    c = s["cmd"]
    tag = "biggest tag" if s.get("tag") is None else "tag %d" % s["tag"]
    if c in ("forward", "back"):
        t = "%s %g in" % (c, s["inches"])
    elif c == "turn":
        t = "turn %s %g deg" % (s["direction"], s["degrees"])
    elif c == "arc":
        t = "arc %s%s %g deg r=%g in" % ("back " if s.get("travel") == "reverse" else "",
                                          s["direction"], s["degrees"], s["radius_in"])
    elif c == "aim":
        t = "aim at %s" % tag
    elif c == "find":
        t = "find %s, turning %s" % (tag, s["direction"])
    elif c == "toward":
        t = "toward %s until %g%% (~%.0f in)" % (tag, s["width_pct"],
                                                  REF_DISTANCE_IN * REF_WIDTH_PCT / s["width_pct"])
    else:
        t = c
    return t + ("" if s.get("stop_after", True) else "  ->")


# ---- the app --------------------------------------------------------------------

TOOLS = [("start", "Start pose", "s"), ("tag", "Place tag", "t"), ("aim", "Turn to tag", "a"),
         ("toward", "Go to tag", "g"), ("line", "Straight fwd", "l"),
         ("line_back", "Straight back", "b"), ("arc", "Arc fwd", "c"),
         ("arc_back", "Arc back", "v"), ("spin", "Spin", "p")]
ACTIONS = [("mode", "m"), ("snap", "n"), ("undo", "u"), ("clear", ""), ("save", "w"),
           ("load", "o")]


class App:
    def __init__(self, width_in, height_in, out=None, name=DEFAULT_NAME):
        self.W, self.H = float(width_in), float(height_in)
        self.out_arg = Path(out) if out else None   # --out pins the file; else from the name
        self.name = (name or DEFAULT_NAME).strip()[:NAME_MAX] or DEFAULT_NAME
        self.typing = None          # the name being typed, while editing it
        self.saved_path = None      # file this drawing came from / was last saved to
        self.overwrite_armed = 0.0
        self.s = min((CANVAS_W - 2 * MARGIN) / self.W, (WIN_H - 2 * MARGIN - 30) / self.H)
        self.ox, self.oy = TOOLBAR_W + MARGIN, MARGIN
        self.panel_x = int(self.ox + self.W * self.s + MARGIN)
        self.win_w = self.panel_x + PANEL_W
        self.start = {"x": 24.0, "y": round(self.H / 2), "heading": 0.0}
        self.tags, self.intents = [], []
        self.tool, self.smooth, self.snap = "line", False, True
        self.R, self.stop_in = 18.0, 18.0
        self.history = []
        self.hover = None       # (world point, shift)
        self.down = None        # world point where the button went down (start/tag drag)
        self.msg, self.msg_t = "draw with the mouse; keys in the toolbar", time.time()
        self.scroll = 0
        self.clear_armed = 0.0
        self.buttons = []       # [(x0, y0, x1, y1, name)]

    @property
    def out(self):
        return self.out_arg or PLANS_DIR / (file_stem(self.name) + ".json")

    # naming
    def start_naming(self):
        self.typing = self.name
        self.say("type the plan name, Enter to keep it, Esc to cancel")

    def finish_naming(self, keep):
        if self.typing is None:
            return
        new = self.typing.strip()
        self.typing = None
        if not keep:
            return self.say("name unchanged: %s" % self.name)
        if not new:
            return self.say("a plan needs a name: kept %s" % self.name)
        self.name = new
        self.say("plan named %s (saves to %s)" % (self.name, self.out))

    def name_key(self, k):
        if k in (13, 10):
            self.finish_naming(True)
        elif k == 27:
            self.finish_naming(False)
        elif k in (8, 127):
            self.typing = self.typing[:-1]
        elif 32 <= k < 127 and len(self.typing) < NAME_MAX:
            self.typing += chr(k)

    # coordinates
    def px(self, p):
        return (int(round(self.ox + p[0] * self.s)), int(round(self.oy + (self.H - p[1]) * self.s)))

    def world(self, X, Y):
        return ((X - self.ox) / self.s, self.H - (Y - self.oy) / self.s)

    def in_canvas(self, X, Y):
        x, y = self.world(X, Y)
        return 0 <= x <= self.W and 0 <= y <= self.H

    def snapped(self, p):
        if not self.snap:
            return p
        return (round(p[0] / SNAP_IN) * SNAP_IN, round(p[1] / SNAP_IN) * SNAP_IN)

    def say(self, text):
        self.msg, self.msg_t = text, time.time()

    # state
    def state(self):
        return {"start": self.start, "tags": self.tags, "intents": self.intents}

    def push(self):
        self.history.append(copy.deepcopy(self.state()))
        self.history = self.history[-500:]

    def undo(self):
        if not self.history:
            return self.say("nothing to undo")
        st = self.history.pop()
        self.start, self.tags, self.intents = st["start"], st["tags"], st["intents"]
        self.say("undone")

    def end_pose(self):
        return simulate(self.start, self.intents, self.tags, (self.W, self.H))[1]

    def nearest_tag(self, p, max_in=None):
        max_in = max_in if max_in is not None else max(12.0, 30 / self.s)
        best = min(self.tags, key=lambda t: math.hypot(t["x"] - p[0], t["y"] - p[1]),
                   default=None)
        if best is None or math.hypot(best["x"] - p[0], best["y"] - p[1]) > max_in:
            return None
        return best

    def make_intent(self, p, shift):
        """The intent the current tool would add for a click at world point p."""
        stop = not self.smooth
        if self.tool in ("aim", "toward"):
            tag = self.nearest_tag(p)
            if tag is None:
                raise ValueError("click on a tag" if self.tags else "place a tag first")
            if self.tool == "aim":
                return {"kind": "aim", "tag": tag["id"], "stop": True}
            return {"kind": "toward", "tag": tag["id"], "stop_in": self.stop_in, "stop": stop}
        pose = self.end_pose()
        q = self.snapped(p)
        rev = self.tool.endswith("_back")
        tool = self.tool[:-5] if rev else self.tool
        if tool == "line":
            if shift:
                fwd, _ = local(pose, p)
                d = round(fwd / SNAP_IN) * SNAP_IN if self.snap else fwd
                return {"kind": "line_dist", "dist": d, "stop": stop}
            return {"kind": "line", "target": list(q), "smooth": self.smooth, "R": self.R,
                    "rev": rev, "stop": stop}
        if tool == "arc":
            return {"kind": "arc", "target": list(q), "rev": rev, "stop": stop}
        if self.tool == "spin":
            h = math.degrees(math.atan2(q[1] - pose[1], q[0] - pose[0]))
            if self.snap:
                h = round(h / SNAP_DEG) * SNAP_DEG
            return {"kind": "spin", "heading": h, "stop": True}
        raise ValueError(self.tool)

    def click(self, p, shift):
        try:
            it = self.make_intent(p, shift)
            expand(it, self.end_pose(), self.tags)        # refuse it now, not later
        except ValueError as e:
            return self.say("can't: %s" % e)
        self.push()
        self.intents.append(it)
        self.say("added %s" % it["kind"])

    def drag_done(self, a, b):
        """Start pose / tag: a = where pressed, b = where released."""
        a = self.snapped(a)
        if math.hypot(b[0] - a[0], b[1] - a[1]) * self.s > 6:
            h = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        else:
            h = None
        if self.snap and h is not None:
            h = round(h / SNAP_DEG) * SNAP_DEG
        self.push()
        if self.tool == "start":
            self.start = {"x": a[0], "y": a[1],
                          "heading": self.start["heading"] if h is None else h}
            self.say("start pose set")
        else:
            near = self.nearest_tag(a, 6.0)
            if near is not None and h is not None:       # drag on an existing tag: re-aim it
                near["face"] = h
                return self.say("tag %d now faces %.0f deg" % (near["id"], h))
            if h is None:       # default: face the robot's start
                h = math.degrees(math.atan2(self.start["y"] - a[1], self.start["x"] - a[0]))
            tid = max((t["id"] for t in self.tags), default=-1) + 1
            self.tags.append({"id": tid, "x": a[0], "y": a[1], "face": round(h, 1)})
            self.say("tag %d placed" % tid)

    def action(self, name):
        if name == "mode":
            self.smooth = not self.smooth
            self.say("smooth: moves flow into each other" if self.smooth
                     else "stop between moves")
        elif name == "snap":
            self.snap = not self.snap
            self.say("snap %s" % ("on" if self.snap else "off"))
        elif name == "undo":
            self.undo()
        elif name == "clear":
            if time.time() - self.clear_armed < 2.0:
                self.push()
                self.intents, self.tags = [], []
                self.say("cleared (Undo brings it back)")
            else:
                self.clear_armed = time.time()
                self.say("press Clear again to clear everything")
        elif name == "save":
            self.save()
        elif name == "load":
            self.load(self.out)

    def save(self, path=None, force=False):
        path = Path(path or self.out)
        if (path.exists() and path != self.saved_path and not force
                and time.time() - self.overwrite_armed > 2.0):
            self.overwrite_armed = time.time()
            self.say("can't: %s already exists: press Save again to overwrite it" % path.name)
            return None
        self.overwrite_armed = 0.0
        rows, end = simulate(self.start, self.intents, self.tags, (self.W, self.H))
        doc = {"format": "ellama plan drawing v1", "name": self.name, "units": "in",
               "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
               "canvas_in": [self.W, self.H],
               "start": self.start, "tags": self.tags, "intents": self.intents,
               "settings": {"smooth": self.smooth, "snap": self.snap, "R": self.R,
                            "stop_in": self.stop_in},
               "end_pose": {"x": round(end[0], 2), "y": round(end[1], 2),
                            "heading": round(math.degrees(end[2]), 1)},
               "problems": ["move %d: %s" % (i + 1, e) for i, _, _, _, _, e in rows if e],
               "steps": to_steps(rows)}
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(json.dumps(doc, indent=1))
        tmp.replace(path)
        self.saved_path = path
        self.say("saved %s: %d steps to %s" % (self.name, len(doc["steps"]), path))
        return doc

    def load(self, path):
        path = Path(path)
        try:
            doc = json.loads(path.read_text())
        except Exception as e:
            return self.say("can't load %s: %s" % (path, e))
        self.push()
        self.start, self.tags, self.intents = doc["start"], doc["tags"], doc["intents"]
        self.name = doc.get("name") or path.stem
        self.saved_path = path
        st = doc.get("settings", {})
        self.smooth = st.get("smooth", self.smooth)
        self.snap = st.get("snap", self.snap)
        self.R, self.stop_in = st.get("R", self.R), st.get("stop_in", self.stop_in)
        self.say("loaded %s from %s" % (self.name, path))

    # input
    def on_mouse(self, ev, X, Y, flags, _):
        shift = bool(flags & cv2.EVENT_FLAG_SHIFTKEY)
        if ev == cv2.EVENT_MOUSEWHEEL:
            if X > self.panel_x:
                self.scroll = max(0, self.scroll + (-3 if cv2.getMouseWheelDelta(flags) > 0 else 3))
            return
        if ev == cv2.EVENT_LBUTTONDOWN:
            if self.typing is not None:         # clicking away keeps what was typed
                self.finish_naming(True)
            for x0, y0, x1, y1, name in self.buttons:
                if x0 <= X <= x1 and y0 <= Y <= y1:
                    if name == "name":
                        self.start_naming()
                    elif name in dict((t[0], 1) for t in TOOLS):
                        self.tool = name
                        self.say(dict((t[0], t[1]) for t in TOOLS)[name])
                    else:
                        self.action(name)
                    return
            if self.in_canvas(X, Y):
                p = self.world(X, Y)
                if self.tool in ("start", "tag"):
                    self.down = p
                else:
                    self.click(p, shift)
        elif ev == cv2.EVENT_LBUTTONUP:
            if self.down is not None:
                self.drag_done(self.down, self.world(X, Y))
                self.down = None
        elif ev == cv2.EVENT_MOUSEMOVE:
            self.hover = (self.world(X, Y), shift) if self.in_canvas(X, Y) else None

    def on_key(self, k):
        if self.typing is not None:             # every key goes to the name
            self.name_key(k)
            return True
        if k in (27, ord("q")):
            return False
        if k == ord("r"):
            self.start_naming()
            return True
        keys = {t[2]: t[0] for t in TOOLS}
        ch = chr(k) if 0 <= k < 256 else ""
        if ch in keys:
            self.tool = keys[ch]
            self.say(dict((t[0], t[1]) for t in TOOLS)[self.tool])
        elif ch == "m":
            self.action("mode")
        elif ch == "n":
            self.action("snap")
        elif ch == "u" or k == 8:
            self.undo()
        elif ch == "w":
            self.save()
        elif ch == "o":
            self.load(self.out)
        elif ch == "[":
            self.stop_in = max(4.0, self.stop_in - 2)
            self.say("go-to-tag stops %g in from the tag" % self.stop_in)
        elif ch == "]":
            self.stop_in = min(170.0, self.stop_in + 2)
            self.say("go-to-tag stops %g in from the tag" % self.stop_in)
        elif ch == "-":
            self.R = max(MIN_ARC_RADIUS_IN, self.R - 2)
            self.say("smooth fillet radius %g in" % self.R)
        elif ch == "=":
            self.R = min(120.0, self.R + 2)
            self.say("smooth fillet radius %g in" % self.R)
        return True

    # drawing
    def text(self, img, s, xy, scale=0.48, col=FG, thick=1):
        cv2.putText(img, s, xy, FONT, scale, col, thick, cv2.LINE_AA)

    def draw_robot(self, img, pose, col, thick=2):
        x, y, th = pose
        L = 7.0
        nose = (x + L * math.cos(th), y + L * math.sin(th))
        l = (x + L * 0.7 * math.cos(th + 2.4), y + L * 0.7 * math.sin(th + 2.4))
        r = (x + L * 0.7 * math.cos(th - 2.4), y + L * 0.7 * math.sin(th - 2.4))
        pts = np.array([self.px(nose), self.px(l), self.px(r)], np.int32)
        cv2.polylines(img, [pts], True, col, thick, cv2.LINE_AA)

    def draw_rows(self, img, rows, dim=False):
        for _, p, pts, before, after, err in rows:
            if p is None:
                continue
            col = COL[p["type"]]
            if dim:
                col = tuple(int(c * 0.55) for c in col)
            if len(pts) >= 2:
                back = (p["type"] == "line" and p["in"] < 0) or p.get("rev")
                poly = [self.px(q) for q in pts]
                if back:            # dashed: every other ~10 px of path
                    dense = []
                    for a, b in zip(poly, poly[1:]):
                        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 2))
                        dense += [(a[0] + (b[0] - a[0]) * j / n, a[1] + (b[1] - a[1]) * j / n)
                                  for j in range(n)]
                    dense.append(poly[-1])
                    for j in range(0, len(dense) - 1, 10):
                        seg = np.array(dense[j:j + 6], np.int32)
                        if len(seg) >= 2:
                            cv2.polylines(img, [seg], False, col, 2 if dim else 3, cv2.LINE_AA)
                else:
                    cv2.polylines(img, [np.array(poly, np.int32)], False,
                                  col, 2 if dim else 3, cv2.LINE_AA)
            if p["type"] in ("spin", "aim", "find"):
                c = self.px(before[:2])
                a0, a1 = -math.degrees(before[2]), -math.degrees(after[2])
                cv2.ellipse(img, c, (14, 14), 0, a0, a1, col, 2, cv2.LINE_AA)
                if p["type"] != "spin":
                    tg = next((t for t in self.tags if t["id"] == p["tag"]), None)
                    if tg:
                        cv2.line(img, c, self.px((tg["x"], tg["y"])), col, 1, cv2.LINE_AA)
            if not dim:
                cv2.circle(img, self.px(after[:2]), 5 if p["stop"] else 3,
                           STOP_DOT if p["stop"] else FLOW_DOT, -1, cv2.LINE_AA)
                if p.get("warn"):
                    self.text(img, "!", (self.px(after[:2])[0] + 6, self.px(after[:2])[1] - 6),
                              0.6, ERR, 2)

    def render(self):
        img = np.full((WIN_H, self.win_w, 3), BG, np.uint8)
        # canvas + grid
        x0, y0 = self.px((0, self.H))
        x1, y1 = self.px((self.W, 0))
        cv2.rectangle(img, (x0, y0), (x1, y1), CANVAS_BG, -1)
        for gx in np.arange(0, self.W + 0.01, GRID_IN):
            col = GRID_MAJOR if int(round(gx)) % 60 == 0 else GRID
            cv2.line(img, self.px((gx, 0)), self.px((gx, self.H)), col, 1)
        for gy in np.arange(0, self.H + 0.01, GRID_IN):
            col = GRID_MAJOR if int(round(gy)) % 60 == 0 else GRID
            cv2.line(img, self.px((0, gy)), self.px((self.W, gy)), col, 1)
        cv2.rectangle(img, (x0, y0), (x1, y1), GRID_MAJOR, 1)
        sb0, sb1 = self.px((12, 6)), self.px((72, 6))
        cv2.line(img, sb0, sb1, MUT, 2)
        self.text(img, "5 ft  (grid 1 ft)", (sb0[0], sb0[1] - 6), 0.42, MUT)
        # tags
        for t in self.tags:
            c = self.px((t["x"], t["y"]))
            h = 3.0 * self.s
            cv2.rectangle(img, (int(c[0] - h), int(c[1] - h)), (int(c[0] + h), int(c[1] + h)),
                          (240, 240, 240), -1)
            cv2.rectangle(img, (int(c[0] - h / 2), int(c[1] - h / 2)),
                          (int(c[0] + h / 2), int(c[1] + h / 2)), (20, 20, 20), -1)
            f = math.radians(t["face"])
            cv2.arrowedLine(img, c, self.px((t["x"] + 10 * math.cos(f), t["y"] + 10 * math.sin(f))),
                            (200, 200, 200), 1, cv2.LINE_AA, tipLength=0.3)
            self.text(img, "tag %d" % t["id"], (c[0] + int(h) + 4, c[1] - int(h)), 0.45, FG)
        # plan
        rows, end = simulate(self.start, self.intents, self.tags, (self.W, self.H))
        self.draw_rows(img, rows)
        self.draw_robot(img, (self.start["x"], self.start["y"], math.radians(self.start["heading"])),
                        ACCENT, 2)
        self.draw_robot(img, end, FG, 2)
        # hover preview
        if self.hover and self.down is None and self.tool not in ("start", "tag"):
            p, shift = self.hover
            try:
                it = self.make_intent(p, shift)
                prims = expand(it, end, self.tags)
                prow, pose = [], end
                for pr in prims:
                    pts, e = sim_prim(pose, pr)
                    prow.append((0, pr, pts, pose, e, None))
                    pose = e
                self.draw_rows(img, prow, dim=True)
                self.draw_robot(img, pose, MUT, 1)
                hint = ", ".join(step_text(s) for s in to_steps(prow))
            except ValueError as e:
                hint = "can't: %s" % e
            hx, hy = self.px(p)
            self.text(img, hint[:90], (hx + 12, hy + 18), 0.42, MUT)
        if self.down is not None and self.hover:
            cv2.arrowedLine(img, self.px(self.snapped(self.down)), self.px(self.hover[0]),
                            ACCENT, 2, cv2.LINE_AA)
        if self.hover:
            p = self.hover[0]
            self.text(img, "x %.1f in  y %.1f in" % p, (x0, WIN_H - 10), 0.45, MUT)
        self.text(img, self.msg[:110], (x0 + 220, WIN_H - 10), 0.45,
                  ERR if self.msg.startswith("can't") else FG)
        img[:, :TOOLBAR_W] = BG          # paths that leave the map stay off the side panels
        img[:, self.panel_x:] = BG
        self.draw_toolbar(img)
        self.draw_panel(img, rows, end)
        return img

    def draw_toolbar(self, img):
        self.buttons = []
        self.text(img, "PLAN DRAWER", (14, 30), 0.6, ACCENT, 2)
        y = 46
        if self.typing is not None:
            shown = self.typing + ("_" if int(time.time() * 2) % 2 == 0 else " ")
            self._button(img, y, shown[-17:], "name", True)
        else:
            self._button(img, y, ("Name: " + self.name)[:16] + " [r]", "name", False)
        y += 50
        for name, label, key in TOOLS:
            active = self.tool == name
            self._button(img, y, "%s  [%s]" % (label, key), name, active)
            y += 44
        y += 14
        labels = {"mode": "Smooth  [m]" if self.smooth else "Stop between  [m]",
                  "snap": "Snap %s  [n]" % ("on" if self.snap else "off"),
                  "undo": "Undo  [u]", "clear": "Clear", "save": "Save  [w]",
                  "load": "Load  [o]"}
        for name, _ in ACTIONS:
            self._button(img, y, labels[name], name,
                         name == "mode" and self.smooth or name == "snap" and self.snap)
            y += 44
        y += 10
        for line in ("fillet R  %g in  [- =]" % self.R,
                     "tag stop  %g in  [[ ]]" % self.stop_in,
                     "shift+Straight: along", "  current heading",
                     "dashed = backing up",
                     "red dot = stops", "green dot = flows on"):
            self.text(img, line, (14, y), 0.42, MUT)
            y += 20

    def _button(self, img, y, label, name, active):
        x0, x1, y1 = 10, TOOLBAR_W - 10, y + 36
        cv2.rectangle(img, (x0, y), (x1, y1), (70, 110, 170) if active else (52, 52, 50), -1)
        self.text(img, label, (x0 + 10, y + 24), 0.5, FG)
        self.buttons.append((x0, y, x1, y1, name))

    def draw_panel(self, img, rows, end):
        x = self.panel_x + 12
        steps = to_steps(rows)
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for _, p, pts, _, _, _ in rows if p for a, b in zip(pts, pts[1:]))
        self.text(img, self.name[:30], (x, 30), 0.6, FG, 2)
        self.text(img, str(self.out.relative_to(HERE)) if self.out.is_relative_to(HERE)
                  else str(self.out), (x, 50), 0.4, MUT)
        self.text(img, "STEPS (%d)   path %.0f in" % (len(steps), length), (x, 78), 0.55, ACCENT, 2)
        self.text(img, "end  x %.0f  y %.0f  heading %.0f deg" % (
            end[0], end[1], math.degrees(wrap(end[2]))), (x, 100), 0.42, MUT)
        y = 126
        lines = []
        for i, s in enumerate(steps, 1):
            lines.append(("%2d. %s" % (i, step_text(s)), FG))
            if s.get("warning"):
                lines.append(("     ! " + s["warning"], ERR))
        for i, _, _, _, _, e in rows:
            if e:
                lines.append(("move %d skipped: %s" % (i + 1, e), ERR))
        fit = (WIN_H - y - 20) // 20
        self.scroll = min(self.scroll, max(0, len(lines) - fit))
        for s, col in lines[self.scroll:self.scroll + fit]:
            self.text(img, s[:52], (x, y), 0.42, col)
            y += 20
        if not lines:
            self.text(img, "(click on the map to add moves)", (x, y), 0.42, MUT)


# ---- self test ------------------------------------------------------------------

def selftest(snapshot=None):
    def close(a, b, tol=0.05):
        return all(abs(p - q) <= tol for p, q in zip(a, b))
    tags = []
    pose = (0.0, 0.0, 0.0)
    # arc through a point
    for tgt in [(20, 15), (20, -15), (-10, 30), (5, -40)]:
        prims = expand({"kind": "arc", "target": tgt}, pose, tags)
        _, e = sim_prim(pose, prims[0])
        assert close(e[:2], tgt), ("arc", tgt, e)
    # smooth fillet: arc ends facing the target, line ends on it
    for tgt in [(40, 40), (40, -40), (-30, 10), (-30, -50), (60, 1)]:
        prims = expand({"kind": "line", "target": tgt, "smooth": True, "R": 18}, pose, tags)
        p = pose
        for pr in prims:
            if pr["type"] == "line":
                assert abs(math.degrees(bearing(p, tgt))) < 0.05, ("fillet heading", tgt)
            _, p = sim_prim(p, pr)
        assert close(p[:2], tgt), ("fillet end", tgt, p)
        assert prims[0]["stop"] is False
    try:
        expand({"kind": "line", "target": (0, 10), "smooth": True, "R": 18}, pose, tags)
        raise AssertionError("fillet inside the circle should be refused")
    except ValueError:
        pass
    # backing up: reverse arcs and straights land on points behind
    for tgt in [(-20, 15), (-20, -15), (10, 30), (-5, -40)]:
        prims = expand({"kind": "arc", "target": tgt, "rev": True}, pose, tags)
        assert prims[0]["type"] == "arc" and prims[0].get("rev"), prims
        _, e = sim_prim(pose, prims[0])
        assert close(e[:2], tgt), ("reverse arc", tgt, e)
        st = to_steps([(0, prims[0], [], pose, e, None)])
        assert st[0]["travel"] == "reverse", st
    for smooth in (False, True):
        for tgt in [(-40, 30), (-40, -30), (30, 50), (-60, 1)]:
            prims = expand({"kind": "line", "target": tgt, "rev": True, "smooth": smooth,
                            "R": 18}, pose, tags)
            p = pose
            for pr in prims:
                _, p = sim_prim(p, pr)
            assert close(p[:2], tgt), ("reverse line", smooth, tgt, p)
            assert prims[-1]["type"] == "line" and prims[-1]["in"] < 0, prims
            # backing up: the robot ends with its rear toward where it came from
            assert abs(math.degrees(bearing(flip(p), (0, 0)))) > 90 or smooth, (tgt, p)
    st = to_steps([(0, pr, [], pose, pose, None) for pr in
                   expand({"kind": "line", "target": (-48, 0), "rev": True}, pose, tags)])
    assert st == [{"cmd": "back", "inches": 48.0, "pwm": 50, "stop_after": True}], st
    # arc too tight
    try:
        expand({"kind": "arc", "target": (2, 3)}, pose, tags)
        raise AssertionError("tight arc should be refused")
    except ValueError:
        pass
    # splits respect drive_arc limits
    rows = [(0, {"type": "arc", "r": 100.0, "deg": 180.0, "stop": True}, [], pose, pose, None)]
    st = to_steps(rows)
    assert len(st) == 2 and all(s["radius_in"] * math.radians(s["degrees"]) <= MAX_ARC_IN
                                for s in st), st
    assert st[0]["stop_after"] is False and st[1]["stop_after"] is True
    # rectangle, stop mode: forward, then 3x (turn left 90 + forward), then a turn home
    start = {"x": 24.0, "y": 24.0, "heading": 0.0}
    corners = [(72, 24), (72, 60), (24, 60), (24, 24)]
    intents = [{"kind": "line", "target": c, "smooth": False, "R": 18, "stop": True}
               for c in corners]
    intents.append({"kind": "spin", "heading": 0.0})
    st = to_steps(simulate(start, intents, tags)[0])
    assert [s["cmd"] for s in st] == ["forward"] + ["turn", "forward"] * 3 + ["turn"], st
    assert all(s["direction"] == "left" and abs(s["degrees"] - 90) < 0.1
               for s in st if s["cmd"] == "turn"), st
    # smooth rectangle: arcs instead of turns, no stops inside
    sm = [dict(i, smooth=True, stop=False) for i in intents[:4]]
    st = to_steps(simulate(start, sm, tags)[0])
    assert "turn" not in [s["cmd"] for s in st] and "arc" in [s["cmd"] for s in st], st
    # tags
    tags = [{"id": 3, "x": 200.0, "y": 30.0, "face": 180.0}]
    rows, end = simulate(start, [{"kind": "aim", "tag": 3},
                                 {"kind": "toward", "tag": 3, "stop_in": 18, "stop": True}], tags)
    assert all(r[5] is None for r in rows), rows
    assert abs(math.hypot(200 - end[0], 30 - end[1]) - 18) < 0.6, end
    st = to_steps(rows)
    assert [s["cmd"] for s in st] == ["aim", "toward"], st
    rows, _ = simulate({"x": 24, "y": 24, "heading": 180}, [{"kind": "aim", "tag": 3}], tags)
    assert rows[0][1]["type"] == "find" and rows[0][1]["dir"] in ("left", "right")
    rows, _ = simulate({"x": 24, "y": 24, "heading": 180},
                       [{"kind": "toward", "tag": 3, "stop_in": 18}], tags)
    assert rows[0][1] is None and "out of view" in rows[0][5], rows
    rows, _ = simulate(start, [{"kind": "line_dist", "dist": 400}], tags, (288, 240))
    assert rows[0][1]["warn"] == "leaves the map"
    # save / load round trip
    app = App(288, 240, Path(tempfile.mkdtemp()) / "d.json")
    app.start, app.tags = dict(start), copy.deepcopy(tags)
    app.intents = copy.deepcopy(intents) + [{"kind": "aim", "tag": 3},
                                            {"kind": "toward", "tag": 3, "stop_in": 20,
                                             "stop": True},
                                            {"kind": "arc", "target": [150, 150], "stop": True}]
    doc = app.save()
    b = App(288, 240, app.out)
    b.load(app.out)
    assert b.state() == json.loads(json.dumps(app.state())), "round trip"
    # names
    assert file_stem("garage loop #2") == "garage_loop_2" and file_stem("  ") == DEFAULT_NAME
    c = App(288, 240)
    c.start_naming()
    for ch in "\b" * 20 + "garage loop":
        c.name_key(ord(ch))
    c.name_key(13)
    assert c.name == "garage loop" and c.out == PLANS_DIR / "garage_loop.json", c.out
    c.start_naming(); c.name_key(ord("x")); c.name_key(27)
    assert c.name == "garage loop", "Esc keeps the old name"
    named = App(288, 240, Path(tempfile.mkdtemp()) / "n.json", name="lap A")
    named.save()
    again = App(288, 240)
    again.load(named.out)
    assert again.name == "lap A", again.name
    # saving onto someone else's file needs a second press
    other = App(288, 240, named.out, name="other")
    assert other.save() is None and other.save() is not None
    assert to_steps(simulate(b.start, b.intents, b.tags)[0]) == doc["steps"]
    if snapshot:
        app.hover = ((120.0, 200.0), False)
        app.tool = "arc"
        cv2.imwrite(str(snapshot), app.render())
    print("selftest ok: %d steps in the sample drawing" % len(doc["steps"]))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--width-in", type=float, default=288, help="map width, inches (24 ft)")
    ap.add_argument("--height-in", type=float, default=240, help="map height, inches (20 ft)")
    ap.add_argument("--name", default=DEFAULT_NAME, help="plan name (also the file name)")
    ap.add_argument("--out", help="save / load file (default plans/<name>.json)")
    ap.add_argument("--load", help="drawing to open at start")
    ap.add_argument("--selftest", action="store_true", help="check the geometry and exit")
    ap.add_argument("--snapshot", help="with --selftest: write a rendered frame here")
    a = ap.parse_args()
    if a.selftest:
        selftest(a.snapshot)
        return
    app = App(a.width_in, a.height_in, a.out, a.name)
    if a.load:
        app.load(a.load)
    win = "plan drawer"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(win, app.win_w, WIN_H)
    cv2.setMouseCallback(win, app.on_mouse)
    title = None
    while True:
        if title != app.name:
            title = app.name
            try:
                cv2.setWindowTitle(win, "plan drawer: " + title)
            except cv2.error:
                pass
        cv2.imshow(win, app.render())
        k = cv2.waitKey(20)
        if k != -1 and not app.on_key(k & 0xFF):
            break
        try:
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
        except cv2.error:
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
