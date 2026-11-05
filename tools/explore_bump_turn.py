#!/usr/bin/env python3
"""Bump-and-turn coverage of an enclosed, flat, drop-free space, with NO WHEEL ENCODERS.

Not mapping and not SLAM: there is no pose estimate here and none is needed. Drive until
the forward cones see a wall, ask llama-router whether that move actually worked, back off
and turn if it did not, repeat. Early-Roomba, with a language model as the supervisor
between moves instead of a bump switch.

WHY NO ENCODERS. Both encoder channels on this robot have frozen intermittently, and the
encoder-closed-loop version of this script spent most of its complexity on that: a
channel-rediscovery loop around every straight move, a `wheels` question that was always
dropped, and a stop-the-run verdict whenever a channel went quiet mid-run. So this version
does not read them at all. It drops "enc" from the server's required-sensor set at startup,
which means the drive tools will move the robot with the encoder link dead or absent.

WHAT MEASURES A MOVE NOW. Two instruments, and nothing else:

  * the forward ToF, for distance. `drive_until_range` already closes its loop on the
    range sensor rather than the encoders, so an approach is still closed-loop; a move
    across open floor is a timed `jog` and is not.
  * the IMU, for rotation and for heading hold. `turn_degrees` was always IMU-only -- the
    encoders over-read an in-place spin by ~1.7-2x through wheel scrub -- so turns are
    unchanged from the encoder version. Straight moves now also carry an IMU yaw delta,
    which is what is left of "is one side dragging": a dragging side cannot be measured
    any more, but it still swings the nose.

WHAT THIS COSTS, plainly:

  * A straight move across open floor is open-loop and unverified. With nothing in range
    there is no displacement channel at all, so `progress` and `cause` are not asked --
    answering them from nothing is how `unmeasured` would become the verdict on every
    move in a big room and halt the run a minute in. The check on such a move is the next
    range read, which is why --step-in defaults to 9 in here rather than 18.
  * `stalled` and `blocked` have collapsed into one `not_advancing`. They differ only in
    whether the wheels were turning, which is exactly what is gone. The robot can no
    longer tell "something is holding me" from "the motors are not driving".
  * Distance commanded open-loop comes off the measured PWM/speed curve in
    mcp_server/drive_math.py plus the 0.195 s the motor board spends ramping
    (ROBOT_CHARACTERIZATION.md, "Timing and heading"). It is an estimate, not a
    measurement, and skid-steer open-loop driving curves off line on its own.

WHAT THIS REQUIRES OF THE SPACE, and does not check for you:
  * enclosed -- walls the robot may touch at walking pace, no open sides
  * flat, and NO DROPS. The tilt guard reports a fall, it does not prevent one: it stops
    the motors, which does nothing once the centre of mass is past the balance point and
    nothing at all in free fall. There is no downward sensor on this robot. Flash one as
    TOF_DOWN and `sensors()` picks it up with no code change, then revisit this.
  * nothing fragile, and nobody underfoot
  * "nothing in range" is NOT clearance. Two narrow cones at a fixed height cannot see the
    floor, a drop, the sides, or anything below the beam, and a dark or oblique surface
    returns nothing at all from a foot away. With the encoders gone the forward step is
    the one move with nothing watching it, so it is kept short deliberately.

SAFEGUARDS HERE, on top of the server's own:
  * pwm capped at --pwm (default 60, about 6.1 in/s) -- a bump at walking pace
  * a live forward ToF is required at startup and stops the run if it goes quiet: it is
    now the only thing that can measure displacement, not merely one channel of several
  * the tilt guard's lookahead_s raised to 0.4, because the motor board needs 0.4 s to
    ramp down from high PWM while the guard defaults to assuming 0.3
  * waits at startup for the tilt guard's level reference instead of driving without it,
    and stops if it never arrives
  * one typed confirmation at startup, because dropping "enc" from the required set
    widens what the robot will do with nothing checking it (see set_required_sensors)
  * stops on: a tilt event, a blind or dead range sensor, a dead IMU, an `unmeasured`
    the model believes, two consecutive failures to make progress after backing off,
    --max-moves, --max-minutes
  * asks before every move unless --no-ask
  * stop() on every exit path, including ctrl-c and exceptions

The MCP server owns the serial port, so this starts its own copy over stdio the way
ollama_bridge.py does. Shut down any other robot_server.py first or they will fight over
/dev/ttyUSB0.

The venv matters: the system python3 has no `mcp` module, so run it through the one the
server uses.

  export LLAMA_ROUTER_KEY=lr_...
  mcp_server/.venv/bin/python tools/explore_bump_turn.py --max-moves 20
  mcp_server/.venv/bin/python tools/explore_bump_turn.py --max-moves 40 --no-ask --log /tmp/run.csv
"""

import argparse, asyncio, csv, json, os, random, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
# The server's own measured speed curve and forward-PWM convention, imported rather than
# copied. The copy this script used to carry had drifted: 5.9 in/s at pwm 60 against the
# 6.13 in drive_math, which is a 4% distance error on every open-loop move -- survivable
# when an encoder was closing the loop, and not when nothing is.
sys.path[1:1] = [REPO, os.path.join(REPO, "mcp_server")]
import drive_math as dm
import decision_client as dc

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

SERVER = os.path.join(REPO, "mcp_server", "robot_server.py")

# Sensors the drive tools must have live for this run. "enc" is deliberately absent; "tof"
# is deliberately present, because with the encoders gone a quiet ToF leaves nothing at all
# measuring whether the robot moved.
REQUIRED_SENSORS = ["imu", "tof"]

UNMEASURED_STOP_CONF = 0.7      # see decide_straight(): below this, `unmeasured` is a guess
# Movement starts 0.195 s after the command, set by the motor board's PWM ramp of ~333
# PWM/s (ROBOT_CHARACTERIZATION.md, "Timing and heading"). An open-loop move that does not
# add it is short by that much travel every time -- 1.2 in at pwm 60.
RAMP_LEAD_S = 0.195
# "durations under ~0.5 s are mostly ramp", same source: below this the distance a jog
# covers is not a function of the speed curve at all.
MIN_JOG_S = 0.5
# How long to wait at startup for the tilt guard to capture its level reference. It needs
# ~0.5 s of a still, live IMU; 15 s is long enough that a slow serial start or a couple of
# interrupted attempts still make it, and short enough to be an obvious wait, not a hang.
REF_WAIT_S = 15.0

# drive_until_range aborts for four reasons, and they are not all faults. Three of them are
# what a bump looks like from the range sensor's side, and ending the run on those was the
# encoder version's mistake: it is the encoders that used to tell a bump from a fault, and
# they are gone.
BUMP_ABORTS = ("nothing got closer", "lost the target", "short of the target after")
BLIND_ABORTS = ("stopped reporting",)

# What drive_until_range measured about a move, logged on every row. Fixed, and present
# even where it does not apply (a spin, an open-loop jog), for two reasons: csv.DictWriter
# takes its fieldnames from the FIRST row and raises on any later row with extra keys, so
# "add the column only when there is one" loses the whole log at the end of any run that
# happens to open with a spin; and a null in a column that exists reads as "not that kind
# of move", where a missing column reads as nothing at all.
#
# `abort_reason` is the one that was missing, and it is why the last run could not say
# which of the six identical-looking aborts were obstructions: the loop printed it to the
# terminal and logged only the model's verdict. hit_timeout and stopped_short come with
# it because abort_reason is null on exactly those two outcomes, which is the same gap
# one column further along.
#
# wheel_travel_in is logged and never read, the way `veering` is. This run does not
# require the encoders and does not trust them, so it is usually null -- but when a
# channel is alive it is the only independent witness that the robot moved at all, which
# is precisely what telling a bump from a lost target needs and what the range sensor,
# measuring the thing the robot may be stuck against, cannot give.
MEASURED = ("abort_reason", "hit_timeout", "stopped_short", "within_tolerance",
            "pulses", "range_closed_mm", "wheel_travel_in")


def approach_facts(m):
    """The MEASURED fields out of a drive_until_range result."""
    return {k: m.get(k) for k in MEASURED}


class Stop(Exception):
    """Something the operator must look at. Always ends the run."""


def result_json(res):
    for block in getattr(res, "content", []) or []:
        txt = getattr(block, "text", None)
        if txt:
            try:
                return json.loads(txt)
            except json.JSONDecodeError:
                return {"ok": False, "raw": txt}
    return {"ok": False, "raw": "(no content)"}


def jog_seconds(inches, pwm):
    """How long to drive at `pwm` to cover `inches`, open-loop. Raises if the answer is
    not meaningful: below the deadband the robot does not move, and under MIN_JOG_S the
    move is mostly ramp."""
    v = dm.speed_for_pwm(pwm)
    if v is None:
        raise Stop(f"pwm {pwm} is below the measured deadband of {dm.PWM_DEADBAND}, where "
                   f"this robot does not move at all")
    dur = RAMP_LEAD_S + inches / v
    if dur < MIN_JOG_S:
        raise Stop(f"{inches:.1f} in at pwm {pwm} is {dur:.2f} s, which is mostly the "
                   f"motor board's ramp -- ask for a longer move or a lower pwm")
    return dur


def yaw_delta(before, after):
    """Shortest signed yaw change in degrees, wrap-safe. The filter's yaw is free to run
    past 360 or wrap, and neither should turn a 3 deg drift into a 357 deg one."""
    return (after - before + 180.0) % 360.0 - 180.0


class Run:
    def __init__(self, session, args):
        self.s, self.a = session, args
        self.rows = []
        self.moves = 0
        self.t0 = time.monotonic()
        self.rng = random.Random(args.seed)
        self.no_progress = 0
        # For the temporal lens check in read_range. Commanded inches, not measured ones:
        # there is nothing left that measures them. See the stop message for what that
        # costs -- the check can no longer tell a blocked lens from a robot standing still.
        self.last_near = None
        self.travel_since_near = 0.0

    async def call(self, _tool, **kw):
        return result_json(await self.s.call_tool(_tool, kw))

    async def guard_check(self):
        g = await self.call("guards")
        # A server that cannot read its guards is not a server that may drive. This is
        # where a dead serial link shows up first -- including another robot_server.py
        # already holding /dev/ttyUSB0, which locks the port exclusively.
        if not g.get("ok"):
            raise Stop(f"cannot read the guards: {g.get('error') or g.get('raw')}")
        t = (g.get("guards") or g).get("tilt") or {}
        if t.get("tripped"):
            raise Stop(f"tilt guard tripped: {(t.get('fault') or {}).get('reason')}")
        return t

    async def wait_for_level_reference(self):
        """Block until the tilt guard has a level reference, or give up and stop the run.

        This script starts its own copy of the server, so the very first tool call lands a
        fraction of a second after that server opened the serial port -- usually before the
        25 still IMU samples that capture the reference have arrived. The server says so
        plainly ("needs ~0.5 s of a still, live IMU") and that is a wait, not a fault.

        It is deliberately not a sleep-and-hope: the reference is what the guard measures
        lean against, so if it never arrives -- dead IMU, or a robot that is being carried
        while it tries to capture one -- the run stops here rather than driving with the
        guard inactive.
        """
        t = await self.guard_check()
        if t.get("ready") or t.get("mode") == "off":
            return t
        print("waiting for the tilt guard's level reference -- leave the robot still "
              "on level ground")
        t0 = time.monotonic()
        while True:
            if t.get("sensor_live") is False:
                raise Stop("no live IMU, so nothing can capture a level reference or "
                           "measure lean. Check the IMU board's power and link.")
            if time.monotonic() - t0 >= REF_WAIT_S:
                raise Stop(
                    f"the tilt guard still has no level reference after {REF_WAIT_S:.0f} s "
                    f"(tilt rate {t.get('tilt_rate_dps')} deg/s). It captures one from "
                    f"~0.5 s of a STILL IMU, so either the robot is being moved, or the "
                    f"IMU is noisy enough to never look still. With the robot sitting on "
                    f"level ground, set_level_reference(confirmed_level=true) takes it "
                    f"directly.")
            await asyncio.sleep(0.2)
            t = await self.guard_check()
            if t.get("ready"):
                pr = t.get("reference_pitch_roll_deg")
                print(f"    level reference captured ({t.get('reference_source')}) "
                      f"at pitch/roll {pr} after {time.monotonic()-t0:.1f} s")
                return t

    def ask_operator(self, desc):
        if self.a.no_ask:
            return True
        print(f"\n  MOVE: {desc}")
        try:
            return input("  type yes to run it (anything else stops the run): ").strip().lower() == "yes"
        except (EOFError, KeyboardInterrupt):
            print()
            return False

    async def read_yaw(self):
        """IMU yaw now, in degrees. The IMU is one of the two instruments left, so a dead
        one ends the run here rather than at the next turn."""
        t = await self.call("read_telemetry")
        if not t.get("ok"):
            raise Stop(f"cannot read telemetry: {t.get('error') or t.get('raw')}")
        if not t.get("imu_live"):
            raise Stop("the IMU has stopped reporting. With no encoders it is the only "
                       "thing that can measure a turn or a veer, and the tilt guard reads "
                       "it too. Check the IMU board's power and link.")
        return t.get("yaw_deg")

    async def read_range(self):
        r = await self.call("read_range")
        if not r.get("live"):
            raise Stop("forward range sensor is not reporting -- that is blind, not clear. "
                       "With no encoders it is the only instrument that can measure "
                       "whether the robot moved, so this ends the run.")
        chans = r.get("channels") or {}
        latest = [c.get("latest_mm") for c in chans.values()
                  if c.get("latest_mm") is not None and c.get("latest_in_range")]
        near = min(latest) if latest else None

        # Disagreement ALONE is not a fault. A thread across a lens and a table leg beside
        # a gap under the table produce the same instantaneous signature: one cone near,
        # one far. What separates them is motion -- the thread on this robot read 92-167 mm
        # across 22 in of travel and a 35 deg spin and never changed, while a table leg's
        # range tracked the robot move for move. So the check is temporal: flag a channel
        # that disagrees wildly AND refuses to move while the robot does.
        if len(latest) == 2 and max(latest) > 5 * min(latest):
            if (self.last_near is not None and self.travel_since_near >= 6.0
                    and abs(near - self.last_near) < 10):
                raise Stop(
                    f"range channels disagree {min(latest)} vs {max(latest)} mm, and the "
                    f"near one has not moved ({self.last_near} -> {near} mm) across "
                    f"{self.travel_since_near:.0f} in of COMMANDED travel. Either a lens "
                    f"is obstructed or the robot is not moving at all -- with no encoders "
                    f"this cannot tell which, and both want the same thing: look at the "
                    f"robot. Check both lenses first.")
            print(f"    cones disagree ({min(latest)} vs {max(latest)} mm) -- normal for "
                  f"an object beside a gap; watching whether the near one tracks motion")
        if near is not None and (self.last_near is None or self.travel_since_near >= 6.0):
            self.last_near, self.travel_since_near = near, 0.0
        return r, near

    async def jog_straight(self, inches, direction):
        """An open-loop timed straight move. The only straight move left when there is
        nothing in range to close a loop on.

        `jog` applies no direction-aware sign correction, unlike drive_distance: positive
        PWM is not necessarily forward, so the sign comes from the firmware convention in
        drive_math. require_telemetry is passed explicitly rather than left to the server's
        policy -- this script's whole premise is that wheel feedback is not available, and
        the refusal it would otherwise produce names the encoders as the problem.

        The returned dist_l_in/dist_r_in are deliberately not read. On this run they are
        null, and if the encoder link happens to be alive they are the one number here
        that nothing has decided to trust.
        """
        dur = jog_seconds(inches, self.a.pwm)
        sign = dm.FORWARD_PWM_SIGN * (1 if direction == "forward" else -1)
        m = await self.call("jog", left_pwm=sign * self.a.pwm, right_pwm=sign * self.a.pwm,
                            duration_s=round(dur, 2), confirmed_safe=True,
                            require_telemetry=False)
        if not m.get("ok"):
            raise Stop(f"jog refused: {m.get('error') or m.get('raw')}")
        return m, dur

    async def decide_straight(self, cmd, elapsed_s, rng_before, rng_after, yaw_drift,
                              expected_in=None, tapered=False, measured=None):
        """Ask the router about a straight move, from the range sensor and the IMU alone.

        The question set comes from dc.blind_questions and shrinks with the state: with no
        valid range reading there is no displacement channel, so `progress` and `cause` are
        not asked at all. That is the whole reason `unmeasured` can still mean something
        here -- it is reserved for a channel that was there and has gone bad, rather than
        being the honest answer to every move across an empty floor.
        """
        range_valid = rng_before is not None and rng_after is not None
        v_in_s = dm.speed_for_pwm(self.a.pwm)
        bits = [f"This robot measures {v_in_s:.1f} in/s at pwm {self.a.pwm}"]
        bits.append("." if expected_in is None else
                    f", so {expected_in:.1f} in should take about "
                    f"{expected_in / v_in_s:.1f} s.")
        if tapered:
            # drive_until_range slows as it closes and finishes in <=1 s pulses BY DESIGN,
            # so a correct approach is genuinely slow and must not be scored as a fault.
            bits.append(" This move slows deliberately as it closes on its target and "
                        "finishes in short pulses, so its average speed is expected to be "
                        "well below that.")
        if yaw_drift is not None:
            # Without this the heading question has no scale to answer against. Open-loop
            # skid-steer driving curves off line by itself (ROBOT_CHARACTERIZATION.md), so
            # a few degrees is the drivetrain working, not a drag on one side.
            bits.append(" Open-loop straight driving on this skid-steer robot curves off "
                        "line on its own, so a few degrees of yaw across a move is normal; "
                        "a large swing is not.")
        state = dc.build_state(cmd, elapsed_s, expected_in=expected_in,
                               range_before_mm=rng_before, range_after_mm=rng_after,
                               range_valid=range_valid, blind=True,
                               yaw_drift_deg=yaw_drift, extra="".join(bits))
        qs = dc.blind_questions(range_valid, yaw=yaw_drift is not None)
        answers, rtt = dc.ask(state, model=self.a.model, questions=qs)
        v = dc.verdict(answers)
        self.log_verdict(cmd, "straight", state, v, rtt, measured=measured)
        if not range_valid:
            print("    (no displacement channel for this move: nothing was in range before "
                  "or after, so progress was not asked)")
        # Only stop on an `unmeasured` the model actually believes. Measured on the encoder
        # version: the three real ones -- nothing measured at all, or no encoder travel in
        # the state -- came in at 1.00, 0.94 and 0.92, while weaker ones were the `cause`
        # question having no category that fit rather than a sensor fault.
        if v["cause"] == "unmeasured" and v["confidence"] >= UNMEASURED_STOP_CONF:
            raise Stop("the model cannot tell what happened (%.2f) -- the range sensor has "
                       "gone bad, and it is the only distance instrument on this run. Look "
                       "at the robot rather than driving it further." % v["confidence"])
        if v["cause"] == "unmeasured":
            print(f"    (ignoring `unmeasured` at {v['confidence']:.2f}: under the "
                  f"{UNMEASURED_STOP_CONF:.2f} gate, so it is a guess, not a reading)")
        return v

    async def decide_spin(self, cmd, move, expected_deg):
        """Ask the router about a turn. Unchanged by the loss of the encoders: the
        encoders never measured a spin on this robot -- wheel scrub makes them over-read
        one by ~1.7-2x, by a factor that moves with the surface -- so turn_degrees was
        always IMU-only and the spin state and questions are exactly as they were.

        No range term: the forward cones sweep the room during a rotation, so a range
        change says nothing about displacement.
        """
        state = dc.build_state(cmd, move.get("elapsed_s") or 0.0, range_valid=False,
                               kind="spin", expected_deg=expected_deg,
                               # Magnitude: the direction is already in the command text,
                               # and "expected about 98 deg, IMU reports -100.2" invites
                               # the model to read a sign convention as a mismatch.
                               imu_deg=abs(move.get("actual_degrees_turned") or 0.0))
        answers, rtt = dc.ask(state, model=self.a.model,
                              questions=dc.questions_for(False, "spin"))
        v = dc.verdict(answers)
        self.log_verdict(cmd, "spin", state, v, rtt)
        return v

    def log_verdict(self, cmd, kind, state, v, rtt, measured=None):
        bits = []
        if v["cause"] is not None:
            bits.append(f"{v['cause']} ({v['confidence']:.2f})")
        if v["progress"] is not None:
            bits.append(f"progress {v['progress']:.2f}")
        if v["heading"] is not None:
            bits.append(f"heading {v['heading']:.2f}")
        bits.append(f"retry {v['retry']:.2f}")
        print(f"    -> {'  '.join(bits)}  [{rtt*1000:.0f} ms]")
        self.rows.append(dict(move=self.moves, t_s=round(time.monotonic()-self.t0, 2),
                              cmd=cmd, kind=kind, state=state,
                              **{k: v[k] for k in ("progress", "heading", "retry",
                                                   "cause", "confidence")},
                              rtt_ms=round(rtt*1000),
                              **{k: (measured or {}).get(k) for k in MEASURED}))

    def log_facts(self, cmd, kind, measured, note):
        """Log a move that ended the run before anything could judge it.

        The approach that stopped the last run left no row at all: the row is written
        after the verdict comes back, and that move never got one. The move that ends a
        run is the one most worth having in the file, so it goes in with the measurements
        it does have, `note` where the model's state would be, and no verdict.
        """
        print(f"    -> no verdict: {note}")
        self.rows.append(dict(move=self.moves, t_s=round(time.monotonic()-self.t0, 2),
                              cmd=cmd, kind=kind, state=note,
                              progress=None, heading=None, retry=None, cause=None,
                              confidence=None, rtt_ms=None,
                              **{k: (measured or {}).get(k) for k in MEASURED}))

    async def one_move(self):
        self.moves += 1
        await self.guard_check()
        r, closest = await self.read_range()
        print(f"\n[move {self.moves}] closest "
              f"{closest if closest is not None else 'nothing in range'} mm")
        yaw0 = await self.read_yaw()

        if closest is None or closest > self.a.target_mm + 150:
            if closest is None:
                # Open floor as far as the cones reach. This is the one move with nothing
                # watching it, so it is short and the next range read is its only check.
                step = self.a.step_in
                dur = jog_seconds(step, self.a.pwm)
                desc = (f"jog({step} in ~= {dur:.2f} s, forward, pwm {self.a.pwm}) -- "
                        f"nothing in range, open-loop")
                if not self.ask_operator(desc):
                    raise Stop("not approved")
                m, dur = await self.jog_straight(step, "forward")
                exp, tapered, bumped, failed = step, False, False, False
                arrived = False
                measured = None   # a jog measures nothing; the next range read is its check
                self.travel_since_near += step
            else:
                desc = (f"drive_until_range({self.a.target_mm}mm, pwm {self.a.pwm}) -- "
                        f"now {closest} mm")
                if not self.ask_operator(desc):
                    raise Stop("not approved")
                m = await self.call("drive_until_range", target_mm=self.a.target_mm,
                                    pwm=self.a.pwm, confirmed_safe=True)
                exp = (closest - self.a.target_mm) / 25.4
                tapered = True
                self.travel_since_near += abs((m.get("range_closed_mm") or 0.0) / 25.4)
                measured = approach_facts(m)
                try:
                    bumped, failed, arrived = self.classify_approach(m)
                except Stop as e:
                    # An approach that ends the run still measured something, and it is
                    # the row the next person will want. Log it, then stop as before.
                    self.log_facts(f"drive forward at pwm {self.a.pwm}", "straight",
                                   measured, str(e))
                    raise

            _, after = await self.read_range()
            yaw1 = await self.read_yaw()
            v = await self.decide_straight(
                f"drive forward at pwm {self.a.pwm}", m.get("elapsed_s") or 0.0,
                closest, after, yaw_delta(yaw0, yaw1), expected_in=exp, tapered=tapered,
                measured=measured)

            # The model's own verdict, on top of whatever the approach loop reported. With
            # no range reading it has no displacement channel and `progress` is None, so
            # there is nothing to add: the next move's range read is the check.
            if v["progress"] is not None and v["progress"] < self.a.progress_floor:
                bumped = failed = True
            if v["cause"] == "not_advancing":
                bumped = failed = True
            # `veering` is logged and not acted on. Open-loop skid-steer driving curves off
            # line by itself, and this loop is covering floor, not following a line -- a
            # heading threshold would have to be measured on this robot before it could
            # mean "something is dragging" rather than "the drivetrain is normal".
            if v["cause"] == "veering":
                print("    (veering: logged, not acted on -- open-loop driving on this "
                      "robot curves off line by itself)")
            # An approach that reached its target has parked the robot a few inches off a
            # wall facing it. That needs the big turn, exactly like a bump does -- a small
            # turn here just points it at the same wall again.
            if arrived and not failed:
                print("    arrived at the target range -> treat as an obstacle, turn away")
                bumped = True
        else:
            print("    already at the wall")
            bumped, failed = True, False

        if bumped:
            # Arriving at a wall on purpose is not a failure, so it must not trip the
            # give-up counter -- otherwise a tidy room stops the run after two walls.
            self.no_progress = self.no_progress + 1 if failed else 0
            if self.no_progress >= 2:
                raise Stop("two moves in a row made no progress even after backing off")
            print(f"    bumped or stuck -> back off {self.a.backoff_in} in and turn")
            dur = jog_seconds(self.a.backoff_in, self.a.pwm)
            desc = f"jog({self.a.backoff_in} in ~= {dur:.2f} s, reverse, pwm {self.a.pwm})"
            if not self.ask_operator(desc):
                raise Stop("not approved")
            # Open-loop and unmeasured: the ToF faces forward, so nothing sees what the
            # robot is backing into. Keep --backoff-in small for that reason.
            await self.jog_straight(self.a.backoff_in, "reverse")
            self.travel_since_near += self.a.backoff_in
            deg, direction = self.rng.uniform(70, 150), self.rng.choice(["left", "right"])
        else:
            self.no_progress = 0
            deg, direction = self.rng.uniform(20, 60), self.rng.choice(["left", "right"])

        await self.guard_check()
        desc = f"turn_degrees({deg:.0f}, {direction}, pwm {self.a.spin_pwm})"
        if not self.ask_operator(desc):
            raise Stop("not approved")
        t = await self.call("turn_degrees", degrees=round(deg), direction=direction,
                            pwm=self.a.spin_pwm, confirmed_safe=True)
        if not t.get("ok"):
            raise Stop(f"turn failed: {t.get('abort_reason') or t.get('error') or t.get('raw')}")
        print(f"    turned {t.get('actual_degrees_turned')} deg (IMU)")
        await self.decide_spin(
            f"spin in place to the {direction} at pwm {self.a.spin_pwm}", t, deg)

    def classify_approach(self, m):
        """-> (bumped, failed, arrived) for a drive_until_range result.

        The encoder version stopped the run on any not-ok approach. That was the encoders
        talking: they were what distinguished "the wall arrived" from "the drivetrain is
        broken". Without them, three of the four aborts are simply what a bump looks like
        from the range sensor's side, and treating them as faults would end most runs on
        the first chair leg. The fourth -- the sensor going quiet -- is still a fault, and
        a worse one than before, because nothing else measures distance now.
        """
        if m.get("ok"):
            return False, False, bool(m.get("within_tolerance"))
        reason = (m.get("abort_reason") or m.get("error") or "")
        if any(k in reason for k in BLIND_ABORTS):
            raise Stop(f"the range sensor went quiet mid-approach ({reason}). It is the "
                       f"only distance instrument on this run, so this ends it.")
        if m.get("hit_timeout") or any(k in reason for k in BUMP_ABORTS):
            print(f"    approach aborted: {reason or 'timed out'}")
            print("    -> reading that as a bump, not a fault")
            return True, True, False
        raise Stop(f"drive_until_range failed in a way this loop does not classify: "
                   f"{reason or m.get('raw')}")

    async def confirm_no_encoders(self):
        """Drop "enc" from the server's required sensors, after telling the operator what
        that costs and getting a typed go-ahead.

        set_required_sensors' own docstring asks for exactly this: confirmed_safe "only
        after the operator has agreed to drive this way". The agreement is one typed
        phrase covering both this and --no-ask, rather than two prompts a few lines apart
        that get answered the same way without being read.
        """
        s = await self.call("sensors")
        if not s.get("ok", True) and s.get("error"):
            raise Stop(f"cannot read sensors: {s['error']}")
        streams = s.get("streams") or {}
        print(f"\nsensors reporting: {', '.join(sorted(streams)) or '(none)'}")
        print("\nThis run drives with NO WHEEL ENCODERS. Dropping \"enc\" from the")
        print("required-sensor set means the drive tools will move the robot with the")
        print("encoder link dead or absent, and these become true:")
        print("  * a straight move across open floor is open-loop and unverified")
        print("  * nothing can tell a stalled drivetrain from a blocked one")
        print("  * the forward ToF is the only instrument measuring distance")
        if self.a.no_ask:
            print("\n--no-ask: the robot will also move repeatedly with no prompt between")
            print("moves. This needs an enclosed, flat, drop-free space and you watching")
            print("it, with a hand near the power switch. DRIVING_POLICY.md asks per move")
            print("by default.")
            want = "i am watching it"
        else:
            want = "yes"
        if input(f'\ntype "{want}" to drive this way: ').strip().lower() != want:
            raise Stop("not confirmed")
        res = await self.call("set_required_sensors", names=REQUIRED_SENSORS,
                              confirmed_safe=True)
        if not res.get("ok"):
            raise Stop(f"could not set the required sensors: "
                       f"{res.get('error') or res.get('raw')}")
        print(f"required sensors: {res.get('was')} -> {res.get('required')}")
        for cost in res.get("costs") or []:
            print(f"    cost: {cost}")
        if res.get("missing"):
            raise Stop(f"still missing a required sensor: {res['missing']}. "
                       f"{REQUIRED_SENSORS} is the minimum this loop can run on -- the IMU "
                       f"for turns and the tilt guard, a ToF for distance.")
        if not res.get("can_drive"):
            raise Stop(f"server will not drive: {res.get('why_not')}")

    async def go(self):
        g = await self.call("set_guard", name="tilt", mode="on",
                            lookahead_s=self.a.lookahead_s)
        print(f"tilt guard: {json.dumps(g)[:160]}")
        await self.wait_for_level_reference()
        await self.confirm_no_encoders()
        # Both instruments, proven live, before the first command rather than at the first
        # move that needs them.
        await self.read_range()
        await self.read_yaw()
        try:
            while True:
                if self.moves >= self.a.max_moves:
                    print(f"\nreached --max-moves {self.a.max_moves}")
                    return
                if (time.monotonic() - self.t0) / 60 >= self.a.max_minutes:
                    print(f"\nreached --max-minutes {self.a.max_minutes}")
                    return
                await self.one_move()
        finally:
            await self.call("stop")


async def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pwm", type=int, default=60, help="straight-line PWM (default 60)")
    ap.add_argument("--spin-pwm", type=int, default=80,
                    help="spins need more than straights: below ~60 they stall in place")
    ap.add_argument("--target-mm", type=int, default=400, help="how close to a wall to stop")
    ap.add_argument("--step-in", type=float, default=9.0,
                    help="open-loop forward step when nothing is in range (default 9; it "
                         "was 18 when an encoder closed the loop on it)")
    ap.add_argument("--backoff-in", type=float, default=6.0,
                    help="open-loop reverse, with no sensor facing that way -- keep it small")
    ap.add_argument("--progress-floor", type=float, default=0.5,
                    help="p(progress) below this counts as a bump, when progress was asked")
    ap.add_argument("--lookahead-s", type=float, default=0.4)
    ap.add_argument("--max-moves", type=int, default=20)
    ap.add_argument("--max-minutes", type=float, default=15.0)
    ap.add_argument("--model", default=dc.DEFAULT_MODEL)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--log", help="write every move and verdict to this CSV")
    ap.add_argument("--no-ask", action="store_true",
                    help="do not ask before each move (needs a typed confirmation)")
    a = ap.parse_args()

    if a.pwm > 80 or a.spin_pwm > 100:
        sys.exit("refusing: this loop bumps into walls on purpose, so keep pwm <= 80 "
                 "(straight) and <= 100 (spin). Raise these in the source deliberately.")
    if dm.speed_for_pwm(a.pwm) is None:
        sys.exit(f"pwm {a.pwm} is below the measured deadband of {dm.PWM_DEADBAND}: the "
                 f"robot does not move at all there, and an open-loop move has nothing "
                 f"to notice that with.")
    if not os.environ.get("LLAMA_ROUTER_KEY"):
        sys.exit("set LLAMA_ROUTER_KEY")

    params = StdioServerParameters(command=sys.executable, args=[SERVER])
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            run = Run(session, a)
            try:
                await run.go()
            except Stop as e:
                print(f"\n*** STOPPED: {e}")
            except KeyboardInterrupt:
                print("\n*** ctrl-c")
                await run.call("stop")
            finally:
                print(f"\n{run.moves} moves in {(time.monotonic()-run.t0)/60:.1f} min")
                if a.log and run.rows:
                    with open(a.log, "w", newline="") as fh:
                        w = csv.DictWriter(fh, fieldnames=list(run.rows[0]))
                        w.writeheader(); w.writerows(run.rows)
                    print(f"{len(run.rows)} verdicts -> {a.log}")


if __name__ == "__main__":
    asyncio.run(main())
