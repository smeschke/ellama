#!/usr/bin/env python3
"""One place that asks llama-router about the move that just happened.

Shared by decide.py (one call from the shell) and explore_bump_turn.py (a call between
every move). The criteria and question wordings live here because llama-router keys a
calibration on the model plus the exact question text: changing a string here silently
orphans any calibration fitted against it.

What today's testing on this robot settled about how to build the state:

* Never pass a channel that cannot measure right now. Feeding a stale range median while
  the ToF was out of range turned a healthy 13.37 in move into "blocked" at 0.59
  confidence with "safe to repeat" at 0.148. Saying the channel is unavailable gave
  "fine" and progress 0.940 from the same move.
* Annotating a bad number does not neutralise it: "left wheel 0.00 in" plus "that channel
  is frozen" still returned one_side at 0.66. Omit the number.
* Every state needs its own criterion. A missing category does not come back as
  uncertainty, it comes back as a confident wrong label -- three times in one afternoon.
* `unmeasured` is why this beats a threshold. A threshold always returns a number; this
  says "I cannot tell" at 0.94, which is what a frozen encoder and a thread over a lens
  both needed.
"""

import json, os, time, urllib.error, urllib.request

URL = os.environ.get("LLAMA_ROUTER_URL", "https://llama-router.pdxhackerspace.org")
DEFAULT_MODEL = os.environ.get("LLAMA_ROUTER_MODEL", "unsloth/Qwen3.8-27B-NVFP4")

CRITERIA = {
    "stalled": "the motors are loaded but the wheels are not turning",
    "blocked": "the wheels are turning but the robot is not advancing",
    "one_side": ("one side's wheels are measured to be turning and the other side's are "
                 "measured to be stopped"),
    "slow": "the robot is advancing, but well short of the commanded speed",
    "fine": "the robot is moving normally, close to the commanded speed",
    # Without this option the model invents a mechanical fault instead of admitting it
    # cannot tell: one_side at 0.37 became unmeasured at 0.94 once this existed.
    "unmeasured": ("a sensor channel is not reporting, so whether that part of the "
                   "drivetrain is turning is simply unknown"),
}

# "measured to be turning" rather than "are turning": the tighter wording moved the
# answer on a frozen-channel state from 0.294 to 0.085, which is the honest reading.
# The gate question must match the move. "The robot is making progress" asks about
# TRANSLATION, so a textbook 112.6-for-110 deg spin scored 0.469 on it -- under the 0.5
# gate, for a perfect turn. Asking about rotation instead gave 0.999, against 0.012 for a
# stalled spin and 0.047 for a half turn. Same question id either way, so callers and any
# fitted calibration keyed on the wording both stay straight.
# --- the same job on a robot with no wheel sensor at all ---------------------------
# A second criteria set rather than an edit of the one above. decide.py still runs on a
# robot whose encoders work, and llama-router keys a calibration on the model plus the
# exact question text, so these two wordings have to coexist.
#
# Losing the encoders does not lower the confidence of the answers, it changes which
# questions have an answer at all:
#   * `stalled` and `blocked` collapse into one. "Loaded but the wheels are not turning"
#     and "the wheels are turning but the robot is not advancing" differ only in what the
#     wheels did, which is precisely what is no longer measured. Offering both would make
#     the model pick one, and this module's docstring already records the price of a
#     category the state cannot support: a confident wrong label.
#   * `one_side` goes entirely. Nothing reports per-side anything any more.
#   * `veering` arrives, because the IMU still measures it. Note what it is NOT: open-loop
#     straight driving curves off line on any skid-steer robot by itself
#     (ROBOT_CHARACTERIZATION.md, "Timing and heading"), so a few degrees is the drivetrain
#     working normally, which is why explore_bump_turn.py logs this answer and does not
#     steer on it.
BLIND_CRITERIA = {
    "not_advancing": ("the robot is not getting closer to what the forward range sensor "
                      "is measuring, and with no wheel sensor there is no way to tell "
                      "whether the wheels are turning"),
    "veering": ("the robot is advancing, but the IMU shows its heading swinging well "
                "beyond the drift of a normal straight move"),
    "slow": "the robot is advancing, but well short of the commanded speed",
    "fine": "the robot is moving normally, close to the commanded speed",
    "unmeasured": ("the forward range sensor could not measure this move, so whether the "
                   "robot advanced is simply unknown"),
}

GATE_STRAIGHT = "The robot is making progress"
GATE_SPIN = "The robot is rotating as commanded"

QUESTIONS = {
    "progress": {"type": "noul", "instructions": GATE_STRAIGHT},
    "wheels": {"type": "noul",
               "instructions": "Both sides of the drivetrain are measured to be turning"},
    "retry": {"type": "noul",
              "instructions": "It is safe to send another move like this one"},
    # Only fair to ask when a yaw figure is in the state. On a robot with no encoders this
    # is what is left of "both sides of the drivetrain are turning": a side that drags
    # cannot be measured any more, but it still swings the nose.
    "heading": {"type": "noul",
                "instructions": "The robot held its heading during this move"},
    "cause": {"type": "choice", "instructions": "What is happening", "criteria": CRITERIA},
}


def build_state(cmd, elapsed_s, expected_in=None, enc_in=None,
                range_before_mm=None, range_after_mm=None, range_valid=True,
                imu_deg=None, expected_deg=None, extra=None,
                kind="straight", enc_left_in=None, enc_right_in=None,
                blind=False, yaw_drift_deg=None):
    """A state carrying only channels that can measure. See the module docstring.

    `blind` is for a run driving with no wheel encoders at all. It is not the same as
    having no encoder figure to pass: "no wheel encoder channel is reporting travel"
    describes a channel that should be there and is not, which is a fault, and a state
    that says it gets `unmeasured` back. A robot that was never going to measure wheel
    rotation on this run has to say so, the same way the spin wording does.

    `kind` matters more than it looks. For a spin, absent encoder travel is not a missing
    channel -- the encoders do not measure rotation on this robot at all, because the
    wheels scrub (turn_degrees' own docstring: "the encoders measure arcs and straight
    lines; they do not measure spins"). Saying "no encoder is reporting" for a spin reads
    as a sensor fault and returned `unmeasured` at 0.98, which would halt a run on every
    turn. Naming the IMU as the right instrument instead is the honest state.
    """
    parts = [f"Command: {cmd}, ran for {elapsed_s:.2f} s."]
    if expected_in is not None:
        parts.append(f"Expected {expected_in:.1f} in.")
    if kind == "spin":
        parts.append("Wheel encoders do not measure rotation on this robot, because the "
                     "wheels scrub during a spin; the IMU is the instrument for this "
                     "move and it is reporting.")
    elif blind:
        parts.append("This robot is driving with no wheel encoders on this run: wheel "
                     "rotation is not measured at all, so distance comes from the forward "
                     "range sensor and heading from the IMU.")
    elif enc_left_in is not None and enc_right_in is not None:
        parts.append(f"Left wheel encoder {enc_left_in:.2f} in, right wheel encoder "
                     f"{enc_right_in:.2f} in.")
    elif enc_in is not None:
        parts.append(f"Wheel encoder reports {enc_in:.2f} in of travel.")
    else:
        parts.append("No wheel encoder channel is reporting travel for this move.")
    if expected_deg is not None:
        parts.append(f"Expected about {expected_deg:.0f} deg of rotation.")
    if imu_deg is not None:
        parts.append(f"IMU gyro reports {imu_deg:.1f} deg of yaw.")
    if yaw_drift_deg is not None:
        # A straight move's heading change, as a magnitude: which way it drifted is not
        # what the question asks, and a sign invites the model to read a convention as a
        # fault (the same reason explore_bump_turn passes abs() for a spin).
        parts.append(f"IMU yaw changed {abs(yaw_drift_deg):.1f} deg across the move.")
    if range_valid and range_before_mm is not None and range_after_mm is not None:
        # Signed, and phrased per direction: a reverse move makes the forward range GROW,
        # and "advanced -8.6 in" is not a sentence anyone should have to interpret.
        adv = (range_before_mm - range_after_mm) / 25.4
        moved = (f"so the robot moved {adv:.2f} in closer to what that sensor is "
                 f"measuring" if adv >= 0 else
                 f"so the robot moved {-adv:.2f} in further away from what that sensor "
                 f"is measuring")
        parts.append(f"Forward range sensor went {range_before_mm:.0f} mm to "
                     f"{range_after_mm:.0f} mm, {moved}.")
    else:
        parts.append("The forward range sensor cannot measure advance for this move and "
                     "is therefore not given.")
    if extra:
        parts.append(extra)
    return " ".join(parts)


def questions_for(per_side, kind="straight", yaw=False):
    """The `wheels` question is only fair to ask when the state actually carries per-side
    figures. Asked without them it answered about information it had not been given --
    0.10 on a healthy move -- which is noise, not a signal. `heading` goes the same way,
    with one extra reason to be careful: a spin is SUPPOSED to change the heading, so
    asking whether the robot held it would score a textbook turn near zero."""
    qs = {k: dict(v) for k, v in QUESTIONS.items()}
    if not per_side:
        qs.pop("wheels")
    if not yaw or kind == "spin":
        qs.pop("heading")
    if kind == "spin":
        qs["progress"]["instructions"] = GATE_SPIN
    return qs


def blind_questions(range_valid, yaw=True):
    """The questions a no-encoder STRAIGHT move can support, and no others.

    Spins do not come through here. `turn_degrees` is IMU-only and the encoders never
    measured a spin on this robot, so a spin state and its questions are the same with the
    encoders gone as with them fitted -- use questions_for(False, "spin").

    With no valid range reading there is no displacement channel left, so `progress` and
    `cause` are dropped rather than answered from nothing. That keeps `unmeasured` meaning
    what it meant before -- a channel that was there has gone bad -- instead of becoming
    the honest answer on every move across open floor, which would halt a run a minute in.
    """
    qs = {}
    if range_valid:
        qs["progress"] = {"type": "noul", "instructions": GATE_STRAIGHT}
    if yaw:
        qs["heading"] = dict(QUESTIONS["heading"])
    qs["retry"] = dict(QUESTIONS["retry"])
    if range_valid:
        qs["cause"] = {"type": "choice", "instructions": "What is happening",
                       "criteria": BLIND_CRITERIA}
    return qs


def ask(state, model=DEFAULT_MODEL, key=None, timeout=60, questions=None):
    """-> (answers dict, round-trip seconds). Raises RuntimeError on an HTTP error."""
    key = key or os.environ.get("LLAMA_ROUTER_KEY")
    if not key:
        raise RuntimeError("set LLAMA_ROUTER_KEY (dashboard -> My API keys)")
    if questions is None:
        questions = questions_for(
            "Left wheel encoder" in state,
            "spin" if "do not measure rotation" in state else "straight",
            yaw="IMU yaw changed" in state)
    body = json.dumps({"model": model, "state": state, "questions": questions}).encode()
    req = urllib.request.Request(f"{URL}/v1/systemone", body,
                                 {"Content-Type": "application/json", "API-KEY": key})
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r)["answers"], time.monotonic() - t0
    except urllib.error.HTTPError as e:
        raise RuntimeError(
            f"HTTP {e.code}: {e.read()[:200].decode('utf-8', 'replace')}") from None


def verdict(answers):
    """The numbers a caller acts on, plus the diagnosis. Any of them is None when the
    question was not asked, because a question dropped for want of a channel to answer it
    is the normal case here, not an error -- see blind_questions."""
    def noul(name):
        return answers[name]["noul"] if name in answers else None
    cause = answers.get("cause")
    return dict(progress=noul("progress"), wheels=noul("wheels"),
                heading=noul("heading"), retry=noul("retry"),
                cause=cause["choice"] if cause else None,
                confidence=cause["confidence"] if cause else None,
                probabilities=cause["probabilities"] if cause else None)
