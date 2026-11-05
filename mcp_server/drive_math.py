#!/usr/bin/env python3
"""Pure drive maths for the MCP robot server: PWM/speed curve, encoder
selection, frozen-wheel detection and the range-approach taper.

No I/O and no robot state here -- everything is a function of its arguments
(plus the measured constants below), so it is unit-testable without hardware.
Extracted from robot_server.py; robot_server re-exports these names.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import bridge  # noqa: E402

# ---------------------------------------------------------------------------

# "Scripted ground-truth-run ceiling" in DRIVING_POLICY.md -- same value the deleted
# toandfro.py used for encoder-verified runs where the robot actually rolls.
MAX_PWM_CEILING = 125

# True gear ratio confirmed by calibration/30_inches.csv (a real tape-measured 30in
# straight run) -- see calibration/analyze_run.py's TRUE_GEAR_RATIO and its docstring.
# calibration/config.json still says 6.33 and is being left as-is for now, so this
# overrides it in memory only; it does not touch the file.
TRUE_GEAR_RATIO = 4.5

# A stream with no packet for this long is treated as silent. Everything reports at 5 Hz
# (ToF) or 50 Hz (encoders, IMU), so this is many missed packets for any of them.
TELEMETRY_STALE_S = 2.0

# Measured PWM -> steady forward speed, ROBOT_CHARACTERIZATION.md section 1 (sweep of
# 2026-09-27, reproduced within ~4% on 09-29). Used ONLY to time an open-loop move when the
# encoders are absent and the operator has said that is acceptable -- it is a prediction
# from a curve, never a measurement, and every result that uses it says so.
PWM_SPEED_IN_S = ((50, 4.11), (60, 6.13), (80, 9.36), (100, 12.03), (125, 14.46))

# No TURN_SCRUB_FACTOR here any more. It existed to rescue an encoder-measured in-place
# turn by dividing out ~2x of wheel scrub, but scrub varies with the surface, so the
# divisor was only ever right on the floor it was calibrated on. turn_degrees now requires
# the IMU rather than correcting a measurement it should not be taking: operator policy,
# 2026-10-24 -- encoders measure arcs and straight lines on this robot, never spins.
# Below this the robot did not move at all in either direction (pwm 40: 0.02 in/s).
PWM_DEADBAND = 45

# Which wheels a distance is measured by. "both" averages them, which is the distance the
# chassis covered; a single wheel is the same number on a straight line and the only one
# available when the other channel is dead. Offered on the straight and arc tools only --
# during a spin in place the wheels turn opposite ways and scrub, so neither one wheel nor
# their average is a distance the robot travelled (see turn_degrees, which uses the IMU).
ENCODER_SELECTIONS = ("both", "left", "right")
WHEEL_NAMES = ("left", "right")

# A channel that was already frozen when a move started is not a dropout: last_counts()
# keeps handing back the same number, so the loop would drive to its timeout chasing a
# target it can never measure (encoder="left"/"right"), or to twice the distance asked
# because the average of a moving wheel and a dead one rises at half rate
# (encoder="both"). Real motion ticks fast -- ~587 ticks/in, so even 1 in/s is hundreds of
# counts a second -- so anything that has not moved by now is not moving.
ENCODER_FIRST_TICK_S = 0.6

# The timer above is the backstop, not the main check: a short move measured by an average
# that includes a dead channel reaches its doubled target in a fraction of a second, so the
# fault has to be caught the moment the two wheels disagree instead. One selected wheel
# having covered this much while another has not ticked at all is that disagreement.
FROZEN_COMPARE_IN = 0.25

# --- range-terminated approach (drive_until_range) -------------------------------------
# Measured 5.3-5.8 Hz on TOF_FRONT; the slow end is the safe one to plan against.
RANGE_SAMPLE_HZ = 5.0
# Brake and settle after the last command the loop sends.
RANGE_STOP_LEAD_S = 0.2
# Steering reads the MEDIAN of the last few samples (see TofState.median_mm), which is the
# right call for noise but lags while the robot is moving: half the window, by definition.
# Observed 2026-10-22 backing up at ~170 mm/s, median 355 mm against a raw 257 mm.
RANGE_MEDIAN_LAG_SAMPLES = bridge.TofState.HISTORY // 2
# How out of date the number the loop is steering by can be once the wheels have stopped.
RANGE_LATENCY_S = (1.0 + RANGE_MEDIAN_LAG_SAMPLES) / RANGE_SAMPLE_HZ + RANGE_STOP_LEAD_S
# speed_for_pwm is a prediction off a curve, not a measurement, so leave headroom rather
# than driving at exactly the speed the arithmetic says is stoppable.
# Raised 1.25 -> 5.0 on 2026-10-24. At 1.25 the taper started far too late to ever finish:
# four approaches to a wall (targets 250/100/150/150, pwm 60-110) all ended by the tyres
# touching it at ~102mm, and every one reported pulses: 0 -- the continuous phase never
# wound down far enough to hand over to the pulse endgame, so the careful part of this
# tool had never once run. pwm_used.min bottomed out at 66 on the last of them, well above
# the 60 floor it was given. The arithmetic said 1.25 was enough; the robot disagreed.
RANGE_APPROACH_MARGIN = 5.0
# After a pulse, the median needs a full window of new samples before it means anything.
RANGE_MEDIAN_REFRESH_S = (bridge.TofState.HISTORY + 1) / RANGE_SAMPLE_HZ
# Long enough that a slow creep still registers as progress, short enough that driving
# into something is caught in about the time a watching operator would call it.
RANGE_NO_PROGRESS_S = 2.0
RANGE_MIN_PULSE_S = 0.2
RANGE_MAX_PULSE_S = 1.0
RANGE_MAX_PULSES = 8
# A ToF this close is inside the robot's own bumper on most mountings, and VL53L0X
# accuracy falls apart in the last few cm.
RANGE_MIN_TARGET_MM = 50.0


def speed_for_pwm(pwm):
    """Predicted steady speed in in/s for a PWM magnitude, linearly interpolated between
    the measured points, or None below the deadband where the robot does not move."""
    pwm = abs(pwm)
    if pwm < PWM_DEADBAND:
        return None
    if pwm <= PWM_SPEED_IN_S[0][0]:
        return PWM_SPEED_IN_S[0][1] * pwm / PWM_SPEED_IN_S[0][0]
    for (p0, v0), (p1, v1) in zip(PWM_SPEED_IN_S, PWM_SPEED_IN_S[1:]):
        if pwm <= p1:
            return v0 + (v1 - v0) * (pwm - p0) / (p1 - p0)
    return PWM_SPEED_IN_S[-1][1]

def pwm_for_speed(speed_in_s):
    """The highest PWM whose predicted speed does not exceed `speed_in_s`, or None when
    even PWM_DEADBAND -- the slowest the robot moves at all -- is faster than that.

    The inverse of speed_for_pwm over the same measured curve, so the two agree by
    construction. None is the useful answer, not a failure: it means "you cannot creep
    this slowly", which is what tells an approach loop to stop and pulse instead of
    trying to drive continuously."""
    if speed_in_s <= 0:
        return None
    if speed_in_s < speed_for_pwm(PWM_DEADBAND):
        return None
    best = PWM_DEADBAND
    for p in range(PWM_DEADBAND, MAX_PWM_CEILING + 1):
        v = speed_for_pwm(p)
        if v is None or v > speed_in_s:
            break
        best = p
    return best


def selected_wheels(encoder):
    """Indices into an (left, right) pair that `encoder` names."""
    if encoder == "left":
        return (0,)
    if encoder == "right":
        return (1,)
    return (0, 1)


def traveled_inches(start, cur, encoder, in_per_tick):
    """Distance covered between two cumulative count pairs, by the selected encoder(s).

    Unsigned: every caller here already knows which way it commanded the wheels, and a
    reversed encoder would otherwise count a move backwards (see ENCODER_FORWARD_SIGN,
    which has flipped on this robot before)."""
    idx = selected_wheels(encoder)
    return sum(abs(cur[i] - start[i]) for i in idx) / len(idx) * in_per_tick


def frozen_wheels(start, cur, encoder):
    """Selected channels whose count has not changed at all since `start`."""
    return [WHEEL_NAMES[i] for i in selected_wheels(encoder) if cur[i] == start[i]]


def frozen_wheel_fault(start, cur, encoder, in_per_tick, elapsed_s):
    """Frozen selected channels, once that is a fault rather than a move still getting
    going. Either another selected wheel has already covered FROZEN_COMPARE_IN -- so the
    two disagree and one of them is wrong -- or nothing has ticked for
    ENCODER_FIRST_TICK_S, so nothing is measuring this move at all.

    Waiting only on the timer would miss the case that matters most: with encoder="both"
    and one channel dead, the average rises at half rate and a short move sails past twice
    the distance asked before the timer ever fires."""
    frozen = frozen_wheels(start, cur, encoder)
    if not frozen:
        return []
    moved_in = max(abs(cur[i] - start[i]) for i in selected_wheels(encoder)) * in_per_tick
    if moved_in >= FROZEN_COMPARE_IN or elapsed_s > ENCODER_FIRST_TICK_S:
        return frozen
    return []


def approach_pwm(remaining_mm, pwm_max, pwm_min):
    """The fastest PWM that still stops inside `remaining_mm`, or None to stop and pulse.

    The loop can only ever act on a number that is RANGE_LATENCY_S out of date, so the
    distance still to go has to cover the travel in that blind window: at PWM 80 (measured
    288 mm in a 1 s burst on 2026-10-22) that window is around 230 mm of unseen motion.
    Driving no faster than remaining/latency is what keeps the overshoot inside the gap
    rather than inside the wall."""
    allowed_in_s = (remaining_mm / bridge.MM_PER_IN) / (RANGE_LATENCY_S * RANGE_APPROACH_MARGIN)
    p = pwm_for_speed(allowed_in_s)
    if p is None or p < pwm_min:
        return None
    return min(pwm_max, p)


# Pause after every stop before starting a new move, satisfying "no sign reversal without
# a stop in between" unconditionally rather than trying to detect reversals.
STOP_SETTLE_S = 0.25

# computer_bridge.ino drops to listen-only if no serial line arrives for 500ms
# (SERIAL_IDLE_TIMEOUT_MS), so every drive loop re-sends its command at least this often.
# Verified 2026-09-28: without this, drive_distance(6in, pwm=50) stopped after ~3in.
DRIVE_KEEPALIVE_S = 0.1

# drive_arc() caps. A circle is one continuous closed-loop motion, so it gets its own
# path-length cap; it still aborts the moment telemetry goes stale. Radius floor keeps it
# from degenerating into an in-place spin (use turn_degrees for that).
MAX_SINGLE_ARC_DEG = 360.0
MAX_SINGLE_ARC_IN = 160.0
MIN_ARC_RADIUS_IN = 6.0
# How often drive_arc re-measures curvature and corrects the wheel-speed split, and how
# hard it corrects (fraction of pwm per unit of relative curvature error).
ARC_CONTROL_DT_S = 0.1
ARC_GAIN = 0.15
# With the inner wheel stopped this robot can't curve tighter than ~12.5in radius
# (measured 2026-09-28, pwm=60), so drive_arc lets the inner wheel run backwards, down to
# this PWM, for tighter circles. The wheel split changes by at most ARC_MAX_STEP_PWM per
# control step, so the inner wheel eases through zero into reverse instead of jumping.
ARC_MAX_INNER_REVERSE_PWM = int(0.9 * MAX_PWM_CEILING)  # 112; at -60 it bottomed out at ~7.9in radius
ARC_MAX_STEP_PWM = 5

# Encoder count direction when that wheel rolls forward. (-1, -1) since 2026-10-24.
# Every earlier value here -- (1, -1) on 2026-09-28, (1, 1) on 2026-09-29 -- was read off a
# drive_distance(direction="forward") move, which applies FORWARD_PWM_SIGN. That sign was
# itself wrong until 2026-10-24, so those tests recorded the tick direction of a BACKWARD
# move and labelled it forward. The two constants then agreed with each other no matter
# which way the robot really went, and no encoder-based check could catch it.
# Settled by watching the robot instead: with FORWARD_PWM_SIGN = 1, a jog(+50, +50, 1s)
# moved it physically forward (operator confirmed by eye; there is now an arrow on the
# chassis, since a symmetrical robot gives you nothing else to go on) while the right
# encoder ran -2247 ticks / -3.83in. So forward counts DOWN on this robot.
# Verified on the right channel; the left -1 is assumed by symmetry. Confirm both per
# robot.
# Only drive_arc uses this; re-check it after any encoder wiring or firmware change, and
# check it by where the robot ends up, never by a "forward" command's own ticks.
ENCODER_FORWARD_SIGN = (-1, -1)

# Re-verified 2026-10-24, robot on the ground and driving: with this at -1, repeated
# direction="forward" moves at pwm=60 carried the robot visibly backwards, confirmed by
# the operator watching it. +1 makes direction="forward" match real forward travel. This
# supersedes the 2026-09-27 on-blocks check, which read the wheel rotation by eye with
# the robot jacked up and concluded the opposite; trust the on-the-ground result.
# Re-check this after any change to wiring, motor board firmware, or which physical
# wheel is "left" vs "right".
FORWARD_PWM_SIGN = 1

