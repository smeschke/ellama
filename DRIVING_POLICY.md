# Driving Policy

Rules for anyone or anything sending drive commands to eLlama's real
motors — a human at a joystick or teleop client, or an AI testing firmware,
running a script, or sending raw `<left> <right>` values over
`computer_bridge.ino`. This is not a design-rationale document like
`context-pack/`; it's a set of constraints for actually operating the robot.

Most of what's written here is how a human already drives this robot by
instinct — ease up from zero, watch what happens, stop before changing your
mind about direction. It's written down because AI operators have
repeatedly done the opposite: picking PWM values that look "clearly
visible" for a demo instead of conservative, jumping straight to a target
instead of testing upward from nothing, reversing direction without
stopping first, and chaining moves together assuming they'll blend cleanly.
An AI operating this robot should follow every rule here exactly, even when
a task seems to justify skipping one. A human operator will likely recognize
most of it as what they already do.

## 1. Get permission before anything moves

Before an AI operator sends any command with a nonzero left or right value
to real hardware:

1. Confirm the robot is physically safe to move — clear area, no people or
   pets nearby, not balanced somewhere it could fall.
2. State the exact command about to be sent, and wait for the go-ahead.

This does not apply to read-only or zero-motion actions: reading encoder
telemetry, checking serial connectivity, or sending `stop`. It applies to
every command with a nonzero PWM, even a small one, even a "quick test,"
even if the request was phrased as a general question rather than an
explicit instruction to move the robot. (A human driving directly is
already the one making this judgment in real time — this rule exists for
the case where the thing sending commands isn't the thing watching the
robot.)

## 2. This is not a simulation — there is no reset

In a simulator, a bad command just means you re-run it. On the real robot,
a bad command can flip it, drive it into something, or drop it off a curb —
and there is no undo. If it flips and the Pi breaks on impact, there is no
"try again," there is a robot that no longer has onboard compute. Every
command is one-shot and consequential, not an iteration you get to throw
away if it doesn't work.

The practical effect: treat every real-world command with the caution
you'd give an action you can only take once, because that's what it is. If
a plan only works out when several things go right in a row, that plan is
wrong for real hardware, even if it would be fine in
`simulation/`.

## 3. Start at the lowest PWM that might work, not a value that will obviously work

When you don't already know the right PWM for a task, the first command is
not a best guess at what will produce a clean result — it's the smallest
value with a chance of producing *any* visible result. Then evaluate and
adjust:

1. Send the low value.
2. Observe: did it move at all? Which direction? Was that the expected
   result?
3. If it didn't move enough, increase and try again.
4. If it overshot, something may already be stressed or damaged — there is
   no equivalent "step down and retry" once metal has moved.

This asymmetry is the whole reason to be conservative: undershooting costs a
few seconds of iteration. Overshooting risks the gearbox, the wheels, or
whatever the robot just ran into. Never pick a value because it will make
the result unambiguous to observe — a convincing demo is not the goal, and
"clearly visible" is exactly the reasoning that has produced bad commands
before.

**Concrete numbers for this robot — sourced, not guessed:**

- **Ramp rate:** `robot_motor_bts7960.ino` moves the commanded PWM by at most 1
  count per 3 ms toward its target
  (`RAMP_STEP`/`RAMP_DT_MS`,
  [`firmware/robot_motor_bts7960/robot_motor_bts7960.ino:32-33`](firmware/robot_motor_bts7960/robot_motor_bts7960.ino#L32-L33))
  — about 333 PWM/s, or ~0.77 s to go from 0 to full 255.
- **Manual/teleop operating ceiling:** 128/255, chosen after top speed was
  found too fast
  (`SPEED_MAX`,
  [`teleop/command_client.py:37`](teleop/command_client.py#L37)).
- **Scripted ground-truth-run ceiling:** 125/255, used for calibration and
  encoder-verification runs where the robot actually rolls
  (`MAX_PWM_CEILING`,
  [`firmware/computer_bridge/toandfro.py:29`](firmware/computer_bridge/toandfro.py#L29)).
- **Absolute hard limit:** 255/255 is the most the wire format can express
  (`DrivePacket.left`/`.right`, range -255..255,
  [`firmware/robot_motor_bts7960/robot_motor_bts7960.ino:37-40`](firmware/robot_motor_bts7960/robot_motor_bts7960.ino#L37-L40)).
  This is a format ceiling, not a safe value — it says what the hardware
  *can* be told, never what it *should* be told.
- **First command on a setup you haven't personally verified:** there is no
  code constant for this — it's a floor this document sets, not a fact
  read from a file. Start at 5–10% duty (13–26/255) regardless of any
  script's own default, then work upward per the loop above.

These are ceilings, not targets — start below them and work up. If the
code changes, the file is right and this document is stale: check the
actual constant before trusting a number written here.

## 4. Ramp, don't jump — and never skip through a reversal

Never command a large instantaneous change in PWM, in either direction:

- No 0 → high jumps. Ease up in steps; even a coarse few-second ramp is
  enough. This is both mechanical (the drivetrain is a 100:1 triple-spur
  gearbox off a ride-on-toy chassis — sudden torque is exactly what shears
  gear teeth) and social (easing a joystick forward gives anyone nearby a
  couple of seconds to notice the robot is about to move and get clear; a
  sudden jump doesn't).
- No sign reversals without a stop in between. Going from `+100` to `-150`
  in one step is worse than either value alone — it reverses load on the
  gear train at full magnitude instantly. Always command `0` (or let it
  reach zero) before commanding the opposite sign.
- Braking is not exempt. Sudden PWM-to-zero is a hard stop, not a neutral
  action — `robot_motor_bts7960.ino` drives both motor terminals near ground at
  PWM 0, which is active dynamic braking through the BTS7960 drivers, not
  coasting. Treat a stop command with the same "don't do it abruptly at
  high speed" caution as a throttle command.

Note: `robot_motor_bts7960.ino` already ramps toward whatever target it receives
(`RAMP_STEP`/`RAMP_DT_MS`), and has a fail-safe zero on signal loss
(`CMD_TIMEOUT_MS`). That is a mechanical safety net, not a substitute for
choosing sane targets — it limits *how fast* a bad command gets applied, it
doesn't make a bad command a good one. Don't rely on it to excuse picking a
value you wouldn't otherwise choose.

## 5. You can string commands together — but chaining needs more sensing, not less

Stopping fully between every single command isn't a hard requirement. What
matters is that each command is issued with an accurate picture of where
the robot actually is and what it's actually doing — and stopping to check
is simply the easiest way to guarantee that when you don't have another way
to know.

If you chain moves without stopping, you're acting on your *prediction* of
where the robot will be when the next command starts, not a measurement.
That's fine as long as something is actively confirming the prediction as
you go — encoder counts, lidar, a camera, a timeout with a sane fallback —
so a sequence that's drifting off-plan gets caught and corrected instead of
compounding silently until something is already wrong. Chaining fixed-
duration, open-loop moves with no feedback in between is how a small error
in step one becomes a bigger one by step three.

Default to the stop → check → decide → move loop for anything without solid
live feedback, which today is most of this robot's scripts (`toandfro.py`
and friends drive at fixed PWM for a computed duration or until an encoder
threshold, not with continuous position tracking). Example: aligning the
robot square and centered in a doorway isn't done by computing the right
trajectory and driving it in one motion — it's driving up, stopping,
backing off, stopping, checking the alignment, and driving up again,
correcting a little each pass. That works because every pass ends with a
clean, motionless read of position. A continuous "smart" approach doesn't
give you that read while the robot is still moving and carrying momentum
from what you just told it to do — so without sensing good enough to
replace the stop-and-look step, don't skip it.

## 6. Why this reads differently from how AI defaults to operating

Left to its own judgment, AI tends to pick the PWM value that most cleanly
demonstrates the intended behavior, apply it immediately at full target
magnitude, treat reversing direction as just a sign flip, and chain moves
together the way you'd chain function calls — assuming each one starts
clean and composes predictably, the way it would in a simulation. All of
that is wrong for a physical drivetrain with inertia, backlash, gear teeth
that don't forgive a shock load, and no undo if something goes wrong. A
human who has actually held the joystick does none of this — they ease up
from zero, watch what happens, let it settle or actively confirm it's on
track, and never expect the first input to be the last one they need.
That's the standard this document is asking an AI operator to hold itself
to — and, hopefully, a description a human operator already recognizes as
just how you drive the thing.
