# How this robot drives

Numbers for predicting what a command will do before sending it. Measured on one unit on
painted wood with the battery above 80% and no compute box aboard; re-measure per robot
with `tools/characterize_speed.py` and `tools/characterize_turns.py`.

## Straight line — `jog(pwm, pwm, seconds)`

Steady speed, forward and reverse within 3% of each other:

| pwm | in/s | 1s | 2s | 3s |
|---|---|---|---|---|
| 50 | 3.9 | 3.9in | 7.8in | 12in |
| 60 | 5.9 | 5.9in | 12in | 18in |
| 80 | 9.3 | 9.3in | 19in | 28in |
| 100 | 11.9 | 12in | 24in | 36in |
| 125 | 14.3 | 14in | 29in | 43in |

`distance ≈ speed × duration` is reliable here: a measured `jog(50, 50, 1.0)` travelled
3.9in against 3.9in predicted. The ~0.2s ramp-up at the start and the coast after the stop
roughly cancel each other out.

Speed is near-linear above the deadband, ~0.14 in/s per PWM count, so interpolating
between rows is safe.

## Spin in place — `jog(pwm, -pwm, seconds)` or `turn_degrees`

A spin needs far more PWM than a straight move: wheel scrub loads the motors much harder.
Steady rate once up to speed:

| pwm | deg/s |
|---|---|
| 60 | 12 |
| 80 | 27 |
| 100 | 38 |
| 125 | 51 |

But a short spin never reaches that rate, so **do not multiply the steady rate by the
duration** — the ramp eats a large fraction of a brief turn. Measured at pwm 80:

| duration | turned | effective rate |
|---|---|---|
| 2.0s | 32deg | 16 deg/s |
| 4.4s | 93deg | 21 deg/s |
| 14.0s | 331deg | 24 deg/s |
| steady | — | 27 deg/s |

So a spin at pwm 80 for 2s gives about 32deg, not the 55deg the steady rate suggests.
Budget ~60% of the steady rate under 2s, ~80% around 5s, ~90% past 10s.

## Closed-loop calls overshoot

`drive_distance` coasts past its target: a 12in request lands +0.7in long at pwm 50, +1.3
at 60, +2.8 at 80, ~+4 at 100, ~+5 at 125. A 30in request at pwm 80 runs ~2.1in long
forward, ~2.8in reverse.

`turn_degrees` overshoots more at speed: a 90deg request yields 91deg at pwm 60, ~94 at
80, ~97 at 100, ~102 at 125. Ask for less than you want at high PWM, or use a lower PWM
when the angle matters.

## What will not work

- **Straight below pwm 50.** At pwm 40 the robot does not move at all, either direction.
- **Spinning below pwm 60.** At pwm 50 a spin produced 2.4deg in 39s — a stall, not a slow
  turn. Start spins at 70-80.
- **A stalled `turn_degrees` does not give up early.** It has no stall detection and runs
  its full timeout, `max(4, degrees × 0.1 + 3)` seconds — 39s for a 360deg request — with
  the motors at stall current the whole time. Check the PWM is above the spin deadband
  before asking for a large angle.
- **Above the ceilings.** `MAX_PWM_CEILING` is 125; the LLM bridge caps itself at pwm 120
  and 4s per call.

## Timing and heading

Movement starts 0.195s after the command, set by the motor board's PWM ramp (~333 PWM/s),
not by link latency — so durations under ~0.5s are mostly ramp. If the controlling process
dies, the firmware stops the robot in about 0.5s on its own
(`SERIAL_IDLE_TIMEOUT_MS = 500`).

Open-loop straight driving curves off line on any skid-steer robot, so distance from the
encoders does not imply heading. Take heading from the IMU: encoder-differential heading,
`(dr - dl) / track_width_in`, overstates rotation by ~1.7-2x during a spin because the
wheels scrub, and the factor moves with the surface, so no fixed correction recovers it.
`turn_degrees` uses IMU yaw alone and refuses to run without a live IMU.


