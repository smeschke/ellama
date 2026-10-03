# Line-follow runbook: what to do when the script stops

The follower (`start_line_follow`) is a simple rule-based script on purpose. It is good at
steady tracking and bad at judgment calls (paint chips, glare, scratches, shadows). So the
design is: **the script drives; when it stops, the AI looks at the images and decides.**
A script status is a *claim*, not a verdict -- that includes `finish_candidate`.

## The procedure (every stop, no exceptions)

1. **Look first.** Call `line_follow_report`. View the contact sheet and the fresh still
   before reasoning about logs or code. If the overlay is ambiguous, take a raw `look()`
   (the camera is mounted sideways: the robot's front is on the right of that photo).
2. **Give a verdict**, in the user's terms: what the script said, and what the images show.
3. **Pick the recovery** from the table below.
4. **Authority.** Suggest the move and wait for a go-ahead, unless the user has said
   **FULL AUTO** (all caps). Under FULL AUTO, do it, then tell the user what you did.
   Even under FULL AUTO, stop and ask after 3 recoveries in a row without progress, and
   for anything that is not a small nudge/restart.
5. **Restart** with `start_line_follow` (needs `confirmed_safe`; the line must be in view
   with >= 3 bands).

Don't jump to changing code. Most stops are a nudge and a restart.

## Verdicts and recoveries

| Script says | Check in the images | Verdict | Recovery |
|---|---|---|---|
| `finish_candidate` | Is there a real perpendicular strip across the whole line, or glare/paint/a scratch? Compare the annotated and RAW images. | Real finish, or false finish | Real: done, tell the user. False: restart with `ignore_finish_s` (about 6-8) |
| `end_of_tape` | The line vanished right after a cross strip was in view. Is a T/crossbar going under the camera in the frames, and bare floor after? | Real end of course (usually) | Confirm with a raw `look()`; if the floor ahead is bare, it's done. If tape continues, nudge toward it and restart |
| `stale` | Is the line still in view and the robot where it should be? Do the last log rows repeat identically? | Stalled on the line (spin below the wheels' breakaway effort) or a late camera frame | Nudge toward the line (`turn_degrees` ~10 deg, pwm 70), restart |
| `lost` | Which side did the line leave on (`last_line_seen.side`)? Is it visible in the still? | Off the line, left or right | Turn toward that side by the angle the still suggests, restart |
| `telemetry_stale` | Is the board link alive (`read_telemetry`)? | Link problem | Reconnect (`connect`); don't drive until telemetry is live |
| `timeout` | Where is the robot now, and is it making progress? | Slow or very long course | Restart if it's on the line |
| `stopped` | Someone called `stop()`/`listen()` | Operator stop | Do nothing; ask the user |

Nudge sizes that worked: right 10-20 deg for a mid-curve stall (it came out at 11-21 deg
by IMU).

## Known quirks (as of 2026-10-03)

- **The follower does not need the encoders.** Its start gate and watchdog use the IMU
  stream. A dead encoder (right encoder's magnet fell off) doesn't block it. Other tools
  (`drive_distance`, `drive_arc`) still do.
- **Glare on the test floor** (dark glossy paint) creates fake "cross strips" next to the
  line, often together with a scratch. A real strip spans both sides of the line.
- **Spin deadband is around 50 raw on this floor**, not the 28 in `line_follow.py`. A spin
  commanded at +/-44 did not move the robot, so the aim angle never changed and the run
  ended `stale`. Spins at +/-55 to +/-64 worked. Fixed in code (see "Code changes").
- **Stale-frame stops** can also come from a camera hiccup (fps dips toward ~11 during
  spins; the limit is 0.5 s without a frame). Check whether the log rows were still
  changing before blaming the camera.
- After an `/mcp` reconnect the camera stream needs a warmup; `start_line_follow` enforces it.
- `take_photo` only works while a recording is running; use `look()` otherwise.
- The competition surface (white melamine) should be easier than the test floor.

## Findings from the first full AUTO run (2026-10-03)

Full course completed in six follower starts and five recoveries (nudges of 8-30 deg and
restarts). What it taught us:

- **False finishes are the most common stop on this floor.** Three of six stops were chipped
  paint or a worn/peeled stretch of tape being read as a cross strip. Always confirm with the
  contact sheet AND a raw `look()` before accepting `finish_candidate`. Use `ignore_finish_s` of about
  6-8 s to get past a known patch; shorter near the end of the course.
- **The old detector MISSED the real finish** (fixed 2026-10-03, see "Code changes"). The T was
  flagged on three frames, but the rule wanted two *consecutive* frames and a glared frame
  broke the run, so the script reported `lost` after the robot drove over it. Now a hit
  count over a window is used and `end_of_tape` exists. Still check a `lost` for a crossbar.
- **Start gate needs 3 bands.** After a sharp turn the line may show only 2 bands (it is
  leaving the analysis zone). Nudge toward the line and re-check `line_view`; retry once if a
  fresh `line_view` shows 8 bands but the gate disagrees (a transient frame).
- **Camera fps sagged from ~16 to ~10-11 over a long session.** Slow frames add lag, and the
  robot then weaves (spin right, spin left). Cause not found. If the weaving gets bad, check
  the phone (heat, battery saver, Wi-Fi) before blaming the controller.
- **Sharp corners:** the line leaves the frame at the side the corner turns toward. Nudge
  that way by 20-30 deg (right 25-30 and left 30 both worked), then restart.
- **Frozen log values mean a stall.** If consecutive status polls show identical offset, aim
  and spin command while frames keep arriving, the robot is not moving. Stop it, look, nudge,
  restart; don't wait for `stale`.
- **A start-of-course restart is fine** after turning 180 deg: the follower doesn't care
  which direction it faces, only that the line is in view.

## Lessons from the second full run, the reverse direction (2026-10-03, new controller)

- **Check the start, not just the gate.** The first start of this run began with the line 4 in
  off-center and pressed against the left edge of the frame (the detector zeroes the border), next to a
  crossbar, and the follower locked onto the chipped-paint scar on the right edge instead.
  It spun right for 3 s and drove off the line. Before `start_line_follow`: the tape should be
  roughly centered and the green dots in `line_view` must be on the tape (not on paint). If the
  robot ends up lost after a bad start, undo the rotation using the IMU yaw before and after
  (it was 38.7 deg right), then look for the tape.
- **Sunlight and window-frame shadows** produce soft dark bars and crosses that the detector
  reads as a line and as a cross strip. The raw images tell them apart immediately: a shadow has
  soft edges, tape has sharp ones. At the very end of the run the follower was tracking a
  shadow band beside the real tape.
- **The new finish rule fires on paint, floorboard seams and shadow crosses** (three false
  candidates in one run, plus one real one). That is expected and cheap: look at the raw frames,
  then restart with `ignore_finish_s` of 5-6 s.
- **Sharp bends overshoot.** At a ~50 deg right bend the robot kept going straight for a moment
  (camera and motor lag), the tape slid out of the corner of the frame, and the run ended `lost`
  after 40 s. A nudge of about the bend angle toward the side the tape left from recovered it.
  The stall boost fired once on that run (boost 16, spin 80) and the spin completed.
- **A real far-end T looks like the first one:** a crossbar with its stem coming from the
  bottom of the frame. Both ends of this course have one.

## Code changes made 2026-10-03 (after the first full run)

All in `line_follow.py` unless noted; checked in `line_follow_sim.py` (84 runs all arrive; with
the stall boost off 22 of the sticky-floor runs time out) and by replaying the recorded runs.

- **Forward and turn limited separately.** `F_MAX = 64` is the highest forward effort
  (`start_line_follow`'s `speed_pwm` cap, was 70). Turning may take the outer wheel to
  `TURN_OUTER_MAX = 100` (was 90), spins go up to `T_SPIN_MAX = 85`. `OUTER_MAX` and
  `T_DEADBAND` stay as they were because `person_track.py` (turn_to_me) imports them.
- **Spin breakaway floor.** A spin never commands less than `T_SPIN_MIN = 52`.
- **Stall boost.** If a spin's aim angle hasn't shrunk by 1 deg in 0.6 s, add 8 to the spin
  effort (repeatedly, up to the ceiling); forward effort is untouched. The boost resets when
  the spin ends or flips direction. Visible as `info.boost` in `line_follow_status`.
- **Finish is a hand-off.** `finished` became `finish_candidate`: fires when the strip is at
  least 0.28 down the frame and at least 2 of the last 5 frames had a cross strip (not
  necessarily consecutive). On a replay of the last run it fires 1.3 s before the old `lost`.
  It is also more sensitive to paint patches than before (it fires a few seconds earlier on
  the patch that fooled it), which is the intended trade: a stop costs seconds, a missed
  strip costs the run.
- **`end_of_tape`.** If the line is lost within 3 s of a cross strip being in view, the status
  is `end_of_tape` instead of `lost`.
- **`line_follow_report` returns raw (unannotated) images too:** annotated sheet, raw sheet,
  annotated still, raw still. Pass `raw=false` for the old output.
- Not changed: pure-pursuit steering, camera reader, IMU gate, `ignore_finish_s`. Not tried
  on the robot yet: only the simulator and recorded-video replays.

## PWM rules of thumb (from the user, 2026-10-03)

Straight and turning behave very differently, so don't scale them together:

- **Straight (forward) PWM is very sensitive.** A small increase gives a big change in
  speed, so move it in small steps and keep its ceiling modest. The current cruise (58) and the
  top end used for straights are probably too high for driving straight.
- **Turning PWM often has to be large.** Wheel scrub, a crack, a small stone or a sticky spot
  can need a big command to break free. Spins that were too gentle (+/-44) stalled; +/-55 to
  +/-64 turned. The upper limit for turns is about right and could go a little higher.
- So when choosing efforts by hand (`jog`, `turn_degrees`), use **gentle, small increments on
  forward** and **generous effort on turns**, and don't raise both together when something is
  stuck. For a stuck turn, raise the turn effort and leave the forward effort alone.

## Tools to use

`line_follow_report` (the main one), `line_view` (live look, also mid-run),
`line_follow_status`, `look`, `turn_degrees`, `start_line_follow(ignore_finish_s=...)`,
`stop`.
