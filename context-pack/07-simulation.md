# 07 — Simulation

## A calibrated dry run, not a guess

eLlama has no wheel encoders or other odometry — every drive command is open-loop PWM and
duration, sent and forgotten (`04-control-architecture.md`). That makes a simulator that
just *looks* plausible worthless; it has to actually predict what the robot will do.

The simulator (`simulation/`) earns that by being calibrated against the real robot rather
than modeled from spec sheets. Simple commands were run on the physical robot, the actual
distance and turn were measured on the ground, and the simulator's speed and turn-rate
model was fit to those measurements. The result holds up well enough to plan on: straight
runs under about 8 ft land within a fraction of an inch of the simulated endpoint. Turns
drift more, since a 4-wheel skid-steer chassis scrubs sideways unpredictably, but the
simulator models that scrub too rather than ignoring it.

## Start low: generated commands default to fast

Every simulator UI requires a MAX PWM ceiling before it will run or export a path, and
there is deliberately no built-in default value.

The reason: click-to-waypoint planning — and, more generally, any motion code asked to
"get to this point" — tends to reach for the fastest PWM that satisfies the request,
since that's the straightforward solution and nothing tells it otherwise. The same bias
shows up in AI-assisted or AI-generated control code more broadly: asked to move a robot
from A to B, the natural output drives at whatever speed accomplishes that, which in an
unconstrained PWM range means close to maximum. A planned path that looks reasonable on
screen can be startlingly fast the first time it runs on hardware.

The rule that earns: start with the lowest PWM that's still effective, watch it run, and
only raise the ceiling once the path is trusted — whether it came from clicking
waypoints, typed commands, or anything a person or an LLM wrote to drive the two-number
interface. This is a property of the interface, not of the simulator specifically: the
same caution applies to any code that ends up calling `esp_bridge.ino`,
`motor_receiver.ino`, or the ESP-NOW link directly. Separately, PWM magnitudes much below
~40 tend not to move the robot at all — mechanical static friction, not a firmware
deadband — so a low-PWM command can look fine simulated and simply do nothing for real.

## A third way to send commands

`04-control-architecture.md` names two senders for the two-number interface: the handheld
controller, and onboard compute talking to the motor board over ESP-NOW. The simulator
adds a third, off-board one — a laptop or desktop, not the robot, doing the sending.

Plan a path visually in the simulator, export it as a CSV, then replay it with
`send_csv.py`: an ESP32 flashed with the unmodified `esp_bridge.ino`, plugged into the
same computer, streams the commands straight to the robot's motor board over ESP-NOW,
exactly as if a Pi onboard the robot had sent them. Nothing about the robot changes; as
far as the motor board is concerned this is indistinguishable from any other sender.

That is the same "compute is an expansion, not a dependency" argument as onboard compute,
just relocated. The planning, the simulation, the operator — none of it has to be within
arm's reach of the chassis, or even on it.

## Where this goes: attachments with no brain of their own

Push that off-board-compute pattern to its conclusion and it stops being a workflow
convenience and starts being a product shape: an attachment that carries no compute of its
own at all, because the compute never left your desk.

A lawn mower attachment is the clearest version. The mower deck itself only needs to be
cheap and mechanically robust — it never runs a model, never plans a route, never needs an
SBC riding along to get dinged by a branch or rained on. All of that lives on a desktop
computer at home: plan the mowing path in the simulator against your yard's actual
geometry, verify it clears the flower beds and the fence, then deploy the robot and drive
that path with the desktop and its ESP32 bridge, the same way as above. The robot stays
dumb and disposable; the part that's expensive and needs upgrading over time never leaves
the house.

This is also what makes the platform attachment-agnostic rather than mower-specific. The
same pattern — dumb chassis, brain left at home, path planned and calibrated in the
simulator first — applies to anything that reduces to "drive this route": patrol, a fixed
inspection loop, hauling between two points (`01-overview.md`). The mower is just the
sharpest example of why that split matters.
