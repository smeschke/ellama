# 04 — Control Architecture

## The interface is two numbers

The chassis accepts a left speed and a right speed. That is the entire contract.

Everything above that line is replaceable and everything below it is fixed. The
drivetrain, wiring, and motor firmware never change, regardless of what produces the
numbers. There is no SDK and no middleware to learn.

This is what makes manual and autonomous the same robot. They are not modes — they are
different senders. Switching between them is unplugging one command source and plugging
in another, and the motor board cannot tell the difference.

## Two ways to send them

**Handheld controller** — a single analog stick, talking directly to the motor board over
ESP-NOW. No computer involved. This is what ships working.

**Onboard compute** — a program on the Pi writes left/right values over serial to an
ESP32 acting as a radio bridge. The Pi can be connected to the internet for
teleoperation.

The handheld path needs none of the others to exist. That ordering matters: the robot is
complete without compute, and compute is an expansion rather than a dependency.

Onboard compute doesn't have to be onboard, either: a desktop running the simulator and an
ESP32 running the same `esp_bridge.ino` is the same sender, just relocated off the chassis
entirely (`07-simulation.md`).

## One controller drives every robot — on purpose

The controller does not pair to a robot. It broadcasts, and any motor board listening on
the channel drives from it — no bind procedure, no MAC address to enter, no per-unit
configuration at the factory or by the owner.

That sounds like a limitation — "the controller can't be exclusive to one robot" — until
you notice what it buys: a controller and a robot that have never seen each other work
together the moment both are powered on. That is what "ships working" actually rests on.
There is no pairing step to document, no pairing step to get wrong, no pairing step to
walk a customer through over email when it fails.

It also means the same controller scales up instead of down. Someone with more than one
robot does not need a second controller or a rebind ritual to switch between them — one
controller drives whichever robot is in front of them. For anyone herding a small fleet,
that is the same broadcast behavior read as a feature rather than a workaround.

This is the other side of the tradeoff already stated above: the link has no
authentication because authentication is what would turn "any controller drives any
robot" back into "one controller, one robot, and a pairing step in between."

## Stock firmware is a starting point, not a ceiling

Everything above describes what ships: broadcast, unencrypted, unauthenticated, ESP-NOW on a fixed channel. That is a deliberate default, not the only supported mode. It is intended that owners experiment with it and change it.

The firmware is plain Arduino with no framework to fight, so the modifications this invites are direct:

Unicast instead of broadcast — address the packet to one robot's MAC and get exclusivity back, for someone who wants "my controller, my robot" instead of the out-of-the-box fleet behavior.

Encryption and authentication — ESP-NOW supports encrypted peers; adding that trades the zero-setup default for a link that can't be casually driven by a stranger, for anyone whose use case makes that tradeoff worth it (see the tradeoff already named above).

Deliberate delay or offset — staggering when a robot acts on a broadcast packet, or filtering which packets it responds to, is what turns "every robot obeys every controller instantly" into controlled herding of a group, rather than all of them doing the same thing in lockstep.

Or whatever the owner's own use case actually needs — the point is not this specific list, it's that the stock configuration is a known-good starting state, not a locked one.

This is the same argument as the tiers in 01-overview.md, one level down: the platform ships with one deliberate, working default so it's usable immediately, and leaves the protocol itself as open territory for the owner to shape, the same way the compute bay leaves the brain open.

## Why a direct radio link rather than WiFi

The controller link is connectionless: no access point to join, no DHCP, no pairing
ritual, no reconnect storm when the robot drives behind a shed. For a machine that works
in a field with no infrastructure, *the absence of a network to join* is the feature.

Network teleop layers TCP on top only when a Pi is fitted and a network exists — and even
then the last hop to the motors is still the direct radio link.

**One deliberate tradeoff, stated plainly:** the link is unencrypted and unauthenticated.
Anyone in radio range with the right hardware can drive the robot. That is acceptable for
a hobby platform at walking speed that a person can physically stop; it is not acceptable
for anything unattended or valuable. A bounded decision, not an oversight.

## Layered failsafes

If the MCU receives no signal from the controller for a fraction of a second, it enters
failsafe and commands PWM zero.

PWM zero is not neutral: the BTS7960 drivers hold both terminals of each motor near
ground rather than letting them float, shorting the motor's back-EMF through the driver.
That is active (dynamic) braking, confirmed by testing — not just the 100:1 gear
reduction holding the robot on a slope, though that helps too.

*(Specific timeout values live in the firmware.)*

## Wiring is standardized

The custom PCB is what makes every robot wire up identically. It is documented well
enough that a user can reflash the MCU and change behaviour without tracing a harness
first.

The ESP32 DevKit is socketed and hand-populated onto the board rather than reflowed as a
bare module (`06-manufacturing-cost.md`), which keeps its USB-C port exposed at the edge
of the board. That single port is what makes the three control paths above physically
possible on the same hardware: flash new firmware over it, or plug it into a bench PC and
drive the robot directly with `esp_bridge.ino` — the same port, no case to open, no
programmer to attach.
