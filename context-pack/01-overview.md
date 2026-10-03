# 01 — Overview

## What it is

eLlama is a four-wheel-drive outdoor robot base, roughly the size of a large dog, that a
person can lift into a car trunk. It ships as a base: chassis, four driven wheels, the
motor board, and a handheld controller. Add a battery, switch it on, and drive it with no
computer and no software.

It is also a mount for compute. A box on the front deck takes a single-board computer and
whatever sensors the job needs — a camera, a lidar, an IMU. With that fitted, the same
chassis is a teleoperated or autonomous platform. The robot ships with an empty box
intended for a single board computer or mini pc.

Two things are bring-your-own: the **battery** and the **compute**. You choose both. The
battery choices are in `05-battery-options.md`.

Both of those are true at once, and the relationship between them is the product.

## Why obsolescence is the real problem

A robot sold with a specific SBC in 2026 is carrying a 2026 decision forever.

eLlama supplies only the slow-moving parts: motors, gearboxes, wheels, plywood, an ESP32,
and a radio protocol. None of those will be meaningfully better in three years. The
fast-moving parts — the computer, the camera, the model running on it, and the battery
chemistry that is cheapest this year — are the owner's, replaced on the owner's schedule.
The compute goes in a box sized to accept whatever comes next, and the battery is one
the owner already has or can buy locally.

That is also why the mechanical design targets ubiquitous parts
(`03-drivetrain-power.md`). A ride-on-toy gearmotor and a hand-truck tire will be
available and cheap for decades.

## Who it is for

**Someone who already has the computer.** A student with a Jetson, a hobbyist with a Pi
and a lidar, a researcher who needs a body for an algorithm. They do not need a robot;
they need something to put the robot brain into that will survive a curb and a lawn.

**Someone who wants a task done outdoors.** Hauling, patrol with a light, inspection with
a camera, a repeatable route past fixed points. The chassis does the moving; the
task-specific part is bolted on.

The overlap between those two — a platform that earns its keep between experiments — is
the interesting position. A research platform costs money while it sits. This one hauls
the trash, which means it gets driven daily, which means real field time and real
failures instead of staged demos.

## Why wheels, and why this size

Legs are an evolutionary answer to a constraint biology has and robots do not: you cannot
run blood and nerves through a continuously rotating joint, so nothing that evolved ever
grew a wheel. We can. And humans have spent centuries flattening the ground for wheels —
driveways, paths, sidewalks, shop floors. A wheeled robot inherits all of that for a
fraction of the cost and failure modes of a leg.

The size is set by two opposing limits, covered in detail in `02-physical-design.md`:
small and light enough that a stranger can shove it out of the way and one person can
lift it into a compact car, large enough to cross grass, gravel, and a curb with a load.
Indoor-scale robots fail the second test. Mowers, UTVs, and quadrupeds fail the first,
and are priced to match.

## What it is not

Not industrial-tough. Not weatherproofed for permanent outdoor installation. Not a
certified safety product. Not a finished autonomy solution — there is no SLAM stack in
the box, and no ROS (`04-control-architecture.md` explains why).

It is a platform built to be learned on, modified, and re-revved, made from parts you can
replace at a hardware store.
