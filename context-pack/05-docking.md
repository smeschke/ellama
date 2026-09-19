# 05 — Docking

**Status: built and working.** Drive the robot in, the pins meet the contacts, the
charger runs. This works with the handheld controller standing next to the robot, and it
works driven remotely. The robot does not find the dock on its own, and it is not meant
to.

## What it is

A low base the robot drives into, with a pair of pogo pins that meet contacts that are mounted high up on the
robot. Guides on either side push the robot into alignment as it enters. Drive in, pins
touch, charger clicks on.

That is the whole mechanism. There is no electronics in the dock and no firmware anywhere
in the charging path.

## Why it is nearly free

The dock is simple because the battery chemistry allows it.

Sealed lead-acid tolerates float charging from a dumb 12 V charger with no communication
whatsoever. **Contact closure is the charging protocol.** No handshake, no BMS
negotiation, no state-of-charge exchange, no protocol version. Two pieces of metal touch
and current flows.

With lithium this is a different product: the dock has to talk to a battery management
system, which makes it a smart device with firmware, failure modes, and a support burden.

The usual argument for SLA is that it is cheap and available at any hardware store. That
is true, and it is the weaker argument. **The real argument is that SLA is what makes the
dock trivial.** See `03-drivetrain-power.md`.

## Why autonomous docking is out of scope

The robot has no idea where the dock is, and eLlama does not ship the part that would
tell it.

Finding a dock means a camera or a lidar, a detector, and a control loop — which means
compute, and compute is the part of the system this platform deliberately does not
supply. A fiducial on the dock face and a detector on a Pi is a well-worn path, and so
are half a dozen others. Which one is right depends entirely on what the owner put in the
bay.

So the split here is the same one the whole platform is built on:

**The dock is the half that never ages** — a base, two contacts, and a dumb charger.
Nothing about it will be meaningfully better in ten years. **The half that would age is
the owner's**, and it is not in the box.

Autonomous return is not a missing feature. It is Tier 3, and Tier 3 is the owner's.

## What is built, and what is not

Built:

- Drive-in charging under manual control, standing next to the robot
- Drive-in charging driven remotely
- Contacts on the robot, pins in the dock, charger runs on contact

Not built, and out of scope:

- Any detection of the dock by the robot
- Any return-on-low-battery behaviour
- Any dock-detected or charging-state signal back to the robot
