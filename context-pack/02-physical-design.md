# 02 — Physical Design

## The two limits that set the size

Everything about the footprint comes from a pair of opposing constraints:

**Upper bound — one person, one lift, one trunk.** The robot has to go in the back of a
compact car without a ramp, a second person, or a decision about whether this is a good
idea. That caps weight in the mid-30s of pounds and the footprint at something a person
can get their arms around.

**Lower bound — real ground.** It has to cross grass, gravel, a sidewalk lip, and a
slope, carrying something. That sets a minimum wheel diameter and a minimum track width
before it becomes a toy.

Most mobile robots fail one of these. Indoor platforms clear the first and fail the
second — clean odometry on a flat floor hides every hard problem. Mowers, UTVs, and
quadrupeds clear the second and fail the first, being heavy or fast enough that you do
not want a stranger standing next to one.

## Weight, and why the battery ships separately

At **~32 lb dry / ~37 lb with one battery**, this is a real one-person lift with a
handle. At 42 lb with two batteries it is at the edge of comfortable.

Shipping without a battery started as a practical annoyance and turned out to solve three
problems at once:

1. **Shipping classification.** Sealed lead-acid falls under UN2800 (batteries, wet,
   non-spillable). Properly marked and packed it is largely excepted from dangerous goods
   rules — but it still carries marking requirements, carrier restrictions and
   surcharges, and per-unit compliance overhead. Not shipping one removes that entire
   category of work from fulfilment.
2. **Carton weight.** The shipped box stays under 40 lb.
3. **Substitution.** The user can fit an e-bike pack and a converter instead, without
   fighting a design built around one specific cell.

A 12 V 7 Ah SLA is available at any hardware store, auto parts store, or alarm supplier,
worldwide, for about $21. Requiring one is not a burden.

## Geometry: 17 wide × 16 long

**This is the single best design detail in the robot, and it cost one inch of plywood.**

The wheels are one inch further apart than they are front-to-back — 17 in track, 16 in
wheelbase, a ratio of 0.94.

A four-wheel skid-steer robot has no steering. To turn, all four tires must scrub
sideways against the ground. Wider skid-steer platforms need less differential thrust
than longer ones.

It is also the cheapest possible fix. It is not a mechanism, a differential, or a control
strategy. It is where you put the axle holes.

## Four wheels, not two

**The original prototype was two driven wheels plus a caster. It tipped over, and it was
miserable.** That is the largest design iteration in the project's history. On one tip it
broke the Raspberry Pi 4 riding on the deck — the instability was not a hypothetical
failure mode, it took out real hardware, and it is the direct reason the design moved to
four driven wheels. No footage of the tip exists; it predates the project's habit of
filming test runs.

A two-wheel-plus-caster robot is statically stable only over the triangle formed by its
three contact points, and the caster end of that triangle moves as the caster swivels.
Accelerate, brake, or hit a rock and the effective support polygon shrinks exactly when
you need it. Outdoors on uneven ground the failure is frequent, and every occurrence
risks whatever is bolted to the deck — which, for the intended use, is someone's camera
or lidar.

Four driven wheels give a fixed rectangular support polygon that does not change with
steering, and four contact patches for traction on loose surfaces.

## The compute module

The compute module bolts onto the chassis. Power from the main pack is available inside the module.
It holds whatever compute and sensors the owner prefers — a Raspberry Pi, a camera, a
lidar, an IMU, a GPS, or something else entirely. Internal volume is 10 W × 4 D × 10 in H.

### Sensors live in the module, not on the chassis

An IMU is the clearest case. Anything that *measures*
belongs next to the thing that *decides*, and deciding is the owner's half of the system.

So the chassis carries no sensors at all, and the module is where an IMU, a camera, a
lidar, or a GPS goes. The one thing that would ever argue for putting an IMU back on the
robot is tip detection — a chassis-level safety behaviour that cannot depend on a module
which may not be fitted. That is not built, and it is not planned.

### A roll cage as well

It is also a small roll cage. A platform meant for outdoor experimentation *will* end up
on its side, and the thing that breaks should not be the owner's lidar.

The module is optional. With the handheld controller the robot needs neither the module,
the compute, nor the lidar — and a laptop or a phone can stand in for the module
entirely.

## Materials

**¾ in hardwood plywood deck, table-saw cut.** Plywood is stiff, cheap, dimensionally
forgiving, takes a screw anywhere, and can be re-cut by the owner with tools they already
have. It is repairable with a jigsaw rather than a purchase order.

**3D-printed mounts, adapters, and body shell.** Motor mounts, hub adapters, axle blocks,
battery holder, motor driver mount, and body — all intended to be reprinted and
modified, and the body shell in particular is meant to be reshaped by the owner.

**½ in allthread axles in plywood capture blocks.** Cheap, straight enough, available
everywhere, and replaceable at any hardware store on a Sunday.

## Clearance and carrying

**4 in of ground clearance** under 10 in wheels puts the deck underside just below the
axle line. That is the right relationship: the tires decide what the robot can climb, not
the belly. A platform with big wheels and a low pan grounds out on exactly the obstacles
the wheels were chosen to clear, and this one does not.
