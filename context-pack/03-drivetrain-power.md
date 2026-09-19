# 03 — Drivetrain and Power

## The ride-on-toy drivetrain

Four RS550 brushed DC motors with 100:1 triple-spur gearboxes — the drivetrain out of a
children's ride-on car.

This is the highest-leverage sourcing decision in the project. That market has already
solved the exact problem: move roughly 100 lb at walking speed on 12 V, cheaply, reliably
enough for a product sold to parents, at a volume that makes the part permanently
available. The gearmotor costs about $8.

Consequences worth naming:

- **Torque is not a constraint.** The grade ceiling is a question about tires and
  surfaces, not motors. This is also why it will tow an adult — demonstrated so far as a
  proof of concept (a person in a wagon, towed behind the robot), not a rated capability.
  Towing an actual trailer is a stated future goal, not a current one.
- **The gear ratio sets the speed correctly with no second stage.** 100:1 on a 10 in
  wheel lands at walking pace. No additional reduction, no belt, no chain.
- **Availability outlives the product.** These are stocked by dozens of sellers and are
  not going away.
- **The speed figures come from testing.** At 12 V the robot moves at about 1 m/s.

**The cost is backlash.** Triple spur reduction in a toy-grade housing has meaningful
play. It is fine for open-loop hauling and teleoperation, and it is the first thing that
will hurt closed-loop odometry. It is the single biggest known weakness of the
drivetrain.

## Brushed, not brushless

Brushed DC at this price point is simply easier: two wires, direction by polarity, speed
by PWM, no commutation, no encoder required, no controller firmware to get wrong. It
fails gracefully and predictably, and anyone can diagnose it with a battery and two
clips.

Brushless would be more efficient and quieter, but it requires an ESC per motor, sensored
or sensorless commutation, and a materially harder debugging story for a platform whose
buyers are learning.

## Why 10 inch pneumatic wheels

Six reasons converge on this one part, which is why it is worth stating fully rather than
as "handles sidewalks."

1. **Obstacle clearance.** A 5 in radius rolls over a sidewalk lip, a root, a curb cut,
   or a rock without a run-up. Small hard casters stop dead at exactly the features that
   are everywhere outdoors.
2. **The air is the suspension.** A low-pressure pneumatic tire absorbs impact and
   conforms to uneven ground. This removes suspension from the design entirely — no
   springs, no pivots, no articulated axles, no extra failure modes.
3. **Ubiquity and price.** 10 × 3.50-4 is the hand-truck and wheelbarrow size, stocked at
   every hardware store on Earth for about $7.
4. **The bore is a standard.** A 5/8 in bearing bore takes ½ in allthread with an
   off-the-shelf bushing — an axle assembly from the plumbing and fastener aisles.
5. **Contact patch on turf.** A wide, low-pressure patch crosses grass without tearing it
   and floats on loose gravel instead of digging in.
6. **Diameter sets the gear ratio.** A 10 in wheel with a single 100:1 stage lands on
   walking speed. A smaller wheel would need a second reduction stage.

Point 5 interacts with skid steering in a way worth understanding: a larger contact patch
means more scrub resistance in a turn. That cost is what the 17 × 16 geometry in
`02-physical-design.md` buys back.

## Speed

Measured working speed is **1.72 mph average over 460 logged minutes**, with the fastest
logged run at 2.87 mph.

## Power

**12 V 7 Ah sealed lead-acid, one or two wired in parallel, user-supplied.** The system
runs on 12 V only — a second battery adds capacity, not voltage. There is no 24 V
configuration.

SLA is the unfashionable choice and the correct one here:

- **The dock is nearly free because of it.** SLA tolerates float charging from a dumb
  12 V charger with no communication. That means contact closure *is* the charging
  protocol — pogo pins touch, charger clicks on, done. No BMS conversation, no handshake,
  no smart dock. With lithium the dock becomes a smart device with a protocol. See
  `05-docking.md`.
- **Available anywhere.** Hardware store, auto parts store, alarm supplier, ~$21.
- **Carries shipping overhead, so we do not ship it.** UN2800 non-spillable: largely
  excepted when marked and packed correctly, but still marking rules, carrier surcharges,
  and weight. See `02-physical-design.md`.
- **Safe to abuse.** No thermal runaway, no fire risk from a shop-floor puncture.

The costs are honest: weight (~4.85 lb), poor energy density, and limited cycle life
compared to lithium. Runtime is roughly 1.5 hours per battery at working duty — an
inference from logged voltage drop, not a measurement. The design accommodates the user
substituting an e-bike pack and a converter.

## Accessory power

The compute module takes 12 V from the robot battery through an inline-fused barrel jack.
