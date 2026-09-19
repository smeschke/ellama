# 06 — Manufacturing and Cost

## Make vs. buy

**Bought:** everything where a commodity market has already solved the problem —
gearmotors, wheels, ESP32s, BTS7960 drivers, allthread, fasteners, plywood, battery. This
is the sourcing philosophy from `01-overview.md` applied to procurement: buy the
ubiquitous part, and inherit its price, availability, and longevity.

**Made in-house:** the parts that are specific to this robot and that nobody sells —
motor mounts, hub adapters, axle blocks, battery holder, electronics carrier, and body
shell (all FDM printed); the plywood deck and axle capture blocks (table saw and drill
press); the fixed axles (bandsaw and bench grinder); and the Llama Carrier Board
(fabricated at JLCPCB, through-hole connectors hand-soldered).

Every in-house part is documented with material, method, equipment, setup time, and cycle
time in the Manufactured Parts sheet. Setup is 5–10 minutes per part; cycle times run
0.1 h for a saw cut to 4 h for the body shell print.

## The carrier board

The Llama Carrier Board is a 2-layer PCB carrying a socketed ESP32 DevKit, four keyed 2×4
IDC headers for the motor drivers, and screw-terminal power.

JLCPCB fabricates the bare board only — traces, pads, silkscreen, no assembly. Pricing at
that scope moves with order quantity and whatever promo is running, so the honest number
is a range rather than one figure: a 100×100 mm bare board is well under $1 landed at
typical small-batch pricing, and one order of five landed at $7 shipped (helped by a
first-order coupon, so treat that as a favorable data point, not the steady-state price).
Either way, the bare board itself is a small fraction of the board's total cost.

Assembly is by hand: the ESP32 DevKit V1 goes on socketed rather than reflowed as a bare
module, along with a screw terminal and a handful of headers. Because the board isn't
committed to a fixed connector layout at fab time, the harness side stays flexible — the
motor and power connections are made with Dupont jumpers into those headers rather than
a cable set baked into the PCB order, so a different cable run or a one-off harness is a
matter of re-jumpering, not a new board spin.

Hand-populating the DevKit also leaves its USB-C port sitting exposed at the board edge:
the same port flashes the board and, with `esp_bridge.ino`, lets a bench PC drive the
robot directly over serial (`04-control-architecture.md`). A reflowed bare-module design
would have buried that port inside the enclosure.

Keyed connectors matter for a kit as much as they do for a factory: a connector that only
goes in one way is documentation that cannot be misread.

The boards are running in both prototype robots.

## Community and validation

- Developed and demonstrated at **Ctrl-H** (Portland hackerspace), where members see it
  doing real hauling work weekly
- Presented to **PARTS** (Portland Area Robotics Society)
- Demoed at **Teardown 2026** — Crowd Supply's own event, Portland
