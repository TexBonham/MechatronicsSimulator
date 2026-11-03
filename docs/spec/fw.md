# Spec: Firmware and board interface (FW)

## Purpose

The firmware rules that keep native mode honest and the board interface header the simulator owns
(`board_interface/`), plus the behaviors of the reference design's controller under `firmware/`. FW-001 to FW-012 apply
to every firmware tree; FW-013 onward are requirements on the gantry reference firmware only, labelled as the reference
design, and exist so G1 to G14 have a firmware side to verify. Calling the firmware is CTL's; the models behind the pins
are SIG's.

Sources: "Architecture doc" is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`;
"Requirements doc" is `docs/Mechatronic Simulator — Requirements.md`. Milestones for the reference firmware rows follow the
G case they serve, per the Requirements doc, Instructions for the planning session (milestone plan).

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| CTL | FW-003, FW-004, FW-009, FW-010, FW-011 | CTL-002, CTL-003, CTL-004, CTL-005, CTL-006, CTL-016 |
| SIG | FW-025, FW-029 | SIG-018, SIG-026, SIG-027 |
| ACC | FW-013 to FW-030 | ACC-001 to ACC-024, ACC-027, ACC-028 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| FW-001 | Firmware application code shall be written in C11. | C1; ADR 0011, C1 | M1 | unit | n/a |
| FW-002 | C++ in a firmware tree shall appear only inside a module behind a C interface. | C1; ADR 0011, C1 | M1 | unit | n/a |
| FW-003 | The board interface shall be a C header. | C1; ADR 0011, C1 | M1 | unit | n/a |
| FW-004 | The board interface and firmware application code shall use fixed-width integer types. | Architecture doc, Technology recommendation ("Native mode") | M1 | unit | n/a |
| FW-005 | Firmware application code shall include the board interface header and nothing else from the simulator or any vendor HAL. | Architecture doc, Repository starting point (invariant 8) | M1 | invariant | n/a |
| FW-006 | The host build preset shall fail on any include in application code outside the board interface. | C2; ADR 0011, C2 and Consequences | M1 | invariant | n/a |
| FW-007 | The firmware build shall provide a host preset and an arm-none-eabi preset that build the same application sources. | C2; ADR 0011, C2; Requirements doc, Controller requirements (portability) | M1 | unit | n/a |
| FW-008 | The host and arm-none-eabi presets shall use identical warning flags, with warnings as errors. | C2; ADR 0011, C2 | M1 | unit | n/a |
| FW-009 | The board interface header shall be owned by the simulator and live in `board_interface/`. | Architecture doc, Technology recommendation ("Native mode"); Architecture doc, Repository starting point (layout) | M1 | unit | n/a |
| FW-010 | The board interface shall declare the firmware's init and tick entry points, a tick-rate query the simulator reads once, and the plain C input and output structs they exchange. | C3; ADR 0011, C3 | M1 | unit | n/a |
| FW-011 | Firmware application code shall be correct under polled semantics in native mode, with no dependence on interrupt timing. | ADR 0011, Consequences; C3 | M1 | unit | n/a |
| FW-012 | Vendor HAL code shall appear only under `firmware/board/<chip>/`. | C5; ADR 0011, C5 | M1 | invariant | n/a |
| FW-013 | Reference design (gantry): the firmware shall start "not homed", refuse every target until homing finishes against the limit switches, and report "not homed" with no actuator motion. | Requirements doc, Controller requirements (gantry reference controller); Requirements doc, Test cases (G1) | M1 | acceptance | n/a |
| FW-014 | Reference design (gantry): homing shall drive each axis to its limit switch, set the origin and report "ready". | Requirements doc, Test cases (G2); Requirements doc, Controller requirements (gantry reference controller) | M1 | acceptance | 1 cm (origin against true) |
| FW-015 | Reference design (gantry): after homing, a target inside the workspace shall move the hook to the target without exceeding any travel limit. | Requirements doc, Test cases (G3) | M1 | acceptance | 1 cm (final position) |
| FW-016 | Reference design (gantry): a target outside the workspace shall be rejected with a fault and no motion, never clipped to the edge. | Requirements doc, Controller requirements (gantry reference controller); Requirements doc, Test cases (G4) | M1 | acceptance | n/a |
| FW-017 | Reference design (gantry): after any power loss the firmware shall return to "not homed" and allow no motion until homed. | Requirements doc, Controller requirements (gantry reference controller); Requirements doc, Test cases (G9) | M2 | acceptance | n/a |
| FW-018 | Reference design (gantry): with a load over the limit attached before a move, the firmware shall ignore lift commands and execute lower commands. | Requirements doc, Controller requirements (gantry reference controller); Requirements doc, Test cases (G5) | M2 | acceptance | n/a |
| FW-019 | Reference design (gantry): when the load becomes overweight mid-lift, the firmware shall stop lifting and still allow lowering. | Requirements doc, Controller requirements (gantry reference controller); Requirements doc, Test cases (G6) | M2 | acceptance | n/a |
| FW-020 | Reference design (gantry): when encoder position disagrees with the commanded position, the firmware shall stop, fault and require re-homing. | Requirements doc, Controller requirements (gantry reference controller); Requirements doc, Test cases (G10) | M2 | acceptance | not set (owed by gantry team: mismatch threshold) |
| FW-021 | Reference design (gantry): when a limit switch trips where it should not, the firmware shall stop, fault and require re-homing. | Requirements doc, Controller requirements (gantry reference controller); Requirements doc, Test cases (G11) | M2 | acceptance | n/a |
| FW-022 | Reference design (gantry): the firmware shall keep every commanded motion within its acceleration and deceleration limits. | Requirements doc, Controller requirements (gantry reference controller) | M1 | acceptance | not set (owed by gantry team: limit values) |
| FW-023 | Reference design (gantry): the firmware shall slow near targets and near travel limits. | Requirements doc, Controller requirements (gantry reference controller) | M1 | acceptance | not set (owed by gantry team) |
| FW-024 | Reference design (gantry): after an E-stop, the firmware shall stay stopped until a manual reset and a new command, with no automatic resume. | Requirements doc, Controller requirements (gantry reference controller); Requirements doc, Test cases (G7) | M2 | acceptance | n/a |
| FW-025 | Reference design (gantry): the firmware shall send the "heartbeat" to the E-stop relay while running, so that a freeze lets the watchdog cut motor power. | Requirements doc, Controller requirements (gantry messages); Requirements doc, Test cases (G8) | M2 | acceptance | not set (owed by gantry team: heartbeat timeout) |
| FW-026 | Reference design (gantry): a new target during a move shall be handled by the one policy set as a firmware parameter (cancel and replace, or queue), identically every run. | Requirements doc, Controller requirements (gantry reference controller); Planning rulings, Still open, owed by the gantry team (new-target-during-move policy) | M2 | acceptance | n/a |
| FW-027 | Reference design (gantry): a lift command while the gantry is moving shall be handled by the one policy set as a firmware parameter (queue or reject), identically every run. | Requirements doc, Test cases (G13); Planning rulings, Still open, owed by the gantry team (lift-while-moving policy) | M2 | acceptance | n/a |
| FW-028 | Reference design (gantry): rapid-fire and conflicting commands shall never produce overlapping actuator commands. | Requirements doc, Test cases (G14) | M2 | acceptance | n/a |
| FW-029 | Reference design (gantry): the firmware shall accept the operator messages "target (x, y)" and "home" and report "current position, ready / fault". | Requirements doc, Controller requirements (gantry messages); ADR 0012, D2 | M1 | acceptance | n/a |
| FW-030 | Reference design (gantry): the firmware shall log every state change, command and fault through the runtime's fault log. | Requirements doc, Controller requirements (logging) | M1 | acceptance | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| New-target-during-move policy (FW-026) | not set | gantry team |
| Lift-while-moving policy (FW-027) | not set | gantry team |
| Workspace dimensions (FW-016) | not set | gantry team |
| Rated load and overload limit (FW-018, FW-019) | not set | gantry team |
| Encoder mismatch threshold (FW-020) | not set | gantry team |
| Acceleration and deceleration limits (FW-022) | not set | gantry team |
| Slow-down zones near targets and limits (FW-023) | not set | gantry team |
| Heartbeat period and timeout (FW-025) | not set | gantry team |
| Encoder placement | not set | gantry team |
| Firmware-declared tick rate (FW-010) | Checked against the design's control rate at load; a mismatch is an error | set (C6 ruling) |
| Board interface sizes (FW-010) | Fixed contract constants: 32 digital in, 8 counters, 16 ADC, 64-byte UART, 8 axes, 32 digital out, 16 events | set (C6 ruling) |
| STM32 part and arm-none-eabi target flags | not set | gantry team, after the peripheral smoke test |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Calling the entry points, freeze, reset and brownout | CTL |
| E-stop relay and watchdog cutting motor power | SIG |
| Pass criteria and scoring of G1 to G16 | ACC, SCO |
| G12, G15 and G16 (swing, structural, motor sweep) | ACC, INT; no firmware behavior beyond FW-013 to FW-030 |
| Interrupt-dependent behavior | Emulated mode only, milestone 4 (ADR 0011) |
