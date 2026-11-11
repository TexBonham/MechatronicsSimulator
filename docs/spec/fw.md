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
| FW-020 | Reference design (gantry): when encoder position disagrees with the commanded position, the firmware shall stop, fault and require re-homing. | Requirements doc, Controller requirements (gantry reference controller); Requirements doc, Test cases (G10) | M2 | acceptance | not set (owed by gantry team, proposed 2 mm) |
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
| FW-031 | Reference design (gantry): the committed `firmware/app/gantry/channels.h` shall be the builder's generated channel header for the gantry design (`MECHSIM_CH_<SIGNAL>`), byte for byte, and the firmware shall name channels only through it. | M1-X3 coordinator ruling 4 (2026-11-04), for owner confirmation | M1 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| New-target-during-move policy (FW-026) | not set; firmware placeholder `GANTRY_NEW_TARGET_POLICY` = reject (`fault 3`), queue (one slot, second refused `fault 5`) selectable | gantry team |
| Lift-while-moving policy (FW-027) | not set; firmware placeholder `GANTRY_LIFT_WHILE_MOVING_POLICY` = reject (`fault 3`), queue selectable, defaulting to the new-target policy; applies to `lift` while any axis is MOVING. Targets and lifts share the one queue slot (a second refused `fault 5`); a queued item starts on the tick after arrival, the other axes held | gantry team |
| Workspace dimensions (FW-016) | not set; firmware placeholder 1.0 m by 0.8 m, bounds inclusive, origin at the min switches | gantry team |
| Rated load and overload limit (FW-018, FW-019) | not set; firmware placeholder 3.45 kg (rated 3 kg + 15 %, ruling 6), load cell 1000 ADC counts/kg from 0 counts, over means strictly above. Lift up (a `lift` z above the current position, at command time and again when a queued lift starts) refused with `fault 10` and the state kept; lower always executed. Mid-lift: over the limit on 2 consecutive ticks of an upward lift profile reports the refusal (`fault 10`, status line, event 3) and stops at the deceleration limit, ending in READY with the normal arrival lines; a single tick is ignored | gantry team |
| Encoder mismatch threshold (FW-020) | not set; firmware placeholder 120 counts (2 mm at 60000 counts/m, ruling 6). While MOVING, each tick per axis, \|encoder counts since the origin - steps × counts/steps per m\| over the threshold stops every axis at the deceleration limit (state MOVING until at rest, other commands refused `fault 3`), then latches `fault 12`; `target`, `lift` and `reset` refused with 12, exit by `home` only | gantry team |
| Unexpected limit switch (FW-021) | firmware: a travel-end switch active in READY or MOVING outside the slow zone (50 mm) of its own end trips: in READY at once, in MOVING after a stop at the deceleration limit as for FW-020; latches `fault 13`, refusals with 13, exit by `home` only. Not checked in NOT_HOMED, HOMING or FAULT | firmware |
| E-stop (FW-024) | firmware: E-stop relay status input low (motor rail cut) in HOMING, READY or MOVING latches `fault 11` on that tick with no steps (profiles end at once); `target`, `lift`, `home` refused with 11; `reset` with the input high goes to NOT_HOMED (`status 1 0`, clarification 7), with it low is refused with 11. In NOT_HOMED the input is not checked. `reset` outside FAULT refused `fault 14`; in another FAULT refused with that fault's refusal code | firmware; clarification 7 open |
| Acceleration and deceleration limits (FW-022) | not set; firmware placeholder 0.5 m/s^2, cruise 0.2 m/s, homing 0.02 m/s; per-axis trapezoid in Q16 integer steps | gantry team |
| Slow-down zones near targets and limits (FW-023) | not set; firmware placeholder 50 mm zone at each travel end capped at 0.05 m/s, entered under the deceleration limit; every move decelerates to its target | gantry team |
| Heartbeat period and timeout (FW-025) | not set; firmware placeholder period 10 ticks (10 ms at 1 kHz, ruling 6): the heartbeat output toggles on the first tick after init and every 10 ticks after, in every state but FAULT, where it holds low; on leaving FAULT it restarts from its first toggle. Watchdog timeout placeholder 50 ms (ruling 6, SIG) | gantry team |
| Encoder placement | not set; firmware placeholder 60000 counts/m, the sum of the channel A and B edge counts (quadrature), 80000 steps/m | gantry team |
| Firmware-declared tick rate (FW-010) | Checked against the design's control rate at load; a mismatch is an error | set (C6 ruling) |
| Board interface sizes (FW-010) | Fixed contract constants: 32 digital in, 8 counters, 16 ADC, 64-byte UART, 8 axes, 32 digital out, 16 events | set (C6 ruling) |
| STM32 part and arm-none-eabi target flags | not set | gantry team, after the peripheral smoke test |
| Gantry operator protocol (FW-029; SIG-027) | ASCII lines, `\n` terminated (a trailing `\r` dropped), fields split by spaces. In: `target <x> <y>` in metres (optional sign, up to 6 decimals), `home`, `lift <z>` (metres above the lift home switch, same format, travel 0 to 0.5 m inclusive placeholder; clarification 8), `reset`. Out: `current_position <x> <y>` (metres, 6 decimals, from the encoder deltas every 100 ticks in READY and MOVING and on arrival), `status <state> <fault>` (the design's "ready / fault" message, DES-045 code tables: on homing done and move arrived with fault 0, on every refusal with that tick's fault code and the state kept, on a latched fault with state 5). Lines over 48 bytes, unknown words and bad numbers are refused with `fault 4` | set (owner ruling 2026-11-04) |
| Gantry state, fault and event codes (FW-013 to FW-030) | `state_code`: 1 NOT_HOMED, 2 HOMING, 3 READY, 4 MOVING, 5 FAULT. Refusals (state kept, `fault_code` for that tick): 1 not homed, 2 outside workspace (or lift travel), 3 busy, 4 malformed, 5 queue full, 6 in fault, 10 overload, 14 nothing to reset. Latched in FAULT until `home`: 7 homing found no switch within 1.2 m, 8 event queue overflow, 9 transmit queue overflow, 12 encoder mismatch, 13 limit trip; until `reset`: 11 E-stop. In FAULT, commands are refused with 11, 12 or 13 when that is the latched fault, otherwise with 6. Events: 1 state change (from << 8 \| to), 2 command (1 target, 2 home, 3 unknown, 4 lift, 5 reset), 3 refusal, 4 fault, 5 origin set (axis), 6 motion start (axis), 7 motion end (axis; the profile reached zero velocity, or ended by a fault); axes 0 X, 1 Y, 2 lift. Every start is followed by its end before the next start on that axis (FW-028); homing starts a profile on every axis, ended at its origin; queued beyond 16 per tick and 64 bytes of UART per tick, continued next tick | firmware (`firmware/app/gantry/gantry_config.h`) |
| Gantry channel map | The builder's generated header (`MECHSIM_CH_<SIGNAL>`, per peripheral mode numbered from 0 in pin-table order), committed as `firmware/app/gantry/channels.h` (FW-031) | set (M1-X3 ruling 4) |
| Lift axis (FW-017, FW-018) | Third axis: homed against the lift min switch with X and Y, the same trapezoid, slow zones and switches as X and Y. Its driver is enabled in HOMING, READY and MOVING only; the brake release output is high exactly while the lift driver is enabled outside FAULT (power-off brake, ruling 1), so after init every output is low but the heartbeat | gantry team (brake, ruling 1) |
| Firmware channels not yet bound by the design | `gantry_config.h` placeholders (ruling 6) until the integration wave aliases the generated names: load cell ADC 0, E-stop relay status digital in 6 (high = rail live), brake release digital out 7 (high = released); the heartbeat aliases the generated `MECHSIM_CH_WATCHDOG_HEARTBEAT` | integration wave |
| Direction and enable source (invariant 1) | The pin-table-bound `digital_out` channels are the single source of truth for driver direction and enable; the native shim fills the stepper `direction[]` and `enable[]` arrays from them, and firmware holds those arrays low; `step_count[]` stays the firmware's | set (review ruling 2026-11-04) |
| Homing maximum travel (FW-014) | not set; firmware placeholder 1.2 m per axis before `fault 7` | gantry team |
| Limit switch active level | not set; firmware placeholder high | gantry team |
| Driver direction levels | not set; firmware placeholder high for positive travel, low for negative | gantry team |
| Driver enable levels | not set; firmware placeholder high active, low inactive | gantry team |
| Position status period (FW-029) | not set; firmware placeholder 100 ticks | gantry team |
| Firmware TX queue | firmware placeholder 512 bytes, drained 64 per tick; overflow latches `fault 9` | firmware |
| Firmware event queue | firmware placeholder 64 events, drained 16 per tick; overflow latches `fault 8` | firmware |
| Overflow while in FAULT (FW-030) | In FAULT, further event-log and TX overflows are dropped silently behind the latched fault code (milestone 1); milestone 2 adds a counter | firmware |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Calling the entry points, freeze, reset and brownout | CTL |
| E-stop relay and watchdog cutting motor power | SIG |
| Pass criteria and scoring of G1 to G16 | ACC, SCO |
| G12, G15 and G16 (swing, structural, motor sweep) | ACC, INT; no firmware behavior beyond FW-013 to FW-030 |
| Interrupt-dependent behavior | Emulated mode only, milestone 4 (ADR 0011) |
