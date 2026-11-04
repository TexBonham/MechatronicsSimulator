# Spec: Kernel (KER)

## Purpose

The co-simulation kernel under `core/kernel/` owns simulated time. It steps every registered party (physics, signal
layer, controller runtime, the tier 1 integrity check and any later party) at its own rate, exchanges coupling variables
at sync points in a fixed order, derives every party's random generator from the run seed, and ends the run with a
report on any party fault. What each party computes is left to ENG, SIG, CTL and INT; storing what the kernel streams is
left to REC.

Sources: "Architecture doc" is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`;
"Requirements doc" is `docs/Mechatronic Simulator — Requirements.md`.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| ENG | KER-004, KER-009, KER-010, KER-015, KER-016, KER-018 | ENG-002, ENG-023, ENG-024, ENG-025, ENG-026, ENG-027, ENG-028 |
| SIG | KER-009, KER-010, KER-011, KER-018, KER-020, KER-021 | SIG-005, SIG-012, SIG-013, SIG-016, SIG-024, SIG-028 |
| CTL | KER-005, KER-006, KER-009, KER-018, KER-020, KER-021 | CTL-002, CTL-008, CTL-011, CTL-014, CTL-019 |
| REC | KER-009, KER-022 | REC-001, REC-011, REC-019, REC-023 |
| INT | KER-009, KER-018, KER-019 | INT-023, INT-024, INT-026, INT-027 |
| TST | KER-014, KER-017 | TST-021, TST-022, TST-028, TST-036 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| KER-001 | The kernel shall own simulated time; no party shall advance simulated time except through a kernel step. | Architecture doc, Modules: core (Co-simulation kernel row) | M1 | unit | n/a |
| KER-002 | The kernel shall start every run at simulated time t = 0. | Architecture doc, Data flows (one run, step 3) | M1 | unit | n/a |
| KER-003 | The kernel shall step each registered party at that party's own rate, including signal parties at rates finer than the physics step and an emulator advanced in chunks of virtual time. | Architecture doc, Modules: core (Co-simulation kernel row) | M1 | unit | n/a |
| KER-004 | The kernel shall take the physics step from the run configuration, defaulting to 1 ms, with no rate held as a code constant. | Architecture doc, Data flows (per-tick order); ADR 0011, C4 and Consequences | M1 | unit | n/a |
| KER-005 | The kernel shall take the control tick rate from the run configuration, defaulting to 1 kHz, with no rate held as a code constant. | ADR 0011, C4 and Consequences | M1 | unit | n/a |
| KER-006 | The kernel shall support a control tick rate unequal to the physics rate. | ADR 0011, C4 | M1 | unit | n/a |
| KER-007 | The kernel shall reject, before t = 0, a rate combination that the rate ratio rule does not allow, with an error naming the parties. | Architecture doc, Repository starting point (invariant 7); ADR 0011, C4 | M1 | unit | n/a |
| KER-008 | The kernel shall exchange coupling variables between parties only at sync points. | Architecture doc, Modules: core (Co-simulation kernel row) | M1 | unit | n/a |
| KER-009 | Within each physics tick the kernel shall run, in this order: physics advances one step and publishes poses, velocities, forces and member end-loads; the tier 1 integrity party evaluates the published end-loads; sensor models sample the new state; the controller runtime advances by the same interval; the signal layer collects pin outputs; driver and stepper models turn them into motor shaft torque gated by the power rail, which the transmissions turn into joint force; the tick's recorder row is written last. | Architecture doc, Data flows (within one physics tick, steps 1 to 5); ADR 0010, Decision (tier 1 is a kernel party) | M1 | unit | n/a |
| KER-010 | Actuator forces computed in tick n shall be applied by physics in step n + 1. | Architecture doc, Data flows (within one physics tick, step 5) | M1 | unit | n/a |
| KER-011 | Between sync points the kernel shall hold every coupling variable a party consumes at its value from the last sync point (zero-order hold). | Architecture doc, Data flows (within one physics tick, steps 3 and 4: inputs applied for the interval, levels latched); ADR 0011, C3 | M1 | unit | n/a |
| KER-012 | The kernel shall accept one integer seed per run and derive one child generator per party instance from it, in a fixed instance order. | D1; ADR 0012, D1 | M1 | unit | n/a |
| KER-013 | The kernel shall step parties within a sync point in a fixed order taken from the model data, independent of memory addresses, hashing and thread timing. | Architecture doc, Repository starting point (invariant 4) | M1 | unit | n/a |
| KER-014 | Two runs with the same design, test case, seed and controller mode shall produce identical recordings. | Architecture doc, Repository starting point (invariant 4); Architecture doc, Modules: core (Co-simulation kernel row) | M1 | invariant | identical |
| KER-015 | The kernel shall end the run with a report when any party raises a fault, naming the party, the simulated time and the cause. | Architecture doc, Repository starting point (invariant 7) | M1 | unit | n/a |
| KER-016 | The kernel shall report constraint drift and solver failure from physics as run errors, never as recorded values. | Architecture doc, Modules: core (Co-simulation kernel row); Requirements doc, Physics and structural analysis requirements (Dynamics, solver health) | M1 | unit | n/a |
| KER-017 | The kernel shall end the run when the test case's end condition is met or a run error occurs, and shall report which. | Architecture doc, Data flows (one run, step 4) | M1 | unit | n/a |
| KER-018 | Physics, the signal layer, the controller runtime and the tier 1 integrity check shall each register with the kernel through one party interface (rate, step, published and consumed coupling variables). | Architecture doc, Modules: core (Co-simulation kernel row); ADR 0010, Decision (tier 1 is a kernel party) | phase 1 | unit | n/a |
| KER-019 | A party implemented on the Python slow path shall register and step through the same party interface as a C++ party. | ADR 0005, Decision (plugin tiers); ADR 0010, Consequences | phase 1 | unit | n/a |
| KER-020 | The kernel shall route physics outputs only to the signal layer and other non-controller parties; the controller runtime party shall consume only pin inputs and power rail state. | Architecture doc, Repository starting point (invariant 1); Architecture doc, System overview | M1 | invariant | n/a |
| KER-021 | While the controller runtime is frozen, the kernel shall keep stepping physics and the signal layer. | Architecture doc, Data flows (fault fan-out, controller freeze); Requirements doc, Controller requirements (safety outside the controller) | M2 | unit | n/a |
| KER-022 | The kernel shall stream every tick's published coupling variables and signals to a recorder sink interface, present from milestone 1. | R-15; Architecture doc, Modules: core (Co-simulation kernel row, interface out) | M1 | unit | n/a |
| KER-023 | The kernel shall deliver a scheduled operator command or injected fault to its owning party at the sync point given by the scheduling rule. | Architecture doc, Data flows (one run, step 3; fault fan-out) | M1 | unit | n/a |
| KER-024 | The kernel shall advance simulated time independently of wall-clock time. | Requirements doc, Physics and structural analysis requirements (Dynamics: runs need not be real time) | M1 | unit | n/a |
| KER-025 | A party added later, such as a thermal party, shall register with the kernel without a change to kernel code. | Architecture doc, Seams reserved for the roadmap (thermal engine) | roadmap | unit | n/a |
| KER-026 | The kernel shall carry no Qt or display dependency. | Architecture doc, Repository starting point (invariant 9) | M1 | invariant | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Rate ratio rule (KER-007): integer multiples of the physics step only, or arbitrary | arbitrary positive rational rates; base rate = lcm of the physics rate and every party rate, computed at run start and recorded in `RunReport` and `RunConfig`; int64 overflow and an end time off the base tick are config errors before t = 0; owner ruling 2026-11-04 | owner (brief C5 fix) |
| Scheduling rule for commands and faults (KER-023): first sync point at or after the scheduled time, or nearest | not set | owner |
| Representation of simulated time (integer base ticks or floating seconds) | integer ticks (`SimTime`, `Duration`, int64) of a per-run base period 1 / base rate; seconds only through `to_seconds()`; owner ruling 2026-11-04 | owner (brief C5 fix) |
| Party order source in the model data (KER-013) | not set | owner, phase 1 contracts |
| Party interface signature (KER-018) | `mechsim::kernel::Party`: `name`, `nominal_rate`, `published`, `consumed`, `start`, `step(t, dt)`, `on_fault`; defaulted milestone 1 hooks `set_seed`, `read_inputs`, `write_outputs`, `finished`, `record`; `core/kernel/CLAUDE.md` | phase 1 contracts (brief C5); hooks M1-B |
| Coupling exchange (KER-008, KER-011) | value table keyed by dotted name, one publisher per name; a step is `read_inputs` (consumed names only), `step`, `write_outputs` (published names only); unwritten values hold; values scalar only (double, int64, bool), unset until first written, fixed type per run; a consumed name nobody publishes, or two publishers of one name, is a config error before t = 0 | M1-B; owner confirmation pending |
| Child seed derivation (KER-012) | SplitMix64 outputs from the run seed, one per party in registration order, delivered by `set_seed` before `start` | M1-B; owner confirmation pending |
| End conditions (KER-017) | `EndCondition{end_time` (maximum duration, always), `stop}` plus `EndTriggers{at_time, on_signal` (published boolean true), `party_state` (party `finished()`)`}` set by `Kernel::set_end_triggers`; checked only at physics-step boundaries, after the row, in the order signal, party state, stop, at time, so a run never ends between rows; `at_time` must be a whole number of physics steps; `RunReport.ticks` counts base ticks and `end_reason` names the condition; a non-boolean end signal is a run error naming its publisher | M1-B, for owner review |
| Controller and physics rates by role (KER-004, KER-005) | `Party::role()`; effective rate = `party_rates[name]`, else `control_rate` for a controller-role party, else `nominal_rate()`; `RunReport.party_rates` records rate and source; a physics-role party off `physics_rate` is a config error before t = 0 | M1-B, for owner review |
| Throwing recorder subscriber (KER-022, REC-020) | dropped from the sink, its error kept and copied into `RunReport.sink_errors`; the run continues; unsubscribing it once is not an error | M1-B; owner confirmation pending |
| Recorder sink interface (KER-022) | `mechsim::kernel::RecorderSink`: `write_row(TickRecord)`, `subscribe`, `unsubscribe`; `core/kernel/CLAUDE.md`; each row carries every coupling variable (`TickRecord::coupling`) and each party's `record()` slice; a throwing sink is a run error of party `recorder` | phase 1 contracts (brief C5); row content M1-B |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| The physics computation inside a step | ENG, PHY |
| Sensor, driver, rail and operator-link models | SIG |
| Firmware entry points and the emulator bridge | CTL |
| Limit evaluation on member loads | INT |
| Storing and exporting the stream | REC |
| Loading the test case and its timelines | TST |
