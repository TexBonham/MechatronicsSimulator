# Spec: Recorder and archives (REC)

## Purpose

The recorder (`mechsim/record/`) is the single sink of every run: it captures every tick's physical, electrical,
controller and event data with the simulated time, writes one HDF5 archive per run that identifies exactly what
produced it, exports CSV, JSON and MAT files, and exposes a streaming seam from milestone 1. It computes nothing about
the recording; integrity (INT), scoring (SCO) and the viewer (VIE) read from it.

"Architecture doc" below is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`;
"Requirements doc" is `docs/Mechatronic Simulator — Requirements.md`.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| KER | REC-001, REC-011, REC-019, REC-023 | KER-014, KER-018, KER-022, KER-024 |
| PHY | REC-002, REC-003, REC-004, REC-026 | PHY-003, PHY-004, PHY-007 |
| SIG | REC-005, REC-006, REC-009, REC-027 | SIG-013, SIG-016, SIG-026, SIG-028 |
| CTL | REC-007 | CTL-018 |
| TST | REC-012, REC-013, REC-025 | TST-022, TST-031, TST-033 |
| INT | REC-001, REC-004, REC-010 | INT-022, INT-023, INT-024 |
| SCO | REC-001, REC-008 | SCO-001 |
| FAC | REC-019, REC-024 | FAC-006, FAC-008 |
| VIE | REC-024 | VIE-001 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| REC-001 | The recorder shall be the single sink of run data: integrity, scoring, the viewer and exports read the recording and never live run state, except the tier 1 integrity party, which reads live member loads through the kernel. | Architecture doc, Modules: outputs, introduction; ADR 0010, Decision (tier 1 is a kernel party); KER-018; INT-023 | M1 | unit | n/a |
| REC-002 | The recorder shall record every body's pose and velocity on every physics tick. | Architecture doc, Modules: outputs, Recorder row; Data flows, run step 4 | M1 | unit | n/a |
| REC-003 | The recorder shall record every joint force and contact force on every physics tick. | Architecture doc, Modules: outputs, Recorder row | M1 | unit | n/a |
| REC-004 | The recorder shall record every structural member's end-loads on every physics tick. | Architecture doc, Modules: outputs, Recorder row; ADR 0010, Decision, interface requirement | M3 | unit | n/a |
| REC-005 | The recorder shall record every signal on every pin, in both directions between the signal layer and the controller. | Architecture doc, Modules: outputs, Recorder row; Modules: core, Signal layer row | M1 | unit | n/a |
| REC-006 | The recorder shall record the state of every power rail. | Architecture doc, Modules: outputs, Recorder row | M1 | unit | n/a |
| REC-007 | The recorder shall record the controller states and the controller fault log the controller runtime reports. | Architecture doc, Modules: outputs, Recorder row; Modules: core, Controller runtime row | M1 | unit | n/a |
| REC-008 | The recorder shall record every injected fault and every safety event with its simulated time. | Architecture doc, Modules: outputs, Recorder row | M1 | unit | n/a |
| REC-009 | The recorder shall record every operator command and status message with its simulated time, using the message labels verbatim. | Architecture doc, Modules: outputs, Recorder row; ADR 0012 (D2) | M1 | unit | n/a |
| REC-010 | The recorder shall record every in-loop integrity event with its simulated time. | ADR 0010, Consequences | M3 | unit | n/a |
| REC-011 | The recorder shall stamp every recorded sample and event with the simulated time. | Architecture doc, Modules: outputs, Recorder row | M1 | unit | n/a |
| REC-012 | The recorder shall write one HDF5 archive per run through h5py. | ADR 0012 (L8) | M1 | unit | n/a |
| REC-013 | The run archive shall carry the design file, the test case, the seed, the controller mode and the git hashes as attributes. | ADR 0012 (L8) | M1 | unit | n/a |
| REC-014 | The run archive shall carry the design version as the semver string plus the design file's git blob hash. | ADR 0012 (D3) | M1 | unit | n/a |
| REC-015 | The recorder shall export any recorded series as CSV. | Architecture doc, Modules: outputs, Recorder and Exports rows; Requirements doc, Outputs and acceptance criteria, Outputs per run | M1 | unit | n/a |
| REC-016 | The recorder shall export any recorded series as JSON. | Architecture doc, Modules: outputs, Recorder and Exports rows; Requirements doc, Outputs and acceptance criteria, Outputs per run | M1 | unit | n/a |
| REC-017 | The recorder shall export any recorded series as a MAT file. | Architecture doc, Modules: outputs, Recorder and Exports rows; Requirements doc, Outputs and acceptance criteria, Outputs per run | M1 | unit | n/a |
| REC-018 | Every archive series and export shall hold SI values and state the unit of each quantity explicitly. | Architecture doc, Repository starting point, invariant 3; ADR 0012 (L5) | M1 | unit | n/a |
| REC-019 | The recorder shall expose a streaming interface through which a consumer receives recorded ticks during a run. | ADR 0006, Decision, Recorder; Ruling R-15 | M1 | unit | n/a |
| REC-020 | A streaming consumer shall not change the recording or the run it observes. | ADR 0006, Decision, Recorder; Architecture doc, Repository starting point, invariant 4 | M1 | unit | n/a |
| REC-021 | No run archive shall be tracked in git. | ADR 0012 (D4) | M1 | invariant | n/a |
| REC-022 | Two archives from the same design, test case, seed and controller mode shall be identical in every recorded series and attribute. | Architecture doc, Repository starting point, invariant 4; Requirements doc, Physics and structural analysis requirements, Dynamics, Deterministic runs | M1 | invariant | n/a |
| REC-023 | The recorder shall write no wall-clock time or other nondeterministic value into the recorded series or attributes. | Architecture doc, Repository starting point, invariant 4 | M1 | unit | n/a |
| REC-024 | A run archive shall hold everything replay needs, so that a run replays without the engine that produced it. | ADR 0006, Consequences | M1 | unit | n/a |
| REC-025 | The run archive shall record how the run ended: end condition reached, or the run error report with its simulated time. | Architecture doc, Data flows, run step 4; Repository starting point, invariant 7 | M1 | unit | n/a |
| REC-026 | The recorder shall record every rope tension on every physics tick. | Architecture doc, Modules: core, Physics engine row, Interface out | M3 | unit | n/a |
| REC-027 | The recorder shall record the actuator torques and forces the signal layer applies to physics on every tick. | Architecture doc, Modules: core, Signal layer row, Interface out; Requirements doc, Outputs and acceptance criteria, Outputs per run | M1 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Recording rate of pulse-level signals finer than the physics step | not set | phase 1 contracts |
| HDF5 archive layout (groups, datasets, attribute names) | not set | phase 1 schemas |
| Which git hashes are recorded (repository, firmware tree, design file) | not set | phase 1 schemas |
| Archive comparison rule for run-twice determinism (bitwise file or recorded content) | recorded content | phase 1 contracts |
| Streaming interface shape and back-pressure | C++ seam: `RecorderSink::subscribe`, callback per row, synchronous, in subscription order (brief C5); facade-side shape and back-pressure not set | phase 1 ADR (facade protocol) |
| MAT file version | not set | phase 1 contracts |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Computing stress, safety factors and load cases from the recording | INT |
| Evaluating pass criteria | SCO |
| Decimation for clients | FAC |
| Replay display | VIE |
| Thermal losses (recorded but not integrated in v1) | Roadmap, Architecture doc Seams table |
