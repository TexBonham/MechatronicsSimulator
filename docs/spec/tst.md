# Spec: Test runner (TST)

## Purpose

The test runner (`mechsim/testing/`) loads a test case file, sets the initial state, plays the operator command timeline
through the operator link, fans injected faults out to the module that owns the thing being broken, and runs the kernel
to the end condition or a run error. It runs suites, parameter sweeps and seeded variations in batch, and owns the CLI,
the first client and the one CI uses. Evaluating pass criteria and diffing against baselines is scoring's job (SCO);
storing the recording is the recorder's (REC).

"Architecture doc" below is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`;
"Requirements doc" is `docs/Mechatronic Simulator — Requirements.md`.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| KER | TST-011, TST-012, TST-021, TST-022, TST-028, TST-036 | KER-002, KER-012, KER-014, KER-017, KER-023 |
| SIG | TST-013, TST-014, TST-016, TST-017, TST-019, TST-037 | SIG-006, SIG-019, SIG-026, SIG-034, SIG-036 |
| PHY | TST-015, TST-020 | PHY-006, PHY-008, ENG-035, ENG-046 |
| CTL | TST-017, TST-018, TST-036 | CTL-001, CTL-011, CTL-013, CTL-021 |
| DES | TST-001, TST-011 | DES-008, DES-011, DES-036 |
| REC | TST-022, TST-023, TST-031, TST-033 | REC-012, REC-013, REC-025 |
| SCO | TST-005, TST-023, TST-027, TST-032 | SCO-001, SCO-009, SCO-011 |
| FAC | TST-033, TST-034 | FAC-001, FAC-010 |
| ACC | TST-024, TST-032, TST-037 | ACC-022, ACC-023, ACC-024, ACC-036 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| TST-001 | A test case file shall name the design version it runs against, as the design's semver string. | Architecture doc, Modules: inputs, Test case files row; ADR 0012 (D3) | M1 | unit | n/a |
| TST-002 | A test case file shall define the initial state the run starts from. | Architecture doc, Modules: inputs, Test case files row; Requirements doc, Test cases, Definition | M1 | unit | n/a |
| TST-003 | A test case file shall define a timeline of operator commands, each with its scheduled simulated time. | Architecture doc, Modules: inputs, Test case files row; Requirements doc, Test cases, Definition | M1 | unit | n/a |
| TST-004 | A test case file shall define a timeline of injected faults, each with its fault type, target, start time and window. | Architecture doc, Modules: inputs, Test case files row; Requirements doc, Test cases, Fault injection | M2 | unit | n/a |
| TST-005 | A test case file shall define its pass criteria, each naming a measured value and a limit. | Architecture doc, Modules: inputs, Test case files row; Requirements doc, Test cases, Definition | M1 | unit | n/a |
| TST-006 | A test case file shall carry one integer random seed. | Architecture doc, Modules: inputs, Test case files row; ADR 0012 (D1) | M1 | unit | n/a |
| TST-007 | A test case or suite file shall be able to define parameter sweeps and seeded variations. | Architecture doc, Modules: inputs, Test case files row; Requirements doc, Test cases, Batches and sweeps | M3 | unit | n/a |
| TST-008 | A suite file shall name the regression baseline it is compared against. | Architecture doc, Modules: inputs, Test case files row; ADR 0012 (D4) | M2 | unit | n/a |
| TST-009 | The test runner shall validate every test case file against its schema in `schemas/` before a run starts and reject an invalid file with an error naming the failing field. | Architecture doc, Repository starting point, invariant 3; ADR 0012 (L4) | M1 | unit | n/a |
| TST-010 | The test runner shall parse quantities in test case files from unit strings into SI values at load, and never use unit objects inside the run. | ADR 0012 (L5) | M1 | unit | n/a |
| TST-011 | The test runner shall load the design the test case names, have the model builder build it, and end with an error when the design file's version differs from the version the test case names. | Architecture doc, Data flows, run step 1; ADR 0012 (D3) | M1 | unit | n/a |
| TST-012 | The test runner shall set the test case's initial state before the kernel starts at t = 0. | Architecture doc, Modules: core, Test runner row; Data flows, run step 3 | M1 | unit | n/a |
| TST-013 | The test runner shall deliver each operator command through the operator link of the signal layer at its scheduled simulated time, and by no other path to the controller. | Architecture doc, Modules: core, Test runner row; Data flows, run step 3; Repository starting point, invariant 1 | M1 | unit | n/a |
| TST-014 | The test runner shall route a sensor stuck, noisy or dropped fault to the signal layer, which sets that sensor model's failure mode for the fault window. | Architecture doc, Data flows, fault table, row 1 | M2 | unit | n/a |
| TST-015 | The test runner shall route a belt or rope slip fault to physics, which makes that transmission model slip for the fault window. | Architecture doc, Data flows, fault table, row 2 | M2 | unit | n/a |
| TST-016 | The test runner shall route a motor stall or missed-steps fault to the signal layer, which reduces the behavioral stepper's available torque for the fault window. | Architecture doc, Data flows, fault table, row 3 | M2 | unit | n/a |
| TST-017 | The test runner shall route a power loss and restore fault to the signal layer, which drops and restores the rail voltage; the controller runtime resets the controller only when the rail falls below the chip's brownout threshold. | Architecture doc, Data flows, fault table, row 4 | M2 | unit | n/a |
| TST-018 | The test runner shall route a controller freeze fault to the controller runtime, which stops calling the tick function (native) or halts the core (emulated) for the fault window. | Architecture doc, Data flows, fault table, row 5 | M2 | unit | n/a |
| TST-019 | The test runner shall route an E-stop press to the signal layer, which opens the relay on the motor rail; the test runner sends the controller nothing for it. | Architecture doc, Data flows, fault table, row 6; Repository starting point, invariant 1 | M2 | unit | n/a |
| TST-020 | The test runner shall route a payload change to physics, which changes the payload mass or attachment at the scheduled time. | Architecture doc, Data flows, fault table, row 7 | M2 | unit | n/a |
| TST-021 | The test runner shall run the kernel until the test case's end condition is met. | Architecture doc, Modules: core, Test runner row; Data flows, run step 4 | M1 | unit | n/a |
| TST-022 | The test runner shall end the run on a solver failure, constraint drift beyond limit or emulator fault, and report the error with its simulated time; such a run is never reported as passed. | Architecture doc, Data flows, run step 4; Repository starting point, invariant 7 | M1 | unit | n/a |
| TST-023 | The test runner shall hand every run to scoring with the reason it ended, including runs ended by a solver error or a fault. | Architecture doc, Modules: core, Test runner row; Data flows, run step 5; Architecture doc, Repository starting point, invariant 7 | M1 | unit | n/a |
| TST-024 | The test runner shall run every case of a suite in one batch invocation. | Requirements doc, Test cases, Batches and sweeps; Requirements doc, Outputs and acceptance criteria, criterion 2 | M1 | unit | n/a |
| TST-025 | The test runner shall sweep any design or test case parameter over the values a sweep definition lists, one run per value, each run recorded with the value it used. | Requirements doc, Test cases, Batches and sweeps; Architecture doc, Modules: core, Test runner row | M3 | unit | n/a |
| TST-026 | The test runner shall run seeded randomized variations of a case, each variation recorded with its own integer seed so that it can be rerun alone with an identical result. | Requirements doc, Test cases, Batches and sweeps; Architecture doc, Seams reserved for the roadmap, Monte Carlo row; ADR 0012 (D1) | M3 | unit | n/a |
| TST-027 | The test runner shall rerun a suite and pass its scoring summary with the suite's regression baseline to scoring for the regression diff. | Requirements doc, Test cases, Regression; Architecture doc, Modules: core, Test runner row; ADR 0012 (D4) | M2 | unit | n/a |
| TST-028 | The test runner shall seed each run from the one integer seed, spawning one child generator per module instance from a SeedSequence in a fixed order. | ADR 0012 (D1) | M1 | unit | n/a |
| TST-029 | The test runner shall take every random draw it makes from its own seeded child generator and from no other source. | Architecture doc, Repository starting point, invariant 4; ADR 0012 (D1) | M1 | unit | n/a |
| TST-030 | The CLI command `validate` shall validate design, test case and suite files against their schemas without running anything. | ADR 0012 (L9); Ruling L9 | M1 | unit | n/a |
| TST-031 | The CLI command `run` shall run one test case and write its run archive. | ADR 0012 (L9); Ruling L9 | M1 | unit | n/a |
| TST-032 | The CLI command `suite` shall run a suite in batch and write the suite's pass/fail report. | ADR 0012 (L9); Ruling L9 | M1 | unit | n/a |
| TST-033 | The CLI command `replay` shall replay a run from its archive alone, without the engine that produced it. | ADR 0012 (L9); ADR 0006, Decision, Replay | M1 | unit | n/a |
| TST-034 | The CLI shall drive the simulator only through the facade and hold no domain logic of its own. | ADR 0006, Decision, Facade and First client; Ruling R-15 | M1 | unit | n/a |
| TST-035 | The CLI shall run with no display, so that every run CI performs is a headless CLI run. | ADR 0006, Consequences; Ruling R-15 | M1 | unit | n/a |
| TST-036 | The test runner shall take the controller mode (native or emulated) as a run input and pass it to the kernel and controller runtime. | Architecture doc, Modules: core, Test runner row, Interface in | M1 | unit | n/a |
| TST-037 | The fault timeline shall be able to inject a forced-level fault on a digital sensor, which the test runner routes to the signal layer for the fault window. | Requirements doc, Test cases (G11); Architecture doc, Data flows (fault fan-out, sensor stuck, generalized by SIG-036) | M2 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| End condition form (time limit, event, both) | exactly one trigger (a time, an observed operator message, or a controller state) plus a required `max_duration` (`schemas/test_case.json`) | set in phase 1 C4 |
| Delivery rule for a command or fault scheduled between ticks | not set | phase 1 contracts |
| Sweep parameter addressing syntax | `${name}` for a design open parameter, else `<bodies\|joints\|components>.<item name>.<field>[.<field>]` into the design file | set in phase 1 C4 |
| Seed of each seeded variation (derived from the case seed or listed) | not set; the case stores only the count and a distribution per varied parameter | runner (TST-026) |
| Drift limit that counts as constraint drift | not set | engine ADR, PHY spec |
| Controller mode for a design without firmware | the case omits `controller_mode`; it is required when the design has firmware (TST-036) | set in phase 1 C4 |
| Where the baseline path lives (TST-008 names a suite file; no suite file schema yet) | each case may name its suite's baseline file, relative to the case folder | phase 1 suite file schema |
| Fault parameter forms | sensor noise as an absolute standard deviation in the signal's unit; motor stall as the available-torque fraction in [0, 1); slip as a fraction in (0, 1] | owner ruling 2026-11-04 |
| Position criterion per axis or planar distance (G3) | per axis, x and y each within 1 cm (owner ruling 2026-11-04); a per-axis pass admits a 1.41 cm planar error | owner |
| Initial joint state for multi-DOF joints | single-axis (prismatic, revolute) starting positions only | phase 1 contracts |
| `initial_state.controller` | a precondition the runner checks before t = 0 against the firmware's reported state, not a state it sets; a mismatch is an error | owner ruling 2026-11-04 |
| CLI exit status for failed, errored and passed suites | not set | phase 1 contracts |
| Run archive output location | not set | phase 1 contracts |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Evaluating pass criteria and the regression diff | SCO |
| Storing the recording and exports | REC |
| Applying a fault inside its owning model | SIG, PHY, CTL |
| Tick order and rates | KER |
| Building the design | BLD |
| Monte Carlo over tolerance spreads (variations use nominal values in v1) | Roadmap, Architecture doc Seams table |
