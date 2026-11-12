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
| TST-023 | The test runner shall hand every run to scoring with the reason it ended, including runs ended by a solver error or a fault, and a run the driver cannot carry out (a typed driver error from the run call, cause kept) as a run_error score with its message. | Architecture doc, Modules: core, Test runner row; Data flows, run step 5; Architecture doc, Repository starting point, invariant 7; brief M1-X7 coordinator ruling 1 (2026-11-04) | M1 | unit | n/a |
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
| TST-038 | The record view shall read `joint:<name>.position` and `.velocity` of a revolute or prismatic joint from the recorded body poses as the coordinate that is zero at the joint's neutral placement, and `pose:<body>.<x,y,z,roll,pitch,yaw>` as the body frame's displacement and rotation (Z-Y-X angles, world axes) from its pose at the reference configuration where every joint coordinate is zero; a case's start joint position shall be placed exactly in the engine model, not approximated. | Owner ruling 2026-11-04 (G3 targets in the homed origin frame); ADR 0003 (no silent approximation) | M1 | unit | n/a |
| TST-039 | The test runner shall resolve every open parameter of the design and the case in the order `--set`, the case's `parameters`, the parameter's default (DES-044), check each given value against the parameter's dimension, accept choice values from `--set` and `parameters`, and reject a value for an undeclared parameter. | M1-X3 coordinator ruling 2 (2026-11-04), for owner confirmation | M1 | unit | n/a |
| TST-040 | The test runner shall select the physics adapter by the run-configuration field `physics.adapter` (`mujoco`, the default until X2, or `native`), set in the test case and overridden by `--set physics.adapter=<name>`, reject any other value as a validation error naming the field, and record the adapter in the run identity and the archive. Identity rule: a run on the default adapter keeps the run id `<case>-seed<seed>` (sweep and variation forms as TST-025 and TST-026); a run on any other adapter appends `-<adapter>` to it; the archive's root attribute `physics_adapter` names the adapter on every run (an archive without it was a bridge run). | Round 5 ruling 41 (N review) | M3 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| End condition form (time limit, event, both) | exactly one trigger (a time, an observed operator message, or a controller state) plus a required `max_duration` (`schemas/test_case.json`) | set in phase 1 C4 |
| Delivery rule for a command or fault scheduled between ticks | a scheduled time is taken to 12 significant digits (removing unit-conversion float error from the decimal the file states); a command or fault edge between base ticks is delivered at the next base tick, the first n with n / base_rate >= t; a fault is active for start_tick <= n < end_tick and a window covering no tick is an error; `end.at` and `max_duration` must lie on a base tick (KER); base rate = lcm(physics rate, control rate), physics step 1 ms unless the case sets `rates`, control rate the case's else the firmware `tick_rate` | ruled form, `mechsim/testing/runner.py` (M1-I) |
| Sweep parameter addressing syntax | `${name}` for a design open parameter, `instance:<instance>.<parameter>` or `instance:<instance>.part` for a catalog parameter or part of one component instance (LIB-016, LIB-017, checked against the catalog like `--set`), else `<bodies\|joints\|components>.<item name>.<field>[.<field>]` into the design file | set in phase 1 C4; `instance:` M2-D |
| Seed of each seeded variation (derived from the case seed or listed) | derived: variation k's seed is the first 64-bit state word of `SeedSequence(case seed, spawn_key=(k,))`; it is the run's seed (module seeds, D1) and draws each varied parameter in file order from `default_rng(seed)`: nominal + spread x N(0, 1) (normal) or x U(-1, 1) (uniform), written in SI base units to 17 significant digits. The nominal is the resolved open parameter, the `instance:` value the case or `--set` gives, or the design field's value. Run id `<case>-variation-seed<seed>`; `run_variation(case, services, seed)` reruns one alone with an identical result | runner (TST-026), brief R |
| Drift limit that counts as constraint drift | not set | engine ADR, PHY spec |
| Controller mode for a design without firmware | the case omits `controller_mode`; it is required when the design has firmware (TST-036) | set in phase 1 C4 |
| Where the baseline path lives (TST-008) | the suite file `suite.yaml` (`schemas/suite.json`: `name`, `design` name and version, `cases` relative files, at least one, no duplicates, `baseline` required, optional `note`); every case must be written against the suite's design, and a case naming another baseline is an error. A suite folder without `suite.yaml` keeps the milestone 1 form: its case files, the baseline the cases name, else the folder's `baseline.yaml` | M2-D |
| Open-parameter values at run time | `run`/`suite --set name=value` binds a design or case open parameter, parsed with its declared dimension; every `${}` in the case and the design must be bound before t = 0, else the run is an error naming each path; a sweep value binds its parameter for that run only | `mechsim/testing/runner.py` (M1-I) |
| Instance overrides at run time (LIB-016, LIB-017) | `--set` or the case's `parameters` (`--set` wins) take `instance:<name>.<parameter>=<quantity>` (dimension checked against the type's parameter; a flag takes `true` or `false`) and `instance:<name>.part=<part>` (a catalog part of the instance's own type), applied to the design's component instance before the builder; an unknown instance, parameter or part, a wrong dimension, or a part of another type is an error before t = 0 naming the override; every applied override is listed in the run metadata bindings and so in the archive | M2-D |
| Sweep runs | each sweep of a case runs separately, one run per value beside the nominal run (no cross product); only nominal runs enter a baseline. A suite file's `sweeps` run for every case after the case's own; its `variations` apply to each case that has none (TST-007). A design field path through a per-DOF list (a joint's `angular` entries) sets every entry | `mechsim/testing/runner.py` (M1-I; suite file and lists brief R) |
| Seed per module instance (TST-028) | `SeedSequence(seed).spawn(n)` over the builder's module instances in kernel registration order; each instance gets the child's first 64-bit state word | `mechsim/testing/runner.py` (M1-I) |
| Base rate the runner schedules against | the runner computes lcm(physics rate, control rate); the kernel's base rate is the lcm over every party rate, so in milestone 1 the real driver registers no party at another rate, or converts the runner's ticks to the kernel's base tick | M1-I review; real driver at integration |
| CLI through the facade (TST-034) | stays a stub until the facade service lands; for now `mechsim/cli.py` calls the runner directly | FAC |
| Fault parameter forms | sensor noise as an absolute standard deviation in the signal's unit; motor stall as the available-torque fraction in [0, 1); slip as a fraction in (0, 1] | owner ruling 2026-11-04 |
| Position criterion per axis or planar distance (G3) | per axis, x and y each within 1 cm (owner ruling 2026-11-04); a per-axis pass admits a 1.41 cm planar error | owner |
| Initial joint state for multi-DOF joints | single-axis (prismatic, revolute) starting positions only | phase 1 contracts |
| `initial_state.controller` | a precondition the runner checks before t = 0 against the firmware's reported state, not a state it sets; a mismatch is an error. With DES-045 code tables the driver reports `{state: name, fault: name}` from a private runtime reset at the controller rail and ticked once with idle inputs, and the case names `state` and `fault` only | owner ruling 2026-11-04 |
| CLI exit status for failed, errored and passed suites | 0 passed (every file valid for `validate`); 1 failed (a criterion failed, a file invalid); 2 error (a run error, a case that could not run, an unknown file kind, a missing path, a usage error, no driver or archive reader wired) | `mechsim/cli.py` (M1-I) |
| Run archive output location | CLI `--out`, default `./runs`; one archive `<case id>-seed<seed>[-sweep<i>-<j>-<value>].h5` per run (`<case id>-variation-seed<seed>.h5` for a variation, characters outside `A-Za-z0-9_.-` as `_`); the suite report `<suite>_report.txt` beside them | `mechsim/cli.py` (M1-I); owner may move it |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Evaluating pass criteria and the regression diff | SCO |
| Storing the recording and exports | REC |
| Applying a fault inside its owning model | SIG, PHY, CTL |
| Tick order and rates | KER |
| Building the design | BLD |
| Monte Carlo over tolerance spreads (variations use nominal values in v1) | Roadmap, Architecture doc Seams table |
