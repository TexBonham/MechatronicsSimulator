# testing/: contract

Test case files (TST) and regression baselines (SCO, D4). Schemas `schemas/test_case.json` from `Case` and
`schemas/baseline.json` from `Baseline`, `schemas/suite.json` from `SuiteFile`; regenerate with
`uv run python -m mechsim.testing.schema --write-schema --write-suite-schema` and
`uv run python -m mechsim.testing.baseline --write-schema`, never hand-edit. `__init__.py` exports nothing.
Examples: `test_cases/gantry/G1.yaml` to `G15.yaml` (draft design; G12 and G15 on native with substeps 10), `test_cases/pendulum/P1_period.yaml` (release), `P2_damping_sweep.yaml` (sweep example),
`test_cases/gantry/baseline.yaml`, `test_cases/gantry/suite.yaml` (suite file).

| File | Holds |
|---|---|
| `schema.py` | `Case` and parts, `Quantity` (any dimension), `cross_check(case, design)`, `validate_test_case_file`, `load_test_case`, `SuiteFile`, `load_suite_file`/`validate_suite_file` (`SuiteError`), `INSTANCE_PREFIX`, CLI |
| `baseline.py` | `Baseline`, `entry_key`, `baseline_text`, `diff_baselines`, `check_against_cases`, `load_baseline`, CLI |
| `driver.py` | Protocols the runner drives: `SimulationDriver`, `BuildResult`/`Builder`, `RecordView`, `Archiver`, `ArchiveReader`, `IntegrityCheck` (`(build, case, recording) -> IntegrityResults`, brief I1); value types `RunConfig` (+ `physics_substeps`, TST-042), `ScheduledCommand`, `ScheduledFault`, `EndSpec`, `RunReport` (+ `notes`: report lines that are not approximations; `EndReason` + `integrity_halt`), `Series`, `RecordedEvent`, `RunMetadata` (+ `members`, `material_overrides`, `variation`, `integrity`); structural: `INTEGRITY_KIND`/`INTEGRITY_SOURCE` = `integrity`, `IntegrityCrossing(member, quantity, value, limit, tier=1)` (`detail()`, `parse`; brief I2 emits it), `MemberIntegrity(member, peak_stress, peak_stress_time, deflection, deflection_time, safety_factor, safety_factor_time, flags)`, `RopeIntegrity(rope, peak_tension, peak_tension_time, flags)`, `IntegrityResults(members, ropes=())` (`member(name)`, JSON `{members, ropes}`), `MemberSection(name, material, area, second_moment_z, length, second_moment_y, fibre_y, fibre_z)` (y-axis fields None for an area-and-moment section), `MaterialOverridesRecord(text, sha256)`, `MATERIAL_OVERRIDES_KEY` |
| `core_driver.py` | `CoreDriver(subscribers=(), designs_root=DESIGNS_ROOT, tier1=None)` (`integrity.yaml` from `<designs_root>/<design>/` through `Tier1Service(designs_root)`, or `tier1` in its place; rope snap gated by its `rope_snap` entry, else breaking tensions dropped with a `ROPE_SNAP_OFF` line, INT-031; tier 1 party from `tier1(build, model, native document)`, the limits table built on the native adapter only, registered after physics, its trace on `integrity_trace`, yield wiring `integrity.model_requests`, halt from the case's `integrity.halt_on_crossing` only (absent or empty: false) through the `party_state` trigger: end reason `integrity_halt` and the halt line in `RunReport.notes`, crossing details canonicalised by `IntegrityCrossing`; on the bridge the `NOT_EVALUATED` line and no party, ruling 34; physics-only or signal layer + native controller; payload body from the case (none when the case attaches none: no stand-in, the load cell has no target and reads 0 N, BLD-036), a fixed child of its carrier when attached at t = 0 and never released (ruling 24), else welded; a payload a design rope ends at (`BuildResult.payload_ropes`) hangs on the rope on native (`with_payload(..., rope, starts)`, its load cell reading the shackle's `<payload>_fixed` reaction; a mid-run release is a `DriverError`), and on the bridge stays fixed with the `payload_stand_in_line` approximation; `physics_substeps` above 1 on the bridge is a `DriverError`, on native it goes to `prepare_model(..., substeps)` (TST-042), slip and payload changes on `PhysicsParty`, freeze on `ControllerParty`, `model_columns` labels, body series named neutral bodies then (native) `member_node_names` (PHY-003), at the end of a native run one `driven_mode_lines` line per driven joint from the adapter's `scheme_decay_record()` (taken before `PhysicsParty` takes the adapter, read after the run) (rulings 44, 45), on the bridge `BRIDGE_NODE_POSES` when the build lists members, firmware `event_codes` events from the runner party, argument decoded by kind (ruling 28), `controller_fault` events on each change of the reported fault code from none (`FAULT_KIND`, ruling 30; none at the first sample is no event), firmware events attached to the first row at or after their tick), `member_frames(model)`, `member_frame_loads(record, frames)` (bridge world-axis member wrenches into the member frame, ruling 24), `RecordArchiver` (extra: kinematics, labels, actuators, sink_errors, hook_errors, `notes` when any), `core_services(root)` (driver `partial(CoreDriver, designs_root=root)`); re-exports `DriverError`, `Kinematics`, `JointKinematics`, `RecordingView` (`actuator:<name>.position` = its shaft joint; operator events -> `operator_message` by the label table) and `RecordArchiveReader` from `record/reader.py` (engine-free, ruling 17) |
| `core_wiring.py` | `signal_layer` (M2: each instance built as `core_type(instance)` = its catalog `plugin.cpp`; an encoder whose `resolution` is a length gets `linear: true` (ruling 32); a load cell target that does not resolve is a `WiringError` unless the run has no payload (then 0 N); a brake off a revolute shaft joint is a `WiringError` (ruling 29); signal wires as connections, brake at the actuator stage with `brake_parameters`, load cell payload index, amplifier ADC from `design_chip`, power_loss on the rail's supply, `undrivable_brake` is a `WiringError` naming brake and joint (ruling 29); slots from bindings, driver-to-motor connections from coil wires, rails, actuator DOFs, sensors on joints, faults injected), `controller_party` (native runtime, app target `mechsim_fw_<design name>`, host library built once per process under `$TMP/mechsim-native-build/<target>-<hz>-<tree hash>`, `host_build_dir`), `RunnerParty` (commands and E-stop presses at its first step at or after their tick, KER-023), `joint_places`, `component_parameters` (steppers also get `shaft_body` and `shaft_axial_inertia` = a^T I a, SIG-037), `axial_inertia` |
| `runner.py` | `prepare`, `run_case` (`variant: Variant(sweep, sweep_index, seed, variation)`), `sweep_runs`, `variation_runs`, `run_variation` (one alone from its seed), `variation_seed`, `draw_variation`, `nominal_values`, `member_sections`, `material_overrides`, `get_design_path`/`set_design_path` (per-DOF lists: every entry), `run_suite`, `suite_cases`, `case_files`, `split_instance_values`, `apply_instance_overrides`, timing (`next_tick`, `exact_tick`, `rational_lcm`), `command_timeline`, `fault_timeline` (`FAULT_OWNERS`), `end_spec`, `check_precondition`, `module_seeds`, `Services`, `RunnerError` |

## Runner (`runner.py`, `driver.py`)

| Step | Contract |
|---|---|
| prepare | validate case (TST-009), design version before build (TST-011), `cross_check`, resolve `${}` by `resolve_open_values`: `--set`, then the case's `parameters`, then the parameter's `default` (TST-039, DES-044; dimension-checked), substitute in case and design, any `${}` left is an error; `instance:<name>.<parameter>` / `instance:<name>.part` entries of `--set` (wins) and `parameters` go to `apply_instance_overrides` (checked by `Registry.parameter_override` / `substitute_part`, `Services.registry` or the shipped one; design validated with that registry) and into `Prepared.bindings` -> `RunMetadata.bindings` (LIB-016, LIB-017) |
| substeps | `physics.substeps` (TST-042, rulings 21, 49): integer >= 1, default 1, engine steps per physics step on native (h / substeps), rates and rows unchanged; above 1 on the bridge the run fails; `RunConfig.physics_substeps` |
| adapter | `physics.adapter` (TST-040): case field `physics: {adapter}` (`mujoco` default, `native`), `--set physics.adapter=<name>` wins; `RunConfig.physics_adapter`, `RunMetadata.physics_adapter`, archive root attr; `Builder(design, adapter)`: `core_builder` rigid members on the bridge, flexible on native; run id gains `-<adapter>` off the default; no environment variable |
| driver order | `load(build, case)`, `configure(RunConfig)`, `set_initial_state`, `reported_controller_state` (firmware designs; mismatch is an error), `schedule_command`/`schedule_fault`, `run(EndSpec) -> RunReport`, `recording()` |
| timing | times to 12 significant digits, then the next base tick; base = lcm(physics, control); fault active start <= n < end; `end.at`, `max_duration` on a tick |
| seeds | `SeedSequence(seed).spawn` over `BuildResult.module_instances` in registration order (D1) |
| faults | sensor_*, motor_stall, power_loss (target = rail), estop_press -> signals; slip, payload_change -> physics; controller_freeze -> controller |
| variations | variation k: seed `variation_seed(case seed, k)` (SeedSequence child, 64 bits) is the run seed and draws each parameter from `default_rng(seed)` about its nominal; run id `<case>-variation-seed<seed>`; `RunMetadata.variation`, `CaseScore.variation`; suite file `sweeps` run after each case's, suite `variations` for a case without its own |
| results | every run scored (`CaseScore`); `Services.integrity` (none until I1) attaches `IntegrityResults` to metadata and scoring; metadata `members` from `BuildResult.members`, `material_overrides` from the design's `materials.overrides`; pre-t0 errors `ran=False`; a `RunnerError`/`DriverError` from `driver.run` -> `run_failed` (TST-023; `CoreDriver.run` types every binding, translation and wiring error as `DriverError`, cause kept); `RunReport.approximations` = the bridge report (SCO-013); `RunMetadata.control_rate` = the run config's, archived (base rate without a controller); archive `<out>/<run_id>.h5` when an `Archiver` and `out_dir` are given |
| integrity | `core_services` wires the driver with its `designs_root` (INT-023, INT-031) and `Services.integrity = IntegrityService(designs_root)` (INT-022): `integrity.yaml` beside the design file (`<designs_root>/<name>/`, ruling 27); absent, the hook returns None and the run carries no table; an `IntegrityError` fails the run (`run_failed`); report and load case written beside the archive |
| suites | `run_suite(path)`: a `suite.yaml` (or a folder holding one) gives name, cases in its order, design (each case must match) and baseline (a case naming another is an error, TST-008); a folder without one: `case_files` (YAML with `criteria`, natural order), the baseline the cases name, else the folder's `baseline.yaml`. Nominal run plus each sweep value (`instance:` sweeps bind as overrides); `SuiteRun.baseline` = next baseline, `.diff` = `regression_diff` (TST-027); a baseline that fails to load or names another suite is a `SuiteRun.errors` line |

`RecordView.series(ref)` takes case refs plus `actuator:<name>.position`; operator messages are events of kind
`operator_message` with the message name as detail. Tests use `tests/unit/test_runner_fakes.py` (`InMemoryFakeDriver`).

## Case (`extra="forbid"`, strict)

| Section | Key points |
|---|---|
| integrity | optional `integrity: {halt_on_crossing}` (bool, default false, absent or empty section alike; TST-041, INT-026, rulings 4, 50) |
| header | `id` `^[A-Z][A-Z0-9_]*[0-9]+$` (file `<id>.yaml` or `<id>_*.yaml`), design name + semver, `controller_mode` iff design has firmware, seed 0..2^64-1, optional `rates`, `parameters` (name -> value for open parameters, TST-039) |
| `initial_state` | single-DOF joint position/velocity, controller assumptions, payload |
| `commands` | non-decreasing `at`, operator-sourced messages, payload = message fields |
| `faults` | `kind` discriminated: sensor_stuck/noisy/dropped/forced, slip, motor_stall, power_loss (`rail`), controller_freeze, estop_press: window `end` xor `duration`; payload_change instantaneous |
| `end` | one of at/message/controller_state, required `max_duration` |
| `criteria` | `check` discriminated: value (statistics final, minimum, maximum, peak_absolute, period, and SCO-027 mean (needs a window start), amplitude, period_about_mean, frequency_about_mean (`1 / [time]`)), interval, event, no_motion, hold_band (joints within band of their first sample, SCO-026), no_overlap (`actuator`, `start_event`, `end_event` both `event:` refs and different, `closed` default false), settle_time; below/at_most/above/at_least/within; quantity refs `signal:`, `pose:`, `joint:`, `rail:<name>` (1 on, 0 off; `.voltage`; ruling 27, scored from the rails group), `state:`, `fault:<name>` (1 while the controller reports that fault, ruling 30), `count:` (message, state, fault); event refs `message:`, `state:`, `fault:` (each change to that fault), `event:` (needs firmware); `limit_text()` feeds baselines |
| others | sweeps, variations (count, parameters: `${name}`, `<section>.<item>.<field>` or `instance:<name>.<field>`, normal or uniform, spread; run by the runner, TST-026), `baseline` path relative to the case; quantity refs also `safety_factor:<member>`, `deflection:<member>` (a body with `flexible`) |

## Criteria on recorded events (SCO-021, SCO-025)

| Check | Contract |
|---|---|
| `event:<name>` ref | a recorded event of kind `<name>` (`RecordView.events()`); a firmware log event is recorded with kind = its name from the design's event table and detail = its argument decoded by the entry's kind: `axis` the step binding's instance, `fault`/`state` the code name (`from>to` for a packed state pair), `number`, `none` (ruling 28) |
| `interval` | first start occurrence to the first end occurrence at or after it; start never recorded, end never recorded, or ends only before the first start: each an unevaluable error naming the ref |
| `no_overlap` | start/end events with detail == `actuator`, ordered by time then record order; a start while one is open is a violation; `closed: true` adds one per start still open at the end (worst time = last sample); measured = violations; no start on the actuator or an end with no open start is an unevaluable error naming the ref |

## Rules

- `${name}` refers to a design or case open parameter; a case for a release design has none.
- Dimensions: model where the reference fixes them, else `cross_check`.
- Baseline optional `note` (gantry: placeholder-geometry); entries sorted by case (G3 before G10) then criterion; diff: newly passing, newly failing (was pass), unchanged, missing, added.

## Section index

| Section | File | Line |
|---|---|---|
| Rows | core_driver.py | 116 |
| Driver | core_driver.py | 259 |
| Member frames | core_driver.py | 791 |
| Archives | core_driver.py | 861 |
| Services (+ rope snap gate) | core_driver.py | 964 |
| Errors and runner state | core_wiring.py | 105 |
| Runner party | core_wiring.py | 159 |
| Joint places and inertia | core_wiring.py | 249 |
| Component parameters | core_wiring.py | 326 |
| Fault injection and ticks | core_wiring.py | 532 |
| Assembly | core_wiring.py | 557 |
| Controller build | core_wiring.py | 580 |
| Signal layer | core_wiring.py | 700 |
| Event and operator wiring | core_wiring.py | 808 |
| Timing | runner.py | 108 |
| Loading and resolution | runner.py | 136 |
| Timelines | runner.py | 391 |
| Runs | runner.py | 518 |
| Variations | runner.py | 773 |
| Suites | runner.py | 903 |
| Quantities of any dimension | schema.py | 97 |
| Header | schema.py | 194 |
| Initial state | schema.py | 218 |
| Commands and faults | schema.py | 292 |
| End condition | schema.py | 429 |
| Pass criteria | schema.py | 461 |
| Sweeps and variations | schema.py | 726 |
| Test case | schema.py | 773 |
| Cross-check against the design | schema.py | 839 |
| Suite file | schema.py | 1148 |
| Files and schema | schema.py | 1224 |
| Diff | baseline.py | 130 |
| Files and schema | baseline.py | 182 |
