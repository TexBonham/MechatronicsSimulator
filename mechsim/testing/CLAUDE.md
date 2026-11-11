# testing/: contract

Test case files (TST) and regression baselines (SCO, D4). Schemas `schemas/test_case.json` from `Case` and
`schemas/baseline.json` from `Baseline`, `schemas/suite.json` from `SuiteFile`; regenerate with
`uv run python -m mechsim.testing.schema --write-schema --write-suite-schema` and
`uv run python -m mechsim.testing.baseline --write-schema`, never hand-edit. `__init__.py` exports nothing.
Examples: `test_cases/gantry/G1.yaml` to `G4.yaml` (draft design), `test_cases/pendulum/P1_period.yaml` (release),
`test_cases/gantry/baseline.yaml`, `test_cases/gantry/suite.yaml` (suite file).

| File | Holds |
|---|---|
| `schema.py` | `Case` and parts, `Quantity` (any dimension), `cross_check(case, design)`, `validate_test_case_file`, `load_test_case`, `SuiteFile`, `load_suite_file`/`validate_suite_file` (`SuiteError`), `INSTANCE_PREFIX`, CLI |
| `baseline.py` | `Baseline`, `entry_key`, `baseline_text`, `diff_baselines`, `check_against_cases`, `load_baseline`, CLI |
| `driver.py` | Protocols the runner drives: `SimulationDriver`, `BuildResult`/`Builder`, `RecordView`, `Archiver`, `ArchiveReader`; value types `RunConfig`, `ScheduledCommand`, `ScheduledFault`, `EndSpec`, `RunReport`, `Series`, `RecordedEvent`, `RunMetadata` |
| `core_driver.py` | `CoreDriver` (physics-only or signal layer + native controller), `RecordArchiver` (extra: kinematics, labels, actuators, sink_errors, hook_errors), `core_services`; re-exports `DriverError`, `Kinematics`, `JointKinematics`, `RecordingView` (`actuator:<name>.position` = its shaft joint; operator events -> `operator_message` by the label table) and `RecordArchiveReader` from `record/reader.py` (engine-free, ruling 17) |
| `core_wiring.py` | `signal_layer` (slots from bindings, driver-to-motor connections from coil wires, rails, actuator DOFs, sensors on joints, faults injected), `controller_party` (native runtime, app target `mechsim_fw_<design name>`, host library built once per process under `$TMP/mechsim-native-build/<target>-<hz>-<tree hash>`, `host_build_dir`), `RunnerParty` (commands and E-stop presses at its first step at or after their tick, KER-023), `joint_places`, `component_parameters` (steppers also get `shaft_body` and `shaft_axial_inertia` = a^T I a, SIG-037), `axial_inertia` |
| `runner.py` | `prepare`, `run_case`, `sweep_runs`, `run_suite`, `suite_cases`, `case_files`, `split_instance_values`, `apply_instance_overrides`, timing (`next_tick`, `exact_tick`, `rational_lcm`), `command_timeline`, `fault_timeline` (`FAULT_OWNERS`), `end_spec`, `check_precondition`, `module_seeds`, `Services`, `RunnerError` |

## Runner (`runner.py`, `driver.py`)

| Step | Contract |
|---|---|
| prepare | validate case (TST-009), design version before build (TST-011), `cross_check`, resolve `${}` by `resolve_open_values`: `--set`, then the case's `parameters`, then the parameter's `default` (TST-039, DES-044; dimension-checked), substitute in case and design, any `${}` left is an error; `instance:<name>.<parameter>` / `instance:<name>.part` entries of `--set` (wins) and `parameters` go to `apply_instance_overrides` (checked by `Registry.parameter_override` / `substitute_part`, `Services.registry` or the shipped one; design validated with that registry) and into `Prepared.bindings` -> `RunMetadata.bindings` (LIB-016, LIB-017) |
| driver order | `load(build, case)`, `configure(RunConfig)`, `set_initial_state`, `reported_controller_state` (firmware designs; mismatch is an error), `schedule_command`/`schedule_fault`, `run(EndSpec) -> RunReport`, `recording()` |
| timing | times to 12 significant digits, then the next base tick; base = lcm(physics, control); fault active start <= n < end; `end.at`, `max_duration` on a tick |
| seeds | `SeedSequence(seed).spawn` over `BuildResult.module_instances` in registration order (D1) |
| faults | sensor_*, motor_stall, power_loss (target = rail), estop_press -> signals; slip, payload_change -> physics; controller_freeze -> controller |
| results | every run scored (`CaseScore`); pre-t0 errors `ran=False`; a `RunnerError`/`DriverError` from `driver.run` -> `run_failed` (TST-023; `CoreDriver.run` types every binding, translation and wiring error as `DriverError`, cause kept); `RunReport.approximations` = the bridge report (SCO-013); `RunMetadata.control_rate` = the run config's, archived (base rate without a controller); archive `<out>/<run_id>.h5` when an `Archiver` and `out_dir` are given |
| suites | `run_suite(path)`: a `suite.yaml` (or a folder holding one) gives name, cases in its order, design (each case must match) and baseline (a case naming another is an error, TST-008); a folder without one: `case_files` (YAML with `criteria`, natural order), the baseline the cases name, else the folder's `baseline.yaml`. Nominal run plus each sweep value (`instance:` sweeps bind as overrides); `SuiteRun.baseline` = next baseline, `.diff` = `regression_diff` (TST-027); a baseline that fails to load or names another suite is a `SuiteRun.errors` line |

`RecordView.series(ref)` takes case refs plus `actuator:<name>.position`; operator messages are events of kind
`operator_message` with the message name as detail. Tests use `tests/unit/test_runner_fakes.py` (`InMemoryFakeDriver`).

## Case (`extra="forbid"`, strict)

| Section | Key points |
|---|---|
| header | `id` `^[A-Z][A-Z0-9_]*[0-9]+$` (file `<id>.yaml` or `<id>_*.yaml`), design name + semver, `controller_mode` iff design has firmware, seed 0..2^64-1, optional `rates`, `parameters` (name -> value for open parameters, TST-039) |
| `initial_state` | single-DOF joint position/velocity, controller assumptions, payload |
| `commands` | non-decreasing `at`, operator-sourced messages, payload = message fields |
| `faults` | `kind` discriminated: sensor_stuck/noisy/dropped/forced, slip, motor_stall, power_loss (`rail`), controller_freeze, estop_press: window `end` xor `duration`; payload_change instantaneous |
| `end` | one of at/message/controller_state, required `max_duration` |
| `criteria` | `check` discriminated: value, interval, event, no_motion, hold_band (joints within band of their first sample, SCO-026), no_overlap (`actuator`, `start_event`, `end_event` both `event:` refs and different, `closed` default false), settle_time; below/at_most/above/at_least/within; quantity refs `signal:`, `pose:`, `joint:`, `state:`, `count:`; event refs `message:`, `state:`, `event:` (needs firmware); `limit_text()` feeds baselines |
| others | sweeps, variations (stored only; parameter `${name}`, `<section>.<item>.<field>` or `instance:<name>.<field>`), `baseline` path relative to the case |

## Criteria on recorded events (SCO-021, SCO-025)

| Check | Contract |
|---|---|
| `event:<name>` ref | a recorded event of kind `<name>` (`RecordView.events()`); a firmware log event is recorded with kind = its name from the design's event table and detail = its argument (for `motion_start`/`motion_end` the actuator instance name; decoding is the integration wave's) |
| `interval` | first start occurrence to the first end occurrence at or after it; start never recorded, end never recorded, or ends only before the first start: each an unevaluable error naming the ref |
| `no_overlap` | start/end events with detail == `actuator`, ordered by time then record order; a start while one is open is a violation; `closed: true` adds one per start still open at the end (worst time = last sample); measured = violations; no start on the actuator or an end with no open start is an unevaluable error naming the ref |

## Rules

- `${name}` refers to a design or case open parameter; a case for a release design has none.
- Dimensions: model where the reference fixes them, else `cross_check`.
- Baseline optional `note` (gantry: placeholder-geometry); entries sorted by case (G3 before G10) then criterion; diff: newly passing, newly failing (was pass), unchanged, missing, added.

## Section index

`schema.py`: Quantities of any dimension 96, Header 187, Initial state 211, Commands and faults 285, End condition 422,
Pass criteria 454, Sweeps and variations 713, Test case 759, Cross-check against the design 825, Suite file 1118,
Files and schema 1187.
`baseline.py`: Diff 130, Files and schema 182.
`runner.py`: Timing 98, Loading and resolution 126, Timelines 358, Runs 485, Suites 638.

`tests/unit/test_test_case_schema.py`: Shipped examples 120, Header 182, Open parameters 332, Initial state 393,
Commands 528, Faults 599, End condition 763, Pass criteria 813, Sweeps, variations, baseline 1108, Suite file 1230,
Files 1300.
`tests/unit/test_runner.py`: Loading 90, Initial state and precondition 154, Command timeline 197, Fault timeline 252,
End conditions 368, Configuration 444, Suites and sweeps 520, Suite files 624, Fault routing through the driver 733,
Overrides and part substitution 777.
