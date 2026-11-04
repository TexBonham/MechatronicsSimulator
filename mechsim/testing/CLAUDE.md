# testing/: contract

Test case files (TST) and regression baselines (SCO, D4). Schemas `schemas/test_case.json` from `Case` and
`schemas/baseline.json` from `Baseline`; regenerate with `uv run python -m mechsim.testing.schema --write-schema` and
`uv run python -m mechsim.testing.baseline --write-schema`, never hand-edit. `__init__.py` exports nothing.
Examples: `test_cases/gantry/G1.yaml` to `G4.yaml` (draft design), `test_cases/pendulum/P1_period.yaml` (release),
`test_cases/gantry/baseline.yaml`.

| File | Holds |
|---|---|
| `schema.py` | `Case` and parts, `Quantity` (any dimension), `cross_check(case, design)`, `validate_test_case_file`, `load_test_case`, CLI |
| `baseline.py` | `Baseline`, `entry_key`, `baseline_text`, `diff_baselines`, `check_against_cases`, `load_baseline`, CLI |
| `driver.py` | Protocols the runner drives: `SimulationDriver`, `BuildResult`/`Builder`, `RecordView`, `Archiver`, `ArchiveReader`; value types `RunConfig`, `ScheduledCommand`, `ScheduledFault`, `EndSpec`, `RunReport`, `Series`, `RecordedEvent`, `RunMetadata` |
| `core_driver.py` | `CoreDriver` (physics-only or signal layer + native controller), `RecordingView` (`actuator:<name>.position` = its shaft joint; operator events -> `operator_message` by the design's label table), `RecordArchiver`/`RecordArchiveReader` (extra: kinematics, labels, actuators, sink_errors, hook_errors), `core_services` |
| `core_wiring.py` | `signal_layer` (slots from bindings, driver-to-motor connections from coil wires, rails, actuator DOFs, sensors on joints, faults injected), `controller_party` (native runtime, app target `mechsim_fw_<design name>`, host library built once per process under `$TMP/mechsim-native-build/<target>-<hz>-<tree hash>`, `host_build_dir`), `RunnerParty` (commands and E-stop presses at its first step at or after their tick, KER-023), `joint_places`, `component_parameters` (steppers also get `shaft_body` and `shaft_axial_inertia` = a^T I a, SIG-037), `axial_inertia` |
| `runner.py` | `prepare`, `run_case`, `sweep_runs`, `run_suite`, `case_files`, timing (`next_tick`, `exact_tick`, `rational_lcm`), `command_timeline`, `fault_timeline` (`FAULT_OWNERS`), `end_spec`, `check_precondition`, `module_seeds`, `Services`, `RunnerError` |

## Runner (`runner.py`, `driver.py`)

| Step | Contract |
|---|---|
| prepare | validate case (TST-009), design version before build (TST-011), `cross_check`, resolve `${}` by `resolve_open_values`: `--set`, then the case's `parameters`, then the parameter's `default` (TST-039, DES-044; dimension-checked), substitute in case and design, any `${}` left is an error |
| driver order | `load(build, case)`, `configure(RunConfig)`, `set_initial_state`, `reported_controller_state` (firmware designs; mismatch is an error), `schedule_command`/`schedule_fault`, `run(EndSpec) -> RunReport`, `recording()` |
| timing | times to 12 significant digits, then the next base tick; base = lcm(physics, control); fault active start <= n < end; `end.at`, `max_duration` on a tick |
| seeds | `SeedSequence(seed).spawn` over `BuildResult.module_instances` in registration order (D1) |
| faults | sensor_*, motor_stall, power_loss (target = rail), estop_press -> signals; slip, payload_change -> physics; controller_freeze -> controller |
| results | every run scored (`CaseScore`); pre-t0 errors `ran=False`; a `RunnerError`/`DriverError` from `driver.run` -> `run_failed` (TST-023; `CoreDriver.run` types every binding, translation and wiring error as `DriverError`, cause kept); `RunReport.approximations` = the bridge report (SCO-013); `RunMetadata.control_rate` = the run config's, archived (base rate without a controller); archive `<out>/<run_id>.h5` when an `Archiver` and `out_dir` are given |
| suites | `case_files` (YAML with `criteria`, natural order), nominal run plus each sweep value; one baseline named by the cases, diffed |

`RecordView.series(ref)` takes case refs plus `actuator:<name>.position`; operator messages are events of kind
`operator_message` with the message name as detail. The real driver, builder, archiver and reader are wired at
integration; tests use `tests/unit/test_runner_fakes.py` (`InMemoryFakeDriver`).

## Case (`extra="forbid"`, strict)

| Section | Key points |
|---|---|
| header | `id` `^[A-Z][A-Z0-9_]*[0-9]+$` (file `<id>.yaml` or `<id>_*.yaml`), design name + semver, `controller_mode` iff design has firmware, seed 0..2^64-1, optional `rates`, `parameters` (name -> value for open parameters, TST-039) |
| `initial_state` | single-DOF joint position/velocity, controller assumptions, payload |
| `commands` | non-decreasing `at`, operator-sourced messages, payload = message fields |
| `faults` | `kind` discriminated: sensor_stuck/noisy/dropped/forced, slip, motor_stall, power_loss (`rail`), controller_freeze, estop_press: window `end` xor `duration`; payload_change instantaneous |
| `end` | one of at/message/controller_state, required `max_duration` |
| `criteria` | `check` discriminated: value, interval, event, no_motion, hold_band (joints within band of their first sample, SCO-026), no_overlap, settle_time; below/at_most/above/at_least/within; refs `signal:`, `pose:`, `joint:`, `state:`, `count:`; `limit_text()` feeds baselines |
| others | sweeps, variations (stored only), `baseline` path relative to the case |

## Rules

- `${name}` refers to a design or case open parameter; a case for a release design has none.
- Dimensions: model where the reference fixes them, else `cross_check`.
- Baseline optional `note` (gantry: placeholder-geometry); entries sorted by case (G3 before G10) then criterion; diff: newly passing, newly failing (was pass), unchanged, missing, added.

## Section index

`schema.py`: Quantities of any dimension 92, Header 183, Initial state 207, Commands and faults 281, End condition 418,
Pass criteria 450, Sweeps and variations 659, Test case 705, Cross-check against the design 764, Files and schema 1018.
`baseline.py`: Diff 114, Files and schema 166.
`runner.py`: Timing 82, Loading and resolution 110, Timelines 251, Runs 378, Suites 522.

`tests/unit/test_test_case_schema.py`: Shipped examples 113, Header 175, Open parameters 325, Initial state 386,
Commands 521, Faults 592, End condition 756, Pass criteria 806, Sweeps, variations, baseline 1066, Files 1159.
`tests/unit/test_runner.py`: Loading 82, Initial state and precondition 146, Command timeline 189, Fault timeline 244,
End conditions 360, Configuration 436, Suites and sweeps 512.
