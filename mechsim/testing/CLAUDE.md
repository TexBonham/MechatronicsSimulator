# testing/: contract

Test case files (TST) and regression baselines (SCO, D4). Schemas `schemas/test_case.json` from `Case` and
`schemas/baseline.json` from `Baseline`; regenerate with `uv run python -m mechsim.testing.schema --write-schema` and
`uv run python -m mechsim.testing.baseline --write-schema`, never hand-edit. `__init__.py` exports nothing.
Examples: `test_cases/gantry/G3.yaml` (draft design), `test_cases/pendulum/P1_period.yaml` (release),
`test_cases/gantry/baseline.yaml`.

| File | Holds |
|---|---|
| `schema.py` | `Case` and parts, `Quantity` (any dimension), `cross_check(case, design)`, `validate_test_case_file`, `load_test_case`, CLI |
| `baseline.py` | `Baseline`, `entry_key`, `baseline_text`, `diff_baselines`, `check_against_cases`, `load_baseline`, CLI |

## Case (`extra="forbid"`, strict)

| Section | Key points |
|---|---|
| header | `id` `^[A-Z][A-Z0-9_]*[0-9]+$` (file `<id>.yaml` or `<id>_*.yaml`), design name + semver, `controller_mode` iff design has firmware, seed 0..2^64-1, optional `rates` |
| `initial_state` | single-DOF joint position/velocity, controller assumptions, payload |
| `commands` | non-decreasing `at`, operator-sourced messages, payload = message fields |
| `faults` | `kind` discriminated: sensor_stuck/noisy/dropped/forced, slip, motor_stall, power_loss (`rail`), controller_freeze, estop_press: window `end` xor `duration`; payload_change instantaneous |
| `end` | one of at/message/controller_state, required `max_duration` |
| `criteria` | `check` discriminated: value, interval, event, no_motion, no_overlap, settle_time; below/at_most/above/at_least/within; refs `signal:`, `pose:`, `joint:`, `state:`, `count:`; `limit_text()` feeds baselines |
| others | sweeps, variations (stored only), `baseline` path relative to the case |

## Rules

- `${name}` refers to a design or case open parameter; a case for a release design has none.
- Dimensions: model where the reference fixes them, else `cross_check`.
- Baseline entries sorted by case (G3 before G10) then criterion; diff: newly passing, newly failing (was pass), unchanged, missing, added.

## Section index

`schema.py`: Quantities of any dimension 92, Header 183, Initial state 207, Commands and faults 281, End condition 418,
Pass criteria 450, Sweeps and variations 659, Test case 705, Cross-check against the design 764, Files and schema 1018.
`baseline.py`: Diff 114, Files and schema 166.

`tests/unit/test_test_case_schema.py`: Shipped examples 113, Header 175, Open parameters 325, Initial state 386,
Commands 521, Faults 592, End condition 756, Pass criteria 806, Sweeps, variations, baseline 1066, Files 1159.
