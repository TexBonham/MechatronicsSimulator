# M2-G: chip data and the milestone 2 catalog types

Agent: schema-author (Opus). Worktree `../MechatronicsSimulator-04-g` on branch `wip/04-g`, from
`round/04-milestone-2`. Read first: root `CLAUDE.md`, `mechsim/CLAUDE.md`, `mechsim/design/CLAUDE.md` (Chip section),
`mechsim/library/CLAUDE.md`, `schemas/README.md`, `chips/README.md`, spec rows DES-029 in `docs/spec/des.md` and
LIB-019, LIB-025, LIB-027, LIB-021 in `docs/spec/lib.md`. References by section: `mechsim/library/schema.py`
(Parameters 88, Facets 166, ComponentType 239), the catalog files `limit_switch.yaml` (sensor shape),
`watchdog.yaml` and `estop_relay.yaml` (safety shape), `step_dir_driver.yaml` (driver shape).

Call cap 120. Filtered tests while iterating:
`uv run pytest -q tests/unit/test_chip_schema.py tests/unit/test_library_catalog.py tests/unit/test_catalog_registry.py`.
Exactly one full run at the end: `tools/locked_build.sh sync` first (it runs `uv sync` under the machine build lock;
wait for it), then `uv run pytest -q`, `uv run ruff check .`, `uv run ruff format --check .`, `uv run mypy`,
`python3 tools/check_imports.py`. Run `uv run python tools/traceability.py --write` before the full run so the matrix
is current. Commit once as WIP at the end ("WIP M2-G: chip data and catalog types"); do not push.

## Ownership

`mechsim/design/chip.py`, `mechsim/library/schema.py`, `mechsim/library/catalog/load_cell.yaml`,
`load_cell_amplifier.yaml`, `imu.yaml`, `brake.yaml` (new), `chips/stm32_placeholder/chip.yaml`, `chips/README.md`,
`schemas/chip.json`, `schemas/component.json`, `schemas/examples/` entries for the new kinds,
`tests/unit/test_chip_schema.py`, `tests/unit/test_library_catalog.py`, `tests/unit/test_catalog_registry.py`,
`docs/spec/des.md` and `docs/spec/lib.md` (row edits only), `docs/spec/traceability.md` (regenerated),
`mechsim/design/CLAUDE.md` Chip section, `mechsim/library/CLAUDE.md`. Nothing else; list anything more under Open
issues.

## 1. Chip definition beyond pin names (DES-029; consumers CTL-013, SIG-014)

Add to `Chip`, every value a unit string in the file and SI inside (L5):

| Field | Form | Placeholder value in `stm32_placeholder/chip.yaml` |
|---|---|---|
| `core_voltage` | voltage | `3.3 V` |
| `brownout_threshold` | voltage, strictly below `core_voltage` | `2.0 V` |
| `adc` | `{resolution_bits: int 1..32, reference_voltage: voltage > 0}` | 12 bits, `3.3 V` |
| `counter_bits` | int 1..64 (width of the timer encoder counters) | 16 |

Required fields for every chip, placeholder or verified. Validation: `brownout_threshold < core_voltage`; bounds on
each int at, inside and outside. Keep the existing pin-map rules untouched. Mark each placeholder value with a
comment "placeholder until the part is selected (gantry team)". Add a row to DES-029's spec text only if the row's
wording does not already cover the new fields; otherwise add the fields to the DES open-parameters table with the
owed-by note. `chips/README.md` lists the new fields.

## 2. Catalog types (LIB-019, LIB-025, LIB-027)

Four new catalog files, one type each, one placeholder part each (part values flagged in a `placeholders` list like
the existing files). Dimensions in pint strings; every parameter in exactly one tier; the `ideal` tier first.
`Category` and `SignalKind` literals may grow only as listed here. Add `"front_end"` to `Category` (LIB-026 keeps
front ends separate from sensors). `FailureMode` gains nothing.

### `load_cell` (category sensor, LIB-025)

Parameters: `mass`; `capacity` [force] > 0; `sensitivity` dimensionless (output volts per excitation volt at
capacity, typical 2e-3) > 0; `excitation_voltage` [V] > 0; `resolution` [force] >= 0; `noise` [force] >= 0,
default 0; `delay` [time] >= 0, default 0; `sample_rate` 1/[time] >= 1 Hz; `nonlinearity` dimensionless 0..1,
default 0 (second tier `nonlinear`). Failure modes: stuck, noisy, dropped. Pins: `excitation` power_in (power),
`signal` output analog, `ground` ground. Mechanical facet: mass, one mount `body` (bolt_pattern). Ratings: absolute
maximum force = 1.5 x capacity expressed as a parameter `overload_limit` [force] with the ratings facet naming it.
Placeholder part: capacity `5 kg` x g written as `49.05 N`, sensitivity `2e-3`, excitation `5 V`, resolution
`0.01 N`, sample rate `80 Hz`.

### `load_cell_amplifier` (category front_end)

Parameters: `mass`; `gain` dimensionless > 0; `output_full_scale` [V] > 0; `offset` [V], default 0; `bandwidth`
1/[time] > 0; `supply_voltage` [V] > 0; `supply_current` [A] >= 0. Pins: `signal_in` input analog, `out` output
analog (binds to an `adc_input` pin), `vcc` power_in, `ground` ground. Placeholder part: gain 128, full scale
`3.3 V`, bandwidth `10 Hz`, supply `5 V`, `1.5 mA`.

### `imu` (category sensor)

Open ruling 2 in `docs/queue.md`: the chip vocabulary has no serial peripheral mode, so the IMU exposes analog
outputs until one exists; write that sentence in the file header. Parameters: `mass`; `accel_range`
[length]/[time]^2 > 0; `gyro_range` 1/[time] > 0; `resolution` [length]/[time]^2 >= 0 (accelerometer LSB);
`gyro_resolution` 1/[time] >= 0; `noise` [length]/[time]^2 >= 0, default 0; `gyro_noise` 1/[time] >= 0, default 0;
`delay` [time] >= 0, default 0; `sample_rate` >= 1 Hz; `supply_voltage` [V]; `supply_current` [A]. Failure modes:
stuck, noisy, dropped. Pins: `accel_x`, `accel_y`, `accel_z`, `gyro_x`, `gyro_y`, `gyro_z` output analog; `vcc`
power_in; `ground`. Placeholder part: ranges `4 g` as `39.24 m/s^2` and `500 deg/s`, resolutions `1.2e-3 m/s^2`,
`0.015 deg/s`, sample rate `200 Hz`.

### `brake` (category safety, LIB-027)

Power-off brake on a joint. Parameters: `mass`; `brake_torque` [torque] > 0; `engage_delay` [time] >= 0;
`release_delay` [time] >= 0; `coil_voltage` [V] > 0; `coil_current` [A] >= 0; `power_off_engaged` flag default true.
Pins: `release` input digital (high releases when `power_off_engaged`), `vcc` power_in, `ground`. Ratings name
`brake_torque` as the holding torque. The joint it acts on is design data (an `acts_on` block analogous to DES-043
`senses`), not in this brief; write that in the file header so the integration brief knows. Placeholder part:
`0.5 N m`, delays `20 ms`/`30 ms`, `24 V`, `0.25 A`.

## 3. Tests (LIB-021 reference tests are not yours; the validation tests are)

For each new file: the shipped file loads and validates; every required parameter missing; each bound at, inside
and outside; wrong dimension; unknown key; `front_end` accepted and an unknown category refused; the chip fields as
in section 1. `test_catalog_registry.py`: the four types resolve from the default registry by name. Every test
cites its requirement IDs in `@pytest.mark.req`.

## Report

The schema-author template. Under Open issues list every spec row you believe is now satisfied but whose stub you
did not convert (the stubs for LIB-019, LIB-025, LIB-027 in `tests/unit/test_lib_stubs.py` are converted by brief
A2 once the C++ side exists; leave them).
