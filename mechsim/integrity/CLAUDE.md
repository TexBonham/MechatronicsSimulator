# mechsim/integrity/: contract

Mechanical integrity over a finished run (INT, ADR 0009; the tier 2 Python side of round 5 ruling 5). Rules of each
check: `docs/spec/int.md`, section "Check rules". Schema `schemas/integrity.json` from `config.IntegrityConfig`;
regenerate with `uv run python -m mechsim.integrity.config --write-schema`, never hand-edit.

| File | Holds |
|---|---|
| `plugin.py` | `IntegrityCheck` (ABC: `name`, `tiers` fidelity tiers, `parameters` model, `evaluate(inputs, parameters, tier) -> CheckResult`, `check_tier`), `RunInputs(times, members, materials, temperature, load_factor, minimum_safety_factor, deflection_limit, joints, series, ropes, design_tension)`, `MemberInputs`, `SectionProperties` (I_y, I_z, c_y, c_z, `single_axis`), `RopeLoads`, `Wrenches` (member frame, (rows, 3) each), `JointLoads` (mount or fastener, world-aligned reaction), `MaterialView` protocol (= `materials.IntegrityMaterialView`), `Spread`, `CheckRow`, `CheckResult` (`cycle_counts`, `notes`, `rows_of_tier`), `ForceQuantity`/`TimeQuantity`, `IntegrityError` |
| `beam.py` | `axial`, `normal_stress`, `shear_stress`, `fibre_histories`, `deflection` (quasi-static, 201-point grid), `end_conditions`, `effective_length_factor`, `first_frequency`, `mount_stress`, `peak` |
| `checks/*.py` | One check per module, each exporting `CHECK`: `strength` (tiers 1, 2), `deflection`, `buckling`, `fatigue` (`rainflow`, `reversals`, `cycles_to_failure`, `damage`), `fastener`, `impact` (`events`), `resonance` |
| `registry.py` | `discover()`: modules under `checks/` with `CHECK`, plus entry points `mechsim.integrity_checks`; duplicates and non-checks are errors |
| `config.py` | `IntegrityConfig` (discriminated `checks` generated per registered check, `halt_on_crossing`, `thermal` slot), `entries()` (strength and deflection always), `load_config`, `parse_config`, `config_for(design_dir)` (`integrity.yaml`, default when absent), `--write-schema` |
| `inputs.py` | `run_inputs(build, recording, materials)`: members from `BuildResult.members` (+ calibrated frequency from `lumped`), end-loads from `recording.recording.series["physics/members/..."]` by column name, fastenings from `physics.fastenings` with joint reactions, ropes from `physics.ropes` with `physics/ropes/max_tension`, design-load factor, minimum safety factor, accuracy target, ambient temperature; `section_properties` (`second_moment_y`, `fibre_y`, `fibre_z`; single-axis fallback) |
| `table.py` | `evaluate(config, inputs) -> IntegrityReport` (subclass of `testing.driver.IntegrityResults(members, ropes)`: rope rows with `snap`, `design_load`; `approximation: single-axis section` member flag; + `checks`, `load_factor`, `notes`, `cycle_counts()`, `report_json()`); `THERMAL_NOTE` |
| `export.py` | `load_case(view, members, design_name, design_version, run_id) -> LoadCase` (`to_json`, `to_csv`), `write_beside(archive, results, recording, metadata)` -> `<run>.integrity.json`, `<run>.load_case.json/.csv` |
| `service.py` | `IntegrityService(designs_root, config=None, design_dir=None)`: wired by `testing.core_driver.core_services` as `Services.integrity` (`(build, case, recording) -> IntegrityReport | None`, None without `integrity.yaml` beside the design file, ruling 27); materials from `MaterialSet.from_files(<design folder>/<materials.overrides>)` |

## Conventions

- End-loads in the member frame (ruling 24); internal axial force -F_start.x, +F_end.x.
- Member axis for direction-dependent strength: the lumped member's placement x in its parent frame (`inputs.member_axes`, ruling 24); body x when the build has no lumped record (rigid build).
- Deflection rows always carry `approximation: static deflection from end loads` (ruling 28); impact rows flag `impact_margin_below_minimum`.
- Fastener ratings from configuration only in milestone 3 (catalog grade ratings owed, ruling 27).
- Table values at the design load (recorded loads times 1 + allowance from the design file, INT-019).
- Safety factors use the minimum strength spread for flags and the table; spreads kept in `CheckRow.margin`.
- Integrity never emits `integrity` events (ruling 26: tier 1 only).
- Runner: `Services.integrity` result attached to metadata; `IntegrityError` ends the run as `run_failed`;
  `write_beside` after the archive is written.
- Facade: `integrity_results(run_id)`, `load_case(run_id, format)` (FAC-013); `validate` file kind `integrity`.
- `report_json()` keys: `checks`, `cycle_counts`, `load_factor`, `notes`, `table` (R's `to_json`).
