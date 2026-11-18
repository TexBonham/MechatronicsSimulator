# M3-M: materials lookups, process-aware strengths, S-N curves, overrides

Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-m` on branch `wip/05-m` from
`round/05-milestone-3`. Python only; one `tools/locked_build.sh sync` at the start. Read first: root `CLAUDE.md`,
`mechsim/CLAUDE.md`, `mechsim/materials/CLAUDE.md`, `schemas/README.md`, spec rows MAT-002, 003, 006 to 012, 014 to
017, 021, 024 in `docs/spec/mat.md` (and the M1 rows already met, to keep them), INT-005, INT-008 (what integrity
will ask of you), round 5 ruling 7 in `docs/queue.md`, the shipped files under `mechsim/materials/data/`, the stubs
in `tests/unit/test_mat_stubs.py`. Call cap 150. Filtered: `uv run --no-sync pytest -q tests/unit/test_material*.py
tests/unit/test_mat_stubs.py`; one full `uv run --no-sync pytest -q`, ruff, format, mypy, check_imports,
traceability `--write` first. Commit once as WIP ("WIP M3-M: materials"), no trailer, no push. Spec edits same
commit.

## Ownership

`mechsim/materials/**`, `schemas/material.json`, a new `schemas/material_overrides.json`, `mechsim/materials/data/*`,
`schemas/examples/material*.yaml` if the examples live there, `tests/unit/test_material*.py`, `tests/unit/test_mat_stubs.py`,
`docs/spec/mat.md`, `docs/spec/traceability.md`, `mechsim/materials/CLAUDE.md`. The design-file field that
references an overrides file belongs to brief D (`mechsim/design/`); you define the overrides file and the
lookup API, and name the field D must add (`materials.overrides: <path>`).

## Deliverables

1. Lookups (MAT-006 to 008): `lookup(material, property, temperature, spread="typ")` evaluates the tabulated
   `temperature_dependence` by linear interpolation; outside the stated range is a `MaterialError` naming the
   property and the range (never extrapolate); `spread` in min, typ, max. Every property, not only strengths.
2. Process attributes (MAT-009, 024): `printed` (build_direction, layer_height, walls >= 1, infill 0 to 100
   percent), `machined`, `molded` already in the model; validation of out-of-range values at load with errors
   naming file and field (tests at, inside, outside each bound).
3. Direction-dependent strength (MAT-010, 011; ruling 7 placeholder): `strength(material, kind, direction_body,
   process, temperature, spread)` with kind yield, ultimate, shear: for a printed process, the loading direction's
   angle theta to the build direction blends in-layer and interlayer factors as f = f_in cos^2 theta + f_inter sin^2
   theta (document it as the placeholder axes model owed by the owner); machined and molded return the isotropic
   value. Reference: along the build direction the interlayer factor, across it the in-layer factor, at 45 degrees
   the mean, each within 1e-12.
4. S-N lookup (MAT-012, 003): `allowable_amplitude(material, cycles)` by log-log interpolation between the curve's
   points; outside the data range a `MaterialError`; test a two-point curve at the endpoints, the geometric midpoint
   and one outside point each side.
5. Shipped printed materials (MAT-014, 015): PLA, PETG, ABS present with orientation and infill attributes and
   conservative wide spreads (check and complete the fields the lookups need: thermal, S-N with at least three
   points, direction factors); `placeholder` and `wide_spread_default` true.
6. Overrides (MAT-016, 017): a `material_overrides.yaml` file kind listing per material the properties overridden
   (value or full spread, with a `source` string such as "coupon test 2026-11-01"); `load_overrides(path)` and a
   `MaterialSet(shipped, overrides)` whose lookups apply the override before the shipped value; the design
   references the file (field defined here, added by D); a run's recorded inputs include the overrides file (name the
   metadata key for brief R).
7. MAT-021: one `IntegrityMaterialView` (or equivalent) giving yield, ultimate, shear, S-N allowable and
   direction-dependent strengths with spreads, the API brief I1 consumes; document it in the contract.
8. MAT-002 conversions and any M1 rows the stubs still hold.

## Report

Under 250 words: files, commands with counts, the API names for D and I1, stubs converted, Open issues.
