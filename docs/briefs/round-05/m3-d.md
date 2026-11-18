# M3-D: structural data: design rows, flexible beam, hook and rope types, builder members and ropes

Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-d` on branch `wip/05-d` from
`round/05-milestone-3`. Read first: root `CLAUDE.md`, `mechsim/CLAUDE.md`, the `CLAUDE.md` of `mechsim/design`,
`mechsim/library`, `mechsim/builder`, `mechsim/physics` (neutral model: lumped members, ropes if present), spec
rows DES-031 to 034 in `docs/spec/des.md`, LIB-020, 030, 032 in `lib.md`, BLD-008, 009, 014 to 019, 031, 032 in
`bld.md`, ENG-022 (what a lumped beam is), round 5 rulings 2 and 6 in `docs/queue.md`, the gantry design's
`structural` block, members and open parameters (`rail_section_area`, `rail_section_second_moment`,
`rail_calibration`, `deflection_target_mode`), `docs/gantry-team-inputs.md` Structure rows, the stubs in
`tests/unit/test_des_stubs.py`, `test_bld_stubs.py`, `test_lib_stubs.py`, `tests/reference/test_bld_stubs.py`,
`tests/reference/test_lib_stubs.py`. Brief M defines `materials.overrides: <path>` for the design file (add the
field; validation that the file exists and loads through M's API if M has landed, else a path check with a note).

Call cap 150 with a handoff rule (commit WIP at the cap, list the rest). Python only plus the engine binding for
reference tests (`_core_ext.engine` is importable from `tests/`); one `tools/locked_build.sh sync` at the start.
Filtered: `uv run --no-sync pytest -q tests/unit/test_design_schema.py tests/unit/test_builder*.py
tests/unit/test_library_catalog.py tests/unit/test_des_stubs.py tests/unit/test_bld_stubs.py tests/unit/test_lib_stubs.py
tests/reference/test_bld_stubs.py tests/reference/test_lib_stubs.py`; one full `uv run --no-sync pytest -q`, ruff,
format, mypy, check_imports, traceability `--write` first. Commit once as WIP ("WIP M3-D: structural data"), no
trailer, no push. Spec edits same commit.

## Ownership

`mechsim/design/**`, `mechsim/library/**` (catalog files `flexible_beam.yaml`, `hook.yaml`, `rope.yaml` new),
`mechsim/builder/**`, `mechsim/physics/neutral.py` (additive: rope element, member section and support fields if
absent), `schemas/`, `designs/gantry/design.yaml` (only the fields this brief adds, with placeholders flagged;
the rails become flexible members per ruling 6 but the suite keeps passing on the bridge: if the bridge cannot run
the flexible rails at four segments, keep `rigid_members: true` for milestone 2 cases and say so), the tests
listed, the three `CLAUDE.md` files, `docs/spec/{des,lib,bld}.md`, `docs/spec/traceability.md`.

## Deliverables

1. DES-031 to 034: design-load rule as data (exists: confirm and convert), accuracy target as the deflection
   limit (DES-032), flexible member declarations with cross-section (shape and dimensions, or area and second
   moments) and segment count (BLD-018 data), per-member calibration (known deflection and first natural
   frequency, source hand calculation or FEA; DES-034). Schema regenerated; tests per field.
2. LIB-020, 030, 032: catalog types `flexible_beam` (section and material parameters on top of the rigid body's;
   LIB-030), `hook` (a rigid attach point with capacity), `rope` (EA, damping, mass per length, breaking tension,
   rest length; the ADR 0016 element). Reference tests (LIB-032): a cantilever and a simply supported flexible beam
   built from a catalog part through the builder, stepped on the engine binding, match the closed-form static
   deflection and first natural frequency within the tolerance the lib.md row states (set the row's proposal if
   "not set": 2 percent at eight segments, derived from the lumped chain's known convergence; T4 proposed).
3. BLD-008, 009: the structural member list (name, section, material, support conditions, mounts) emitted by the
   builder as part of the build result and recorded with the run (name the field for brief R).
4. BLD-014 to 017: every flexible member emitted as a multi-segment lumped beam in the neutral model; the three
   closed-form checks through the builder on the engine binding (cantilever tip deflection and frequency, simply
   supported midspan deflection and frequency) with derivations in the tests.
5. BLD-019: calibration: tune the member's effective E I (and the tip mass if needed) so the lumped beam
   reproduces the design's recorded deflection and first natural frequency; report the scale factors in the build
   result; a member without calibration data uses the section and material as is, flagged.
6. BLD-031: every fastener and mount emitted as an individually addressable constraint (name, bodies, kind) in the
   neutral model; a test that a named mount maps to one constraint index the adapter can disable.
7. BLD-032: a rope emitted as a lumped chain element in the neutral form (ADR 0016 shape; the rope element in
   `neutral.py` is additive and validated); the gantry's payload rope is declared in the design but the case files
   keep the spherical-joint hook until X1 (say so in the design note).
8. Conversions of the DES, LIB and BLD stubs this brief proves; contracts updated.

## Report

Under 300 words: files, commands with counts, the calibration scheme in two lines, stubs converted, items left,
Open issues.
