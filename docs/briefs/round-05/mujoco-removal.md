# MuJoCo bridge removal plan (written only; brief M3-X2 item 6)

Nothing here is done. The removal is one commit after the owner's retirement decision at the gate (ADR 0004,
`docs/milestones.md` retirement table: criterion 1 met, criterion 2 the owner's call, ruling 83). Until then the
bridge stays and the gate runs the reference suite on both adapters. The removal commit carries an ADR 0004 status
line, a Decisions log row and the spec edits below (R-18).

## What replaces the differential tests

The fixed bridge baselines (PHY-025): `test_cases/gantry/bridge_baselines/G1.yaml` to `G14.yaml` (13 files, no G12)
and `test_cases/pendulum/bridge_baselines/P1.yaml`, `P2.yaml`, generated once on 2026-11-14 and never regenerated.
PHY-024's comparison (`tests/acceptance/test_phy_stubs.py`: `test_phy_024` and `test_phy_024_misses_of_record`,
per-axis tolerances and the list of record since brief M3-X2d) already reads them, not a live bridge run, so it
survives unchanged. Its list of record since brief M3-X2h (the native baseline regenerated after E8): four misses, all G10
(`stall_fault`, `slip_fault` explained by brief M3-X2i, a model difference at the homing latch; `stall_rehomed`,
`slip_carriage_stopped` their consequences); G7 and G9 are explained and inside their tolerance; the efficiency-damping
release artefact is bounded and published per run (brief M3-X2i; its end-rate form milestone 4 item E9). PHY-021's live two-adapter comparisons (`tests/reference/test_phy_stubs.py::test_phy_021` and every
`mujoco`-parametrized reference test) are deleted; the drive-free references keep their closed forms, which the
native adapter already meets (PHY-023). PHY-016 (`test_phy_016`, the bridge carrying G5 to G14) is retired with
its row; its values of record are the fixed baselines.

## C++ (`core/`)

| Item | Action |
|---|---|
| `core/physics/mujoco/` (`include/`, `src/`, `tests/`, `CMakeLists.txt`, `CLAUDE.md`, `HISTORY.md`, `README.md`) | Delete; `HISTORY.md` content moves to `docs/adr/0004` notes or the physics `HISTORY.md` |
| `core/physics/CMakeLists.txt` `add_subdirectory(mujoco)` | Delete the line |
| `core/cmake/MuJoCo.cmake` (pinned release fetch with SHA-256), included only from `core/physics/mujoco/CMakeLists.txt` | Delete |
| CMake target `mechsim_physics_mujoco` | Gone with the directory; remove from `core/bindings/CMakeLists.txt` lines 21 (`set_target_properties`) and 39 (`target_link_libraries`) |
| `core/bindings/CMakeLists.txt` lines 42 to 47: `install(FILES $<TARGET_FILE:mujoco::mujoco> ...)` and the `$ORIGIN` rpath note | Delete: libmujoco no longer ships in the wheel |
| `core/bindings/` `make_physics_adapter("mujoco")` branch and any MuJoCo-only binding (prepared MJCF text in `PreparedModel`) | Delete the branch; `make_physics_adapter` accepts `native` only, anything else an `AdapterError` naming the value; `PreparedModel`'s MJCF field removed or left empty with a contract note (owner's contract review) |
| Catch2 tests tagged with bridge requirement IDs (PHY-011 to PHY-016 under `core/physics/mujoco/tests/`) | Deleted with the directory; traceability regenerated |

## CI

| Item | Action |
|---|---|
| `.github/workflows/cpp.yml` step "MuJoCo include boundary (PHY-011, ADR 0014)" | Replace by a step that fails on any `mujoco/` include anywhere in `core/` |
| `.github/workflows/cpp.yml` configure comment on the MuJoCo fetch | Delete; check the cache keys that name the MuJoCo archive |
| `.github/workflows/python.yml` | No MuJoCo step; the `python-long` job loses `test_phy_016` with the test |

## Python (`mechsim/`)

| Item | Action |
|---|---|
| `mechsim/physics/mujoco/` (`__init__.py`, `prepare.py`, `translate.py`, `CLAUDE.md`) | Delete |
| `mechsim/testing/schema.py` `PhysicsAdapterName = Literal["mujoco", "native"]`, `MUJOCO_ADAPTER`, `PHYSICS_ADAPTERS` | `Literal["native"]`; a case or `--set` naming `mujoco` is a validation error naming the field and saying the bridge was retired (TST-040 restated) |
| `schemas/test_case.json` `physics.adapter` enum value `mujoco` | Regenerate from the model (enum `["native"]`) |
| `mechsim/testing/core_driver.py`: `_bridge_*` translation import (the lazy import that PHY-026 checks), `payload_stand_in_line`, `BRIDGE_NODE_POSES`, the `NOT_EVALUATED` rigid-member line, `member_frame_loads` (bridge wrenches), the non-native branch of `_run`, `core_builder`'s `rigid` flag | Delete the bridge branches; `core_builder(design)` builds as the design states |
| `mechsim/testing/runner.py` `_adapter_suffix` (`-mujoco` run id) | Keep the rule for any non-default adapter or delete with TST-040's identity clause (owner) |
| `mechsim/testing/driver.py` `RunConfig.physics_adapter`, `Builder` adapter argument | Keep the field (archive identity) with `native` the only value |
| `mechsim/record/archive.py` `DEFAULT_PHYSICS_ADAPTER = "mujoco"` (an archive without the attribute was a bridge run) | Keep: old archives still read; reading never needs the engine (ruling 17) |
| `mechsim/builder/` rigid-member build for the bridge (BLD-033, ruling 29) | Delete the rigid path and its line |
| `mechsim/integrity/` "built rigid, not evaluated" note (rulings 29, 34) | Delete the note path (not this brief's owner: integrity) |
| `tools/check_imports.py` MuJoCo boundary rule and `tests/unit/test_check_imports.py` cases for it | Replace by a rule that `mujoco` is imported nowhere |
| `pyproject.toml` | Check for a `mujoco` dependency or wheel include and remove it; the wheel no longer carries libmujoco |

## Tests

Delete or rewrite every bridge test: `tests/unit/test_mjcf_translation.py` (delete); the bridge cases in
`tests/unit/test_core_driver.py` (prepared model), `test_core_wiring_m2.py` (payload start), `test_driver_errors.py`
(unmappable design, bridge approximations, the `mujoco` party-name probe), `test_integrity_rigid_runs.py` (bridge
run), `test_native_clamped_members.py` (rigid bridge build), `test_native_adapter_selection.py`,
`test_facade_service.py`, `test_runner_fakes.py`, `test_bindings.py`, `test_viewer.py`, `test_builder_damping.py`,
`test_physics_substeps.py` (bridge refusal of substeps), `test_native_run_lines.py`, `test_integrity_tier1.py`
(bridge paths), `tests/unit/test_phy_stubs.py` (keep PHY-025), `tests/reference/test_phy_stubs.py` (PHY-021 live
comparison) and every `mujoco`-parametrized reference test, `tests/invariant/test_phy_stubs.py` (PHY-026 stays,
simplified: nothing imports MuJoCo at all), `tests/invariant/test_import_boundaries.py` (bridge boundary),
`tests/acceptance/test_acc_stubs.py` (`bridge_g7`, `bridge_gantry`, `BRIDGE_RUNS` (empty since M3-X2c: G5 and G6
run on native), `test_phy_016`,
`test_bridge_keeps_the_rope_payload_on_its_carrier_with_one_line`), `tests/acceptance/test_phy_stubs.py` (PHY-023's
`not mujoco` filter becomes the whole tier).

## Specs and contracts

`docs/spec/phy.md` rows PHY-011 to PHY-016, PHY-019 (bridge mapping) and PHY-021 marked retired with the date;
PHY-023 to PHY-026 restated as met; TST-040; BLD-033, BLD-039 (rope stand-in); the ADR 0003 approximation list for
the bridge; ADR 0004 status; ADR 0014 (include boundary); `docs/spec/traceability.md` regenerated. Contracts:
`core/physics/CLAUDE.md`, `core/bindings/CLAUDE.md`, `mechsim/physics/CLAUDE.md`, `mechsim/testing/CLAUDE.md`
(bridge lines), root `CLAUDE.md` invariant 5 and the Layout block (`adapters: mujoco/ native/`).

## Inventory (brief M3-RM item 1, 2026-11-15)

`git grep -l -i mujoco` at the brief's base (63215f2): 158 files.

| Class | Files | Action |
|---|---|---|
| Bridge packages and their fixtures | 25: `core/physics/mujoco/**` (adapter, tests, 11 MJCF fixtures, contracts), `core/cmake/MuJoCo.cmake`, `mechsim/physics/mujoco/**`, `tests/unit/test_mjcf_translation.py` | Delete; `core/physics/mujoco/HISTORY.md` moves into `core/physics/HISTORY.md` |
| Fixed baselines (PHY-025) | 15: `test_cases/*/bridge_baselines/*.yaml` | Keep unchanged: their provenance lines name the bridge run that made them |
| History | 47: `docs/briefs/**`, `docs/rulings/**`, `docs/cost-log*.md`, `docs/queue.md`, the architecture and requirements documents | Keep: records of what was decided and run; not this brief's to edit (queue, cost log) |
| ADRs | 10 under `docs/adr/` | Keep as decision history, except ADR 0004 (status: implemented, for the owner's review) and ADR 0014 (status line: the bridge half retired); `docs/adr/README.md` rows follow |
| Restate | 61: build (`core/physics/CMakeLists.txt`, `core/bindings/CMakeLists.txt`, bindings `factories.cpp`, `bind_physics.cpp`), CI (`cpp.yml`), the kernel's `PreparedModel::engine_artifact` comment and the `<mujoco/>` literals in the kernel and native tests (field kept, generic: no shipped adapter takes an artifact and native refuses one), comments in `safety.hpp` and `test_stepper.cpp` (kept as the origin of the 400 envelope, worded as history), `mechsim/physics/__init__.py`, `mechsim/testing/{schema,core_driver,core_wiring}.py`, `mechsim/record/archive.py` (kept: an archive without the attribute was a bridge run and still reads, ruling 17), `schemas/test_case.json`, `tools/check_imports.py`, `designs/gantry/design.yaml` comment, every bridge test listed above, the contracts (`CLAUDE.md` root, `core/`, `core/bindings/`, `core/kernel/`, `mechsim/`, `mechsim/physics/`, `mechsim/record/`, `mechsim/testing/`, `mechsim/viewer/`, `tools/`, `docs/`), READMEs, `.claude/agents/invariant-auditor.md`, `docs/milestones.md`, specs `phy`, `tst`, `bld`, `int`, `inv`, `sig`, `traceability` | Restate in the commit that changes the code they describe (R-18) |

Added to the plan by this inventory (beyond the gate review's additions): `core/signals` `safety.hpp:95` and
`test_stepper.cpp:106` comments, `docs/spec/sig.md` brake-hold row, `docs/spec/int.md` rigid-build row,
`mechsim/viewer/CLAUDE.md`, `tools/CLAUDE.md`, `docs/CLAUDE.md`, `core/kernel/CLAUDE.md`, the facade's adapter check
(`mechsim/facade/service.py` `run_id_of`), the driver's `physics.adapter` check, `tests/unit/test_bindings.py`
adapter-name list. Kept against the plan: the builder's `rigid_members` option (BLD-033) is a generic build option
used by the builder and integrity tests and the integrity rigid-note path (not this brief's owner); BLD-033 is
restated so no adapter selects it, and the option's removal is left to the owner.
