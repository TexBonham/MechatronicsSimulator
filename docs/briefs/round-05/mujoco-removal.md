# MuJoCo bridge removal plan (written only; brief M3-X2 item 6)

Nothing here is done. The removal is one commit after the owner's retirement decision at the gate (ADR 0004,
`docs/milestones.md` retirement table: criterion 1 met, criterion 2 the owner's call, ruling 83). Until then the
bridge stays and the gate runs the reference suite on both adapters. The removal commit carries an ADR 0004 status
line, a Decisions log row and the spec edits below (R-18).

## What replaces the differential tests

The fixed bridge baselines (PHY-025): `test_cases/gantry/bridge_baselines/G1.yaml` to `G14.yaml` (13 files, no G12)
and `test_cases/pendulum/bridge_baselines/P1.yaml`, `P2.yaml`, generated once on 2026-11-14 and never regenerated.
PHY-024's comparison (`tests/acceptance/test_phy_stubs.py`) already reads them, not a live bridge run, so it survives
unchanged. PHY-021's live two-adapter comparisons (`tests/reference/test_phy_stubs.py::test_phy_021` and every
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
