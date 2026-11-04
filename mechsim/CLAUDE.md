# mechsim/: Python application layer

One subpackage per module (R-14, ADR 0005). Python 3.12, uv, ruff, mypy strict (L1-L3).

| Subpackage | Holds | Spec prefix |
|---|---|---|
| `library/` | Component types, plugin interface, shipped catalog (YAML) | LIB |
| `materials/` | Materials database, process rules, direction-dependent limits | MAT |
| `design/` | Design-file schema, loader, validation, CAD import (STEP, URDF) | DES |
| `builder/` | Design file to engine-neutral physics model, wiring, chip config (ADR 0003) | BLD |
| `physics/` | Adapters: `mujoco/` (bridge), `native/` (own engine binding), `chrono/` (reserved in the architecture doc, not created; ADR 0004 open item) | PHY |
| `testing/` | Test-case schema, runner, fault fan-out, sweeps, regression | TST |
| `record/` | Recorder, HDF5 run archive, CSV/JSON/MAT exports | REC |
| `integrity/` | Mechanical integrity check plugins, safety factors, load-case export (ADR 0009) | INT |
| `scoring/` | Pass criteria, suite reports, regression diffs | SCO |
| `facade/` | Facade service and protocol (ADR 0006) | FAC |
| `viewer/` | Replay and event timeline, served through `facade/` | VIE |
| `schema_base.py` | `StrictModel` (extra forbid, strict) every data-file model derives from; `format_errors` (one line per validation error) | INV |

Physics adapters: everything under `physics/`. Only the neutral model crosses into an adapter (invariant 5).

## Import rules

| Import | Allowed only in |
|---|---|
| MuJoCo | `physics/mujoco/` |
| Qt, display libraries | `viewer/` (invariant 9) |
| Reference collision library | Tests only (ADR 0002) |
| pint | Load-time parsing; never the tick loop (L5) |

Checked by the invariant tier's import-boundary test.

## Conventions

- pydantic v2 models are the source; JSON Schemas in `schemas/` are generated, checked in, staleness-tested (L4).
- Quantities in files are unit strings (`"2.5 kg"`), parsed by pint at load; plain SI floats inside (L5).
- YAML via ruamel.yaml round-trip mode (L6).
- Signal names dotted `instance.signal`, unique per design; operator messages use requirements doc labels verbatim (D2).
- One integer seed per run; SeedSequence spawns one child generator per module instance in fixed order (D1).
- HDF5 archives record design file, test case, seed, mode, git hashes (L8).
- CLI (`cli.py`, console script `mechsim` -> `cli:main_exit`): argparse, commands validate, run, suite, replay (L9);
  `run`/`suite` take `--set name=value`, `--out`, `--design <designs>/<name>/design.yaml`. Exit 0 passed (or all
  files valid), 1 a criterion failed (or a file invalid), 2 error: run error, case not run, unknown file kind, missing
  path, usage error, or no driver wired. `default_services()` wires `testing.core_driver` (unwired if the extension is
  not built).
- Tests: `@pytest.mark.req("LIB-001")`; stubs are strict xfail.

## Contracts

| Subpackage | Contract |
|---|---|
| `library/` | `library/CLAUDE.md` |
| `materials/` | `materials/CLAUDE.md` |
| `design/` | `design/CLAUDE.md` |
| `physics/` | `physics/CLAUDE.md` |
| `testing/` | `testing/CLAUDE.md`; real driver `testing/core_driver.py`: `CoreDriver`, `RecordingView`, `RecordArchiver`, `RecordArchiveReader`, `core_builder`, `core_services`; core parties from a build `testing/core_wiring.py` |
| `record/` | `record/CLAUDE.md` |
| `scoring/` | `scoring/CLAUDE.md` |
| `facade/` | `facade/CLAUDE.md` |
