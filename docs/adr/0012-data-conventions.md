# ADR 0012: Units, seeding, signal naming, versioning, archives, baselines, tooling

Date: 2026-11-02
Status: Accepted
Closes rulings: L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, D1, D2, D3, D4, R1, R2
Supersedes: Architecture doc, Technology recommendation, "Data and files" paragraph in part (schema source and unit
notation in files)

## Context
The architecture doc's invariants 3 (data files are the source of truth) and 4 (determinism), and its Data and files
paragraph, need concrete conventions before phase 1 contracts and schemas are written. The tooling rulings were tabled
and accepted as a block on 2026-11-02. Note that tooling rulings R1 and R2 are distinct from product rulings R-01 and
R-02.

## Decision
Python toolchain:

| # | Decision |
|---|---|
| L1 | Python 3.12 floor, pinned for the toolchain; newer only when every dependency wheel ships |
| L2 | uv, PEP 621 `pyproject.toml`, committed lock file |
| L3 | ruff for lint and format; mypy strict across `mechsim/`, per-module relaxation only for missing stubs |
| L7 | numpy and scipy for analysis-side numerics |
| L9 | argparse CLI with commands validate, run, suite, replay |
| L10 | Standard-library logging with a structured formatter |

Files, units and archives:

| # | Decision |
|---|---|
| L4 | pydantic v2 models are the source; JSON Schemas in `schemas/` are generated and checked in, with a staleness test |
| L5 | Quantities in files as unit strings (`"2.5 kg"`), parsed by pint at load; plain SI floats inside; pint never in the tick loop |
| L6 | ruamel.yaml in round-trip mode |
| L8 | HDF5 run archives via h5py; design file, test case, seed, mode and git hashes recorded as attributes |

Determinism, naming and versioning:

| # | Decision |
|---|---|
| D1 | One integer seed per run; a SeedSequence spawns a child generator per module instance in fixed order |
| D2 | Dotted signal names `instance.signal`, unique per design; operator messages use the requirements doc labels verbatim |
| D3 | Design version is a semver string plus the design file's git blob hash, recorded in every archive |
| D4 | Regression baselines are YAML scoring summaries in git; archives are never in git |

Repository and CI:

| # | Decision |
|---|---|
| R1 | CI on Ubuntu for lint, types and all test tiers, headless; Windows joins the matrix when milestone 1 closes |
| R2 | A round branch per phase or milestone off main; merge commits only; cut fresh after each PR |

## Consequences
- Schema staleness is a CI failure; hand edits to `schemas/` cannot land.
- Units are checked once at load; the tick loop sees only SI floats.
- Round-trip YAML keeps comments and ordering when tools rewrite design files.
- Every archive identifies exactly what produced it (design, test case, seed, mode, git hashes, design version), which
  run-twice determinism tests (ADR 0008) and replay (ADR 0006) depend on.
- The fixed spawn order is part of the determinism contract: reordering module instances changes the generator each
  one receives.
- Baseline diffs are reviewable text; large archives stay outside the repository.
- Windows support is a milestone 1 exit item.

## Open items
- C++ toolchain versions and the C++ CI matrix beyond ADR 0001's gates: phase 0c scaffolding.
