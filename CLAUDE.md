# MechatronicsSimulator

General mechatronic simulator (R-01). The gantry is the first test case and reference design, never the design target.

## Read first

| What | Where |
|---|---|
| Current state and next brief | `docs/queue.md` |
| Planning rulings (R-, L-, P-, C-, D-, T-, W- IDs) | `docs/rulings/` |
| Decisions of record | `docs/adr/README.md` |

Cost log: `docs/cost-log.md` (archive `docs/cost-log-archive.md`), never in any `CLAUDE.md` (W-07).

## Invariants

Full text: architecture doc, "Repository starting point".

| # | Invariant | Enforced by |
|---|---|---|
| 1 | Controller never sees true state; all physics-controller traffic crosses the signal layer via the pin-binding table | Review |
| 2 | Nothing design-specific in code; gantry only under `designs/`, `test_cases/`, `firmware/` | Review |
| 3 | Data files are the source of truth; every YAML validates; SI inside, units explicit in files | Schema-validation hook; CI invariant tier: schema validity |
| 4 | Determinism: same design, test case, seed, mode give an identical recording | CI invariant tier: run-twice determinism |
| 5 | Engines and emulators stay behind `mechsim/physics/` and `core/controller/`; only the neutral model crosses | Import-boundary hook; CI invariant tier: import boundaries |
| 6 | Fidelity tiers are additive | Review |
| 7 | Failures are errors: drift, solver failure, emulator fault end the run with a report | Review |
| 8 | Board interface is the only coupling to firmware | Firmware host build rejects other includes (C2) |
| 9 | Core is headless: no Qt/display outside viewer code; one serializable facade; CLI first client | Import-boundary hook; CI invariant tier: import boundaries |

Hooks per `docs/queue.md` phase 0c; CI gates per ADR 0008.

## Key constants

| Constant | Value | Scope | Source |
|---|---|---|---|
| Physics step | 1 ms | MuJoCo adapter (implicitfast); own-engine integrator not set (engine ADR) | P2 |
| Control tick | 1 kHz, equal to physics step | Gantry default, data not code; kernel supports unequal rates | C4 |
| Design-load rule | Rated load + 15% (hoist, trolley) | Gantry example; rule is per design | Requirements, Physics |
| Rated load | not set | Gantry; owed by gantry team | Rulings, still open |
| Position accuracy | 1 cm | Gantry (G2, G3) | Requirements |
| Deflection target | 1 cm; static or dynamic not set | Gantry (G15) | Decisions log row 17 |
| Min safety factor, G12 limits | not set | Gantry; owed by gantry team | ADR 0009 |
| Python | 3.12 floor | All | L1 |
| C++ | C++20 | `core/` | R-04 |
| Firmware C | C11 | `firmware/` | C1 |

## Layout

```
core/            C++ core runtime, one library, own CMake tree
  engine/ kernel/ signals/ controller/ bindings/ viewer_debug/
mechsim/         Python package, one subpackage per module
  library/ materials/ design/ builder/ physics/ testing/ record/ integrity/ scoring/ facade/ viewer/
board_interface/ C header firmware writes against
chips/           chip definitions (platform description, SVD, pin map)
schemas/         JSON Schema, generated from pydantic models
designs/         gantry/ (reference), pendulum/ (second)
test_cases/      test suites; gantry/ holds G1-G16
firmware/        controller sources; vendor HAL only under board/<chip>/
web/             browser client
tests/           the simulator's own tests
docs/            adr/ spec/ rulings/ queue.md cost-log.md
```

## Where to find X

| Task | Touch, in order | Done when |
|---|---|---|
| Component type | `mechsim/library/` plugin class, schema entry, catalog YAML, reference test | Reference test matches analytical result or datasheet curve within stated tolerance |
| Chip | `chips/<name>/` platform description, SVD, pin-map template | Peripheral smoke test passes in emulated mode for every pin-map peripheral |
| Physics engine | `mechsim/physics/<name>/` adapter + neutral-model translation | Analytical reference suite passes with it |
| Design | `designs/<name>/`, `test_cases/<name>/`; no code | Builds, runs, scores with library as shipped |
| Controller mode | `core/controller/<name>/` implementing runtime interface | G1-G4 pass with same firmware |
| Integrity check | `mechsim/integrity/` plugin class, schema entry, reference test | Reference matches analytical result within stated tolerance |
| Spec requirement | `docs/spec/<module>.md`, then a stub | Traceability test passes |
| Test stub | `tests/<tier>/` test with requirement marker, strict xfail | Reported xfail; traceability passes |
| ADR | copy `docs/adr/0000-template.md`, `docs/adr/README.md` row, Decisions log row | Owner reviewed |
| Agent | `.claude/agents/<name>.md`, Agents table below | First run in cost log, reference size recorded |
| Tolerance | `docs/spec/<module>.md` only (T4) | Reviewed spec edit; never edit the test alone |
| Run the suite | `uv run pytest` | All tiers green, stubs xfail |
| Run the C++ build | `cmake --preset debug-sanitize`, `cmake --build --preset debug-sanitize`, `ctest --preset debug-sanitize` | Builds clean, tests green |

## Test conventions

| Tier (T1) | Directory | Content |
|---|---|---|
| Unit | `tests/unit/` | Per-module |
| Analytical reference | `tests/reference/` | Closed-form results within spec tolerance |
| Acceptance | `tests/acceptance/` | G1-G16 plus pendulum |
| Invariant | `tests/invariant/` | Import boundaries, schema validity, run-twice determinism |

- Python: `@pytest.mark.req("ENG-001")`; C++ Catch2: tag `[ENG-001]`.
- Stubs are strict expected failures before code (T3); a stub that passes unconverted fails the run.
- ID pattern `^[A-Z]{2,5}-\d{3}$`. Prefixes: ENG engine, KER kernel, SIG signals, CTL controller, LIB library, MAT materials, DES design, BLD builder, PHY physics adapters, TST testing, REC record, INT integrity, SCO scoring, FAC facade, VIE viewer, FW firmware, INV invariants, ACC acceptance.
- Behavior changes cite requirement IDs and carry the spec edit in the same commit (R-18).

## C++ rules (`core/`)

C++20, Eigen, pybind11, CMake presets `debug-sanitize` and `release`. ASan, UBSan, clang-tidy, warnings as errors. No raw owning pointers across any API. Curly braces on all control flow; no ternary operators; no comments unless the why is non-obvious.

## Firmware rules (`firmware/`)

C11, fixed-width types. Application code includes nothing outside `board_interface/`. Vendor HAL only under `firmware/board/<chip>/`. Native tick is polled (ADR 0011).

## Agents and dispatch

| Agent | Model | Purpose | Reference run |
|---|---|---|---|
| module-writer | Opus | Writes module code | not yet measured |
| schema-author | Opus | Pydantic models, schemas | not yet measured |
| invariant-auditor | Sonnet; Opus at gates | Audits invariants beside stage review | not yet measured |
| reference-validator | Sonnet | Analytical reference tier | not yet measured |
| firmware-builder | Sonnet; Opus for first shim | Firmware, native shim | not yet measured |
| docs-indexer | Haiku | Docs and `CLAUDE.md` upkeep | not yet measured |
| (docs-writer, general-purpose) | Opus | ADR set, 2026-11-02 | 102K tokens, 17 calls |

Model-gated stage commits per W-02; owner reviews gates, ADRs, contracts and one stage commit in five.

## Phases and milestones

Detail: `docs/milestones.md`.

| Phase | Content |
|---|---|
| 0a | Rulings and ADRs: done |
| 0c | Scaffolding, agents, hooks, CI |
| 0b | Spec and stubs; calibration spike alongside |
| 1 | Contracts, schemas, facade protocol |
| M1-M5 | Requirements doc milestone plan |

## Decisions

Decisions log rows 1-44 in the architecture doc; ADRs in `docs/adr/`; a new decision gets an ADR and a Decisions log row.
