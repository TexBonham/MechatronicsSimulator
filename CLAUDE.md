# MechatronicsSimulator

General mechatronic simulator (R-01). The gantry is the first test case, never the design target.

## Read first

| What | Where |
|---|---|
| Current state and next brief | `docs/queue.md` |
| Planning rulings (R-, L-, P-, C-, D-, T-, W- IDs) | `docs/rulings/` |
| Decisions of record | `docs/adr/README.md` |

Cost log: `docs/cost-log.md`, archive `docs/cost-log-archive.md`; never in a `CLAUDE.md` (W-07).

## Invariants

| # | Invariant | Enforced by |
|---|---|---|
| 1 | Controller never sees true state; physics-controller traffic crosses the signal layer via the pin-binding table | Review |
| 2 | Nothing design-specific in code; gantry only under `designs/`, `test_cases/`, `firmware/` | Review |
| 3 | Data files are the source of truth; every YAML validates; SI inside, units explicit in files | Schema hook; CI invariant tier |
| 4 | Determinism: same design, test case, seed, mode give an identical recording | CI invariant tier: run-twice determinism |
| 5 | Engines and emulators stay behind `mechsim/physics/`, `core/physics/` and `core/controller/`; only the neutral model crosses | Import-boundary hook; CI invariant tier |
| 6 | Fidelity tiers are additive | Review |
| 7 | Failures are errors: drift, solver failure, emulator fault end the run with a report | Review |
| 8 | Board interface is the only coupling to firmware | Firmware host build rejects other includes (C2) |
| 9 | Core is headless: no Qt/display outside viewer code; one serializable facade; CLI first client | Import-boundary hook; CI invariant tier |

## Key constants

| Constant | Value | Scope | Source |
|---|---|---|---|
| Physics step | 1 ms default, run configuration | Both adapters; own engine per ADR 0013 | P2, ENG-002 |
| Control tick | 1 kHz, equal to physics step | Gantry default, data not code; unequal rates supported | C4 |
| Design-load rule | Rated load + 15% (hoist, trolley) | Gantry example; rule is per design | Requirements, Physics |
| Rated load | not set | Gantry; owed by gantry team | Rulings, still open |
| Position accuracy | 1 cm | Gantry (G2, G3) | Requirements |
| Deflection target | 1 cm; static or dynamic not set | Gantry (G15) | Decisions row 17 |
| Min safety factor, G12 limits | not set | Gantry; owed by gantry team | ADR 0009 |
| Python | 3.12 floor | All | L1 |
| C++ | C++20 | `core/` | R-04 |
| Firmware C | C11 | `firmware/` | C1 |

## Layout

```
core/            C++ core runtime, one library, own CMake tree
  engine/ kernel/ signals/ controller/ physics/ (adapters: mujoco/ native/) bindings/ viewer_debug/
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
| Component type | `mechsim/library/` plugin class, schema entry, catalog YAML, reference test | Reference test within stated tolerance of analytical result or datasheet |
| Chip | `chips/<name>/` platform description, SVD, pin map | Peripheral smoke test passes emulated |
| Physics engine | `mechsim/physics/<name>/` adapter + neutral-model translation | Analytical reference suite passes with it |
| Design | `designs/<name>/`, `test_cases/<name>/`; no code | Builds, runs, scores with the shipped library |
| Controller mode | `core/controller/<name>/` implementing runtime interface | G1-G4 pass with same firmware |
| Integrity check | `mechsim/integrity/` plugin class, schema entry, reference test | Reference within stated tolerance |
| Spec requirement | `docs/spec/<module>.md`, then a stub | Traceability passes |
| Test stub | `tests/<tier>/` test with requirement marker, strict xfail | Reported xfail; traceability passes |
| ADR | copy `docs/adr/0000-template.md`, `docs/adr/README.md` row, Decisions log row | Owner reviewed |
| Agent | `.claude/agents/<name>.md`, Agents table below | First run logged, reference size recorded |
| Tolerance | `docs/spec/<module>.md` only (T4) | Reviewed spec edit; never edit the test alone |
| Run the suite | `uv run pytest` | All tiers green, stubs xfail |
| Run the C++ build | `cmake --preset debug-sanitize`, `cmake --build --preset debug-sanitize`, `ctest --preset debug-sanitize` (`-L reference` = reference tier) | Clean, green |

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
- Behavior changes cite requirement IDs and carry their spec edit in the same commit (R-18).

## C++ rules (`core/`)

C++20, Eigen, pybind11, CMake presets `debug-sanitize` and `release`. ASan, UBSan, clang-tidy, warnings as errors. No raw owning pointers across any API. Curly braces on all control flow; no ternary operators; no comments unless the why is non-obvious.

## Firmware rules (`firmware/`)

C11, fixed-width types. Application code includes nothing outside `board_interface/`. Vendor HAL only under `firmware/board/<chip>/`. Native tick is polled (ADR 0011).

## Agents and dispatch

Measured on the spike and phase 1 (2026-11-03). Supersedes the global dispatch defaults (override rule).

| Agent | Model | Purpose | Reference run (tokens / calls) |
|---|---|---|---|
| module-writer | Opus | Engine or module package against spec IDs | 110K-220K / 21-50; fix round on resume ~15K-70K / 7-50; integration brief spanning packages 300K-350K / 110-166, cut at 150 calls and hand off |
| schema-author | Opus | Models, schemas, examples, tests | 123K-210K / 26-42; fix round ~3K-30K |
| invariant-auditor | Sonnet; Opus at gates | Audits invariants beside stage review | 25K-36K / 5-7 |
| reference-validator | Sonnet | Reference tier, reports numbers | 30K / 8 |
| firmware-builder | Sonnet; Opus for first shim | Firmware, native shim | 118K / 27 (Opus); fix round ~29K |
| docs-indexer | Haiku | Docs and `CLAUDE.md` upkeep | 57K / 15 |
| Fable stage review (general-purpose) | Fable | Review from spec and diff | 108K-182K / 12-24 per brief; paired briefs 160K-240K; a grouped wave 330K / 35; re-review on resume ~11K-35K |
| docs writer (general-purpose) | Opus | ADRs, specs, scaffolding | 93K-143K / 17-29 |

Per engine package all in: ~290K small, ~430K large, one fix round each; per schema brief ~350K with review. One
blocker so far (p1, a wrong formula in the brief), no algorithm defects since. Briefs carry derivations and
physics-derived bounds; tests derived from the implementation do not catch the implementation.

Model-gated stage commits per W-02; owner reviews gates, ADRs, contracts and one stage commit in five.

## Phases and milestones

Detail: `docs/milestones.md`.

| Phase | Content |
|---|---|
| 0a, 0c, 0b | Rulings and ADRs; scaffolding, agents, hooks, CI; specs and stubs: done |
| Spike | Engine packages 1, 2a, 2b: done |
| 1 | Contracts, schemas, facade protocol: done |
| M1 | Gantry G1-G4 and pendulum P1 on the real stack, MuJoCo bridge, native firmware: done, gate passed 2026-11-05 |
| M2-M5 | Requirements doc milestone plan; M2 planned after the pause |

## Decisions

Decisions log rows 1-47 in the architecture doc; ADRs in `docs/adr/`. A new decision gets both.
