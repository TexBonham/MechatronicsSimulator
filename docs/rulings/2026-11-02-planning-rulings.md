# Planning rulings, 2026-11-02

Rulings from the workflow and architecture planning session of 2026-11-02. Each becomes an ADR in `docs/adr/` and,
where it changes the architecture doc, a row in its Decisions log. This file is the handoff record; the ADRs are the
documents of record once written.

Reading order for a new session: this file, then `docs/Mechatronic Simulator — Architecture and Technology
Recommendation.md`, then `docs/Mechatronic Simulator — Requirements.md`.

## Product

| # | Ruling |
|---|---|
| R-01 | Standalone general mechatronic simulator. The gantry is the first test case and reference design, never the design target. Nothing is scoped to fit only the gantry. |
| R-02 | Firmware lives in this repo under `firmware/`. Split to its own repo only if it becomes independently useful. |
| R-03 | Pendulum second design under `designs/pendulum/` from milestone 1, as the rope analytical reference and the first generality guard. The two-joint arm remains the milestone 5 proof. |

## Engine and core runtime

| # | Ruling |
|---|---|
| R-04 | Own physics engine in C++20. Eigen for linear algebra, pybind11 bindings, CMake. Address and undefined-behavior sanitizers, clang-tidy and warnings as errors in CI from the first commit. No raw owning pointers across any API. |
| R-05 | Collision detection written in-house. A reference collision library is used only inside the test suite as a differential oracle. Convex decomposition stays a library. |
| R-06 | v1 contact scope: primitives and convex hulls, soft contact with implicit integration. Mesh contact is a later tier. |
| R-07 | Engine lives in this repo under its own CMake tree. |
| R-08 | Core runtime is one compiled library: engine, co-simulation kernel, signal-layer built-in components, and both controller runtimes (native shim, Renode bridge). One Python binding exposes it. |
| R-09 | Tiered component plugins: built-in types in C++; user-added types in Python first through a per-tick slow path; promoted to C++ when stable. Both paths implement the same interface and pass the same reference tests. |
| R-10 | Builder emits an engine-neutral physics model (bodies, joints, constraints, contacts, lumped members, transmissions). Each adapter translates it. Supersedes the earlier ruling that the builder emits MJCF. |
| R-11 | A thin MuJoCo adapter carries milestones 1 and 2 and serves as the differential oracle for the own engine on whole-design runs. It is retired once the own engine passes the analytical suite and matches the gantry baselines; the comparison recordings are kept as fixed baselines. |
| R-12 | Calibration spike before phase 1: engine packages 1 (math core, integrators) and 2 (rigid multibody, six joint types, loops) plus the pendulum and loaded-beam references, run through the real agent pipeline, logging tokens, briefs and defects per package. Produces the project's measured cost model. |
| R-13 | Lumped domain models are ours under every option: transmissions (belt, lead screw, gears), stepper and driver tiers, rope as a lumped chain, flexible members as multi-segment lumped beams with stress recovery and plasticity. |

## Application layer

| # | Ruling |
|---|---|
| R-14 | Python for everything outside the core runtime: component catalog, materials, design loading, CAD import, builder, test runner orchestration, mechanical integrity, scoring, exports, facade service. |
| R-15 | Headless core (invariant 9): no Qt or display dependency outside the viewer; one serializable facade; replay reads archives; recorder streaming seam present from milestone 1. CLI is the first client and the one CI uses. |
| R-16 | Front end: browser client over a local Python service for early versions, replaceable by a standalone app. The facade is a versioned protocol with a schema and a client-independent conformance suite; clients hold no domain logic; streaming and decimation live in the service. Protocol chosen by ADR in phase 1. |
| R-17 | Engine debug viewer: minimal immediate-mode C++ window for engine development. Replaces reliance on MuJoCo's built-in viewer. |
| R-18 | Spec first: one spec per module in `docs/spec/` with numbered requirements traced to the source docs; every requirement has a test stub committed as a strict expected failure before code; a traceability test fails on any requirement without a test and on any test citing an unknown requirement; tolerances live in the spec. Spec maintenance: every behavior change cites requirement IDs and carries its spec edit in the same commit; the traceability matrix is regenerated in CI; stage review rejects a diff whose behavior is not covered by the spec as edited. |

## Mechanical integrity

| # | Ruling |
|---|---|
| R-19 | Checks are plugins like component types: data-driven, tiered, each shipped with an analytical reference. Module scope is mechanical integrity, not structural analysis. |
| R-20 | v1 check set: strength, deflection, buckling, fatigue, fastener, impact, resonance screening in milestone 3. Bearing life and build-time design rules (support spacing, cantilever ratio, clearance, misalignment binding, tip-over) in milestone 4. Thermal and wear as seams. |
| R-21 | Mid-run failure. Tier 1: a kernel party evaluates member stress, buckling margin, fastener load and deflection every tick from live loads; limit crossing raises a timestamped event, fails the case, optionally halts. Tier 2: failure changes the model (yield sets a permanent spring offset, buckling collapses stiffness, fastener break disables its constraint, rope snap drops the load). Tier 1 and multi-segment beams in milestone 3; tier 2 across milestones 3 and 4. Live member end-loads are a physics interface requirement. |

## Tooling (tabled 2026-11-02, accepted as a block)

| # | Ruling |
|---|---|
| L1 | Python 3.12 floor, pinned for the toolchain; newer only when every dependency wheel ships. |
| L2 | uv, PEP 621 `pyproject.toml`, committed lock file. |
| L3 | ruff for lint and format; mypy strict across `mechsim/`, per-module relaxation only for missing stubs. |
| L4 | pydantic v2 models are the source; JSON Schemas in `schemas/` are generated and checked in with a staleness test. |
| L5 | Quantities in files as unit strings (`"2.5 kg"`), parsed by pint at load; plain SI floats inside; pint never in the tick loop. |
| L6 | ruamel.yaml round-trip mode. |
| L7 | numpy and scipy for analysis-side numerics. |
| L8 | HDF5 run archives via h5py; design file, test case, seed, mode and git hashes as attributes. |
| L9 | argparse CLI: validate, run, suite, replay. |
| L10 | Standard-library logging with a structured formatter. |
| P2 | Integrator choice is per adapter; the own engine's integrator is an engine ADR. (MuJoCo adapter: implicitfast at 1 ms.) |
| P3 | Belts, lead screws and ropes are our transmission and lumped models in the neutral form; adapters map them. |
| P4 | Rope model detail deferred to a milestone 3 ADR; milestone 1 pendulum is a rigid link. |
| C1 | C11 for firmware application code; C++ only inside a module behind a C interface; the board interface is a C header. |
| C2 | CMake with host and arm-none-eabi presets, identical warning flags, warnings as errors; host preset fails on any include outside the board interface. |
| C3 | Native tick contract: init and tick entry points at a fixed rate; inputs and outputs cross as a plain C struct; native mode is polled (accumulated counts, latched levels, no interrupts). Interrupt semantics only in emulated mode. |
| C4 | 1 kHz control tick as the gantry default, equal to the physics step. Kernel supports unequal rates. |
| C5 | Vendor HAL allowed only under `firmware/board/<chip>/`. |
| C6 | Renode transport (External Control socket vs pyrenode3) deferred to a milestone 4 ADR. The C client library favors direct use from the core runtime. |
| D1 | One integer seed per run; SeedSequence spawns a child generator per module instance in fixed order. |
| D2 | Dotted signal names `instance.signal`, unique per design; operator messages use the requirements doc labels verbatim. |
| D3 | Design version: semver string plus the file's git blob hash recorded in every archive. |
| D4 | Regression baselines as YAML scoring summaries in git; archives never in git. |
| T1 | Four test tiers: unit, analytical reference, acceptance (G1 to G16 plus pendulum), invariant (import boundaries, schema validity of every shipped YAML, run-twice determinism). |
| T2 | Spec format per R-18. |
| T3 | Stubs as strict expected failures before code. |
| T4 | Tolerances stated per test in the spec; changing one is a reviewed spec edit. |
| R1 | CI on Ubuntu for lint, types and all tiers, headless; Windows joins the matrix when milestone 1 closes. |
| R2 | Round branch per phase or milestone off main; merge commits only; cut fresh after each PR. |

## Process

| # | Ruling |
|---|---|
| W-01 | Agents live in `.claude/agents/` of this repo, all six from the start: module-writer (Opus), schema-author (Opus), invariant-auditor (Sonnet; Opus at gates), reference-validator (Sonnet), firmware-builder (Sonnet; Opus for the first shim), docs-indexer (Haiku). Briefs may override. |
| W-02 | Review: Opus writes. Fable reviews every stage commit from the spec and the diff only, never the writer's summary, with the invariant auditor alongside. Opus or better on contracts, ADRs and gate reports. Owner reviews at gates (one to two hours, daytime) and audits one stage commit in five. Oracle tests are the merge gate. Requires a scoped amendment to the global approval rule in claude-config. |
| W-03 | Checkpoint: pushed stage commit, CI green, cost log current, `docs/queue.md` updated with the next brief written. "Pause at next checkpoint" finishes in-flight briefs only. |
| W-04 | Usage reserve: dispatch stops at 90 percent of the weekly allowance. Owner checks usage and says pause, or supplies the allowance so the cost log can enforce it. |
| W-05 | One coordinator session per round; state lives in `docs/queue.md`, the plan, `docs/cost-log.md` and the open-questions list. |
| W-06 | Phase sequence: 0a rulings and ADRs; 0c scaffolding, agents, hooks and CI (the spike needs the engine tree and sanitizer CI); 0b spec and stubs with the calibration spike alongside; phase 1 contracts, schemas, facade protocol; milestone 1 on the MuJoCo adapter with the core runtime and engine in parallel. No application or engine code before phase 1 other than the spike. |
| W-07 | Cost log at `docs/cost-log.md` with `docs/cost-log-archive.md`; never in any `CLAUDE.md`. Already applied to claude-config. |

## Still open, owed by the gantry team (parameters until ruled)

- STM32 part, after the peripheral smoke test
- Encoder placement
- Flexible members, sections, static or dynamic 1 cm target
- Workspace dimensions and rated load
- Swing angle and settle time limits for G12; minimum safety factor
- New-target-during-move and lift-while-moving policies

## Deferred by ruling

- Facade protocol (phase 1 ADR)
- Own-engine integrator (engine ADR)
- Rope model (milestone 3 ADR)
- Renode transport (milestone 4 ADR)
