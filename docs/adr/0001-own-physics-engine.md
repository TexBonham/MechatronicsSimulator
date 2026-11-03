# ADR 0001: Own physics engine in C++

Date: 2026-11-02
Status: Accepted
Closes rulings: R-04, R-07, R-12, R-13
Supersedes: Architecture doc, Technology recommendation, "Physics engine" paragraph and verdict (MuJoCo as the v1
engine); Decisions log row 2 (build on an existing physics engine), for the physics engine only

## Context
The architecture doc's Technology recommendation chose MuJoCo as the v1 engine behind an adapter, with Chrono as a
fallback for beam elements. The 2026-11-02 rulings make the product a standalone general simulator (R-01), which needs
control over contact, lumped members, live member loads (R-21) and determinism that a third-party engine does not give
on our terms. The requirements doc's milestone plan (milestones 1 to 5) still has to be carried while the engine is built,
which ADR 0004 addresses.

## Decision

| Aspect | Decision |
|---|---|
| Engine | Own physics engine written in C++20 |
| Linear algebra | Eigen |
| Python binding | pybind11 (one binding for the whole core runtime, ADR 0005) |
| Build | CMake; the engine is its own CMake tree in this repo, under `core/engine/` |
| CI gates from the first commit | AddressSanitizer, UndefinedBehaviorSanitizer, clang-tidy, warnings as errors |
| Ownership rule | No raw owning pointers across any API |
| Integrator | Chosen by a separate engine ADR (see ADR 0003, P2) |

Lumped domain models are ours whichever engine runs them (R-13):

| Domain | Model |
|---|---|
| Transmissions | Belt, lead screw, gears |
| Stepper and driver | Our fidelity tiers (behavioral first, per Decisions log row 13) |
| Rope | Lumped chain |
| Flexible members | Multi-segment lumped beams with stress recovery and plasticity |

Calibration spike before phase 1 (R-12):

| Item | Content |
|---|---|
| Engine package 1 | Math core, integrators |
| Engine package 2 | Rigid multibody, six joint types, kinematic loops |
| References | Pendulum and loaded beam |
| Method | Run through the real agent pipeline, logging tokens, briefs and defects per package |
| Output | The project's measured cost model |

## Consequences
- The engine can expose exactly the interfaces the rest of the tool needs, including live member end-loads every tick
  (ADR 0010) and mid-run model changes.
- Contact, joints, loops and integration become our code to write, test and maintain; the analytical reference tier
  (ADR 0008) and the MuJoCo differential oracle (ADR 0004) are what keep it honest.
- The sanitizer, clang-tidy and warnings-as-errors gates must exist in CI before the first engine commit; the spike
  depends on them.
- The lumped models are engine-independent data in the neutral model (ADR 0003), so the MuJoCo adapter maps the same
  transmissions, ropes and beams the own engine runs.
- The spike's measured cost model supersedes default dispatch costs for later phases.
- Milestone placement: packages 1 and 2 run in the spike before phase 1; the engine is built in parallel with milestone 1,
  which runs on the MuJoCo adapter (ADR 0004).

## Open items
- Own-engine integrator: engine ADR, before engine package 1 is briefed.
- Engine package breakdown beyond packages 1 and 2: after the spike, using its cost model.
- Rope model detail: milestone 3 ADR (ADR 0003, P4).
