# ADR 0003: Engine-neutral physics model behind adapters

Date: 2026-11-02
Status: Accepted
Closes rulings: R-10, P2, P3, P4
Supersedes: The earlier ruling that the builder emits MJCF; architecture doc Repository starting point, `builder/`
("design file -> engine models") and `physics/` adapter descriptions

## Context
The architecture doc's invariant 5 keeps engines behind adapters, but the builder previously targeted MuJoCo's model
format. With two engines live at once (the MuJoCo bridge, ADR 0004, and the own engine, ADR 0001), the builder needs one
output that both consume, and the lumped domain models (R-13) must not belong to either engine.

## Decision
The builder emits an engine-neutral physics model. Each adapter translates it into its engine.

| Neutral model element | Notes |
|---|---|
| Bodies | Rigid bodies with mass properties |
| Joints | The six joint types of engine package 2 (ADR 0001) |
| Constraints | Including kinematic loops |
| Contacts | Scope per ADR 0002 |
| Lumped members | Our models (ADR 0001, R-13) |
| Transmissions | Belts, lead screws, gears; our models (P3) |

| # | Ruling |
|---|---|
| P2 | Integrator choice is per adapter. The MuJoCo adapter uses implicitfast at 1 ms. The own engine's integrator is an engine ADR. |
| P3 | Belts, lead screws and ropes are our transmission and lumped models, expressed in the neutral form; adapters map them onto their engine. |
| P4 | Rope model detail is deferred to a milestone 3 ADR. The milestone 1 pendulum is a rigid link. |

Adapters live under `mechsim/physics/<name>/`: `mujoco/` (the bridge) and `native/` (the own engine's binding).

## Consequences
- Adding an engine means an adapter plus its translation of the neutral model; no other module changes.
- No engine type crosses the neutral model or the adapter boundary (invariant 5 now has a concrete boundary object).
- Differential comparison between engines is meaningful because both run the same neutral model.
- An element an adapter cannot map ends the build with a report (invariant 7), not a silent approximation.
- Every adapter must provide the live member end-loads required by ADR 0010.
- Milestone 1: the pendulum (R-03) is a rigid link; no rope element is required before milestone 3.

## Open items
- Rope model: milestone 3 ADR.
- Own-engine integrator: engine ADR (ADR 0001).
- The neutral model's schema: phase 1 contracts and schemas.
