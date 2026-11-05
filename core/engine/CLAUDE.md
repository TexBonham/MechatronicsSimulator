# core/engine: contract

Namespace `mechsim::engine`; q (nq), v (nv). Eigen only, dense (ENG-001).

## Headers (`include/mechsim/engine/`)

| Header | Holds |
|---|---|
| `step_config.hpp` | `StepConfig { h = 1e-3 s; symmetry_tolerance = 1e-12 }` (ENG-002, ENG-049) |
| `state.hpp` | `State { q; v; t }` |
| `dynamics_provider.hpp` | `DynamicsTerms { mass, bias, force, stiffness, damping }`, abstract `DynamicsProvider` |
| `integrator.hpp` | `Integrator`, `SemiImplicitEuler` (ADR 0013 A, ENG-003); ctor throws `InvalidConfigError` |
| `step_health.hpp` | `StepHealth { max_constraint_drift, converged, iterations, time, step }` (ENG-026) |
| `engine_error.hpp` | `EngineError(time, cause)`; `NonFiniteStateError`, `SolverNonConvergenceError`, `DriftError` (+`constraint_index()`), `InvalidModelError`, `InvalidConfigError` |
| `spatial.hpp` | Featherstone 6-vectors (angular first), `Transform` B_X_A (`motion`, `force`, `inverse_*`, `then_from`), `inertia`, `cross_motion/force` |
| `multibody_model.hpp` | `JointType`, `Joint`, `Body`, `LoopConstraint` (Point 3, Hinge 5 rows), `MultibodyModel`, `MultibodyConfig` |
| `multibody_provider.hpp` | `MultibodyProvider`, output structs |
| `transmission.hpp` | `Transmission` (belt, screw, gear), `TransmissionOutput`, `PowerFlow`, `couple` (derivation) |
| `lumped_beam.hpp` | `BeamSpec`, `add_lumped_beam(model, spec, parent, position, orientation)`, `beam_point(beam, x)` (ENG-022) |

## DynamicsProvider hooks

`position_size`, `velocity_size`, `evaluate(t, q, v, terms)`, `integrate_positions(q, v, h)` (pure);
defaulted: `solve_constraints(t, h, q, system, v) -> iterations` (0), `project(q, v) -> drift` (0),
`project_positions(t, q, v)` (calls `project`).

## Step order

1. `evaluate` at (q, v); H finite (else NonFinite), `|H - H^T| <= tol max|H|`, LLT of `selfadjointView<Lower>`.
2. `(H + hD + h^2 K) dv = h (f - c) - h^2 K v`; `v += dv`.
3. `solve_constraints`: impulses at q so J(q) v = 0; `health.iterations`.
4. `q = integrate_positions(q, v, h)`; 5. `project_positions` -> drift; 6. commit, `t += h`. Throw leaves state as it was.

## MultibodyProvider

`MultibodyProvider(MultibodyModel, MultibodyConfig = {})` validates both (InvalidModel/ConfigError, t 0).
H by CRBA (lower triangle mirrored), c by RNEA (gravity as base acceleration).
Joint springs (ENG-051): `Joint.stiffness/damping/rest` per v DOF (empty = none); diagonal K, D; f gets
-K(q - rest) - D v; stiffness only on Revolute, Prismatic, Planar.
Joints: Revolute 1/1, Prismatic 1/1, Fixed 0/0, Planar 3/3 [x, y, theta], Spherical 4/3, Free 7/6 [p, quat w-first];
spherical/free v in child coordinates; quaternions renormalized in `integrate_positions`.
Constraints: block Gauss-Seidel (per-constraint pseudo-inverse of J A^-1 J^T) until `max|J v| <= velocity_tolerance`,
cap `iteration_cap` (50) else `SolverNonConvergenceError`; then `projection_iterations` (3) Gauss-Newton
min-norm steps; drift > `drift_threshold_m/_rad` (1e-6) throws `DriftError(t, cause, index)`.
`step(integrator, state)`: steps, records `outputs()` (body pose and twists; joint `S^T f` and wrench; constraint
impulse, force, errors; empty `contacts`), clears inputs.
Inputs: `set_generalized_forces(tau)`, `apply_body_wrench(body, torque, force)` world-aligned at the com.
Also `constraint_errors(q)`, `body_states(q, v)`, `neutral_positions()`, `*_offset(b)`.
Hooks write mutable `impulse_`/`error_`; single-threaded (ENG-023).
Transmissions: Revolute shaft to Prismatic or (gear) Revolute output; K, D += k, c a b^T in contact.
`set_shaft_torque(i, tau)` (per step), `set_slip(i, s)` (persists), `slip(i)`, `transmission_states(q, v)`.
`outputs().transmissions`, `transmission_states`: end state, step's drive (next step's pre-image); `shaft_load_torque`
= g F, the torque the shaft must supply along its + axis (PHY-033; `couple()` keeps the applied sign -g F internally);
`power_flow` not for per-step energy bookkeeping. Slip offsets move only in `step()`.

## Section index, `src/multibody_provider.cpp`

| Section | Line |
|---|---|
| Helpers | 39 |
| Construction | 262 |
| Kinematics | 338 |
| Dynamics | 405 |
| Transmissions | 509 |
| Constraints | 607 |
| Inputs and outputs | 750 |

## Lumped beam

N segments, spring E I / l at each segment midpoint; clamped or pinned root, free or roller end.
Planar bending about root z; two-axis bending plus torsion: next tier. Derivation in the header.

## Tests

Unit: `mechsim_engine_tests`. Reference: `mechsim_engine_reference`, `ctest -L reference`.

## Conventions

SI, units per field (ENG-004) No threads, unordered containers or fast-math in the step path (ENG-023).
