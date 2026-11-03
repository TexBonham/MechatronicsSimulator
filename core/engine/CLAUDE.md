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

`MultibodyProvider(MultibodyModel, MultibodyConfig = {})` validates both (InvalidModel/ConfigError, time 0).
H by CRBA (lower triangle mirrored, exact symmetry), c by RNEA (gravity as base acceleration), K = D = 0.
Joints: Revolute 1/1, Prismatic 1/1, Fixed 0/0, Planar 3/3 [x, y, theta], Spherical 4/3, Free 7/6 [p, quat w-first];
spherical/free v in child coordinates; quaternions renormalized in `integrate_positions`.
Constraints: block Gauss-Seidel (per-constraint pseudo-inverse of J A^-1 J^T) until `max|J v| <= velocity_tolerance`,
cap `iteration_cap` (50) else `SolverNonConvergenceError`; then `projection_iterations` (3) Gauss-Newton
min-norm steps; drift > `drift_threshold_m/_rad` (1e-6) throws `DriftError(t, cause, index)`.
`step(integrator, state)`: steps, records `outputs()` (bodies: pose, Pluecker world twist, origin velocity; joints:
`S^T f`, parent-on-child wrench; constraints: impulse, force, errors; `contacts` empty slot), clears inputs.
Inputs: `set_generalized_forces(tau)`, `apply_body_wrench(body, torque, force)` world-aligned at the com.
Also `constraint_errors(q)`, `body_states(q, v)`, `neutral_positions()`, `position/velocity_offset(body)`.
Hooks write `impulse_`/`error_` (mutable); single-threaded (ENG-023).

## Section index, `src/multibody_provider.cpp`

| Section | Line |
|---|---|
| Helpers | 38 |
| Construction | 215 |
| Kinematics | 269 |
| Dynamics | 336 |
| Constraints | 473 |
| Inputs and outputs | 616 |

## Conventions

SI everywhere, units per field (ENG-004). No threads, unordered containers or fast-math in the step path (ENG-023).
