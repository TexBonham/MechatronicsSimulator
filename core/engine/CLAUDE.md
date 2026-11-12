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
| `lumped_beam.hpp` | `BeamSpec`, `add_lumped_beam(model, spec, parent, position, orientation)` (registers `model.members`), `beam_point(beam, x)` (ENG-022) |
| `member_loads.hpp` | `LumpedBeam`, `member_nodes(beam)`, `MemberOutput` (start/end moment, force), `member_end_loads` (ENG-030) |
| `rope.hpp` | `Rope`, `RopeOutput`, `add_rope(model, rope, from_world, to_world)`, `rope_segment`, `validate_rope_parameters` (ENG-047, ADR 0016) |

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
Joints: Revolute 1/1, Prismatic 1/1, Fixed 0/0, Planar 3/3 [x, y, theta], Spherical 4/3, Free 7/6 [p, quat w-first],
Translation 3/3 [p] (rope nodes);
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

Members (ENG-030): `outputs().members` per `model.members`, support-on-member wrench at x = 0 and x = L, moments about
the end point, member root frame axes, from the step's start state like `joints`.
Mid-run changes (ENG-031 to 034), next step on, each listed in the next step's `outputs().events` (`ModelChange`):
`set_spring_offset(member, node, offset)` (rest = built + offset), `set_spring_stiffness(member, node, k)`,
`disable_constraint(c)` / `release_load(c)` (zero rows; twice throws), `release_rope(r)` (same path); `constraint_enabled`, `rope_attached`.
Ropes (ENG-047, ADR 0016, rulings 14, 15): N segments, N - 1 Translation nodes, end half-masses on the end bodies;
tension-only spring-dampers, slack or taut from the step start; K gets k n n^T + (T / d)(I - n n^T), D gets c n n^T;
top attachment damping and friction (spherical joint torque, in D); `set_rope_rest_length(r, L)` (winch);
`outputs().ropes` per segment tension, max, load end, `snatch_omega_h` = sqrt(EA / (L M)) h (flag above 0.2:
first snatch peak about 15 % low); snap: end-of-step tension
over `breaking_tension` releases `to_body` from the next step, event `RopeSnapped`.
Ruling 21: swing and belt modes go through K, so they integrate as backward Euler: with x = omega h the mode loses
(1 + x^2)^(-pi / atan x) of its amplitude per period (about pi x; 1 % at 1 ms for a 1 m pendulum) and runs x^2 / 3
slow (1 % at 27 Hz at 1 kHz); `RopeOutput::scheme_decay` and `TransmissionOutput::scheme_decay` publish pi omega h
for the adapter to record. Attachment friction regularised below `MultibodyConfig::friction_slip_threshold` (1e-4).
Load releases (constraint or rope, requested or by snap) go through one `release_load` path: `LoadReleased` + source.

## Section index, `src/multibody_provider.cpp`

| Section | Line |
|---|---|
| Helpers | 40 |
| Construction | 266 |
| Kinematics | 343 |
| Dynamics | 415 |
| Transmissions | 521 |
| Constraints | 620 |
| Members, ropes and model changes | 770 |
| Inputs and outputs | 1194 |

## Lumped beam

N segments, spring E I / l at each segment midpoint; clamped or pinned root, free or roller end.
Euler buckling (ENG-042): N = 8 critical load (2N / L)^2 E I sin^2(pi / 2N) = 0.9872 P_Euler.
Planar bending about root z; two-axis bending plus torsion: next tier. Derivation in the header.

## Tests

Unit: `mechsim_engine_tests`. Reference: `mechsim_engine_reference`, `ctest -L reference`.

## Conventions

SI, units per field (ENG-004) No threads, unordered containers or fast-math in the step path (ENG-023).
