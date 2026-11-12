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
| `collision.hpp` | `ShapeKind`, `shape_kind(name)` (unknown throws, ENG-017), `Shape`, `Pose`, `ContactPoint{point, normal a->b, depth}`, `Aabb`, `bounds`, `broad_phase`, `collide(a, pa, b, pb)`, `support` |
| `contact.hpp` | `ContactParameters{K_n, C_n, mu}`, `MaterialPair`, `Geometry` (neutral sizes, body -1 world), `ContactModel{geometries, default, pairs, exclusions, slip_velocity 1e-4}`, `contact_force`, `WeldAttachment`, `PayloadOutput` (derivations in the header) |
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
impulse, force, errors; `contacts`; `payloads`), clears inputs.
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
| Transmissions | 631 |
| Constraints | 729 |
| Inputs and outputs | 872 |

## Section index, `src/collision.cpp`

| Section | Line |
|---|---|
| Shapes | 16 |
| Broad phase | 183 |
| Closed forms | 246 |
| GJK | 417 |
| EPA | 582 |
| Narrow phase | 734 |

## Collision and contact (ADR 0002, ENG-016 to ENG-021)

Broad phase: all-pairs AABB in geometry order (plane unbounded). Narrow phase in-house, no library (ENG-018):
closed forms for sphere-sphere, sphere-capsule, capsule-capsule, sphere-box and every shape against a plane (one
contact per penetrating feature: box corners, hull vertices, capsule caps, 4 rim points per cylinder cap); GJK + EPA
(`CollisionLimits`: tolerance 1e-12 m, GJK cap 64, EPA cap 128) for the rest, one contact. Point = midpoint of the
witness points. Box-box and hull-hull carry one witness point in milestone 3 (a box on a box tips; payload-on-floor
uses the per-corner plane form), ruling 23. Empty = no contact only for a geometric answer (separated, touching,
GJK separating within tolerance, flat Minkowski difference); GJK/EPA cap exhaustion or a degenerate EPA face before tolerance throws
`SolverNonConvergenceError` (rethrown by `evaluate` with the step time).
Provider: `set_contact_model(ContactModel)` validates (planes only on the world); `evaluate` adds per contact
G^T F, G^T K_s G, G^T D_s G (G = J_b(p) - J_a(p)); same-body and excluded pairs skipped; `contacts(q, v)`;
`outputs().contacts` at the step's start state, force on body a; joint outputs include contact and weld loads.
Payload (ENG-035): `set_body_mass(b, m)` (inertia scaled), `set_attachment(body, carrier, attached, q, f = 1 kHz)`
weld as implicit 6-DOF spring-damper at the payload com captured from q; `attachments()`, `outputs().payloads`.
Weld is milestone 3 scope: becomes a six-row fixed loop constraint with disable (ruling 23). Attach, release and
mass change push `ModelChangeEvent{PayloadAttached|PayloadReleased|PayloadMass}` into the next step's
`outputs().events` (E2's event shape; release unifies with `release_load` at the merge).

## Lumped beam

N segments, spring E I / l at each segment midpoint; clamped or pinned root, free or roller end.
Planar bending about root z; two-axis bending plus torsion: next tier. Derivation in the header.

## Tests

Unit: `mechsim_engine_tests`. Reference: `mechsim_engine_reference`, `ctest -L reference`. Contact helpers:
`tests/contact_helpers.hpp`.

## Conventions

SI, units per field (ENG-004) No threads, unordered containers or fast-math in the step path (ENG-023).
