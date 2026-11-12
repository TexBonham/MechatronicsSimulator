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
impulse, force, errors; `contacts`; `payloads`), clears inputs.
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
Load releases (constraint, rope or payload weld, requested or by snap) go through one `release_load` path:
`LoadReleased` + source.

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
Weld is milestone 3 scope: becomes a six-row fixed loop constraint with disable (ruling 23). Attach and mass
change push `PayloadAttached` / `PayloadMass` into the next step's `outputs().events`; release (attached false) goes
through `release_load(Payload, body)`: `LoadReleased`, index the carrier left; no held weld throws.

## Lumped beam

N segments, spring E I / l at each segment midpoint. `BeamEnds`: Cantilever (clamped root, free end),
SimplySupported (pin segment at the root, `pinned_root`; point constraint onto a prismatic roller at x = L),
Clamped (ruling 16; clamped root, and the roller constraint becomes a Hinge about the beam x axis: point plus two
rotation rows, so the far stub moves with the roller; the axial slide stays free). Clamped-clamped midspan chain value
P L^3 / (192 E I) (1 - 4 / N^2): -6.25 % at N = 8, so the 2 % references run at N = 16 (-1.56 %); first mode
+2.49 % at N = 8, +0.61 % at N = 16 (header derivation). `ConstraintOutput::torque` (world, on body_a) gives the
clamped end moment in `MemberOutput::end_moment`.
Euler buckling (ENG-042): N = 8 critical load (2N / L)^2 E I sin^2(pi / 2N) = 0.9872 P_Euler.
Planar bending about root z; two-axis bending plus torsion: next tier. Derivation in the header.

## Tests

Unit: `mechsim_engine_tests`. Reference: `mechsim_engine_reference`, `ctest -L reference`. Contact helpers:
`tests/contact_helpers.hpp`.

## Conventions

SI, units per field (ENG-004) No threads, unordered containers or fast-math in the step path (ENG-023).
