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
| `member_loads.hpp` | `LumpedBeam` (+ `carried`), `member_nodes(beam)`, `MemberOutput` (start/end moment, force, `carried_deflection`, work and validity fields), `carried_ratio_limit`, `member_end_loads` (ENG-030); `CarriedLoadOutput`, `load_nodes`, `carried_pair` (ENG-052, derivation in the header) |
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
Joint limits (ENG-013, ruling 43): `Joint.lower/upper` (empty or 1 entry; Revolute, Prismatic only; -inf/+inf open;
lower == upper locks; else InvalidModelError naming the body). Every row J = s e_k (s +1 lower, -1 upper), impulse >= 0,
anticipatory s v >= -max(gap, 0) / h (gap at step start), after the loop rows in each sweep, joint order; a binding row sets
v_k to its target exactly; no position projection for limits. Start coordinate past a bound by more than the drift
threshold: InvalidModelError. `JointOutput.limit_force` = s lambda / h; `limit_overshoot` round-off pass.
Joint friction (ENG-014, ruling 38): `Joint.friction` per DOF (Revolute, Prismatic, Planar), `add_joint_friction`:
c = F / max(|v_start|, `friction_slip_threshold`), f -= c v, D += c; creep v_s L / F published as `friction_creep`.
Drive (ENG-051, ruling 42): `set_joint_drive(joint, dof, rest, stiffness, damping)` stores `drive_*_` per DOF, added
on top of the model spring by `add_joint_drives` (rest = end-of-step rest), persistent, no event;
`clear_joint_drive(joint, dof)`. `JointOutput.scheme_decay` only on joints with a drive (ruling 44,
`drive_omega_squared`: single DOF or a transmission pair's lower root); no mass-matrix work on undriven models.
Constraints: block Gauss-Seidel (per-constraint pseudo-inverse of J A^-1 J^T) until `max|J v| <= velocity_tolerance`,
cap `iteration_cap` (50) else `SolverNonConvergenceError`; then `projection_iterations` (3) Gauss-Newton
min-norm steps; drift > `drift_threshold_m/_rad` (1e-6) throws `DriftError(t, cause, index)`.
`step(integrator, state)`: steps, records `outputs()` (body pose and twists; joint `S^T f` and wrench; constraint
impulse, force, errors; `contacts`; `payloads`), clears inputs.
Inputs: `set_generalized_forces(tau)`, `apply_body_wrench(body, torque, force)` world-aligned at the com.
Also `constraint_errors(q)`, `body_states(q, v)`, `neutral_positions()`, `*_offset(b)`.
Hooks write mutable `impulse_`/`error_`; single-threaded (ENG-023).
Transmissions: Revolute shaft to Prismatic or (gear) Revolute output; K, D += k, c a b^T in contact, a = [g_k, -1].
Efficiency (ruling 47): gain n (1 + s_w alpha), s_w = min(|omega| / omega_s, 1), omega_s = `friction_slip_threshold`;
explicit gain exact n / eta_f, n eta_b at and above omega_s; g_k = n always and the shaft gets implicit damping
c = |alpha| n |F| / max(|omega|, omega_s) (larger loss at omega = 0), so the loss never adds energy (M3-E6). Static sag n F / K exact; creep tau_f = (n F (1 / eta_f - 1) /
omega_s + D_d) / K, tau_b = (n F (1 - eta_b) / omega_s + D_d) / K. `Static` = shaft at exactly zero speed (or F = 0).
Self-locking screw (ruling 48): `lock_rows_` from `evaluate` (transmission order), scalar row s omega >= 0 after the
limit rows in `solve_constraints`, absent when the shaft's own torque T (tau, body wrenches, drive and model joint
spring-damper) has T L > 0 (lowering: gain 0); two-sided only at omega == 0 with F = 0 and T = 0 (ruling 55); `TransmissionOutput::lock_torque` = s lambda / h; Locked when P >= 0 and T L >= 0
(published load torque 0). `TransmissionCoupling`: `gain`, `stiffness_gain`, `shaft_damping`, `shaft_torque`, `lock`.
`set_shaft_torque(i, tau)` (per step), `set_slip(i, s)` (persists), `slip(i)`, `transmission_states(q, v)`.
`outputs().transmissions`, `transmission_states`: end state, step's drive (next step's pre-image); `shaft_load_torque`
= g F, the torque the shaft must supply along its + axis (PHY-033; `couple()` keeps the applied sign -g F internally);
`power_flow` not for per-step energy bookkeeping. Slip offsets move only in `step()`.

Members (ENG-030): `outputs().members` per `model.members`, support-on-member wrench at x = 0 and x = L, moments about
the end point, member root frame axes, from the step's start state like `joints`.
Carried joints (ENG-052, ruling 63): `LumpedBeam::carried` child bodies of the member's parent (validated at
construction: in range, not a member body or roller, one member each). Load = -(previous step's `reaction_wrench`), the
first step's from Newton-Euler at the start state with zero accelerations (`static_carried_loads`). Rerouted: only F . y
and M_p . z (member frame, ruling 65); axial, out-of-plane and torsion stay on the parent. Lever rule onto `load_nodes`
(x = 0: parent if clamped, pin segment if pinned; spring nodes; x = L: last segment tip), exact opposite on the parent
at the same points, added in `applied_body_wrenches` (so `evaluate`, joint reactions and end loads see it). Explicit,
one step lagged, not in K, D or H. `outputs().carried` per carried joint; `MemberOutput::carried_deflection` per member;
`carried_generalized_forces(q)` for the energy oracle. Foot point off [0, L] by more than `drift_threshold_m`: InvalidModelError.
Lag stability (brief M3-E7b, header Stability): at the first step, modes of K, M on the loop null space; per joint
`CarriedLoadOutput::inverse_inertia`, `lag_ratio`, `lag_omega_h`, `lag_factor` (damped spectral radius,
`lag_spectral_radius`; one mode `lag_growth_factor`, boundary `lag_stability_boundary` = (1 + x^2) / 2); undamped
radius > 1 refused (InvalidModelError, ruling 67); `MultibodyConfig::refuse_unstable_carried` is a test seam only.
Work and validity (brief M3-E7c, ruling 69, header Work and validity): per carrying member `MemberOutput::carried_work`
(cumulative h Q_n . v_{n+1}), `carried_force`/`peak_carried_force`, `quasi_static_deflection`/`peak_quasi_static`
(node compliances and gravity deflection from the elastic modes at load, `member_compliance_`), `dynamic_ratio`,
`peak_ratio`, `unresolved` + `unresolved_time` above `carried_ratio_limit` = 2 (1 + `carried_validity_margin` 0.02);
information only, nothing clamped.
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
`LoadReleased` + source. `outputs().events` is overwritten every step: a caller running several steps per tick
collects the events after each step.

## Section index, `src/multibody_provider.cpp`

| Section | Line |
|---|---|
| Helpers | 41 |
| Construction | 302 |
| Kinematics | 397 |
| Dynamics | 469 |
| Transmissions | 704 |
| Constraints | 856 |
| Members, ropes and model changes | 1092 |
| Inputs and outputs | 1722 |

## Section index, `src/collision.cpp`

| Section | Line |
|---|---|
| Shapes | 16 |
| Broad phase | 180 |
| Closed forms | 243 |
| GJK | 415 |
| EPA | 583 |
| Narrow phase | 744 |

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
clamped end moment in `MemberOutput::end_moment`. Clamped needs N >= `BeamSpec::min_clamped_segments` (8, ruling 31),
else InvalidModelError; twist and axial slide at the clamped end stay free until ruling 23's six-row constraint (M4).
Euler buckling (ENG-042): N = 8 critical load (2N / L)^2 E I sin^2(pi / 2N) = 0.9872 P_Euler.
Planar bending about root z; two-axis bending plus torsion: next tier. Derivation in the header.

## Tests

Unit: `mechsim_engine_tests`. Reference: `mechsim_engine_reference`, `ctest -L reference`. Contact helpers:
`tests/contact_helpers.hpp`; energy oracle `tests/energy_helpers.hpp` (`mechanical_energy`); carried rigs and the
lever-rule chain value `tests/carried_helpers.hpp`.

## Conventions

SI, units per field (ENG-004) No threads, unordered containers or fast-math in the step path (ENG-023).
