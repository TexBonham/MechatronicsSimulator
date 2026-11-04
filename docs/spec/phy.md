# Spec: Physics adapters (PHY)

## Purpose

The physics adapter package (`mechsim/physics/`) defines the interface every engine implements and holds one adapter
per engine: `mujoco/`, the bridge that carries milestones 1 and 2, and `native/`, the binding of the own engine. An
adapter translates the engine-neutral physics model into its engine, steps it and reports state, loads and solver
health, and no engine type crosses its boundary. The engine itself is ENG and time ownership is KER.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| BLD | PHY-001, PHY-012, PHY-019 | BLD-001, BLD-024, BLD-026, BLD-030 |
| ENG | PHY-018, PHY-020, PHY-021, PHY-023 | ENG-022, ENG-043, ENG-044, ENG-045, ENG-046, ENG-047, ENG-048 |
| KER | PHY-002, PHY-005, PHY-007, PHY-009, PHY-010 | KER-004, KER-009, KER-010, KER-016, KER-018 |
| REC | PHY-003, PHY-004, PHY-007, PHY-025 | REC-002, REC-003, REC-004, REC-026 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| PHY-001 | Every adapter shall implement one interface whose load operation takes an engine-neutral physics model. | ADR 0003; ruling R-10 | M1 | unit | n/a |
| PHY-002 | The adapter interface shall advance the loaded model by one physics step per call. | Architecture doc, Modules: core, Physics engine (adapter) row; ADR 0003 | M1 | unit | n/a |
| PHY-003 | The adapter interface shall report body poses, velocities, joint forces and contact forces after every step. | Architecture doc, Modules: core, Physics engine (adapter) row | M1 | unit | n/a |
| PHY-004 | The adapter interface shall report rope tension after every step for every rope in the model. | Architecture doc, Modules: core, Physics engine (adapter) row | M3 | unit | n/a |
| PHY-005 | The adapter interface shall accept actuator torques and forces from the signal layer before every step. | Architecture doc, Modules: core, Physics engine (adapter) row | M1 | unit | n/a |
| PHY-006 | The adapter interface shall accept injected physical faults: transmission slip and payload change. | Architecture doc, Modules: core, Physics engine (adapter) row; motor stall belongs to the signal layer (SIG-034) | M2 | unit | n/a |
| PHY-007 | Every adapter that carries milestone 3 or later shall publish live member end-loads every tick. | ADR 0010, Interface requirement and Consequences; ADR 0003, Consequences | M3 | unit | n/a |
| PHY-008 | The adapter interface shall accept model changes mid-run: spring offsets, stiffness changes, constraint removal and load release. | ADR 0010, Consequences | M3 | unit | n/a |
| PHY-009 | An adapter shall report constraint drift beyond its threshold and solver failure as run errors that end the run with a report, never as returned numbers. | Requirements doc, Dynamics (solver health); Architecture doc, Repository starting point, invariant 7 | M1 | unit | not set (owed by owner) |
| PHY-010 | The same neutral model, inputs and seed run twice on the same adapter shall give identical outputs at every step. | Architecture doc, Repository starting point, invariant 4; Requirements doc, Dynamics (deterministic runs) | M1 | invariant | n/a |
| PHY-011 | No MuJoCo or other engine type shall be imported or referenced outside `mechsim/physics/<name>/` (Python, import-boundary checker) and `core/physics/<name>/` (C++, CI include check per ADR 0014). | Architecture doc, Repository starting point, invariant 5; ADR 0003, Consequences | M1 | invariant | n/a |
| PHY-012 | The only model type that crosses into an adapter shall be the engine-neutral physics model, and the only values that cross out shall be neutral types. | Architecture doc, Repository starting point, invariant 5; ADR 0003 | M1 | unit | n/a |
| PHY-013 | The MuJoCo adapter shall translate the neutral model into MJCF. | ADR 0004; ADR 0003; ruling R-11 | M1 | unit | n/a |
| PHY-014 | The MuJoCo stepping adapter in `core/physics/mujoco/` shall implement the core PhysicsAdapter interface over MuJoCo's C API with the implicitfast integrator at the configured step (ADR 0014). | Ruling P2; ADR 0003; ADR 0004 | M1 | unit | n/a |
| PHY-015 | The MuJoCo adapter shall carry the milestone 1 acceptance cases, G1 to G4 and the pendulum. | ADR 0004; ruling R-11; Architecture doc, Repository starting point, milestone 1 paragraph | M1 | acceptance | n/a |
| PHY-016 | The MuJoCo adapter shall carry the milestone 2 acceptance cases, G5 to G11, G13 and G14. | ADR 0004; ruling R-11; Requirements doc, Instructions for the planning session, milestone plan item 2 | M2 | acceptance | n/a |
| PHY-017 | The MuJoCo adapter shall map only elements of the neutral model and shall add no element type the neutral model lacks. | ADR 0004, Consequences (the adapter stays thin) | M1 | unit | n/a |
| PHY-018 | Every adapter shall map the neutral model's transmissions and lumped members onto its engine. | Ruling P3; ADR 0003 | M1 | unit | n/a |
| PHY-019 | An adapter shall declare the neutral elements it can map and shall refuse to load a model containing any other element, ending with a report. | ADR 0003, Consequences; Architecture doc, Repository starting point, invariant 7 | M1 | unit | n/a |
| PHY-020 | The native adapter shall bind the own engine through the core runtime's single Python binding. | ADR 0003; ruling R-08 | roadmap | unit | n/a |
| PHY-021 | Two adapters run on the same neutral model, inputs and seed shall agree on body poses, velocities and forces within the differential tolerance. | ADR 0003, Consequences; ADR 0004 (differential oracle); ruling R-11 | roadmap | reference | not set (owed by owner) |
| PHY-022 | A new adapter shall pass the analytical reference suite before it is accepted. | Architecture doc, How to extend, "A physics engine" row | M1 | reference | n/a |
| PHY-023 | The MuJoCo retirement check shall pass only when the native adapter passes the whole analytical suite. | ADR 0004, Decision table (retirement criteria); ruling R-11 | roadmap | acceptance | n/a |
| PHY-024 | The MuJoCo retirement check shall pass only when the native adapter matches the reference-design baselines within the differential tolerance. | ADR 0004, Decision table (retirement criteria); ruling R-11 | roadmap | acceptance | not set (owed by owner) |
| PHY-025 | The comparison recordings used for retirement shall be kept as fixed baselines, stored as YAML scoring summaries in git. | ADR 0004, Decision table and Consequences; ruling D4 | roadmap | unit | n/a |
| PHY-026 | After retirement, no runtime code path shall import MuJoCo. | ADR 0004, Consequences | roadmap | invariant | n/a |
| PHY-027 | Every adapter shall apply the design's gravity, joint friction and joint limits. | Requirements doc, Dynamics (rigid multibody dynamics) | M1 | unit | n/a |
| PHY-028 | A rigid pendulum run on an adapter shall match the closed-form small-angle period. | Architecture doc, Technology recommendation, Validation paragraph; ruling P4 | M1 | reference | 0.1 % of period |
| PHY-029 | A body resting on the floor under gravity shall stay at rest on an adapter, with penetration and drift within tolerance. | Requirements doc, Dynamics (contact, payload resting on the floor); ADR 0002 | M1 | reference | not set (owed by owner) |
| PHY-030 | A body on a prismatic joint with Coulomb friction shall decelerate at the closed-form rate on an adapter. | Requirements doc, Dynamics (friction) | M1 | reference | not set (owed by owner) |
| PHY-031 | Integrator choice and step size shall be per-adapter configuration, and no adapter shall change another adapter's configuration. | Ruling P2; ADR 0003 | M1 | unit | n/a |
| PHY-032 | An adapter shall advance simulated time only on a step call and shall not read wall-clock time. | Architecture doc, Repository starting point, invariant 4; Modules: core, Co-simulation kernel row (kernel owns time) | M1 | unit | n/a |
| PHY-033 | Every adapter shall publish, as each transmission shaft joint's generalized force after every step, the shaft load torque tau_load = tau_applied - J * qacc_shaft: the torque the shaft supplies along its + axis beyond accelerating its own rotor inertia J about the axis (qacc_shaft = qacc_driven / c for a rigid ratio driven = c * shaft). | ENG-048 (stepper reads it); M1-C stage review | M1 | unit | n/a |
| PHY-034 | Every adapter shall apply each joint drive in its physics inputs as a spring-damper on a revolute or prismatic joint toward the drive's rest coordinate, solved implicitly so that no explicit-integration (stiffness-times-step) limit applies, within an envelope of the adapter's own mapping that the adapter states (MuJoCo: K_d A0 h^2 < 400, that is omega h < 20, Open parameters), exact in static sag and steady tracking lag; shall publish each joint's coordinate and, one per drive, the torque or force the drive applied; and shall count the drive torque on a transmission shaft in that shaft's load torque (PHY-033). A disabled drive applies nothing; a drive on a joint the adapter cannot drive, or beyond its envelope, is an adapter error. | Brief M1-X5 coordinator ruling 2 (2026-11-04), for owner confirmation; brief M1-X7 coordinator ruling 5 (2026-11-04); ADR 0010 | M1 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Differential tolerance between adapters (PHY-021, PHY-024) | not set | owner |
| Constraint drift threshold that ends a run (PHY-009) | not set | owner |
| Pendulum period tolerance (PHY-028) | 0.1 % of period (coordinator ruling, as ENG-036; owner confirms at the gate) | owner |
| Resting-contact penetration and drift tolerance (PHY-029) | not set | owner |
| Friction deceleration tolerance (PHY-030) | not set | owner |
| Milestone at which the native adapter is required (PHY-020) | not ruled | owner (ADR 0004 open item) |
| Engine carrying milestone 3 if retirement criteria are unmet | not ruled; ADR 0004 reads MuJoCo continues | owner |
| Status of the reserved Chrono adapter | not ruled | owner (ADR 0004 open item) |
| MuJoCo joint drive form (PHY-034) | soft one-joint equality `<joint>.drive` (polycoef rest 0 0 0 0, `eq_data[0]` = rest c each step, `eq_active` toggled per step) in MuJoCo's direct form: solref = (-s0, -s1) with s0 = (1 - d) K_d A0, s1 = (1 - d) D_d A0, K_d = K / c^2, D_d = D / c^2, A0 = `dof_invweight0` of the driven DOF (MuJoCo's `diagApprox` for the row, so R = (1 - d) / d A0); then (m / (1 - d)) a = F - D_d v - K_d (q - rest_d) with m = 1 / A0, exact static sag F / K_d and steady lag D_d w / K_d for every d. solimp = (d, d, 0.001, 0.5, 2), d = clamp(1 - 1 / (K_d A0 h^2), 1e-4, 0.99): d at its floor (exact dynamics) while K_d A0 h^2 <= 1, else the loop frequency is held at sqrt(s0) h = 1 and the drive carries an apparent inertia m / (1 - d) (decay rate D_d / (2 m) scaled by 1 - d). No explicit-integration limit; the mapping's own envelope: s0 h^2 + 2 s1 h < 4, that is K_d A0 h^2 < 400 (omega h < 20 with omega^2 = K_d A0) at d = 0.99, else `AdapterError`. The brief's form (-K_d / m, -D_d / m) with solimp (0.99, 0.99, 0.001) sags (1 - d) F / K_d, 1 percent of the closed form, and is explicit at omega h = sqrt(K_d / m) h; replaced (M1-X5 report) | M1-X5; mapping confirmed by the coordinator (M1-X6 ruling 1), for owner confirmation |
| Drive torque through a transmission's efficiency (PHY-034) | the drive acts on the driven joint lossless (K / c^2, no eta); its shaft torque is F c; the transmission's power flow and gain still follow the explicit shaft torque only; stays lossless (M1-X6 ruling 1), riding on the open owner question of where the transmission loss sits (queue, C re-review) | owner (with the loss-placement question) |

Which adapter carries milestone 3 is not ruled (ADR 0004 open item); live member end-loads at M3 require it.

## MuJoCo adapter, milestone 1 mapping

Recorded with the stepping adapter (ADR 0014; R-18). Full table: `mechsim/physics/mujoco/CLAUDE.md` and
`core/physics/mujoco/CLAUDE.md`.

| Item | Milestone 1 behavior |
|---|---|
| Release | MuJoCo 3.9.0, prebuilt Linux x86_64 archive pinned by SHA-256; other platforms not pinned |
| Joint drives (PHY-034) | Every transmission shaft joint and every joint listed in `signals.drive_joints` (revolute or prismatic; not a shaft, not a driven joint) gets an inactive soft joint equality `<joint>.drive` on the DOF it moves, listed in `mechsim:drives` and in the report; the adapter's form is in Open parameters. `joint_positions`: (q - qpos0) per revolute or prismatic joint, (q_driven - qpos0) / c for a reflected shaft, 0 otherwise; `joint_drive_torques`: the drive row's `efc_force` (times c on a shaft), 0 when disabled |
| Transmissions (PHY-018, PHY-033) | Reflection, exact for a rigid ratio driven = c * shaft (c = r, lead / 2 pi, 1 / ratio): the shaft body is not emitted, the driven joint carries armature J / c^2; the shaft torque tau acts on the driven joint as F = eta tau / c when driving (tau qd_driven c >= 0, standing still included) and F = tau / (eta c) when back-driven; the shaft's pose and rate are q_driven / c, qd_driven / c; its generalized force is the load torque (PHY-033). Finite stiffness is treated as rigid and reported; shaft mass, off-axis inertia and shapes are reported as not carried; backlash and slip are refused until milestone 2 (PHY-006), self-locking until the own engine's package 7 |
| Contacts | Explicit pairs; bodies joined by a non-free joint do not collide, each skip reported |
| Lumped members (PHY-018) | The own engine's midpoint-spring chain (ENG-022) as hinge joints; end-loads from the first segment's interaction force and the roller connect force |
| Refused (PHY-017, PHY-019) | One-sided joint limits, non-uniform damping or friction on spherical or free joints, free joints below a body, unknown element types |
| Spring index space (PHY-008) | Revolute, prismatic and planar joint DOFs in joint order, then lumped-member nodes root to tip |
| Payload index space (PHY-008) | Neutral bodies; mass change rescales inertia |
| Load release, re-attachment (PHY-008) | No mapping (no ropes, no re-parenting); the call is a run error |
| Drift (PHY-009) | Reported in health; the threshold check stays with the kernel's drift configuration |

Limits: implicitfast takes joint damping implicitly but member-node stiffness explicitly, so a lumped member on the
bridge is stable only for omega h < 2, with omega = sqrt(3 E / rho) r / l^2 per segment (r = sqrt(I / A), l the
segment length). The retirement baselines and milestone 3 bridge runs stay inside that envelope.

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Engine math, integrators, multibody, collision and contact solvers | ENG |
| Time ownership, rates and sync points | KER |
| Building the neutral model | BLD |
| Recording and baseline storage format | REC, SCO |
| Structural limit detection on end-loads | INT |
