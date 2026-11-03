# Spec: Engine (ENG)

## Purpose

The own physics engine under `core/engine/` (ADR 0001): math core and integrators, rigid multibody dynamics with joints
and kinematic loops, in-house collision detection, soft contact, lumped flexible members, solver health reporting and the
live loads the rest of the tool reads every tick. It runs the engine-neutral physics model handed to it by the native
adapter; translating that model, scheduling the step and every electrical path are left to PHY, KER and SIG.

Sources: "Architecture doc" is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`;
"Requirements doc" is `docs/Mechatronic Simulator — Requirements.md`. Milestone "spike" is the calibration spike before
phase 1 (R-12), engine packages 1 and 2.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| KER | ENG-002, ENG-023, ENG-024, ENG-025, ENG-026, ENG-027, ENG-028, ENG-029, ENG-030 | KER-004, KER-009, KER-010, KER-015, KER-016, KER-018 |
| PHY | ENG-003, ENG-022, ENG-028, ENG-030, ENG-031 to ENG-035, ENG-043 to ENG-048 | PHY-001, PHY-008, PHY-012, PHY-018, PHY-019, PHY-020 |
| INT | ENG-030, ENG-031, ENG-032, ENG-033, ENG-034 | INT-023, INT-028, INT-029, INT-030, INT-031 |
| SIG (via KER) | ENG-027, ENG-028, ENG-046, ENG-048 (shaft torque in, shaft load torque out) | SIG-013, SIG-016, SIG-032, SIG-033 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| ENG-001 | The engine shall do its linear algebra through Eigen and shall carry no other linear algebra dependency. | ADR 0001, Decision; R-04 | spike | unit | n/a |
| ENG-002 | The engine shall advance the model by a fixed step whose length is supplied by the run configuration, with 1 ms as the default and no step length held as a code constant. | Architecture doc, Data flows (per-tick order); ADR 0011, C4 | spike | unit | n/a |
| ENG-003 | The engine shall integrate the equations of motion with the scheme ruled by the own-engine integrator ADR (ADR 0013), behind an integrator interface internal to the engine. | ADR 0003, P2; ADR 0001, Open items | spike | unit | n/a |
| ENG-004 | The engine shall hold every state, parameter and output quantity in SI units. | Architecture doc, Repository starting point (invariant 3) | spike | unit | n/a |
| ENG-005 | The engine shall simulate rigid bodies with mass, center of mass and inertia tensor under a gravity vector supplied by the model. | Requirements doc, Physics and structural analysis requirements (Dynamics); Requirements doc, Component library requirements (Bodies and structure) | spike | unit | n/a |
| ENG-006 | The engine shall provide a revolute joint. | Requirements doc, Component library requirements (Joints); ADR 0001, Decision (package 2) | spike | unit | n/a |
| ENG-007 | The engine shall provide a prismatic joint. | Requirements doc, Component library requirements (Joints); ADR 0001, Decision (package 2) | spike | unit | n/a |
| ENG-008 | The engine shall provide a fixed joint. | Requirements doc, Component library requirements (Joints); ADR 0001, Decision (package 2) | spike | unit | n/a |
| ENG-009 | The engine shall provide a planar joint. | Requirements doc, Component library requirements (Joints); ADR 0001, Decision (package 2) | spike | unit | n/a |
| ENG-010 | The engine shall provide a spherical joint. | Requirements doc, Component library requirements (Joints); ADR 0001, Decision (package 2) | spike | unit | n/a |
| ENG-011 | The engine shall provide a free joint (six degrees of freedom) so an unattached body such as a payload resting on the floor is a tree root with no constraint to the world. | Requirements, Physics and structural analysis (payload resting on the floor); Decisions log row 28; ADR 0001 (package 2, six joint types) | spike | unit | n/a |
| ENG-012 | The engine shall close kinematic loops through bilateral constraints between bodies. | ADR 0001, Decision (package 2); ADR 0003, Decision (Constraints) | spike | unit | n/a |
| ENG-013 | The engine shall enforce joint travel limits as one-sided constraints. | Requirements doc, Component library requirements (Joints); Requirements doc, Physics and structural analysis requirements (Dynamics) | M1 | unit | n/a |
| ENG-014 | The engine shall apply joint friction and joint damping from per-joint parameters. | Requirements doc, Component library requirements (Joints); Architecture doc, Modules: core (Physics engine row) | M1 | unit | n/a |
| ENG-015 | The engine shall apply joint backlash from a per-joint parameter. | Requirements doc, Component library requirements (Joints) | M1 | unit | n/a |
| ENG-016 | The engine shall detect collisions between primitives and between convex hulls with in-house code. | ADR 0002, Decision; R-05; R-06 | M2 | unit | n/a |
| ENG-017 | The engine shall end the run with an error when a contact geometry is neither a primitive nor a convex hull. | ADR 0002, Consequences; Architecture doc, Repository starting point (invariant 7) | M2 | unit | n/a |
| ENG-018 | The engine shall not link the reference collision library; that library shall be used only inside the test suite. | ADR 0002, Decision and Consequences; R-05 | M2 | invariant | n/a |
| ENG-019 | The engine's collision detection shall agree with the reference collision library on contact existence, normal and penetration depth over a shared case set. | ADR 0002, Decision (differential oracle) | M2 | reference | not set (owed by owner, proposed 1e-9 m depth, 1e-6 rad normal) |
| ENG-020 | The engine shall model contact as soft contact integrated implicitly, with contact friction. | ADR 0002, Decision; R-06; Requirements doc, Physics and structural analysis requirements (Dynamics) | M2 | unit | n/a |
| ENG-021 | The engine shall bring a body dropped onto a plane to rest with steady penetration within tolerance, and shall stop a body driven into a fixed hard stop. | Architecture doc, Decisions log row 28; ADR 0002, Consequences | M2 | reference | not set (owed by owner) |
| ENG-022 | The engine shall simulate a flexible member as a multi-segment lumped beam built from the member's section and material. | R-13; ADR 0001, Decision (lumped domain models); ADR 0010, Decision (milestone 3) | spike | unit | n/a |
| ENG-023 | Given the same model, inputs and step on the same machine and build, the engine shall produce bitwise identical state after every step. | Architecture doc, Repository starting point (invariant 4); Requirements doc, Physics and structural analysis requirements (Dynamics) | spike | invariant | bitwise identical |
| ENG-024 | The engine shall measure constraint drift every step and end the run with a drift error, carrying the time and the offending constraint, when drift exceeds the drift threshold. | Requirements doc, Physics and structural analysis requirements (Dynamics, solver health); Architecture doc, Repository starting point (invariant 7) | spike | unit | not set (owed by owner, proposed 1e-6 m and 1e-6 rad) |
| ENG-025 | The engine shall end the run with a solver-failure error, carrying the time and cause, when a state value becomes non-finite or the solver fails to converge within its iteration limit. | Requirements doc, Physics and structural analysis requirements (Dynamics, solver health); Architecture doc, Repository starting point (invariant 7) | spike | unit | n/a |
| ENG-026 | The engine shall publish a solver health record every step: maximum constraint drift and solver convergence status. | Architecture doc, Modules: core (Co-simulation kernel row) | spike | unit | n/a |
| ENG-027 | The engine shall publish body poses and velocities, joint forces and contact forces after every step. | Architecture doc, Modules: core (Physics engine row, interface out); Architecture doc, Data flows (per-tick order, step 1) | spike | unit | n/a |
| ENG-028 | The engine shall accept actuator torques and forces before each step and apply them for that step only. | Architecture doc, Modules: core (Physics engine row, interface in); Architecture doc, Data flows (per-tick order, step 5) | spike | unit | n/a |
| ENG-029 | The engine shall run faster than real time for batch runs. | Requirements doc, Physics and structural analysis requirements (Dynamics, solver health) | M1 | unit | not set (owed by owner, proposed real-time factor of at least 1 on the CI machine) |
| ENG-030 | The engine shall publish the end-loads (three forces and three moments at each end) of every lumped member every step. | ADR 0010, Decision (interface requirement); R-21 | M3 | unit | n/a |
| ENG-031 | The engine shall accept a mid-run change that sets a permanent offset on a lumped spring. | ADR 0010, Decision (tier 2, yield) and Consequences | M3 | unit | n/a |
| ENG-032 | The engine shall accept a mid-run change of a lumped spring's stiffness. | ADR 0010, Decision (tier 2, buckling) and Consequences | M3 | unit | n/a |
| ENG-033 | The engine shall accept a mid-run disable of a constraint. | ADR 0010, Decision (tier 2, fastener break) and Consequences | M3 | unit | n/a |
| ENG-034 | The engine shall accept a mid-run release of a load from its attachment. | ADR 0010, Decision (tier 2, rope snap) and Consequences | M3 | unit | n/a |
| ENG-035 | The engine shall accept a mid-run change of a payload's mass or attachment at a scheduled time. | Architecture doc, Data flows (fault fan-out, payload change mid-run) | M2 | unit | n/a |
| ENG-036 | A rigid-link pendulum's small-amplitude period shall match the compound pendulum closed form 2*pi*sqrt(I/(m*g*d)). | Architecture doc, Technology recommendation ("Validation"); R-12 | spike | reference | not set (owed by owner, proposed 0.1 % of period) |
| ENG-037 | A pendulum with viscous joint damping shall decay with the logarithmic decrement of the linear damped oscillator closed form. | Architecture doc, Technology recommendation ("Validation", swing decay); R-12 | spike | reference | not set (owed by owner, proposed 1 % of decrement) |
| ENG-038 | A lumped cantilever under a static tip load shall deflect by P*L^3/(3*E*I) at the tip. | Architecture doc, Technology recommendation ("Validation"); R-12; R-13 | spike | reference | not set (owed by owner, proposed 2 % of deflection) |
| ENG-039 | A lumped cantilever's first natural frequency shall match (1.875^2/(2*pi))*sqrt(E*I/(rho*A*L^4)). | Architecture doc, Technology recommendation ("Validation"); Requirements doc, Physics and structural analysis requirements (Dynamics, flexible members); R-12 | spike | reference | not set (owed by owner, proposed 3 % of frequency) |
| ENG-040 | A lumped simply supported beam under a static midspan load shall deflect by P*L^3/(48*E*I) at midspan. | Architecture doc, Technology recommendation ("Validation"); R-12; R-13 | spike | reference | not set (owed by owner, proposed 2 % of deflection) |
| ENG-041 | A lumped simply supported beam's first natural frequency shall match (pi/(2*L^2))*sqrt(E*I/(rho*A)). | Architecture doc, Technology recommendation ("Validation"); R-12 | spike | reference | not set (owed by owner, proposed 3 % of frequency) |
| ENG-042 | A lumped pinned-pinned column under axial load shall lose stability at the Euler load pi^2*E*I/L^2. | R-20 (buckling); R-13; ADR 0010, Decision (buckling margin) | M3 | reference | not set (owed by owner, proposed 5 % of critical load) |
| ENG-043 | The engine shall simulate a belt and pulley transmission element of the neutral model with its ratio, efficiency, stiffness, backlash and slip parameters. | Requirements doc, Component library requirements (Transmissions row); ADR 0003, P3; ADR 0001, R-13 | M1 | unit | n/a |
| ENG-044 | The engine shall simulate a lead screw transmission element with its lead, efficiency, stiffness, backlash and slip parameters, and a self-locking lead screw shall hold its joint against back-driving load with no motor torque. | Requirements doc, Component library requirements (Transmissions row); ADR 0003, P3; ADR 0001, R-13; Decisions log row 15 | M1 | unit | n/a |
| ENG-045 | The engine shall simulate a gear pair transmission element with its ratio, efficiency, stiffness, backlash and slip parameters. | Requirements doc, Component library requirements (Transmissions row); ADR 0003, P3; ADR 0001, R-13 | M2 | unit | n/a |
| ENG-046 | Every transmission element's slip parameter shall be zero by default, and the engine shall accept a mid-run change of it so that a slip fault makes that transmission slip for the fault window. | Requirements doc, Component library requirements (Transmissions row); ADR 0003, P3; ADR 0001, R-13; Architecture doc, Data flows (fault fan-out, belt or rope slip) | M2 | unit | n/a |
| ENG-047 | The engine shall simulate a rope as a lumped chain with unilateral tension, slack and length change. | Requirements doc, Component library requirements (Transmissions row); ADR 0003, P3 and P4; ADR 0001, R-13; Decisions log row 15 | M3 | unit | n/a |
| ENG-048 | Each transmission element shall map the motor shaft torque the signal layer applies into joint force for the step, and the engine shall publish the load torque on each motor shaft after every step for the stepper model. | Architecture doc, Data flows (within one physics tick, step 5); ADR 0003, P3; ADR 0001, R-13 | M1 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Primitive set for collision | not set | owner, at the collision package brief |
| Reference collision library | not set | owner, at the collision package brief (ADR 0002, Open items) |
| Collision differential tolerance (ENG-019) | not set, proposed 1e-9 m depth, 1e-6 rad normal | owner |
| Contact rest penetration tolerance (ENG-021) | not set | owner |
| Constraint drift threshold (ENG-024) | 1e-6 m and 1e-6 rad (ADR 0013; change by reviewed spec edit, T4) | owner |
| Solver iteration limit (ENG-025) | fixed cap, value set in the engine package 1 brief (ADR 0013) | owner |
| Real-time factor floor and benchmark model (ENG-029) | not set, proposed at least 1 | owner |
| Pendulum period tolerance (ENG-036) | not set, proposed 0.1 % | owner |
| Pendulum decay tolerance (ENG-037) | not set, proposed 1 % | owner |
| Beam deflection tolerances (ENG-038, ENG-040) | not set, proposed 2 % | owner |
| Beam first-mode tolerances (ENG-039, ENG-041) | not set, proposed 3 % | owner |
| Euler buckling tolerance (ENG-042) | not set, proposed 5 % | owner |
| Lumped beam segment count for the references | 4 by default, per member override (ADR 0013) | owner |
| Milestone of each tier 2 model change (ENG-031 to ENG-034) | not set; tier 2 spans milestones 3 and 4 | owner, milestone 3 plan (ADR 0010, Open items) |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Translating the neutral physics model into engine objects | PHY (native adapter) |
| Rope model detail beyond ENG-047 | Milestone 3 ADR (ADR 0003, P4) |
| Stepper torque-speed curve, stall and missed steps | SIG (behavioral stepper, SIG-032 to SIG-034) |
| Mesh contact | Later contact tier (ADR 0002) |
| Convex decomposition of imported meshes | BLD (library, ADR 0002) |
| Stress recovery, safety factors and limit checks from member end-loads | INT |
| Scheduling the step and routing outputs | KER |
| Engineering beam elements | Roadmap seam (Architecture doc, Seams reserved for the roadmap) |
| Engine debug viewer | `core/viewer_debug/` (R-17) |
