# Spec: Model builder (BLD)

## Purpose

The model builder (`mechsim/builder/`) turns a validated design into the engine-neutral physics model of ADR 0003, the
signal-layer wiring, the chip and pin configuration and the structural member list. It resolves catalog parts,
computes mass and inertia, generates collision shapes within the v1 contact scope and builds multi-segment lumped beams
for flexible members, and it fails loudly rather than emit a partial model. It does not translate the model into any
engine; that is PHY.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| DES | BLD-001, BLD-011, BLD-014, BLD-019 | DES-001 to DES-007, DES-022, DES-034, DES-035 |
| LIB | BLD-005, BLD-006, BLD-027 | LIB-001, LIB-002, LIB-005, LIB-023, LIB-024 |
| MAT | BLD-010, BLD-020 | MAT-020, MAT-022, MAT-023 |
| PHY | BLD-001, BLD-024, BLD-026, BLD-030 | PHY-001, PHY-012, PHY-019 |
| SIG | BLD-006, BLD-023 | SIG-015, SIG-023, SIG-024, SIG-029 |
| KER | BLD-006, BLD-007 | KER-003, KER-013, KER-018 |
| INT | BLD-008, BLD-009, BLD-019 | INT-005, INT-020, INT-023, INT-034 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| BLD-001 | The builder shall turn a validated design into one engine-neutral physics model made of bodies, joints, constraints, contacts, lumped members and transmissions. | ADR 0003; ruling R-10 | M1 | unit | n/a |
| BLD-002 | Every body in the neutral model shall carry its mass, center of mass and inertia. | ADR 0003, Decision table (bodies) | M1 | unit | n/a |
| BLD-003 | Every joint in the neutral model shall be one of the six joint types of engine package 2. | ADR 0003, Decision table (joints) | M1 | unit | n/a |
| BLD-004 | A kinematic loop in the design shall be emitted as a constraint in the neutral model. | ADR 0003, Decision table (constraints) | M1 | unit | n/a |
| BLD-005 | Belts, lead screws and gears shall be emitted as our transmission models in the neutral form, not as engine features. | Ruling P3; ruling R-13; ADR 0003 | M1 | unit | n/a |
| BLD-006 | The builder shall emit the signal-layer wiring for every component instance's signals and every cross-domain binding. | Architecture doc, Modules: core, Model builder row | M1 | unit | n/a |
| BLD-007 | The builder shall emit the chip and pin configuration from the design's pin-binding table. | Architecture doc, Modules: core, Model builder row | M1 | unit | n/a |
| BLD-008 | The builder shall emit a structural member list naming every structural member and mount of the design. | Architecture doc, Modules: core, Model builder row; Requirements doc, Structural checks | M3 | unit | n/a |
| BLD-009 | Every entry of the structural member list shall record the member's section, material and support conditions. | Requirements doc, Structural checks; Decisions log row 17 | M3 | unit | n/a |
| BLD-010 | The builder shall compute mass, center of mass and inertia from a body's geometry and its material's density, matching closed-form values for primitive solids. | Architecture doc, Modules: core, Model builder row | M1 | reference | 1e-12 relative (owner ruling 2026-11-05) |
| BLD-011 | Where a design declares mass properties, the builder shall use the declared values; it shall compute them only where none are declared. | Decisions log row 22 | M1 | unit | n/a |
| BLD-012 | Every collision shape the builder emits shall be a primitive or a convex hull. | ADR 0002; rulings R-05, R-06 | M1 | unit | n/a |
| BLD-013 | The contacts element shall express floor contact, a body catching an obstacle and a carriage hitting a hard stop using primitives and convex hulls only. | ADR 0002, Consequences | M1 | unit | n/a |
| BLD-014 | Every flexible member shall be emitted as a multi-segment lumped beam. | Ruling R-13; ADR 0010, milestone table; Architecture doc, Modules: core, Model builder row | M3 | unit | n/a |
| BLD-015 | A lumped cantilever beam built from a known section and material shall match the closed-form static tip deflection. | Decisions log row 18; Architecture doc, Technology recommendation, Validation paragraph | M3 | reference | not set (owed by owner) |
| BLD-016 | A lumped cantilever beam built from a known section and material shall match the closed-form first natural frequency. | Decisions log row 18; Architecture doc, Technology recommendation, Validation paragraph | M3 | reference | not set (owed by owner) |
| BLD-017 | A lumped simply supported beam shall match the closed-form midspan deflection and first natural frequency. | Architecture doc, Technology recommendation, Validation paragraph | M3 | reference | not set (owed by owner) |
| BLD-018 | The segment count of each lumped beam shall be a data parameter, not a constant in code. | Ruling R-13; Architecture doc, Repository starting point, invariant 3 | M3 | unit | n/a |
| BLD-019 | The builder shall calibrate each flexible lumped beam so that it reproduces the deflection and first natural frequency recorded for that member in the design file. | Decisions log row 18; Requirements doc, Dynamics and Load handoff item 2 | M3 | reference | not set (owed by owner) |
| BLD-020 | A body with no material shall end the build with a report naming the body. | Architecture doc, Modules: core, Model builder row | M1 | unit | n/a |
| BLD-021 | An unconstrained body shall end the build with a report naming the body. | Architecture doc, Modules: core, Model builder row | M1 | unit | n/a |
| BLD-022 | Overlapping bodies shall end the build with a report naming both bodies. | Architecture doc, Modules: core, Model builder row | M1 | unit | n/a |
| BLD-023 | A controller-facing signal with no pin binding shall end the build with a report naming the signal. | Architecture doc, Modules: core, Model builder row; Repository starting point, invariant 1 | M1 | unit | n/a |
| BLD-024 | When the build targets an adapter, an element of the neutral model that adapter cannot map shall end the build with a report naming the element and the adapter. | ADR 0003, Consequences; Architecture doc, Repository starting point, invariant 7 | M1 | unit | n/a |
| BLD-025 | A failed build shall emit no model, wiring or configuration, and its report shall list every failure found. | Architecture doc, Repository starting point, invariant 7 | M1 | unit | n/a |
| BLD-026 | Building the same design with the same inputs twice shall produce an identical neutral model, wiring, chip configuration and member list. | Architecture doc, Repository starting point, invariant 4 | M1 | invariant | n/a |
| BLD-027 | The builder shall resolve every catalog part reference, combining the catalog part's parameters with the instance's overrides. | Architecture doc, Modules: core, Model builder row | M1 | unit | n/a |
| BLD-028 | Every cross-domain binding shall resolve to an existing joint or mount face, and an unresolved binding shall end the build with a report. | Architecture doc, Modules: core, Model builder row (validates every binding) | M1 | unit | n/a |
| BLD-029 | The milestone 1 pendulum link shall be emitted as a rigid body on a revolute joint with no rope element. | Ruling P4; ADR 0003, Consequences | M1 | unit | n/a |
| BLD-030 | The neutral model shall hold only plain SI floats and builder-owned types, and no unit object or engine type. | Ruling L5; Architecture doc, Repository starting point, invariant 5 | M1 | unit | n/a |
| BLD-031 | Every fastener and mount shall be emitted as an individually addressable constraint, so that a modeled failure can disable it. | ADR 0010, Tier 2 table and Consequences | M3 | unit | n/a |
| BLD-032 | A rope shall be emitted as a lumped chain in the neutral form. | Ruling R-13; ruling P4 | M3 | unit | n/a |
| BLD-033 | The builder shall offer a rigid-members option that builds every flexible member as a rigid body while keeping the structural member list, for milestone 1 runs. | Requirements doc, Instructions for the planning session, milestone plan item 1 | M1 | unit | n/a |
| BLD-034 | The builder shall set the axial inertia of the shaft body an actuator instance's shaft mount drives (about the shaft joint's axis through its center of mass) to that instance's rotor inertia: the shaft body's axial inertia is the motor's rotor inertia, so the motor instance is the one source of rotor inertia for physics and the signal layer (SIG-037). A shaft body that declares its own mass properties, and so its own axial inertia, is a build error naming the body and the motor. | Coordinator ruling 2026-11-04 (shaft load torque, rotor inertia one source); SIG-037; brief M1-X7 coordinator ruling 4 (2026-11-04) Two actuator instances on one shaft joint shall be refused at build, naming both. | M1 | unit | 1e-12 relative |
| BLD-035 | The builder shall carry each sensor instance's sensed joint, switch position, actuation side and count direction (DES-043), and each actuator part's torque-speed curve in SI, into the wiring table, so the signal layer reads the sensor on its joint coordinate and limits the motor by its curve. A brake's held joint reaches the signal layer, with, on a motor shaft joint, the inverse inertia 1 / (J + m c^2) of the shaft with the driven mass reflected; a brake acts on a revolute (shaft) joint, and one on a joint the physics carrier cannot hold (a transmission's driven joint) or on any other joint is a WiringError naming the brake and the joint (round 4 ruling 29); a load cell its payload index, an amplifier the chip's ADC bits and reference; a safety instance without a power_out port on a rail is a consumer, not a gate. | M1-X3 coordinator ruling 1 (2026-11-04), for owner confirmation | M1 | unit | n/a |
| BLD-036 | The case payload shall reach the engine-neutral model as a body of the payload's mass (placeholder solid box of 0.1 m edge, inertia m a^2 / 6 per axis) hung below its carrier along gravity clear of the carrier's shapes: a payload attached at t = 0 that no fault releases as a fixed child of the carrier (joint `<payload>_fixed`, placed in the carrier's frame; rigid, no equality; round 4 ruling 24), any other as a free body under world welded to its carrier by the adapter's payload argument, following the carrier to the start configuration (mid-run attach or release); a case that attaches no payload gets no payload body (no stand-in), and a load cell that weighs a payload then has no target and reads zero force (SIG-014). | M2-X1 integration brief (2026-11-10); round 4 ruling 24, for confirmation; stand-in removed, briefs M2-A4 and M2-X5 | M2 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Mass property tolerance on primitive solids (BLD-010); proposed 1e-9 relative | not set | owner |
| Lumped beam deflection and first-mode tolerances (BLD-015 to BLD-017, BLD-019); proposed 2 percent | not set | owner |
| Default segment count per lumped beam (BLD-018) | 4 (ADR 0013); 8 for a pinned root with a roller end (ENG-040); overridable per member in the neutral model | owner |
| Overlap test geometry and allowed contact clearance (BLD-022) | World axis-aligned bounds of each primitive or hull at the reference configuration (all joint coordinates zero); bodies joined by a joint or excluded in the contacts element are skipped; touching passes (zero clearance); milestone 1 brief M1-A | owner (to confirm) |
| Draft design whose physics holds open values | The physics model and member list are not emitted and the open paths are listed as pending; wiring and chip configuration are still emitted; a release design with a pending value fails; no BLD row covers it (milestone 1 brief M1-A) | owner |
| Element kinds an adapter declares (BLD-024) | `body`, `joint.<type>`, `shape.<shape>`, `contacts`, `constraint.<type>`, `lumped_member`, `transmission.<type>` (milestone 1 brief M1-A) | PHY |
| Flexible member supports with no lumped form (BLD-014) | The neutral beam takes a clamped or pinned root and a free or roller end; any other pair (the gantry rail_x planning default clamped-clamped, Decisions log row 17) fails the build once its physics is set | owner, with ENG |
| Lumped member placement (BLD-014) | Rooted on the flexible body itself at its frame origin along its x axis, named `<body>_beam`; the rigid body stays in the model | milestone 3 |
| Joint backlash | No neutral joint element; a non-zero design backlash fails the build | owner |
| Floor plane material | The world plane carries the material name `floor` | owner |
| Signal slots | Load torque on every joint an actuator's shaft mount binds; a pose slot for every body | SIG |
| Board channel index | Per peripheral mode, counted from 0 in pin-binding order; `channels.h` defines `MECHSIM_CH_<INSTANCE>_<SIGNAL>` | SIG, FW |
| Physics step in the neutral model | Not taken from the design's tick rate: physics step is run configuration (C4, KER run configuration keeps physics and control rates apart); the model carries the 1 ms default (ADR 0013) and the run configuration overrides it | KER |
| Module instance order (`module_instances`) | KER-009 tick order: physics, integrity, sensor instances, controller, signal layer, power and safety instances, drivers, actuators, transmissions, any other instance, recorder; design order within a group; the runner's D1 seeding spawns child generators in this order | KER |
| Neutral model schema | `mechsim/physics/neutral.py`, generated `schemas/physics_model.json` (phase 1 brief C1) | phase 1 contracts and schemas (ADR 0003) |
| Rope lumped-chain detail (BLD-032) | not set | milestone 3 ADR (ruling P4) |
| Default soft-contact parameters (normal stiffness, normal damping, friction coefficient) in the neutral model's contacts element; ADR 0002 fixes soft contact but no values, so the model requires them per design | not set | owner |
| Contact material-pair overrides and body-pair exclusions in the neutral model (structure added in phase 1 brief C1; not sourced to ADR 0002 or a BLD/PHY row) | not set | owner |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Translation of the neutral model into an engine | PHY |
| Design loading and load-time validation | DES |
| Stress recovery and limit checks on members | INT |
| Running signal wiring and pin configuration | SIG, CTL |
| Time stepping and rates | KER |
