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
| BLD-010 | The builder shall compute mass, center of mass and inertia from a body's geometry and its material's density, matching closed-form values for primitive solids. | Architecture doc, Modules: core, Model builder row | M1 | reference | not set (owed by owner) |
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

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Mass property tolerance on primitive solids (BLD-010); proposed 1e-9 relative | not set | owner |
| Lumped beam deflection and first-mode tolerances (BLD-015 to BLD-017, BLD-019); proposed 2 percent | not set | owner |
| Default segment count per lumped beam (BLD-018) | 4 (ADR 0013), overridable per member | owner |
| Overlap test geometry and allowed contact clearance (BLD-022) | not set | owner |
| Neutral model schema | not set | phase 1 contracts and schemas (ADR 0003) |
| Rope lumped-chain detail (BLD-032) | not set | milestone 3 ADR (ruling P4) |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Translation of the neutral model into an engine | PHY |
| Design loading and load-time validation | DES |
| Stress recovery and limit checks on members | INT |
| Running signal wiring and pin configuration | SIG, CTL |
| Time stepping and rates | KER |
