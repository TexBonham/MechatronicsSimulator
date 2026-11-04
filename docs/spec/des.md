# Spec: Design files and CAD import (DES)

## Purpose

The design module (`mechsim/design/`) owns the design-file schema, the loader and its validation, and CAD import (STEP
through Open CASCADE, URDF parsed directly). It turns YAML design files into a validated in-memory design, reporting
every error at load or import rather than at run time. It leaves model construction to BLD, component and material
definitions to LIB and MAT, and serving designs to clients to FAC.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| LIB | DES-003, DES-023, DES-030 | LIB-012, LIB-013, LIB-014, LIB-015, LIB-028 |
| MAT | DES-001 | MAT-009, MAT-023 |
| BLD | DES-001 to DES-007, DES-018, DES-019, DES-022, DES-034, DES-035 | BLD-001, BLD-006, BLD-007, BLD-011, BLD-014, BLD-019 |
| FAC | DES-015, DES-036 | FAC-010, FAC-017 |
| REC | DES-008 | REC-013, REC-014 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| DES-001 | A design file shall declare its bodies, each with a material and a process. | Architecture doc, Modules: inputs, Design files row | M1 | unit | n/a |
| DES-002 | A design file shall declare its joints and mates, each with limits, friction and backlash. | Architecture doc, Modules: inputs, Design files row | M1 | unit | n/a |
| DES-003 | A design file shall declare component instances, each naming a library type or catalog part and giving its parameters. | Architecture doc, Modules: inputs, Design files row | M1 | unit | n/a |
| DES-004 | A design file shall declare cross-domain bindings, such as a motor shaft to a joint and a housing to a mount face. | Architecture doc, Modules: inputs, Design files row | M1 | unit | n/a |
| DES-005 | A design file shall declare a pin-binding table that maps every controller-facing signal to an MCU pin and peripheral mode. | Architecture doc, Modules: inputs, Design files row; Repository starting point, invariant 1 | M1 | unit | n/a |
| DES-006 | A design file shall declare its power hardware and rails. | Architecture doc, Modules: inputs, Design files row | M1 | unit | n/a |
| DES-007 | A design file shall declare its environment, including gravity and ambient conditions. | Architecture doc, Modules: inputs, Design files row | M1 | unit | n/a |
| DES-008 | A design file shall carry a semver design version, and the loader shall expose it together with the design file's git blob hash. | Ruling D3; ADR 0012 | M1 | unit | n/a |
| DES-009 | Every signal name in a design shall have the dotted form `instance.signal`, and a name of any other form shall be rejected at load. | Ruling D2; ADR 0012 | M1 | unit | n/a |
| DES-010 | Every signal name shall be unique within its design, and a duplicate shall be rejected at load with an error naming both declarations. | Ruling D2; ADR 0012 | M1 | unit | n/a |
| DES-011 | Every shipped design file shall be YAML and shall validate against the design JSON Schema. | Decisions log row 29; Architecture doc, Repository starting point, invariant 3 | M1 | invariant | n/a |
| DES-012 | The design schema shall be generated from pydantic v2 models, checked in under `schemas/`, and a staleness test shall fail when the checked-in schema differs from a fresh generation. | Ruling L4; ADR 0012, Consequences | phase 1 | invariant | n/a |
| DES-013 | Every quantity in a design file shall be a unit string parsed by pint at load and held as a plain SI float afterward. | Ruling L5; ADR 0012 | phase 1 | unit | n/a |
| DES-014 | Loading a design file with ruamel.yaml in round-trip mode and writing it back unchanged shall reproduce the file byte for byte, comments and key order included. | Ruling L6; ADR 0012, Consequences | phase 1 | unit | n/a |
| DES-015 | Every design validation shall run at load or import, before any run starts, and each error shall name the file, the path within it and the reason. | Requirements doc, Component library requirements (validation on load, clear errors); Architecture doc, Modules: core, Model builder row | M1 | unit | n/a |
| DES-016 | A reference to a library type, catalog part, material, body, joint or signal that does not exist shall be rejected at load with an error naming the reference. | Requirements doc, Component library requirements (validation on load); Architecture doc, Modules: core, Model builder row | M1 | unit | n/a |
| DES-017 | STEP files shall be read with Open CASCADE through the OCP Python bindings for body geometry. | Architecture doc, Technology recommendation, Data and files; Requirements doc, Relationship to existing tools, Fusion 360 CAD row | M4 | unit | n/a |
| DES-018 | STEP import shall compute volume, center of mass and inertia of each body from its geometry and its material's density. | Architecture doc, Technology recommendation, Data and files; Decisions log row 22 | M4 | reference | not set (owed by owner) |
| DES-019 | STEP import shall export collision meshes by convex decomposition with a library (CoACD), producing convex hulls. | Architecture doc, Technology recommendation, Data and files; ADR 0002 | M4 | unit | n/a |
| DES-020 | An imported body whose collision geometry needs true mesh contact shall be flagged at import as outside the v1 contact scope. | ADR 0002, Consequences | M4 | unit | n/a |
| DES-021 | URDF files shall be parsed directly into bodies and joints. | Architecture doc, Technology recommendation, Data and files; Requirements doc, Relationship to existing tools, Fusion 360 CAD row | M4 | unit | n/a |
| DES-022 | Declared mass properties from Fusion or URDF shall be used as the design's values, and the values recomputed from STEP shall be compared with them and a mismatch beyond tolerance reported at import. | Decisions log row 22 | M4 | unit | not set (owed by owner) |
| DES-023 | An imported STEP body shall be declarable as a catalog part, and the design shall then take its component type and parameters from that part. | Decisions log row 19; Architecture doc, Modules: inputs, Design files row | M4 | unit | n/a |
| DES-024 | A URDF joint type with no matching library joint type shall be rejected at import with an error naming the joint. | Architecture doc, Repository starting point, invariant 7 | M4 | unit | n/a |
| DES-025 | The gantry shall ship as `designs/gantry/` with its test cases under `test_cases/gantry/` and shall build, run and score with the library as shipped and no code outside those folders and `firmware/`. | Architecture doc, Repository starting point, invariant 2 and How to extend, "A design" row; Modules: inputs, Design files row | M1 | acceptance | n/a |
| DES-026 | The pendulum shall ship as `designs/pendulum/` with a rigid link and shall build, run and score with the library as shipped and no design-specific code. | Architecture doc, Repository starting point, milestone 1 paragraph and invariant 2; ruling P4 | M1 | acceptance | n/a |
| DES-027 | A CAD file referenced from a design shall be resolved relative to the design file, and a missing file shall be rejected at load. | Requirements doc, Component library requirements (clear errors for missing values); Architecture doc, Repository starting point, invariant 3 | M4 | unit | n/a |
| DES-028 | A joint limit whose lower bound exceeds its upper bound shall be rejected at load. | Requirements doc, Component library requirements (impossible values) | M1 | unit | n/a |
| DES-029 | Every pin-binding table entry shall name a pin and peripheral mode that the chip's pin map provides, and any other entry shall be rejected at load. | Architecture doc, Modules: inputs, Design files row; Decisions log row 11 | M1 | unit | n/a |
| DES-030 | Every component instance's parameters shall be validated at load against its library type's parameter model. | Requirements doc, Component library requirements (validation on load) | M1 | unit | n/a |
| DES-031 | A design file shall carry its design-load rule as data. | Requirements doc, Physics and structural analysis requirements, Structural checks | M3 | unit | n/a |
| DES-032 | A design file shall carry its accuracy target used as the deflection limit for structural checks. | Requirements doc, Physics and structural analysis requirements, Structural checks; Decisions log row 17 | M3 | unit | n/a |
| DES-033 | A design file shall declare which members are flexible and the cross-section of each. | Decisions log row 17; Requirements doc, Dynamics (flexible members) | M3 | unit | n/a |
| DES-034 | A design file shall declare, for each flexible member, the known deflection and first natural frequency it is calibrated to, from hand calculation or FEA. | Decisions log row 18; Requirements doc, Dynamics (flexible members) | M3 | unit | n/a |
| DES-035 | Imported FEA results (stiffness, deflection, natural frequencies) for a flexible member shall be accepted into the design together with the design version they belong to. | Requirements doc, Load handoff with FEA tools, items 2 and 3 | M4 | unit | n/a |
| DES-036 | Loading and validating a design shall be callable without starting a run and shall return every error found, for the CLI validate command. | Ruling L9; ADR 0012 | phase 1 | unit | n/a |
| DES-037 | An imported body's material and process shall be declared in the design file when the CAD source does not supply them, and a body without them shall be rejected at load. | Architecture doc, Modules: inputs, Design files row; Modules: core, Model builder row (missing material) | M4 | unit | n/a |
| DES-038 | A draft design may declare open parameters (name, dimension, owed by, note) and reference them as `${name}` in place of a value; a release design shall declare none, and a reference to an undeclared open parameter or one of the wrong dimension shall be rejected at load. | ADR 0012, Architecture doc invariant 3; Requirements doc, Reference design ("not set" items are parameters) | phase 1 | unit | n/a |
| DES-039 | The design loader shall resolve component types and catalog parts through one registry built from every shipped catalog file, and a type or part name declared in two catalog files shall be rejected. | Requirements doc, Component library requirements (validation on load); Decisions log row 19 | phase 1 | unit | n/a |
| DES-040 | A design file shall name its firmware tree, its chip definition and its control tick rate as data. | C4; ADR 0011; Architecture doc, Modules: inputs, Design files row | phase 1 | unit | n/a |
| DES-041 | A design file shall declare the electrical wiring between component instances (driver to motor, supply to driver) and the consumers of each power rail, so the builder can gate motor power and wire the signal layer (BLD-006, G7, G8). | Architecture doc, Modules: inputs, Design files row (cross-domain bindings, power rails) | M1 | unit | n/a |
| DES-042 | A design file's environment shall declare the contact model defaults (normal stiffness, damping, friction), per-material-pair overrides, body-pair exclusions and the floor plane, from which the builder fills the neutral model's contact section. | Owner ruling 2026-11-04; ADR 0002; ADR 0003 (contacts element) | M1 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Mass cross-check tolerance between declared and recomputed values (DES-022); proposed 5 percent on mass and center-of-mass offset relative to body size | not set | owner |
| Whether a mass cross-check mismatch is an error or a reported warning | not set | owner |
| STEP mass-property reference tolerance on closed-form solids (DES-018); proposed 1e-6 relative | not set | owner |
| Convex decomposition settings (hull count, concavity) | not set | design brief, M4 |
| Gantry rated load and workspace dimensions | not set | gantry team |
| Whether the gantry deflection target is static or dynamic (DES-032) | not set; planning default static with dynamic reported | gantry team |
| Which gantry members are flexible and their sections (DES-033) | not set; planning default rails, extrusion sections | gantry team |
| Where the minimum safety factor is declared (design or test case); the design schema carries an optional `structural.minimum_safety_factor` meanwhile | not set | owner |
| Wiring between component instances (driver coils to motor, supply to driver) beyond the controller-facing pin-binding table | ruled: connectivity only; physical wire properties are a later component type (owner ruling 2026-11-04). The design's `wiring` list connects `instance.port` to `instance.port`, ports being the pins of each type's electrical facet; output to input or bidirectional, bidirectional to input or bidirectional, power out to power in, ground to ground; a signal port carries one connection (a controller-facing signal counts as one), power and ground ports may be shared; each rail lists its `consumers` (DES-041) | owner |
| Closed-loop mates (DES-002): not representable yet; every body has exactly one parent joint, and loop closure arrives with the constraint element | not set | owner |
| Where contact model parameters and the floor live | design environment (owner ruling 2026-11-04); DES-042 carries it as `environment.contact`: `default`, `pairs`, `exclusions` field for field with the neutral model's contacts element, plus `floor` (normal and offset, on the world only) | owner |
| Chip pin-map check (DES-029) | a concrete `firmware.chip` names `chips/<name>/chip.yaml`; an open chip skips the check. Pin-map modes use the design's peripheral-mode names | owner |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Building the neutral physics model, wiring and chip configuration | BLD |
| Component type and catalog definitions | LIB |
| Material properties and process rules | MAT |
| Test case files | TST |
| Serving designs and validation results to clients | FAC |
| Recording the design version in archives | REC |
| Load-case export to FEA | INT |
