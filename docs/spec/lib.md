# Spec: Component library (LIB)

## Purpose

The component library (`mechsim/library/`) holds parameterized component types as data, each with mechanical,
electrical, ratings and thermal facets and its named fidelity tiers, plus the plugin interface for user-added types and
the shipped v1 catalog in YAML. It validates catalog and instance parameters on load and exposes every parameter for
sweeps. It leaves design assembly to DES, model construction to BLD and runtime signal behavior to SIG.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| DES | LIB-012, LIB-013, LIB-014, LIB-015, LIB-016, LIB-028 | DES-003, DES-023, DES-030 |
| BLD | LIB-001, LIB-002, LIB-005, LIB-023, LIB-024, LIB-033 | BLD-005, BLD-006, BLD-027 |
| SIG | LIB-002, LIB-022, LIB-025, LIB-026, LIB-027 | SIG-001, SIG-012, SIG-029, SIG-032, SIG-035 |
| TST | LIB-015, LIB-016 | TST-007, TST-025, TST-026 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| LIB-001 | Every component type shall declare a mechanical facet holding its mass, mounts and shaft axis as parameters. | Architecture doc, Modules: inputs, Component library row | M1 | unit | n/a |
| LIB-002 | Every component type shall declare an electrical facet listing its pins and the signals it emits or consumes. | Architecture doc, Modules: inputs, Component library row | M1 | unit | n/a |
| LIB-003 | Every component type shall declare a ratings facet holding its absolute maximums, safe operating area and, where applicable, holding torque. | Architecture doc, Modules: inputs, Component library row | M1 | unit | n/a |
| LIB-004 | Every component type shall carry a thermal facet slot whose value is held constant in v1, so that no v1 computation changes it during a run. | Architecture doc, Modules: inputs, Component library row; Decisions log row 1 | M1 | unit | n/a |
| LIB-005 | Every component type shall name its fidelity tiers, and a component instance shall select one of the tiers its type names. | Architecture doc, Modules: inputs, Component library row | M1 | unit | n/a |
| LIB-006 | Adding a higher fidelity tier to a component type shall leave the results of every lower tier of that type unchanged. | Architecture doc, Repository starting point, invariant 6 | M1 | invariant | n/a |
| LIB-007 | A user shall be able to add a new component type through the plugin interface without changing any file outside the plugin and its catalog entry. | Requirements doc, Component library requirements; Architecture doc, Modules: inputs; ruling R-09 | M1 | unit | n/a |
| LIB-008 | A built-in component type and a user-added component type shall implement the same plugin interface and shall be checked by the same reference tests. | Ruling R-09 | M1 | unit | n/a |
| LIB-009 | A user-added component type written in Python shall run through the per-tick slow path without being compiled into the core runtime. | Ruling R-09 | M2 | unit | n/a |
| LIB-010 | Every shipped catalog file shall be YAML and shall validate against the JSON Schema for its component type. | Decisions log row 29; Architecture doc, Repository starting point, invariant 3 | M1 | invariant | n/a |
| LIB-011 | Every component type shall define its parameters as a pydantic v2 model from which its JSON Schema entry is generated. | Ruling L4; ADR 0012; Architecture doc, How to extend, "A component type" row | phase 1 | unit | n/a |
| LIB-012 | The library shall parse every quantity in a catalog or instance file from a unit string with pint at load and hold it as a plain SI float afterward. | Ruling L5; ADR 0012; Requirements doc, Component library requirements | phase 1 | unit | n/a |
| LIB-013 | A unit string whose dimension does not match its parameter shall be rejected at load with an error naming the file, the parameter and the expected dimension. | Ruling L5; Requirements doc, Component library requirements | phase 1 | unit | n/a |
| LIB-014 | A missing required parameter shall be rejected at load with an error naming the file, the component type and the parameter. | Requirements doc, Component library requirements | M1 | unit | n/a |
| LIB-015 | An impossible parameter value, such as a negative mass or a step angle outside its type's declared range, shall be rejected at load with an error naming the file, the parameter, the value and the violated bound. | Requirements doc, Component library requirements | M1 | unit | n/a |
| LIB-016 | Every parameter of every component type shall be addressable by a sweep or variation definition and overridable for one run without editing the catalog. | Requirements doc, Component library requirements | M2 | unit | n/a |
| LIB-017 | A sweep shall be able to substitute one catalog part for another of the same component type, such as one stepper size for another. | Requirements doc, Component library requirements | M2 | unit | n/a |
| LIB-018 | The shipped catalog shall provide the types rigid body, revolute, prismatic, fixed, planar and spherical joints, stepper (behavioral), stepper driver, belt and pulley, lead screw, limit switch, encoder, power supply and E-stop relay. | Architecture doc, Modules: inputs, Component library row; Repository starting point, milestone 1 paragraph | M1 | unit | n/a |
| LIB-019 | The shipped catalog shall provide the types load cell, load-cell amplifier, IMU and watchdog. | Architecture doc, Modules: inputs, Component library row; Requirements doc, Instructions for the planning session, milestone plan item 2 | M2 | unit | n/a |
| LIB-020 | The shipped catalog shall provide the types flexible beam, hook and rope, the rope carrying EA, axial damping, mass per length, an optional breaking tension (absent: never snaps, ruling 19) and the mass of its top end fitting (`end_fitting_mass`, default 1 g: the shackle a case payload's rope hangs from, BLD-036, printed in the run's notes). | Architecture doc, Modules: inputs, Component library row; Requirements doc, Instructions for the planning session, milestone plan item 3; ADR 0016 | M3 | unit | n/a |
| LIB-021 | Every shipped component type shall have a reference test that compares it with an analytical result or a datasheet curve within a tolerance stated in this spec. | Architecture doc, How to extend, "A component type" row | M1 | invariant | n/a |
| LIB-022 | The behavioral stepper type shall reproduce the pull-out torque of its catalog part against the published torque-speed curve. | Architecture doc, Technology recommendation, Validation paragraph; Requirements doc, Component library requirements, Actuators row | M1 | reference | not set (owed by owner) |
| LIB-023 | Joint types shall carry range, friction, damping and backlash parameters, and revolute, prismatic, fixed, planar and spherical joints shall support travel limits. | Requirements doc, Component library requirements, Joints row | M1 | unit | n/a |
| LIB-024 | Transmission types shall carry ratio, efficiency, stiffness, backlash, self-locking and slip parameters, with slip zero by default. | Requirements doc, Component library requirements, Transmissions row | M1 | unit | n/a |
| LIB-025 | Sensor types shall carry resolution, noise, delay, sample rate and failure-mode parameters. | Requirements doc, Component library requirements, Sensors row | M2 | unit | n/a |
| LIB-026 | Drivers and sensor front-ends shall be component types separate from the motors and sensors they serve. | Decisions log row 10 | M1 | unit | n/a |
| LIB-027 | Power and safety hardware types shall carry cutoff delay, brake torque and heartbeat timeout parameters where they apply. | Requirements doc, Component library requirements, Power and safety hardware row | M2 | unit | n/a |
| LIB-028 | A catalog part shall be referenceable from an imported STEP body, so that the body takes its component type and parameters from the catalog part. | Decisions log row 19; Architecture doc, Modules: inputs, Design files row | M4 | unit | n/a |
| LIB-029 | The rigid body type shall carry mass, center of mass, inertia, geometry and material parameters. | Requirements doc, Component library requirements, Bodies and structure row | M1 | unit | n/a |
| LIB-030 | The flexible beam type shall carry cross-section and material parameters in addition to the rigid body parameters. | Requirements doc, Component library requirements, Bodies and structure row | M3 | unit | n/a |
| LIB-031 | The library shall model rigid payloads with mass and attach point, and end effectors with capacity and attach and release events. | Requirements doc, Component library requirements, Loads and End effectors rows | M2 | unit | n/a |
| LIB-032 | The flexible beam type shall reproduce cantilever and simply supported beam deflection and first natural frequency against their closed-form results. | Architecture doc, Technology recommendation, Validation paragraph; Requirements doc, Dynamics | M3 | reference | not set (owed by owner) |
| LIB-033 | No pint quantity or unit string shall be reachable from a loaded component type's parameter values. | Ruling L5; ADR 0012, Consequences | phase 1 | unit | n/a |
| LIB-034 | A part of a type whose ratings require a torque-speed curve shall carry one that starts at speed 0 with the part's holding torque, speeds strictly increasing; above the last speed no torque is available. | Queue, rulings during milestone 1 wave 1 (catalog alignment: holding torque alone stalls any motion above zero speed) | M1 | unit | n/a |
| LIB-035 | A compliant transmission type (belt_pulley, lead_screw) shall take `damping_ratio`, the damping ratio of its driven mode on its own stiffness (dimensionless in [0, 1], tier compliant, default 0: undamped), overridable per instance like its other parameters. | Ruling 78 (round 5); brief M3-D2 item 2 | M3 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Operator link component type | `uart_link` (catalog `uart_link.yaml`, pins uart_tx and uart_rx, parameter baud) ships in milestone 1 under SIG-029; LIB-018's milestone 1 type list should name it; no LIB row yet | owner |
| Stepper pull-out torque tolerance against the published curve (LIB-022); proposed 10% of the curve value per speed point | not set | owner |
| Flexible beam deflection and first-mode tolerance (LIB-032) | proposed 2 percent, holding for N >= 8 (ruling 20; T4, owner to confirm; -1.56 % midspan chain error at 8, -6.25 % at 4), derived as for BLD-015 to BLD-017 (`docs/spec/bld.md`): the chain's largest error at N = 8 is the midspan deflection's -1/N^2 = -1.56 % | owner |
| Milestone 3 structural types (LIB-020, LIB-030) | `flexible_beam` (category body: area, second_moment, length, elastic_modulus, density; mass from geometry; parts `steel_bar_10mm_1m` exact, `extrusion_2020_500mm_generic` placeholder), `hook` (end_effector: mass, capacity; mounts attach and rope; `hook_5kg_generic` placeholder), `rope` (new category rope requiring axial_stiffness, damping, mass_per_length, rest_length; optional breaking_tension in its own tier `breaking`, ruling 19; `rope_3mm_steel_generic` placeholder) | gantry team (part values) |
| Fidelity tier names per component type | set per type in `mechsim/library/catalog/*.yaml` (phase 1 C2); new types name their own | library briefs, per type |
| Milestone 2 catalog part values (LIB-019, LIB-025, LIB-027) | `load_cell_5kg_generic`, `load_cell_amplifier_generic`, `imu_6axis_generic`, `brake_24v_generic` carry placeholder values listed in each part's `placeholders` | gantry team |
| IMU interface | analog output per axis until the chip vocabulary has a serial peripheral mode (open ruling 2, `docs/queue.md`) | owner |
| Joint a brake acts on | design data, an `acts_on` block analogous to DES-043 `senses`; not in the catalog type | integration brief, DES |
| Milestone 2 reference tolerances (LIB-021) | load cell bridge voltage 1e-15 V and ADC count exact; amplifier step response to its exact discretization 1e-12, and 1 - 1/e at the sample nearest 1 / (2 pi f) within one sample's increment; IMU at rest within half a resolution step, free fall below 1e-10 m/s^2 (rounding of the difference quotient), gyro within half a resolution step; brake hold torque 1e-9 N m and sag 1e-12 rad, slip torque 1e-6 N m (`core/signals/tests/test_reference_m2_sensors.cpp`, brief M2-A2); the milestone 1 types' reference tests are still owed, so the LIB-021 stub stays | owner |
| Encoder sample rate (LIB-025) | `encoder_600ppr_generic` placeholder 1 kHz, the signal layer rate (a rate above it is a configuration error, SIG-004) | gantry team |
| Linear encoder (LIB-025, SIG-011, ruling 32) | type `linear_encoder` (sensor, `resolution` and `noise` lengths, the core's `incremental_encoder` with `linear: true`); part `linear_encoder_generic` 1 / 60000 m per quadrature count (the firmware's 60000 counts per metre), 1 kHz, values placeholders; the gantry's three encoders sense the axis joints (frame side) with it (brief M2-X5) | gantry team |
| Payload and end effector (LIB-031) | categories `payload` (requires `mass` and an `attach` mount) and `end_effector` (requires `capacity` of dimension mass, an `attach` mount and `events: [attach, release]`; no other type lists events); parts `payload_2kg_generic` (2 kg, 0.1 m box) and `hook_generic` (capacity 5 kg, mass 0: the hook body carries it), placeholders; the gantry's `hook_effector` instance on the hook body. Attach is the case payload attached at t = 0, release its `payload_change` to 0 kg (TST-020); a case payload naming its catalog part and end effector is not yet in the case schema (brief M2-X5) | gantry team; case reference owed |
| Strict lower bounds | a quantity parameter may set `minimum_exclusive: true`, so a value equal to its minimum is rejected (used for the strictly positive parameters of the milestone 2 types) | set (brief M2-G) |
| Rope parameters and model detail | not set | milestone 3 ADR (ruling P4) |
| Datasheet curve source for each shipped stepper part | not set | owner |
| One-run override and part substitution form (LIB-016, LIB-017) | `instance:<instance>.<parameter>` and `instance:<instance>.part` in `--set`, a case's `parameters`, sweeps and variations; checked by `Registry.parameter_override` and `Registry.substitute_part`; stored in the design instance's `overrides`/`part`, resolved to SI by the catalog; the shipped catalog has one stepper part, so a substitution needs a second part of the type | M2-D; second stepper part owed by the gantry team |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Runtime behavior of sensors, drivers, rails and safety hardware | SIG |
| C++ promotion of built-in component types | ENG, KER (core runtime) |
| Sweep and variation execution | TST |
| Assembling instances into a design | DES |
| Turning instances into the neutral physics model | BLD |
| Thermal behavior beyond the constant slot | Roadmap |
