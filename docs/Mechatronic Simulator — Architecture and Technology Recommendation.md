# Mechatronic Simulator — Architecture and Technology Recommendation

Nov 1, 2026 · @Tex Bonham

## Purpose: the founding document for the repository

This doc is the basis a new project repository starts from and expands on. It is written for two readers: the humans who will build and use the simulator, and the Claude agents who will be handed this doc and asked to create the repository, implement a module or add a component. Read it top to bottom once; afterwards the Decisions log, the module tables and the Repository starting point are the sections to return to.

It fixes what the simulator is, the modules it is made of, the interfaces between them, the technology it is built on and the invariants every change must keep. It answers deliverables 1 (technology recommendation) and 2 (architecture) from the companion Mechatronic Simulator Requirements doc of Nov 1, 2026, which holds the gantry reference design, the G1 to G16 test suite and the acceptance criteria. File formats, the milestone plan and the full risk list are the next deliverables; an agent asked to produce them should derive them from this doc and the requirements doc together.

Three rulings extend the requirements doc, and the architecture is shaped by them: the controller is the real C/C++ firmware, run natively or under emulation; structural checks are done in-house because the tool replaces Fusion Simulation and Simscape; and the materials model is process-aware. Where a ruling changes a requirement, the Decisions log says so, and the requirements doc should be read with this doc taking precedence.

## Decisions log

These rulings were made in the planning session of Nov 1, 2026 and are the basis for everything below. Rows marked "changes doc" alter a requirement in the requirements doc.

| # | Decision | Ruling | Changes doc |
| --- | --- | --- | --- |
| 1 | Product scope | General mechatronic simulator; the gantry is the first customer. Real chips, motor heating and material stress are the roadmap, with seams reserved in v1. | No |
| 2 | Build vs. orchestrate | Build on an existing physics engine and an existing MCU emulator; the tool's own work is integration, the library and the test layer. | No |
| 3 | Controller code | The real C/C++ firmware that will run on the chip. No scripting-API controllers. | Yes |
| 4 | Controller modes | Both: native (firmware compiled for the host behind a board-interface shim) and emulated (real binary on an MCU emulator). | Yes |
| 5 | Chip | STM32, chosen for emulator support; F4 or H7 on a Nucleo board recommended; part not yet picked. | No |
| 6 | Operator link | UART. | No |
| 7 | Signal layer | Supports step/dir, timer encoder mode, external counter ICs, HX711-style protocols, ADC, UART and USB CDC. | Yes |
| 8 | Board interface | Defined by the simulator; nothing in the firmware depends on a HAL yet. | Yes |
| 9 | Stateflow logic | Rewritten in C/C++, not imported. | No |
| 10 | Drivers and sensor front-ends | Library components, modeled separately from motors and sensors. | Yes |
| 11 | Chip support | A chip is data: emulator platform description, SVD register map, pin map, plus any missing peripheral models. | Yes |
| 12 | Power loss | Modeled through the power rail: supply and hold-up or brownout hardware are components; the chip's brownout-reset threshold is a chip parameter. | No |
| 13 | Stepper fidelity | Behavioral in v1 (torque-speed curve; stall or missed step when demand exceeds available); phase-current tier later. | No |
| 14 | Encoder placement | Undecided; motor shaft or axis side is a design parameter and both are supported. | No |
| 15 | Load holding on power cut | Self-locking lead screw. | No |
| 16 | Gyro | IMU on the hook, measuring load swing for test G12. | No |
| 17 | Flexible members | Which members, their sections and whether the 1 cm target is static or dynamic are not yet known. Planning default: rails flexible, extrusion sections, static target with dynamic reported alongside. | No |
| 18 | Flexible model | Reduced model tuned to a known deflection and first natural frequency. | No |
| 19 | CAD body as catalog part | Yes: an imported STEP body can be declared a catalog part ("this is a NEMA 17"). | No |
| 20 | Process-aware materials | In v1: print orientation, walls and infill; machined; molded; direction-dependent strengths. | Yes |
| 21 | Existing models | No Fusion model and no Simscape model of the gantry exist. | Yes |
| 22 | Mass properties | Trust Fusion/URDF values; recompute from STEP as a cross-check. | No |
| 23 | Pipeline | The tool replaces Fusion Simulation and Simscape. Fusion 360 stays as the CAD source; parts are modeled there and imported. | Yes |
| 24 | Validation | Analytical references, then hardware measurements once the gantry exists. Acceptance criteria 3 and 4 are rewritten. | Yes |
| 25 | Team and scale | Built by the project owner; small team; under 50 runs a day. | No |
| 26 | GUI editor | Not required; preferred by the end of v1. Data files stay the source of truth. | No |
| 27 | Interface | Desktop app; a browser-based 3D viewer only if easier. | No |
| 28 | Contact | Floor contact plus collisions (hook catching an obstacle, carriage hitting a hard stop). | No |
| 29 | File format | YAML with JSON Schema validation. | No |
| 30 | Physics engine | Own physics engine in C++20 with Eigen, pybind11 and CMake, in its own CMake tree in this repo; sanitizers, clang-tidy and warnings as errors from the first commit; lumped domain models are ours; a calibration spike precedes phase 1 (docs/adr/0001-own-physics-engine.md). | Yes |
| 31 | Collision and contact | Collision detection is in-house with a reference library only as a test oracle; v1 contact is primitives and convex hulls with soft contact and implicit integration (docs/adr/0002-collision-and-contact-scope.md). | Yes |
| 32 | Physics model | The builder emits an engine-neutral physics model that each adapter translates, replacing MJCF output; integrator is per adapter (docs/adr/0003-neutral-physics-model-and-adapters.md). | Yes |
| 33 | MuJoCo role | A thin MuJoCo adapter carries milestones 1 and 2 and is the own engine's differential oracle, retired once the own engine passes the analytical suite and matches the gantry baselines (docs/adr/0004-mujoco-adapter-lifecycle.md). | Yes |
| 34 | Core runtime and language | Engine, kernel, built-in signal components and both controller runtimes form one C++ library with one Python binding; Python for everything else; component plugins are tiered (docs/adr/0005-core-runtime-and-language-split.md). | Yes |
| 35 | Headless core | No Qt or display dependency outside viewer code, one serializable facade, replay from archives, a recorder streaming seam from milestone 1, and the CLI as first client (docs/adr/0006-headless-core-and-facade.md). | No |
| 36 | Front end | A browser client over a local Python service, replaceable by a standalone app, plus a minimal C++ engine debug viewer; PySide6 and MuJoCo's viewer are dropped (docs/adr/0007-front-end.md). | Yes |
| 37 | Spec first | One spec per module with numbered, traced requirements, strict expected-failure stubs before code, a traceability test, and four test tiers (docs/adr/0008-spec-first-and-test-tiers.md). | No |
| 38 | Mechanical integrity | Checks are data-driven, tiered plugins with analytical references; the structural module becomes integrity/ with a milestone 3 and 4 check set (docs/adr/0009-mechanical-integrity-scope.md). | Yes |
| 39 | Mid-run failure | A kernel party detects member limit crossings every tick from live loads, and failures change the model; physics publishes live member end-loads (docs/adr/0010-mid-run-structural-failure.md). | No |
| 40 | Firmware build and native tick | C11 firmware against a C board interface, CMake host and target presets, a polled fixed-rate native tick through a plain C struct, vendor HAL only under firmware/board/\<chip>/ (docs/adr/0011-firmware-and-native-tick-contract.md). | Yes |
| 41 | Data and tooling conventions | Python 3.12 with uv, ruff and mypy; pydantic-generated schemas; unit strings in files; HDF5 archives; one seed per run; dotted signal names; YAML baselines in git; Ubuntu CI; merge-commit round branches (docs/adr/0012-data-conventions.md). | No |
| 42 | Product scope (R-01) | Standalone general mechatronic simulator; the gantry is the first test case and reference design, never the design target. | No |
| 43 | Firmware location (R-02) | Firmware lives in this repo under firmware/, split out only if it becomes independently useful. | No |
| 44 | Second design (R-03) | A pendulum design under designs/pendulum/ from milestone 1 as the rope analytical reference and first generality guard; the two-joint arm remains the milestone 5 proof. | No |
| 45 | Own-engine integrator | Semi-implicit Euler with implicit springs and contacts, post-step projection, projected Gauss-Seidel with a fixed cap, Newmark reserved as a later tier, four lumped-beam segments by default (docs/adr/0013-own-engine-integrator.md). | No |
| 46 | MuJoCo bridge location | Neutral-model-to-MJCF translation stays in Python; MuJoCo stepping runs in C++ inside the core behind the PhysicsAdapter interface (docs/adr/0014-mujoco-bridge-split.md). | Yes |
| 47 | Facade protocol | Proposed in docs/adr/0015-facade-protocol.md: JSON messages over WebSocket with binary frames for time series; owner ruling pending. | No |

## System overview

The tool is a desktop simulator that runs a mechatronic design's real firmware against a physics model of its structure, actuators and sensors, and scores the result against test cases. Engineers use it to size hardware, check structures and prove controller logic before building anything.

The core loop: import parts and CAD, wire and mount them in a design file, point the design at the firmware, run a test case, read the report and replay, change the design or the firmware, run again. Everything in the architecture serves that loop.

One rule governs the whole design: the physics model and the controller touch only through electrical signals. The controller receives switch levels, quadrature edges, ADC samples and UART bytes, and emits step pulses, direction levels, enable lines and UART bytes. It never reads a true position, and it never emits "move to x". This is what lets the same firmware run on the bench, in native mode and in emulated mode without change, and it is why the signal layer is a module in its own right.

Two dials run through every module. Fidelity: each component exists at a behavioral tier in v1 with a slot for a higher tier later (a behavioral stepper now, a phase-current stepper later). Controller mode: native for everyday iteration and the batch suite; emulated for sign-off, where the real binary runs on an emulated STM32.

## Module map

&#91;embedded content: module map · 5 inputs, 6 core modules, 5 outputs\]

Inside the kernel, physics and the signal layer exchange physical state and actuator torques; the signal layer and the controller exchange pin inputs and pin outputs. The highlighted signal layer is the only path between them, in both controller modes.

## Modules: inputs

Five inputs, all data files except the firmware. Nothing gantry-specific lives in code; the gantry is one design file, one test suite and one firmware tree.

| Module | What it holds | Consumed by | v1 contents |
| --- | --- | --- | --- |
| Component library | Parameterized component types, each with facets: mechanical (mass, mounts, shaft axis), electrical (pins, signals it emits or consumes), ratings (absolute maximums, SOA, holding torque), and a thermal slot held constant in v1. Each type names its fidelity tiers. User-added types through a plugin interface. | Model builder, signal layer | Rigid body, flexible beam, revolute/prismatic/fixed/planar/spherical joints, stepper (behavioral), stepper driver, belt and pulley, lead screw, limit switch, encoder, load cell, load-cell amplifier, IMU, power supply, E-stop relay, watchdog, hook, rope |
| Materials database | Per material: density, modulus, Poisson's ratio, yield, ultimate and shear strength, S-N fatigue curve, thermal conductivity, specific heat, expansion coefficient, max service temperature, each with min/typ/max spread and temperature dependence. Process attributes (printed: orientation, walls, infill; machined; molded) with direction-dependent strengths. | Model builder (mass, inertia), structural analysis | Aluminum extrusion grades, steel fastener grades, PLA/PETG/ABS printed with orientation and infill, common bearing steels |
| Design files (YAML) | Bodies with material and process; joints and mates with limits, friction, backlash; component instances with parameters; cross-domain bindings (shaft to joint, housing to mount face); pin-binding table (signal to MCU pin and peripheral mode); power hardware and rails; environment (gravity, ambient); design version. CAD arrives through STEP (geometry, collision meshes, mass cross-check) and URDF (bodies and joints); an imported body may be declared a catalog part. | Model builder | The gantry: frame, rails, trolley, lift, hook, two belt axes, one lead-screw axis, sensors, E-stop relay, STM32 pin map |
| Test case files (YAML) | Design version, initial state, timeline of operator commands, timeline of injected faults, pass criteria with measured value and limit, random seed, sweep and variation definitions, regression baseline. | Test runner | G1 to G16 |
| Firmware | The C/C++ source tree written against the board interface, plus a chip definition: emulator platform description, SVD register map, pin map. The simulator builds the source for the host in native mode and takes the compiled .elf in emulated mode. | Controller runtime | The gantry controller; STM32 F4 or H7 chip definition |

## Modules: core

Six core modules. The co-simulation kernel owns time; the signal layer is the only path between physics and controller; everything else is a plugin behind an interface.

| Module | Responsibility | Interface in | Interface out |
| --- | --- | --- | --- |
| Model builder | Turns a design file into runnable models: validates every parameter and binding, resolves catalog parts, computes mass and inertia from geometry and material, generates collision meshes, builds the lumped structural model for flexible members, and emits one model per engine. Fails loudly on a missing material, an unconstrained body, overlapping bodies or a signal with no pin. | Design file, component library, materials database, CAD files | Physics model, signal-layer wiring, chip and pin configuration, structural member list |
| Co-simulation kernel | Owns simulated time and steps every party at its own rate: physics at about 1 kHz, pulse-level signals at their own finer rate, the controller at its tick, the emulator in chunks of virtual time. Exchanges coupling variables at sync points. Deterministic given the same seed. Reports constraint drift and solver failure as run errors, never as silent bad numbers. | Models from the builder, run configuration (rates, seed, mode) | Ordered sequence of ticks; signals to the recorder |
| Physics engine (adapter) | Rigid multibody dynamics with joints, friction, limits and contact; ropes and cables with length change and slack; flexible members as reduced models; collisions between bodies. One adapter per engine so the engine can be swapped. | Physics model, actuator torques and forces from the signal layer, injected physical faults (slip, stall, payload change) | Body poses and velocities, joint forces, contact forces, rope tension, member end-loads |
| Signal layer | Every electrical and power path. Sensor models turn physical state into signals with resolution, noise, delay and failure modes. Front-end components (drivers, amplifiers, encoder interfaces, debounce) sit between physics and pins. Power rails carry supply voltage; the E-stop relay and watchdog gate the motor rail without the controller. The operator link encodes scripted messages into UART bytes and parses status back. The pin-binding table maps every signal to a pin and peripheral mode. | Physical state from physics, pin outputs from the controller, operator commands, injected signal faults (stuck, noisy, dropped, power loss) | Pin inputs to the controller, actuator torques and forces to physics, operator status messages, every signal to the recorder |
| Controller runtime | One interface, two implementations. Native: the firmware built for the host as a shared library, called each tick through the board-interface shim, which reads the pin-binding table and presents signals as the interface's values. Emulated: the real .elf running on the MCU emulator, with the signal layer wired to emulated peripherals (GPIO, timer encoder and PWM channels, ADC, USART). Freeze, reset and brownout are operations on this module in both modes. | Pin inputs, power rail state, control commands (freeze, reset), firmware build or binary | Pin outputs, controller state and fault log (native: through the shim; emulated: through a debug channel) |
| Test runner | Loads a test case, sets the initial state, plays the operator command timeline through the operator link, fans injected faults out to the right module, runs the kernel to the end condition, then hands the recording to scoring. Runs suites, sweeps and seeded variations in batch, and diffs a suite against its regression baseline. | Test case file, design version, controller mode | Run configuration to the kernel, commands and faults to modules, run results to scoring and regression |

## Modules: outputs

The recorder is the single sink; the other four read from it. Structural analysis is in-house because the tool replaces Fusion Simulation; FEA load export stays as an optional output for anyone who still wants it.

| Module | Responsibility | Reads | Produces |
| --- | --- | --- | --- |
| Recorder | Time-series store for every run: body poses, velocities, joint and contact forces, member end-loads, every signal on every pin, power rail state, controller states, faults and safety events, operator messages, all with the simulated time and the run's seed and design version. | Kernel, physics, signal layer, controller runtime | A run archive; CSV, JSON and MAT exports |
| Structural analysis | For each structural member and mount: peak stress from the member end-loads over the run using beam and plate theory on the member's section, deflection, and safety factor against the material's process- and direction-dependent limits; fatigue cycles against the S-N curve. Applies the design-load rule (for the gantry, rated load plus 15%). Flags members under the minimum safety factor or over the deflection target. | Recorder (member loads), materials database, design file (sections, design-load rule) | Per-member table of peak stress, deflection and safety factor; load-case export (location, direction, magnitude per joint and mount) |
| Scoring | Evaluates each pass criterion in the test case against the recording: measured value beside its limit, pass or fail, with the time of the worst case. Aggregates a suite and diffs it against the regression baseline. | Recorder, test case file, structural analysis | Pass/fail report per case and per suite; regression diff |
| Viewer and replay | 3D replay of the run with an event timeline (commands, faults, safety events, criterion failures) and probes on any body, joint, pin or signal. Scrub, pause, step. Overlays for member stress and swing angle. | Recorder, design file (geometry) | On-screen only |
| Exports | CSV, JSON and MAT files of any recorded series; the load-case file; the pass/fail report as a shareable document. | Recorder, structural analysis, scoring | Files |

## Data flows

One run, end to end:

1. The test runner loads the test case and its design version; the model builder validates the design and emits the physics model, signal wiring and chip configuration.
2. The controller runtime builds the firmware for the host (native) or loads the .elf into the emulator (emulated), then resets it with the power rail up.
3. The kernel starts at t = 0 and advances tick by tick (below), while the test runner feeds operator commands and faults at their scheduled times.
4. The recorder captures every tick; the run ends at the test case's end condition or on a solver error.
5. Structural analysis computes per-member stress, deflection and safety factor from the recorded loads; scoring evaluates the pass criteria and the regression diff.
6. The viewer loads the run archive for replay; exports are written on request.

Within one physics tick (about 1 ms), in order:

1. Physics advances by one step and publishes poses, velocities and forces.
2. Sensor models sample the new state and update their signals (a switch level, encoder edges accumulated over the step, an ADC sample, an IMU reading), applying noise, resolution, delay and any active fault.
3. The controller runtime advances by the same interval: native mode calls the firmware's tick function; emulated mode runs the emulator for 1 ms of virtual time with the pin inputs applied.
4. The signal layer collects pin outputs: step pulses are counted over the interval by the driver model, direction and enable levels are latched, UART bytes are parsed by the operator link.
5. Driver and transmission models turn the pulse count into motor torque and then joint force for the next physics step; the power rail state gates whether any torque is applied at all.

Injected faults fan out to the module that owns the thing being broken:

| Fault in the test case | Lands in | How |
| --- | --- | --- |
| Sensor stuck, noisy, dropped | Signal layer | Sensor model's failure mode set for the window |
| Belt or rope slip | Physics | Transmission model slips for the window |
| Motor stall or missed steps | Physics (behavioral stepper) | Available torque reduced or demand exceeds it |
| Power loss and restore | Signal layer, then controller runtime | Rail voltage drops; below the chip's brownout threshold the runtime resets the controller |
| Controller freeze | Controller runtime | Native: tick function not called; emulated: core halted |
| E-stop press | Signal layer | Relay opens the motor rail; the controller sees it only through its own inputs |
| Payload change mid-run | Physics | Payload mass or attachment changed at the scheduled time |

## Technology recommendation

Superseded in part on 2026-11-02: the physics engine recommendation (MuJoCo as the v1 engine) is replaced by an own C++ engine, ADR 0001, with MuJoCo kept as a bridge and oracle per ADR 0004; the builder's output and the adapter model by the engine-neutral physics model, ADR 0003; Python for the kernel and signal layer by the C++ core runtime, ADR 0005; the PySide6 shell and MuJoCo viewer by a browser client and an engine debug viewer, ADR 0007. The comparison table and prose below are kept as the historical record.

Recommendation: Python for the application, MuJoCo as the v1 physics engine behind an adapter, Renode for emulated mode, a PySide6 desktop shell, YAML files with JSON Schema. Project Chrono is held as a second physics adapter if flexible-member fidelity ever needs true beam elements.

**Language.** Python for every module the tool owns: model builder, kernel, signal layer, test runner, structural analysis, scoring, viewer shell. The hot loops live inside the physics engine and the emulator, both compiled; what remains is per-tick arithmetic on a few hundred signals, which numpy handles at well over real time. Under 50 runs a day on a small team makes development speed and a clear codebase worth more than raw throughput. If the signal layer ever becomes the bottleneck, that one module can move to C++ or Rust with Python bindings without touching the rest. The firmware is C/C++ by ruling.

**Physics engine.** Three candidates were compared against the requirements that matter here. Facts are from each project's own documentation as of Nov 1, 2026 (links in Sources).

| Criterion | MuJoCo 3.9 | Project Chrono 10 | Drake 1.53 |
| --- | --- | --- | --- |
| Ropes and swinging loads | Tendons with pulleys and wrapping; a native cable model and composite flexible objects that interact fully with the rest of the scene. Good for a hoist rope. | Cable and beam finite elements in the FEA module. Highest fidelity. | No native cable; a rope is a chain of bodies and joints. Workable, tuned by hand. |
| Flexible beams | Composite flexible objects or a tuned spring-damper joint pair. Fits the v1 ruling (reduced model tuned to deflection and first natural frequency); not an engineering beam element. | Euler-Bernoulli, Timoshenko/Cosserat and IGA beam elements with section properties. Engineering-grade. | Volumetric deformable bodies (FEM); no beam elements. |
| Contact and collisions | Soft convex contact in generalized coordinates; fast and stable at 1 ms steps; mesh collision through convex decomposition. | Penalty and complementarity contact; robust, more tuning. | Hydroelastic and point contact; robust, slower. |
| Actuators and transmissions | General actuator model with transmission types; joint and tendon limits, dry friction. Steppers and drivers are modeled in the signal layer either way. | Motors, gears and belt links as native objects. | Actuation through the systems framework. |
| Determinism and solver health | Deterministic for a fixed model and seed; constraint solver reports available. | Deterministic; solver settings exposed. | Deterministic; strong integrator diagnostics. |
| Python and install | pip wheels on Windows, macOS and Linux; URDF loads directly; built-in OpenGL viewer. | PyChrono through conda only (the PyPI name is a placeholder as of Jul 2026); cross-platform. | pip wheels on Ubuntu and macOS; no native Windows. |
| License | Apache 2.0 | BSD-3 | BSD-3 |
| Fit for a small team | Smallest API, best documentation, fastest to a first running gantry. | Largest API; forum reports of version drift in PyChrono. | Excellent systems framework for fixed-rate controllers, but the platform gap and missing cables rule it out for v1. |

Verdict: MuJoCo for v1. It installs everywhere the team works, its tendons map directly onto belts, pulleys and the hoist rope, contact is robust at the 1 ms step the kernel needs, and it loads the URDF the design pipeline produces. Its weakness, the absence of engineering beam elements, does not bite in v1: the ruling is a reduced flexible model, and structural stress is computed by the tool's own structural analysis from member loads, not by the engine. Chrono is the named fallback: if milestone 3 shows the reduced model cannot match the rails' measured deflection and first mode, a second adapter brings in Chrono's beam elements without changing any other module. Drake's one idea worth taking is its systems framework, where every component is a block with periodic discrete updates; the kernel's design borrows that shape.

**MCU emulation.** Renode. It has a central virtual time controller, so a run is deterministic; an External Control API whose client library exposes run-for and get-current-time calls, which is exactly the lockstep the kernel needs, with an ADC example in the same tree; and pyrenode3, a Python library that exposes Renode's entire internal API through Python.NET, for injecting pin levels and UART bytes and reading timer and GPIO state. STM32 coverage is strong and actively improved, with an H753 reference platform published in Jul 2026. The lockstep loop: for each kernel tick, apply pin inputs through the API, run the emulator for the tick's virtual time, read pin outputs. The first emulated-mode task is a smoke test of the gantry's exact peripherals on the chosen part: timers in encoder mode, PWM output, ADC, USART, independent watchdog. Support is per peripheral, and this test decides the chip.

**Native mode.** The firmware's application layer is compiled with the host compiler into a shared library and loaded through cffi. The board-interface header is owned by the simulator. Rules that keep native mode honest: fixed-width types, no vendor headers above the interface, the same compiler warnings as the target build, and a build that fails if the application layer includes anything but the interface.

**Viewer and interface.** A PySide6 (Qt) desktop application for the design and test editors, run control, reports and the event timeline. For 3D replay, MuJoCo's built-in viewer is used through milestone 3 so replay costs nothing to build; a proper embedded 3D panel (Qt OpenGL, or a local three.js page if that proves easier) replaces it once the data model is stable. The GUI editor arrives late in v1 as forms generated from the YAML schemas, so it can never hold state the files do not.

**Data and files.** Component, material, design and test files in YAML, validated against JSON Schema with explicit SI units. STEP is read with Open CASCADE through the OCP Python bindings for geometry, mass properties and mesh export; collision meshes come from convex decomposition (CoACD). URDF is parsed directly. Run archives are HDF5, with CSV, JSON and MAT exports through pandas and scipy.

**Validation.** With no Simscape or Fusion model to match, acceptance criteria 3 and 4 become analytical checks: pendulum period and swing decay for the rope, cantilever and simply supported beam deflection and first mode for the rails, stepper pull-out torque against the published curve, settle time of a step response. Hardware measurements replace these as the gantry is built.

## Repository starting point

An agent or a person creating the repository should lay it out as below, enforce the invariants from the first commit, and build the milestone 1 slice before anything else. One Python package, one subpackage per module in this doc, so the code map and the architecture map are the same map.

```markdown
core/                 C++ core runtime: one compiled library, own CMake tree
  engine/             own physics engine: math core, integrators, multibody, collision, contact
  kernel/             co-simulation kernel: time, rates, sync points, seeds
  signals/            built-in signal-layer components: sensors, front-ends, rails, safety hardware, operator link, pin bindings
  controller/         controller runtimes: native shim, Renode bridge
  physics/            C++ PhysicsAdapter implementations: mujoco/ (bridge stepping, ADR 0014), native/ (own engine)
  bindings/           pybind11 binding exposing the core runtime to Python
  viewer_debug/       minimal immediate-mode C++ window for engine development
mechsim/              Python package; one subpackage per module
  library/            component types, plugin interface, shipped catalog (YAML)
  materials/          materials database, process rules, direction-dependent limits
  design/             design-file schema, loader, validation, CAD import (STEP, URDF)
  builder/            model builder: design file -> engine-neutral physics model, wiring, chip config
  physics/            builder-side adapters: mujoco/ (neutral model to MJCF), native/ (pass-through), chrono/ (reserved)
  testing/            test-case schema, runner, fault fan-out, sweeps, regression
  record/             recorder, run archive (HDF5), CSV/JSON/MAT exports
  integrity/          mechanical integrity check plugins, safety factors, load-case export
  scoring/            pass criteria, suite reports, regression diffs
  facade/             facade service and protocol
  viewer/             replay and event timeline, served to clients through facade/
board_interface/      the C header firmware writes against
chips/                chip definitions: platform description, SVD, pin-map template
schemas/              JSON Schema for every YAML file type, generated from the pydantic models
designs/              design files; designs/gantry/ is the reference design, designs/pendulum/ the second
test_cases/           test suites; test_cases/gantry/ holds G1 to G16
firmware/             controller source trees in this repo; vendor HAL only under firmware/board/<chip>/
web/                  browser client
tests/                the simulator's own unit and reference tests
docs/                 this doc, the requirements doc
  adr/                architecture decision records
  spec/               one spec per module, numbered requirements
  rulings/            planning rulings, the handoff records for ADRs
  queue.md            the dispatch queue and next brief
  cost-log.md         session cost log
```

The layout was revised on 2026-11-02 per ADRs 0001, 0005, 0006 and 0007.

Invariants every change must keep:

1. The controller never sees true state. Everything between physics and controller crosses the signal layer as electrical signals through the pin-binding table. No convenience path from controller code to physics state, in either mode.
2. Nothing design-specific in code. The gantry exists only under designs/, test\_cases/ and firmware/. A requirement that can only be met by special-casing a design is a gap in the library and is flagged, not patched.
3. Data files are the source of truth. Every YAML file validates against its schema; SI units internally, units explicit in files; the GUI editor, when it exists, writes the same files.
4. Determinism. The same design, test case, seed and controller mode give an identical recording. Any source of nondeterminism is seeded or removed.
5. Engines and emulators stay behind adapters. No MuJoCo or Renode type crosses the mechsim/physics/ and core/physics/ adapter boundaries or the core/controller/ runtime boundary; the engine-neutral physics model (ADR 0003) is the only thing that crosses into an adapter.
6. Fidelity tiers are additive. Adding a higher tier to a component never changes the lower tier's results.
7. Failures are errors. Solver drift, solver failure and emulator faults end the run with a report; they never produce silent numbers.
8. The board interface is the only coupling to firmware. Firmware application code includes the interface header and nothing else from the simulator or any vendor HAL.
9. The core is headless. Nothing outside viewer code depends on Qt or a display. Every client drives the simulator through one serializable facade; replay reads run archives; the recorder's streaming seam exists from milestone 1. The CLI is the first client and the one CI uses.

Milestone 1, the first vertical slice, is the requirements doc's milestone 1 made concrete: load designs/gantry and designs/pendulum, build the engine-neutral physics model and run it on the MuJoCo adapter (ADR 0004), run a test case against the native controller runtime, record, score, replay from the run archive, and pass G1 to G4 plus the pendulum references. The signal layer at that point needs only limit switches, encoders, step/dir drivers, the UART operator link and a power rail with the E-stop relay. Flexible members, ropes, emulated mode and structural analysis come in later milestones as the requirements doc orders them.

How to extend the repository:

| To add | Touch | Done when |
| --- | --- | --- |
| A component type | library/ plugin class, its schema entry, a catalog YAML, a reference test | The reference test matches an analytical result or a datasheet curve within a stated tolerance |
| A chip | chips/\<name>/ with platform description, SVD and pin-map template | The peripheral smoke test passes in emulated mode for every peripheral the pin map names |
| A physics engine | mechsim/physics/\<name>/ adapter plus its translation of the engine-neutral physics model | The analytical reference suite passes with the new adapter |
| A design | designs/\<name>/ and test\_cases/\<name>/, no code | The design builds, runs and scores with the library as shipped |
| A controller mode | core/controller/\<name>/ implementing the runtime interface | G1 to G4 pass with the same firmware in the new mode |
| A mechanical integrity check | integrity/ plugin class, its schema entry, a reference test | The reference test matches an analytical result within a stated tolerance |

When a question is not settled here, the Decisions log wins over the requirements doc, and anything still open is listed under Risks and open questions. New decisions go into docs/ as architecture decision records and get a row in the Decisions log.

## Seams reserved for the roadmap

Each of these is a slot in the v1 data model or an interface with one implementation, so the roadmap item plugs in without a redesign. None is built in v1.

| Roadmap item | Where the seam is | What v1 does with it |
| --- | --- | --- |
| Thermal engine (motor heating, driver losses, material drift) | A thermal facet on every component type; a thermal party registered with the kernel; loss outputs on driver and motor models | Facet held constant; kernel party absent; losses recorded but not integrated |
| Phase-current stepper with driver current limit and microstepping | Fidelity tier 2 of the stepper and driver types | Tier 1 behavioral model only |
| Engineering beam elements and embedded FEA | A beam-element tier in the own engine, or a physics/chrono/ adapter; integrity/ accepts engine-computed stresses as an alternative source | Multi-segment lumped beams with stress recovery (ADR 0010); beam theory from member loads |
| Wear and backlash growth over cycles | A wear facet on transmission and joint types; a cycle counter in integrity/ | Facet held constant; cycles recorded but not integrated (ADR 0009) |
| Real-chip library growth | chips/ as data; peripheral models contributed to the emulator | One STM32 part with a smoke-tested pin map |
| Hardware-in-the-loop | A third controller runtime implementation driving a real board over a debug probe and serial, with the signal layer as its I/O | Runtime interface designed so the real board is one more implementation |
| Monte Carlo over tolerances | min/typ/max spreads already in the materials and component schemas; the test runner's seeded variations | Spreads stored; variations seeded but nominal values used |
| GUI design and test editors | Forms generated from the YAML schemas in viewer/ | Files edited by hand; schema validation gives the errors |
| Operator app and Bluetooth or joystick input | The operator link's codec and transport are separate classes | UART codec with scripted messages only |

## Risks and open questions

The risks below could block a milestone; the open questions are decisions still owed by the gantry team. The next deliverables are the file-format schemas with the gantry and test G3 as worked examples, then the milestone plan.

| Risk | Blocks | Mitigation |
| --- | --- | --- |
| The chosen STM32 part lacks an emulator model for a peripheral the gantry uses (timer encoder mode, PWM, ADC, USART, watchdog) | Milestone 4, emulated mode | Peripheral smoke test before the part is bought; pick from parts with a Renode platform file; write the missing peripheral from the reference manual as a last resort |
| Emulator and physics time drift apart in lockstep, or the emulated run is too slow to be useful | Milestone 4 | Run the emulator in whole-tick chunks through the run-for call; keep emulated mode for sign-off, native mode for batches |
| The reduced flexible model cannot match the rails' measured deflection and first mode | Milestone 3 | The Chrono adapter with beam elements is the named fallback; the adapter interface exists from milestone 1 |
| Host-compiled firmware behaves differently from the Cortex-M build (type sizes, float, undefined behavior) | Milestone 1 onward | Fixed-width types and the same warnings as the target build; emulated mode catches what native mode cannot |
| STEP import is fragile (bad geometry, missing materials, slow meshing) | Milestone 4 | URDF carries bodies and joints; STEP is used for shapes and a mass cross-check; validation errors on import rather than at run time |
| Process-aware material data for printed parts is thin and varies by printer | Milestone 3 | Ship conservative defaults with wide spreads; let the team override from coupon tests |
| Scope creep from the roadmap items into v1 | Every milestone | The Seams table is the boundary: a slot, not an implementation |
| The own engine's contact solver slips | The own engine becoming the default engine | Per ADR 0004: the MuJoCo adapter keeps carrying runs until the own engine passes the analytical suite and matches the gantry baselines |
| The C++ core runtime delays milestone 1 by the time to build the kernel and native runtime | Milestone 1 | Build the kernel and native runtime concurrently with the engine packages; the MuJoCo adapter carries milestone 1 |
| Agent-written C++ contains undefined behavior | Milestone 1 onward | Address and undefined-behavior sanitizers, clang-tidy, warnings as errors and a coding subset from the first commit |

Open questions, owed by the gantry team:

- [ ] Which STM32 part, after the peripheral smoke test
- [ ] Encoder placement: motor shaft or axis side
- [ ] Which members are flexible, their sections, and whether the 1 cm deflection target is static or dynamic
- [ ] Workspace dimensions and rated load
- [ ] Swing angle and settle time limits for G12, and the minimum structural safety factor
- [ ] Policy for a new target during a move, and for lifting while moving
- [x] Whether the firmware lives in this repository or in its own: in this repository under firmware/ (Decisions log row 43)

## Sources

Engine and emulator facts were checked against these pages on Nov 1, 2026. Requirements, test cases and acceptance criteria come from the companion Mechatronic Simulator Requirements doc; rulings come from the planning session recorded in the Decisions log.

- [MuJoCo 3.9 documentation: overview](https://mujoco.readthedocs.io/en/3.9.0/overview.html) (tendons, cable and composite flexible objects, contact model, URDF loading)
- [MuJoCo repository](https://github.com/jvlmdr/mujoco) (prebuilt binaries for Linux, Windows and macOS; pip install; Python 3.10+)
- [Project Chrono: features](https://projectchrono.org/) (FEA module with beams, cables and shells; BSD-3; PyChrono)
- [PyChrono on Anaconda](https://anaconda.org/channels/projectchrono/packages/pychrono/overview) (version 10.0.0, BSD-3)
- [projectchrono on PyPI](https://pypi.org/project/projectchrono/) (placeholder release while wheels are prepared, Jul 2026)
- [Chrono forum: FEA plasticity in 10.0](https://groups.google.com/g/projectchrono/c/ZKXeQ5bAg5M) (state of the FEA module and beam section framework)
- [Drake v1.53.0 release notes](https://drake.mit.edu/release_notes/v1.53.0.html) and [v1.52.0](https://drake.mit.edu/release_notes/v1.52.0.html) (supported platforms, pip)
- [pyrenode3 repository](https://github.com/antmicro/pyrenode3) and [Antmicro: Python-driven automation with pyrenode3](https://antmicro.com/blog/2023/11/python-driven-automation-and-scripting-in-renode-with-pyrenode3/) (Python access to Renode's internal API)
- [Renode External Control client README](https://opensecura.googlesource.com/3p/renode/renode/+/226477a5f4fd19bf7dae2c8d6bc9abedeb16ab0f/tools/external_control_client) (run-for and current-time calls, ADC example)
- [Antmicro: STM32H7 Renode Reference Platform](https://antmicro.com/blog/2026/07/stm32h7-renode-reference-platform) (STM32 support status, H753 reference board)
- [Antmicro: Renode 1.14 release](https://antmicro.com/blog/2023/09/renode-1-14-release) (STM32F412 and H743 improvements)
- [Antmicro: Precursor and Renode](https://antmicro.com/blog/2022/03/precursor-developing-advanced-products-with-renode/) (central virtual time controller)
