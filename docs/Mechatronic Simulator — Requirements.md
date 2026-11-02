# Mechatronic Simulator Requirements

Nov 1, 2026 · @Tex Bonham

## Project overview

We need a standalone simulator app that can model any mechatronic design, both its components and its physical structure, and run it through repeatable test cases. The app is not built yet; this doc is the requirements guide for planning it.

It is a general tool. Nothing about a specific machine should be hard-coded: designs, components, controllers, and test cases are all data the user supplies.

"Any mechatronic design" means mechanisms built from bodies and joints (gantries, robot arms, linkages, cranes, mobile bases), driven by actuators (motors, belts, lead screws, gears, pneumatics), observed by sensors, run by controller logic the user writes, and tested under loads and faults the user defines.

The first design the app must handle is our senior design project, a gantry-based material handling system, described in the next section. It is the reference design and first test suite: when the app can build, run, and score the gantry, it has reached its first milestone.

## Reference design: the gantry (Version 0)

The gantry is an overhead X-Y machine whose hook lifts, carries, and lowers small loads across a rectangular workspace. It exercises most of what the app must support: multi-axis motion, a suspended load, several sensor types, a safety-critical controller, and a structure that must stay stiff under load.

Its three CORE features become requirements on the app:

- **Routing/Movement:** the operator gives a target (x, y); the controller homes, tracks position, and moves the hook inside software limits.
- **Safety:** an E-stop that cuts motor power independently of the controller, movement limits, an overload check, and stall/position fault detection.
- **Attachments:** Version 0 uses one hook on a lead-screw lift; a standard attachment interface allows other tools later.

Reference systems the team studied: FarmBot Genesis, Gorbel Work Station cranes with the G-Force lift, and the CSUN ARCS concrete printer. Items marked "not set" below should be parameters in the app's gantry model, not fixed values.

| Item | Version 0 decision |
| --- | --- |
| Workspace | Rectangular X-Y area; dimensions not set |
| Rated load | A few pounds; exact value not set |
| Position accuracy | About 1 cm |
| Axes | X and Y gantry travel; vertical motion from a lead-screw hook lift |
| Drives | Stepper motors, NEMA 17 baseline, NEMA 23 if load calculations require; X and Y on timing belts, lift on a lead screw |
| Sensors | Limit switches at each axis end (homing and hard stops), motor encoders, weight sensor (load cell); a gyroscopic sensor was proposed but has no defined role yet |
| Controller | STM32 microcontroller, chip not selected; logic built first in Simulink/Stateflow |
| Operator input | Browser app on phone and laptop, the only input for Version 0; Bluetooth and joystick later |
| E-stop | Physical button that cuts motor power directly, so it works if the controller freezes |
| App visual | Simple 2D top-down view, cosmetic only, drawn from controller-reported position |

Data that moves between parts (these labels should match the simulator's signal names):

- App → Controller: "target (x, y)", "home"
- Controller → Motors: "move command"
- Sensors → Controller: "position, limit hit, weight"
- Controller → App: "current position, ready / fault"
- Controller → E-stop relay: "heartbeat" (stops if the controller freezes)
- E-stop → Motors: "motor power" (cut on press or lost heartbeat)

## Purpose and scope

The app is a development tool: engineers use it to size hardware, check structures, and prove controller logic before building anything. It does not need to connect to a machine's operator app or real hardware.

For any design, it must answer:

1. Is the structure strong and stiff enough under the rated loads, with deflection inside the accuracy target?
2. Are the actuators and transmissions sized for the required speeds, accelerations, and loads?
3. How do dynamic effects behave (swinging or shifting loads, vibration, sudden stops), and how long do they take to settle?
4. Does the controller logic behave correctly in every test case, including faults and safety events?
5. What peak forces reach each joint, mount, and structural member during worst-case events?

In scope: 3D rigid-body dynamics with joints, flexible members, actuators and transmissions, sensors with noise and faults, user-written controller logic, loads, fault injection, batch test runs, reports, and CAD import.

Proposed out of scope for version 1:

- Live hardware connection (hardware-in-the-loop is a later goal)
- A full general-purpose FEA solver; version 1 uses simplified structural checks and imports results from FEA tools
- Fluids, thermal, and circuit-level electronics beyond what affects actuator torque and power cuts
- Code generation for microcontrollers

## Relationship to existing tools and technology options

The team's current pipeline (Fusion 360, Fusion Simulation, Simulink/Simscape) stays in use. The app should import from it and be checked against it, not replace it.

| Existing tool | How the app relates to it |
| --- | --- |
| Fusion 360 CAD | Import geometry and mass properties: STEP for shapes, URDF (e.g., from a fusion2urdf exporter) for bodies and joints |
| Fusion Simulation | Take in stiffness, deflection, and natural frequencies for flexible members; export peak forces from test runs as load cases to check there |
| Simulink/Simscape Multibody | Benchmark: run the same gantry case in both and compare positions, load swing, and forces |
| MATLAB | Export logged results as CSV or MAT files for further analysis |

The team hit three gaps in that pipeline that the app should close: no direct Fusion-to-Simscape export, no automatic transfer of flexible-body data, and a manual load-transfer loop. CAD import and the load handoff should be built-in steps.

Technology is not decided; the planning session should recommend a stack. Options to weigh:

- **Language:** Python for speed of development and scientific libraries, or a C++/Rust core with Python scripting if performance demands it.
- **Physics:** build on an open-source engine rather than writing a solver. Candidates: MuJoCo (joints, contact, tendons, deformable bodies), Project Chrono (multibody plus an FEA module for flexible parts), Drake (multibody and control). PyBullet is less actively maintained.
- **Interface:** a 3D viewer (desktop, or browser-based with three.js) plus editors for designs and test cases.

## Component library requirements

The app needs a library of reusable component types, each defined by parameters, so any design can be assembled from them. The gantry column shows how the reference design maps onto the library.

| Category | Must model | Key parameters | Gantry instance |
| --- | --- | --- | --- |
| Bodies and structure | Rigid bodies; flexible beams and plates for long members | Mass, center of mass, inertia, geometry, material (modulus, yield strength), cross-section | Frame, rails, trolley, hook |
| Joints | Revolute, prismatic, fixed, planar, spherical; travel limits | Range, friction, damping, backlash | X and Y carriages, lift slide, hook pivot |
| Actuators | DC, brushless, stepper, servo motors; linear actuators | Torque-speed curve, holding torque, current limit, step angle, missed-step behavior | NEMA 17 or 23 steppers |
| Transmissions | Belts and pulleys, lead and ball screws, gears, cables and rope | Ratio, efficiency, stiffness, backlash, self-locking | Timing belts on X and Y, lead screw on the lift |
| Sensors | Limit switches, encoders, load cells, IMU/gyro, distance sensors | Resolution, noise, delay, sample rate, failure modes | Limit switches, encoders, load cell, optional gyro |
| Loads | Rigid payloads, loads hung on rope or cable, external forces | Mass, attach point, rope length | A few-pound box on the hook (the team's walkthrough used 3 kg) |
| Power and safety hardware | Power supply, E-stop relay, watchdog, brakes | Cutoff delay, brake torque, heartbeat timeout | E-stop that cuts motor power, controller heartbeat |
| End effectors | Hooks, grippers, tool mounts; attach and release events | Capacity, grip force | Hook |

Library requirements:

- Components are defined as data files with explicit units (SI internally), so designs can be saved, shared, and versioned.
- Users can add new component types (a plugin or subclass interface) without changing the core.
- Every parameter can be swept in a test run (for example, NEMA 17 versus NEMA 23, or rope length).
- Parameters are validated on load, with clear errors for missing or impossible values.

## Controller requirements

The app must run controller logic the user writes, with the same limits a real controller has: it sees only sensor readings, acts only through actuator commands, and runs at a fixed rate.

General requirements:

- **User logic:** controllers are written as code through a scripting API, with built-in support for state machines (states, transitions, timers).
- **Realistic separation:** the controller never reads the simulator's true positions; it gets sensor values with their noise, resolution, and delay.
- **Timing:** a configurable control-loop rate, input-to-output latency, and the ability to freeze or slow the controller on demand (to test watchdogs).
- **Operator interface:** commands in and status out as named messages; test cases script the operator's commands with timestamps.
- **Safety outside the controller:** E-stops, watchdogs, brakes, and hard limit switches are hardware components that act even when the controller is frozen or wrong.
- **Logging:** every state change, command, fault, and safety event is recorded with its time.
- **Portability (nice to have):** controller code structured so it can be translated to a microcontroller later.

The gantry's reference controller, which the app must be able to express:

- Starts "not homed" and refuses every target until homing finishes against the limit switches; after any power loss it returns to "not homed".
- Rejects targets outside the workspace with a fault instead of clipping them to the edge.
- On overload, blocks lifting but still allows lowering so the load can be set down.
- Stops and faults when encoder position disagrees with the commanded position, or a limit switch trips where it should not; then requires re-homing.
- Uses acceleration and deceleration limits, and slows near targets and travel limits.
- After an E-stop, stays stopped until a manual reset and a new command; no automatic resume.
- Handles a new target during a move with one defined policy (cancel and replace, or queue).

Gantry messages: operator to controller "target (x, y)" and "home"; controller to operator "current position, ready / fault"; controller to E-stop relay "heartbeat".

## Physics and structural analysis requirements

The physics layer must handle 3D multibody dynamics with suspended loads and flexible members, and turn the forces it computes into structural checks.

Dynamics:

- Rigid multibody dynamics with joint constraints, friction, gravity, and contact (for example, a payload resting on the floor before pickup).
- Ropes and cables: suspended loads that swing as pendulums, with rope length changes during lifting and slack when unloaded.
- Flexible members: beam models for rails and arms, tuned to match a known deflection and first natural frequency (from hand calculations or FEA).
- Deterministic runs: the same design, test case, and random seed always give the same result.
- Solver health: report constraint drift or solver failure instead of returning bad numbers silently. Runs need not be real time; faster than real time is preferred for batches.

Structural checks:

- For each structural member and mount, compute peak stress, deflection, and safety factor from the forces seen during each test case.
- Flag any member below a user-set minimum safety factor, or with deflection beyond the design's accuracy target.
- Support a design-load rule per design. Example for the gantry: rated load plus 15% for hoist and trolley weight, the rule Gorbel uses for its work station cranes.

Load handoff with FEA tools (closing the team's manual loop):

1. Export peak force and torque at each joint and mount, per test case, as a load-case file (location, direction, magnitude) that can be applied in Fusion Simulation.
2. Import FEA results back (stiffness, deflection, natural frequencies) to tune the flexible members.
3. Record which design version each exchange belongs to, so the two models stay in sync.

## Test cases

A test case is data, not code: a design, an initial state, scripted operator commands, injected faults, and pass criteria. The app runs many of them in a batch and scores each one.

Test system requirements:

- **Definition:** each case names its design version, starting state, a timeline of operator commands, a timeline of injected events, and measurable pass criteria.
- **Fault injection:** sensor faults (stuck, noisy, dropped), actuator stalls and missed steps, belt or rope slip, power loss and restore, controller freeze, E-stop press, payload change mid-run.
- **Batches and sweeps:** run a whole suite at once, sweep any parameter, and run randomized variations from a seed.
- **Regression:** rerun the suite after a design change and highlight what newly passes or fails.

Gantry test suite (the first suite the app must run):

| ID | Scenario | Expected behavior | Pass criteria |
| --- | --- | --- | --- |
| G1 | Power on, target (40, 25) sent before homing | Target refused, status "not homed" | No actuator motion; refusal reported |
| G2 | Homing | Each axis drives to its limit switch and sets the origin | Origin within 1 cm of true; status "ready" |
| G3 | Target (40, 25) after homing | Hook moves to the target | Final position within 1 cm; no travel limit exceeded |
| G4 | Target outside the workspace | Rejected with a fault, not clipped | No motion; fault reported |
| G5 | Load over the limit attached before a move | Lifting blocked, lowering allowed | Lift command ignored; lower command works |
| G6 | Load becomes overweight mid-lift | Further lifting blocked, lowering allowed | Lift stops; load can be set down |
| G7 | E-stop pressed mid-move with a load | Motor power cut, load held, no automatic resume | Power cut within the relay delay; hook does not drop; motion only after reset and a new command |
| G8 | Controller freezes mid-move | Watchdog cuts motor power | Motors stop within the heartbeat timeout |
| G9 | Power lost, then restored | Returns to "not homed" and requires homing | No motion until homed; load not dropped |
| G10 | Motor stall or belt slip during a move | Encoder mismatch detected | Stop, fault, re-home required |
| G11 | Limit switch trips mid-workspace | Position tracking treated as wrong | Stop, fault, re-home required |
| G12 | Load swing after a fast move and after an E-stop | Swing measured; lowering waits for settling if that policy is set | Peak swing angle and settle time within limits (not set) |
| G13 | Lift command while the gantry is moving | The defined policy applies (queue or reject) | Behavior matches the policy every run |
| G14 | Rapid-fire and conflicting commands | Handled predictably | No overlapping actuator commands |
| G15 | Worst case: maximum acceleration with rated load, and E-stop with a swinging load | Peak forces exported for Fusion | Every member above the minimum safety factor; deflection under 1 cm |
| G16 | Sweep NEMA 17 versus NEMA 23 with the rated load | Torque margin compared | Report torque margin and any missed steps per motor |

## Outputs and acceptance criteria

Every run should leave enough evidence to see what happened and why a case passed or failed.

Outputs per run:

- Time-series logs of positions, velocities, actuator commands, sensor readings, joint forces, and controller states
- A 3D replay of the run with events marked on a timeline
- A pass/fail report per criterion, with the measured value beside each limit
- A peak-force table per joint and mount, exportable for FEA tools
- Exports as CSV, JSON, and MAT files

The app is ready for the team when:

1. It builds the gantry from data files or an imported URDF with no gantry-specific code.
2. It runs G1 to G16 in one batch and produces the pass/fail report.
3. G3 and G12 match a Simscape Multibody run of the same case within an agreed tolerance (not set).
4. G15's structural results match Fusion Simulation for the same loads within an agreed tolerance (not set).
5. A second, different design (for example, a two-joint arm) can be built and tested using only the library and data files.
6. Rerunning any case with the same seed gives identical results.

## Assumptions and open questions

Assumptions:

- The app is a development tool; live hardware connection is not part of version 1.
- The gantry is the first reference design, not the only target.
- The technology stack is not chosen, and the out-of-scope list is proposed, not agreed.

Open questions:

- [ ] Gantry workspace dimensions and rated load ("a few pounds" versus the 3 kg, about 6.6 lb, used in the walkthrough)
- [ ] Limits for load swing angle and settle time
- [ ] Tolerances for matching Simscape and Fusion Simulation results
- [ ] Minimum structural safety factor
- [ ] Role of the proposed gyroscopic sensor, if any
- [ ] Policy for a new target during a move, and for lifting while moving
- [ ] How the gantry holds its load when power is cut: brake or self-locking lead screw
- [ ] Desktop or browser app, and whether designs and results are shared across the team
- [ ] Which STM32 chip, and whether controller scripts should be portable to C

## Instructions for the planning session

Use this doc to plan the app before writing code. Keep everything gantry-specific in data files; if a requirement can only be met by special-casing the gantry, flag it.

Deliverables, in order:

1. **Technology recommendation:** compare at least two physics engines against these requirements (ropes and swinging loads, flexible beams, stepper and transmission models, determinism, speed) and recommend a language, engine, and interface.
2. **Architecture:** refine the module sketch below into components, their interfaces, and what data flows between them.
3. **File formats:** draft the schemas for components, designs, and test cases, using the gantry and test G3 as worked examples.
4. **Milestone plan:**
   1. Rigid gantry model, controller runtime, and tests G1 to G4
   2. Sensors, fault injection, and safety hardware; tests G5 to G11, G13, G14
   3. Rope swing, flexible members, and structural checks; tests G12, G15, G16
   4. CAD import, FEA load export, and the Simscape and Fusion cross-checks
   5. A second, different design to prove the app is general
5. **Risks and open questions:** list what could block each milestone, adding to the open questions above.

Proposed module sketch (a starting point, not a decision):

&#91;embedded content: proposed module sketch · 3 input files, 5 modules, 1 output\]

Sensor readings and actuator commands are the only link between the controller and the physics, so controller code never sees true positions.
