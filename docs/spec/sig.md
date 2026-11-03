# Spec: Signal layer (SIG)

## Purpose

The signal layer carries every electrical and power path between physics and the controller: sensor models, front ends,
power rails, the E-stop relay and watchdog, the operator link and the pin-binding table. Built-in components live in
`core/signals/`; user-added components run on the Python slow path until promoted (ADR 0005). It is the only path
between physics and the controller in both controller modes; the physics computation and the firmware are left to ENG and
CTL/FW.

Sources: "Architecture doc" is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`;
"Requirements doc" is `docs/Mechatronic Simulator — Requirements.md`.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| CTL | SIG-021, SIG-022, SIG-023, SIG-024, SIG-025, SIG-026 | CTL-006, CTL-009, CTL-013, CTL-014 |
| ENG (via KER) | SIG-001, SIG-013, SIG-016, SIG-032, SIG-033 (shaft torque out, shaft load torque back) | ENG-027, ENG-028, ENG-043, ENG-044, ENG-048 |
| KER | SIG-005, SIG-012, SIG-013, SIG-017, SIG-024, SIG-032 | KER-009, KER-010, KER-011, KER-018, KER-020, KER-021 |
| REC | SIG-028 | REC-005, REC-006, REC-009, REC-027 |
| TST | SIG-006, SIG-007, SIG-008, SIG-018, SIG-019, SIG-022, SIG-034, SIG-036 | TST-013, TST-014, TST-016, TST-017, TST-019, TST-037 |
| LIB | SIG-029, SIG-030, SIG-032, SIG-035 | LIB-005, LIB-022, LIB-025, LIB-026, LIB-027 |
| ACC | SIG-033, SIG-034, SIG-036 | ACC-019, ACC-020, ACC-021, ACC-022, ACC-023, ACC-024 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| SIG-001 | A sensor model shall turn the physical state published by physics into a signal quantized to the sensor's resolution parameter. | Requirements doc, Component library requirements (Sensors row); Architecture doc, Modules: core (Signal layer row) | M1 | unit | n/a |
| SIG-002 | A sensor model shall add noise with the distribution and magnitude given by its noise parameters. | Requirements doc, Component library requirements (Sensors row); Architecture doc, Data flows (within one physics tick, step 2) | M2 | unit | n/a |
| SIG-003 | A sensor model shall delay its signal by its delay parameter. | Requirements doc, Component library requirements (Sensors row); Requirements doc, Controller requirements (realistic separation) | M2 | unit | n/a |
| SIG-004 | A sensor model shall update its signal at its sample-rate parameter and hold it between samples. | Requirements doc, Component library requirements (Sensors row) | M1 | unit | n/a |
| SIG-005 | Every random draw in the signal layer shall come from the drawing instance's child generator derived from the run seed. | D1; Architecture doc, Repository starting point (invariant 4) | M2 | invariant | n/a |
| SIG-006 | A sensor in the stuck failure mode shall hold its last value for the fault window. | Architecture doc, Data flows (fault fan-out, sensor stuck); Requirements doc, Component library requirements (Sensors row, failure modes) | M2 | unit | n/a |
| SIG-007 | A sensor in the noisy failure mode shall apply the fault's noise for the fault window. | Architecture doc, Data flows (fault fan-out, sensor noisy) | M2 | unit | n/a |
| SIG-008 | A sensor in the dropped failure mode shall deliver no new samples for the fault window. | Architecture doc, Data flows (fault fan-out, sensor dropped) | M2 | unit | n/a |
| SIG-009 | A limit switch model shall present a level that changes when its actuator crosses the switch position. | Architecture doc, Repository starting point (milestone 1 signal set); Requirements doc, Component library requirements (Sensors row) | M1 | unit | n/a |
| SIG-010 | A debounce front end shall suppress level changes shorter than its debounce time parameter. | Architecture doc, Modules: core (Signal layer row, front ends) | M2 | unit | n/a |
| SIG-011 | An encoder model with its encoder-interface front end shall accumulate quadrature edges over each physics step from the joint motion and its counts-per-revolution resolution. | Architecture doc, Data flows (within one physics tick, step 2); Architecture doc, Repository starting point (milestone 1 signal set) | M1 | reference | not set (owed by owner, proposed exact count) |
| SIG-012 | A step/dir driver model shall count the step pulses of each tick and latch the direction and enable levels. | Architecture doc, Data flows (within one physics tick, step 4); Architecture doc, Modules: core (Signal layer row, drivers) | M1 | unit | n/a |
| SIG-013 | The driver model shall hand each tick's pulse count, direction and enable to the stepper model it drives, whose motor shaft torque the transmission (ENG) turns into joint force for the next physics step. | Architecture doc, Data flows (within one physics tick, steps 4 and 5); ADR 0003, P3 | M1 | unit | n/a |
| SIG-014 | An amplifier front end shall turn its sensor's output into an ADC sample with its gain and resolution parameters. | Architecture doc, Modules: core (Signal layer row, front ends); Architecture doc, Data flows (within one physics tick, step 2) | M2 | unit | n/a |
| SIG-015 | A power rail model shall carry its supply voltage to every component bound to it. | Architecture doc, Modules: core (Signal layer row) | M1 | unit | n/a |
| SIG-016 | When a motor rail is off, no actuator torque or force from components on that rail shall reach physics. | Architecture doc, Data flows (within one physics tick, step 5) | M1 | unit | n/a |
| SIG-017 | The E-stop relay shall open the motor rail within its cutoff delay parameter of an E-stop press, with no action by the controller. | Architecture doc, Data flows (fault fan-out, E-stop press); Requirements doc, Component library requirements (Power and safety hardware row) | M1 | unit | n/a |
| SIG-018 | The watchdog shall open the motor rail when its heartbeat input shows no heartbeat for longer than its heartbeat timeout parameter, with no action by the controller. | Requirements doc, Controller requirements (safety outside the controller); Requirements doc, Component library requirements (Power and safety hardware row) | M2 | unit | n/a |
| SIG-019 | A power-loss fault shall drop the rail voltage for the fault window and restore it after. | Architecture doc, Data flows (fault fan-out, power loss and restore) | M2 | unit | n/a |
| SIG-020 | A brake component shall apply its brake-torque parameter to its joint while engaged. | Requirements doc, Component library requirements (Power and safety hardware row) | M2 | unit | n/a |
| SIG-021 | The controller shall see an E-stop press, rail loss or watchdog trip only through its own pin inputs. | Architecture doc, Data flows (fault fan-out, E-stop press); Architecture doc, Repository starting point (invariant 1) | M1 | invariant | n/a |
| SIG-022 | The signal layer shall pass rail voltage to the controller runtime so that it can apply the chip's brownout threshold. | Architecture doc, Data flows (fault fan-out, power loss and restore) | M2 | unit | n/a |
| SIG-023 | The pin-binding table shall map each dotted signal name (instance.signal), unique per design, to one MCU pin and peripheral mode. | ADR 0012, D2; Architecture doc, Modules: core (Signal layer row) | M1 | unit | n/a |
| SIG-024 | Every value crossing between the signal layer and the controller runtime shall cross through a pin-binding table entry. | Architecture doc, Repository starting point (invariant 1) | M1 | invariant | n/a |
| SIG-025 | The operator link codec and its transport shall be separate classes, with UART as the v1 transport. | Architecture doc, Seams reserved for the roadmap (operator app); Architecture doc, Modules: core (Signal layer row) | M1 | unit | n/a |
| SIG-026 | The operator link shall encode scripted operator messages into UART bytes on the controller's receive pin and parse the controller's transmitted bytes back into named status messages. | Architecture doc, Modules: core (Signal layer row); Architecture doc, Data flows (within one physics tick, step 4) | M1 | unit | n/a |
| SIG-027 | The operator link's message set shall come from design data, using the requirements doc labels verbatim, so that the reference design's "target (x, y)", "home" and "current position, ready / fault" messages need no code. | ADR 0012, D2; Requirements doc, Controller requirements (gantry messages); Architecture doc, Repository starting point (invariant 2) | M1 | unit | n/a |
| SIG-028 | The signal layer shall publish every signal on every pin and every rail state to the recorder stream each tick. | Architecture doc, Modules: core (Signal layer row, interface out); Architecture doc, Modules: outputs (Recorder row) | M1 | unit | n/a |
| SIG-029 | The built-in component set shall include, by milestone 1, the limit switch, encoder, step/dir driver, UART operator link and power rail with E-stop relay. | Architecture doc, Repository starting point (milestone 1 paragraph) | M1 | unit | n/a |
| SIG-030 | A component on the Python slow path and its C++ built-in counterpart shall implement the same interface and produce identical results on the shared reference tests. | ADR 0005, Decision and Consequences; R-09 | phase 1 | reference | identical |
| SIG-031 | Adding a higher fidelity tier to a signal-layer component shall not change the results of its lower tier. | Architecture doc, Repository starting point (invariant 6) | M3 | invariant | identical |
| SIG-032 | The behavioral stepper model (tier 1) shall produce motor shaft torque each tick from the driver's pulse count, direction and enable, using its torque-speed curve and the state of its motor rail. | Decisions log row 13; Architecture doc, Modules: core (Signal layer row); Architecture doc, Data flows (within one physics tick, steps 4 and 5) | M1 | unit | n/a |
| SIG-033 | When the torque the stepper model demands exceeds the torque available at the current shaft speed, given the load torque physics publishes for the shaft joint, the stepper model shall stall or miss steps and report the stall or the missed steps. | Decisions log row 13; Architecture doc, Data flows (within one physics tick, step 5); Architecture doc, Data flows (fault fan-out, motor stall or missed steps) | M1 | unit | n/a |
| SIG-034 | A motor stall or missed-steps fault shall, for the fault window, reduce the stepper model's available torque or raise its demand above the available torque. | Architecture doc, Data flows (fault fan-out, motor stall or missed steps; owned by the signal layer per Data flows step 5 and R-13) | M2 | unit | n/a |
| SIG-035 | The stepper component shall reserve a phase-current fidelity tier slot, which v1 accepts in data and does not evaluate. | Decisions log row 13; Architecture doc, Seams reserved for the roadmap; Architecture doc, Repository starting point (invariant 6) | phase 1 | unit | n/a |
| SIG-036 | A digital sensor in the forced-level failure mode shall present the level the fault names for the fault window, whatever its physical input, generalizing the stuck mode (SIG-006) to a chosen level. | Requirements doc, Test cases (G11); Architecture doc, Data flows (fault fan-out, sensor stuck, the nearest row) | M2 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Noise distribution family (SIG-002) | not set | owner |
| Encoder count tolerance (SIG-011) | not set, proposed exact count | owner |
| Sensor delay granularity (whole ticks or sub-tick) | not set | owner |
| Signal-layer internal rate for pulse-level signals | not set | owner |
| Brake engagement semantics (rail-driven or pin-driven) (SIG-020) | not set | owner |
| E-stop relay cutoff delay value | not set | gantry team (reference design data) |
| Watchdog heartbeat timeout value | not set | gantry team (reference design data) |
| UART framing and baud handling in the transport | not set | owner, phase 1 contracts |
| Promotion criteria from Python to C++ beyond "stable" | not set | owner (ADR 0005, Open items) |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Transmission from motor shaft torque to joint force, and the shaft load torque | ENG (transmission elements, ENG-043 to ENG-048) |
| Belt and rope slip | ENG (transmission model, ENG-046) |
| Brownout reset of the controller | CTL |
| Building the wiring and pin table from the design file | BLD |
| Component type parameters and catalog | LIB |
| Operator app and Bluetooth or joystick transport | Roadmap seam (Architecture doc, Seams reserved for the roadmap) |
| Evaluating the phase-current stepper tier (slot reserved by SIG-035) | Roadmap seam (fidelity tier 2) |
