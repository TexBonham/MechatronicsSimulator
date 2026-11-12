# Spec: Controller runtimes (CTL)

## Purpose

The controller runtime under `core/controller/` runs the real firmware behind one interface with two implementations:
native (the firmware built for the host and called each tick through the board-interface shim) and emulated (the real
.elf on Renode in lockstep). Freeze, reset and brownout are operations in both modes. The firmware itself and the board
interface header are FW's; the electrical models on the other side of the pins are SIG's.

Sources: "Architecture doc" is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`;
"Requirements doc" is `docs/Mechatronic Simulator — Requirements.md`.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| FW | CTL-002, CTL-003, CTL-004, CTL-005, CTL-016 | FW-003, FW-004, FW-009, FW-010, FW-011 |
| SIG | CTL-006, CTL-009, CTL-013, CTL-014 | SIG-021, SIG-022, SIG-023, SIG-024, SIG-026 |
| KER | CTL-002, CTL-008, CTL-011, CTL-014, CTL-019 | KER-005, KER-006, KER-009, KER-018, KER-020, KER-021 |
| REC | CTL-018 | REC-005, REC-007 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| CTL-001 | The controller runtime shall expose one runtime interface with a native and an emulated implementation. | Architecture doc, Modules: core (Controller runtime row) | M1 | unit | n/a |
| CTL-002 | The native runtime shall call the firmware's init entry point once after each reset and its tick entry point at the configured fixed control rate. | ADR 0011, C3; C3 | M1 | unit | n/a |
| CTL-003 | The native runtime shall pass inputs to the tick entry point and read outputs from it as a plain C struct. | ADR 0011, C3; C3 | M1 | unit | n/a |
| CTL-004 | The native runtime shall present inputs with polled semantics: counters as counts accumulated over the tick, levels as latched values, and no interrupts. | ADR 0011, C3; C3 | M1 | unit | n/a |
| CTL-005 | The native shim shall call the firmware entry points directly from the core runtime, with no cffi or other foreign-function layer. | ADR 0011, Consequences; ADR 0005, Supersedes | M1 | unit | n/a |
| CTL-006 | The native shim shall read the pin-binding table and present each bound signal as the board interface's value for its pin. | Architecture doc, Modules: core (Controller runtime row) | M1 | unit | n/a |
| CTL-007 | The native runtime shall build the firmware source for the host before the run starts. | Architecture doc, Data flows (one run, step 2); Architecture doc, Modules: inputs (Firmware row) | M1 | unit | n/a |
| CTL-008 | The emulated runtime shall run the firmware's compiled .elf on Renode in lockstep: for each kernel tick, apply pin inputs, run the emulator for the tick's virtual time, then read pin outputs. | Architecture doc, Technology recommendation ("MCU emulation"); Architecture doc, Data flows (within one physics tick, step 3) | M4 | unit | n/a |
| CTL-009 | The emulated runtime shall wire the signal layer to the emulated peripherals: GPIO, timer encoder and PWM channels, ADC and USART. | Architecture doc, Modules: core (Controller runtime row) | M4 | unit | n/a |
| CTL-010 | Interrupt semantics shall exist only in emulated mode. | ADR 0011, C3; C3 | M4 | unit | n/a |
| CTL-011 | A freeze operation shall stop the controller in both modes: in native mode the tick entry point is not called, in emulated mode the core is halted; in both, levels hold and per-tick buffers are empty. | Architecture doc, Data flows (fault fan-out, controller freeze); Architecture doc, Modules: core (Controller runtime row) | M2 | unit | n/a |
| CTL-012 | A reset operation shall restart the firmware from its reset state in both modes, with the power rail up at run start. | Architecture doc, Data flows (one run, step 2); Architecture doc, Modules: core (Controller runtime row) | M1 | unit | n/a |
| CTL-013 | When rail voltage falls below the chip's brownout threshold, the runtime shall hold the controller in reset, with its pin outputs at zero, until the voltage recovers, and then restart it from its reset state, in both modes. | Architecture doc, Data flows (fault fan-out, power loss and restore); Architecture doc, Modules: core (Controller runtime row) | M2 | unit | n/a |
| CTL-014 | The controller runtime shall receive only pin inputs and power rail state, never true physical state, in either mode. | Architecture doc, Repository starting point (invariant 1); Requirements doc, Controller requirements (realistic separation) | M1 | invariant | n/a |
| CTL-015 | The same firmware run in native and emulated mode shall agree on the G1 to G4 pass criteria and on recorded pin outputs within the cross-mode tolerance. | Architecture doc, Repository starting point (How to extend: a controller mode) | M4 | acceptance | not set (owed by owner) |
| CTL-016 | The emulated runtime shall reach Renode through the transport ruled by the milestone 4 ADR, from the core runtime directly. | ADR 0011, C6; C6 | M4 | unit | n/a |
| CTL-017 | No Renode type shall cross the `core/controller/` runtime boundary. | Architecture doc, Repository starting point (invariant 5) | M4 | invariant | n/a |
| CTL-018 | The runtime shall publish the controller's state and fault log each tick: in native mode through the shim, in emulated mode through a debug channel. | Architecture doc, Modules: core (Controller runtime row, interface out); Requirements doc, Controller requirements (logging) | M1 | unit | n/a |
| CTL-019 | An emulator fault shall end the run with a report carrying the simulated time and the emulator's error. | Architecture doc, Repository starting point (invariant 7) | M4 | unit | n/a |
| CTL-020 | The runtime shall delay the controller's pin outputs by a configurable input-to-output latency. | Requirements doc, Controller requirements (timing) | M2 | unit | n/a |
| CTL-021 | The runtime shall slow the controller on demand for a scheduled window by the slow-down factor in the run data. | Requirements doc, Controller requirements (timing) | M2 | unit | n/a |
| CTL-022 | An emulated run shall be deterministic, with Renode's virtual time driven only by the kernel. | Architecture doc, Technology recommendation ("MCU emulation"); Architecture doc, Repository starting point (invariant 4) | M4 | invariant | identical |
| CTL-023 | The runtime interface shall admit a further implementation, such as hardware in the loop, without a change to its callers. | Architecture doc, Seams reserved for the roadmap (hardware in the loop) | roadmap | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Native versus emulated cross-mode tolerance (CTL-015) | not set | owner |
| Renode transport: External Control socket or pyrenode3 (CTL-016) | not set; C client library favored | milestone 4 ADR (C6) |
| Brownout threshold source in the chip definition (CTL-013) | the chip file's `brownout_threshold` (`chips/<chip>/chip.yaml`), handed to the runtime as `NativeRuntimeConfig::brownout_threshold_v` (M2-B); the driver wiring is the integration wave's | owner, with chips/ |
| STM32 part | not set | gantry team, after the peripheral smoke test |
| Latency granularity (CTL-020) | whole control ticks: `ControllerPartyConfig::latency_ticks` (default 0) delays every published output, events and state codes with them, zeros before the first delayed output (M2-B); the FIFO is transport latency and is not flushed on freeze or brownout | owner, to confirm |
| Slow-down mechanism (CTL-021): skipped ticks or stretched virtual time | skipped ticks: integer factor >= 2, the runtime ticks on the window's first control step and once per factor steps; between, levels hold and per-tick buffers are empty, as when frozen (M2-B) | owner, to confirm |
| Fault windows (CTL-011, CTL-021) | kernel ticks [start, end), scheduled before the run, acting at the party's first step at or after each bound (KER-023); overlapping windows of one kind are a configuration error; each start and end is a row event (source `kernel`, kind `fault`) | M2-B |
| Step pulses while frozen (CTL-011) | none: bit-banged pulses stop on a hang (M1); timer-driven pulses that continue are a milestone 2 option | owner, M2 |
| Native work-dir layout (CTL-007) | build in `<work_dir>/native-firmware/host`; per-load copies in `<work_dir>/native-controller/<pid>-<n>/` | owner |
| Private library copy (CTL-002, CTL-012) | every load, reset and brownout recovery opens a fresh copy, so no firmware static survives a reset | owner |
| Version entry point (CTL-003) | `mechsim_app_board_interface_version()` returns the header macro the app was compiled against (header 2.2); mismatch rejects the load | owner, with FW |
| Native build configuration (CTL-007) | `app_target` default `mechsim_fw_app` (runtime builds `<app_target>_native`); `cmake_command` default `cmake` | owner |
| Brownout threshold default (CTL-013) | placeholder 2.7 V until chips/ gives the chip's value; an unset threshold is a configuration error, never "no brownout" | owner, with chips/ |

Owner note: CTL-011 (levels hold, per-tick buffers empty) and CTL-013 (outputs zero while held, restart from reset state on recovery) were reworded in milestone 1 brief M1-E.

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Firmware sources, board interface header and firmware build rules | FW |
| Peripheral smoke test and chip definitions | `chips/` |
| Sensor, rail, watchdog and E-stop models | SIG |
| Scheduling the controller party | KER |
