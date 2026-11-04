# firmware/: controller source trees

Real firmware, the controller itself (Decisions log rows 3, 4; R-02). Rules per ADR 0011.

## Rules

| # | Rule |
|---|---|
| C1 | C11 for application code; C++ only inside a module behind a C interface |
| C1 | Board interface is the C header `board_interface/mechsim_board.h` |
| - | Fixed-width integer types |
| 8 | Application code includes nothing outside `board_interface/` and the freestanding headers |
| C5 | Vendor HAL only under `firmware/board/<chip>/` |
| C3 | Native tick is polled: init once, tick at a fixed rate, one input and one output struct, no interrupts |
| - | Curly braces on all control flow; no ternary operators |

## Where code lives

| Path | Holds |
|---|---|
| `app/main_tick.c` | Reference stub: NOT_HOMED (1) to READY (2) after `MECHSIM_APP_READY_TICKS` (10); heartbeat toggles on digital_out[0]; UART echo; zero steps; one tick event per tick plus one per echoed byte, fault 1 on log overflow |
| `app/gantry/gantry.c` | Gantry controller G1 to G4, library `mechsim_fw_gantry` |
| `app/gantry/channels.h` | Channel macros `GANTRY_CH_*`, per mode in pin-table order; swapped for the builder's generated header at integration |
| `app/gantry/gantry_config.h` | Placeholder parameters (`#ifndef`, overridable), state/fault/event codes; values in fw.md Open parameters |
| `board/<chip>/` | Vendor HAL; STM32 part not set (gantry team) |
| `tests/` | Catch2 via `core/cmake/Catch2.cmake`; `test_fw_reference`, `test_fw_variant` (READY after 1, 250 Hz); `test_gantry` and `test_gantry_variant` (queue policy, 500 Hz, 64-byte TX queue) over the plant in `test_gantry_plant.cpp` (included, not compiled alone); include-check fixtures |

## Contracts

Header v2.1, fixed sizes (fw.md Open parameters); field units and semantics are in the header comments.

| Struct | Fields |
|---|---|
| `mechsim_board_inputs` | `digital_in[32]` u8, `counter_delta[8]` i32 (counts this tick), `adc_sample[16]` u16, `uart_rx[64]` u8 + `uart_rx_count` u16 |
| `mechsim_board_outputs` | `step_count[8]` u32, `direction[8]` u8, `enable[8]` u8, `digital_out[32]` u8, `uart_tx[64]` u8 + `uart_tx_count` u16, `state_code`, `fault_code` u8, `event_log[16]` (`mechsim_board_event`: `code` u8, `arg` u16) + `event_count` u8 |

| Entry point | Contract |
|---|---|
| `void mechsim_app_init(void)` | Once after each reset |
| `void mechsim_app_tick(const mechsim_board_inputs*, mechsim_board_outputs*)` | Once per tick; firmware writes every output field |
| `uint32_t mechsim_app_tick_hz(void)` | Returns `MECHSIM_APP_TICK_HZ` (header default 1000; cache var `MECHSIM_APP_TICK_HZ` sets it from the design); simulator errors on mismatch |

| Preset (run from `firmware/`) | Notes |
|---|---|
| `host` | Host C compiler, tests on, `build/host` |
| `arm-none-eabi` | `cmake/arm-none-eabi.cmake`; `-mcpu=cortex-m4 -mthumb` placeholder, chip not set |

Gantry controller: state machine NOT_HOMED, HOMING, READY, MOVING, FAULT; homes X and Y to the min switches; per-axis
Q16 trapezoid with end-of-travel slow zones; ASCII line codec (fw.md Open parameters); UART and event output queued
across ticks. `mechsim_fw_add_app(name [sources...])` defaults to `main_tick.c`.

Direction and enable (invariant 1, review ruling 2026-11-04): the pin-table `digital_out` channels are the single source
of truth; the native shim fills the stepper `direction[]` and `enable[]` arrays from them, so firmware holds those low.
Firmware owns `step_count[]`.

| Section index (`Read(file, offset=N, limit=80)`) | Lines |
|---|---|
| `app/gantry/gantry.c` | State 30, Fixed point 80, Queues 142, Status lines 177, State machine 255, Command parsing 343, Motion 543, Outputs 677, Entry points 735 |
| `tests/test_gantry.cpp` | Helpers 11, G1 not homed 74, G2 homing 130, G3 moves 233, G4 workspace 389, Operator link 436, Queues and event log 509, Heartbeat, status, outputs 570 |

Both: `-std=c11`, no extensions, `MECHSIM_FW_WARNINGS` from `cmake/FirmwareWarnings.cmake` (FW-008).

Include rule (FW-005, FW-006): `cmake/include_check.cmake`, at configure and as target `firmware_include_check`. Allowed:
`mechsim_board.h` resolving to `board_interface/` (a same-named file under `app/` fails), the C11 freestanding headers,
or a quoted header inside `app/` without `..`. Loose `.c`/`.h` directly in `board/` fail (C5).
