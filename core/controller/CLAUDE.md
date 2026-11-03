# core/controller: contract

Namespace `mechsim::controller`. Target `mechsim_controller`; tests `mechsim_controller_tests`.
Includes only `signals/power_rail.hpp` and `kernel/time.hpp`; never a kernel physics header (CTL-014, invariant 1).

## Header `include/mechsim/controller/controller_runtime.hpp`

| Item | Fields / signature |
|---|---|
| `PinInputs` | `digital_in[32]` u8; `counter_delta[8]` i32 (accumulated in the tick); `adc_sample[16]` u16; `uart_rx[64]` u8 + `uart_rx_count` u16. No rail field |
| `PinOutputs` | `step_count[8]` u32; `direction[8]`, `enable[8]` u8; `digital_out[32]` u8; `uart_tx[64]` u8 + `uart_tx_count` u16; `heartbeat`, `state_code`, `fault_code` u8; `event_log[16]` of `EventLogEntry{u8 code, u16 arg}` + `event_count` u8 (CTL-018) |
| Constants | `digital_in_channels` 32, `counter_channels` 8, `adc_channels` 16, `uart_buffer_bytes` 64, `stepper_channels` 8, `digital_out_channels` 32, `event_log_entries` 16; fixed, not overridable |
| `FirmwareImage` | `path`, `tick_hz` (firmware's `mechsim_app_tick_hz`) |
| `ControllerRuntime` | `load(const FirmwareImage&)`, `reset(const signals::PowerRail&)`, `tick(const PinInputs&, PinOutputs&)`, `freeze(bool)`, `brownout(double voltage)` |
| Errors | `ControllerConfigError` (load), `ControllerError` (run) |
| `NullControllerRuntime(Rate control_rate)` | zeroes outputs per tick; `ticks_run()`, `resets()`, `frozen()`, `supply_voltage()`, `firmware_path()` |

## Mapping to the board interface (brief C6)

The C header in `board_interface/` has the same layout and widths. The native shim copies field for field, both
directions; nothing is renamed, widened or reordered. More than 64 UART bytes in one tick is a `ControllerError`
(run error), never silent truncation. Rail state reaches the runtime only via `reset` and `brownout` (CTL-013).

## Per-tick use (KER-009)

Step 4: after sensors latch, `tick` runs once per control period; outputs then go to the signal layer. Frozen: no
tick, outputs zeroed, physics and signals keep stepping (KER-021).

## Conventions

- Polled native semantics (ADR 0011 C3, CTL-004): counter deltas, latched levels, no interrupts.
- `load` compares `tick_hz` with `RunConfig::control_rate` and throws `ControllerConfigError` on mismatch.
- No Renode type crosses this interface (CTL-017).

## Milestone 1 implements

Native shim calling firmware init and tick directly (CTL-002 to CTL-007, CTL-012). Emulated runtime: milestone 4.
