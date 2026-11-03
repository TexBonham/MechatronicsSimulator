# core/signals: contract

Namespace `mechsim::signals`. Target `mechsim_signals` (links `mechsim_kernel`); tests `mechsim_signals_tests`.

## Headers (`include/mechsim/signals/`)

| Header | Signatures |
|---|---|
| `signal_name.hpp` | `is_dotted_signal_name(sv)`, `is_instance_name(sv)`, `signal_instance(sv)`; `SignalNameError` |
| `pin_binding.hpp` | `PeripheralMode` (digital_input, digital_output, counter_input, adc_input, pulse_output, uart_rx, uart_tx); `PinBinding{signal, pin, mode}`; `PinBindingTable`: `add()`, `find_signal()`, `find_pin()`, `entries()`; `PinBindingError` |
| `power_rail.hpp` | `PowerRail{name, voltage, on}`; no physics include (controller-safe) |
| `signal_component.hpp` | `SignalComponent`: `instance_name()`, `inputs()`, `outputs()`, `sample(const PhysicalState&)`, `set_inputs(span<const double>)`, `latch_outputs()`, `output_values()`, `inject(const FailureInjection&)`; `PythonSlowPathComponent`; `SignalTable::add()`; `NullSignalComponent`; `FailureMode`, `FailureInjection{mode, start, end, noise_magnitude, forced_level}`, `validate_injection()` |

`PhysicalState` = `kernel::PhysicsOutputs` (time as completed physics `steps`).

## Per-tick use (KER-009)

Sensors: `sample(state)` after physics and tier 1 integrity, then `latch_outputs()`, read `output_values()` into
pin inputs. Drivers: `set_inputs(pin outputs)` after the controller tick, `latch_outputs()`.

## Conventions

- Names: `instance.signal`, two identifier segments, instance part equals `instance_name()` (ADR 0012 D2).
- Unique per design: `SignalTable` rejects a repeat; `PinBindingTable` rejects a repeat signal or pin, keeps insertion order.
- Values parallel `inputs()`/`outputs()`; levels 0 or 1, counts as doubles, otherwise the signal's unit.
- Failure window is [start, end) in kernel base ticks; noise family is an open SIG parameter.
- Python tier derives from `PythonSlowPathComponent` (= `SignalComponent`), so both tiers share one surface (ADR 0005).

## Milestone 1 implements

Limit switch, encoder, step/dir driver, stepper tier 1, UART operator link, power rail with E-stop relay (SIG-029);
signal-layer party; pin-binding loading from design data.
