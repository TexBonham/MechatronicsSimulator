# core/signals: contract

Namespace `mechsim::signals`. Target `mechsim_signals` (links `mechsim_kernel`, `mechsim_pins`; never the controller's
directory); tests `mechsim_signals_tests`.

## Headers (`include/mechsim/signals/`)

| Header | Signatures |
|---|---|
| `signal_name.hpp` | `is_dotted_signal_name`, `is_instance_name`, `signal_instance` |
| `pin_binding.hpp` | `PeripheralMode` (7 modes); `PinBinding{signal, pin, mode}`; `PinBindingTable` |
| `power_rail.hpp` | `PowerRail{name, voltage, on}` (controller-safe) |
| `signal_component.hpp` | `SignalComponent` (+ defaulted `start(ComponentClock{base_rate, period, seed})`, `begin_tick(t)`); `FailureMode` stuck, noisy, dropped, forced_level, stall, slip; `FailureInjection{mode, start, end, noise_magnitude, forced_level, fraction}`; `SignalTable`; `NullSignalComponent` |
| `component_base.hpp` | `BuiltinComponent`; `SignalRng`; `derive_child_seed`; `periods_floor/ceil`; `JointCoordinate`, `CoordinateReader`; `SignalConfigError`, `SignalRunError` |
| `sensors.hpp` | `LimitSwitch(inst, LimitSwitchParams)` out; `IncrementalEncoder(inst, EncoderParams)` channel_a, channel_b |
| `stepper.hpp` | `StepDirDriver` step, dir, enable, vmot -> full_steps, direction, enabled, supply; `TorqueSpeedTable`; `BehavioralStepper` -> torque, stalled, missed_steps |
| `safety.hpp` | `PowerSupply` voltage; `EStopRelay` estop_loop, reset -> status, contact_closed; `Watchdog` heartbeat -> tripped, contact_closed |
| `operator_link.hpp` | `MessageSpec`, `OperatorMessage`; `LineCodec` / `AsciiLineCodec`; `UartTransport(baud)`; `OperatorLink` tx -> rx, `post`, `accept_controller_bytes`, `take_status` |
| `signal_layer.hpp` | `SignalLayer(SignalLayerConfig{..., physics_outputs, pin_outputs})` Party; `read_inputs`/`write_outputs` (names in `core/kernel/CLAUDE.md`); `ExternalInputs`, `OperatorCommands`; `add_component(ptr, Stage)`, `add_operator_link`; setters/getters for kernel-free unit tests; `record(TickRecord&)` |

## Layer step

Pin outputs (via slots) and externals onto the bus; power stage; rails (supply > 0 and every gate 1, consumers get
the voltage); sensor, driver, actuator stages (registration order); pin inputs from slots only (SIG-024);
actuator outputs summed into `generalized_forces[dof]`. Every component input needs exactly one source: connection,
output slot, external, rail consumer. `ChannelSlot{signal, PinField, channel}` places a bound signal in an array
element; counter channels may sum several signals (A + B).

## Conventions

- Time parameters to layer periods: cutoff delay and watchdog timeout `periods_floor` (within the delay), debounce
  `periods_ceil`. Trip when no edge for more than the timeout periods; the first sample counts as an edge.
- Faults: stuck holds levels; on counters stuck and dropped deliver no edges (counts lost); forced_level sets the
  value; noisy draws N(0, magnitude) from the child generator; stall scales available torque; slip misses steps.
- Stepper load torque = `transmissions[transmission].shaft_load_torque` (PHY-033 sign); demand = J alpha + load.
  `shaft_axial_inertia` given: must equal `rotor_inertia` to 1e-9 relative, else SignalConfigError naming both (SIG-037).
- ASCII line: `name *(SP value) LF`, shortest round-trip SI decimals; UART 8N1, baud / 10 bytes per s, <= 64 per tick.
- Built-ins hold no design names; the gantry message set is test data only.

## `src/signal_layer.cpp` sections

Helpers 24, Construction 110, Resolve 186, Start 305, ReadPinOutputs 325, RunStage 360, UpdateRails 381,
WritePinInputs 408, Step 445, Coupling 490, Record 549.
