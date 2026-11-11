# core/signals: contract

Namespace `mechsim::signals`. Target `mechsim_signals` (links `mechsim_kernel`, `mechsim_pins`; never the controller's
directory); tests `mechsim_signals_tests`.

## Headers (`include/mechsim/signals/`)

| Header | Signatures |
|---|---|
| `signal_name.hpp` | `is_dotted_signal_name`, `is_instance_name`, `signal_instance` |
| `pin_binding.hpp` | `PeripheralMode` (7 modes); `PinBinding{signal, pin, mode}`; `PinBindingTable` |
| `power_rail.hpp` | `PowerRail{name, voltage, on}` (controller-safe) |
| `signal_component.hpp` | `SignalComponent` (+ defaulted `start(ComponentClock{base_rate, period, seed})`, `begin_tick(t)`, `joint_drives()`, `take_notes()`); `FailureMode` stuck, noisy, dropped, forced_level, stall, slip, power_loss; `FailureInjection{mode, start, end, noise_magnitude, forced_level, fraction}`; `SignalTable`; `NullSignalComponent` |
| `component_base.hpp` | `BuiltinComponent` (+ `generator()`, protected `injections()`, `note(line)`); `SignalRng` (+ `state()`); `derive_child_seed`; `periods_floor/ceil`; `JointCoordinate`, `CoordinateReader`; `SignalConfigError`, `SignalRunError` |
| `sensors.hpp` | `SampledSensor` base (`SensorFidelity{noise, delay, sample_rate}`, `SensorOutputKind` digital/analog/counter; override `measure`, `convert`, opt. `present`, `part_noise(q)`); `LimitSwitch(inst, LimitSwitchParams)` out; `IncrementalEncoder(inst, EncoderParams)` channel_a, channel_b (`linear` true: prismatic coordinate, m per count; must match the coordinate kind); `LoadCell(inst, LoadCellParams)` signal; `Imu(inst, ImuParams)` accel_x/y/z, gyro_x/y/z |
| `front_end.hpp` | `AdcParams{bits, reference}`; `LoadCellAmplifier(inst, AmplifierParams)` signal_in -> out (counts), `voltage()` |
| `stepper.hpp` | `StepDirDriver` step, dir, enable, vmot -> full_steps, direction, enabled, supply; `TorqueSpeedTable`; `detent_tangent`; `BehavioralStepper` -> torque, stalled, missed_steps, `joint_drives()` |
| `safety.hpp` | `PowerSupply` voltage (power_loss); `EStopRelay` estop_loop, reset -> status, contact_closed; `Watchdog` heartbeat -> tripped, contact_closed; `Brake(inst, BrakeParams)` release, vcc -> engaged, torque, `joint_drives()`, `stiffness()` |
| `operator_link.hpp` | `MessageSpec`, `OperatorMessage`; `LineCodec` / `AsciiLineCodec`; `UartTransport(baud)`; `OperatorLink` uart_tx -> uart_rx (the catalog `uart_link` pins), `post`, `accept_controller_bytes`, `take_status` |
| `signal_layer.hpp` | `SignalLayer(SignalLayerConfig{..., physics_outputs, pin_outputs})` Party; `read_inputs`/`write_outputs` (names in `core/kernel/CLAUDE.md`); `ExternalInputs`, `OperatorCommands`; `add_component(ptr, Stage)`, `add_operator_link`; setters/getters for kernel-free unit tests; `record(TickRecord&)` |

## Layer step

Pin outputs (via slots) and externals onto the bus; power stage; rails (supply > 0 and every gate 1, consumers get
the voltage); sensor, front_end, driver, actuator stages (registration order; `Stage::front_end` is the last enum
value, run second after rails); each component's `take_notes()` becomes an `approximation` event; pin inputs from slots only (SIG-024);
applied actuator outputs summed into `generalized_forces[dof]`; every component's `joint_drives()` (defaulted empty) gathered in registration order and combined per joint as parallel springs before publishing `joint_drives` (ruling 23: over the enabled drives K = sum K_i, rest = sum K_i r_i / K, D = sum D_i; at most one enabled drive passes unchanged; K = 0 keeps the first enabled rest; joints in first-drive order), so KER-027's one drive per joint holds. Each component samples the state with its own share of the read-back torque on its joint (SIG-039, `apportion_drive_torques` before the stages): 0 for a disabled drive, the whole for the joint's only enabled drive, else tau_i = K_i (r_i - theta) - D_i omega of its last published drive (omega the joint coordinate's backward difference over the layer period) plus the residual T - sum tau in proportion to |tau_i| (stiffness, damping, equal when all tau_i are 0); the layer swaps the shares into `state_.joint_drive_torques` around that component's `sample` and restores them. Every component input needs exactly one source: connection,
output slot, external, rail consumer. `ChannelSlot{signal, PinField, channel}` places a bound signal in an array
element; counter channels may sum several signals (A + B).

## Conventions

- Time parameters to layer periods: cutoff delay and watchdog timeout `periods_floor` (within the delay), debounce
  `periods_ceil`. Trip when no edge for more than the timeout periods; the first sample counts as an edge.
- Sensors (`SampledSensor`, SIG-002 to 008): measure every step; sample at step % period == 0 (period
  `periods_floor(1 / rate)`, >= 1; rate above the layer's is an error); part noise then fault noise, N(0, sd) in the
  measured unit before quantization, no draw at 0; delay ring of `periods_ceil(delay) + 1` frames prefilled with the
  first. Faults [start, end): stuck holds output; dropped takes no sample and holds (counters keep counting: counts
  lost); forced_level digital 0/1, counter round(level) counts, analog an error; overlapping windows an error at start.
- Debounce (SIG-010): a level sampled first at i appears at the first sample >= i + N if every sample between saw it.
- Actuator faults: stall scales available torque; slip misses steps.
- Stepper (SIG-033, PHY-034): position-commanded; theta_cmd += steps x step_angle; theta = `joint_positions[joint]`; each tick publishes `joint_drives()` = {joint, rest theta_p + tau / K_t, K_t, damping, enabled}, the tangent of tau = holding sin(pi delta / (2 step_angle)) at the predicted lead delta = theta_cmd - theta_p, theta_p = theta + omega h (pull-out also on it), K_t = K_hold cos(.) floored at 0.05 K_hold (delta 0: K_hold toward theta_cmd), which the layer copies into `physics_inputs.joint_drives`; output `torque` is the drive torque read back from `joint_drive_torques`. Pull-out when |delta| > 2 step angles (re-synchronizes to the step nearest theta) or |drive torque| > table(|omega|) (the same, but at least one step toward the load: sign of delta, or of the drive torque at delta 0); counts missed steps, stall at zero commanded speed. Loss of synchronism (brief M2-A3): while |slip| = |omega h - n' step| (n' the steps applied last tick, 0 when disabled) exceeds one step (1e-9 relative margin, `slip_rounding`), theta_cmd re-syncs every tick to the step nearest theta_p (missed counted) and the drive brakes: rest theta_p - sign(slip) step, K = table(|omega|) / step, damping D; tracking resumes when |slip| <= one step. Its `ActuatorOutput` is `applied = false` (recorded only). `shaft_axial_inertia` given: must equal `rotor_inertia` to 1e-9 relative, else SignalConfigError naming both (SIG-037).
- Per-tick pin buffers (SIG-038): `controller_rate` (default the layer's); counter deltas and UART rx accumulate over layer steps until a controller boundary lies in [last step, t), outputs' step counts and UART tx are taken only at the first layer step after one; over 64 rx bytes per controller step throws.
- ASCII line: `name *(SP value) LF`, shortest round-trip SI decimals; UART 8N1, baud / 10 bytes per s, <= 64 per tick.
- Built-ins hold no design names; the gantry message set is test data only.

## Milestone 2 built-ins (brief M2-A2)

| Type (category, stage) | Class | Model |
|---|---|---|
| `load_cell` (sensor) | `LoadCell` | at most one target (none: 0 N, noise and rounding still apply); payload weld-force magnitude, joint generalized_force[0] (signed) or `joint_reaction` (a fixed joint's reaction.force magnitude, ruling 24); noise in N; round to resolution, or past +-overload_limit hold at the limit (no error, one `approximation` note per run); V = S E F / C + nonlinearity (F / C)^2 S E |
| `load_cell_amplifier` (front_end) | `LoadCellAmplifier` | x = clamp(gain V + offset, 0, FS); y += (1 - exp(-2 pi f h))(x - y) from the first x; counts = round(y / ref (2^bits - 1)) clamped |
| `imu` (sensor) | `Imu` | f = R^T (a - g), a backward difference of world velocity (0 at first step); gyro R^T omega; `gyro_noise` separate; clamp, round; outputs are SI values (ruling 2) |
| `brake` (safety, power stage in the wiring) | `Brake` | see the brake drive bound below |
| `power_supply` | `PowerSupply` | `power_loss` fault: 0 V for [start, end) (SIG-019) |

Brake drive bound (PHY-034 envelope (K_d h^2 + D_d h) A0 <= 400): K_b = `brake_envelope_product` / (A0 h^2) = 100 / (A0 h^2)
with A0 (`inverse_inertia`, the joint's `dof_invweight0`) given, so K_b A0 h^2 = 100; without it K_b = brake_torque /
`brake_default_compliance` (0.01 rad), which the wiring must keep under the envelope. Hold: rest at the engagement
coordinate or where it stuck (static sag L / K_b), K_b and the brake's damping. Slip (SIG-020, brief M2-C2) on
engaging a moving joint (s = -sign omega) or when |read-back share| > brake_torque (s its sign): K_s =
`brake_slip_stiffness_ratio` K_b = 1e-5 K_b, damping 0, rest = theta + omega h + s T_b / K_s, so the implicit drive
applies s T_b at the step's end state to the apparent inertia K_s A0 h^2 <= 4e-3 (a pure Coulomb torque). Sticks (rest theta, K_b) once omega
or omega + alpha h no longer opposes s. In the power stage the brake reads its `vcc` rail one step late (rails update
after that stage).

## Section index

| Section | File | Line |
|---|---|---|
| Helpers | src/signal_layer.cpp | 26 |
| Construction | src/signal_layer.cpp | 165 |
| Resolve | src/signal_layer.cpp | 244 |
| Start | src/signal_layer.cpp | 364 |
| ReadPinOutputs | src/signal_layer.cpp | 397 |
| ApportionDriveTorques | src/signal_layer.cpp | 439 |
| RunStage | src/signal_layer.cpp | 522 |
| UpdateRails | src/signal_layer.cpp | 558 |
| WritePinInputs | src/signal_layer.cpp | 585 |
| Step | src/signal_layer.cpp | 634 |
| Coupling | src/signal_layer.cpp | 734 |
| Record | src/signal_layer.cpp | 793 |
| Load cell | tests/test_m2_components.cpp | 125 |
| Amplifier | tests/test_m2_components.cpp | 367 |
| IMU | tests/test_m2_components.cpp | 418 |
| Brake | tests/test_m2_components.cpp | 483 |
| Power loss | tests/test_m2_components.cpp | 704 |
| Layer: front_end stage and approximation notes | tests/test_m2_components.cpp | 744 |
| Layer: drives on one joint combine (ruling 23) | tests/test_signal_layer.cpp | 400 |
| Layer: the combined drive's torque apportioned (ruling 23, SIG-039) | tests/test_signal_layer.cpp | 496 |
| Encoders and limit switch | tests/test_sensors.cpp | 37 |
| Linear encoder | tests/test_sensors.cpp | 227 |
| Sensor pipeline | tests/test_sensors.cpp | 327 |
| Noise, delay and sample rate | tests/test_sensors.cpp | 388 |
| Sensor faults | tests/test_sensors.cpp | 500 |
| Debounce and chatter | tests/test_sensors.cpp | 578 |
| Encoder delay and child generators | tests/test_sensors.cpp | 642 |
| Fixtures | tests/test_stepper.cpp | 35 |
| Driver and torque-speed table | tests/test_stepper.cpp | 66 |
| Shaft stub | tests/test_stepper.cpp | 102 |
| Holding torque and static load | tests/test_stepper.cpp | 221 |
| Pull-out and loss of steps | tests/test_stepper.cpp | 343 |
| Disabled, stall and slip | tests/test_stepper.cpp | 443 |
| Parameters and rotor inertia | tests/test_stepper.cpp | 502 |
| Loss of synchronism (SIG-033, brief M2-A3) | tests/test_stepper.cpp | 551 |
