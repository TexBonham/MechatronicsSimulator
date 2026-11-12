# core/bindings: contract

One pybind11 module, `mechsim._core_ext` (ADR 0005), imported through `mechsim/_core.py` (raises
`CoreNotBuiltError` naming the build step if the module is missing). Target `_core_ext`; links `mechsim_engine`,
`mechsim_controller` (-> signals -> kernel), `mechsim_integrity`, `mechsim_physics_mujoco` (libmujoco installed next to the extension in
`mechsim/`, found through `INSTALL_RPATH $ORIGIN`; the build tree keeps its own RPATH). pybind11 v3.0.1 via FetchContent; `MECHSIM_BUILD_BINDINGS` (ON).

## Build

| Path | Command |
|---|---|
| Python env | `uv sync` (scikit-build-core, root `CMakeLists.txt`, Release, build dir `build/python/<wheel tag>`, target `_core_ext` only); `[tool.uv] cache-keys` rebuild it when C++ sources or CMake files change |
| Fallback | `uv sync --reinstall-package mechsim` if a rebuild is missed |
| Presets | `cmake --preset debug-sanitize` builds it with the rest (clang-tidy, sanitizers); not loaded by ctest |

## Sources (`src/`)

| File | Binds |
|---|---|
| `bindings.hpp` | `bind_*` declarations; `call_override<R>(self, name, args...)` (takes the GIL, Python error -> C++); `party_fault_object`; `rethrow_as_cpp` |
| `factories.hpp/.cpp` | `make_kernel(RunConfig)` (CoSimKernel); name registries `make_physics_adapter(name)`, `make_controller_runtime(name, rate)`, `*_names()`; adapters "mujoco", "native" (`core/physics/native/`), "null"; runtimes "null" |
| `errors.cpp` | Python exception classes and the translator |
| `bind_kernel.cpp` | `Rate`, `Seconds`, `Duration`, `SimTime`, `lcm`, `period_in_ticks`, `ticks_at`, `to_seconds`; `RunConfig`, `SolverConfig`, `DriftThresholds`, `ControllerMode`; `EndCondition(end_time, stop=None)`, `RunOutcome`, `RunReport` (`fault` -> `PartyFault` or None); `Party` (trampoline); `Kernel`, `NullKernel`, `make_kernel` |
| `bind_physics.cpp` | Neutral physics types (`TransmissionState` + `PowerFlow`, `PayloadState`); `PreparedModel`; `PhysicsAdapter` (trampoline; optional `set_transmission_slip`), `MujocoAdapter` (+ `integrator`, `timestep`), `NullPhysicsAdapter`; `physics_adapter_step(adapter)` (C++ call path) |
| `bind_recorder.cpp` | `TickRecord`, `SeriesSample`, `RecordedEvent`, `SubscriptionId`; row dict physics adds `joint_positions`, `joint_drive_torques` [{joint, torque}], `payloads` [{body, carrier (-1 world), attached, force}]; `RecorderSink` (trampoline), `NullRecorderSink`, `BufferedRecorderSink` (`subscribe`, `drain_dicts`, `drain_arrays`, `pending`) |
| `bind_signals.cpp` | `PeripheralMode`, `PinBinding`, `PinBindingTable`, `PowerRail`, name checks; `FailureMode` (+ power_loss), `FailureInjection`, `validate_injection`; `SignalComponent` (= `PythonSlowPathComponent`, trampoline; + `joint_drives()`, `take_notes()`), `NullSignalComponent`, `SignalTable`, `signal_component_tick` |
| `bind_controller.cpp` | Channel constants; `PinInputs`, `PinOutputs`, `EventLogEntry`, `FirmwareImage`; `ControllerRuntime`, `NullControllerRuntime`, `make_controller_runtime` |
| `bind_cosim.cpp` | Coupling conversions (`coupling_to_python`/`from_python`); `CouplingReader`, `CouplingWriter`, `CouplingTable`; `CoSimKernel` (`set_end_triggers`, `coupling`); `InMemoryRecorderSink`; `PhysicsParty(+Config)` (+ `schedule_slip(transmission, start, end, fraction)`, `schedule_payload_change(payload, tick, *, mass=None, carrier=None)`, exactly one, carrier -1 releases), `ControllerParty(+Config, `latency_ticks`)` (+ `schedule_freeze(start, end)`, `schedule_slowdown(start, end, factor)`); schedule before `register_party`; `ComponentClock`, `derive_child_seed`, `JointCoordinate`, `make_signal_component(type, instance, params)` (table below); `Stage` power, sensor, driver, actuator, front_end; operator types, `make_operator_link`; `SignalLayer` and config types; `ChannelMap` and friends; `NativeControllerRuntime`, `NativeRuntimeConfig`, `build_host_firmware` |
| `bind_integrity.cpp` | `ModelChangeRequest(kind, index, value)`, `IntegrityCrossingRecord`, `IntegrityTrace()` (readonly peaks, `crossings`, `requests`), `IntegrityPartyConfig`, `IntegrityParty(limits_json, config, trace=None)` (ValueError on a malformed table), `check_integrity_limits(json)`; `PhysicsPartyConfig.model_requests` in `bind_cosim.cpp` |
| `bind_engine.cpp` | Submodule `_core_ext.engine` only: `StepConfig`, `State`, `StepHealth`, `JointType` (+ Translation), `Joint`, `Body`, `LoopConstraint`, `MultibodyModel` (+ `members`, `ropes`), `MultibodyConfig` (+ `friction_slip_threshold`), outputs (+ `ConstraintOutput.torque`, `RopeOutput`, `MemberOutput`, `ModelChange`, `LoadSource`, `ModelChangeEvent`; `MultibodyOutputs.members`, `ropes`, `events`; `TransmissionOutput.scheme_decay`), `Rope`, `add_rope` (model in place), `SemiImplicitEuler`, `MultibodyProvider` (+ `set_spring_offset`, `set_spring_stiffness`, `disable_constraint`, `release_load(constraint)`, `release_rope`, `constraint_enabled`, `rope_attached`, `set_rope_rest_length`, `rope_states`); lumped beam `BeamEnds` (+ Clamped), `BeamSpec`, `BeamPoint`, `LumpedBeam` (+ `pinned_root`), `add_lumped_beam` (model in place), `beam_point`, `member_nodes` (ENG-022) |
| `bind_contact.cpp` | Same submodule: `ContactParameters`, `MaterialPair`, `Geometry`, `ContactModel`, `CollisionLimits`, `ContactPoint`, `PayloadOutput`, `collide(a, b, limits=CollisionLimits())`; adds `ContactOutput.normal`/`depth`, `MultibodyOutputs.payloads`, provider `set_contact_model`, `contact_model`, `contacts`, `set_body_mass`, `set_attachment` (tests `tests/unit/test_engine_bindings.py`) |

## `make_signal_component` parameters

Unknown keys raise `ValueError` naming the key. Fidelity keys `noise`, `delay`, `sample_rate` (SIG-002 to SIG-004) on
every sensor.

| Type | Keys |
|---|---|
| `limit_switch` | coordinate, position, actuate_above, hysteresis, resolution, debounce_time, normally_open + fidelity |
| `incremental_encoder` | coordinate, resolution, reversed, linear (bool, prismatic coordinate, m per count) + fidelity |
| `load_cell` | at most one of payload, joint, joint_reaction (index; none reads 0 N), capacity, sensitivity, excitation_voltage, resolution, overload_limit, nonlinearity + fidelity |
| `load_cell_amplifier` | gain, offset, output_full_scale, bandwidth, adc_bits, adc_reference (the chip's `adc` block) |
| `imu` | body, gravity [x, y, z], accel_range, gyro_range, resolution, gyro_resolution, gyro_noise + fidelity |
| `brake` | joint, brake_torque, engage_delay, release_delay, power_off_engaged, damping, inverse_inertia (A0) |
| `step_dir_driver`, `stepper_behavioral`, `power_supply`, `estop_relay`, `watchdog` | as milestone 1 |

## Exceptions

| Python class | Base | Attributes | From |
|---|---|---|---|
| `KernelConfigError` | `ValueError` | `cause` | `kernel::KernelConfigError` |
| `PartyFault` | `RuntimeError` | `party`, `time` (`SimTime`), `cause` | `kernel::PartyFault`; `RunReport.fault`; `Party.on_fault(fault)` |
| `AdapterError` | `RuntimeError` | `step`, `cause` | `kernel::AdapterError` |
| `EngineError` (+ `NonFiniteStateError`, `SolverNonConvergenceError`, `DriftError` + `constraint_index`, `InvalidModelError`, `InvalidConfigError`) | `RuntimeError` | `time` (s), `cause` | `engine::*` |
| `ControllerError` / `ControllerConfigError` | `RuntimeError` / `ValueError` | `cause` | `controller::*` |
| `PinBindingError`, `SignalNameError` | `ValueError` | `cause` | `signals::*` |

A Python exception inside an override becomes `std::runtime_error("Type: message")`, so the kernel's `PartyFault`
cause reads `"ValueError: boom"`; a Python `PartyFault` or `AdapterError` keeps its own `cause`.

## Conventions

- Holder `py::smart_holder` on every polymorphic class: `Kernel.register_party(p)` takes ownership and keeps a Python
  party alive; the Python handle is disowned afterwards (keep shared state outside the party object).
- Trampolines subclass in Python with `super().__init__()`; every pure method must be defined, else the call raises.
  `Party.on_fault` too: a missing or raising `on_fault` cannot propagate (noexcept), so it goes to `sys.unraisablehook`.
- Invariant 5: engine types live only in the submodule `mechsim._core_ext.engine`, re-exported as
  `mechsim.physics.native._engine`; `tools/check_imports.py` allows those imports only under `mechsim/physics/native/`
  and `tests/`, and `mechsim._core.engine` raises. Neutral types and exception classes stay top-level.
- Views (`string_view`, `span`) returned to C++ are copied out of Python into trampoline members.
- Arguments to Python overrides and subscriber callbacks are copies (REC-020): Python never holds kernel memory.
- `Kernel.run` releases the GIL; every trampoline and callback takes it per call. No threads (invariant 4).
- Party hooks `role`, `set_seed`, `read_inputs`, `write_outputs`, `finished`, `record` and SignalComponent `start`,
  `begin_tick` are optional in Python subclasses; the trampoline takes the GIL before casting their arguments.
- Containers (`list`, `dict`) and Eigen vectors (`State.q`, `State.v`) are copied in and out: assign
  `model.bodies = [...]` or `state.q = array`; `model.bodies.append(b)` and `state.q[0] = 1.0` change nothing.
- Quaternions cross as `[w, x, y, z]`; Eigen vectors and matrices as numpy arrays.
- Coupling values reach Python through `TickRecord.to_dict()["coupling"]` (name -> value); `drain_dicts` and
  `drain_arrays` carry no coupling column by ruling.
- `BufferedRecorderSink.drain_arrays()` needs a constant body and rope count; series columns are NaN where missing.

## Extending

New kernel: point `make_kernel` in `factories.cpp` at it. New adapter or runtime: one registry entry in
`factories.cpp` plus its target in `target_link_libraries`. Bind its own class only if Python needs its extras.

## Section index

| Section | File | Line |
|---|---|---|
| Parameter readers | src/bind_cosim.cpp | 50 |
| Coupling values | src/bind_cosim.cpp | 118 |
| Signal components | src/bind_cosim.cpp | 195 |
| Module binding | src/bind_cosim.cpp | 348 |
