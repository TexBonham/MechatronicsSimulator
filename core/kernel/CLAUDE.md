# core/kernel: contract

Namespace `mechsim::kernel`; target `mechsim_kernel`; tests `mechsim_kernel_tests`. Also defines the header-only
`mechsim_pins` target (`pins/include/mechsim/pins/pins.hpp`, namespace `mechsim::pins`: PinInputs, PinOutputs,
EventLogEntry, channel constants, PowerRail, payload kinds) that signals and controller both link (CTL-014).

## Headers (`include/mechsim/kernel/`)

| Header | Signatures |
|---|---|
| `time.hpp` | `Rate(num, den=1)` rational Hz; `Seconds`; `SimTime`, `Duration` (int64 ticks); `lcm`, `period_in_ticks`, `ticks_at`, `to_seconds`; `KernelConfigError` |
| `party.hpp` | `PartyRole`; `Party`: `name`, `nominal_rate`, defaulted `role`, `published`, `consumed`, `start(base)`, `step(t, dt)`, `on_fault`; defaulted `set_seed`, `read_inputs(CouplingReader)`, `write_outputs(CouplingWriter)`, `finished`, `record(TickRecord&)`; `PartyFault(party, time, cause)` |
| `coupling.hpp` | `CouplingValue` variant<monostate, double, int64, bool, `PayloadPtr`>; `CouplingPayload`, `TypedPayload<T>`, `make_payload(value, kind)`, `payload_as<T>`, `CouplingReader::payload<T>`; `CouplingTable`; `CouplingReader` (`value`, `has_value`, `real`, `integer`, `flag`); `CouplingWriter::set`; `CouplingError` |
| `run_config.hpp` | `RunConfig{physics_rate, control_rate, party_rates, base_rate (kernel-written), seed, controller_mode, solver, drift}` |
| `kernel.hpp` | `Kernel`: `register_party`, `set_end_triggers(EndTriggers{at_time, on_signal, party_state})`, `run(EndCondition{end_time, stop}, sink) -> RunReport{outcome, base_rate, end_time, ticks, fault, end_reason, step_counts}`, `now`, `config`; `NullKernel` |
| `cosim_kernel.hpp` | `CoSimKernel(RunConfig)`, `coupling()`; `splitmix64_next`, `derive_child_seeds` |
| `physics_party.hpp` | `PhysicsPartyConfig{name, rate, inputs, drift}`; `PhysicsParty(unique_ptr<PhysicsAdapter>, config)`: role physics, publishes `<name>.outputs`, sums the `inputs` PhysicsInputs payloads, faults on non-convergence or drift > max(thresholds) (KER-016), `record` fills `row.physics` |
| `null_party.hpp` | `NullParty(name, rate)`: tier 1 integrity slot until M3 |
| `physics_types.hpp` | `kWorldBody` (contact body index of the world); `PhysicsOutputs{steps, bodies, joints, constraints, contacts, members, ropes, transmissions}`, `PhysicsInputs`, `PhysicsHealth{drift, converged, iterations, steps, physics_rate}` |
| `physics_adapter.hpp` | `PhysicsAdapter`: `supported_elements()`, `load(PreparedModel{json, engine_artifact, design_version}, physics_rate)`, `reset()`, `step()`, `outputs()`, `health()`, `set_inputs()`, mid-run `set_spring_offset/_stiffness`, `disable_constraint`, `release_load`, `set_payload_mass`, `set_payload_attachment`; `AdapterError(step, cause)` |
| `tick_record.hpp` | `TickRecord{time, physics, pin_signals, rails, actuator_efforts, controller_state, events, coupling}` |
| `recorder_sink.hpp` | `RecorderSink`: `write_row`, `subscribe`, `unsubscribe`, defaulted `subscriber_errors`; `NullRecorderSink` |
| `memory_recorder_sink.hpp` | `InMemoryRecorderSink`: `rows`, `drain`, `drained`, `subscriber_count`, `subscriber_errors` |

## Coupling names (KER-008, invariant 1)

| Name | Publisher | Payload | Consumers |
|---|---|---|---|
| `physics.outputs` | PhysicsParty | PhysicsOutputs | SignalLayer (`physics_outputs`) |
| `signals.physics_inputs` | SignalLayer | PhysicsInputs | PhysicsParty (`inputs`) |
| `signals.pin_inputs` | SignalLayer | PinInputs | ControllerParty (`pin_inputs`) |
| `signals.rails` | SignalLayer | PowerRails (`vector<PowerRail>`) | ControllerParty (`rails`, with `rail` set) |
| `signals.operator_status` | SignalLayer | OperatorStatus (`vector<StatusMessage>`, this step's) | test runner |
| `controller.pin_outputs` | ControllerParty | PinOutputs | SignalLayer (`pin_outputs`) |
| `signals.external_inputs` | test runner | ExternalInputs (`map<string, double>`) | SignalLayer, with externals |
| `signals.operator_commands` | test runner | OperatorCommands{sequence, messages} | SignalLayer, with a link; posted once per sequence |

A controller-role party consuming a physics-role party's name is a config error (KER-020).

## Per-tick order (KER-009)

Registration order is step order: physics, integrity, sensors, controller, outputs, drivers; one row per
physics step, last, stamped at the step's end.

## Conventions

- Rate: `party_rates[name]`, else `control_rate` for role controller, else nominal; physics role must equal
  `physics_rate`. Base = lcm of all; party steps when t % (base / rate) == 0.
- A step is `read_inputs` (consumed names only), `step`, `write_outputs` (published only); unwritten values hold.
- Values unset until first written; type fixed per run.
- Config errors before t = 0: rate overflow, unknown `party_rates` name, end/at time off tick, at time 0,
  two publishers, consumed but unpublished, unknown end signal or party. Then `set_seed`, `start` per party.
- Any throw (hooks, `record`, sink as `recorder`) -> `PartyFault(name, t, cause)`, `on_fault` to all, no row.
- End checks at physics-step boundaries only: signal, party state, stop, at time; `end_time` is the max duration.
- Child seeds: SplitMix64 from `RunConfig::seed`, registration order.
- `InMemoryRecorderSink`: stores before dispatch; a throwing subscriber is dropped, error kept; changes made
  inside a callback apply from the next row.
- No unordered containers, threads or wall clock.

## `tests/test_cosim_kernel.cpp` lines

Ordering and rates 108; coupling 325; end conditions 449; config errors 550; faults 589; seeds and
determinism 714; streaming and triggers 822.

## Scheduling (KER-023)

Each party steps on its own period boundaries; its outputs hold between its steps; end checks at physics
boundaries (`tests/test_physics_party.cpp`, 1 kHz physics with a 500 Hz controller). Gantry: equal rates.

## Open

KER-023 rule awaits owner confirmation.
