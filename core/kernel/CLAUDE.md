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
| `physics_party.hpp` | `PhysicsPartyConfig{name, rate, inputs, drift, model_requests}`; `ModelChangeRequest{kind, index, value}`, `ModelChangeRequests{sequence, requests}` (kind `ModelChangeRequests`): with `model_requests` set the party consumes that name and applies each new sequence once, in order, through `set_spring_offset/_stiffness`, `disable_constraint`, `release_load` before its next step (other kinds: KernelConfigError; unset: unwired, KER-028); `PayloadMass{kg}`, `PayloadCarrier{body}`; `PhysicsParty(unique_ptr<PhysicsAdapter>, config)`: role physics, publishes `<name>.outputs`, sums the `inputs` PhysicsInputs payloads (forces summed, wrenches and joint drives concatenated; two drives on one joint in a tick: KernelConfigError, KER-027), faults on non-convergence or drift > max(thresholds) (KER-016); `schedule_slip(transmission, start, end, fraction)`, `schedule_payload_change(payload, tick, PayloadMass or PayloadCarrier)`; `record` fills `row.physics`, the fault events and one `RecordedEvent{t, "physics", "model_change", model_change_detail(e)}` per `model_changes` entry, once per step (`<kind> <source> <index> value <shortest value>`, ruling 26) |
| `null_party.hpp` | `NullParty(name, rate)`: a do-nothing party (kernel tests); the tier 1 slot is `core/integrity/` `IntegrityParty` from M3 |
| `physics_types.hpp` | `kWorldBody` (contact body index of the world; payload carrier once released); `PhysicsOutputs{steps, bodies, joints, constraints, contacts, members, ropes, transmissions (+ `slip`), joint_positions, joint_drive_torques, payloads, model_changes}`, `RopeTension{tension (load end), max_tension, slack}`, `ModelChangeEvent{kind, source, index, value}` (`ModelChangeKind`: SpringOffset, SpringStiffness, ConstraintDisabled, LoadReleased, RopeSnapped, PayloadAttached, PayloadMass; `ModelChangeSource`: Spring, Constraint, Rope, Payload; each delivered in one tick only, ruling 40), `PayloadState{body, carrier, attached, force}`, `PhysicsInputs{generalized_forces, body_wrenches, joint_drives}`, `JointDrive{joint, rest, stiffness, damping, enabled}`, `JointDriveTorque{joint, torque}`, `PhysicsHealth{drift, converged, iterations, steps, physics_rate}` |
| `physics_adapter.hpp` | `PhysicsAdapter`: `supported_elements()`, `load(PreparedModel{json, engine_artifact, design_version}, physics_rate)`, `reset()`, `step()`, `outputs()`, `health()`, `set_inputs()`, mid-run `set_spring_offset/_stiffness`, `disable_constraint`, `release_load`, `set_payload_mass`, `set_payload_attachment` (release: `set_payload_attachment(p, kWorldBody)`, index based), `set_transmission_slip(transmission, fraction)` (default refuses: no slip model); `AdapterError(step, cause)` |
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
| `physics.outputs` | PhysicsParty | PhysicsOutputs | IntegrityParty (`physics_outputs`), when tier 1 is on |
| `integrity.model_requests` | IntegrityParty, yield on | ModelChangeRequests | PhysicsParty (`model_requests`); unwired without the party |

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

Ordering and rates 110; coupling 327; end conditions 451; config errors 552; faults 591; seeds and
determinism 716; streaming and triggers 824.

## Scheduled physics faults (KER-023, `PhysicsParty`)

Known before t = 0, in kernel ticks. Slip: `set_transmission_slip(t, fraction)` before the first step at or after
`start`, `set_transmission_slip(t, 0)` before the first at or after `end` (adjacent windows on one transmission: one
call at the boundary); fraction in (0, 1]. Payload change: `set_payload_mass` (finite, > 0 kg) or
`set_payload_attachment` before the first step at or after its tick. Malformed: `KernelConfigError` at the call;
overlapping slip windows on one transmission or two changes to one payload at one tick: `KernelConfigError` at
`start()`, as is a slip window holding no step at the party's rate; after `start()`: `KernelConfigError`; with slip
windows, a step period other than the nominal one faults the first step. Each start, end
and change: `RecordedEvent{t, "kernel", "fault", detail}` (`slip start transmission i fraction f`, `slip end
transmission i`, `payload i mass m` / `carrier b` / `released`) on the first row after the step. Tests:
`tests/test_physics_party.cpp`, section "Scheduled faults"; controller-side windows in `core/controller/CLAUDE.md`.

## Scheduling (KER-023)

Each party steps on its own period boundaries; its outputs hold between its steps; end checks at physics
boundaries (`tests/test_physics_party.cpp`, 1 kHz physics with a 500 Hz controller). Gantry: equal rates.

## Section index

| Section | File | Line |
|---|---|---|
| Fixtures | tests/test_physics_party.cpp | 29 |
| Exchange and config errors | tests/test_physics_party.cpp | 99 |
| Scheduling (KER-023) | tests/test_physics_party.cpp | 263 |
| Fault fixtures | tests/test_physics_party.cpp | 315 |
| Scheduled faults | tests/test_physics_party.cpp | 365 |
