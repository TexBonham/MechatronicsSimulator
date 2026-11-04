# core/kernel: contract

Namespace `mechsim::kernel`; target `mechsim_kernel`; tests `mechsim_kernel_tests`.

## Headers (`include/mechsim/kernel/`)

| Header | Signatures |
|---|---|
| `time.hpp` | `Rate(num, den=1)` rational Hz; `Seconds`; `SimTime`, `Duration` (int64 ticks); `lcm`, `period_in_ticks`, `ticks_at`, `to_seconds`; `KernelConfigError` |
| `party.hpp` | `PartyRole`; `Party`: `name`, `nominal_rate`, defaulted `role`, `published`, `consumed`, `start(base)`, `step(t, dt)`, `on_fault`; defaulted `set_seed`, `read_inputs(CouplingReader)`, `write_outputs(CouplingWriter)`, `finished`, `record(TickRecord&)`; `PartyFault(party, time, cause)` |
| `coupling.hpp` | `CouplingValue` variant<monostate, double, int64, bool>; `CouplingTable`; `CouplingReader` (`value`, `has_value`, `real`, `integer`, `flag`); `CouplingWriter::set`; `CouplingError` |
| `run_config.hpp` | `RunConfig{physics_rate, control_rate, party_rates, base_rate (kernel-written), seed, controller_mode, solver, drift}` |
| `kernel.hpp` | `Kernel`: `register_party`, `set_end_triggers(EndTriggers{at_time, on_signal, party_state})`, `run(EndCondition{end_time, stop}, sink) -> RunReport{outcome, base_rate, end_time, ticks, fault, end_reason, step_counts}`, `now`, `config`; `NullKernel` |
| `cosim_kernel.hpp` | `CoSimKernel(RunConfig)`, `coupling()`; `splitmix64_next`, `derive_child_seeds` |
| `null_party.hpp` | `NullParty(name, rate)`: tier 1 integrity slot until M3 |
| `physics_types.hpp`, `physics_adapter.hpp` | `PhysicsOutputs/Inputs/Health`; `PhysicsAdapter`; `AdapterError` |
| `tick_record.hpp` | `TickRecord{time, physics, pin_signals, rails, actuator_efforts, controller_state, events, coupling}` |
| `recorder_sink.hpp` | `RecorderSink`: `write_row`, `subscribe`, `unsubscribe`, defaulted `subscriber_errors`; `NullRecorderSink` |
| `memory_recorder_sink.hpp` | `InMemoryRecorderSink`: `rows`, `drain`, `drained`, `subscriber_count`, `subscriber_errors` |

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

## Open

KER-016 via `PhysicsParty`; KER-023 scheduling rule (owner); KER-019 Python party.
