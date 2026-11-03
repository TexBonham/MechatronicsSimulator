# core/kernel: contract

Namespace `mechsim::kernel`; target `mechsim_kernel`, tests `mechsim_kernel_tests`. No engine types.

## Headers (`include/mechsim/kernel/`)

| Header | Signatures |
|---|---|
| `time.hpp` | `Rate(num, den = 1)` reduced rational Hz; `Seconds(num, den)`; `SimTime`, `Duration` (int64 base ticks); `lcm(Rate, Rate)`, `period_in_ticks(rate, base)`, `ticks_at(Seconds, base)`, `to_seconds(d or t, base)`; `KernelConfigError` |
| `party.hpp` | `Party`: `name()`, `nominal_rate()`, `published()`, `consumed()`, `start(base_rate)`, `step(t, dt)`, `on_fault(const PartyFault&) noexcept`; `PartyFault(party, time, cause)` |
| `run_config.hpp` | `RunConfig{physics_rate=1 kHz, control_rate=1 kHz, party_rates, base_rate (kernel-written), seed, controller_mode, solver, drift}` |
| `kernel.hpp` | `Kernel`: `register_party(unique_ptr<Party>)`, `run(EndCondition{Seconds, stop}, RecorderSink&) -> RunReport{outcome, base_rate, end_time, ticks, fault}`, `now()`, `config()`; `NullKernel(RunConfig)` |
| `physics_types.hpp` | `PhysicsOutputs{steps, bodies, joints, constraints, contacts, members, ropes, transmissions}`, `PhysicsInputs`, `PhysicsHealth{drift, converged, iterations, steps, physics_rate}` |
| `physics_adapter.hpp` | `PhysicsAdapter`: `supported_elements()`, `load(PreparedModel{json, engine_artifact, design_version}, physics_rate)`, `reset()`, `step()`, `outputs()`, `health()`, `set_inputs()`, mid-run `set_spring_offset/_stiffness`, `disable_constraint`, `release_load`, `set_payload_mass`, `set_payload_attachment`; `AdapterError(step, cause)` |
| `tick_record.hpp` | `TickRecord{time, physics, pin_signals, rails, actuator_efforts, controller_state, events}` |
| `recorder_sink.hpp` | `RecorderSink`: `write_row()`, `subscribe(cb) -> SubscriptionId`, `unsubscribe()`; `NullRecorderSink` |

## Per-tick order (KER-009)

Registration order is step order: physics, tier 1 integrity, sensors, controller, pin-output collection,
drivers/steppers/transmissions; the kernel writes one recorder row per physics step, last.

## Conventions

- Time base: at `run` start, base rate = lcm(physics rate, every party rate); a tick is 1 / base rate. Any
  rational rate works (3 kHz, 60 Hz, 0.1 Hz); int64 overflow is a config error naming the party.
- `end_time` off the base tick: `KernelConfigError` before t = 0. Seconds only via `to_seconds()`.
- A party steps when t % period == 0, period = base / rate ticks. `party_rates[name]` beats `nominal_rate()`.
- Party exception -> `PartyFault(name, start tick, what())`; `on_fault` fans out; no row; `run_error`.
- Adapter: neutral JSON parsed by each adapter (invariant 3); step length fixed at load; inputs one step;
  mid-run changes persist; indices in neutral-model file order; misuse -> `AdapterError`.
- No unordered containers, threads or wall clock on the tick path.

## Milestone 1 implements

Real kernel: coupling exchange (KER-008, KER-011), seeds, scheduling; `PhysicsParty`; adapters in `core/physics/`.
