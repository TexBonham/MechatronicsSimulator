# Dispatch queue

State for the coordinating session. A new session reads this, then `docs/rulings/`, then the two source docs.

## Current checkpoint

Milestone 1 in flight on `round/03-milestone-1`. Wave 0 landed (M1-S plus the vocabulary and heartbeat contract
edits, commits 3f316b9 and 2eb0a2e). Wave 1 dispatched: ten briefs in worktrees `../MechatronicsSimulator-m1-{a..j}`
on branches `wip/m1-{a..j}` from base 2eb0a2e: A builder, B kernel, C MuJoCo bridge, D signal built-ins, E native
runtime, F recorder, G bindings, H gantry firmware, I runner and scoring and CLI, J engine package 7. Owner returns at
the milestone 1 gate.

## Milestone 1 rulings (owner, 2026-11-04)

Fault parameter forms (noise absolute SD, stall available-torque fraction, slip fraction); controller precondition
checked before t = 0, mismatch is an error; watchdog type and E-stop heartbeat input added now; DES-041 wiring is
connectivity only, physical wires a later component type; ASCII line operator codec for M1; MuJoCo pinned at the latest
3.9 patch via CMake; pybind11 through scikit-build-core; engine package 7 in parallel, not gating M1; chip definitions
as data with an STM32-family placeholder, real parts drop in by file. MVP: milestone 2 for the gantry team, milestone
3 plus CLI and replay for the standalone product.

## Next

1. Wave 1 lands: Fable reviews in pairs, audits, merge-back, stage commits; one fix round each budgeted.
2. (dispatched) Wave 1, ten concurrent briefs: M1-A builder, M1-B kernel, M1-C MuJoCo bridge, M1-D signal built-ins, M1-E native
   runtime, M1-F recorder, M1-G bindings, M1-H gantry firmware, M1-I runner, scoring and CLI; M1-J engine package 7
   non-blocking.
3. Wave 2: M1-X integration, G1 to G4 and P1 end to end, run-twice determinism, reference-validator; owner gate.

## Rulings made during milestone 1 wave 1 (coordinator, 2026-11-04), for owner confirmation

- Direction and enable: the pin-table-bound digital outputs are the single source; the native shim derives the
  stepper arrays from them; firmware holds the stepper arrays low.
- Frozen controller: latched levels hold, per-tick buffers are empty (no pulses, no bytes, no events); timer-driven
  pulses that continue on a hang are a milestone 2 option.
- Reset and brownout recovery reload a private copy of the firmware library, so no firmware static survives.
- Board interface 2.2: `mechsim_app_board_interface_version()` entry point implemented by every app.
- Physics step is run configuration, not design data; the builder no longer ties it to the firmware tick.
- Milestone 1 runs rigid: BLD-033 rigid-members option; the gantry ships placeholder primitive geometry until CAD.
- Kernel: control_rate maps onto the controller party by role; a physics party whose rate differs from
  physics_rate is a config error.
- MuJoCo bridge transmissions (from the C review): a joint equality leaks metres with a light shaft on a heavy
  load, so the bridge maps a rigid transmission by reflection: no shaft hinge in MJCF, rotor inertia reflected
  as `armature = J / c^2` on the driven joint, shaft torque applied as `tau / c`, shaft position and velocity
  reported as `q / c`, `qd / c`. Efficiency by power direction (driving `eta tau / c`, back-driven `tau / (eta c)`).
  `self_locking: true` refused in the bridge (own engine handles it); finite stiffness treated as rigid and recorded
  as an approximation; backlash and slip stay refused until milestone 2.
- Shaft load torque (ENG-048, SIG-033): `tau_load = tau_applied - J_rotor * alpha_shaft`, positive along the shaft's
  + axis, published as the shaft joint's generalized force; the stepper's demand is `J alpha_commanded + tau_load`.
  Rotor inertia has one source, the motor instance's `rotor_inertia`: the builder copies it into the shaft body and
  the signal layer reads the same value.
- Explicit MuJoCo contact pairs bypass the parent-child filter: the bridge skips pairs between joint-connected bodies
  and records them.
- PHY-028 pendulum period tolerance set to 0.1 percent (T4 spec edit; the reviewer measured 0.0135 percent).
- Signal layer: counters refuse a forced-level fault (recommendation accepted); relay floor, debounce ceiling and
  watchdog rounding to whole ticks accepted; UART pacing is 8N1 at the configured baud.
- Catalog alignment owed to the integration brief: `stepper_behavioral` gains a torque-speed table parameter
  (holding torque alone stalls any motion above zero speed); `estop_relay` keeps no heartbeat input, the watchdog's
  trip output drives it; power supply output named `v_out` to match the pin.
- Milestone 3 note: MuJoCo implicitfast treats member-node stiffness explicitly, stable only for omega h < 2 per
  segment, so the bridge cannot be the oracle for fine-segment beams; retirement baselines stay inside that envelope.

## Rulings made overnight 2026-11-03/04, confirmed by the owner 2026-11-04

- Kernel time: integer ticks of a per-run base period (1 / lcm of all party and physics rates), rates as rational
  hertz, instead of int64 nanoseconds with whole-nanosecond periods (which rejected 3 kHz, 60 Hz and sub-hertz rates).
  Reviewer's recommendation; cheaper now than after milestone 1 builds on it.
- Firmware-declared tick rate (`mechsim_app_tick_hz`) kept as a consistency check: the runtime errors at load when it
  differs from the design's control rate; the host build injects the design's value so 1 kHz lives only in data (C4).
- Board interface sizes are fixed contract constants shared by the C header and the C++ PinInputs/PinOutputs: 32
  digital in, 8 counters (per-tick deltas), 16 ADC, 64-byte UART each way, 8 axes, 32 digital out, 16-entry event log
  with state and fault codes (CTL-018 channel). The rail voltage is not a firmware input.
- PhysicsAdapter.load takes a prepared model (neutral JSON plus an engine artifact such as MJCF) per ADR 0014.

## Owner rulings 2026-11-04 (from the C3 and C4 reviews)

- Operator message labels: split into two messages, confirmed.
- Heartbeat routing: a pin signal into the E-stop relay or watchdog, not an operator message (applied in the C3 fix
  round). The catalog has no watchdog type yet (LIB-019, M2) and estop_relay has no heartbeat input; both are owed.
- Contact model parameters and the floor live in the design environment (DES-042, M1).
- Instance wiring and rail consumers are DES-041 (design file), owed before the builder brief.
- G3 targets carry units: 40 cm, 25 cm in the homed origin frame (G2). Per-axis 1 cm criterion confirmed. P1 damped
  period at 0.1 percent confirmed. ADR 0015 accepted.
- Needs discussion with the owner: fault parameter forms, the controller precondition check, heartbeat wiring once a
  watchdog type exists.
- Gantry kinematic chain, hook on a spherical joint standing in for the M3 rope, joint limits 0 to workspace: writer
  defaults until the gantry team's CAD arrives; record the hook placeholder in the M3 rope ADR.

## Rulings made at merge-back, confirmed by the owner 2026-11-03 with the round 1 commit

- Steppers and drivers are signal-layer models (SIG-032 to SIG-036); transmissions, ropes and lumped beams are
  engine-side neutral-model elements (ENG-043 to ENG-048). Shaft torque crosses out, load torque crosses back.
- ENG-011: the sixth joint type is the free joint (six degrees of freedom) for unattached bodies.
- Flexible-member tuning: the builder tunes (BLD-019); integrity imports FEA results into the design file (INT-034).
- Tier 1 integrity reads live loads through the kernel; everything else reads the recording (REC-001 carve-out).

## Parked

- C++ strict expected-failure convention for Catch2 stubs (ADR 0008 open item; `[!shouldfail]` is the candidate): rule in the phase 0b plan.
- Stage-review nits carried to the next docs-indexer pass: hooks and `tests/` outside mypy scope; engine package working names in `docs/milestones.md` unruled; `INV-nnn` numbering assumed equal to invariant numbers until the spec exists.

- Chrono adapter status (ADR 0004 open item): decide with the phase 1 contracts.
- C++ test markers for requirement citations (ADR 0008 open item): phase 0c.
