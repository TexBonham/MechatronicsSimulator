# Dispatch queue

State for the coordinating session. A new session reads this, then `docs/rulings/`, then the two source docs.

## Current checkpoint

Phase 1 complete. Waves 1 and 2 landed as four stage commits on `round/02-phase-1`; wave 3 (test case and baseline
files) is STAGED as the phase 1 closing commit, awaiting the owner gate. Every check green on the staged tree: 2 187
Python tests, 444 strict-xfail stubs, 558 requirements all covered, 109 core tests, 32 firmware tests, hooks pass on
every shipped YAML. Nothing dispatched.

## Next

1. Owner gate: review the phase 1 closing commit, confirm the overnight rulings below, rule on the owner items below,
   and accept or change ADR 0015 (facade protocol, Proposed).
2. Round-close hygiene for round 2 (archive cost-log rows, promote the shared `_Strict`/`_format_errors` helpers to a
   public module, sweep contract files), then the round 2 PR to main.
3. Milestone 1 plan for sign-off on the measured cost model: builder (design to neutral model), kernel and signal
   built-ins, native controller runtime, MuJoCo stepping adapter (ADR 0014), recorder, scoring, CLI, gantry firmware
   for G1 to G4, engine package 7 (transmissions) first.

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
