# Dispatch queue

State for the coordinating session. A new session reads this, then `docs/rulings/`, then the two source docs.

## Current checkpoint

Round 4 (milestone 2) in flight on `round/04-milestone-2`, cut from main at 46cbfe6 on 2026-11-10. Plan of record:
`docs/briefs/round-04/plan.md` (signed off 2026-11-10). Wave 0 done inline: `debug` preset without clang-tidy,
`jobs: 2` on every preset, `tools/locked_build.sh` (one build at a time), README `python3-dev` row. Briefs of record
in `docs/briefs/round-04/`. Worktrees `../MechatronicsSimulator-04-<letter>` on `wip/04-<letter>`.

## Round 4 open rulings (owner answers when available; the planned default applies until then)

| # | Question | Planned default | Status |
|---|---|---|---|
| 1 | G9 hold on the bridge: the self-locking screw is refused by the bridge, so a de-energised lift falls. Brake on the lift as the `power_loss_hold` placeholder (a), a bridge self-locking approximation (b), or both (c)? E's firmware now turns the lift driver off in NOT_HOMED, so the hold rests on the brake output (reviewer: (a) or (c) needed for ACC-018). | (a) brake | open |
| 2 | IMU electrical interface: the chip vocabulary has no I2C or SPI mode. Model the IMU with analog outputs (accel x, y, z; gyro x, y, z) until a serial peripheral mode exists? | analog outputs, placeholder | open |
| 3 | FW-008 identical warning flags: on MSVC the host preset cannot share GCC flags. Read FW-008 as GCC host versus arm-none-eabi only, with the closest MSVC equivalent (`/W4 /WX`) on Windows? | yes | open |
| 4 | Re-tag ENG-016 to 021 (own-engine collision and contact) and LIB-009 (Python slow-path plugin) from M2 to M3? | re-tag | open |
| 5 | macOS stays out of the CI matrix for milestone 2 (R1 names Windows; the MuJoCo dmg cannot be extracted by CMake)? | out | open |
| 7 | After an E-stop `reset` the firmware returns to NOT_HOMED (steps may be lost while power was cut), not READY? | NOT_HOMED | open |
| 8 | Lift operator message is `lift <z>`, z in metres from the lift home switch; lowering is a lower z? | yes | open |
| 9 | C review: for a drive-driven shaft under slip, only the explicit-torque path is scaled by (1 - s); the reported transmission force is the force actually applied. Recorded as an approximation. | as stated | coordinator ruling, for confirmation |
| 10 | C review: a drive disabled then re-enabled under slip or backlash resets the slip and play references to the new rest, counting no increment. | as stated | coordinator ruling, for confirmation |
| 11 | A1 review (high): the stepper detent tangent is taken at the predicted lead delta_p = theta_cmd - (theta + omega h), rest theta + omega h + tau(delta_p) / K_t(delta_p), pull-out on delta_p; exact in steady tracking, identical to milestone 1 at rest. SIG-033 row edited. | adopted | coordinator ruling, for confirmation |
| 12 | A1 review (low): encoder noise and noisy-fault magnitude are in radians of shaft angle (the sensed quantity), not counts; alters the M1-D reading. | radians | coordinator ruling, for confirmation |
| 13 | A1 review (low): debounce passes a pulse of N = periods_ceil(debounce_time) samples (the brief's rule), although an N-sample pulse is only known to have held (N - 1) periods. | N samples | coordinator ruling, for confirmation |
| 14 | E review: FW-021 trips on any end switch active outside its own slow zone while READY or MOVING (wider than "moving away"); consistent with G11 "position tracking treated as wrong". | accept, row edited | coordinator ruling, for confirmation |
| 6 | Placeholders set by the coordinator for the gantry team's slots: overload limit rated + 15 percent (3.45 kg), heartbeat period 10 ms, watchdog timeout 50 ms, brownout threshold 2.0 V, encoder mismatch threshold 2 mm, ADC 12 bits at 3.3 V | as listed | open |

## Next

1. Wave 0 dispatch: G (schema-author) and W (Windows CI) in parallel. Then wave 1: A1, C, D, E.
2. Owner ruling 2026-11-05: the implicit joint drive stays while the MuJoCo bridge is the physics carrier; the
   question of how the stepper hold is applied (drive input versus the own engine's implicit springs, ADR 0013) is
   re-decided when the native adapter replaces the bridge (PHY-020, ADR 0004 retirement).
3. Contract files over 4 500 characters after the sweep (core/bindings 6.7K, mechsim/testing 6.4K, core/kernel 6.2K, firmware 4.9K, core/physics/mujoco 4.8K, core/signals 4.7K, core/engine 4.6K): current contract text, split or accept at the milestone 2 close.
4. Owed from reviews: two actuators on one shaft (builder sums rotor inertias, SIG-037 then refuses); PHY-020 native
   adapter and PHY-021 differential tolerance; mypy lambda note at tests/acceptance/test_acc_stubs.py:30.

## Milestone 1 rulings (owner, 2026-11-04)

Fault parameter forms (noise absolute SD, stall available-torque fraction, slip fraction); controller precondition
checked before t = 0, mismatch is an error; watchdog type and E-stop heartbeat input added now; DES-041 wiring is
connectivity only, physical wires a later component type; ASCII line operator codec for M1; MuJoCo pinned at the latest
3.9 patch via CMake; pybind11 through scikit-build-core; engine package 7 in parallel, not gating M1; chip definitions
as data with an STM32-family placeholder, real parts drop in by file. MVP: milestone 2 for the gantry team, milestone
3 plus CLI and replay for the standalone product.

## Rulings made during milestone 1 waves 1 and 2 (coordinator, 2026-11-04), confirmed by the owner 2026-11-05

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
- Engine binding surface (from the G review, invariant 5): engine types live in the submodule `_core_ext.engine`,
  importable only from `mechsim/physics/native/` and `tests/`; the import-boundary check enforces it. Neutral types
  stay flat in `mechsim._core`.
- Own-engine load torque (J): `shaft_load_torque = -g F` is the torque the transmission applies to the shaft,
  evaluated at the end state; the bridge publishes `tau_applied - J alpha` (the torque the shaft must supply). The
  integration brief aligns both to PHY-033 (the shaft-must-supply sign) so the stepper reads one convention.
- ENG-045 and ENG-046 (gear pair, slip parameter) pulled forward from milestone 2: delivered in package 7.
- Self-locking screw lowering under drive is unresisted in this tier (no nut-friction model); recorded in eng.md.
- Sensor placement is design data (DES-043, from the X2 blocker): a sensor instance carries `senses` (joint; limit
  switch position and side; encoder reversed) and the builder maps it to the signal layer's joint coordinate.
- Open parameters resolve case value, then the design's `default`, else an error (DES-044); the gantry draft ships
  placeholder defaults flagged until the gantry team rules.
- Gantry lift screw runs with zero backlash in milestone 1 (bridge refuses backlash until PHY-006, M2).
- Generated channel header names (`MECHSIM_CH_<SIGNAL>`) are the firmware convention; the committed gantry header is
  the builder's output, identity-tested.
- Kernel coupling values may be typed immutable payloads beside scalars (KER-008 row); KER-023 scheduling rule as
  written in ker.md; the physics party tests drift against the larger of the two thresholds until health carries
  per-kind drift (KER-016 row).
- Behavioral stepper is position-commanded (SIG-033 model defect found by G1 and G4: the feedforward torque law
  let the enabled lift creep under gravity): restoring torque `tau_hold sin(pi delta / (2 step_angle))` on the
  commanded-minus-shaft angle, viscous term, capped by the torque-speed curve; pull-out at two step angles
  re-synchronizes and counts missed steps, stall when it happens at zero commanded speed; disabled is zero torque.
- Design firmware section declares `state_codes` and `fault_codes` name maps (DES-045); the controller state the
  runner reports is `{state, fault}` by name and a case precondition names a state. Operator status message carries
  `state` and `fault` fields. App target `mechsim_fw_<design name>` is the convention.
- Stepper hold through physics (X4 found the signal-layer hold spring explicit and unstable on the lift at 1 ms,
  omega h 2.3): `PhysicsInputs.joint_drives` {joint, rest, stiffness, damping, enabled} applied implicitly by the
  adapter (MuJoCo: soft joint equality with direct solref; own engine: ADR 0013 implicit spring); the stepper
  publishes its commanded angle as the drive rest with the linearized detent stiffness and reads the drive torque back
  for pull-out and stall. Owner 2026-11-05: keep the drive on the bridge; re-decide with the native adapter (PHY-020). The 0.25 ms
  alternative (holds, 4x slower) stays recorded; the UART byte-loss defect it exposed is fixed (SIG-038).
- No-motion criterion (ACC-001, ACC-007): no step pulse commanded on any axis for the whole run and no axis leaving
  the power-on hold band, placeholder 1 mm (hold sag plus ringing; the gantry team owes the number). Hold ringing of
  a few milliradians is physical, not motion. Stepper damping placeholder 2e-3 N m s/rad (damping ratio near 0.1 at
  the lift).
- Signal layer accumulates controller-facing per-tick buffers (UART bytes, counter deltas) across its own steps and
  clears them at controller-tick boundaries (the root cause of the UART loss at 4 kHz physics with a 1 kHz tick).
- Owed after the gate: recorded joint positions and drive torques series; stepper sinusoidal detent (M2).
- Stepper above-curve torque (X review Q1): the motor loses synchronism whatever the lead; re-synchronize by at least
  one step toward the load, count it, stall at zero commanded speed.
- Shaft body axial inertia is the motor's rotor inertia, set by the builder (BLD-034 reworded); a design shaft body
  declaring its own is a build error; SIG-037 refusal wired through the Python wiring.
- PHY-034 drive mapping envelope: K_d A0 h^2 < 400 (omega h < 20), AdapterError beyond; the gantry lift sits at 2.7.
- Reference tier (validator): BLD-010 test asserts 1e-12 against a "not set, proposed 1e-9" row; SIG-011 asserts an
  exact count against "not set". Owner sets both rows at the gate (T4).
- Owner question for the gate (C re-review): where the transmission loss sits. The bridge reflects the rotor
  lossless, `(m + J/c^2) a = eta tau / c`; if the loss is in the nut or belt the exact rigid relation is
  `(m + eta J/c^2) a` driving and `(m + J/(eta c^2)) a` back-driven (5 percent of acceleration on the review
  fixture). Both adapters must use the same reading; the lossless one stands until ruled.

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
