# M3-N: native adapter over the own engine (PHY-020), carrying every case

Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-n` on branch `wip/05-n` from the round branch
after E3 lands (the engine with contact, members, ropes, mid-run changes, bindings and the clamped far end). Read
first: root `CLAUDE.md`, `core/CLAUDE.md`, `core/engine/CLAUDE.md`, `core/kernel/CLAUDE.md` (`PhysicsAdapter`,
`PhysicsOutputs`, `PhysicsInputs`, joint drives, scheduled faults), `core/physics/mujoco/CLAUDE.md` and
`mechsim/physics/mujoco/CLAUDE.md` (the behaviours the native adapter must reproduce: reflection of transmissions,
PHY-033 load torque, PHY-034 drives, slip and backlash, payload weld, contact pairs, joint positions and drive
torques, approximation report), `core/physics/native/README.md` and `mechsim/physics/native/` (the skeleton),
`mechsim/physics/CLAUDE.md` (neutral model, `prepare_model`), spec rows PHY-001 to 012, 018, 019, 020 to 022, 027 to
034 in `docs/spec/phy.md`, ENG-043 to 048, round 5 rulings 1, 9, 13, 16, 21 in `docs/queue.md`, ADR 0003, 0013,
0014, 0016. Call cap 150 with a handoff rule (commit WIP at the cap, list the rest for N2). Iterate with
`tools/locked_build.sh core debug -R "'native|Native'"` (name your ctest cases so the regex finds them) and the
reference tier on the native adapter; at the end `tools/locked_build.sh core debug-sanitize`, `tools/locked_build.sh
sync`, `tools/locked_build.sh run uv run --no-sync pytest -q tests/reference tests/unit/test_phy_stubs.py
tests/acceptance/test_phy_stubs.py tests/invariant`, then P1 and G3 on the native adapter through the CLI (`--set
physics.adapter=native` or the run-configuration field you add: state it). Commit as you go on `wip/05-n`; final
message "WIP M3-N: native adapter", no trailer, no push. Spec edits same commit.

## Ownership

`core/physics/native/**` (the C++ adapter: `NativeAdapter` implementing `kernel::PhysicsAdapter` over
`engine::MultibodyProvider` and `SemiImplicitEuler`), `mechsim/physics/native/**` (translation of the neutral model
into the engine's `MultibodyModel`: bodies, six joint types, geometries and contact pairs from `ContactModel`,
lumped members with their support conditions, ropes, transmissions, fastenings, payloads; `prepare_model` for the
native adapter produces the engine model JSON, no artifact), `core/bindings/` for the adapter, `mechsim/testing/
core_driver.py` and `core_wiring.py` only for the adapter selection (run configuration `physics.adapter`, default
`mujoco` until X2 switches it), `tests/reference/test_phy_stubs.py`, `tests/unit/test_phy_stubs.py`,
`tests/acceptance/test_phy_stubs.py`, `tests/unit/test_native_*.py` (new), `core/physics/native/CLAUDE.md`,
`mechsim/physics/native/CLAUDE.md`, `docs/spec/phy.md`, `docs/spec/traceability.md`. Not yours: `core/engine/`
(list any engine gap under Open issues with the test that shows it), `core/physics/mujoco/`, `designs/`, `test_cases/`.

## Deliverables, in order

1. Load and step (PHY-001, 002, 013-equivalent for the engine, 020, 031, 032): the neutral model to a
   `MultibodyModel` in neutral order, joint types mapped (free joint for payloads), the engine stepped at the run's
   physics rate with the engine's step config; health mapped (drift, convergence, iterations) and every engine error
   ends the run as `AdapterError` (PHY-009, invariant 7); determinism (PHY-010, run twice bitwise).
2. Outputs (PHY-003, 004, 007, 033, 034): bodies, joints (generalized forces and reactions), constraints, contacts,
   members (start and end wrenches from E2's member loads; state the frame E2 reports them in, and gate the
   root-to-member rotation in `core_driver.py` (R review note, line ~463) on the bridge so it is never applied
   twice: the recorder's REC-004 series are in the member frame of ruling 24), ropes (load-end and max tension from E2's per-segment
   output; the mapping stated), transmissions (PHY-033 load torque, force, stretch, power flow, slip, scheme_decay),
   joint positions, joint drive torques, payload states; joint drives applied as the engine's implicit joint springs
   with rest, stiffness and damping per tick (PHY-034 on the own engine: exact sag and steady lag, the envelope
   stated for backward Euler: no stability limit, the x^2 / 3 frequency error and e^(-pi x) decay published per
   driven joint as approximation values, ruling 21).
3. Transmissions and faults (PHY-006, PHY-018, ENG-043 to 048): shaft joints as the engine's transmissions with
   efficiency by power direction, compliant belts (ENG-043 stiffness), lead screws with self-locking, gear pairs,
   backlash, scheduled slip through `set_slip`, two actuators on one shaft refused; payload mass change, attach and
   release through E1's calls; mid-run spring offset, stiffness, constraint disable, load release (PHY-008) for
   brief I2.
4. Contact (PHY-027, 029, 030): gravity, joint friction and limits, the design's `ContactModel` pairs and exclusions
   into E1's contact (K_n, C_n, mu per pair, the slip-velocity default), the floor; the two references pass on the
   native adapter.
5. Reference tier (PHY-022, PHY-028): every analytical reference that runs on the bridge runs on the native adapter
   too (parametrise the existing reference tests over the adapter); the native adapter passes all of them, including
   the pendulum at 0.1 percent of period, the beam references at 2 percent with eight segments, the rope pendulum
   reference at its 50 microsecond step (ruling 21: the physics step is per run).
6. The gantry: G3 and P1 pass on the native adapter through the CLI with the suite's criteria (the drive, belts,
   switches and encoders all on the own engine); report every criterion's measured value against the bridge's
   baseline (the differential check X2 will formalise under PHY-021 with ruling 13's tolerances).
7. Approximations recorded per run (SCO-013 line): rigid-vs-compliant choices, the one-witness-point contact,
   the backward-Euler decay of every oscillatory mode the adapter can name.

## Report

Under 300 words: files, ctest counts on `debug` and `debug-sanitize`, the reference tier result on the native
adapter, G3 and P1 criterion values against the bridge, engine gaps found (with the test that shows each), items
left for N2, Open issues.
