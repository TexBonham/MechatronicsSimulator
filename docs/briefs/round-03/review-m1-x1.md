# Stage review: M1-X1 core wiring

Fable stage reviewer (W-02): review from the spec and the diff only; do not read the writer's report, cost log,
HISTORY.md or rulings files. Read-only, change nothing. Call cap 30.

Worktree ../MechatronicsSimulator-m1-x1, commits f02150e and 09b6065 on top of d4c7459:
`git diff d4c7459..09b6065`. Specs: docs/spec/{ker,sig,ctl,phy,eng,lib}.md for every ID the diff cites or edits
(new rows SIG-037, LIB-034, PHY-033 alignment, KER-023). Contracts: the CLAUDE.md of core/kernel, core/signals,
core/controller, core/bindings, core/physics/mujoco, core/engine, mechsim/physics. Root CLAUDE.md invariants.
Rulings in docs/queue.md section "Rulings made during milestone 1 wave 1" are the coordinator's decisions the
diff implements (load-torque convention PHY-033, rotor inertia single source, estop_relay without heartbeat input,
engine binding behind `_core_ext.engine`); do not relitigate them, check the code matches them.

Check, in this order:
1. Invariant 1: every physics-to-controller and controller-to-physics path now crosses the kernel coupling table
   through the signal layer (`physics.outputs` -> signals -> `signals.pin_inputs` -> controller; `controller.pin_outputs`
   -> signals -> `signals.physics_inputs` -> physics). Any direct setter or getter call bypassing the table, or any
   payload that carries true physics state to the controller party, is a blocker.
2. Load torque: one sign convention end to end (engine publish point, bridge publish point, neutral TransmissionState,
   stepper demand). Derive the expected sign from PHY-033 and check all four. A mismatch is a blocker.
3. Rotor inertia: the signal layer's check against the shaft body's axial inertia, and the stepper's demand formula.
4. The header-only pins target: no include from core/controller into core/signals or back.
5. Bindings: trampolines forward the new hooks; lifetimes (no raw owning pointers, objects outliving the kernel);
   libmujoco installed next to the extension with `$ORIGIN` rpath; error translation for MujocoAdapter.
6. Catalog alignment: `stepper_behavioral` torque-speed table semantics (interpolation, zero above the last point),
   `estop_relay` without heartbeat, `v_out`; schemas regenerated; designs still validate.
7. KER-023 rule and its test; typed coupling payloads (ruled, check determinism and immutability).
8. Tests derived from physics not implementation (pendulum closed form, negative-ratio signs, load-torque case).
9. Spec maintenance (R-18) and C++ rules (braces, no ternaries, no comments without a non-obvious why).

Build and test once, -j2: `cmake --build --preset debug-sanitize -j2` and `ctest --preset debug-sanitize -j2` in the
worktree, and `uv run pytest -q tests/unit/test_bindings.py` (if the extension is stale, `uv sync` rebuilds it).

Report: verdict (accept / accept with fixes / block); findings table `| Severity | File:line | Finding | Required change |`;
items the driver brief (M1-X2) must know; numbers once.
