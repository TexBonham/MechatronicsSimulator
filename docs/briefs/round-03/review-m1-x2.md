# Stage review: M1-X2 to X6 (driver, gantry end to end, stepper hold through physics)

Fable stage reviewer (W-02): from the spec and the diff only; do not read writer reports, cost log, HISTORY.md or
rulings files. Read-only. Call cap 35.

Worktree ../MechatronicsSimulator-m1-x2, diff `git diff ae4f7c7..1826496` (commits 4052643, eb2f669, 5c2daab, 6c96988, 1826496). Specs: docs/spec/{des,bld,tst,rec,sig,fw,acc,phy}.md for every ID cited or added (DES-043 to DES-045,
BLD-034, BLD-035, TST-039, REC-028, FW-031, SIG-033 reworded, SIG-037, SIG-038, KER-027, PHY-034, SCO-026, ACC-001 to ACC-008, ACC-033, ACC-034).
Contracts: mechsim/CLAUDE.md and the CLAUDE.md of mechsim/testing, mechsim/record, mechsim/builder, mechsim/design,
core/signals, firmware. Root CLAUDE.md invariants. docs/queue.md section "Rulings made during milestone 1 wave 1"
holds the coordinator's rulings the diff implements (sensor placement, open-parameter defaults, state-code names,
stepper hold model, zero backlash on the gantry lift screw, generated channel names); check the code matches them,
do not relitigate them.

Check, in order:
1. Stepper hold through physics (KER-027 joint drives, PHY-034 MuJoCo soft-equality drive, SIG-033 reworded): derive
   independently. The stepper publishes a drive {rest = commanded angle, K_hold = tau_hold pi / (2 step_angle),
   D_m, enabled}; the adapter applies it implicitly as a soft joint equality on the (reflected) joint with the
   writer's solref mapping (derivation in phy.md: s = (1 - d) K_d A0 with d = clamp(1 - 1/(K_d A0 h^2), 1e-4, 0.99));
   check that mapping reproduces a spring of stiffness K_d (sag F / K) and damping D_d for all K at h = 1 ms, that
   there is no omega h limit, that the drive torque fed back and the PHY-033 load torque keep one sign convention,
   pull-out at two step angles or above the torque-speed curve with re-synchronization, stall at zero commanded
   speed, fault scaling, disabled = no force. Signs with direction input and reversed shafts. A sign, mapping or
   clamp error is a blocker.
1b. Unequal rates (SIG-038): the layer accumulates controller-facing per-tick buffers across its steps and clears
   at controller-tick boundaries; check nothing arrives twice or is lost at 4 kHz / 1 kHz and that equal rates are
   unchanged.
1c. No-motion criterion (SCO-026, ACC-001, ACC-007): zero commanded step pulses on every axis plus the hold band;
   G4's criteria start at `${homing_time}` (homing must move the axes): judge the reading.
2. Invariant 1 on the Python side: CoreDriver assembles parties and the coupling names; nothing hands physics state to
   the controller party; the firmware sees only PinInputs.
3. Invariant 2: nothing gantry-specific in mechsim/ or core/ (sensor placement, state names, channel header names and
   the app target convention are generic; the gantry values live in designs/ and test_cases/ and firmware/).
4. Invariant 3: schemas regenerated for DES-043 to DES-045 and the test-case schema; designs and cases validate;
   open-parameter defaults flagged as placeholders.
5. Invariant 4: the G3 run-twice determinism test compares recorded content and byte identity; no wall-clock or
   unordered iteration in the driver, prepare, record or wiring code.
6. Invariant 7: every failure path in CoreDriver and prepare ends the run with a typed error; translation report
   lines reach the run report and archive metadata; `sink_errors` and a raising Python `on_fault` reach the report.
7. ACC-001 to ACC-008 conversions: each test runs the real stack, asserts the case criterion from the spec row, and
   where the row's tolerance is "not set (owed by gantry team)" asserts the behavior and names the ID; no
   implementation-derived expected values; baselines flagged placeholder.
8. KER-023 delivery half: commands and faults reach their owner at the first step at or after the scheduled time, with
   a test through the runner timelines.
9. R-18 and code rules (braces, no ternaries, comments only for a non-obvious why; Python typed, every edge case).

Run once: `uv sync` if needed, `uv run pytest -q tests/acceptance tests/unit/test_core_driver.py tests/unit/test_gantry_integration.py`,
and `ctest --preset debug-sanitize -R "signal|stepper" -j2` after `cmake --build --preset debug-sanitize -j2`.

Report: verdict per commit group (accept / accept with fixes / block); findings table
`| Severity | File:line | Finding | Required change |`; owner questions; numbers once (G1 to G4 measured values
against criteria, wall times).
