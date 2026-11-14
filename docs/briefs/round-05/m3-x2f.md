# M3-X2f: G16's sizing criteria over the evaluation window; a motor that must lose steps in motion (ruling 91)

Why: X2e put `missed_steps:<motor>` and `torque_margin:<motor>` into the report and G16's criteria, and found the
NEMA 17 lift losing one step at 0.065 s with its least margin -0.123 N m at 0.022 s: the release transient of the
lift belt, which starts unstretched and lets the hook dip (the rope's snatch 79 N against 44.5 N static). Ruling
87 assigns belts and rails starting unloaded to milestone 4's static start, behind the suite's evaluation start
(9 s), and ruling 91 applies that to the sizing criteria: they judge the evaluation window, and the report still
carries what happened before it. X2e could not build the overloaded run from a payload mass: the load cell's
overload limit (5.216 kg) is below any mass that over-demands either part, and a held lift exceeds its cruise
curve by at most about 10.7 percent; ruling 91 makes the overload a motor-parameter override instead.

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x2` on `wip/05-x2` (head 49e6bc7;
the extension is current). Call cap 80 with the handoff rule (at 70 calls stop adding scope, commit what is green,
report what is left by item). Read first: `docs/briefs/round-05/m3-x2e.md` and its rules, `mechsim/scoring/CLAUDE.md`
(SCO-031, SCO-032 as X2e wrote them; windows and statistics), `mechsim/testing/CLAUDE.md` (criteria windows; how
the suite's 9 s evaluation start reaches a criterion), `mechsim/library/catalog/stepper_behavioral.yaml` (the
parameters an instance may override: `holding_torque` is one; whether the pull-out curve is scaled by it or a
separate table), `test_cases/gantry/G16.yaml`, `tests/acceptance/test_acc_stubs.py` (ACC-031, ACC-032, the
overload test X2e left), rows SCO-031, SCO-032, ACC-031, ACC-032, LIB-016, LIB-017, round 5 rulings 85 to 91 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Builds and runs in the FOREGROUND only through
`tools/locked_build.sh run ...` with a 600000 ms timeout; filtered pytest; G16's sweep (`long_run`, three runs) at
most twice; the overloaded single run as often as needed but through the CLI, not the sweep; no wide run. Lints at
the end: `uv run ruff check .`, `uv run ruff format --check .`, `uv run mypy`, `python3 tools/check_imports.py`.
COMMIT AFTER EACH ITEM; final message "WIP M3-X2f: sizing criteria over the window", no attribution trailer, no
push. Do not edit `docs/queue.md` or `docs/cost-log.md`. No conditional (ternary) expressions. Curly braces on
every control flow. In spec table cells write abs(x), never bare pipe characters.

## Ownership

As X2e (`mechsim/scoring/`, `mechsim/testing/` for the stepper drive data and criteria windows, G16's new criteria
only, `tests/acceptance/test_acc_stubs.py` G16, `tests/unit/test_scoring*.py`, the SCO, ACC and LIB rows,
`docs/spec/traceability.md`, `docs/milestones.md` G16 row, the README's G16 line, the contracts; G16's baseline
entries only). Not yours: `core/**`, the stepper model, any existing criterion, the shipped catalog values.

## Items, in order

1. The sizing criteria judge the evaluation window. G16's `missed_steps:<motor>` criterion reads the steps lost
   within the window (from the case's evaluation start to the end): either a statistic that gives the change over
   the window (final less the value at the window start, a new SCO statistic with its row) or the quantity defined
   relative to the window start; say which and why. The `torque_margin:<motor>` criterion's minimum is taken over
   the window by the same mechanism the suite's other criteria use for the 9 s start (find it; do not invent a
   second one). The limits stay the physics: zero steps within a quarter step, margin above zero. The report then
   also carries, per motor, the steps lost and the least margin BEFORE the evaluation start as a before-evaluation
   line (as `held_tension` is reported for ropes, PHY-036), so the release transient is visible, not hidden. Rows
   SCO-031, SCO-032 and the ACC rows state the window rule and cite ruling 91.
2. A motor that must lose steps in motion. The overloaded run is G16's nominal case with
   `--set instance:lift_motor.holding_torque=<value>` (LIB-016; if the pull-out curve is a table the override does
   not scale, scale it in the drive data by the ratio of the overridden to the catalog holding torque and say so
   in the SCO-032 row, or override the curve's parameter if the catalog exposes one; a test-local registry part is
   the fallback, never a change to the shipped catalog). Choose the value so the lift's demand at cruise exceeds
   the available torque by at least 20 percent by ACC-031's derivation; state the derivation and the value in the
   test header. Assert: stall events inside the lift move's window (not before the evaluation start), the windowed
   missed steps at least one, the windowed margin negative, and the report saying so. The payload mass stays the
   case's.
3. ACC-031 and ACC-032 assert against the report's windowed values: the nominal sweep and the NEMA 17 sweep lose
   none within the window and every windowed margin is positive (X2e's numbers: lift NEMA 17 +0.053 N m at cruise
   by derivation); the before-evaluation line reports the NEMA 17 lift's one lost step and its -0.123 N m at
   0.022 s. The a-priori derivation stays as the cross-check at the tolerance X2e stated.
4. G16's baseline entries regenerated once (the criteria changed), compared entry by entry, every change listed
   with old value, new value and cause; nothing else in `baseline.yaml` touched.
5. Rows and docs: SCO-031, SCO-032, ACC-031, ACC-032 reworded; `docs/milestones.md` G16 row; the README's G16
   line; the contracts (`mechsim/scoring/CLAUDE.md`, `mechsim/testing/CLAUDE.md`).

## Report

Under 250 words: commits; the window mechanism used and the statistic if new; G16's windowed margins and missed
steps per motor and part and the before-evaluation line as the report prints them; the overloaded run's override
value with its derivation, stall times and count; the baseline entries changed; lints; open issues.
