# M3-X2e: X2 fix round, part two (torque margins and missed steps in the simulator's report)

E8 (rope segments taut within the step) and B3 (the payload's static start) are merged into `wip/05-x2`
(8803705) and X2d has reported (3c06a8e): G16's only lost steps before E8 came from the rope start-up blow, which
E8 removes (the E8 writer measured G16's rope peak at 78.9 N against a 333.6 N drop bound, no snap); with it gone
the nominal sweep must lose none and a motor that must lose steps must still be shown losing them in motion. X2d
left the `not long_run` tier at 21.6 minutes on this machine (from 37.6) with a session run cache in
`tests/conftest.py` and the acceptance checks of G2 to G11, G13, G14 behind the long-run marker; item 6 below
takes one more step.

Why: the X2 review (ruling 86) found G16's torque margins and missed steps printed by the acceptance test, not
carried by the simulator's own report, so the sizing result exists only in a pytest log. The simulator shall
report them (Requirements doc, G16: "report any missed steps per motor"; ACC-031, ACC-032).

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x2` on `wip/05-x2` (E8 merged:
C++ changed, so `tools/locked_build.sh sync` first). Call cap 110 with the handoff rule (at 100 calls stop adding
scope, commit what is green, report what is left by item). Read first: `mechsim/scoring/CLAUDE.md` (quantities,
`_SwingView` and `_StructuralView` as the pattern for a derived quantity; SCO-030), `mechsim/testing/CLAUDE.md`
(`RecordView`, `RunMetadata.bindings` for the parts of a sweep run), `mechsim/library/CLAUDE.md` (the stepper
part's torque data: holding torque, the pull-out curve or its placeholder), `tests/acceptance/test_acc_stubs.py`
(`_g16_missed`, `_g16_torque`, `G16_AVAILABLE`, `G16_HOLDING`, ACC-031, ACC-032), `test_cases/gantry/G16.yaml`,
rows ACC-031, ACC-032, ACC-038, SIG-033, LIB-017, SCO-030, round 5 rulings 85 to 87 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Builds and runs in the FOREGROUND only through
`tools/locked_build.sh run ...` with a 600000 ms timeout; filtered pytest while iterating (G16's sweep is a
`long_run` test of three runs: run it by name, at most three times in the brief); no wide run but item 6's one. Lints at the end:
`uv run ruff check .`, `uv run ruff format --check .`, `uv run mypy`, `python3 tools/check_imports.py`. COMMIT
AFTER EACH ITEM; final message "WIP M3-X2e: X2 fix round, part two", no attribution trailer, no push. Do not edit
`docs/queue.md` or `docs/cost-log.md`. No conditional (ternary) expressions. Curly braces on every control flow.
In spec table cells write abs(x), never bare pipe characters.

## Ownership

`mechsim/scoring/` for the two quantities and their report lines, `mechsim/testing/` only where the record view
must expose a part's data, `tests/acceptance/test_acc_stubs.py` (G16), `tests/unit/test_scoring*.py`,
`test_cases/gantry/G16.yaml` criteria ADDED (none changed), `docs/spec/{sco,acc,lib}.md` rows,
`docs/spec/traceability.md`, the contracts. Not yours: `core/**`, the stepper model, any existing criterion,
`baseline.yaml` except the G16 entries the new criteria add (regenerate G16's entries only, compare entry by
entry, list every change).

## Items, in order

1. Quantity `missed_steps:<motor>` (new SCO row): at every sample, the motor's commanded angle (the signed sum of
   its step pin's microsteps) less the rotor's travel, in full steps, less the drive's own lead (the sine law's
   inverse of the drive torque read back against the part's holding torque, as `_g16_missed` derives); the part
   and its holding torque come from the run's bindings and the library, never from a constant in a test. Rows:
   the derivation, its domain (a stepper on a step and direction driver with a recorded rotor position), what an
   unevaluable case is (no such driver, no rotor series: an error naming the motor).
2. Quantity `torque_margin:<motor>` (new SCO row): the part's available torque at the shaft's recorded speed
   (from the library's curve for the part; if the shipped parts carry only a holding torque and a placeholder
   curve, say so in the row and use what the catalog has, named) less the recorded shaft load torque, at every
   sample; the report shows its minimum and the time. Derive in the row what the demand is (PHY-033: g F at the
   step's end state) and how this differs from ACC-031's a-priori demand (m a r plus friction plus the lift's
   gravity term over the efficiency), which stays in the test as the derivation the measured value is checked
   against.
3. G16 gains criteria reading both for every motor: `missed_steps:<motor>` final equals 0 within a quarter step,
   `torque_margin:<motor>` minimum above 0 (the limits are the physics, not tuning: a sizing case passes when no
   motor loses a step and every margin is positive). The sweep's report then carries margins and missed steps per
   motor and part. ACC-031 and ACC-032 assert against the report's values (the test's own derivation stays as the
   cross-check, within a stated tolerance you derive).
4. A motor that must lose steps loses them in motion: a test (long_run) that runs G16's nominal case once with a
   `--set` on the payload mass (or the lift part through `instance:lift_motor.part`) chosen so the lift's demand
   during the move exceeds the part's available torque at the cruise speed by at least 20 percent by ACC-031's
   derivation; assert the stall during the lift move (the stall events' times inside the move window, not at the
   release), `missed_steps:lift_motor` final at least one, the margin negative, and the report saying so; the
   nominal sweep loses none after E8. State the mass or part used and the derivation in the test header.
5. Rows and docs: ACC-031, ACC-032 reworded to the report; LIB-017 if the catalog's curve is read; `docs/milestones.md`
   G16 row; the README's G16 line.
6. Fast tier (ruling 90): `test_phy_023` (191 s; it re-runs the reference tier in a fresh interpreter, which the
   same job has just run) moves behind the `long_run` marker; the workflow comment, the marker text and INV-020
   carry the new numbers (one timed `-m "not long_run"` run at the very end of the brief, your only wide run; expect
   about 18 minutes). The G3 determinism test, ACC-039 and the wiring test's G3 run stay where they are.

## Report

Under 250 words: commits; the two quantities' derivations in one sentence each and what the catalog provides;
G16's margins and missed steps per motor and part as the report prints them; the overloaded run's mass or part,
its stall times and count; the baseline entries changed; lints; open issues.
