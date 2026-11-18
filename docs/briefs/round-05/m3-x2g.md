# M3-X2g: X2d fix round (the real mechanism behind G7's and G9's 1.03 mm; G10's remainder; the driven mass of record)

Why: the X2d review (ruling 92) found the G7 and G9 explanation numerically right (2.180 mm on the bridge and
1.153 mm on native against 2.177 and 1.146 observed) but attributed to a mechanism the model does not apply: it
names the stepper's viscous damping acting on the unpowered rotor on native, while `core/signals/src/stepper.cpp`
publishes a disabled drive with zero damping whenever the supply is 0 or enable is low and the native adapter
clears disabled drives (SIG-033). Something native-only applies about 197 N s/m during those 20 ms; until it is
named from the archive or a unit experiment, G7 and G9 are findings, not explained, and the retirement decision
waits. The review also found G10's model incomplete rather than wrong (the crossing shift of 2 and 5 ms is
predicted; the remainder sits in the stop phase from detection to the fault at zero commanded velocity and in the
carriage's coast during the stall), two small inconsistencies, and a test literal that belongs to the build.

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x2g` on `wip/05-x2g`, cut from
`wip/05-x2` at 9343427 (no build yet: `tools/locked_build.sh sync` first; brief X2f is working concurrently in
`../MechatronicsSimulator-05-x2` on scoring and G16: do not touch that worktree or those files). Call cap 90 with
the handoff rule (at 80 calls stop adding scope, commit what is green, report what is left by item). Read first:
`docs/briefs/round-05/m3-x2d.md` (its rules hold: no tuning; nothing of MuJoCo deleted; defects in code you do not
own reported with a reproduction, not patched; bridge baselines and `baseline.yaml` not regenerated),
`review-m3-x2de.md`, `tests/acceptance/test_phy_stubs.py` (the G7 derivation near line 300, the tolerance near
line 150), `test_cases/gantry/README.md` (the G7, G9, G10 notes X2d added), `docs/milestones.md` row 38,
`core/signals/CLAUDE.md` (the stepper: SIG-033, what a disabled drive publishes), `core/physics/native/CLAUDE.md`
(disabled drives; the belt's regularised efficiency switch, ruling 47; transmission damping, ruling 80, brief D2),
`mechsim/builder/CLAUDE.md` (`driven_mass`, BLD-041), `firmware/` where the stall and slip faults are raised
(`gantry.c` near lines 822 to 837: detection, the stop phase, the fault at zero commanded velocity), rows PHY-024,
SIG-033, BLD-041, round 5 rulings 41, 47, 80, 83, 86, 90 to 92 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Builds and runs in the FOREGROUND only through
`tools/locked_build.sh run ...` with a 600000 ms timeout; single cases through the CLI into the scratchpad
`/tmp/claude-1000/-home-tex-dev-MechatronicsSimulator/e12b8ab0-2af1-4dae-a612-eacb9e002661/scratchpad/x2g/`;
filtered pytest; no wide run. Lints at the end: `uv run ruff check .`, `uv run ruff format --check .`,
`uv run mypy`, `python3 tools/check_imports.py`. COMMIT AFTER EACH ITEM; final message "WIP M3-X2g: G7 and G9
mechanism, G10 remainder", no attribution trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`. No
conditional (ternary) expressions. Curly braces on every control flow. In spec table cells write abs(x), never
bare pipe characters.

## Ownership

`tests/acceptance/test_phy_stubs.py`, `tests/conftest.py` (docstring only), `test_cases/gantry/README.md`,
`docs/milestones.md` (criterion 2 row), the PHY-024 row, `docs/spec/traceability.md`; a unit experiment under
`tests/reference/` or `tests/unit/` if item 1 needs one (data-only rigs, no gantry values in code). Not yours:
`core/**`, `mechsim/scoring/`, `mechsim/testing/`, `firmware/**`, any case file, the baselines, G16.

## Items, in order

1. The mechanism behind G7's and G9's held position. Run G7 once per adapter (`--set physics.adapter=mujoco` for
   the bridge) and read the lift joint through the 20 ms after the E-stop: position, velocity, the shaft load
   torque, the belt tension or transmission force, the brake state, the drive's published damping and enable, and
   whatever the native run-time mode lines report for the lift belt (`scheme_decay`, the efficiency switch, the
   belt's damping ratio from brief D2). Decompose the carriage's deceleration on each adapter into named terms
   from the archive (brake torque, belt force, gravity, any damping) and find what applies the equivalent of about
   197 N s/m on native and nothing on the bridge. Candidates the review named: the belt's regularised efficiency
   switch (ruling 47; a back-driven belt at 0.95 efficiency dissipates), the belt's modal damping (D2: the
   transmission `damping_ratio` 0.01 gives D = 2 zeta sqrt(k m)), the brake model's engagement; the bridge's rigid
   belt applies none of these. If the archive does not separate them, build a unit experiment: a loaded shaft on a
   belt with a dead rail (supply 0) on native against the bridge, one term switched at a time through data. The
   result is a named mechanism with its number reproducing the 1.03 mm within 30 percent (then G7 and G9 are
   explained: a modelling difference, native richer than the bridge), or a finding stating what was ruled out.
   Rewrite the derivation in `test_phy_stubs.py`, the README and the milestones row to the real mechanism; the
   companion test's miss causes too.
2. G10's remainder. From both adapters' G10 archives read, for the stall and the slip: the encoder-mismatch
   detection tick, the commanded speed at it, the carriage speed at it, and the fault tick (the firmware's stop
   phase runs from detection to the fault at zero commanded velocity). Predict native's earlier events as the sum
   of the crossing shift X2d derived (2 and 5 ms from the belt's stretch) plus the stop-phase difference plus the
   carriage's coast during the stall (the mismatch grows at the commanded less the carriage speed), against the
   observed 8 and 20 ms. Within 30 percent: explained, written as numbers; otherwise the finding stays with what is
   now accounted for and what is not.
3. The driven mass: the tolerance uses 0.4064 kg and the G7 derivation 0.4074 kg with the difference unstated;
   both come from the build's `driven_mass` (BLD-041) through the run's metadata or the build result, not from a
   literal, and the 0.001 kg (the rope's end fitting, if that is it) is named. `tests/conftest.py`: the docstring
   says the cache is keyed by overrides; make the docstring say what the key is.
4. `docs/milestones.md` criterion 2 and the README restated from items 1 to 3: which of the six misses are
   explained, which are findings, in one line each with the number.

## Report

Under 250 words: commits; item 1's mechanism with its number against 1.03 mm (or what was ruled out); item 2's
accounting against 8 and 20 ms; item 3's values of record and their source; the state of the six misses; lints;
open issues.
