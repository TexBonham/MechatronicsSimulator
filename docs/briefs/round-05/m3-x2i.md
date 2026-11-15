# M3-X2i: G10's remainder from both archives; the E9 release artefact bounded and published (owner decision 1b)

Why: the owner retires the MuJoCo bridge after this brief (gate decision 1b, 2026-11-15). Two things stand between
the comparison of record and an explained one. G10: native raises `stall_fault` 8 ms and `slip_fault` 20 ms earlier
than the bridge where the x belt's stretch under the drive torque predicts 3.4 and 9.4 ms (rulings 90, 94); the
remainder is thought to sit in the firmware's stop phase (detection to the fault at zero commanded velocity) and in
the carriage's coast during the stall, but nobody has rebuilt the firmware's mismatch from the archives. E9: the
belt's efficiency loss acts as shaft damping frozen at the step's start rate (ruling 47's regularised switch,
`friction_slip_threshold`), 79 N m s/rad at rest, so a rotor released from rest is held back about 2 ms and loses
most of its first steps' momentum: a fidelity artefact with no bound and no approximation line (ruling 94).

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x2i` on `wip/05-x2i`, cut from
`round/05-milestone-3` at its head (no build: `tools/locked_build.sh sync` first; the C++ build for item 2 through
`tools/locked_build.sh core debug -R "<names>"`, one `debug-sanitize` run over the tests you touch at the end).
Call cap 100 with the handoff rule (at 90 calls stop adding scope, commit what is green, report what is left by
item). Read first: round 5 rulings 38, 39, 47, 90, 94, 96 in `docs/queue.md`; `core/physics/native/CLAUDE.md` and
`core/engine/CLAUDE.md` (where the efficiency switch and the friction regularisation live; find the code from the
contract, `scheme_decay_record` and the run-time mode lines as the pattern for an approximation line);
`tests/acceptance/test_phy_stubs.py` (the G10 miss causes as written by X2d and X2h), `test_cases/gantry/G10.yaml`
and `README.md`, the firmware's stall and slip detection and stop phase (`firmware/app/gantry/gantry.c` around lines
822 to 837, `gantry_config.h`), rows SIG-033, FW-0xx for the encoder mismatch, PHY-024, ENG-044 (the efficiency
row). Builds and runs in the FOREGROUND only through `tools/locked_build.sh` with a 600000 ms timeout, never
backgrounded; single cases through the CLI into the scratchpad
`/tmp/claude-1000/-home-tex-dev-MechatronicsSimulator/e12b8ab0-2af1-4dae-a612-eacb9e002661/scratchpad/x2i/`;
filtered pytest; no wide run. Lints at the end. COMMIT AFTER EACH ITEM; final message "WIP M3-X2i: G10 remainder,
E9 bounded", no attribution trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`. No conditional
(ternary) operators. Curly braces on every control flow. In spec table cells write abs(x), never bare pipe
characters.

## Ownership

Item 1: `tests/acceptance/test_phy_stubs.py` (the G10 miss causes), `test_cases/gantry/README.md`,
`docs/milestones.md` criterion 2 row, the PHY-024 row. Item 2: the transmission efficiency code in `core/engine/` or
`core/physics/native/` (wherever ruling 47's switch lives), its tests, the row that states it (ENG-044 or the
transmission tolerance row), `docs/spec/traceability.md`, the directory `CLAUDE.md`. Not yours: `firmware/**`, any
case criterion or window, the baselines (a baseline change you believe item 2 needs is a finding: list the entries).

## Items, in order

1. G10's remainder, accounted for or not. Run G10 once per adapter (`--set physics.adapter=mujoco` for the bridge)
   and, for the stall and the slip, read from each archive: the tick at which the firmware's encoder mismatch
   crosses its 2 mm threshold (rebuild it as the firmware computes it, from the recorded step counts and encoder
   counts, with the homing origin), the commanded speed and the carriage speed at that tick, the fault tick, and
   the stop phase between them. Predict native's lead as the crossing shift X2d derived (belt stretch: 2 and 5 ms)
   plus the stop-phase difference plus the coast term (the mismatch grows at the commanded less the carriage speed),
   against the observed 8 and 20 ms. Within 30 percent: explained, as numbers in the companion test's causes, the
   README and the milestones row (the misses stay in the list; criterion 2 is about tolerance). Otherwise the finding
   stays with what is accounted for and what is not, and the row names the next measurement.
2. E9 bounded and published. Derive, in the header of the code that applies the regularised efficiency: for a
   shaft released from rest, the frozen damping c0 = loss / threshold applied implicitly over a step h to a rotor
   that reaches rate w in that step removes an impulse up to c0 w h where the physics removes at most loss x h;
   the excess ratio is c0 w / loss = w / threshold. State the bound on the released rotor's lag (steps or time)
   as a function of threshold, step and the drive's acceleration, with the G7 numbers (79 N m s/rad, about 2 ms).
   Then publish: when in any step a transmission's frozen efficiency damping would remove more than the loss it
   stands for by a stated factor (propose 2; derive why), the adapter records one approximation line per
   transmission per run naming the transmission, the worst ratio and the time (the pattern of `scheme_decay_record`
   and the run-time mode lines), so the artefact is visible in the report. Do not change the switch's physics in
   this brief (the proper form, the loss evaluated at the step's end rate by one re-solve as the taut set does, is
   the milestone 4 item E9 proper: write it as the row's open item with the derivation). Tests: a unit test on a
   released shaft that the line appears with the predicted ratio and that a steadily turning shaft (rate far above
   the threshold) publishes nothing; G7 on native shows the line for the lift belt with its ratio.
3. Rows and docs: the efficiency row states the bound and the line; the PHY-024 row and the README's miss causes
   from item 1; `docs/milestones.md` criterion 2 restated; `mujoco-removal.md` unchanged unless item 1 changes what
   it says about the misses.

## Report

Under 250 words: commits; item 1's accounting (each term with its number against 8 and 20 ms, the verdict); item 2's
bound, the factor, G7's published ratio, the tests; rows changed; lints and the sanitize run; open issues.
