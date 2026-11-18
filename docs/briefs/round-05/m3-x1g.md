# M3-X1g: fix round on X1f (review at 6f17849: accept after this; no tuning was found)

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1a` on `wip/05-x1a` at 6f17849
(built and synced). Call cap 90 with the handoff rule (at 80 calls stop adding scope, commit what is green,
report what is left). Read first: `docs/briefs/round-05/review-m3-x1f.md`, `m3-x1f.md` (the no-tuning rule holds
here too), `test_cases/gantry/G12.yaml`, `G15.yaml`, `tests/acceptance/test_acc_stubs.py` (ACC-025, 026, 029,
030), `docs/spec/acc.md` rows 58 to 63, `docs/gantry-team-inputs.md`, `mechsim/builder/run_damping.py`, round 5
rulings 60, 69, 73, 77 to 82 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Filtered pytest and single
cases through the CLI in the foreground; no C++; ruff check, ruff format --check, mypy,
`python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final message "WIP M3-X1g:
gantry results fix round", no attribution trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`.
No conditional (ternary) expressions. In spec table cells write abs(x), never bare pipe characters.

## Ownership

`test_cases/gantry/**`, `docs/gantry-team-inputs.md`, `tests/acceptance/test_acc_stubs.py`, `docs/spec/acc.md`,
`mechsim/builder/run_damping.py` with its tests and the BLD and PHY rows that describe its line (item 6), the
ENG-052 row in `docs/spec/eng.md` for the one statement of item 2, `docs/spec/traceability.md`. Not yours: other
code, `designs/gantry/**` values.

## Items

1. High (ACC-029, ACC-030): the stubs' docstrings say ruling 73's conditions (b) and (c) are checked by the
   test, but the fixture runs G15 at 10 substeps only, so a 10-substep pass would convert them. Make the test
   say and do the same thing: either it runs the case at 10 and 20 substeps (with the suite's long-run marker)
   and requires every verdict to reproduce and no member flagged at both, or its reason and assertions state
   that reproduction is checked by hand in the gate report and the stub cannot convert on the 10-substep run
   alone (an explicit assertion that fails while any member is flagged or any verdict differs). Expected
   failures carry `raises=AssertionError`. ACC-025's swing criteria are two-sided (ruling 60).
2. High (ENG-052): `G15.yaml` about line 60 and the report called it an engine defect that the carried mass
   omits the shackle and the payload on the rope. It is not: a body hung on a rope is not a rigid mass on the
   rail (the rope's own bounce is near 21 Hz; with it the coupled pair sits near 14.5 and 52 Hz, beyond one-way
   coupling). The one-way oracle is the rail with its tree-carried bodies only: 36.23 Hz against the measured
   36.25 Hz for rail_y. Correct the case header and the acceptance comments; add to the ENG-052 row one
   sentence: bodies hung on ropes from a carried body are not in the carried mass and reach the member through
   the rope's tension only (milestone 4 with two-way coupling).
3. rail_x's loaded frequency: measured 22.7 Hz; the one-mode estimate with tree-carried bodies gives 26.99 Hz;
   neither that nor the rigid estimate matches. Find out which is wrong: compute the chain's own first
   eigenfrequency for rail_x from the prepared native model with its tree-carried bodies at the foot position
   of the measurement (the model's mass and stiffness, not the one-mode formula; rail_y's beam and trolley ride
   the x carriage) and compare with 22.7 Hz. If they agree within 2 percent the estimate was crude: state the
   right figure in the case header. If they do not, stop on this item and report the numbers and a reproducing
   command: it is a defect to chase before the gate.
4. Medium (ACC-029, ACC-030; T4): `docs/spec/acc.md` about line 62 moved the peak's upper oracle from 1.01 to
   2.04 times static without a derivation, and 2.04 times static is not what "no flag" bounds (its base is the
   larger of the windowed quasi-static peak under the actual loads, the lift's landing included, and the
   worst-case static carried deflection). State the bound as the engine defines it, or restore the earlier band
   with its reason; say which and why in the row.
5. Medium (ACC-025): the new expected failure for the hook's recorded acceleration has a true but only asserted
   reason. Put the derivation in the test's docstring and the row: one 12.5 micrometre step into the belt gives
   q omega^2 = 0.51 m/s^2 in x (measured 0.56) and 1.36 in y (measured 3.32, where the per-tick step-count beat
   crosses the 52.5 Hz belt mode at 8.74 to 8.76 s); check those figures from the data yourself; say that no
   gantry value in the data today converts it and what would (the drive's step size or microstepping, belt
   damping); `acc.md` about line 58 says plainly that this allowance fails.
6. The run-damping line is on the wrong side for members: it counts a rope-hung payload in the mass and prints a
   run ratio of 0.0037 for rail_y where the engine realises 0.0072 (the payload is not in the carried mass). The
   line counts what the engine counts: tree-carried bodies only, for members and for transmissions (a payload
   on a rope is a pendulum to a belt's 50 Hz mode, not a rigid mass); the line says so. Tests and rows.
7. G12 states `integrity.evaluate_from: 8 s` with G15's timeline and derivation (the release from straight is
   not a load case; this is the correction G15 already carries, not tuning). Re-run G12: report the tier 1
   crossings that remain, rail_y's flag (it was flagged from 8.31 s), and what the case's result is. Ruling 82:
   a not-valid flag on a member makes not valid only the criteria and crossings that read that member; a case
   with no criterion on it is not turned into an error by the flag alone, and the run notes carry the line. If
   the code does otherwise, report it with the reproduction (do not patch scoring). Update G12's baseline
   entries.
8. `docs/gantry-team-inputs.md`: the required static factor: the "1.17 times" figure ignores ruling 73 (a):
   scaled by 2.04 over the peak ratio it is about 2.1 times, and with a ratio of 1 any rail needs a static
   factor of 3.06 (1.5 times 2.04) under this validity bound: state both; the rail section placeholder is
   inconsistent and unstated (a solid 40 by 40 mm section has I = 2.13e-7 m^4 against the 2e-8 stated; the two
   rails' calibrations imply E of 3.2 and 12.8 GPa): say so plainly as an inconsistency the team must resolve;
   rows 19 and 27 are stale: bring them to the current data; add a row for the step train exciting the belt
   mode. `G15.yaml` about line 42: the quoted samples extrapolate to 9.04, the 8.81 comes from 0.156 and
   0.219 m: fix the comment.
9. SCO-009: the bridge's G6 `lift_stopped` moved by exactly one tick (0.1 mm at 0.1 m/s). Confirm the cause:
   re-run G6 on the bridge with the rope's end-fitting mass set to zero through `--set` (and, if that does not
   restore it, with the rail height at its old value) and say which change moved it; give the baseline's
   changed entries their cause in the report.

## Report

Under 300 words: commits, filtered test counts, item 3's eigenfrequency against 22.7 Hz, item 7's G12 result,
item 9's cause, each other item done or not, defects found in code you do not own with their reproduction.
