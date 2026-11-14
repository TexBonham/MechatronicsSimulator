# M3-X1f: the gantry's G12 and G15 as results of record on the native adapter

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1a` on `wip/05-x1a`, the
integration branch with everything merged: the carried rails with their mass on the beam (E7 to E7e), the
integrity fixes (I4, I4b, I5), the swing criterion (I3b). Run `tools/locked_build.sh sync` first. Call cap 130 with
the handoff rule (at 115 calls stop adding scope, commit what is green, report what is left). Read first:
`docs/briefs/round-05/m3-x1a.md` and `m3-x1d.md` (the original oracles and the scope rule: milestone 2 cases stay
on the bridge until brief X2), `test_cases/gantry/G12.yaml`, `G15.yaml`, `README.md`, `designs/gantry/` and
`docs/gantry-team-inputs.md` as they stand, `tests/acceptance/test_acc_stubs.py` (ACC-025, 026, 029, 030), round
5 rulings 52, 57, 60, 63 to 73 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`. No C++ should be needed;
filtered pytest and single cases through the CLI in the foreground; at the end ruff check, ruff format --check,
mypy, `python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final message "WIP
M3-X1f: G12 and G15 results of record", no attribution trailer, no push. Do not edit `docs/queue.md` or
`docs/cost-log.md`. In spec table cells write abs(x), never bare pipe characters.

## State since this brief was drafted (read rulings 73 to 80 first)

- The carried-load validity flag is member-wide and judged from the case's evaluation start (the driver passes
  `integrity.evaluate_from` to the engine): the largest nodal deflection relative to the member's static shape
  against the larger of the windowed quasi-static peak and the worst-case static carried deflection; flagged
  above 2.04. A flagged member's results are recorded not valid (the case result is the not-valid error), never
  pass or fail.
- The rails and the three belts now carry a placeholder damping ratio of 0.01 (ruling 78; anchored at the
  unloaded mode and the build-time mass, ruling 80, so the loaded gantry realises about 0.0054 and 0.0072 on the
  rails): each run prints stated, build-time and run ratios. These values are fixed: do not change them.
- Rope snap waits for the evaluation start.
- Strength reads the exact moment under a carried force and couple in both tiers; fatigue governs by damage.
- Ruling 77's reproduction condition replaces "20 substeps" below: run G15 at 10, 20 and 40 substeps (a
  40-substep run takes over ten minutes: run it in the foreground with the 600000 ms timeout, in parts if the
  CLI allows, or say that it could not be completed and give 10 and 20); a result of record needs every verdict
  to reproduce at the step sizes run, and the report states whether the dynamic ratio converges now that the
  rails and belts are damped; no agreement tolerance goes into a row.

## Ownership

`designs/gantry/**`, `test_cases/gantry/**`, `docs/gantry-team-inputs.md`, `tests/acceptance/test_acc_stubs.py`,
the unit and invariant tests that enumerate shipped gantry data, `docs/spec/acc.md` rows, `docs/spec/traceability.md`.
Not yours: all code under `core/` and `mechsim/` (report a defect with a reproducing command and stop on that
item; do not patch). Do not tune: no physics value, limit, band or window is changed to make a criterion pass. A
placeholder that fails is a finding, reported with the value, the limit and what the gantry team would have to
change.

## Items

1. ACC-025's first-ramp bound a-priori (left over from I4b): derive the allowance on the trolley's acceleration
   from the drive (the commanded ramp a plus the peak acceleration a second-order follower of a ramp in rate
   permits at the start of the ramp, in terms of its damping ratio; take the gantry's values from the data),
   assert the recorded hook acceleration within it, bound the swing by 2 a_max / g with the pendulum length to
   the payload's centre (0.37 m); the acc row states it.
2. Explain rail_y's settled stress: the check reads 9.91 MPa where pinned-beam hand statics gave 11.24 MPa.
   From the settled part of a G15 run: the two end shears against the hand load P; the effective position
   a = L F_y1 / (F_y0 + F_y1) against the trolley's coordinate; the `y_axis` joint's reaction moment about the
   bending axis (a carried couple); whether the node moments each side of the load are collinear through the
   ends. Say which term accounts for the difference and correct the hand oracle or report the defect.
3. G15 as a result of record (ruling 73's three conditions): (a) every verdict (each member's safety factor
   against the minimum, deflection against 1 cm, each fastening against its rating, tier 1 crossings) restated
   with deflection and stress scaled by 2.04 over the member's peak dynamic ratio, which is all that "no flag"
   bounds: a verdict that flips under that scaling is reported as not established; (b) the run repeated at 20
   substeps: each rail's peak deflection, dynamic ratio and flag against the 10-substep run; (c) the loaded first
   frequency of each rail from the recording against sqrt(k / (m_c + f m_beam)) for its supports. Report per
   member, measured against oracle: settled deflection, peak deflection in part (a) and through the stop, peak
   stress and safety factor from the evaluation start, the fastening forces against their ratings, crossings
   with times. ACC-029 and ACC-030 convert if the case passes and the three conditions hold; where a placeholder
   fails (the reviews saw rail_y near a safety factor of 1.4 against 1.5 and two fastenings 0.3 to 4 percent
   over a 1.333 kN placeholder limit) the stub stays a strict expected failure whose reason names the
   placeholder, the value and the limit.
4. G12: peaks against the limit and the three derived bounds, the lift on its limit through the stop, the settle
   criterion an expected failure with ruling 64's reason and the derived settle time; whether either rail is
   flagged during G12 and what that touches (G12 has no rail criteria).
5. Shipped data and baseline (SCO-009): G12 and G15 entries on native; the milestone 2 entries regenerated on
   the bridge with the current design hash: give the count of entries, and for every entry whose measured value
   changed the old value, the new value and the cause (the design changed: rail height, frames, section fields).
   The tests that enumerate shipped cases pass. G12 and G15 each run twice with identical recordings.
6. `docs/gantry-team-inputs.md`: every placeholder this round added, with its file, and a short list of the
   findings the gantry team must answer (swing damping and settle limit, rail sections and damping, fastening
   ratings, the safety factor result).

## Report

Under 350 words: commits, filtered test counts, item 1's allowance and measured acceleration, item 2's
explanation, item 3's per-member table with the scaled verdicts and the 20-substep comparison, item 4's table,
the baseline comparison, which ACC stubs converted and which stay expected failures with their reasons, defects
found in code you do not own, open issues.
