# M3-X1d: finish the gantry's G12 and G15 on the native adapter

Fresh agent (X1a stopped at its cap). Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1a` on
`wip/05-x1a`, the integration branch: X1a's gantry work plus E6 (the lift no longer jumps on a stop), I3 and I3b
(integrity on rigid builds, node indexing with run-time bodies, the swing angle quantity SCO-030, runner wiring)
and E7 (a flexible member carries named joints: load-path coupling). Call cap 150 with the handoff rule (at 135
calls stop adding scope, commit what is green, report what is left). Read first: `docs/briefs/round-05/m3-x1a.md`
(the original brief: its oracles and scope rule still hold), `docs/briefs/round-05/m3-e7.md` and the contracts E7
changed (`core/engine/CLAUDE.md`, `mechsim/builder/CLAUDE.md`, `mechsim/design/CLAUDE.md`: the carried-joints
field and what is rerouted), `mechsim/scoring/CLAUDE.md` (swing quantity, settled value, oscillation statistics),
`test_cases/gantry/G12.yaml` and `G15.yaml` as X1a left them, round 5 rulings 52, 57 to 66 in the main repo's
`/home/tex/dev/MechatronicsSimulator/docs/queue.md` (the worktree's copy is older). Builds in the foreground
through `tools/locked_build.sh` (600000 ms timeout; no detached jobs; a ctest filter is passed in plain double
quotes, `-R "name|other"`); the worktree is built and synced. Iterate with filtered pytest and single cases through
the CLI; you should not need C++; at the end ruff check, ruff format --check, mypy,
`python3 tools/check_imports.py`; no full pytest run of your own (it takes 37 minutes; the coordinator runs it at
landing). Commit as you go; final message "WIP M3-X1d: gantry G12 and G15 on native", no attribution trailer, no
push. Do not edit `docs/queue.md` or `docs/cost-log.md`.

## Ownership

`designs/gantry/**`, `test_cases/gantry/**`, `docs/gantry-team-inputs.md`, `mechsim/builder/` (the member frame
and its warning, item 1; the rope payload's BLD row), `mechsim/testing/core_driver.py` (item 3's one-line
follow-up only), `tests/acceptance/test_acc_stubs.py`, the unit and invariant tests that enumerate shipped
gantry data, `docs/spec/{acc,bld,sco}.md` rows you change, `docs/spec/traceability.md`, the contracts. Not yours:
`core/**`, `mechsim/physics/`, `mechsim/integrity/`, `mechsim/scoring/` (report a defect with a reproducing
command; do not patch). Invariant 2: nothing gantry-specific in code.

## Items

1. Rail frames and the load path (rulings 63, 65): the lumped chain bends in one plane, about the member frame's
   z. State each rail's member frame in the design so vertical bending is modelled (frame y vertical), with the
   section's axes and fibre distances matching that frame; the builder warns when gravity lies entirely outside a
   horizontal member's bending plane, and the run's approximation lines say that bending in the other plane is
   rigid (check the line E7 added; add the builder warning with a BLD row and tests on a small design). Turn on
   the carried joints: `rail_x` carries the x carriage's joint, `rail_y` the trolley's (and the lift's parent chain
   as the data requires). Check on a static run before anything else: each rail's node-pose deflection under its
   own weight against w L^4 / (384 E I) (clamped-clamped) and 5 w L^4 / (384 E I) (pinned-roller), and with the
   payload hanging at mid-span against the added P L^3 / (192 E I) and P L^3 / (48 E I), within the 2 percent
   row; the member end loads sum to the weights carried.
2. Shipped data lists: G12 and G15 in the suite, the baseline and the three tests that enumerate shipped cases
   (`tests/invariant/test_schema_test_case.py`, `tests/unit/test_baseline_schema.py`,
   `tests/unit/test_test_case_schema.py`), on native; milestone 2 entries on the bridge unchanged. Run one
   milestone 2 case on the bridge with the integrity file present to show it is unaffected.
3. G12 (ACC-025, 026): the case's own criteria use the swing quantity (SCO-030) for the payload about the hook:
   peak after the fast move and after the stop against the limit, and against the derived bounds of the X1a
   brief as criteria or as assertions in the acceptance test (X1a measured the first ramp's y swing at 0.010510
   against 2 a / g = 0.010381 with a swing already present at the window start: derive the bound including the
   initial swing and say whether the 1 percent excess is that, the scheme, or a defect); the lift stays on its
   limit through the stop (E6); the settle criterion is present with its limit as an open parameter and ACC-026
   stays a strict expected failure with ruling 64's finding in its reason (derived settle time against the
   limit; do not tune the physics, the band or the limit); ACC-025 converts if the peaks pass. Follow-ups from
   the I3b review: `core_driver._sensed_payload` calls the shared `case_payload_body`; the SCO-030 row states
   that the peak bound needs a recorded turning point after release.
4. G15 (ACC-029, 030): the case starts settled (a settle window before the criteria window, its length derived
   from the rails' node damping; say what it is), carries the design load in its data (ruling 52), tier 1 on,
   halt off. Report per member, measured against oracle: minimum safety factor (static bending stress M c / I at
   the carriage position times the dynamic factor you derive for the acceleration and the stop), the dynamic
   peak and settled deflection against the item 1 closed forms, the fastening forces at the stop against their
   ratings (X1a saw 3.0 kN and 2.7 kN against 1.33 kN while the lift was jumping: say what they are now), tier 1
   crossings, and the exported load case. ACC-029 and ACC-030 convert if the case passes; if a member or a
   fastening fails on the placeholder numbers, do not tune: report it as a finding against the placeholder
   (which value, which limit, what the gantry team would have to change).
5. Determinism and rows: G12 and G15 each run twice with identical recordings (X1a rewrote this test and did not
   rerun it); the BLD row for the payload on its rope on native and the stand-in on the bridge, with a test of
   the bridge's approximation line; every placeholder in `docs/gantry-team-inputs.md`.

## Report

Under 300 words: commits, filtered test counts, item 1's four static checks, G12's peaks against limit and
bounds and the derived settle time, G15's per-member table, the fastening forces, which ACC stubs converted and
which stay expected failures with the reason, findings against files you do not own, open issues.
