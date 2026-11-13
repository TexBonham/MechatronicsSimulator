# M3-X1a: the gantry on the native adapter with flexible rails and the payload on its rope; G12 and G15

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1a` on `wip/05-x1a`, cut from
the round branch after N4 (adapter follow-up: substep drive interpolation, node pose series, run-time mode lines)
and X1b (node-pose deflection, two-axis section fields, notes in the score) have landed. Call cap 150 with the
handoff rule (at 135 calls stop adding scope, commit what is green, report what is left). Read first: root
`CLAUDE.md`, `designs/gantry/README.md`, `test_cases/gantry/README.md`, `docs/gantry-team-inputs.md`,
`mechsim/builder/CLAUDE.md` (structure, ropes, BLD-032), `mechsim/testing/CLAUDE.md` (run configuration, tier 1
wiring, end reasons), `mechsim/integrity/CLAUDE.md`, `mechsim/scoring/CLAUDE.md` (structural criteria),
`mechsim/physics/native/CLAUDE.md`, rows ACC-025, 026, 029, 030 and the gantry rows of `docs/spec/acc.md`,
BLD-032, INT-023 to 026, TST-040, TST-041, ADR 0016, round 5 rulings 2, 4, 21, 27 to 32, 34, 41, 45, 49 to 53
(`docs/queue.md`), the plan's assumption 6 (`docs/briefs/round-05/plan.md`: gantry structural placeholders).
Builds in the foreground through `tools/locked_build.sh` (600000 ms timeout; never wait on a background task
without checking `ps`; no detached jobs). Iterate with filtered pytest and single cases through the CLI; at the end
ruff check, ruff format --check, mypy, `python3 tools/check_imports.py` only: no full sanitizer run and no full
pytest run of your own (ruling 56). Commit as you go; final message "WIP M3-X1a: gantry G12 and G15 on native", no
attribution trailer, no push.

## Ownership

`designs/gantry/**`, `test_cases/gantry/**`, `docs/gantry-team-inputs.md`, `mechsim/builder/` (the case payload on
the rope, BLD-032), `mechsim/testing/` (run-configuration field for substeps, item 3), `mechsim/scoring/` only if
a swing or settle criterion is missing, `tests/acceptance/test_acc_stubs.py` (the ACC-025, 026, 029, 030 stubs),
the unit tests of what you touch, `docs/spec/{acc,bld,tst,sco}.md` rows you change, `docs/spec/traceability.md`,
the contracts. Not yours: `core/**`, `mechsim/physics/`, `mechsim/integrity/` (report a defect, do not patch it).
Invariant 2: nothing gantry-specific in code; every number lives in the design or the case, with its unit, and
every placeholder is listed in `docs/gantry-team-inputs.md` with the file it lives in.

## Scope rule

Milestone 2's cases (G1 to G11, G13, G14) keep passing on the bridge unchanged and stay the default until brief
X2 switches the suite (plan assumption 9). G12 and G15 state `physics.adapter: native` in the case. The design
file is shared: it must build on both adapters (the bridge builds `rigid_members=True` and keeps the
spherical-joint stand-in for the payload with its approximation line).

## Items

1. Design: `rail_x` clamped-clamped at sixteen segments with its calibration, `rail_y` pinned-roller at eight
   (rulings 31, 32); remove the stale "no lumped beam yet" comment; the payload hangs on `payload_rope` on native
   (BLD-032: the case payload is the rope's load end, placed at run time); the bridge keeps the stand-in. Show G3
   on native with the rope payload against the values of record without it (final_x 0.40002497 m, final_y
   0.25002500 m, ready_after_homing 5.245 s) and explain any move.
2. `designs/gantry/integrity.yaml` (ruling 27): strength, deflection, buckling and fastener at tier 1,
   `rope_snap`, and the tier 2 checks the design can support; fastener ratings as flagged placeholders; minimum
   safety factor 1.5 and deflection target 1 cm (plan assumption 6). The milestone 2 cases on the bridge must be
   unaffected (tier 1 is native only; check the bridge's line).
3. Step size (rulings 21, 45, 49): backward Euler keeps (1 + x^2)^(-pi / atan x) of a swing per period, x = omega
   h. For the 0.3 m rope, omega = sqrt(g / L) = 5.72 rad/s: at 1 ms the scheme removes about 1.8 percent of the
   amplitude per period, which would decide G12's settle time by itself. G12 and G15 therefore run the physics at
   a step where the numerical loss per period is at most a tenth of the model's physical loss per period (derive
   both, state them in the case header and assert the approximation line shows the mode resolved); the control
   tick stays 1 kHz. Add the run-configuration field that sets the adapter's substeps (`physics.substeps`,
   integer >= 1, default 1, schema regenerated, TST row) unless unequal physics and control rates already cover it
   (say which you used and why).
4. G12 (ACC-025, 026): two parts, each a case or one case with two windows: (a) a fast move at the gantry's limits
   (0.05 m/s^2, 0.1 m/s) to a target; (b) an E-stop at full speed. Criteria: peak swing angle of the payload about
   the hook, and settle time to the settle band, both as open parameters owed by the gantry team (defaults 0.1 rad
   and 5 s, plan assumption 6). Oracles, in the header: accelerating at a the steady swing is a / g and the peak
   at most 2 a / g = 0.0102 rad; a stop from v within a time short against the period gives at most
   v / sqrt(g L) = 0.0583 rad; a stop at the deceleration limit gives at most 2 a / g again. Assert the measured
   peaks against these derived bounds as well as against the limit. Settle time: derive the model's decay per
   period from the rope's and the attachment's stated damping and give the expected settle time; if the model
   with its placeholder damping cannot settle inside the default limit, do not tune the physics or the limit to
   pass: report it as a finding with the numbers (the limit and the damping are both owed by the gantry team).
   If the policy "lowering waits for settling" exists in the firmware, exercise it; if not, record it as not set.
5. G15 (ACC-029, 030): (a) maximum acceleration with the design load (rated load plus 15 percent: the case's
   payload carries it, ruling 52; the rated load is a flagged placeholder); (b) an E-stop with a swinging load.
   Tier 1 on, `integrity.halt_on_crossing` false. Criteria: every member's minimum safety factor at or above the
   minimum, every member's node-pose deflection under 1 cm, no tier 1 crossing event; the peak member end loads
   exported as the load case (the I1 export) and the file checked. Oracles in the header: static mid-span
   deflection of each rail under the design load at mid-travel (P L^3 / (192 E I) clamped-clamped,
   P L^3 / (48 E I) pinned) and the dynamic value within twice that (suddenly applied load); static bending
   stress M c / I against the measured peak. Report each member's measured safety factor and deflection beside
   its oracle.
6. Baseline: add G12 and G15 entries to `test_cases/gantry/baseline.yaml` and the suite on native; convert the
   ACC-025, 026, 029, 030 stubs; both cases run twice bit-identical (invariant 4).

## Report

Under 300 words: commits, the filtered test counts, G3 on native with the rope against the values of record, the
step used and both loss figures, G12's peaks against the three bounds and the settle time against the derived
value, G15's safety factors and deflections against their oracles, every placeholder added, every schema field
added, findings against files you do not own, open issues.
