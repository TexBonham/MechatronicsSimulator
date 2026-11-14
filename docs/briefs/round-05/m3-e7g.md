# M3-E7g: the validity flag is member-wide (review of E7f at 707ec10: accepted as implementing rulings 74, 76)

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-e7` on `wip/05-e7` at 707ec10
(built and synced). Brief I5b runs concurrently in the integrity and scoring packages in another worktree. Call cap
70 with the handoff rule (at 60 calls stop adding scope, commit what is green, report what is left). Read first:
`docs/briefs/round-05/review-m3-e7f.md`, `m3-e7f.md`, the validity derivation in
`core/engine/include/mechsim/engine/member_loads.hpp`, `core/engine/src/multibody_provider.cpp` around lines
1820 to 1910, `core/engine/tests/test_carried_validity.cpp`, rows ENG-052, PHY-035, round 5 rulings 73, 74, 76,
77 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Builds in the foreground through
`tools/locked_build.sh` (600000 ms timeout; no detached jobs; ctest filter in plain double quotes; the gravity
gate test is slow: run it by name with a timeout of 600000 ms, not inside a wide filter). Iterate on
`core debug -R "carried validity"`; end with ONE `tools/locked_build.sh core debug-sanitize -R
"member|beam|carri|energy|native"` waited for in the foreground (clang-tidy clean), then ruff check, ruff format
--check, mypy, `python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final message
"WIP M3-E7g: member-wide validity", no attribution trailer, no push. Do not edit `docs/queue.md` or
`docs/cost-log.md`. In spec table cells write abs(x), never bare pipe characters.

## Ownership

`core/engine/**`, `core/physics/native/**` and `core/bindings/src/bind_physics.cpp` if an output changes,
`mechsim/physics/native/translate.py` and the run-end lines block of `mechsim/testing/core_driver.py` (wording
and the new figures only), `tests/unit/test_carried_members.py`, rows ENG-052 and PHY-035,
`docs/spec/traceability.md`, the contracts.

## Items

1. Major (ruling 77; ENG-052): the flag samples the deflection at the foot point but its floor is on the scale
   of the member's largest static carried deflection, so "no flag" lets the member's peak, away from a foot near
   a support, reach about 2.04 F / phi(s) (13 times the scale at 0.05 L), and a test pins a resonant ring of
   about 2.3 C_max W at mid-span as valid. Make the flag member-wide: the sample is the largest nodal deflection
   relative to the member's own static shape, max over nodes of abs(d_i - g_i); the quasi-static reference is
   the largest nodal quasi-static deflection from the rerouted loads through the chain's static compliance, max
   over nodes of abs(u_qs,i - g_i), its peak held over the window; the floor is unchanged in meaning (the largest
   static deflection anywhere on the member of the carried weight at its worst foot position over the travel).
   Derive in the header that the factor 2 still holds node by node for a step load and therefore for the maxima,
   and state what "no flag" now bounds: the member's largest deflection relative to its static shape is within
   2.04 times the larger of the windowed quasi-static peak and the worst-case static carried deflection. The
   test at about line 642 must now flag (a mid-span ring of 2.3 times the scale is not valid); re-derive its
   expected ratio. Publish per member, for the run-end line and the record: the peak ratio, the foot position at
   the peak, the floor, the window peak and the largest nodal relative deflection.
2. A carried joint whose foot cannot move (no travel) takes the floor at its own position, not the whole span.
3. Tests: assert (not print) that the window's length in time is equal across the two step sizes; the gate's
   title says what it tests; the excitation in the test near line 599 is at the derived 1.26 alpha (or say why 5
   alpha is needed); add a gravity-on force resonance (not moment-driven) asserting the header's shifted flag
   time against its derivation; the "flagged without resonance" list in ENG-052 includes a release from straight
   judged from time 0, and the single load reversal.
4. Invariant 2: remove the gantry narrative from the engine header (about lines 136 and 153): the derivation
   stands without naming a design. ENG-052 row: the missing space after "(ratio up to 4).", its sources cite
   rulings 73, 74, 76, 77; state that the floor's weight counts the carried subtree in the tree only (a load
   hanging on a rope reaches the member through the joint's reaction, which the quasi-static reference includes,
   but not through the floor: conservative).
5. On the gantry (read only), G15 from an evaluation start of 8 s (pass `carried_evaluation_start` the way your
   tree allows) at 10, 20 and 40 substeps: per rail the peak ratio, the foot position at the peak, the floor,
   the window peak, the largest nodal relative deflection and the flag. Say plainly whether the ratio converges
   with the step; if it does not, give the mode that rings (frequency, which element's damping is zero) and its
   amplitude at each step size.

## Report

Under 250 words: commits, filtered test counts and the sanitizer build result, the member-wide definition in
three lines, the re-derived test readings, G15's table at the three step sizes with the convergence statement,
open issues.
