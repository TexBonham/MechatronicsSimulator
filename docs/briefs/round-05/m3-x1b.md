# M3-X1b: structural acceptance on the native adapter (ACC-037, 039 to 043, FAC-013), node-pose deflection

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1b` on `wip/05-x1b`, cut from
the round branch after I2 (tier 1 integrity party) has landed. Brief X1a (the gantry's design, G12, G15) runs
after this one starts and owns `designs/gantry/` and `test_cases/gantry/`: do not edit them. Call cap 140 with the
handoff rule (at 125 calls stop adding scope, commit what is green, report what is left). Read first: root
`CLAUDE.md`, `mechsim/integrity/CLAUDE.md`, `core/integrity/CLAUDE.md`, `mechsim/testing/CLAUDE.md`,
`mechsim/scoring/CLAUDE.md`, `mechsim/design/CLAUDE.md`, `mechsim/physics/native/CLAUDE.md`, rows ACC-037, 039 to
043 and the acceptance tolerance rows (`docs/spec/acc.md`), INT-006, INT-023 to 026, INT-031, FAC-013, DES-033,
ADR 0016, round 5 rulings 21, 24, 25, 28, 31, 32, 34, 51, 53 (`docs/queue.md`). Builds in the foreground through
`tools/locked_build.sh` (600000 ms timeout; build and ctest as separate calls; never wait on a background task
without checking `ps`). Iterate with `core debug -R "'integrity|Integrity'"` and filtered pytest on the files you
touch; at the end ruff check, ruff format --check, mypy and `python3 tools/check_imports.py` only: no full
sanitizer run and no full pytest run of your own (ruling 56: the coordinator runs the one full verification at
landing). Commit as you go; final message "WIP M3-X1b: structural acceptance on native", no attribution trailer,
no push.

## Ownership

`mechsim/integrity/**`, `core/integrity/**`, `mechsim/design/schema.py` and `schemas/design.json` (item 2 only),
`mechsim/builder/` where the section reaches the neutral member (item 2 only), `mechsim/scoring/` and
`mechsim/facade/` (item 3 only), `designs/pendulum/**`, `test_cases/pendulum/**`, new designs under `designs/`
that are reference fixtures (item 4), `tests/acceptance/**`, `tests/unit/test_integrity*.py`,
`tests/reference/` files for the checks, `docs/spec/{acc,int,des,sco,fac}.md` rows you change,
`docs/spec/traceability.md`, the contracts. Not yours: `core/engine/`, `core/physics/`, `core/kernel/`,
`mechsim/physics/`, `mechsim/testing/core_driver.py` (ask in the report if a change there is needed),
`designs/gantry/`, `test_cases/gantry/`.

## Items

1. Node-pose deflection (ruling 28, INT-006, INT-025): the deflection of a lumped member is the largest
   perpendicular distance of its node bodies from the chord through its two end bodies, taken from the recorded or
   published node poses, in the member frame (ruling 24), replacing the static end-load formula in both the tier 2
   check and the tier 1 party (the party reads only kernel physics outputs: node body poses are there). Oracles,
   derive in the test header: cantilever under a tip load P L^3 / (3 E I) and under a uniformly distributed load
   w L^4 / (8 E I) (the end-load formula read 17 percent low here: assert the node-pose value within the lumped
   chain's 2 percent at eight segments); pinned-pinned centre load P L^3 / (48 E I); clamped-clamped centre load
   P L^3 / (192 E I) at sixteen segments. Note that for a cantilever the chord through both ends is not the
   undeformed axis: state which reference line each support pair uses (the clamped root's tangent for a member
   with a clamped end, the chord for pinned ends) and make the row say it. A member built rigid (the bridge) has
   no nodes: the check reports it unevaluated with a line, never zero.
2. Explicit two-axis section (ruling 25, DES-033): the design's `section` accepts `second_moment_y`,
   `second_moment_z`, `fibre_y`, `fibre_z` beside `area` (the single `second_moment` stays as the symmetric
   shorthand: both axes equal, fibre distances then required or derived from a `shape`); the values reach the
   neutral member unchanged; schema regenerated, example and validation tests for every combination and each
   rejection.
3. Carried from the I2 re-review: (a) `RunReport.notes` reaches `CaseScore` and the printed report and the
   facade's run result, so a halt line is readable without opening the archive (INT-026, invariant 7); (b) a test
   through `CoreDriver.run` showing the `rope_snap` gate applied and its line in the report, for the entry
   present and absent (INT-031); (c) `gantry_run(..., None)` in `tests/unit/test_integrity_tier1.py` must not
   pick up a shipped `designs/gantry/integrity.yaml` (X1a adds that file): make "without tier 1" explicit.
4. Acceptance references through the full stack on the native adapter (data-only designs under `designs/`, cases
   under `test_cases/`, run through the CLI or `CoreDriver` exactly as a user would; convert the strict-xfail
   stubs in `tests/acceptance/test_acc_stubs.py`):
   - ACC-037 rope pendulum: the pendulum design gains a rope variant (a second design file or an option in data,
     no code); period within 0.1 percent of the chain's closed form T_N with the rope's added mass and static
     stretch (ADR 0016) and amplitude within 2 percent over 20 periods, at the 50 microsecond step the case states
     (ruling 21); if the run time through the full stack is over five minutes, mark the test with the reference
     marker the suite uses for long cases and say so.
   - ACC-039 settle time of a step response: a mass on a spring-damper joint released from an offset; settle time
     to 2 percent against the closed form for the stated damping ratio (derive it for the underdamped envelope,
     ln(50) / (zeta omega_n) corrected for the scheme's decay at the stated step, ruling 45).
   - ACC-040 to 043: cantilever and simply supported beam designs: static deflection and first natural frequency
     against the closed forms within the 2 percent of rulings 31 and 32, using item 1's deflection and the
     recorded node pose for the frequency (count zero crossings over at least ten periods, at a step small enough
     that the frequency ratio atan(x) / x is within 0.1 percent: state x).
   Propose the tolerance rows for ACC-037 to 043 in `docs/spec/acc.md` with their derivations (owner confirms at
   the gate).
5. FAC-013 conformance stub (`tests/acceptance/test_facade_conformance_stubs.py`): convert on the native adapter
   with a flexible member: the integrity table through the facade equals the direct service result.

## Report

Under 300 words: commits, filtered ctest and pytest counts, each oracle's measured value against its closed form,
every schema or contract field added, each item done or not with its test, anything needed in files you do not
own, open issues.
