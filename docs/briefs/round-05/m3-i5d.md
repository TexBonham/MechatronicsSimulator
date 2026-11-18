# M3-I5d: closing the integrity re-review (I5b and I5c accepted on condition)

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1a` on `wip/05-x1a`, after brief
D2 (structural damping as data, the rope-snap wiring) is committed there. Call cap 70 with the handoff rule (at
60 calls stop adding scope, commit what is green, report what is left). Read first:
`docs/briefs/round-05/m3-i5b.md`, `m3-i5c.md`, the header of `mechsim/integrity/beam.py`, `docs/spec/int.md` rows
INT-005, 008, 023 to 026, 031 and the tolerance rows near lines 100 to 113, `core/bindings/src/bind_integrity.cpp`,
`mechsim/integrity/tier1.py`, round 5 rulings 71, 72, 75, 79 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Builds in the foreground through `tools/locked_build.sh`
(600000 ms timeout; no detached jobs; ctest filter in plain double quotes). Filtered pytest and
`core debug -R "integrity|Integrity"`; end with ONE `tools/locked_build.sh core debug-sanitize -R
"integrity|Integrity"` waited for in the foreground (clang-tidy clean), then ruff check, ruff format --check,
mypy, `python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final message "WIP
M3-I5d: integrity rows and binding boundary", no attribution trailer, no push. Do not edit `docs/queue.md` or
`docs/cost-log.md`. No conditional (ternary) expressions. In spec table cells write abs(x), never bare pipe
characters.

## Ownership

`mechsim/integrity/**`, `core/integrity/**`, `core/bindings/src/bind_integrity.cpp` (and the smallest change to
`bind_physics.cpp` or the binding module that item 3 needs), `mechsim/testing/core_driver.py` only where the
party's validity source is handed over (item 3), their tests, `docs/spec/int.md`, `docs/spec/traceability.md`, the
contracts.

## Items

1. R-18, the condition of acceptance: the Strength rule row (`docs/spec/int.md` about line 100) still states the
   intersection rule, U = abs(C), "tier 1 does not add the bound" and I5's oracles. Rewrite it to the code: the
   exact rule (lines A and B through the clean samples each side, evaluated at the foot point s; the peak the
   larger magnitude; the jump M(s+) - M(s-) = -C in the spring sense), where a bound remains and its value
   (clusters with a couple or three or more loads; a clamped root's stub), the oracles of record (pinned closed
   forms at 1e-9; clamped-clamped against the chain's statics, with the chain's gap from the continuum for a
   couple, w K / N, stated as a figure: K / 16 at sixteen segments can exceed the 2 percent row, so the row says
   what is and is not within 2 percent), both tiers the same. The row near line 105: under a carried load the
   reads follow the load and re-index when clusters merge, so a fatigue history can mix positions: an over-count
   only; members with one carried joint are unaffected. The row near line 113: rope snap waits for the evaluation
   start (the wiring landed with D2: check the row against the code).
2. Never silently low (invariant 7, ruling 79): where a couple's step puts a hogging extremum at s the rule can
   read low by at most 3 w l^2 of static own weight (more while the member rings: node inertia adds). The run
   says so: one line per member and run when the condition can occur (a carried couple on a member whose own
   weight moment hogs at the foot point), with the bound in N m and as a fraction of the read, in both tiers'
   outputs (the table's notes; the party's trace). Fix the synthetic test field whose step is +C where the
   header says -C.
3. Invariant 5: `core/bindings/src/bind_integrity.cpp` includes the native adapter's header (about line 20) so
   the party can read the adapter's live validity flag, and an out-of-range member index reads as valid (about
   line 102). The integrity package and its binding must not depend on an adapter: hand the party a neutral
   source (a callable or a small value interface declared in the integrity or kernel-neutral headers that the
   physics binding fills from the adapter), and make an out-of-range index a typed error. Tests: a party with a
   fake validity source (flag at a stated tick: the member's crossing after it is `integrity_not_valid`, before
   it counts); a mount follows its member in the party (not only at parsing); `tools/check_imports.py` and the
   include check that CI runs for MuJoCo headers show no adapter header under `core/integrity/` or its binding.
4. The native rigid-run test pins the gantry's current stub load as its expected note (a regression value):
   assert the note's structure and the member it names, and that its bound equals the formula for the recorded
   force, not a literal number.

## Report

Under 200 words: commits, filtered test counts and the sanitizer build result, the rewritten rows in one
sentence each, item 3's interface, each item done or not with its test, open issues.
