# M3-I5c: what I5b left (fatigue by damage, two tests, own weight near a clamp)

Fresh agent (I5b stopped at its cap). Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1a` on
`wip/05-x1a` at f4977ea (built and synced). Brief E7g runs concurrently in `core/engine/` in another worktree: do
not edit the engine or the adapter. Call cap 70 with the handoff rule (at 60 calls stop adding scope, commit what
is green, report what is left). Read first: `docs/briefs/round-05/m3-i5b.md` (items 3 and 4 are yours now),
`review-m3-i5.md`, `mechsim/integrity/CLAUDE.md`, the header of `mechsim/integrity/beam.py`, rows INT-005, 008,
round 5 rulings 71, 72, 75 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Filtered pytest in the
foreground; C++ only if item 4 needs tier 1 (then end with ONE `tools/locked_build.sh core debug-sanitize -R
"integrity|Integrity"`, clang-tidy clean); ruff check, ruff format --check, mypy,
`python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final message "WIP M3-I5c:
fatigue by damage, test repairs", no attribution trailer, no push. Do not edit `docs/queue.md` or
`docs/cost-log.md`. No conditional (ternary) expressions. In spec table cells write abs(x), never bare pipe
characters.

## Ownership

`mechsim/integrity/**`, `core/integrity/**`, `tests/unit/test_integrity*.py`, `tests/reference/test_integrity*.py`,
`docs/spec/int.md`, `docs/spec/traceability.md`, the integrity contracts.

## Items

1. First, the state of the tests I5b touched and did not re-run: run `tests/unit/test_integrity.py`,
   `tests/unit/test_integrity_tier1.py`, `tests/unit/test_integrity_rigid_runs.py` and the rest of
   `tests/unit/test_integrity*.py`, `tests/reference/test_integrity*.py`, `tests/unit/test_carried_members.py`.
   `test_the_same_case_on_the_native_adapter_evaluates_every_member` was failing: find why (the merged engine's
   validity flag, the evaluation start, or the carried-moment rule) and fix the cause; if the cause is outside
   your ownership, stop on it and report the reproduction.
2. Fatigue (INT-008; I5b item 3): the governing section is the one with the largest damage, not the largest
   peak stress; a section with no candidate stress history is not evaluated, with a line, never read as zero
   (which fabricates cycles). Oracle: a member where the peak-stress section and the largest-damage section
   differ (a small-amplitude high-cycle section against a single large excursion elsewhere): the check reports
   the damage section and its damage equals the hand count n / N from the S-N data within 1e-9.
3. `tests/unit/test_integrity_rigid_runs.py` about line 74 (I5b item 4): the assertion that became a vacuous
   loop asserts again what the row requires: exactly one not-evaluated line per rigid member and no others.
4. Own weight near a clamp: I5b reports that under own weight the extrapolated line can read up to 3 w l^2 low
   where the moment hogs (near a clamped end), so the carried-load rule is not conservative everywhere. Make it
   never lower than the plain node-spring and end-moment samples (the peak is the larger magnitude of the rule's
   value and the samples in the loaded span and at the ends), in both tiers, and show with a clamped-clamped
   carrying member under own weight plus a carried load at four positions that the read is at or above the
   chain's exact peak and within the stated over-read; the INT row states the remaining over-read.
5. I5b moved an existing "not flagged" test from a 40 mm to a 150 mm carried cube because the merged engine's
   flag now flags the smaller rig (ratio 4.3): say in the report what the flag's base and floor are in the 40 mm
   rig and whether the flag is right there (a light cube on a stiff bar released from straight, judged from
   which time?), so the review can decide; do not change the engine.

## Report

Under 200 words: commits, filtered test counts (every file listed in item 1), the cause of the failing test, the
fatigue oracle, item 4's readings against the exact peak, item 5's numbers, open issues.
