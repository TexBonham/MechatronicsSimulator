# M3-I5b: fix round on I5 (review of a0d37b9): the exact moment under a carried force and couple

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1a` on `wip/05-x1a`, the
integration branch, after `wip/05-e7` (E7e, E7f) has been merged in by the coordinator: the engine's validity flag
has its final definition and the native adapter takes an evaluation-start key. Run `tools/locked_build.sh sync`
first. Call cap 130 with the handoff rule (at 115 calls stop adding scope, commit what is green, report what is
left). Read first: `docs/briefs/round-05/review-m3-i5.md`, `m3-i5.md`, `m3-i4b.md`, `m3-e7f.md`,
`mechsim/integrity/CLAUDE.md`, `core/integrity/CLAUDE.md`, `mechsim/scoring/CLAUDE.md`, `mechsim/record/CLAUDE.md`,
rows INT-005, 008, 023 to 026, 031, SCO-029, FAC-013, TST-044, round 5 rulings 69 to 75 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Builds in the foreground through `tools/locked_build.sh`
(600000 ms timeout; no detached jobs; ctest filter in plain double quotes). Iterate with filtered pytest and
`core debug -R "integrity|Integrity"`; end with ONE `tools/locked_build.sh core debug-sanitize -R
"integrity|Integrity|native"` waited for in the foreground (clang-tidy clean), then ruff check, ruff format
--check, mypy, `python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final message
"WIP M3-I5b: exact carried moment, not-valid fixes", no attribution trailer, no push. Do not edit `docs/queue.md`
or `docs/cost-log.md`. In spec table cells write abs(x), never bare pipe characters. No conditional (ternary)
expressions in Python or C++ (project rule; the review found two: `mechsim/integrity/table.py` about line 153,
`mechsim/scoring/score.py` about line 360).

## Ownership

`mechsim/integrity/**`, `core/integrity/**`, `core/bindings/src/bind_integrity.cpp`, `mechsim/scoring/**`,
`mechsim/record/validity.py` and the record view protocol, `mechsim/testing/core_driver.py` and `runner.py` where
the validity record, the crossings and the evaluation start are handled, `mechsim/facade/` where the table state
is carried, `core/physics/native/**` for item 5 only, schemas regenerated, their tests,
`docs/spec/{int,sco,rec,tst,fac,phy}.md` rows, `docs/spec/traceability.md`, the contracts. Not yours:
`core/engine/`, `core/kernel/`, `designs/gantry/`, `test_cases/gantry/`, `tests/acceptance/test_acc_stubs.py`.

## Items

1. The exact moment under a carried force and couple, both tiers (ruling 75; INT-005, INT-023). The review's
   derivation, to verify and put in the header: spring j joins bodies j-1 and j at x_j; with the load's lever
   shares on bodies k and k+1, every spring at or below x_k and at or above x_(k+2) is exact, and only the spring
   at x_(k+1) is off, by w_(k+1) C. Line A through the samples at k-1 and k, and line B through k+2 and k+3,
   evaluated at the foot point s, give the moment just left and just right of s: A(s) = M(s-), B(s) = M(s+);
   the peak is the larger magnitude. No bound is needed when s is known. Both tiers take s from the carried
   body's pose projected on the member axis at the step's start (the previous row in a recording); tier 1 gets
   the carried body indices in the limits table. Force-only clusters (neighbouring segments, opposite signs) are
   exact by the chord from A(s_1) to B(s_2). A stated bound remains only for: couples inside a cluster, fewer
   than two clean samples on a side (say what is used near each end condition), and own weight within the
   loaded span (B over-reads by up to 3 w l^2: state it, conservative). Remove the abs(C) bound where the exact
   rule applies. Oracles on the real chain through the engine (a small data-only design with `carries` and an
   offset load, not a synthetic moment table): a force with a couple at four positions on a pinned member,
   M(s-) and M(s+) against P a b / L plus the couple's C b / L and C a / L shares within 1e-9 of the chain
   statics; the 400 N push of I5's test reads 8.4 where it read 15.875; a clamped-clamped member; the two tiers
   within 1e-9; a member carrying nothing bit-identical.
2. Not valid must not halt or fail (SCO-029, INT-026): under `halt_on_crossing` a crossing of a member that is
   flagged at that time does not halt and does not score a plain fail; it is the not-valid event and the case
   result is the not-valid error with its reason; a flagged member's mount crossings follow the member (the
   table already marks those mounts); `validity` becomes part of the `RecordView` protocol (a view without it is
   an error, not "valid"); the facade's `state` and `not_valid_from` tested; the validity writer refuses
   non-finite values its reader refuses.
3. Fatigue (INT-008): the governing section is the one with the largest damage, not the largest peak stress, and
   a section with no candidate stress history is not read as zero (which fabricates cycles): it is not
   evaluated, with a line. Oracle: a member where the peak-stress section and the largest-damage section differ
   (a small-amplitude high-cycle section against a single large excursion elsewhere): the check reports the
   damage section.
4. `tests/unit/test_integrity_rigid_runs.py` about line 74: the assertion that became a vacuous loop asserts
   again what the row requires (one not-evaluated line per rigid member and no others).
5. Rope snap and the evaluation start (INT-031): a rope must not snap during the release transient before the
   case's evaluation start (the doubled tension of a release from slack is not a load case). Gate it where the
   snap is decided, with the adapter's evaluation-start key; before the start an exceedance is an event of the
   before-evaluation kind; row and tests.
6. End to end with an engine-set flag (the test I5 monkeypatched): a two-member rig through `CoreDriver` to an
   archive whose carrying member the engine flags under the final definition (matched loaded frequencies under a
   sustained periodic push; take the rig from the engine's own validity tests): its rows and crossings not
   valid from the flag time, the other member unaffected, the facade table carrying the state, and a run without
   a flag unchanged bit for bit.

## Report

Under 300 words: commits, filtered test counts and the sanitizer build result, item 1's oracles (measured against
closed form, both tiers) and where a bound remains, each other item done or not with its test, fields added or
removed, open issues.
