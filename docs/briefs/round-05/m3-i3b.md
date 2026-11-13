# M3-I3b: fix round on I3 (review of eef3732): the swing angle reads displacements, runner wiring

Fresh agent (the I3 transcript is at 159 calls). Agent: module-writer (Opus). Worktree
`../MechatronicsSimulator-05-i3` on `wip/05-i3` at eef3732. Brief E6 runs concurrently in `core/engine/` and
`core/physics/native/`: do not edit those. Call cap 70 with the handoff rule (at 60 calls stop adding scope,
commit what is green, report what is left). Read first: `docs/briefs/round-05/m3-i3.md` (the original brief),
`mechsim/scoring/CLAUDE.md`, `mechsim/record/CLAUDE.md` (what `pose:` series return, TST-038),
`mechsim/testing/CLAUDE.md`, rows SCO-027 to 030, INT-005, INT-006, ACC-025. Builds in the foreground through
`tools/locked_build.sh` (600000 ms timeout; no detached jobs); filtered pytest while iterating; if you touch C++,
end with ONE `tools/locked_build.sh core debug-sanitize -R "'integrity|Integrity'"` (clang-tidy must be clean);
then ruff check, ruff format --check, mypy, `python3 tools/check_imports.py`; no full pytest run of your own.
Commit as you go; final message "WIP M3-I3b: swing angle and runner fixes", no attribution trailer, no push. Do not
edit `docs/queue.md` or `docs/cost-log.md`.

## Ownership

As I3, plus `mechsim/testing/runner.py` (items 2 and 6) and `tests/acceptance/test_acc_stubs.py` (item 7 only).

## Items

1. Blocker (SCO-030): `_SwingView` in `mechsim/scoring/score.py` reads `pose:<body>.x/y/z`, which the record view
   returns as the displacement from the reference pose (TST-038), not the body origin, so the line it forms is the
   change in separation: zero at rest and a wrong angle in swing. The unit tests used a fake view with absolute
   positions. Use absolute origins (the reference pose plus the displacement, or the reader's absolute form if it
   has one) and add an end-to-end test on a real recording: a rigid pendulum design run through `CoreDriver` and
   the runner, released at a stated angle; the scored peak equals the release angle and `period_about_mean` gives
   2 pi sqrt(L / g) within the row's tolerance; at rest the angle is exactly zero and is a value, not an error.
2. Major: `mechsim/testing/runner.py` calls `score_run` without `gravity`, so every `swing:` criterion errors in
   production. Pass the design's gravity (`build.design.environment.gravity`); keep the error when no gravity is
   available, with a message that names the missing input, not the run; the test of item 1 goes through this path.
3. Major: the criteria schema exempts only the literal body name `payload` from the body-name check, but the
   driver names the run-time body after the sensed payload when a load cell reads it
   (`mechsim/testing/core_driver.py`, read only): accept the names the driver can produce, from one shared
   definition, with a test for each.
4. Minor (INT-006): the limits table's `bodies` key is required by the party when any member has nodes (absent is
   a `LimitsError`, not a silently disabled check); `mechsim/integrity/tier1.py` reads the node body names and
   indices from the prepared native document instead of rebuilding them.
5. Minor (INT-005): a rigid member with only some of its end-load series recorded is an error (a partial
   recording), not not-evaluated; the schema hook's `except Exception: return None` around `file_kind` narrows to
   the expected failure and lets anything else surface.
6. The runner drops the integrity report's notes from the run report: carry them (INT-026; they reach
   `CaseScore.notes` since X1b).
7. `tests/acceptance/test_acc_stubs.py` line 481 uses an undefined name `replace` (ruff F821, from the X1a
   merge): fix the import only; do not change the test's substance.

## Report

Under 200 words: commits, filtered test counts (and the sanitizer build result if C++ changed), each item done or
not with its test, the end-to-end swing test's measured peak and period against the closed form, open issues.
