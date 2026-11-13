# M3-X1c: fix round on the structural acceptance cases (review of X1b at 6c68248)

Fresh agent (the X1b transcript is at 160 calls). Agent: module-writer (Opus). Worktree
`../MechatronicsSimulator-05-x1b` on `wip/05-x1b` at 6c68248. First step: `git merge round/05-milestone-3` (N4 has
landed there: recorder body `names`, node pose series, `core_driver.py`; resolve `docs/spec/traceability.md` by
regenerating with `uv run --no-sync python tools/traceability.py --write`; commit the merge with no trailer). Call
cap 110 with the handoff rule (at 100 calls stop adding scope, commit what is green, report what is left). Read
first: the original brief `docs/briefs/round-05/m3-x1b.md`, `docs/briefs/round-05/review-m3-x1b.md`, round 5
rulings 45, 57 to 61 (`docs/queue.md`), `mechsim/scoring/CLAUDE.md`, `mechsim/integrity/CLAUDE.md`,
`mechsim/testing/CLAUDE.md` (case criteria). Builds in the foreground through `tools/locked_build.sh` (600000 ms
timeout; no detached jobs; never wait on a background task without checking `ps`). Iterate with filtered pytest
and `core debug -R "'integrity|Integrity'"`; at the end ONE `tools/locked_build.sh core debug-sanitize -R
"'integrity|Integrity'"` (the sanitizer preset is the only one that runs clang-tidy; the build must be clean),
ruff check, ruff format --check, mypy, `python3 tools/check_imports.py`; no full pytest run of your own (ruling
56). Commit as you go; final message "WIP M3-X1c: acceptance fix round", no attribution trailer, no push.

## Ownership

As X1b, plus `mechsim/scoring/` and the case criteria schema in `mechsim/testing/schema.py` with
`schemas/test_case.json` (item 1), and `tests/invariant/test_schema_design.py`, `test_schema_test_case.py` (item
4). Not yours: `core/engine/`, `core/physics/`, `core/kernel/`, `mechsim/physics/`, `designs/gantry/`,
`test_cases/gantry/`.

## Items

1. The oracle lives in the case (ruling 60; ACC-037, 039 to 043): every reference case carries two-sided criteria
   with the tolerance of its spec row: R1 the period and the amplitude decay over 20 periods; S1 the settle time
   as a band; B1 and B2 the settled deflection as a band around the closed form and the first natural frequency.
   Add the scoring statistics this needs and nothing more: a period (and frequency) taken about the mean of a
   pose-derived series (the review found a `pose:` reference reads the free joint's coordinate from its placement,
   so `period` could not score the rope pendulum), and a settled value (mean over a stated final window) beside
   the existing dynamic peak, so `deflection:<member>` can be read both ways (the dynamic peak of a straight
   release is about twice static: say so in the SCO row). Each acceptance test asserts `score.status == "pass"`
   and, for each case, that one perturbed run (a `--set` that changes the length or stiffness by 5 percent with
   the criteria unchanged) fails on the criterion that should catch it.
2. Not evaluated is an error (invariant 7): a NaN or not-evaluated quantity read by a criterion raises the
   scoring unevaluable error (the case reports unevaluable with the reason), never pass and never a plain fail;
   the facade's integrity table carries not-evaluated as an explicit state (optional value plus flag), not a NaN
   in a float field (FAC-013); tests for both.
3. Node indexing made checked (INT-006): an unknown parent, or a parent that is another member's node, is an
   error, not world; the recorded body count must equal neutral bodies plus member nodes (assert, with a typed
   error naming both counts) instead of re-deriving the adapter's layout from an upper bound; read the node series
   by the recorder's `names` where N4 provides them; a native run missing the body series is an error distinct
   from "built rigid".
4. Data: the steel bars in `designs/cantilever_beam`, `designs/simple_beam` and `designs/rope_spring` carry a
   steel material (their headers say steel; today the body material is `aluminium_6063_t5_extrusion`, so the
   safety factor used aluminium yield); `B1_cantilever.yaml` states its step in a `rates` block, not a comment;
   list the new designs and cases in the two invariant tests that enumerate shipped data.
5. ACC-039 (ruling 59): choose the fixture's step (or stiffness) so the scheme's correction to the settle time is
   inside the row's tolerance, judge the case against the continuous closed form ln(50) / (zeta omega_n), and
   keep the corrected value only as the published approximation line; state x = omega h in the header.
6. Spec rows per rulings 57 to 61: INT-006 tolerance (2 percent against the continuum, 1e-6 against the chain
   value); DES-033 says fibres are optional on the shorthand with the flagged approximation and its error for a
   circle; the ACC-037 to 043 tolerance rows as proposals with their derivations; SCO rows for the new
   statistics.

## Report

Under 250 words: commits, filtered test counts and the sanitizer build result, each case's criteria with measured
value, band and the perturbed run's failing criterion, each item done or not, schema fields added, open issues.
