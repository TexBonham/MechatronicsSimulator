# M3-D2d: the driver's step count follows the engine's rule; tier 1 notes; the belt row

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1a` on `wip/05-x1a` (the engine's
step-count fix has just been merged in: run `tools/locked_build.sh sync` first, foreground, 600000 ms timeout).
Call cap 45 (at 40 stop adding scope, commit what is green, report what is left). Read first:
`docs/briefs/round-05/m3-d2c.md`, `mechsim/physics/native/translate.py` (`steps_before`, about line 586),
`tests/unit/test_damping_run.py`, `mechsim/testing/core_driver.py` (the run-end lines; where the tier 1 party's
results are collected), `mechsim/integrity/tier1.py` (`trace_notes`), rows ENG-047, ENG-052, PHY-036. Filtered
pytest in the foreground with the counts read; no C++; ruff check, ruff format --check, mypy,
`python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final message "WIP M3-D2d:
step count rule, tier 1 notes", no attribution trailer, no push. Do not edit `docs/queue.md` or
`docs/cost-log.md`. No conditional (ternary) expressions. In spec table cells write abs(x), never bare pipe
characters.

## Ownership

`mechsim/physics/native/translate.py` (`steps_before` only), `mechsim/testing/core_driver.py` (the two places
named), `tests/unit/test_damping_run.py` and one new test file for item 2, `docs/gantry-team-inputs.md`, rows
PHY-036 and the native and testing contracts, `docs/spec/traceability.md`.

## Items

1. The engine now decides both evaluation starts by step index: at the first step
   n = max(0, ceil((start - t_0) / h - 1e-9)) with h the engine step (the control step over the substeps), exact
   for whole-step starts and rounding up between steps (ENG-052, ENG-047). `steps_before` still repeats a summed
   floating-point time and counts 11 and 8001 at ten substeps. Make it the same rule (state it in the docstring
   and the PHY-036 row, in adapter steps: say how engine steps map to recorded rows at more than one substep);
   the two strict expected failures in `tests/unit/test_damping_run.py` convert to passing tests; add starts
   that are not a multiple of the step. Fix the contract line in `mechsim/physics/native/CLAUDE.md` that
   describes the old disagreement.
2. D2c item 6, not started: the tier 1 party's hogging under-read lines (`tier1.trace_notes`) reach the run's
   notes where the driver collects the party's results; one test through a real run: the note is present when
   its condition holds (a carried couple on a member whose own-weight moment hogs at the foot point) and absent
   otherwise.
3. `docs/gantry-team-inputs.md`, the belt damping row: there is no sourced range for it in this repository.
   Say exactly that: the value 0.01 was fixed by ruling 78 as the same low figure as the rails', no published
   range is cited for a fibre-reinforced timing belt, and both the value and its source are owed by the gantry
   team (do not invent a range or a citation).

## Report

Under 100 words: commit, filtered test counts, the rule as implemented, each item done or not.
