# M2-D: suites, regression, criteria, overrides (mechsim/testing, scoring, library)

Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-04-d` on branch `wip/04-d`, from
`round/04-milestone-2`. Python only; your one build is `tools/locked_build.sh sync` at the start (waits for the
machine's single build slot). Read first: root `CLAUDE.md`, `mechsim/CLAUDE.md`, `mechsim/testing/CLAUDE.md`,
`mechsim/scoring/CLAUDE.md`, `mechsim/library/CLAUDE.md`, spec rows TST-008, 014 to 020, 027, 037 in
`docs/spec/tst.md`, SCO-009 to 012, 021, 025 in `docs/spec/sco.md`, LIB-016, 017 in `docs/spec/lib.md`, ADR 0012
(D4). Source by section marker: `mechsim/testing/runner.py` (Timelines 289, Runs 416, Suites 566),
`mechsim/testing/baseline.py`, `mechsim/scoring/score.py` (Criteria 98, Suite 373), `mechsim/testing/schema.py`
(Pass criteria 450, Sweeps and variations 688).

Call cap 150. Filtered while iterating: `uv run pytest -q tests/unit/test_runner.py tests/unit/test_scoring.py
tests/unit/test_test_case_schema.py tests/unit/test_tst_stubs.py tests/unit/test_sco_stubs.py
tests/unit/test_lib_stubs.py tests/unit/test_catalog_registry.py`. One full run at the end: `uv run pytest -q`,
ruff, ruff format --check, mypy, `python3 tools/check_imports.py`, traceability `--write` before it. Commit once
as WIP ("WIP M2-D: suites, regression, criteria, overrides"); do not push. Spec edits same commit (R-18).

## Ownership

`mechsim/testing/{runner.py, baseline.py, schema.py, cli parts in mechsim/cli.py}`, `mechsim/scoring/**`,
`mechsim/library/registry.py`, `schemas/test_case.json`, `schemas/baseline.json`, a new `schemas/suite.json`
and its example, `tests/unit/test_runner.py`, `test_scoring.py`, `test_test_case_schema.py`, `test_tst_stubs.py`,
`test_sco_stubs.py`, `test_lib_stubs.py` (LIB-016, 017 only), `test_catalog_registry.py`, `test_cli*.py`, the
three `CLAUDE.md` files, `docs/spec/{tst,sco,lib}.md`, `docs/spec/traceability.md`. Not yours: `core_wiring.py`,
`core_driver.py` (the integration wave routes physics and controller faults once briefs B and C land), the gantry
files, `mechsim/builder/`.

## 1. Suite file and regression (TST-008, TST-027, SCO-009 to SCO-011)

`SuiteFile` model (`suite.yaml` beside the cases): `name`, `design` (name, version), `cases` (relative case files,
at least one, no duplicates), `baseline` (relative path, required by TST-008), optional `note`. `run_suite` accepts
a suite file or, as today, a directory (then the directory's `baseline.yaml` and every `<ID>.yaml` case, so milestone
1 behaviour is unchanged; a directory holding `suite.yaml` uses it). After a suite run: `SuiteScore` plus
`diff_baselines(old, new)` from `baseline.py`, surfaced by scoring as `regression_diff(suite_score, baseline)` and
written by `baseline_text` as the next baseline (SCO-011; identical bytes on a rerun). SCO-010: the diff lists
newly passing and newly failing per case and criterion first, then unchanged, missing, added. CLI `suite` prints
the diff and takes `--write-baseline <path>`. Write `test_cases/gantry/suite.yaml` for G1 to G4 as the example.

## 2. Shareable report (SCO-012)

`report_markdown(suite_score, diff | None) -> str`: a self-contained Markdown document (title, design and version,
git hashes from the archives' metadata, per case a table of criteria with measured value, limit, status, the
approximation lines, the end reason, then the regression diff). CLI `suite --report <path>`. Deterministic text.

## 3. Criteria (SCO-021, SCO-025)

`interval` exists in the schema: two event refs and a limit on the simulated time between the first occurrence of
each (SCO-021); implement in `score.py` if it is not evaluated yet, with: missing first event, missing second
event, second before first (each an unevaluable error naming the ref). `no_overlap` (SCO-025): `actuator` plus
`start_event` and `end_event` refs; recorded events are paired per actuator in time order; fails when a start
occurs before the previous end on the same actuator, or when the run ends with an unpaired start if the case says
`closed: true`. The gantry firmware (brief E) logs `motion_start` and `motion_end` events with the axis as the
argument; the recorder exposes events through `RecordView` as it does for the `event` criterion; use the same ref
form. Document both in `mechsim/testing/CLAUDE.md`.

## 4. Overrides and part substitution (LIB-016, LIB-017)

Extend the runner's `--set` and the case `parameters` to instance parameters and parts, applied to the loaded design
before the builder: `instance:<name>.<parameter>=<quantity>` overrides one catalog parameter for this run (dimension
checked against the type's parameter, SI stored, unknown instance or parameter is an error), and
`instance:<name>.part=<part name>` substitutes a part of the same component type (a part of another type, or an
unknown part, is an error naming both). Record every override in the run metadata so the archive shows it (the
`RunMetadata` field the recorder already stores for `--set`). Tests: each error, dimension mismatch, a substituted
stepper's holding torque reaching `component_parameters` (assert on the prepared build, not on a run).

## 5. Conversions

Convert in `tests/unit/test_tst_stubs.py` what is already true and now tested: TST-014, 016, 019, 037 (signals
faults reach the signal layer through the existing `SENSOR_FAULTS` wiring; test through the driver protocol with
the recorded failure-mode windows), TST-008, TST-027; leave TST-015, 017, 018, 020 as stubs with a docstring note
naming the brief that lands them (X1). Convert SCO-009 to 012, 021, 025 and LIB-016, 017.

## Report

Under 250 words: files, commands with counts, stubs converted, Open issues.
