# scoring/: contract

Scoring (SCO). Reads a run's recording through `testing.driver.RecordView`, never the engine. `__init__.py` exports
nothing.

| File | Holds |
|---|---|
| `statistics.py` | `statistic(name, times, values) -> (value, worst_time)` for final, minimum, maximum, peak_absolute, period; `upward_crossings`, `window_indices`, `compare`, `within`, `StatisticError` |
| `score.py` | `score_run(case, recording, report, limits, sweep, run, integrity) -> CaseScore` (`variation`, `crossings` fields), `not_run(...)`, `run_failed(case, error, sweep)` (driver error from `run`: errored, end reason run_error, ran False), `CriterionResult`, `CaseScore` (`approximations` from the run report, printed one `approximation:` line each, SCO-013), `SuiteScore.status`, `suite_baseline`, `regression_diff(suite_score, baseline)` (SCO-009), `diff_sections` (report order, SCO-010), `report_text`, `report_markdown(suite_score, diff | None)` (shareable Markdown, SCO-012), `ScoringError` |

## Rules

- Case passes only when every criterion passes; suite only when every run passes; any error makes it errored.
- A run error (`RunReport.reason == "run_error"`) errors every criterion (SCO-007); an unevaluable criterion errors.
- Period: upward zero crossings of the raw series, linear interpolation, (last - first) / (N - 1), N >= 2.
- `within` inclusive; limit text is the case's as written (`limits`), so baselines keep `${}` forms.
- Baseline from nominal runs only; pass/fail/error need a `RunIdentity` (archived run); unrun cases are `not_run`.
- Write a baseline with `testing.baseline.baseline_text`; identical archives give identical bytes (SCO-011).
- Diffs report newly passing, newly failing, unchanged, missing, added, in that order (SCO-010).
- `fault:<name>` (ruling 30): as a quantity 1 while the last `controller_fault` event at or before the sample names it, else 0; as an event each change to it.
- Structural (SCO-014): `safety_factor:<member>`, `deflection:<member>` read `IntegrityResults` (one sample at its
  time) through `_StructuralView`; never recomputed; missing results or member is an error.
- Crossings (SCO-015): each `integrity` event is a `crossings` line (report `integrity crossing:`); a passing case
  with one fails; not baselined.
- Variations (SCO-019): titled `[variation seed <s>: <drawn>]`, a per-seed table after the sweep tables; not baselined.
- `interval` and `no_overlap` measures: `testing/CLAUDE.md`, Criteria on recorded events.

## Section index

| Section | File | Line |
|---|---|---|
| Criteria | score.py | 112 |
| Suite, baseline and report | score.py | 543 |
