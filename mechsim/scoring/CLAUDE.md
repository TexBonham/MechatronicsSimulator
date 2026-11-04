# scoring/: contract

Scoring (SCO). Reads a run's recording through `testing.driver.RecordView`, never the engine. `__init__.py` exports
nothing.

| File | Holds |
|---|---|
| `statistics.py` | `statistic(name, times, values) -> (value, worst_time)` for final, minimum, maximum, peak_absolute, period; `upward_crossings`, `window_indices`, `compare`, `within`, `StatisticError` |
| `score.py` | `score_run(case, recording, report, limits, sweep, run) -> CaseScore`, `not_run(...)`, `CriterionResult`, `CaseScore`, `SuiteScore.status`, `suite_baseline`, `report_text`, `ScoringError` |

## Rules

- Case passes only when every criterion passes; suite only when every run passes; any error makes it errored.
- A run error (`RunReport.reason == "run_error"`) errors every criterion (SCO-007); an unevaluable criterion errors.
- Period: upward zero crossings of the raw series, linear interpolation, (last - first) / (N - 1), N >= 2.
- `within` inclusive; limit text is the case's as written (`limits`), so baselines keep `${}` forms.
- Baseline from nominal runs only; pass/fail/error need a `RunIdentity` (archived run); unrun cases are `not_run`.
- Write a baseline with `testing.baseline.baseline_text`.

## Section index

`score.py`: Criteria 97, Suite, baseline and report 344.
`tests/unit/test_scoring.py`: Statistics 71, Criteria 171, Cases and errored runs 365, Suites, report, baselines 401.
