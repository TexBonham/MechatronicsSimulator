# scoring/: contract

Scoring (SCO). Reads a run's recording through `testing.driver.RecordView`, never the engine. `__init__.py` exports
nothing.

| File | Holds |
|---|---|
| `statistics.py` | `statistic(name, times, values) -> (value, worst_time)` for final, minimum, maximum, peak_absolute, period, and (SCO-027) mean, amplitude, period_about_mean, frequency_about_mean (Hz); a NaN sample raises `StatisticError` (SCO-029); `upward_crossings`, `window_indices`, `compare`, `within`, `StatisticError` |
| `score.py` | `score_run(case, recording, report, limits, sweep, run, integrity) -> CaseScore` (`variation`, `crossings` fields), `not_run(...)`, `run_failed(case, error, sweep)` (driver error from `run`: errored, end reason run_error, ran False), `CriterionResult`, `CaseScore` (`approximations` from the run report, printed one `approximation:` line each, SCO-013; `notes` from `RunReport.notes`, printed `note:` / `- Note:` before them, INT-026), `SuiteScore.status`, `suite_baseline`, `regression_diff(suite_score, baseline)` (SCO-009), `diff_sections` (report order, SCO-010), `report_text`, `report_markdown(suite_score, diff | None)` (shareable Markdown, SCO-012), `ScoringError` |

## Rules

- Case passes only when every criterion passes; suite only when every run passes; any error makes it errored.
- A run error (`RunReport.reason == "run_error"`) errors every criterion (SCO-007); an unevaluable criterion errors.
- Period: upward zero crossings of the raw series, linear interpolation, (last - first) / (N - 1), N >= 2; `period_about_mean` / `frequency_about_mean` the same on the series minus the window mean; `mean` needs a window start (the settled value); `amplitude` = (max - min) / 2.
- `within` inclusive; limit text is the case's as written (`limits`), so baselines keep `${}` forms.
- Baseline from nominal runs only; pass/fail/error need a `RunIdentity` (archived run); unrun cases are `not_run`.
- Write a baseline with `testing.baseline.baseline_text`; identical archives give identical bytes (SCO-011).
- Diffs report newly passing, newly failing, unchanged, missing, added, in that order (SCO-010).
- `fault:<name>` (ruling 30): as a quantity 1 while the last `controller_fault` event at or before the sample names it, else 0; as an event each change to it.
- Structural (SCO-014): `safety_factor:<member>`, `deflection:<member>` read `IntegrityResults` (one sample at its
  time) through `_StructuralView`; never recomputed; missing results or member is an error. A deflection history
  (`deflection_series(member)` on the integrity report, `_DeflectionHistory` protocol) is read whole (SCO-028: maximum
  = peak, mean over a final window = settled). A NaN table value or sample is unevaluable: error with the reason (SCO-029).
- Swing (SCO-030): `swing:<a>.<b>` angle from gravity of the line a to b (unsigned), `swing:<a>.<b>.<axis>` signed in
  the vertical plane through that world axis (`_SwingView`, `score_run(..., gravity=)`, the runner passes
  `runner.design_gravity(build.design)`); body origins are the view's `kinematics.reference` origin plus the `pose:`
  displacement (TST-038); no gravity input or no reference poses is unevaluable. Payload names:
  `testing.schema.case_payload_body(senses, joints)`.
- Crossings (SCO-015): each `integrity` event whose detail states tier 1 (ruling 26) is a `crossings` line (report `integrity crossing:`); a passing case
  with one fails; not baselined.
- Variations (SCO-019): titled `[variation seed <s>: <drawn>]`, a per-seed table after the sweep tables; not baselined.
- `interval` and `no_overlap` measures: `testing/CLAUDE.md`, Criteria on recorded events.

## Section index

| Section | File | Line |
|---|---|---|
| Criteria | score.py | 113 |
| Suite, baseline and report | score.py | 548 |
