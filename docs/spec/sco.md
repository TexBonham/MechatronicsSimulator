# Spec: Scoring (SCO)

## Purpose

Scoring (`mechsim/scoring/`) evaluates each pass criterion of a test case against the run's recording, reports the
measured value beside its limit with the time of the worst case, aggregates cases into suites, diffs a suite against its
regression baseline and writes the shareable report. It reads integrity results for structural criteria. It runs no
simulation and computes no structural quantity itself.

"Architecture doc" below is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`;
"Requirements doc" is `docs/Mechatronic Simulator — Requirements.md`.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| REC | SCO-001, SCO-013, SCO-016 | REC-001, REC-008, REC-013 |
| TST | SCO-001, SCO-007, SCO-009, SCO-017, SCO-018, SCO-019 | TST-005, TST-022, TST-023, TST-027 |
| INT | SCO-014, SCO-015 | INT-022, INT-025 |
| FAC | SCO-012 | FAC-012 |
| ACC | SCO-020 to SCO-025 | ACC-001 to ACC-032 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| SCO-001 | Scoring shall evaluate each pass criterion of a test case against the run's recording and nothing else. | Architecture doc, Modules: outputs, Scoring row; Data flows, run step 5 | M1 | unit | n/a |
| SCO-002 | Scoring shall report each criterion's measured value beside its limit. | Architecture doc, Modules: outputs, Scoring row; Requirements doc, Outputs and acceptance criteria, Outputs per run | M1 | unit | n/a |
| SCO-003 | Scoring shall report the simulated time of each criterion's worst case. | Architecture doc, Modules: outputs, Scoring row | M1 | unit | n/a |
| SCO-004 | Scoring shall report pass or fail for each criterion. | Architecture doc, Modules: outputs, Scoring row; Requirements doc, Outputs and acceptance criteria, Outputs per run | M1 | unit | n/a |
| SCO-005 | Scoring shall report a case as passed only when every one of its criteria passes. | Architecture doc, Modules: outputs, Scoring row, Produces | M1 | unit | n/a |
| SCO-006 | Scoring shall report a suite as passed only when every one of its cases passes. | Architecture doc, Modules: outputs, Scoring row, Produces | M1 | unit | n/a |
| SCO-007 | Scoring shall report a case whose run ended in a run error as errored, never as passed. | Architecture doc, Repository starting point, invariant 7 | M1 | unit | n/a |
| SCO-008 | Scoring shall aggregate every case of a suite into one pass/fail report. | Architecture doc, Modules: outputs, Scoring row; Requirements doc, Outputs and acceptance criteria, criterion 2 | M1 | unit | n/a |
| SCO-009 | Scoring shall diff a suite's results against its regression baseline. | Architecture doc, Modules: outputs, Scoring row; Requirements doc, Test cases, Regression | M2 | unit | n/a |
| SCO-010 | The regression diff shall highlight every criterion and case that newly passes or newly fails. | Requirements doc, Test cases, Regression | M2 | unit | n/a |
| SCO-011 | Scoring shall write each suite's scoring summary as YAML in the regression baseline format, so that it can be committed as the next baseline. | ADR 0012 (D4); Ruling D4 | M2 | unit | n/a |
| SCO-012 | Scoring shall produce the pass/fail report as a shareable document. | Architecture doc, Modules: outputs, Exports row | M2 | unit | n/a |
| SCO-013 | The report shall state, for each case, the seed, the design version and the controller mode of the run it scored, and every approximation line of the physics bridge's translation report for that run, followed by each distinct `approximation` event the run recorded (the signal layer's component notes, as `<source>: <detail>`; round 4 ruling 20). | ADR 0012 (L8, D3); ADR 0003 (approximations reported, invariant 7), brief M1-X7 coordinator ruling 2 (2026-11-04) | M1 | unit | n/a |
| SCO-014 | Scoring shall read structural criteria (safety factor, deflection) from the integrity results, not recompute them. | Architecture doc, Modules: outputs, Scoring row, Reads | M3 | unit | n/a |
| SCO-015 | Scoring shall fail a case for which integrity recorded a tier 1 limit crossing; only tier 1 crossings are recorded as `integrity` events, and scoring counts an event only when its detail states tier 1 (ruling 26). | ADR 0010, Decision, Tier 1, On a limit crossing | M3 | unit | n/a |
| SCO-016 | Scoring the same archive twice shall give an identical report. | Architecture doc, Repository starting point, invariant 4 | M1 | unit | n/a |
| SCO-017 | Scoring shall take every criterion, measure and limit from the test case file, with no design-specific criterion in code. | Architecture doc, Repository starting point, invariants 2 and 3; Modules: inputs, Test case files row | M1 | unit | n/a |
| SCO-018 | Scoring shall report a sweep's criterion values per sweep value side by side. | Requirements doc, Test cases, Batches and sweeps | M3 | unit | n/a |
| SCO-019 | Scoring shall report the results of seeded variations per variation seed. | Requirements doc, Test cases, Batches and sweeps | M3 | unit | n/a |
| SCO-020 | Scoring shall support a criterion comparing a statistic of a recorded series with a limit, the statistics being final value, minimum, maximum, peak absolute value, and period (zero-crossing method over a window, as the engine reference tests do). A power rail is a recorded series: `rail:<name>` reads 1 while the rail is energized and 0 while it is off, `rail:<name>.voltage` its voltage (round 4 ruling 27). So is the controller's reported fault: `fault:<name>` reads 1 while the fault code the controller reports is `name` in the design's `fault_codes`, else 0, from the recorded `controller_fault` events (round 4 ruling 30). | Requirements doc, Test cases (G12 swing, pendulum references); Architecture doc, Validation | M1 | unit | n/a |
| SCO-021 | Scoring shall support a criterion comparing the simulated time between two recorded events with a limit. | Requirements doc, Test cases, gantry suite table, Pass criteria column | M2 | unit | n/a |
| SCO-022 | Scoring shall support a criterion requiring the presence or absence of a recorded event or operator message within a time window; a `fault:<name>` event is each change of the controller's reported fault to `name` (round 4 ruling 30). | Requirements doc, Test cases, gantry suite table, Pass criteria column | M1 | unit | n/a |
| SCO-023 | Scoring shall support a criterion requiring no actuator motion within a time window, measured against a motion threshold. | Requirements doc, Test cases, gantry suite table, Pass criteria column | M1 | unit | not set (owed by owner) |
| SCO-024 | Scoring shall support a criterion comparing the settle time of a recorded series into a band with a limit. | Requirements doc, Test cases, gantry suite table, Pass criteria column; Architecture doc, Technology recommendation, Validation | M3 | unit | n/a |
| SCO-025 | Scoring shall support a criterion requiring that no two recorded actuator commands are active at once on the same actuator. | Requirements doc, Test cases, gantry suite table, Pass criteria column | M2 | unit | n/a |
| SCO-026 | Scoring shall support a criterion requiring every listed joint to stay within a band of its first position in a time window (a hold band), so that a hold's sag and ringing inside the band do not count as motion. | M1-X6 coordinator ruling 2 (2026-11-04), for owner confirmation | M1 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Criterion vocabulary and its schema | `value` (final, minimum, maximum, peak_absolute, period against `below`, `at_most`, `above`, `at_least` or `within` an absolute or relative limit), `interval`, `event`, `no_motion`, `hold_band`, `settle_time`, `no_overlap` (`actuator`, `start_event`, `end_event`, optional `closed`) (`schemas/test_case.json`); event refs `message:<name>` (operator message), `state:<name>` (entry into a controller state), `event:<name>` (a recorded event of kind `<name>`, such as a firmware log event, its detail the argument: the actuator instance name for `motion_start`/`motion_end`) | set in phase 1 C4; `event:` refs and the `no_overlap` fields M2-D |
| Motion threshold for "no actuator motion" | not set | owner |
| Statistic definitions (SCO-020) | window: samples with start <= t <= end, an absent bound open; final: last sample; minimum, maximum, peak absolute: the first extreme sample on a tie, its time the worst-case time; period: upward zero crossings of the raw series (before < 0, after >= 0), linearly interpolated, (t_last - t_first) / (N - 1), at least two crossings, worst-case time the last crossing; `within` is inclusive (an error equal to the limit passes), relative limit times the target's magnitude | `mechsim/scoring/statistics.py` (M1-I) |
| Other criterion measures | event: count of occurrences in the window, `state:` occurrences are entries into the state; interval: first start event to the first end event at or after it; a start event never recorded, an end event never recorded, or end events recorded only before the first start are each an error naming the ref; settle time: from the window's first sample to the first sample after which all stay in band, never settling fails; no motion: largest excursion of any listed actuator position from its first sample in the window, passes at or below the threshold; no overlap: the `start_event` and `end_event` occurrences whose detail is the actuator, in time order and record order within one time; a start while a previous one is open is a violation, and with `closed: true` so is each start still open at the end; measured = violation count, worst time the first violation (with only open starts, the last sample); no start on the actuator, or an end with no open start, is an error naming the ref | `mechsim/scoring/score.py` (M1-I) |
| Unevaluable criterion | a series not recorded, an empty window, too few crossings or an unresolved value make the criterion and its case errored, never passed; a case the runner stopped before t = 0 is errored and enters a baseline as `not_run` | `mechsim/scoring/score.py` (M1-I) |
| Shareable report format | Markdown (`report_markdown`, CLI `suite --report <path>`): title with suite verdict, design and version, run counts; per run seed, mode, run id with the archive's design and case git blob hashes and archive sha256 (or no archive), end reason, error, approximation lines, a table of criterion, status, measured, limit, worst time, note (pipes escaped); then the regression diff in five sections (newly passing, newly failing, unchanged, missing, added) or a line saying there is none; deterministic text. The plain-text report (`report_text`) stays for the terminal | M2-D |
| Regression baseline YAML layout | design name and version, suite, entries sorted by case id (natural order) then criterion, each with status `pass\|fail\|not_run\|error`, measured value, limit text, worst-case time and run identity (`schemas/baseline.json`); a diff classes entries as newly passing, newly failing (passed before, not now), unchanged, missing, added, and is reported in that order, changes first (`regression_diff(suite_score, baseline)`); the CLI `suite --write-baseline <path>` writes the next baseline, byte-identical on a rerun of the same archives | set in phase 1 C4; report order M2-D |
| Whether a baseline diff flags changed measured values that still pass | not set | owner |
| Structural criteria (SCO-014, SCO-015) | quantity refs `safety_factor:<member>` (dimensionless, the member's minimum over the run) and `deflection:<member>` (m, its peak) on a `value` criterion read the integrity results attached to the run (`IntegrityResults`, per member peak stress, deflection, safety factor with times and flags), one sample at the time integrity reports; no results or no such member is an error; never recomputed. Each recorded `integrity` event is a crossing line in the report and fails a case that would otherwise pass (an errored run stays errored); crossings do not enter the baseline | brief R (M3) |
| Variation report (SCO-019) | each variation run titled with its seed and drawn values; per case a table with one column per variation seed: the drawn parameter values, then each criterion's measured value and status; variation runs, like sweep runs, are not baselined | brief R (M3) |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Computing structural quantities | INT |
| Running cases, sweeps and variations | TST |
| Recording and series exports | REC |
| Showing criterion failures on the replay timeline | VIE |
