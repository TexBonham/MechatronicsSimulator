# M3-GV2: GV fix round (tests read the data; G15 targets re-derived; the comparison of record frozen; sizing results re-recorded)

Ruling 99 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md` governs. Same worktree `../MechatronicsSimulator-05-gv`
on `wip/05-gv` (head 8f4cb9f), same rules as `m3-gv.md` (data only, except the TESTS this brief names; no requirement
value changes; no tuning; sources recorded). Call cap 110 with the handoff rule (at 100 stop adding scope, commit what
is green, report by item). Ownership adds: `tests/acceptance/test_acc_stubs.py`, `tests/acceptance/test_phy_stubs.py`,
`tests/unit/**` and `tests/reference/**` only for the eighteen failing tests, `test_cases/gantry/G15.yaml` (the two
settled-deflection targets only, by derivation), `test_cases/gantry/bridge_baselines/` (one new fixed file),
`docs/spec/{acc,phy,tst}.md` rows for the tests changed, `docs/spec/traceability.md`. Still not yours: any code outside
tests, any other criterion, firmware.

## Items, in order

1. The eighteen failing tests (`uv run pytest -m "not long_run" -q` from your wide run: list them first): each one
   asserted a gantry value as a literal (the 9 s start, part counts, efficiency 0.95, damping 0.01, rotor inertia,
   the misses of record). Rewrite each to read the value from the data file the design or case declares (the
   design's open parameters, the catalog part the design names, the case field), deriving any expected result from
   those values in the test; a test whose assertion only makes sense for the old placeholder is reworded to the
   quantity it meant to check. No test is deleted, skipped or weakened; its requirement marker stays.
2. G15's settled-deflection targets (`G15.yaml`): the closed form for each rail's 2040 section and support condition
   under the case's design load, derived in the README beside the rail calibration, the two case values replaced by
   the derived ones (state old and new); the acceptance tests ACC-029 and ACC-030 read `G15_RAILS` and `G15_RECORD`
   from the design and the regenerated baseline, not constants, and their strict expected failures convert where
   the criterion now passes for the physical reason (aluminium rails), reason kept as history.
3. The comparison of record frozen (ruling 99 c): copy the landing's native baseline, `git show 66d8b26:test_cases/
   gantry/baseline.yaml`, to `test_cases/gantry/bridge_baselines/native_at_retirement.yaml` with a header note (the
   design hash it was recorded at, the date, "fixed, never regenerated; the comparison of record for ADR 0004");
   `test_phy_024` and `test_phy_024_misses_of_record` compare the bridge baselines against THAT file; PHY-024's and
   PHY-025's rows say so; the live `baseline.yaml` is no longer read by them. The four G10 misses stay the list of
   record.
4. ACC-031 and ACC-032 re-recorded against the real curves (ruling 99 d): the a-priori derivation uses each part's
   published curve at the cruise and homing speeds; the test asserts the report's values against it: the NEMA 23's
   positive margins and zero lost steps, the NEMA 17 lift's negative margin at the 60 rpm point and its lost steps
   during the move (in the span, at rest at the end: say what the report shows), x and y NEMA 17 positive. G16's own
   criteria are not edited: the sweep run with the NEMA 17 lift fails its margin criterion and that is the sizing
   result of record (the README's G16 line says so). Run the sweep once.
5. The lift belt (ruling 99 e): record the overload (about 63 N against 26.2 N working tension) as a design finding in
   `docs/gantry-team-inputs.md` and the milestones G15 and G16 rows. If Gates publishes an allowable working tension
   for GT2 9 mm or wider that covers 63 N with margin, add that part (sourced) and name it on the lift belt only, with
   the finding noted as resolved by substitution; otherwise the finding stands and the 6 mm belt stays flagged.
6. Fix the three stale design comments; regenerate `baseline.yaml` once (the design hash changes); compare with the
   previous entry by entry (statuses and values moved, with cause); `uv run mypy`; then the one wide `-m "not long_run"`
   run (must be green) and `tests/acceptance -m long_run -k "acc_031 or acc_032 or phy_024 or acc_036"` once.

## Report

Under 350 words: commits; the eighteen tests and how each now reads its value; G15's old and new targets with the
derivation; the frozen comparison file and the four misses; ACC-031 and ACC-032's recorded values per motor and part;
the belt decision; the baseline comparison; the wide run and the long-run selection results; lints; open issues.
