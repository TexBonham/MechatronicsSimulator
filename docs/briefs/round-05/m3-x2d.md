# M3-X2d: X2 fix round, part one (differential tolerances per axis, the misses of record, the batch test, the fast tier)

Why: the X2 review (ruling 86) found the PHY-024 comparison pooling one tolerance (0.783 mm, the lift belt's sag
under 5.5 kg) onto every axis and every criterion, so two criteria pass only through it, unlisted; the causes
given for G7's and G9's held-position difference and G10's earlier events are asserted, not shown; the batch test
hard-codes its case list; one unit test still lists G1 to G15 as the shipped cases; and the `not long_run` tier
takes 29 minutes on this machine (target under 15, every CI job under 30) because every milestone 2 gantry case is
now a full flexible-rail, rope-payload run of 80 to 110 s. Part two (X2e: torque margins and missed steps in the
simulator's report, a motor that must lose steps losing them in motion) follows when brief E8 lands.

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x2` on `wip/05-x2` (the native
adapter is the default; its extension is built; `tools/locked_build.sh sync` only if `uv run` reports a stale
build). Call cap 120 with the handoff rule (at 110 calls stop adding scope, commit what is green, report what is
left by item). Read first: `docs/briefs/round-05/m3-x2.md` (its rules hold: no tuning, nothing of MuJoCo deleted,
defects in code you do not own reported with a reproduction, not patched), `review-m3-x2.md`,
`tests/acceptance/test_phy_stubs.py` (`differential_misses`, PHY-024's reason), `tests/acceptance/test_acc_stubs.py`
(the module fixtures that run cases, ACC-036), `test_cases/gantry/README.md`, `docs/milestones.md` (the retirement
table), `.github/workflows/python.yml`, the `long_run` marker in `pyproject.toml`, rows PHY-021, PHY-024, PHY-025,
ACC-036, INV-020, TST-040, round 5 rulings 41, 83, 85 to 87 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`,
`designs/gantry/design.yaml` (the three belt drives: stiffness, pulley, the carried masses). Builds and runs in the
FOREGROUND only through `tools/locked_build.sh run ...` with a 600000 ms timeout, never backgrounded; filtered
pytest while iterating; exactly TWO wide runs, both `uv run pytest -m "not long_run" --durations=30`, one at the
start of item 5 and one at its end, each timed. Lints at the end: `uv run ruff check .`, `uv run ruff format
--check .`, `uv run mypy`, `python3 tools/check_imports.py`. COMMIT AFTER EACH ITEM; final message "WIP M3-X2d: X2
fix round, part one", no attribution trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`. No
conditional (ternary) expressions. Curly braces on every control flow. In spec table cells write abs(x), never
bare pipe characters.

## Ownership

As brief X2, plus `tests/unit/test_baseline_schema.py`, `tests/conftest.py`, `.github/workflows/python.yml`,
the `markers` list in `pyproject.toml`, `docs/spec/{phy,acc,inv,tst}.md` rows you change,
`docs/spec/traceability.md`, `docs/milestones.md`, `test_cases/gantry/README.md`. Not yours: `core/**`,
`mechsim/scoring/`, `mechsim/integrity/`, any criterion, window, limit or expected value in `test_cases/**`,
`test_cases/gantry/bridge_baselines/**` (fixed, PHY-025), `test_cases/gantry/baseline.yaml` (not regenerated in
this brief: a change you believe it needs is a finding in the report).

## Items, in order

1. `tests/unit/test_baseline_schema.py::test_example_baseline_validates_and_matches_g3` fails on the branch (it
   lists G1 to G15 as the shipped cases; the baseline carries G16 since X2c). Read the failure first; fix the test
   so the shipped list comes from `suite.yaml`, not a range; run that file.
2. Differential tolerance per axis and per dimension (PHY-021 and PHY-024 rows; ruling 41 reworded by this brief):
   `differential_misses` gets, per baseline entry, the tolerance of the quantity the criterion reads, derived in
   the test header and stated per axis in the PHY-024 row. Positions on a driven axis: one full motor step times
   that axis's ratio (1.8 degrees on the 40 mm per turn pulley: 0.2 mm, every axis) plus THAT axis's belt stretch
   under the load it carries at rest in THAT case (the lift: the case's payload plus the carriage, hook and cell
   on the belt stiffness; x and y: the belt carries no gravity load at rest, so only the holding force the drive
   exerts against the stopped carriage, state what that is from the record or why it is zero). Times and event
   times: three control ticks. Counts and presences: exact. Every other dimension present in any baseline (force
   from the load cell, angle, anything else): its own derived tolerance stated in the header, or exact; a pooled
   tolerance, or one borrowed from another axis, is not allowed. The miss list names the criterion, both values,
   the tolerance applied and the axis it came from. Then: `test_phy_024` stays a strict expected failure asserting
   no misses (criterion 2 of the retirement table is not met until the owner says so, ruling 83) and a companion
   test with the same requirement marker, not an expected failure, asserts that the sorted miss list equals the
   list of record (a constant in the test, one line per miss with its cause from item 3), so a new miss or a
   vanished one fails the suite. Report which criteria that passed under the pooled tolerance now miss (the review
   named G10 `slip_carriage_stopped`, 0.73 mm at 295 ms, and G6 `lift_stopped`).
3. G7's and G9's held position and G10's event times, explained with numbers or recorded as findings. From the
   archives of both adapters (run each case once per adapter through the CLI into the scratchpad
   `/tmp/claude-1000/-home-tex-dev-MechatronicsSimulator/e12b8ab0-2af1-4dae-a612-eacb9e002661/scratchpad/`, or
   `mechsim replay` if it gives the series): for G7 `hook_held` (bridge 2.177 mm at 12.05 s, native 1.146 mm at
   12.044 s) and G9 `load_held` (native 1.146 mm; the bridge value from its baseline), name the joint and the
   instant of the worst deviation on each adapter, then predict the difference from the model: the stop distance
   of the lift carriage after the brake engages from the recorded speed, the brake torque and the moving inertia
   on the bridge's rigid drive, against the same with the belt's stretch under the brake's holding force and the
   payload hanging on its rope on native. A prediction within 30 percent of the observed 1.03 mm is an
   explanation; otherwise it is a finding with the numbers. For G10's stall, re-home and slip events 8 to 20 ms
   earlier on native: predict the shift of the encoder-mismatch crossing from the belt's stretch under the stall
   torque against the mismatch threshold at the commanded speed; the same 30 percent rule. Write each explanation,
   as numbers, into the PHY-024 companion test's list of record and the suite README; nothing into a case file.
4. ACC-036 (`test_acc_036`): the case list and the run count come from `suite.yaml` and the cases' sweeps, not
   from `range(1, 17)` and `+ 3`; the written Markdown report is checked for every case and criterion like the
   printed one; it stays `long_run`.
5. The fast tier. First wide run: `uv run pytest -m "not long_run" --durations=30`, timed (expect about 29 min);
   from its durations list, say how many times each gantry case is run end to end across the tier and by which
   tests (module-scoped fixtures run a case once per module; the run-twice determinism test runs G3 twice by
   design; `test_phy_023` runs the reference tier in a subprocess). Then bring the tier under 15 minutes on this
   machine with every CI job under 30, in this order of preference, stopping when the target is met: (a) a
   session-scoped cache in `tests/conftest.py` keyed by case path, adapter and overrides, so a case the tier runs
   in several modules runs once (a test that needs its own run, such as determinism, says so and bypasses it);
   (b) full-length native gantry runs over 60 s that the tier's own assertions do not need (the acceptance checks
   of G2 to G11, G13, G14 that are the batch's cases anyway) move to the long-run job behind the existing marker,
   provided every platform still runs at least one native gantry case end to end (G1, 2 s) and the pendulum, and
   the long-run job stays under its own budget (measure: the marker's current set took about 21 minutes for the
   batch alone; state the new total and raise `timeout-minutes` only with the number). Not in this brief: a new
   test dependency (xdist: the per-process firmware host build cache is shared) or shortening any case. Then the
   second wide run, timed, and the long-run set's time if (b) changed it (run `-m long_run` once only if you moved
   tests into it). Update the workflow comments, the marker text and INV-020 with the measured times, and
   `test_cases/gantry/README.md` if it states what runs where.
6. `docs/milestones.md`: the retirement table's criterion 2 row restated from items 2 and 3 (per-axis tolerances,
   the misses of record with their causes, what is explained and what is a finding); nothing marked met without
   evidence; `mujoco-removal.md` only where it names the differential test's replacement.

## Report

Under 300 words: commits; item 1's cause; the per-axis tolerances as derived and the miss list of record, with the
criteria that passed only under the pooled tolerance; item 3's predictions against the observed differences and
the verdict on each (explained or finding); ACC-036's new shape; the two wide-run times and the long-run time if
changed, how many case runs the tier made before and after, and what each platform still runs end to end; the
lints; open issues; what is left if you stopped at the cap.
