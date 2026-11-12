# M3-R: recorder series, structural scoring, sweeps and variations, viewer overlays

Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-r` on branch `wip/05-r` from
`round/05-milestone-3`. Python only; one `tools/locked_build.sh sync` at the start (the lock is shared with two
C++ fix rounds; waiting is normal). Read first: root `CLAUDE.md`, `mechsim/CLAUDE.md`, the `CLAUDE.md` of
`mechsim/record`, `mechsim/scoring`, `mechsim/testing`, `mechsim/viewer`, `mechsim/facade` (the cursor), `core/kernel/CLAUDE.md`
(`PhysicsOutputs.members` start and end wrenches, `ropes` tension, `TickRecord`), spec rows REC-004, 010, 026 in
`docs/spec/rec.md`, SCO-014, 015, 018, 019, 024 in `sco.md`, TST-007, 025, 026 in `tst.md`, VIE-009, 010, 011 in
`vie.md`, INT-022, 024 (what integrity will write: a per-member table and tier 1 events; brief I1 and I2 land
later, so define the shapes here and name them in your report), ADR 0010 (events), ADR 0016 (rope tension per
segment: the kernel's `RopeTension` holds one value per rope; record the rope's load-end tension and max tension as
the adapter maps them), round 5 rulings 8, 9, 21 in `docs/queue.md`, and the two branches whose field names you
consume: `git -C /home/tex/dev/MechatronicsSimulator show wip/05-d:mechsim/builder/CLAUDE.md` (`BuildResult.members`)
and `show wip/05-m:mechsim/materials/CLAUDE.md` (`material_overrides` metadata key). The stubs:
`tests/unit/test_rec_stubs.py`, `test_sco_stubs.py`, `test_tst_stubs.py`, `test_vie_stubs.py`.

Call cap 150. Filtered: `uv run --no-sync pytest -q tests/unit/test_record*.py tests/unit/test_scoring.py
tests/unit/test_runner.py tests/unit/test_viewer.py tests/unit/test_facade_service.py tests/unit/test_rec_stubs.py
tests/unit/test_sco_stubs.py tests/unit/test_tst_stubs.py tests/unit/test_vie_stubs.py`; one full `uv run --no-sync
pytest -q`, ruff, format, mypy, check_imports, traceability `--write` first. Commit once as WIP ("WIP M3-R: recorder,
scoring, sweeps, viewer"), no trailer, no push. Spec edits same commit.

## Ownership

`mechsim/record/**`, `mechsim/scoring/**`, `mechsim/testing/{schema.py, runner.py, driver.py, core_driver.py}`
(sweeps, variations, the member-list and overrides metadata), `mechsim/viewer/**`, `mechsim/facade/{service,
protocol,server}.py` only for the cursor fraction and the overlay results (additive), `schemas/`, the tests named,
`docs/spec/{rec,sco,tst,vie,fac}.md`, `docs/spec/traceability.md`, the contracts. Not yours: `mechsim/integrity/`,
`designs/`, `test_cases/` beyond a sweep example for the pendulum.

## Deliverables

1. REC-004, REC-026: member end-loads (start and end wrenches per member, six values each) and rope tensions (load
   end and max per rope) recorded every physics tick with names and units attrs (format 2 conventions); REC-010:
   integrity events recorded with their simulated time (an event kind `integrity` with member, quantity, value and
   limit in `detail`; the shape I2 will emit). Run metadata gains `material_overrides` (M's key, text plus sha256)
   and the structural member list (D's `BuildResult.members`). Exports carry the new series. Tests on synthetic rows.
2. SCO-014, SCO-015: structural criteria `safety_factor:<member>` and `deflection:<member>` read from the integrity
   results attached to the run (define `IntegrityResults` in `mechsim/scoring` or `testing/driver.py` as the contract
   I1 fills: per-member peak stress, deflection, safety factor, flags), never recomputed; a recorded tier 1 crossing
   fails the case. SCO-024: a `settle_time` criterion (time for a series to enter and stay within a band around a
   target) with a limit; derive the test's expected value from a synthetic damped oscillation.
3. TST-007, TST-025, TST-026, SCO-018, SCO-019: sweep and variation definitions in a case or suite file (parameter
   name, values or a seed count), one run per value or seed with the value or seed in the run id and metadata, a
   side-by-side report of criterion values per sweep value and per variation seed; each variation rerunnable alone
   with an identical result (test on the pendulum design through the driver fakes and one real P1 sweep over the
   joint damping). G16's stepper sweep uses this through `instance:<name>.part=` substitution (ruling 8).
4. VIE-010, VIE-011, VIE-009 fraction: viewer overlay results served through the facade: member stress per member
   over time from the recorded end-loads and the member list's sections (bending stress M c / I at each end; say
   where c and I come from), swing angle of a suspended load over time from the recorded rope or payload pose
   (angle from the vertical through the attachment), both as series the client can draw; the replay cursor keeps a
   fractional time accumulator so `advance` does not drop leftover time (the round 4 carry). Additive protocol
   results; schema regenerated; conformance cases.
5. Conversions of the REC, SCO, TST, VIE stubs this brief proves; contracts updated; the facade protocol doc.

## Report

Under 300 words: files, commands with counts, the `IntegrityResults` and event shapes named for I1 and I2, stubs
converted, Open issues.
