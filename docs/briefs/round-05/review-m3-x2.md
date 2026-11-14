# Review: M3-X2, X2b, X2c (the suite on the native adapter by default, G16, one batch, the retirement check)

Reviewer: Fable (general-purpose), fresh. Read-only; spec and diff only; never the writers' reports; no builds or
runs; call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-05-x2`, branch `wip/05-x2`; a test run is using
the working tree, so read at the commit: `git -C <worktree> diff 44face2 HEAD -- . ':!core' ':!docs/queue.md'
':!docs/cost-log.md'` (the `core/` part is brief N5, reviewed separately), files with `git show HEAD:<path>`.
Briefs `docs/briefs/round-05/m3-x2.md`, `m3-x2b.md`, `m3-x2c.md`; `docs/milestones.md` (retirement criteria),
ADR 0004; rows PHY-021 to 026 and the differential tolerance row, ACC-009 to 012, 031, 032, 036, 038, LIB-017,
TST-040, INV-020; round 5 rulings 1, 41, 46, 81 to 85 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`.

The rules these briefs ran under: no tuning (no criterion, window, limit, physics or damping value changed to
make a case pass on native); nothing of MuJoCo deleted (the bridge stays until the owner's retirement decision);
a failing case is a finding.

Check: (1) the default: `physics.adapter` defaults to `native`, a bridge run is the non-default with the
`-mujoco` run-id suffix (TST-040), stated in one place; every test that means the bridge says so; (2) no tuning:
list every change to a criterion, window, limit or expected value in `test_cases/**` in the diff and classify it
(the one derived evaluation start of 9 s applied to G2 to G16; G12 and G15 moved from 8 s to 9 s; baseline
values regenerated; anything else); the evaluation start's derivation in the suite README (five loaded time
constants of rail_x at 1.74 s: derive tau from the printed damping and frequency) and what G1 (2 s long, no
start) is judged on; (3) the comparison of record, native against the bridge: G1 to G4, G8, G11, G13, G14 within
the ruling 41 tolerance; G7 and G9 (a held position 2.18 mm on the bridge, 1.15 mm on native), G10 (events 8 to
20 ms earlier): are the stated causes (rope payload and compliant lift belt; compliant belts moving the
encoder-mismatch crossing) supported by numbers in the diff or asserted; PHY-024 a strict expected failure
listing exactly these: is its reason complete and would it convert on a false pass; (4) the retirement tests:
PHY-021 (what is compared and at what tolerance; the exclusions listed), PHY-023 (the reference tier enumerated
on native in a subprocess: can a test be silently skipped or deselected and the check still pass), PHY-025 (the
bridge baselines under `bridge_baselines/` are fixed files the suite never regenerates: is there a guard),
PHY-026 (no runtime module imports MuJoCo on a native run: import graph and loaded modules; is the bridge's
Python package imported lazily or does the test pass only because of how it looks); the proposed differential
tolerance row: derived; (5) G16 (ACC-031, 032, LIB-017): the torque demand derivation in the case header (m a r
plus friction plus, for the lift, m g times the lead term over the efficiency) against the stated margins (lift
NEMA 17 +0.053 N m, NEMA 23 +0.877; x and y +0.344); the lift's cruise torque 0.325677 against a derived
0.325666; missed steps from the step and direction pins against the rotor: is that a count of lost steps or of
something else, and the stated 2 lost steps of the NEMA 17 lift at the release snatch: a real result, or the
release transient before the evaluation start (the same artefact that snapped G5's rope before the start
existed); the writer says no G16 criterion reads missed steps because scoring has no statistic that sums a
pin's counts: then what does ACC-032's conversion assert; (6) ACC-036: the batch runs G1 to G16 through the CLI
and the report lists every case with its criteria; the long-run marker and CI: what runs where, anything now
run on no platform, and Windows no longer running any native gantry case end to end: say so if true; (7) G5's
story: on native without the evaluation start the rope snapped on the release transient (6011 N against 5000 N
at 0.012 s) and the case then passed one criterion for the wrong reason (`lift_up_refused` accepts any
refusal): with the start in place G5 passes; is `lift_up_refused` accepting any refusal a weakness in the case
that the acceptance test covers (ACC-009 checks the reason); (8) `docs/milestones.md`: each retirement criterion
marked met or not met with its test, nothing marked met without evidence; `mujoco-removal.md` complete (files,
CMake targets, the fetch, CI steps, the wheel, schema enum values, tests, contracts, what replaces the
differential tests); (9) invariants 2, 3, 4, 5, 9; R-18.

Report, under 450 words: verdict, findings by severity as `file:line, one sentence, row`, your reading of (3),
(5) and (6), and a short statement for the owner: what the evidence does and does not show about retiring the
bridge.
