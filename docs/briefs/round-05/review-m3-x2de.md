# Review: M3-X2d with M3-X2e (the X2 fix round: differential tolerances per axis, the misses of record, the fast tier; margins and missed steps in the simulator's report)

Reviewer: Fable (general-purpose), fresh. Read-only; spec and diff only; never the writers' reports; no builds or
runs; call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-05-x2`, branch `wip/05-x2`, head 49e6bc7.
Diffs: X2d `git -C <worktree> diff deb2bc3 3c06a8e`, X2e `git -C <worktree> diff b593b5a 49e6bc7` (skip brief
files and `docs/queue.md`; the merge 8803705 brought B3 and E8, reviewed separately under ruling 89). Briefs
`docs/briefs/round-05/m3-x2d.md` and `m3-x2e.md` in that worktree; `review-m3-x2.md` (the findings these fix);
`docs/milestones.md` (retirement table), `mujoco-removal.md`, `test_cases/gantry/README.md`; rows PHY-021,
PHY-023, PHY-024, ACC-031, ACC-032, ACC-036, INV-020, SIG-033, LIB-017, the new SCO rows; round 5 rulings 41, 83,
86 to 90 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`.

The rules these briefs ran under: no tuning (no existing criterion, window, limit or expected value changed in
`test_cases/**`; G16 gains criteria); nothing of MuJoCo deleted; defects in code the writers do not own reported,
not patched; the fixed bridge baselines never regenerated, `baseline.yaml` only for G16's new entries.

Check: (1) the per-axis tolerance derivation in `tests/acceptance/test_phy_stubs.py` and the PHY-024 row: one full
step 0.2 mm on every axis; the lift's belt stretch under the case's payload plus a driven mass of 0.4064 kg (where
does that number come from, is it the belt's driven mass of record and does it belong in the test or in data); x
and y adding nothing because the rails are level (is there really no holding force on a stopped carriage, for
instance from the stepper holding against a residual belt tension); times 3 ms, counts exact; is any dimension
present in a baseline left without a rule; is the per-entry tolerance what the miss line prints. (2) The misses
of record: `test_phy_024` a strict expected failure asserting no misses and the companion test asserting the exact
sorted list with causes: would a new miss, a vanished miss or a changed value fail the suite; is the companion
test's list the six the retirement table names; which criteria passed only under the pooled tolerance (the
writer says only G10 `slip_carriage_stopped`) and is G6 `lift_stopped` (0.600 mm) inside its own 5.5 kg
tolerance by the derivation. (3) G7 and G9 explained: the writer predicts 2.180 mm on the bridge and 1.153 mm on
native against 2.177 and 1.146 observed and attributes the difference to the stepper's damping acting on the
unpowered rotor on native while the bridge's fall shows none. Check the prediction's derivation from the archive
numbers; then judge the physics: on an E-stop or power loss the motor is unpowered, so which damping does the
native stepper model apply to a coasting rotor (viscous, detent, the model's own), is it the model's intended
behaviour (SIG rows) and does the bridge's rigid drive simply lack a rotor, so the difference is a modelling
difference (explained) and not a defect in either; say which. (4) The G10 finding: stall and slip events 8 and
20 ms earlier on native where the x belt's stretch under the drive torque predicts 3.4 and 9.4 ms; the writer
records it as an unexplained model difference. Is the prediction's model complete (the stepper's lag under load,
the frame-side encoder's sampling and resolution, the mismatch threshold's definition) or is the gap plausibly
one of those; name what a follow-up should measure. Nothing in the case changed. (5) The fast tier: the session
run cache in `tests/conftest.py` (keyed by what; can two tests that need different runs of one case collide; does
a test that needs its own run, the determinism test, bypass it; FW-025's change); the acceptance checks of G2 to
G11, G13, G14 and `test_phy_023` behind the long-run marker: what each platform still runs end to end (G1, G3, G8
native, G7 bridge, the pendulum, as claimed), INV-020 and the workflow comments carrying measured times, no case
shortened, no new dependency. (6) X2e: `missed_steps:<motor>` and `torque_margin:<motor>` as scoring quantities:
their derivations in the SCO rows against the code (commanded microsteps less rotor travel less the sine-law
lead; the part's available torque at the recorded speed less the shaft load torque, PHY-033 sign); the part's
data from the run's bindings and the library, no constant from a test; what the catalog provides for the
available torque (a curve or a holding torque only) and whether the row says so honestly; the unevaluable cases;
G16's new criteria (added, none changed) and whether their limits are the physics (zero lost steps within a
quarter step, margin above zero) and not tuning; ACC-031 and ACC-032 now assert against the report's values with
the a-priori derivation as a cross-check at a stated tolerance; the overloaded run: the mass or part chosen,
the derivation that the lift's demand exceeds the available torque at cruise by at least 20 percent, the stall
inside the move window (not at the release), the count at least one, the margin negative, the report saying so;
the nominal sweep losing none after E8; the G16 baseline entries regenerated only, every change listed (three
measured values moved by micrometres or less after E8: expected or not). X2e's result: the NEMA 17 lift sweep loses
one step at 0.065 s with least margin -0.123 N m at 0.022 s, the release transient of the unstretched lift belt (the
rope snatches to 79 N against 44.5 N static); ruling 91 puts the sizing criteria behind the 9 s evaluation start with
a before-evaluation line (brief X2f, in flight, not in your diff): is that right, or does it hide a sizing result the
owner should see; X2e's overloaded run was not built (a payload mass cannot over-demand within the load cell's limit;
ruling 91 makes it a motor-parameter override): check the writer's two limits (5.216 kg; a held lift at most 10.7
percent over its cruise curve) from the design and catalog data. X2e also changed `mechsim/testing/schema.py` and
`schemas/test_case.json` (quantity refs), outside its ownership: is the change the minimum the new quantities need
(invariant 3: schema regenerated, example validates).
(7) `docs/milestones.md`: criterion 2's row restated with the per-axis results, the six misses, what is
explained and what is a finding; nothing marked met without evidence; G16's row. (8) invariants 2, 3, 4, 5, 9;
R-18 (spec edits in the same commit as the behaviour).

Report, under 450 words: verdict per brief (accept, accept with a fix round, reject), findings by severity as
`file:line, one sentence, row`, your reading of (3) and (4) for the owner (explained, or a finding, in one
sentence each), and a short statement for the owner: what the evidence now shows and does not show about
retiring the bridge, given the six misses of record.
