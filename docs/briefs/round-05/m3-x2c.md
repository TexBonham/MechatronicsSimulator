# M3-X2c: close X2 (G5 and G6 on native, the baseline, missed steps, the batch test)

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x2` on `wip/05-x2`, with brief
N5 merged in (a mass change on a rope-hung payload now works on the native adapter: C++ changed, so run
`tools/locked_build.sh sync` first). Call cap 110 with the handoff rule (at 100 calls stop adding scope, commit
what is green, report what is left by item). Read first: `docs/briefs/round-05/m3-x2.md` and `m3-x2b.md` (their
rules hold: no tuning; nothing of MuJoCo deleted; defects in core, integrity or scoring reported with a
reproduction, not patched), `test_cases/gantry/README.md`, `G5.yaml`, `G6.yaml`, `G16.yaml`,
`tests/acceptance/test_acc_stubs.py` (`BRIDGE_RUNS`, ACC-009 to 012, 031, 032, 036, 038), round 5 rulings 81 to
85 in `docs/queue.md`. Same build and test rules; COMMIT AFTER EACH ITEM; final message "WIP M3-X2c: X2 closed",
no attribution trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`. No conditional (ternary)
expressions. In spec table cells write abs(x), never bare pipe characters. Ownership as the X2 brief.

## Items, in order

1. G5 and G6 on the default (native) adapter: remove them from `BRIDGE_RUNS`; run both. N5's first look: G6's
   two criteria pass but the case reports fail (likely tier 1 crossings at the release, before the suite's 9 s
   evaluation start existed in that tree: check with the start in place); G5 passes `lift_up_refused` and fails
   `lift_up_ignored` (0.0827 m) and `lowered` (the lift ends at 0.4001 m where the case expects 10 cm within
   1 cm). For G5, find the cause before anything else: read the case (what it commands, what the overload
   policy must do), the firmware's event timeline on native against the bridge's (`mechsim replay` on both
   archives), the load cell reading through the mass change, and the lift's position through the case. Say
   which it is, with the numbers: (a) the load cell on native reads the rope's fitting and the reading through
   the mass change differs from the bridge's fixed payload so the firmware's overload decision differs (a model
   difference: is the native reading the physically right one?); (b) a defect in the adapter or the engine (the
   lift moving when it should hold); (c) the case's timing assuming a rigid drive. Do not edit the case's
   criteria or windows. If it is a defect in code you do not own, stop on G5 with the reproduction; otherwise
   G5 is a finding recorded with its cause, and ACC-009 to 012 state what holds on native.
2. Missed steps (ACC-032): X2b left it an expected failure because "core does not record the stepper's
   missed-step count". Check what is recorded: G10's baseline has stall and slip events, the stepper model has
   an electrical position and a rotor position, and the driver records step pulses. If the recording already
   carries what a missed-step count needs (slip events per motor, or commanded steps against rotor steps), G16
   reports missed steps per motor from it through the existing scoring quantities and ACC-032 converts, with a
   test in which a motor that the torque-margin derivation says must lose steps does lose them (an undersized
   motor through `instance:<name>.part`, or a `--set` on its torque). If the recording does not carry it, say
   exactly what output is missing and where it would come from; leave the expected failure with that reason.
3. ACC-038 (the stepper's pull-out torque against its published curve): say in the stub's reason exactly why it
   cannot convert (which data the shipped stepper parts lack, or which measurement the model cannot make); if
   the catalog does carry a curve and the model can be driven to pull-out in a small data-only case, convert it
   with a proposed tolerance and its derivation.
4. The baseline: one `mechsim suite test_cases/gantry --write-baseline` run on the default adapter after items 1
   and 2 (and the pendulum suite if it has a baseline), so `baseline.yaml` carries G16's entries, G12's current
   case hash and G5 and G6 on native; then compare with the previous file: every entry whose status or measured
   value changed is listed in the report with old value, new value and cause; hash-only changes are counted, not
   listed. The bridge baselines under `bridge_baselines/` are not regenerated (PHY-025).
5. ACC-036: run its pytest form (it has the long-run marker); report the batch's time and the case table (which
   pass, which fail, which are not valid, with the reason of record for each that does not pass). Run the
   long-run marker's whole set once and report its time; run `-m "not long_run"` on `tests/unit tests/invariant
   tests/reference` once and report its time (these two runs replace "estimates" in X2b's CI statement; they
   are the only wide runs you make).
6. `docs/milestones.md` and `docs/briefs/round-05/mujoco-removal.md`: bring both to the state after items 1 to 5
   (G5 and G6, ACC-032, the batch's table).

## Report

Under 300 words: commits, G5's cause with numbers, G6's result, ACC-032 and ACC-038 dispositions, the baseline
comparison, the batch table and the three measured times, retirement criteria met or not, open issues.
