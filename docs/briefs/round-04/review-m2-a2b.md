# Review: M2-A2 (power, safety, new sensors) and M2-B (controller faults, fault scheduling, recorded series)

Reviewer: Fable (general-purpose). Read-only; spec rows and diffs only; never a writer's report; no builds or
tests (the writers' full runs are on record: A2 308 ctest and 3428 pytest green; B 294 ctest and 3428 pytest
green); call cap 45.

| Brief | Worktree | Diff | Spec rows | Brief of record |
|---|---|---|---|---|
| A2 | `/home/tex/dev/MechatronicsSimulator-04-a2` | `git -C <worktree> diff round/04-milestone-2...wip/04-a2` (1e03870) | SIG-002, 003, 004, 014, 018, 019, 020, 022 in `docs/spec/sig.md`; LIB-019, 021, 025, 027 in `docs/spec/lib.md`; PHY-034 | `docs/briefs/round-04/m2-a2.md` (its reference arithmetic for the load cell was wrong: 2e-3 x 5 V x 19.62 / 49.05 = 4.0e-3 V, 0.512 V after gain 128, 635 counts; the writer used the correct values) |
| B | `/home/tex/dev/MechatronicsSimulator-04-b` | `git -C <worktree> diff round/04-milestone-2...wip/04-b` (37ba56f) | CTL-001, 011, 012, 013, 014, 018, 020, 021 in `docs/spec/ctl.md`; KER-021, 023 in `docs/spec/ker.md`; SIG-022; TST-017, 018; REC-027 and the archive layout in `docs/spec/rec.md` | `docs/briefs/round-04/m2-b.md` |

Read first: root `CLAUDE.md`, both briefs, the rows, `core/signals/CLAUDE.md`, `core/controller/CLAUDE.md`,
`core/kernel/CLAUDE.md`, `mechsim/record/CLAUDE.md` (all four in the diffs), then the diffs.

Check:
1. Rows implemented as written and cited; spec edits in the same diff (R-18).
2. A2 physics: load cell conversion, quantization, nonlinearity tier, saturation at the overload limit recorded as
   an approximation (the writer records it as a layer `approximation` event rather than in the driver's list: judge
   against SCO-013); amplifier clamp, first-order low-pass (y += (1 - exp(-2 pi f h))(x - y)) and the ADC
   quantization from the chip's bits and reference; IMU specific force by finite difference (exact for a linear
   velocity ramp, zero in free fall), gyro in the body frame, ranges and resolutions; brake as an implicit joint
   drive at the engagement position with slip at brake_torque (the detent scheme at saturation), engage and release
   delays, the stiffness bound against the PHY-034 envelope and its derivation; `power_loss` injection on the
   supply and the rail reaching the controller party (SIG-019, 022); the writer notes the brake in the power stage
   sees its rail one step late: acceptable or a stage-order defect?
3. B: `schedule_freeze`, `schedule_slowdown` (ticks once per `factor` control periods, outputs hold), overlap and
   factor checks as config errors at start; `latency_ticks` FIFO (zeros before the first delayed output, events and
   state delayed with the outputs, latency 0 byte-identical); brownout threshold from the chip data through
   `NativeRuntimeConfig`; `schedule_slip` and `schedule_payload_change` calling C's adapter operations before the
   step with conflicts refused; fault events written to the `TickRecord`; KER-021 test (physics keeps stepping
   through a freeze). The writer counts latency in whole control ticks and slow-down by skipping ticks: say whether
   that reading of CTL-020 and CTL-021 is the natural one or needs an owner ruling.
4. Recorded series: the writer could not build per-name paths (`TickRecord` carries no joint or payload names) and
   used index columns `physics/joint_positions`, `physics/joint_drive_torques/{torque,driven}`,
   `physics/payloads/{body,carrier,attached,force}` with units "m or rad". Judge against REC-018 (units explicit) and
   REC-024: is an index column with a mixed unit acceptable, or must the first row's metadata carry names and
   per-joint units (as `signals/pins` does with a group attr `names`)? This is the finding that matters most.
5. The Windows `error_text` dedup (`core/controller/src/windows_text.hpp`): POSIX path untouched, Windows path
   behaviour-identical.
6. Cross-brief: `mechsim/testing/core_driver.py` must map the new row-dict fields into `tick.PhysicsOutputs` or the
   new series stay empty (owed to X1); the brake, load cell and IMU need design-side `senses`/`acts_on` and the
   chip's ADC data through the wiring (X1). List what X1 must do, precisely.
7. Invariants 1, 4, 5, 7, 9 and style; ownership (A2's three category edits in `core_wiring.py`, `core_driver.py`,
   `build.py` were granted).

Report, under 450 words: verdict per brief, findings by severity as `file:line, one sentence, spec row`, owner
rulings needed, cost note.
