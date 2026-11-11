# M2-B: controller faults, kernel fault scheduling, recorded joint series

Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-04-b` on branch `wip/04-b`, from
`round/04-milestone-2` after briefs C and W2 have landed (C's additive changes to `physics_adapter.hpp` and
`physics_types.hpp` are yours to build on; W2's `#ifdef _WIN32` branches in the controller sources must stay intact). Read first: root `CLAUDE.md`, `core/CLAUDE.md`,
`core/controller/CLAUDE.md`, `core/kernel/CLAUDE.md`, `mechsim/record/CLAUDE.md`, `core/bindings/CLAUDE.md`, spec
rows CTL-001, 011, 012, 013, 014, 018, 020, 021 in `docs/spec/ctl.md`, KER-021 in `docs/spec/ker.md`, SIG-022 in
`docs/spec/sig.md`, TST-017, 018 in `docs/spec/tst.md`, REC-027 and the archive layout in `docs/spec/rec.md`, the
queue's milestone 1 rulings on the frozen controller and brownout. Source by section marker:
`core/controller/src/*` (native runtime tick, freeze, brownout), `core/kernel/src/physics_party.cpp`,
`controller_party.cpp`, `core/kernel/tests/test_cosim_kernel.cpp` (faults 589), `mechsim/record/tick.py`,
`layout.py`, `archive.py`.

Call cap 150. Iterate with `tools/locked_build.sh core debug -R 'kernel|controller'` (machine-wide build lock
shared with another writer; wait, never bypass). One full run at the end: `tools/locked_build.sh core debug`,
`tools/locked_build.sh sync`, `uv run pytest -q`. Commit once as WIP ("WIP M2-B: controller faults, fault
scheduling, joint series"); do not push. Spec edits in the same commit (R-18). Style per root `CLAUDE.md`.

## Ownership

`core/controller/**`, `core/kernel/**` (except what C changed: extend, do not rewrite), `core/bindings/**` for the
new calls and record fields (list each), `mechsim/record/**`, `mechsim/_core.py` mirrors, `tests/unit/test_ctl_stubs.py`,
`tests/invariant/test_ctl_stubs.py`, `tests/acceptance/test_ctl_stubs.py` (CTL-015 stays a stub, M4),
`tests/unit/test_ker_stubs.py`, `tests/invariant/test_ker_stubs.py`, `tests/unit/test_record*.py`,
`tests/unit/test_rec_stubs.py`, `core/controller/CLAUDE.md`, `core/kernel/CLAUDE.md`, `mechsim/record/CLAUDE.md`,
`docs/spec/{ctl,ker,rec}.md`, `docs/spec/traceability.md`. Not yours: `mechsim/testing/*` (the integration wave
calls your scheduling from the driver), `core/signals/*`, `core/physics/*`.

## 1. Scheduled faults on the parties (TST-017, 018 kernel half; KER-021; SIG-022)

Faults are known before t = 0 (the case timeline), so scheduling is pre-run and the kernel stays
deterministic. `ControllerParty::schedule_freeze(start, end)` (ticks, start inclusive, end exclusive; overlapping
windows a `KernelConfigError` at `start()`): the party calls `freeze(true)` at `start` and `freeze(false)` at `end`
on its runtime; while frozen the runtime's `tick` is not called, held levels and empty buffers are published
(existing behaviour), and the kernel keeps stepping physics and signals (KER-021: test with a 1 kHz physics party
counting steps through a 50-tick freeze). `ControllerParty::schedule_slowdown(start, end, factor)` (integer factor
>= 2): in the window the runtime ticks once per `factor` control periods, outputs hold in between (CTL-021);
`factor` 1 or a window overlap is a config error. Brownout: the rail voltage already reaches the party (SIG-022
coupling `signals.rails`); the chip's `brownout_threshold` now comes from `NativeRuntimeConfig` (keep
`placeholder_brownout_threshold_v` as the default and name the chip field in the `CLAUDE.md`); convert CTL-013 with
a test that drops the rail below threshold for 30 ticks: outputs zero during, `inits()` increments once on
recovery, firmware statics reset (the mirror fixture's counter restarts), and a drop that stays above threshold
changes nothing. `PhysicsParty::schedule_slip(transmission, start, end, fraction)` and
`schedule_payload_change(payload, tick, mass | carrier)` call C's adapter operations at those ticks before the
step (one per tick per target; a conflict is a config error). Each schedule is recorded as a `TickRecord` event
(`source` kernel, `kind` fault, `detail` the fault and target) on the tick it starts and ends.

## 2. Output latency (CTL-020)

`ControllerPartyConfig::latency_ticks` (default 0): the party publishes the runtime's outputs delayed by that many
control ticks through a FIFO, zeros before the first delayed output arrives; events and state codes delay with
them (they are what the controller emitted at that tick). Test: a step on a pin output appears exactly
`latency_ticks` later; latency 0 is byte-identical to today's recording (run the kernel determinism fixture with 0
and compare rows).

## 3. Recorded joint positions and drive torques (queue "Owed after the gate"; REC-027 row text)

`TickRecord.physics` already carries `PhysicsOutputs`; make `joint_positions` and `joint_drive_torques` reach the
archive: Python mirror `tick.py`, `layout.py` `FIXED_SERIES` entries `physics/joint_positions/<joint>` (unit rad or
m by joint type from the first row's metadata, axis tick) and `physics/joint_drive_torques/<joint>` (N m or N),
written and read back by `archive.py`, exported like every series, included in `archives_identical`. Extend the
REC-027 row text to name them (same commit) and the archive layout section of `mechsim/record/CLAUDE.md`. Record
the transmission `slip` output (C's new field) the same way under `physics/transmissions/<name>/slip`.

## 4. Conversions

Convert CTL-001, 011, 012, 013, 014, 018, 020, 021, KER-021 and any REC stub the new series satisfy. CTL-015 and
the emulated rows stay stubs. Update the three `CLAUDE.md` files.

## Report

Under 250 words: files, commands with counts, binding lines, stubs converted, Open issues.
