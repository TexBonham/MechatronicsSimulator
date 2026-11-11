# M2-C: physical faults on the MuJoCo bridge (PHY-006)

Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-04-c` on branch `wip/04-c`, from
`round/04-milestone-2`. Read first: root `CLAUDE.md`, `core/CLAUDE.md`, `core/physics/mujoco/CLAUDE.md`,
`mechsim/physics/mujoco/CLAUDE.md`, `core/kernel/CLAUDE.md` (PhysicsAdapter, PhysicsOutputs), spec rows PHY-006,
PHY-016, PHY-033, PHY-034 in `docs/spec/phy.md`, ENG-035 in `docs/spec/eng.md`, TST-015 and TST-020 in
`docs/spec/tst.md`, the queue's milestone 1 rulings on transmissions and joint drives, and the open rulings table.
Source by section marker only: `core/physics/mujoco/src/mujoco_adapter.cpp` (Shafts 446, Inputs 690, Mid-run
changes 949), `mechsim/physics/mujoco/translate.py` transmissions and payload sections.

Call cap 150. Iterate with `tools/locked_build.sh core debug -R 'mujoco'` (single machine build slot, shared;
wait when it blocks) and `uv run pytest -q tests/unit/test_mjcf_translation.py` for the translation. Exactly one
full run at the end: `tools/locked_build.sh core debug`, `tools/locked_build.sh sync`, `uv run pytest -q`. Commit
once as WIP ("WIP M2-C: bridge faults"); do not push. Spec edits in the same commit (R-18). Style per root
`CLAUDE.md`; no MuJoCo type outside `core/physics/mujoco/` (PHY-011).

## Ownership

`core/physics/mujoco/**`, `mechsim/physics/mujoco/**`, `tests/unit/test_mjcf_translation.py`,
`tests/unit/test_phy_stubs.py`, `tests/acceptance/test_phy_stubs.py` (PHY-006 conversion only; PHY-016 stays a
stub for the integration wave), `core/kernel/include/mechsim/kernel/physics_adapter.hpp` and `physics_types.hpp`
(additive changes only: the new adapter operations and output fields below; brief B rebases onto you),
`core/bindings/` only to expose the new operations (list each binding line in the report), `docs/spec/phy.md`,
`docs/spec/traceability.md` (regenerated). `mechsim/testing/*` is not yours: write the adapter operations and
tests; the integration wave routes `ScheduledFault` physics faults through the driver.

## 1. Transmission slip (PHY-006, TST-015 adapter half)

Adapter operation `set_transmission_slip(name, fraction)` with fraction in [0, 1]; 0 clears it; an unknown
transmission or an out-of-range fraction throws `AdapterError`. Semantics in the reflection mapping (the shaft is
not a body): for every step while slip s > 0, the drive rest on the driven joint advances by (1 - s) of the
commanded rest increment, `Delta r_eff = (1 - s) Delta r_cmd` (increments taken on the shaft-side rest before the
c scaling), and the missed shaft travel `s Delta r_cmd` accumulates in `slip_accum` per transmission. Outputs:
`transmissions[i].slip` = `slip_accum` (shaft radians or metres, in the shaft's unit); the shaft joint's reported
position and velocity become `q_driven / c + slip_accum` and `qd_driven / c + s qd_cmd` so that a motor-side
encoder sees the full commanded motion and a frame-side encoder sees the driven motion only (the G10 mismatch).
When slip clears, `slip_accum` stays (lost travel is not recovered). Explicit shaft torque (no drive) under slip:
transmitted force scaled by (1 - s); record that this is the approximation for torque-driven shafts.

Reference (derive in the test): a drive rest advancing at constant shaft velocity omega for T seconds with
s = 0.3: driven joint advances c (1 - s) omega T, shaft reports omega T, slip = s omega T, each within 1e-9
relative of the rest increments (the rest is set exactly per step; PHY-034 exact steady tracking); the joint
drive torque stays bounded by the drive stiffness times the static lag.

## 2. Payload change (PHY-006, ENG-035 bridge half, TST-020 adapter half)

`set_payload_mass(name, mass)` exists (inertia rescaled). Make attachment real: in `translate.py` a payload body
attached to a carrier (design `initial_state.payload` or an instance of the payload kind the integration wave adds)
is emitted as a body with a free joint under world plus a `<weld>` equality to its carrier body named
`<payload>.attach`, active when attached, listed in a `mechsim:payloads` custom entry
(`name:carrier`). `set_payload_attachment(name, carrier_or_empty)`: carrier equal to the translated one toggles
the weld on, empty toggles it off (release; the payload then falls under gravity and contacts as its geoms allow),
any other carrier throws (MuJoCo cannot re-parent; say so). `release_load(name)` is the empty-carrier case. The
weld's `solref` uses the equality softness convention already in the translation (`2 step, 1`).

Reference tests: a 2 kg payload welded to a hook body held fixed shows the weld's constraint force equal to m g
within 1e-6 relative after one step (equality force from `efc_force`); released, after t = 0.2 s it has fallen
g t^2 / 2 = 0.19620 m within 1 percent (soft contact absent, floor absent in the fixture); re-attached while
resting at the carrier's pose stays within 1e-6 m. Mass change mid-run: the weld force steps to the new m g.

## 3. Backlash (queue: "bridge refuses backlash until PHY-006")

Dead band b on a transmission: the driven joint receives the shaft command through a play element with memory:
`r_eff` moves only when `r_cmd - b / 2 > r_eff` or `r_cmd + b / 2 < r_eff`, pulled to that bound. Apply on the
drive rest (shaft units, before c). Record in the translation report as an approximation (no impact dynamics).
Reference: a reversal of the commanded rest by less than b moves the driven joint by nothing; by b + x moves it by
c x, within 1e-9 relative. Remove `backlash` from the unmappable list in `translate.py` and the two `CLAUDE.md`
files; `slip` as a static parameter stays unmappable (the dynamic fault above is the slip model), `self_locking`
stays refused. If the cap approaches, drop this section and say so.

## 4. Conversions

Convert PHY-006 in `tests/unit/test_phy_stubs.py` through the Python binding (slip and payload operations on the
loaded adapter). Update both `CLAUDE.md` files: operations, outputs, approximations. PHY-016 stays a stub.

## Report

Under 250 words: files, commands with counts, binding lines added, approximations recorded, Open issues.
