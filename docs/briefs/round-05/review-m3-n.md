# Review: M3-N (native adapter over the own engine)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never the writer's report; no builds or runs;
call cap 40. Worktree `/home/tex/dev/MechatronicsSimulator-05-n`, branch `wip/05-n`, commit 530e55b, diff `git -C
<worktree> diff fc6aac6 530e55b` (read per file; start with `core/physics/native/src/native_adapter.cpp`,
`mechsim/physics/native/translate.py`, `mechsim/testing/core_driver.py`, the PHY rows). Brief
`docs/briefs/round-05/m3-n.md`; rows PHY-001 to 012, 018 to 022, 027 to 034 (`docs/spec/phy.md`), ENG-043 to 048,
BLD-017 and BLD-018 (`bld.md`); ADR 0003, 0004, 0013, 0014, 0016; round 5 rulings 1, 9, 13, 16, 21, 24, 29, 31 to
33 (`docs/queue.md`); contracts `core/physics/native/CLAUDE.md`, `mechsim/physics/native/CLAUDE.md`,
`core/kernel/CLAUDE.md` (`PhysicsAdapter`, `PhysicsOutputs`), `core/physics/mujoco/CLAUDE.md` (the behaviour to
reproduce).

Check: (1) translation: the neutral model to the engine model in neutral order, six joint types, free joints for
payloads, geometries and contact pairs, transmissions (reflected inertia, efficiency by power direction, belts
with ENG-043 stiffness, lead screw self-locking, gear pairs, backlash), rope bodies appended last with indices
remapped, nothing design-specific; (2) the terms the adapter adds because the engine lacks them: joint friction,
joint limits and the per-step joint drive as implicit terms. Derive each form and judge it: the drive against
PHY-034 (exact sag K q = tau and steady lag D v / K, backward Euler with no stability limit, the x^2 / 3 frequency
error and e^(-pi x) decay published as approximation values, ruling 21); friction as an implicit damping
f / max(rate, threshold) and whether it holds a loaded joint at rest; limits (penetration, energy, the step it
engages); whether these belong in the engine (say so if the adapter is the wrong place); (3) outputs against
PHY-003, 004, 007, 033, 034: frames and reference points (the writer notes the bridge reports joint positions from
the start coordinate and linear velocity at the centre of mass while the native adapter uses the neutral placement
and the body origin: say which matches the contract and what X2's differential check must do), member wrenches in
the member frame with the recorder's rotation gated on the bridge (ruling 24), one tension per rope crossing the
kernel; (4) errors and determinism: every engine error ends the run as an adapter error with time and cause
(invariant 7), run-twice bitwise identity, no unordered iteration; (5) adapter selection: the writer selects the
adapter by environment variable `MECHSIM_PHYSICS_ADAPTER` or a `CoreDriver` argument. Judge it against invariant 4
and the archive metadata (the adapter must be part of the recorded run configuration, settable from the case or
`--set`, never only from the environment); (6) the hand-written JSON reader in `core/physics/native/src/json.*`:
malformed, truncated, deep, duplicate-key and number edge inputs give a typed error, no undefined behaviour; say
whether an existing dependency should carry it; (7) the G3 table: native against bridge final_x 0.4000250 against
0.4000625, x_travel_lower 2.5e-5 against 6.25e-5, lift_travel 8.26e-5 against 6.88e-5, ready_after_homing 5.245
against 5.247 s. Say whether these differences are explained by the models (rigid belts on the bridge, drive
mapping) and what tolerance ruling 13's differential check should take for driven cases; (8) the builder change
(`structure.py` segment default by support condition; BLD-017, BLD-018) and the three E3 lows; (9) invariants 2,
4, 5, 7, 9, style (braces, no ternary, no raw owning pointers), R-18, T4, tests derived from closed forms rather
than from the implementation.

Report, under 450 words: verdict, findings by severity as `file:line, one sentence, spec row`, owner rulings
needed, what brief N2 must carry, cost note.
