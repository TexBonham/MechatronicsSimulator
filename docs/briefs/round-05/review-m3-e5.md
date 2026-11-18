# Review: M3-E5 (regularised efficiency switch, self-locking screw held by a row)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never the writer's report; no builds or runs;
call cap 20. Worktree `/home/tex/dev/MechatronicsSimulator-05-e5`, branch `wip/05-e5`, commit 8beefd2, diff `git -C
<worktree> diff 14661a9 8beefd2`. Brief `docs/briefs/round-05/m3-e5.md`; rows ENG-013, 014, 025, 043, 044, 045,
048 and the rows "Transmission coupling model" and "Transmission unit-test tolerance" in `docs/spec/eng.md`; ADR
0013; round 5 rulings 37 to 39, 43, 47, 48 (`docs/queue.md`); contract `core/engine/CLAUDE.md`.

Check: (1) the switch: gain n (1 + sigma (1 / eta - 1)) forward and n (1 + sigma (eta_back - 1)) backward with
sigma = clamp(|omega| / omega_s, 0, 1) frozen at the start-of-step rate, omega_s the engine's
`friction_slip_threshold`; derive that the static equilibrium is unique at sag n F / K, that the old gains hold
exactly above omega_s, and the creep time constant the contract publishes; the writer derived 2788 settling steps
and observed 758 and 1073: say whether the derived count is a bound (fine) or a mis-derivation; (2) two choices
the writer made outside the brief: (a) at omega = 0 the implicit damping uses the larger loss of the two sides
("without it a screw lifted between n F and n F / eta ratchets"): derive whether that term is consistent with the
frozen-rate law, can add energy, or shifts the static equilibrium; (b) the lock row is two-sided when the output
force and the shaft torque are both zero: say whether a two-sided row there can hold a shaft that should be free
(for instance a non-loaded screw nudged by an impulse) and whether it is confined to self-locking screws; (3) the
lock row: one-sided, in the velocity-level pass in transmission order (invariant 4), present only in contact and
only when the load would back-drive and the shaft's own torque does not push that way; no positive work; cannot
fight the drive; lowering under a drive unresisted (ENG-044); the pivot-condition reliance gone; tests at rotor
inertia 1e-5 and 1e-2 with drift exactly zero; a non-self-locking screw back-drives; (4) the non-convergence test
uses eta = 1 because the lossy case converges in one sweep: say whether ENG-025 is still exercised for a locking
row; (5) the edit to `core/engine/tests/test_joint_limits.cpp` and to existing cases in `test_transmissions.cpp`:
any assertion weakened, any tolerance changed outside the spec row (T4); (6) the spec rows and contract match the
code; style (braces, no ternary, no raw owning pointers); R-18.

Report, under 350 words: verdict, findings by severity as `file:line, one sentence, spec row`, what brief N4 (the
adapter tightening its sag and screw tests and printing the creep as an approximation line) must know, owner
rulings needed, cost note.
