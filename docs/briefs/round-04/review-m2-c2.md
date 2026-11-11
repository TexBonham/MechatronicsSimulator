# Review: M2-C2 (bridge drive mapping, slip dynamics, brake slip phase)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never the writer's report; no builds or tests;
call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-04-c2`, branch `wip/04-c2`, commit 6032236, diff
`git -C <worktree> diff db7e1de...6032236`. Brief `docs/briefs/round-04/m2-c2.md`; rows PHY-034 (its edited text),
PHY-006, SIG-020, SIG-033, SIG-039; rulings 23, 33 in `docs/queue.md`; contracts `core/physics/mujoco/CLAUDE.md`
and `core/signals/CLAUDE.md` (both in the diff), the "Joint drives (PHY-034)" derivation above `apply_drives`.

This is physics review. Check:
1. The drive mapping: the writer replaced the previous solref/solimp mapping with an exact backward-Euler form
   (`eq_data = rest c - qvel h`, `d = Z / (1 + Z)`, `Z = (K_d h^2 + D_d h) A0`). Derive the one-DOF implicit
   spring-damper step yourself (m a = -K (q_{n+1} - r) - D v_{n+1}) in MuJoCo's constraint impedance terms and say
   whether the mapping is exact for a single DOF with `dof_invweight0` = 1 / m, what it does when A0 is only an
   approximation of the effective inverse inertia (coupled DOFs, the reflected shaft armature), whether static sag
   and steady tracking lag stay exact (PHY-034 promises both), and whether the envelope statement (K_d A0 h^2 < 400)
   still holds or must change. The runaway test: does it derive the 0.83 m/s on the old mapping and the bound on the
   new one from physics?
2. Milestone 1 behaviour: the mapping changes every drive-carried run (G1 to G4, P1 uses none?). Say what the
   reference-validator and the baselines must show at the gate for this to be accepted (PHY-028, PHY-034 reference
   rows, the G1 to G4 scoring baselines).
3. Brake slip phase: a soft slip drive (K_s = 1e-5 K_b, rest = theta + omega h + s T_b / K_s) against SIG-020's
   "apply its brake torque while engaged"; derive the torque it applies at the end state and whether it equals T_b
   opposing motion within the stated tolerance; the engage-on-moving-joint case; the deceleration test's
   (T_b / c - m g) / m_eff.
4. Invariants 4, 5, 7; style; ownership (nothing outside the list); R-18 spec edits present.

Report, under 400 words: verdict, findings by severity as `file:line, one sentence, spec row`, what the gate must
verify, cost note.
