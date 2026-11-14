# Review: M3-D2 (structural damping as design data; rope-snap wiring)

Reviewer: Fable (general-purpose), fresh. Read-only; spec and diff only; never the writer's report; no builds or
runs; call cap 20. Worktree `/home/tex/dev/MechatronicsSimulator-05-x1a`, branch `wip/05-x1a`, commit 578fcca,
diff `git -C <worktree> diff 0cd1617 578fcca` (skip brief files). Brief `docs/briefs/round-05/m3-d2.md` in that
worktree; rows DES-047, BLD-040, BLD-041, LIB-035, PHY-036, PHY-037, INT-031; round 5 rulings 10, 21, 45, 75 to
78 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`.

Check: (1) members: `flexible.damping_ratio` to node damping c = 2 zeta k / omega_1 with k = E I / l and omega_1
the calibrated chain's unloaded first mode: derive that stiffness-proportional damping C = beta K gives modal
ratio beta omega / 2 for every mode (so higher modes are damped more: state it) and that a carried mass gives
zeta omega_c / omega_1; where omega_1 comes from for each support pair (closed form or the chain's own
eigenvalue: which, and consistent with the calibration, BLD-019); an absent ratio emits one line per undamped
member; the oracles (logarithmic decrement 2 pi zeta / sqrt(1 - zeta^2), measured +0.5 percent, which the writer
attributes to the scheme: derive the scheme's contribution at omega h <= zeta / 100 and say whether 0.5 percent
is that); (2) transmissions: neutral `damping` (non-negative, default 0, at the output), catalog `damping_ratio`
on belt and lead screw, D = 2 zeta sqrt(k m) with m the driven subtree's mass at build time (a payload attached
at run time is not in m, so the realised ratio is lower: is that printed per run, and is using the build-time
mass the right convention); the bridge ignoring it with its line; schemas regenerated (invariant 3); (3) the
gantry's placeholders are exactly the brief's fixed values (rails 0.01, the three belts 0.01), flagged in the
design and in `docs/gantry-team-inputs.md` with range and reason; nothing else in the gantry data changed;
nothing gantry-specific in code (invariant 2); (4) rope-snap wiring: `rope_snap_evaluation_start` written by the
translation and set with `carried_evaluation_start` from the case's `integrity.evaluate_from`; the
before-evaluation line built from the recorded per-step `max_tension` (the engine's `held_tension` is not on the
kernel's rope output): does that line report the right thing, and is the row honest about it; the test through
`CoreDriver`; (5) every approximation line added reads correctly and is deterministic (invariant 4); R-18;
style (no conditional expressions).

Report, under 300 words: accept or not, findings by severity as `file:line, one sentence, row`, owner rulings
needed.
