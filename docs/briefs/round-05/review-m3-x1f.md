# Review: M3-X1f (the gantry's G12 and G15 as results of record)

Reviewer: Fable (general-purpose), fresh. Read-only; spec and diff only; never the writer's report; no builds or
runs; call cap 25. Worktree `/home/tex/dev/MechatronicsSimulator-05-x1a`, branch `wip/05-x1a`, commit 6f17849,
diff `git -C <worktree> diff e65c083 6f17849` (skip the brief file); read files at that commit with
`git -C <worktree> show 6f17849:<path>`. Brief `docs/briefs/round-05/m3-x1f.md`; rows ACC-025, 026, 029, 030 and
the acceptance tolerance rows; SCO-009; round 5 rulings 52, 57, 60, 63 to 81 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`; `docs/gantry-team-inputs.md`.

The rule this brief ran under: no tuning. No physics value, damping value, limit, band or window changed to make
a criterion pass; a failing placeholder is a finding; an expected failure's reason names its cause.

Check: (1) no tuning: list every changed number in `test_cases/gantry/G12.yaml`, `G15.yaml` and
`designs/gantry/**` in the diff and say for each whether it is a criterion being made a-priori (allowed), a
window or limit moved (not allowed without a derivation), or data; (2) ACC-025: the first-ramp bound: a_max = a
p(zeta) with p the peak acceleration of a second-order follower of a ramp in rate (derive p(zeta) for small
zeta: it approaches 2; the writer has 1.987 and 1.982), the swing bound R0 + 2 a_max / g with the pendulum
length to the payload's centre; the new strict expected failure for the hook's recorded acceleration (3.32 m/s^2
in y against 0.099 allowed) with the reason that the firmware's step train rings the belt mode: is that reason
supported by the data in the diff (step rate against the belt mode's frequency) or asserted; does ACC-025 itself
still assert something a-priori; (3) ACC-026, 029, 030 stay strict expected failures: each reason specific, true
and naming the placeholder or the limitation (ruling 64: no swing damping, derived settle time; rulings 69, 77,
81: rail_y not valid under the one-way carried-load flag at three step sizes with a ratio that does not
converge; rail_x safety factor 1.42, 1.41, 1.28 against the 1.5 placeholder at 10, 20, 40 substeps); none would
convert on a false pass (two-sided assertions); (4) the corrected hand oracle for rail_y's settled stress (8.78
MPa; the earlier 11.24 counted own weight twice): derive it from the section and loads in the data; (5) the
loaded-frequency comparison: the writer measured rail_y at 36.25 Hz against an oracle of 18.72 Hz and rail_x at
22.7 Hz against 16.99 Hz and calls it an engine defect (the carried mass counts only the joint tree below the
carried joint, not the shackle and payload hanging on the rope). Judge it: a payload on an elastic rope is not a
rigid mass on the rail (the rope's own bounce is near 21 Hz); say what the right oracle is for a one-way model
(the rail with its rigidly carried bodies only), whether 36.25 Hz matches that from the data, and whether the
row for ENG-052 must state that bodies hung on ropes reach the member only through the rope's tension; (6) the
baseline (SCO-009): 88 entries on the new design hash; the milestone 2 entries on the bridge unchanged within
1e-6 m except one (G6 lift_stopped, one tick): is a one-tick change on the bridge explained by the design
change (rail height, frames, damping fields) or a regression to chase; the writer did not attribute changes per
entry; (7) G12 has no `integrity.evaluate_from`, so tier 1 crossings at 0.013 and 0.018 s (the release
transient) count against it: should G12 state an evaluation start, and is adding one tuning or the same
correction G15 already carries; (8) `docs/gantry-team-inputs.md`: complete and honest (the damping rows say no
source is cited); (9) run-twice determinism tests; R-18; invariants 2, 3.

Report, under 400 words: accept or not, findings by severity as `file:line, one sentence, row`, your judgement on
(5), (6), (7), and a three-line statement of what milestone 3 can and cannot claim for G12 and G15.
