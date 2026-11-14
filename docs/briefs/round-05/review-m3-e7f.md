# Review: M3-E7f (the carried-load validity flag's final definition)

Reviewer: Fable (general-purpose), fresh. Read-only; spec and diff only; never the writer's report; no builds or
runs; call cap 20. Worktree `/home/tex/dev/MechatronicsSimulator-05-e7`, branch `wip/05-e7`, commit 707ec10, diff
`git -C <worktree> diff b7221c9 707ec10` (skip the brief files). Briefs `docs/briefs/round-05/m3-e7c.md`,
`m3-e7e.md`, `m3-e7f.md` in that worktree; rows ENG-052, PHY-035; round 5 rulings 63, 69, 73, 74, 76 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`; the header derivations in
`core/engine/include/mechsim/engine/member_loads.hpp`.

Background: a flexible member is a lumped chain bending in one plane; a carried joint's reaction is rerouted onto
the two nodes at its foot point (one-way: the carried body does not ride the deflection) and its mass is added
to the member's inertia for the deflection motion. Because the coupling is one-way, a member driven near its own
frequency by the carried system can be pumped with energy the source never loses; the flag is information that
says when a member's result is not valid. It changes no dynamics.

The definition under review: d the deflection at the foot point, g the member's own-weight deflection there,
u_qs the quasi-static deflection from the rerouted force and moment through the chain's static compliance at the
foot; ratio = abs(d - g) / base; base = max(the peak of abs(u_qs - g) over a window of N = ln 50 / (2 pi zeta)
periods of the first loaded mode, capped at 100 periods, an undamped member getting the cap; a floor equal to
the static deflection of the carried weight at the worst foot position over the carried joint's travel); flagged
when the ratio exceeds 2.04; evaluated from the adapter key `carried_evaluation_start`, the window starting
fresh there.

Check: (1) each piece derived and implemented as stated: the factor 2 (step response) and the 2 percent margin;
the window from node damping only (no dependence on the step); the static shape subtracted from both; the floor
computed once at load from the compliance over the joint's limits (a joint without limits: the whole span); the
evaluation start; determinism (fixed-size ring, bucket edges); (2) what "no flag" bounds and what it does not: a
member can now ring up to 2.04 times its worst-case static carried deflection without a flag even when the
carriage stands where the local static value is small: is that the right trade (the review before this one found
the ratio at a support meaningless), and is it stated in the row and the run-end line; false negatives: a
resonance that stays under the floor; false positives: a single load reversal (ratio up to 4), a release from
straight judged from time 0; (3) the tests: the gravity-on gate at two step sizes and two segment counts; the
discriminating pair at 0.05 L (not flagged at 0.45 of the floor, flagged at 4.1 times it); the E7c rigs run with
gravity off so the floor is zero there: is the floor then exercised in a resonant case at all, and should one
resonant rig have gravity on; (4) the gantry: G15 from 8 s reads rail_x 1.05 (10 substeps) and 1.34 (20
substeps), rail_y 1.41 and 1.53, unflagged; the writer states 30 percent as the agreement tolerance between step
sizes because the belt's and rails' undamped modes ring at a level set only by the scheme's damping: is that an
honest statement and does it belong in the row; (5) rows ENG-052 and PHY-035 match the code; no assertion
weakened (one E7c Python test was given an evaluation start: check it still tests what it meant to); invariants
4, 5, 7; style; R-18.

Report, under 350 words: accept or not, findings by severity as `file:line, one sentence, row`, whether G15's
unflagged results can stand as results of record with the stated bound, owner rulings needed.
