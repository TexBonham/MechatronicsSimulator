# Review: M3-I5 (integrity follow-ups, not-valid results)

Reviewer: Fable (general-purpose), fresh. Read-only; spec and diff only; never the writer's report; no builds or
runs; call cap 25. Worktree `/home/tex/dev/MechatronicsSimulator-05-x1a`, branch `wip/05-x1a`, commit a0d37b9,
diff `git -C <worktree> diff 534b94e a0d37b9`. Brief `docs/briefs/round-05/m3-i5.md` in that worktree (and
`m3-i4.md`, `m3-i4b.md` for what it builds on); rows INT-005, 008, 011, 023 to 028, SCO-029, TST-044, FAC-013;
round 5 rulings 60, 63, 69 to 74 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`; contracts
`mechsim/integrity/CLAUDE.md`, `core/integrity/CLAUDE.md`, `mechsim/scoring/CLAUDE.md`.

Background you need: a flexible member is a lumped chain of node bodies joined by rotational springs, bending in
one plane (about the member frame's z). A carried joint's load reaches the member as lever-rule forces, and
torques for a couple, on the two node bodies adjacent to its foot point (engine, ENG-052). Strength reads the
bending moment at every node spring, (E I / l) times the relative rotation, plus the end moments, and recovers
the peak under a carried load as the intersection of the neighbouring segments' moment lines (ruling 71).

Check: (1) yield requests only from the evaluation start (TST-044): before it an exceedance is the
`integrity_before_evaluation` event only; the test; an existing test was adapted: weakened or not; the writer
says rope snap is decided engine-side and so is not gated: say whether a rope can snap during the release
transient and whether that needs a row; (2) fatigue and impact now read the peak along the member: the oracles (a
pinned member under a sinusoidal centre load: stress amplitude (P L / 4) c / I, the cycle count exact; impact at
mid-span) derived or fitted; the governing section chosen how; (3) the carried couple. The brief proposed a bound
C b / L; the writer found by a sweep that C b / L is not a bound and used abs(C) instead: for a 400 N push the
plain read is 7.875, the conservative read 15.875 and the true value 8.4 (P a b / L + C b / L), so the
conservative read is nearly twice the truth. Derive the exact statics instead: with a force P and a couple C at
the foot point s the moment diagram is linear each side of s with a jump C at s; the chain applies lever-weighted
torques w_k C and w_(k+1) C on the two node bodies, so its own diagram has two smaller jumps at those bodies; say
exactly which spring samples lie outside the loaded bodies on each side, show that the two lines through them,
evaluated at s, give the true moments just left and just right of the jump, and therefore that the peak is the
larger magnitude of the two with no bound needed, provided s is known; say where each tier can get s (the
carried body's pose projected on the member axis, from the limits table's knowledge of `carries`) and whether
anything prevents that. State also what the exact rule gives for loads in neighbouring segments and for
opposite-sign loads in one segment, and where a bound is genuinely still needed. Is abs(C) at least a correct
(if loose) bound in the meantime, and is "tier 1 does not add the bound" an inconsistency between tiers that can
flip a verdict; (4) the three determinate oracles asserted "within theta squared": what is that tolerance and is
it derived; (5) not valid: the archive extra `carried_validity`, the table's `not_valid` rows from the flag
time, the event kind `integrity_not_valid`, scoring's state for it (the writer used "error" with the reason),
the facade's `state` and `not_valid_from`: never a pass, never a plain fail, deterministic; the flagged test
monkeypatches the validity record because no rig was flagged by the engine: what is then untested end to end;
(6) invariants 2, 4, 5, 7; R-18; style; anything in `mechsim/record/` or the facade that the owner's contract
review should see.

Report, under 400 words: accept or not, findings by severity as `file:line, one sentence, row`, the exact rule for
(3) in three sentences, owner rulings needed.
