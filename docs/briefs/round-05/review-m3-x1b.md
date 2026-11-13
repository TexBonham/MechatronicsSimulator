# Review: M3-X1b (structural acceptance on the native adapter, node-pose deflection)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never the writer's report; no builds or runs;
call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-05-x1b`, branch `wip/05-x1b`, commit 6c68248, diff `git
-C <worktree> diff 17c835c 6c68248`. Brief `docs/briefs/round-05/m3-x1b.md`; rows ACC-037, 039 to 043 and the
acceptance tolerance rows (`docs/spec/acc.md`), INT-006, INT-023 to 026, INT-031, FAC-013, DES-033; ADR 0016;
round 5 rulings 21, 24, 25, 28, 31, 32, 34, 45, 51, 53 (`docs/queue.md`); contracts of the packages touched.

Check: (1) node-pose deflection (ruling 28): the reference line per support pair (root tangent for a member with
a clamped end, chord for pinned ends), computed in the member frame from node body poses only (the tier 1 party
reads kernel physics outputs, invariant 5); the tier 2 check and the party agree; derive the four oracles
(P L^3 / 3 E I, w L^4 / 8 E I, P L^3 / 48 E I, P L^3 / 192 E I) and say whether the tests assert the closed form
within the 2 percent row or only the chain's own value (an assertion of "chain value within 1e-6" is a regression
check, not an oracle: both should be present); a rigid member reports unevaluated, never zero, and what scoring
does with an unevaluated or NaN deflection (invariant 7: it must be an error or an explicit not-evaluated result,
never a pass); (2) the two-axis section fields: every combination and rejection tested; the `second_moment`
shorthand leaves the fibre distances optional: say what the strength check does without them and whether that is a
silent default; values reach the neutral member unchanged; schema regenerated (invariant 3); (3) notes into
`CaseScore`, the printed report and the facade result; the rope-snap gate through `CoreDriver.run`; (4) the
acceptance cases: data-only designs and cases (invariant 2), run as a user would; ACC-037: T_N derived with the
rope's added mass and static stretch, the stated 50 microsecond step, amplitude within 2 percent over 20 periods
(measured about -1 percent: is that the scheme's loss pi omega h per period as ruling 21 predicts, i.e. derived);
the writer says a `pose:` reference cannot score the period so the test counts it about the equilibrium itself:
say whether the case's own criteria still fail when the period is wrong, or only the pytest does; ACC-039: the
settle time against ln(50) / (zeta omega_n) corrected for the scheme's decay: check the correction's derivation;
ACC-040 to 043 against rulings 31 and 32, the frequency from zero crossings at a stated x; (5) the proposed
tolerance rows for ACC-037 to 043: derived, not fitted to the measurements (T4); (6) FAC-013 conversion: the
facade table equals the direct service result on a flexible member on native; (7) the writer reports two invariant
tests (`tests/invariant/test_schema_design.py`, `test_schema_test_case.py`) hard-code the shipped designs and cases
and fail until the new ones are listed: say whether listing them is all that is owed; (8) invariants 2, 3, 4, 5,
7; style; R-18.

Report, under 400 words: verdict, findings by severity as `file:line, one sentence, spec row`, what brief X1a
(gantry G12 and G15) must know, owner rulings needed, cost note.
