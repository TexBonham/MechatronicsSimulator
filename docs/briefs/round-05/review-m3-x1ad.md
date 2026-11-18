# Review: M3-X1a and X1d (the gantry on the native adapter: flexible rails, rope payload, G12, G15)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never the writers' reports; no builds or runs;
call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-05-x1a`, branch `wip/05-x1a` (the integration branch),
commit d456cfa. The gantry work is spread over two briefs and interleaved with merges already reviewed (E6, I3,
E7), so read it by path: `git -C <worktree> diff e83666a d456cfa -- designs/gantry test_cases/gantry
docs/gantry-team-inputs.md mechsim/builder mechsim/testing/schema.py mechsim/testing/driver.py
mechsim/testing/core_driver.py tests/acceptance/test_acc_stubs.py tests/unit/test_physics_substeps.py
tests/unit/test_builder.py tests/unit/test_builder_structure.py tests/unit/test_gantry_integration.py
docs/spec/acc.md docs/spec/bld.md docs/spec/tst.md schemas/test_case.json`. Briefs
`docs/briefs/round-05/m3-x1a.md` and `m3-x1d.md` in that worktree; rulings 2, 4, 21, 27 to 32, 41, 45, 49 to 53,
57 to 68 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`; rows ACC-025, 026, 029, 030, BLD-032, BLD-037 to
039, TST-040 to 042.

Check: (1) invariant 2 and 3: nothing gantry-specific in `mechsim/`; every number in the design and cases carries
its unit; every placeholder is listed in `docs/gantry-team-inputs.md` with its file (rail height, rail frames,
second moments and fibres on both axes, fastener ratings, impact parameters, the swing settle band, the design
load mass, rail damping and calibration; the writer mentions a code constant `SHACKLE_MASS` of 1 g in the
builder: is a mass constant in code acceptable or must it be data); (2) the rope payload (BLD-032): the case
payload becomes the rope's load end at run time on native, the stand-in stays on the bridge; the bridge's
approximation line (the writer left BLD-039's test a strict stub because G3 on the bridge showed no stand-in
line: from the code, is the line emitted at all); milestone 2 cases unchanged on the bridge; (3) `physics.substeps`
(TST-042): integer >= 1, default 1, schema regenerated, reaches the adapter, rows and tier 1 stay at the control
rate; (4) rail frames (ruling 65): frame y vertical with world poses unchanged, the section's axes and fibres
consistent with that frame (check I_z and c against the stated section for vertical bending), BLD-038's note;
`carries` on both rails; (5) G12: the criteria use the swing quantity; derive the three bounds (2 a / g steady
ramp, v / sqrt(g L) stop, with an initial swing present) and say whether the first-ramp bound "from the hook's
recorded motion" (0.010594 against a measured 0.010527) is still a derived bound or has become a comparison of
the recording with itself; the settle criterion present, ACC-026 a strict expected failure with ruling 64's
reason; (6) G15: the design load in the case data (ruling 52); the settle window; the oracles for settled
deflection (derive rail_x clamped-clamped and rail_y pinned-roller under self-weight plus the carried load at
its position) against the reported 1.2427 mm vs 1.2623 and 0.8373 vs 0.8493; ACC-029 and 030 strict expected
failures: are their reasons specific and true; (7) four findings the writer raised against the integrity package:
judge each from the integrity code in that worktree and say what the fix is: (a) the design load's 1.15 factor is
applied twice, once in the case's payload and again in the post-run table; (b) the strength check reads member
end moments only, so the pinned-roller rail reads safety factor 9.4 where the mid-span moment gives 1.56 (does
the tier 1 party have the same blind spot, and where should the peak interior moment come from: the node
springs); (c) the post-run fastener check produces no rows; (d) safety factor and tier 1 crossings have no
window, so a start transient from straight rails counts; (8) the engine finding: at the stop, rail_y ringing at
50 Hz drives rail_x at 49.7 Hz through the one-way carried-joint coupling and rail_x stores 10.7 J where about
0.06 J is available: derive the resonant response of a one-way driven member to a decaying force at its own
frequency and say what an honest milestone 3 treatment is (a published work-done figure and an unresolved flag
when the rerouted pair's work is not small against the system's energy; the result marked not valid; full
coupling in milestone 4), and whether anything short of two-way coupling bounds it correctly; (9) determinism
tests; R-18; style.

Report, under 450 words: verdict, findings by severity as `file:line, one sentence, row`, your answers on (5),
(7a to d) and (8), owner rulings needed.
