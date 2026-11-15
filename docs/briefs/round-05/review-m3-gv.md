# Review: M3-GV (real components for the gantry team's open slots)

Reviewer: Fable (general-purpose), fresh. Read-only; data files, sources and diff only; never the writer's report;
no builds or runs; call cap 30; `WebFetch` allowed to spot-check at most five cited sources. Worktree
`/home/tex/dev/MechatronicsSimulator-05-gv`, branch `wip/05-gv`, head GV_HEAD, diff `git -C <worktree> diff e97eeb0
GV_HEAD` (skip brief files, `docs/queue.md`). Brief `docs/briefs/round-05/m3-gv.md`; ruling 98 and the owner's gate
decision 4 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`; `docs/gantry-team-inputs.md` as rewritten; rows
LIB-016, LIB-017, LIB-020, LIB-034, BLD-019, BLD-041, DES-047, ACC-026, ACC-029, ACC-030, ACC-038, TST-044.

The rules this brief ran under: data only, no code; every value sourced with URL and figure or left a flagged
placeholder; no requirement value changed (SF 1.5, 1 cm, 0.1 rad, 5 s, 10 lb, 5.216 kg); no tuning (the evaluation
start and G6's bound change only by the README's derivation from the new data); generic catalog parts keep their
values; bridge baselines untouched; `baseline.yaml` regenerated once.

Check: (1) per slot: the chosen part or value, its source, whether the figure in the file is the figure in the
source (spot-check five), its unit and dimension (invariant 3: SI inside, units explicit); the 2040 extrusion's
section properties and the per-rail calibration computed by the closed form for that section and end condition
(derive BLD-019's deflection and first frequency yourself for one rail); the stepper curves anchored at the holding
torque (LIB-034) and read at the catalog's speed points; the belt stiffness per length and the pulley pitch diameter;
the cable's EA, breaking strength and mass per length; the cell and amplifier's counts per kg arithmetic. (2) The
evaluation start re-derived by the README's method from rail_x's new damping and frequency, derived once; G6's bound
re-derived with the real filter (ruling 35): are both derivations, not fits, and are the edits to `G6.yaml` and the
start fields exactly what the derivations give. (3) The baseline comparison: every status change and value move
listed with cause; expected failures converted only for a stated physical reason (is the reason physical, and is the
test's assertion the spec's quantity); the ones that still fail named honestly (G12's settle without anti-sway,
G15's placeholder-free safety factor and validity, ACC-038). (4) `docs/gantry-team-inputs.md`: every row a value, a
source and a "replaceable" flag, nothing presented as the team's; `docs/milestones.md` rows. (5) Invariants 2 and 3;
nothing gantry-specific crept into code; schemas unchanged (if a schema field was needed it is a finding, not an
edit).

Report, under 400 words: verdict (accept, accept with a fix round, reject), findings by severity as `file:line, one
sentence, row`, the spot-check results, and for the owner: which results of record changed because the data is now
real, in one line each.
