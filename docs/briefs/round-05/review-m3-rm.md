# Review: M3-RM (the MuJoCo removal commit; ADR 0004 close)

Reviewer: Fable (general-purpose), fresh. Read-only; spec, plan and diff only; never the writer's report; no builds or
runs; call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-05-rm`, branch `wip/05-rm`, head de38e39 (the round head b51b28c merged in), diff
`git -C <worktree> diff b51b28c de38e39 --stat` then by file (the removal's own changes against the round head) (skip brief files, `docs/queue.md`). Plan of record
`docs/briefs/round-05/mujoco-removal.md` and the gate report's additions in `/home/tex/dev/MechatronicsSimulator/docs/
queue.md` (Milestone 3 gate report, retirement paragraph); ADR 0004; rows PHY-011, PHY-015, PHY-020 to PHY-026,
TST-040, INV-020; the owner's gate decision 1b and rulings 99 (c) and 101.

Check: (1) completeness against the plan and the additions: every item deleted, restated or kept-as-history with the
reason; `git grep -n -i mujoco` at the head: is the residue only history (ADR 0004, HISTORY files, the fixed
baselines' provenance notes, the rulings) and nothing live (code, CMake, CI, schemas, pyproject, contracts that describe
the present); (2) the enforcement that replaces the removed boundary: the import checker and the invariant tier fail on
any MuJoCo import or include anywhere, with a test that proves the check fires (a positive case), not only that it is
quiet; the CI step restated; (3) `physics.adapter`: `native` the only value; a document naming `mujoco` is refused with
an error naming the retired adapter (invariant 7), the schema enum regenerated (invariant 3), the run-identity suffix
rule (TST-040) restated consistently; (4) the comparison of record: PHY-024 and the companion test read fixed files only
(the bridge baselines and, if GV2 landed first, `native_at_retirement.yaml`; otherwise the live baseline, which the
brief told the writer to leave as is: say which it is), PHY-025 says the files are never regenerated, PHY-021 and
PHY-016 retired or restated exactly as the plan says, their stubs and markers consistent with traceability; (5) R-18:
every row restated in the same commit as the code; ADR 0004's status edit present and flagged for the owner; the
retirement table in `docs/milestones.md` records the commit, the owner's decision and what remained open (the four G10
misses explained, E9); (6) nothing of the native adapter's behaviour changed (the diff touches no native physics
source beyond removing a branch or a literal); invariants 2, 3, 5, 9; the wheel and build still install (CMake and
`pyproject.toml` consistent: no dangling target, option, dependency or rpath note).

Report, under 350 words: verdict (accept, accept with a fix round, reject), findings by severity as `file:line, one
sentence, row`, the residue list in one line, and whether the branch can land as the ADR 0004 close.
