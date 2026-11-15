# Review: M3-X2i (G10's remainder from both archives; the E9 release artefact bounded and published)

Reviewer: Fable (general-purpose), fresh. Read-only; spec and diff only; never the writer's report; no builds or
runs; call cap 25. Worktree `/home/tex/dev/MechatronicsSimulator-05-x2i`, branch `wip/05-x2i`, head 5a0aef9, diff
`git -C <worktree> diff 708a734 5a0aef9` (skip brief files, `docs/queue.md`). Brief `docs/briefs/round-05/m3-x2i.md`;
rulings 38, 39, 47, 90, 94, 96 and the owner's gate decision 1b in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`;
rows ENG-044 (or the transmission row the writer edited), PHY-024, SIG-033, the firmware's encoder-mismatch rows.

Check: (1) G10's accounting: the firmware's mismatch rebuilt from each archive as the firmware computes it (step
counts against encoder counts from the homing origin, the 2 mm threshold): is the rebuild faithful to `gantry.c`
(detection tick, stop phase, fault at zero commanded velocity); each term (belt-stretch crossing shift 2 and 5 ms,
stop-phase difference, coast during the stall) with its number; the sum against 8 and 20 ms within 30 percent, or
the finding stated honestly with what is and is not accounted for; the companion test's causes, the README and the
milestones row say the same thing; nothing in `G10.yaml` changed. (2) E9's derivation in the code header: the frozen
damping c0 = loss / threshold, the excess ratio w / threshold for a release from rest, the bound on the released
rotor's lag as a function of threshold, step and acceleration, the G7 numbers (79 N m s/rad, about 2 ms): derive it
yourself and check; the approximation line: fires when the frozen damping would remove more than the loss by the
stated factor (the writer proposes a factor; is the derivation for it sound, is the line per transmission per run
with the worst ratio and time, does a steadily turning shaft publish nothing); the switch's physics unchanged (bit-
identical results: the baselines untouched and no value moved); the milestone 4 item (loss at the end rate by one
re-solve) written as the row's open item with its derivation. (3) Tests: the released-shaft unit test asserts the
predicted ratio, not a measured one; G7 on native shows the line for the lift belt. (4) R-18, invariants 2, 4, 5, 7;
no gantry value in code; the sanitize run covers the touched tests.

Report, under 300 words: verdict (accept, accept with a fix round, reject), findings by severity as `file:line, one
sentence, row`, your reading of G10 for the owner in one sentence (explained or finding, with the numbers), and
whether the bridge can be retired on this evidence (the owner's decision 1b conditions retirement on this brief).
