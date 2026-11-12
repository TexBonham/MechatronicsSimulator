# Review: gantry team answers (X6, X7, X8; data, placeholders and the tests that encode them)

Reviewer: Fable (general-purpose). Read-only; diff and spec only; no builds or runs; call cap 30. Worktree
`/home/tex/dev/MechatronicsSimulator-04-x6`, branch `wip/04-x6`, diff `git -C <worktree> diff 4a606ea...cc2aae5`.
Briefs `m2-x6.md`, `m2-x7.md`; the owner's answers are transcribed in `m2-x6.md` section 1; rulings 31, 32, 35
(confirmed) in `docs/queue.md`; `docs/gantry-team-inputs.md` is the record. Rows ACC-009 to ACC-012, ACC-019 to
ACC-021, FW-014, FW-018, FW-019, FW-022, LIB-022; invariant 2.

Check: (1) every answer in the table is applied where the brief says, every unanswered item keeps its placeholder
flag, and every removed flag corresponds to an answer; (2) the case windows: each header's derivation uses the new
speeds, workspace and limits (G4's outside target, G6's 21.1 cm bound with the 2 Hz amplifier latency, G7 and G9
lifting 5 cm inside the 10 cm homing travel, G10's +1.5 s stop and 5.7 cm slip limit, G13 and G14's intervals);
any bound set from a measured value rather than derived is a high finding; (3) the NEMA 23-class placeholder
motor: the torque derivation (6.438 kg design load, c = 6.366 mm, efficiency 0.95, damping at 150 rpm, x2 pull-out
margin, 1.205 N m available against 1.10 N m needed) and that it is flagged as owed; (4) the lift as a belt: the
lead-screw instance gone, the belt ratio and `GANTRY_STEPS_PER_M_LIFT` consistent, the brake still on the lift
shaft; (5) tests updated to the new values keep their intent (no assertion weakened to pass; the lead-screw tests
now on the belt or a self-locking screw of the same travel per turn); (6) invariant 2 (nothing gantry-specific
outside `designs/`, `test_cases/`, `firmware/`, the catalog part values) and invariant 3 (YAML valid, schemas
unchanged).

Report, under 350 words: verdict, findings by severity as `file:line, one sentence, spec row`, cost note.
