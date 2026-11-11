# Review: M2-D (suites, regression, criteria, overrides) and M2-E (gantry firmware G5 to G14)

Reviewer: Fable (general-purpose). Read-only. Review from the spec rows and the diff only; never read the writer's
report or transcript. Call cap 40. Do not build or run tests (one machine build slot); read the test code and judge
whether each asserted number is derived from the spec or physics, not from the implementation.

| Brief | Worktree | Diff | Spec rows | Brief of record |
|---|---|---|---|---|
| D | `/home/tex/dev/MechatronicsSimulator-04-d` | `git -C <worktree> diff round/04-milestone-2...wip/04-d` (525fff3) | TST-008, 014, 016, 019, 027, 037 in `docs/spec/tst.md`; SCO-009 to 012, 021, 025 in `docs/spec/sco.md`; LIB-016, 017 in `docs/spec/lib.md`; ADR 0012 (D4) | `docs/briefs/round-04/m2-d.md` |
| E | `/home/tex/dev/MechatronicsSimulator-04-e` | `git -C <worktree> diff round/04-milestone-2...wip/04-e` (5bf3179) | FW-017 to 021, 024 to 028 and Open parameters in `docs/spec/fw.md`; ACC-009 to 024, 027, 028 in `docs/spec/acc.md`; ADR 0011 | `docs/briefs/round-04/m2-e.md` |

Read first: root `CLAUDE.md`, the two briefs, the rows, `mechsim/testing/CLAUDE.md`, `mechsim/scoring/CLAUDE.md`,
`firmware/CLAUDE.md`, `board_interface/mechsim_board.h`, then the diffs. The requirements doc G5 to G14 rows are in
`docs/Mechatronic Simulator — Requirements.md` (Test cases) if a firmware behaviour needs its source.

Check:
1. Every row implemented as written, cited in a test, spec edit in the same diff (R-18). Where a row says "not
   set (owed by gantry team)", the placeholder must be flagged in data or a `#ifndef` macro, never a bare literal.
2. D: suite file semantics against TST-008 and D4 (byte-identical baseline on rerun), the regression diff ordering
   (SCO-010), the report's determinism (SCO-012), the `interval` and `no_overlap` evaluation rules and their error
   cases (SCO-021, 025), the override and substitution paths (LIB-016, 017: dimension check, same-type rule,
   recorded in run metadata). D introduced `event:<name>` refs (kind = event name, detail = actuator instance); say
   whether that ref form is consistent with the recorder's event table and what the integration wave must record.
3. E: the state machine with the lift axis (homing sequence, lift workspace, trapezoid), the overload refusal and
   mid-lift stop (FW-018, 019; two-consecutive-tick filter), E-stop fault and `reset` exit to NOT_HOMED (FW-024;
   open ruling 7), heartbeat stops in FAULT (FW-025), encoder mismatch in counts (FW-020), limit trip (FW-021: the
   writer reports a wider rule than the brief, "any end switch outside its own slow zone"; judge it against the
   row and G11), lift-while-moving policy (FW-027), motion_start and motion_end events alternating per axis
   (FW-028). Firmware rules: C11, fixed-width types, includes only the board interface, braces, no ternaries. Say
   whether turning the lift driver off in NOT_HOMED is required by FW-017 or is a design choice that interacts with
   open ruling 1 (the lift hold under power loss on the bridge).
4. Invariants 1, 2, 3, 4, 8 and the style rules.
5. Ownership: files outside each brief's list and whether the edit is acceptable. Known: D did not touch
   `tests/invariant/test_schema_test_case.py` (its three failures on `suite.yaml` are the coordinator's at
   merge-back); E's channel placeholders moved from the brief's values to free channels.

Report, under 400 words: verdict per brief, findings by severity as `file:line, one sentence, spec row`, owner
rulings needed, cost note.
