# Review: integration wave (X1, X2, X3; A3 inside the range)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never a writer's report; no builds, no test runs,
no CLI runs (the writers' runs are on record: core 337 ctest, firmware 92, pytest 3588 passed with one foreign
failure, G1 to G9 and G11 green on the CLI, G10 one criterion failing on a core defect assigned to brief C2); call
cap 60. Worktree `/home/tex/dev/MechatronicsSimulator-04-x1`, branch `wip/04-x1`, range 58725e9..db7e1de (E's
firmware merge is the base; X1 d774e4f, X2 9030952, A3 fa102dc, X3 db7e1de). Diff:
`git -C <worktree> diff 58725e9...db7e1de`. Briefs of record `m2-x1.md`, `m2-x2.md`, `m2-a3.md`, `m2-x3.md`;
rulings 1, 6, 7, 8, 15 to 27 in `docs/queue.md` (every planned default is the rule of this review). Spec rows
ACC-009 to ACC-024 (`docs/spec/acc.md`), BLD-034, BLD-036 (`bld.md`), DES-043, DES-045 (`des.md`), SIG-020, SIG-033,
SIG-039 (`sig.md`), SCO-013, SCO-021, SCO-025 and the new `rail:` ref (`sco.md`), TST-015, 017, 018, 020 (`tst.md`),
FW-017 to FW-021, 024, 025 and the Open parameters (`fw.md`), REC-018 (`rec.md`), the requirements doc's G5 to G11
rows. Contracts in the diff: `mechsim/{design,builder,testing}/CLAUDE.md`, `core/signals/CLAUDE.md`, `firmware/CLAUDE.md`.

Check, in this order:
1. Cases G5 to G11 against their requirements rows and the ACC rows: does each criterion assert the behaviour the
   row names, with limits from the spec or a flagged placeholder, windows derived (deceleration distances from the
   firmware placeholders, relay cutoff delay, watchdog timeout)? Any criterion made to pass by construction (a
   window or band chosen from the measured value) is a high finding. The case changes X3 made (lift hanging 0.65 m,
   5 N axis friction, 3.6 kg payload instead of 4 kg): design placeholders or tuning to pass?
2. Design and schema: `senses` (payload, body, joint_reaction), `acts_on`, `event_codes` with the writer's
   per-event argument ambiguity (a refused-code argument decoded as an axis: X3's open issue), the new chip pins,
   `power_loss_hold`, `estop.manual_reset`, operator messages `lift` and `reset`, fault and command code tables
   against E's `gantry_config.h`; every placeholder in `docs/gantry-team-inputs.md`.
3. Builder and wiring: BLD-036 payload as fixed child (ruling 24) and the weld path, two actuators on one shaft
   refused at build (BLD-034), the brake on the shaft joint with A0 = 1 / (J + m c^2) (ruling 23), amplifier ADC
   data from the chip, column labels to the recorder (ruling 18), the row mapping of the new series, fault routing to
   the scheduling calls (TST-015, 017, 018, 020), event decoding to `RecordedEvent`.
4. A3's signal-layer changes (SIG-039 loss of synchronism, drive apportioning, `joint_reaction`): derivations in the
   tests, no energy injection in the reference, the apportioning rule for opposed drives (the writer used an additive
   split, not a scale, when torques oppose: judge it).
5. Scoring: `rail:` ref (ruling 27), SCO-013 if touched; the no_overlap and interval evaluations on real events.
6. Firmware edits by X2 (per-axis step scale, brake release wait, E-stop handling) against FW rows and the C11 and
   include rules.
7. Invariants 1, 2, 3, 4, 7 and style; ownership: files outside the briefs' lists and whether each edit is
   acceptable; `tests/unit/test_baseline_schema.py:85` is the coordinator's at merge-back.

Report, under 600 words: verdict for the wave (accept, accept with fixes, blocked), findings by severity as
`file:line, one sentence, spec row`, the owner rulings needed, cost note.
