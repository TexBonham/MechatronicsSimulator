# Review: M3-R (recorder series, structural scoring, sweeps and variations, viewer overlays)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never the writer's report; no builds or runs;
call cap 40. Worktree `/home/tex/dev/MechatronicsSimulator-05-r`, branch `wip/05-r`, commit f6a7926, diff `git -C
<worktree> diff 28f10c9...f6a7926`. Brief `docs/briefs/round-05/m3-r.md`; rows REC-004, 010, 026 (`docs/spec/rec.md`),
SCO-014, 015, 018, 019, 024 (`sco.md`), TST-007, 025, 026 (`tst.md`), VIE-009, 010, 011 (`vie.md`), FAC-011 and the
protocol additions (`fac.md`), ENG-030 (the end-load frame), ADR 0010 (events), ADR 0016 (rope tension); the
contracts in the diff (`mechsim/record`, `scoring`, `testing`, `viewer`, `facade` CLAUDE.md files); round 5 rulings
8, 9, 21.

Check: (1) the recorded structural series (member start and end wrenches, rope load-end and max tension) with
names and units attrs and the writer's frame assumption for ENG-030 (member frame, x along the member: state
whether the ENG-030 row should fix it); (2) the `IntegrityResults`, `MemberIntegrity` and integrity-event shapes
against INT-022 and INT-024 and whether I1 and I2 can fill them without a second schema; (3) the structural
criteria read, never recomputed (SCO-014), the tier 1 crossing failing the case (SCO-015), the settle-time
criterion's definition and its synthetic reference; (4) sweeps and variations: run ids and metadata carry the value
or seed, a variation reruns alone identically (TST-026), the side-by-side report (SCO-018, 019), the P2 pendulum
damping sweep case's derivation (expected periods per damping value from ACC-033's closed form); (5) overlays:
member stress M c / I with c = sqrt(3 I / A) when the section gives no extreme fibre (judge that fallback: it is
exact for a rectangle only; say what the section schema should carry), the swing angle from the attachment,
the cursor's fractional accumulator (VIE-009); (6) invariants 1, 2, 4, 9, style, ownership (test_cli and the
shipped-case invariant test edited to count P2), R-18.

Report, under 400 words: verdict, findings by severity as `file:line, one sentence, spec row`, owner rulings
needed, cost note.
