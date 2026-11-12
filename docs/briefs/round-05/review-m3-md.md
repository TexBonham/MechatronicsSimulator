# Review: M3-M (materials) and M3-D (structural data)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never a writer's report; no builds or runs; call
cap 40.

| Brief | Worktree | Diff | Spec rows | Brief |
|---|---|---|---|---|
| M | `/home/tex/dev/MechatronicsSimulator-05-m` | `git -C <worktree> diff becf97e...wip/05-m` | MAT-002, 003, 006 to 012, 014 to 017, 020 to 024 in `docs/spec/mat.md`; INT-005, INT-008 (the consumer) | `docs/briefs/round-05/m3-m.md` |
| D | `/home/tex/dev/MechatronicsSimulator-05-d` | `git -C <worktree> diff becf97e...wip/05-d` (d3f520e) | DES-031 to 034 in `des.md`; LIB-020, 030, 032 in `lib.md`; BLD-008, 009, 014 to 019, 031, 032 in `bld.md`; ENG-022, 038 to 041 (the closed forms); ADR 0016 (rope element) | `docs/briefs/round-05/m3-d.md` |

Read first: root `CLAUDE.md`, both briefs, round 5 rulings 2, 6, 7, 16 to 18 in `docs/queue.md`, the rows, the
contracts in the diffs (`mechsim/materials/CLAUDE.md`, `mechsim/design/CLAUDE.md`, `mechsim/library/CLAUDE.md`,
`mechsim/builder/CLAUDE.md`), ADR 0016.

Check:
1. M: every lookup rule as the rows state (temperature interpolation and out-of-range errors, min/typ/max, S-N
   log-log interpolation and range errors, process validation bounds), the printed strength-axes placeholder (the
   writer implemented f_inter cos^2 theta + f_in sin^2 theta, correcting the brief's formula: judge which is right
   for theta measured from the build direction, with interlayer the weak one along the build direction), the
   overrides file and `MaterialSet` precedence, the shipped PLA, PETG and ABS spreads (wide, conservative, flagged),
   `IntegrityMaterialView` as the API integrity needs (INT-005, INT-008); tests derived from the data, not the code.
2. D: DES-031 to 034 fields and validation; the catalog types' parameters against LIB-030 and ADR 0016; the lumped
   beam build (BLD-014) and the four closed-form checks (BLD-015 to 017) derived in the tests with the proposed 2
   percent tolerance at eight segments (judge the proposal against the chain's known convergence: ENG-038 to 041 are
   spike references at what tolerance?); the calibration scheme (E scaled by the deflection ratio, density by the
   squared frequency ratio: exact for a single member, but say what happens when both targets are given and the
   section is wrong); fasteners as addressable constraints; the rope element shape against ADR 0016; the gantry
   changes limited to the payload rope declaration with the suite still on rigid members.
3. Invariants 2, 3 (schemas regenerated for design, component, physics model), 4 and style; ownership (D edited
   `test_catalog_registry.py` outside its list; M moved an example out of `schemas/examples`); R-18.
4. Cross-brief: D's `materials.overrides` only checks existence until M merges; the builder still reads
   `shipped_materials()`; `mechsim/physics/CLAUDE.md` lacks Rope, Fastening and mounts; the clamped-clamped rail
   (ruling 16). Say what the merge-back and brief N must do.

Report, under 450 words: verdict per brief, findings by severity as `file:line, one sentence, spec row`, owner
rulings needed, cost note.
