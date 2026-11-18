# Round 5 plan: milestone 3

Signed off by the owner 2026-11-11 ("Go"), with the amendment that milestone 3 replaces MuJoCo with the own C++
engine (ADR 0001, ADR 0004 retirement criteria) rather than only bringing it online. Branch `round/05-milestone-3`
from main at cf5c74f (round 4 merged). Rulings and clarifications are retained in `docs/queue.md` (Round 5 rulings)
and answered by the owner as available; the planned default applies until then.

## Goal

Ropes, flexible members and structural checks on the own engine; gantry G12, G15, G16; the acceptance-criteria
references (ACC-036, 037, 039 to 043); and MuJoCo retired: every case (G1 to G16, P1) runs on the native adapter,
the bridge comparison recordings are kept as fixed YAML baselines (PHY-025), the retirement check passes (PHY-023,
PHY-024), MuJoCo leaves the runtime, the wheel and CI (PHY-026). Fallback per ADR 0004 only if the own engine's
contact solver or the differential check does not hold by the gate: the bridge stays for the rigid cases one more
round, recorded as such; the structural cases still ship on the own engine.

## Scope (93 M3 rows plus the round 4 carry list)

Engine: ENG-016 to 021, 030 to 035, 042, 047, ENG-043 stiffness. Native adapter: PHY-020, 004, 007, 008, 021 to
026. Materials: MAT-002, 003, 006 to 012, 014 to 017, 021, 024. Structural data: DES-031 to 034, LIB-020, 030,
032, BLD-008, 009, 014 to 019, 031, 032. Integrity: INT-001 to 012, 019 to 027, 036, 037, FAC-013. Recorder,
scoring, runner, viewer: REC-004, 010, 026, SCO-014, 015, 018, 019, 024, TST-007, 025, 026, VIE-009 fraction, 010,
011. Acceptance: ACC-025, 026, 029 to 032, 036, 037, 039 to 043, ACC-038. Carry-overs: belt compliance (ruling 4-35),
LIB-017 end to end, the case windows as open parameters, LIB-009. Deferred to M4: the playwright client test, I2C
and SPI, SIG-031.

## Briefs

Rules as round 4: one build at a time through `tools/locked_build.sh` at two jobs, `debug` to iterate and
`debug-sanitize` at landing, worktrees `../MechatronicsSimulator-05-<letter>` on `wip/05-<letter>`, no
attribution trailers, caps in tool calls, every brief carries derivations and physics-derived bounds.

| Brief | Agent | Owns | Spec IDs | Cap | Cost |
|---|---|---|---|---|---|
| A rope ADR 0016, ADR 0010 amendments | docs writer, Opus | `docs/adr/0016-rope-model.md`, ADR 0010 open items, Decisions log rows, ADR README, rope spec rows | ENG-047, BLD-032, PHY-004, REC-026, P4 | 40 | 100K-140K |
| E1 engine collision and contact | module-writer, Opus | `core/engine/` collision and contact files, provider contact integration, `tests/reference` oracle | ENG-016 to 021, 035 | 150 | 250K-320K |
| E2 engine members, ropes, mid-run changes, belt compliance | module-writer, Opus | `core/engine/` lumped beam, rope, transmission stiffness, member loads, mid-run API | ENG-030 to 034, 042, 047, 043 | 150 | 250K-320K |
| M materials | module-writer, Opus | `mechsim/materials/`, `schemas/material.json`, shipped data, overrides file | MAT-002, 003, 006 to 012, 014 to 017, 021, 024 | 150 | 200K-250K |
| D structural data | module-writer, Opus | `mechsim/design/` structural rows, `mechsim/library/` flexible beam, hook, rope, `mechsim/builder/` member list, lumped beams, calibration, fasteners, rope chain | DES-031 to 034, LIB-020, 030, 032, BLD-008, 009, 014 to 019, 031, 032 | 150 | 250K-300K |
| N native adapter (after E1, E2) | module-writer, Opus | `core/physics/native/`, `mechsim/physics/native/`, bindings | PHY-020, 004, 007, 008, 022, 034 on the own engine, 006, ruling 4-26 | 150, handoff to N2 | 300K-400K |
| I1 integrity checks (after M, D) | module-writer, Opus | `mechsim/integrity/` plugins, configuration, per-member table, load-case export | INT-001 to 012, 019 to 022, 036, 037, FAC-013 | 150 | 250K-320K |
| R recorder, scoring, runner, viewer | module-writer, Opus | `mechsim/record/`, `mechsim/scoring/`, `mechsim/testing/` sweeps and variations, `mechsim/viewer/` | REC-004, 010, 026, SCO-014, 015, 018, 019, 024, TST-007, 025, 026, VIE-009, 010, 011 | 150 | 250K-300K |
| I2 tier 1 party and tier 2 changes (after N, I1) | module-writer, Opus | new `core/integrity/`, `core/kernel/` slot, adapter mid-run calls | INT-023 to 027, ENG-031 to 034 through PHY-008 | 150 | 250K-300K |
| X1 integration: flexible rails and rope on the native adapter, G15, G12 | module-writer, Opus | `designs/gantry/`, `test_cases/gantry/`, cross-package fixes | ACC-025, 026, 029, 030, 037, 039 to 043 | 150 | 300K-350K |
| X2 retirement: full suite on the native adapter, G16, batch, MuJoCo out | module-writer, Opus | as X1 plus CI, wheel, `docs/milestones.md`, ADR 0004 close | ACC-031, 032, 036, PHY-021, 023 to 026, LIB-017, ACC-038 | 150 | 300K-350K |

Waves: 0: A (owner reviews the ADR before E2 starts the rope); 1: E1, E2 (C++, share the lock), M, D (Python);
2: N (C++), I1, R (Python); 3: I2 (C++), X1 case authoring; 4: X1 then X2; gate: reference-validator on both adapters,
Opus audit, Fable gate review, owner review and the retirement decision. Reviews per stage, paired: E1 with E2, M with
D, N alone, I1 with R, I2 alone, the X wave grouped. Owner-reviewed commits: ADR 0016, the N contract, the gate.

Cost: 6.0M to 6.8M as listed plus a 1.5M integration contingency; budget 8.0M. Usage reserve W-04: owner supplies.

## Assumptions (as posted, with the owner's amendment)

1. Milestone 3 retires MuJoCo; the fallback above is the only way the bridge survives the round.
2. Rope (ADR 0016 proposal): a lumped chain of point masses joined by unilateral tension springs with viscous
   damping, slack when shorter than rest, length change by a driven end (ENG-047); the gantry's payload hangs on one
   rope from the hook, replacing the spherical-joint stand-in; the rigid-link pendulum stays the M1 reference and a
   rope pendulum joins it (ACC-037).
3. Tier 2 split: milestone 3 takes yield (permanent spring offset) and rope snap (load release); milestone 4 takes
   buckling collapse and fastener break.
4. Halt on a tier 1 crossing is a test-case field, default false (the case fails either way).
5. The tier 1 integrity party is C++ in a new `core/integrity/` package behind the kernel's slot; the tier 2 checks
   are Python plugins in `mechsim/integrity/` over the recording.
6. Gantry structural placeholders until the team rules: PETG printed rails 0.5 m with a placeholder rectangular
   section, static deflection target 1 cm with dynamic reported, minimum safety factor 1.5, swing limit 0.1 rad and
   settle time 5 s for G12, four segments per rail.
7. Materials: shipped printed materials carry conservative defaults with wide spreads; temperature dependence is
   tabulated and lookups outside the range are errors; overrides are a separate data file the design references;
   the strength-axes model for printed parts is in-layer across the build direction and interlayer along it, with a
   cosine-squared blend between, flagged until the owner rules.
8. G16 sweeps the two shipped stepper parts through the sweep machinery; LIB-017 end to end comes with it.
9. Every milestone 2 case keeps passing on the bridge until X2 switches the suite to the native adapter in one commit.
10. Belt compliance lands in the own engine; the bridge keeps rigid belts with its recorded approximation.
11. Models and reviewers as round 4; the docs writer is Opus; the gate audit is Opus.
12. Windows CI stays; a native-adapter test that MSVC blocks is Linux-only and recorded as such.
13. Differential tolerance (PHY-021) proposed: 1e-6 relative on poses and velocities for drive-free references, one
    step angle times the ratio per tick of commanded speed for driven cases; confirmed by the owner at the gate.
