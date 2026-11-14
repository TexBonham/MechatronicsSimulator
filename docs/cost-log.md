# Session Cost Log

One row per main task and one row per agent run. Agent tokens come from the task notification. Inline estimates are
from file reads. Archived rounds move to `docs/cost-log-archive.md` at round close. Reference per-run sizes by agent
type live in the root `CLAUDE.md` dispatch table once measured.

| Date | Task | Mode | Tokens (est.) | Notes |
|------|------|------|:-------------:|-------|
| 2026-11-05 | Round 3 close docs pass: 16 contract files swept, 59 index lines corrected, two HISTORY.md files | docs-indexer (haiku) | 97K | 11 calls, 3 min |
| 2026-11-05 | Round 3 close: rulings applied to spec rows, cost-log archive, queue, root dispatch table, PR | inline | ~30K | coordinator |
| 2026-11-11 | Round 4 close: cost rows archived (60 rows, ~6.33M), queue and plan status, worktrees removed; gate commit candidate prepared for the owner | inline | ~20K | coordinator |
| 2026-11-11 | M2-X6: gantry team answers applied (workspace, rated load, PETG, belts, speeds, homing travel, NUCLEO part); 11 of 13 cases pass, G5 and G6 blocked by the placeholder motor (wip/04-x6 3b5e913) | module-writer (opus), worktree | ~217K | 87 calls, 41 min |
| 2026-11-11 | M2-X7: NEMA 23-class placeholder motor, coupled tests to the new gantry values, G10 timing derived; 12 of 13 pass, G6 on load-cell overshoot (wip/04-x6 1af4dcc) | module-writer (opus), fresh in the X6 worktree | ~238K | 121 calls, 50 min |
| 2026-11-11 | M2-X8: G6 with the 2 Hz amplifier and the derived 21.1 cm bound; suite 13 of 13 (wip/04-x6 cc2aae5) | module-writer (opus), resumed | ~18K | 16 calls |
| 2026-11-11 | M2 review: gantry answers (X6 to X8) | general-purpose (fable) | ~75K | 9 calls; pass, 3 medium (header timing wording, inputs row), 6 low |
| 2026-11-11 | Invariant audit: gantry answers | invariant-auditor (sonnet) | 39K | 5 calls; clean, 2 minor (generic part carried a gantry value; derivation in a part note) |
| 2026-11-11 | Round 5 plan: milestone 3 scope from 93 M3 rows and the carry list, briefs, waves, assumptions; branch cut from main | inline | ~60K | coordinator; 6 calls of spec and contract reads |
| 2026-11-11 | M3-A: ADR 0016 rope model, ADR 0010 open items closed, Decisions log rows 48 to 50 (wip/05-a 81194c9) | general-purpose (opus) | 99K | 21 calls, 6 min |
| 2026-11-11 | M3-M: material lookups, process-aware strengths, S-N, overrides file, integrity view; 11 MAT stubs converted (wip/05-m) | module-writer (opus), worktree | ~148K | 43 calls, 19 min |
| 2026-11-11 | M3-E1: engine collision (GJK and EPA, primitives, hulls) and implicit soft contact with friction, payload attach and release; 383 ctest (wip/05-e1 2e92c0d) | module-writer (opus), worktree | est. 280K | pending usage |
| 2026-11-11 | M3-D: structural design rows, flexible beam, hook and rope types, member list, lumped beams with calibration, fasteners, rope element (wip/05-d d3f520e) | module-writer (opus), worktree | ~251K | 104 calls, 30 min |
| 2026-11-11 | Invariant audit: M and D | invariant-auditor (sonnet) | ~35K | clean |
| 2026-11-11 | M3 review: M and D | general-purpose (fable) | ~95K | 20 calls; M accept, D accept with fixes (2 medium: segment defaults; 5 low; rulings 19, 20) |
| 2026-11-11 | M3-E2: member end-loads, mid-run changes, Euler buckling, rope chain on translation joints, belt compliance; 389 ctest (wip/05-e2 a3638a4) | module-writer (opus), worktree | est. 300K | pending usage; backward-Euler swing damping finding (ruling 21) |
| 2026-11-11 | Invariant audit: E1 and E2 | invariant-auditor (sonnet) | ~40K | clean; minors: GJK/EPA cap path, ENG-019 wording, Attachment name clash at merge |
| 2026-11-11 | M3 review: E1 and E2 (physics; integrator derivation) | general-purpose (fable) | ~95K | 17 calls; both accept with follow-ups, ruling 21 settled against the midpoint proposal |
| 2026-11-11 | M3-D fix round: segment defaults 8, optional breaking tension, calibration bounds and notes, materials wiring, overrides file kind, DES-046; M merged in (wip/05-d fbcd19a) | module-writer (opus), resumed | ~75K | 59 calls |
| 2026-11-11 | M3-E2 fix round: friction floor, snatch wording, T4 rows, scheme_decay outputs, one release_load path (wip/05-e2 ca0cd72) | module-writer (opus), resumed | ~45K | est. |
| 2026-11-11 | M3-E1 fix round: GJK and EPA cap errors with CollisionLimits, ENG-019 oracle wording, witness-point note, weld note, WeldAttachment and model-change events (wip/05-e1 5f8878d) | module-writer (opus), resumed | ~30K | est. |
| 2026-11-11 | M3-R: structural series, integrity result and event shapes, structural criteria, settle time, sweeps and variations, overlays, cursor fraction (wip/05-r f6a7926) | module-writer (opus), worktree | ~283K | 119 calls |
| 2026-11-11 | Invariant audit: R | invariant-auditor (sonnet) | ~30K | clean |
| 2026-11-11 | M3 review: R | general-purpose (fable) | ~65K | 18 calls; accept with 1 major (end-load frame, two-axis sections), 5 minor; rulings 24 to 26 |
| 2026-11-11 | M3-R fix round: member frame, two-axis sections, tier filter, rope names, rope integrity table; round branch merged in (wip/05-r 6c8001b) | module-writer (opus), resumed | ~45K | 28 calls |
| 2026-11-11 | M3-R re-review after the fix round: accept, two carry notes (N brief, queue) | Fable review (general-purpose), resumed | 151K | 6 calls |
| 2026-11-11 | M3-I1 integrity plugins and checks, FAC-013 (wip/05-i1 2b4957a) | module-writer (opus) | 260K | 95 calls; fix round follows (merge R, rope table, wiring) |
| 2026-11-11 | M3-I1 fix round: merge R, two-axis stress, rope table, core_services wiring, integrity file kind (wip/05-i1 d26c5f8) | module-writer (opus), resumed | ~39K | 31 calls; full suite 4234 passed, 144 xfail |
| 2026-11-11 | M3-I1 invariant audit (wip/05-i1 d26c5f8): clean, two minors | invariant-auditor (sonnet) | ~25K | 4 calls |
| 2026-11-11 | M3 review: I1 (wip/05-i1 d26c5f8): accept with fixes, 2 medium, 5 low; rulings 28 to 30 | Fable review (general-purpose) | ~110K | 28 calls |
| 2026-11-11 | M3-I1 fix round 2: deflection row, protocol None, impact flag, member axis (wip/05-i1 e59b5f2); FAC-013 conformance stub stays (flexible bridge member diverges) | module-writer (opus), resumed | ~18K | 14 calls; full suite 4236 passed, 144 xfail |
| 2026-11-11 | M3-I1 re-review after fix round 2: accept; I1 merged to the round branch | Fable review (general-purpose), resumed | ~12K | 3 calls |
| 2026-11-11 | M3-E3 engine merge of E1 and E2, bindings, clamped far end (wip/05-e3 ea37b2d) | module-writer (opus) | 228K | 111 calls; stalled 45 min on lost background tasks, resumed |
| 2026-11-11 | M3 review: E3 (wip/05-e3 ea37b2d): accept with fixes, 3 medium, 6 low; rulings 31 to 33 | Fable review (general-purpose) | ~120K | 23 calls |
| 2026-11-11 | M3-E3 invariant audit: clean | invariant-auditor (sonnet) | ~20K | 4 calls |
| 2026-11-11 | M3-E3 fix round: clamped floor, rope end mass, carrier check, binding test edges (wip/05-e3 1fc9750) | module-writer (opus), resumed | ~25K | 17 calls; ctest debug-sanitize 426 |
| 2026-11-11 | M3-E3 re-review after the fix round: accept, three lows carried to brief N | Fable review (general-purpose), resumed | ~45K | 6 calls |
| 2026-11-12 | M3-N native adapter: G3 and P1 on the own engine, reference tier on native (wip/05-n 530e55b); stopped at the cap, N2 follows | module-writer (opus) | 346K | 150 calls |
| 2026-11-12 | M3 review: N (wip/05-n 530e55b): fix round before the stage commit, 2 high, 5 medium, 3 low; rulings 37 to 41 | Fable review (general-purpose) | 177K | 24 calls |
| 2026-11-12 | M3-N invariant audit: one minor (adapter not in archive metadata) | invariant-auditor (sonnet) | 23K | 4 calls |
| 2026-11-12 | M3-N2 native adapter fix round: adapter field verified, items 2 to 6 coded and unverified at the cap (wip/05-n 30c9536) | module-writer (opus) | 364K | 198 calls; cap overrun, queued behind the E4 build |
| 2026-11-12 | M3-E4 engine joint limits, friction, per-step drive (wip/05-e4 c072dee) | module-writer (opus) | 218K | 74 calls; ctest debug-sanitize 469 |
| 2026-11-12 | M3 review: E4 (wip/05-e4 c072dee): changes requested, 2 major, 5 minor; rulings 42 to 44 | Fable review (general-purpose) | ~85K | 15 calls |
| 2026-11-12 | M3-E4 invariant audit: clean | invariant-auditor (sonnet) | 28K | 7 calls |
| 2026-11-12 | M3-N2b verify and finish the adapter fix round (wip/05-n 0984229): beams within 2 percent, G3 and P1 on native with flexible rails | module-writer (opus) | 119K | 73 calls; ctest debug-sanitize 454 |
| 2026-11-12 | M3-N re-review after N2 and N2b: accept as a stage commit; rulings 45 and 46; findings carried to N3 | Fable review (general-purpose), resumed | ~76K | 12 calls |
| 2026-11-12 | M3-N2 and N2b invariant audit: clean, kernel spec row owed | invariant-auditor (sonnet) | 23K | 4 calls |
| 2026-11-12 | M3-E4 fix round: additive drive, anticipatory limit row, driven-only decay with the two-DOF root (wip/05-e4 0f38738) | module-writer (opus), resumed | ~60K | 26 calls; ctest debug-sanitize 478 |
| 2026-11-12 | M3-E4 re-review: accept; zero-stretch coupling defect in the E2 transmission code found, fixed in a short E4 follow-up | Fable review (general-purpose), resumed | ~35K | 6 calls |
| 2026-11-12 | M3-E4 follow-up: zero-stretch coupling, start on a bound (wip/05-e4 f6b9729); confirmed by the reviewer; E4 merged | module-writer (opus) and Fable review, both resumed | ~25K | 9 + 3 calls; ctest debug-sanitize 482 |
| 2026-11-12 | M3-N3 adapter on the engine's limits, friction and drive (wip/05-n3 8e4d3f4); pose series naming and facade `sets` left for a follow-up | module-writer (opus) | 317K | 157 calls, over the 130 cap; ctest debug-sanitize 499 |
| 2026-11-12 | M3 review: N3 (wip/05-n3 8e4d3f4): pass with a fix round, 3 major, 3 minor; rulings 47 to 49; engine follow-up E5 (review-m3-n3.md) | Fable review (general-purpose) | 128K | 19 calls |
| 2026-11-12 | M3-N3 invariant audit: clean | invariant-auditor (sonnet) | 30K | 6 calls |
| 2026-11-12 | M3-I2 tier 1 integrity party, tier 2 yield and rope snap (wip/05-i2 81b2943) | module-writer (opus), resumed once after the restart | 342K | 149 calls; ctest debug-sanitize 470 |
| 2026-11-12 | M3 review: I2 (wip/05-i2 81b2943): approve after one fix round, 5 major, 6 minor; rulings 50 to 54 (review-m3-i2.md) | Fable review (general-purpose) | ~130K | 13 calls |
| 2026-11-12 | M3-I2 invariant audit: clean | invariant-auditor (sonnet) | 25K | 5 calls |
| 2026-11-12 | M3-E5 regularised efficiency switch, screw lock row (wip/05-e5 8beefd2) | module-writer (opus) | 220K | 94 calls; ctest debug-sanitize 509 |
| 2026-11-12 | M3 review: E5 (wip/05-e5 8beefd2): accept after one small fix round, 2 major, 2 minor; ruling 55 (review-m3-e5.md) | Fable review (general-purpose) | 102K | 9 calls |
| 2026-11-12 | M3-E5 invariant audit: clean | invariant-auditor (sonnet) | 30K | 5 calls |
| 2026-11-12 | M3-I2b integrity fix round, facade run sets (wip/05-i2 0070358) | module-writer (opus) | 258K | 166 calls, over the 110 cap; ctest debug-sanitize 526 |
| 2026-11-12 | M3-I2 re-review after I2b: accept; 3 minor items carried to X1 | Fable review (general-purpose), resumed | ~20K | 5 calls |
| 2026-11-12 | M3-E5 fix round (wip/05-e5 903a63d) and re-review: accept | module-writer (opus) and Fable review, both resumed | ~45K | 30 + 3 calls |
| 2026-11-12 | M3-N4 adapter follow-up on the E5 engine (wip/05-n4 8f5ae5c) | module-writer (opus) | 249K | 120 calls |
| 2026-11-12 | M3 review: N4 (wip/05-n4 8f5ae5c): accept after a short resume, 2 major, 3 minor | Fable review (general-purpose), resumed | ~40K | 6 calls |
| 2026-11-12 | M3-N4 review fixes (wip/05-n4 854c012) and confirmation: accept | module-writer (opus) and Fable review, both resumed | ~50K | 19 + 4 calls |
| 2026-11-12 | M3-X1b structural acceptance on native, node-pose deflection (wip/05-x1b 6c68248) | module-writer (opus) | 332K | 160 calls, over the 140 cap |
| 2026-11-12 | M3 review: X1b (wip/05-x1b 6c68248): land after a small fix round, 7 medium, 4 low; rulings 57 to 61 (review-m3-x1b.md) | Fable review (general-purpose) | 123K | 16 calls |
| 2026-11-12 | M3-X1b invariant audit: pass, one data note | invariant-auditor (sonnet) | 40K | 7 calls |
| 2026-11-12 | M3-X1c acceptance fix round (wip/05-x1b 1b0c3f8) | module-writer (opus) | 243K | 113 calls |
| 2026-11-12 | M3-X1b re-review after X1c: accept; ruling 62 | Fable review (general-purpose), resumed | ~25K | 6 calls |
| 2026-11-13 | M3-X1a gantry G12 and G15 on native (wip/05-x1a fe7a386): stopped at the cap, not green; findings in rulings 63, 64 and briefs I3, E6 | module-writer (opus) | 316K | 164 calls |
| 2026-11-13 | M3-I3 integrity and scoring fixes from the gantry cases (wip/05-i3 eef3732) | module-writer (opus) | 270K | 159 calls, over the 110 cap |
| 2026-11-13 | M3 review: I3 (wip/05-i3 eef3732): fix round needed, 1 blocker (swing angle reads displacements), 2 major, 4 minor; rail_x zero deflection traced to the chain's single bending plane | Fable review (general-purpose), resumed | ~45K | 9 calls |
| 2026-11-13 | M3-I3 invariant audit: pass, 2 minor | invariant-auditor (sonnet) | 48K | 4 calls |
| 2026-11-13 | M3-E6 lift jump on a stop at a limit: adapter spring-law blend per substep, engine efficiency loss as implicit damping (wip/05-e6 715dab3) | module-writer (opus) | 211K | 97 calls |
| 2026-11-13 | M3 review: E6 (wip/05-e6 715dab3): accept, 4 minor; ruling 66 amends 47 | Fable review (general-purpose), resumed | ~50K | 7 calls |
| 2026-11-13 | M3-I3b swing angle and runner fixes (wip/05-i3 87621f6) | module-writer (opus) | 183K | 119 calls, over the 70 cap |
| 2026-11-13 | M3-I3 confirmation after I3b: accept, 2 low items to X1d | Fable review (general-purpose), resumed | ~15K | 4 calls |
| 2026-11-13 | M3-E7 moving load on a flexible member, load-path coupling (wip/05-e7 5ab0704) | module-writer (opus), resumed once after an API error | 351K | about 150 calls |
| 2026-11-13 | M3 review: E7 (wip/05-e7 5ab0704): accept after a small fix round, 2 major, 3 minor; ruling 67 | Fable review (general-purpose), resumed (cut once by an API error) | ~70K | 9 calls |
| 2026-11-13 | M3-E7 invariant audit: clean | invariant-auditor (sonnet) | 34K | 6 calls |
| 2026-11-13 | M3-E7b carried-load stability: derived boundary, refusal, three rigs (wip/05-e7 98891e2) | module-writer (opus) | 207K | 85 calls, over the 70 cap |
| 2026-11-13 | M3-E7b confirmation: accept; ruling 68 | Fable review (general-purpose), resumed | ~20K | 3 calls |
| 2026-11-13 | M3-X1d gantry G12 and G15 on native with carried rails (wip/05-x1a 3775c76): ACC-025 converted; G15 findings | module-writer (opus) | 280K | 158 calls |
| 2026-11-13 | M3 review: X1a and X1d gantry work (wip/05-x1a d456cfa): land after a fix round, 5 medium, 3 low; integrity findings confirmed; ruling 70 (review-m3-x1ad.md) | Fable review (general-purpose), resumed | ~90K | 13 calls |
| 2026-11-13 | M3-E7c carried-load validity: pair work, dynamic ratio, unresolved flag (wip/05-e7 1f8e8b2) | module-writer (opus) | 204K | 79 calls |
| 2026-11-13 | M3-I4 integrity defects from G15 and the gantry review's fixes (wip/05-x1a 4bd6609) | module-writer (opus) | 324K | 145 calls |
| 2026-11-13 | M3-I4b moment under a carried load, yielded spring (wip/05-x1a da8c5bd); ACC-025 a-priori bound not reached | module-writer (opus) | 259K | 101 calls, over the 80 cap |
| 2026-11-13 | M3 review: I4 and I4b (wip/05-x1a da8c5bd): accept with follow-ups, 4 medium, 4 minor; ruling 72 | Fable review (general-purpose), resumed | ~60K | 9 calls |
| 2026-11-13 | M3-E7d carried mass on the member's inertia (wip/05-e7 63cd21b): G15's rail pumping gone (16.8 to 1.32 mm) | module-writer (opus) | 298K | 116 calls |
| 2026-11-13 | M3 review: E7c with E7d (wip/05-e7 63cd21b): accept both, 2 major, 4 minor; ruling 73 | Fable review (general-purpose), resumed | ~60K | 7 calls |
| 2026-11-13 | M3-E7e validity base window, momentum form for the moving carried mass (wip/05-e7 b7221c9); gantry rails now flag during the move, undiagnosed | module-writer (opus) | 265K | 94 calls |
| 2026-11-13 | M3 review: E7e (wip/05-e7 b7221c9): momentum form accepted, flag definition to change, 2 major, 1 minor; ruling 74 | Fable review (general-purpose), resumed | ~40K | 5 calls |
| 2026-11-13 | M3-I5 integrity follow-ups, not-valid results (wip/05-x1a a0d37b9) | module-writer (opus) | 305K | 118 calls |
| 2026-11-13 | M3 review: I5 (wip/05-x1a a0d37b9): one fix round, 4 major, 4 medium; exact rule for a carried couple; ruling 75 (review-m3-i5.md) | Fable review (general-purpose) | 183K | 13 calls |
| 2026-11-13 | M3-E7f validity flag relative to the static shape, physical-time window, evaluation start; G15 diagnosis (wip/05-e7 707ec10, with the ruling 76 follow-up) | module-writer (opus), resumed once | 229K + follow-up | about 140 calls |
| 2026-11-13 | M3 review: E7f (wip/05-e7 707ec10): accepted as implementing rulings 74 and 76, 3 major, 7 minor; ruling 77 (review-m3-e7f.md) | Fable review (general-purpose) | 111K | 13 calls |
| 2026-11-13 | M3-I5b exact carried moment in both tiers, not-valid neither halts nor fails, engine-set flag end to end (wip/05-x1a f4977ea); fatigue and two tests left to I5c | module-writer (opus) | 333K | 137 calls |
| 2026-11-13 | M3-E7g member-wide validity flag (wip/05-e7 92793ba); follow-up: evaluation start gates rope snap | module-writer (opus), resumed once | 213K + follow-up | 85 calls + follow-up |
| 2026-11-13 | M3 review: E7g (wip/05-e7 92793ba): accept, 3 minor open; remedy for the gantry's undamped rails; ruling 78 | Fable review (general-purpose), resumed | ~55K | 7 calls |
| 2026-11-13 | M3-I5c fatigue by damage, test repairs (wip/05-x1a 701f56c) | module-writer (opus) | 156K | 63 calls |
| 2026-11-13 | M3-I5 re-review after I5b and I5c: accept on condition (stale INT row), 8 open items; ruling 79 | Fable review (general-purpose), resumed | ~70K | 15 cap |
| 2026-11-13 | M3-D2 structural damping as design data: members, rope-snap wiring (wip/05-x1a f356526); transmission damping in a follow-up (578fcca) | module-writer (opus), resumed once | 226K + follow-up | 113 calls + follow-up |
| 2026-11-13 | M3 review: D2 (wip/05-x1a 578fcca): one small fix round, 3 medium, 4 low; ruling 80 (review-m3-d2.md) | Fable review (general-purpose) | 130K | 12 calls |
| 2026-11-13 | M3-I5d integrity rows rewritten to the code, neutral validity source for the party (wip/05-x1a 86e59fd) | module-writer (opus) | 246K | 104 calls |
| 2026-11-13 | M3-D2b damping fix round, stopped at its cap part done (wip/05-x1a aba358c) | module-writer (opus) | 146K | 65 calls |
| 2026-11-13 | M3-D2c damping fix round finished except the tier 1 note wiring (wip/05-x1a c15e219) | module-writer (opus) | 164K | 102 calls |
| 2026-11-13 | M3-D2d step count rule in the driver, tier 1 notes into the run notes (wip/05-x1a e65c083) | module-writer (opus) | 104K | 53 calls |
| 2026-11-13 | M3-X1f G12 and G15 at three step sizes, baseline on the new design hash (wip/05-x1a 6f17849); ruling 81 | module-writer (opus) | 242K | 112 calls |
| 2026-11-13 | M3 review: X1f (wip/05-x1a 6f17849): accept after a text-only fix round, 2 high, 4 medium, 2 low; ruling 82 (review-m3-x1f.md) | Fable review (general-purpose) | 139K | 17 calls |
| 2026-11-14 | M3-X1g gantry results fix round (wip/05-x1a f8f4836): rail_x frequency settled against the model (23.285 against 23.289 Hz), G6's one tick traced to the rail height | module-writer (opus) | 206K | 95 calls |
| 2026-11-14 | Integration branch invariant audit before landing: minor only | invariant-auditor (sonnet) | 26K | 6 calls |
| 2026-11-14 | M3-X1h driven mass with member chains, G12 and G15 baseline entries (wip/05-x1a 1e941bc); helper moved to a neutral module inline | module-writer (opus) | 190K | 106 calls |
| 2026-11-14 | M3-X2 suite on native by default, bridge baselines, retirement tests (wip/05-x2 2e40042): stopped at its cap, items 1 and 2 not green; ruling 83 | module-writer (opus) | 245K | 195 calls, over the 150 cap |
| 2026-11-14 | M3-N5 mass change on a rope-hung payload (wip/05-n5 1e62d64) | module-writer (opus) | 112K | 51 calls |
