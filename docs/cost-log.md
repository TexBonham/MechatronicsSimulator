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
