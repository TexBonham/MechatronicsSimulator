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
