# Session Cost Log

One row per main task and one row per agent run. Agent tokens come from the task notification. Inline estimates are
from file reads. Archived rounds move to `docs/cost-log-archive.md` at round close. Reference per-run sizes by agent
type live in the root `CLAUDE.md` dispatch table once measured.

| Date | Task | Mode | Tokens (est.) | Notes |
|------|------|------|:-------------:|-------|
| 2026-11-04 | Round 2 close: public schema_base module, six imports, 11 tests | module-writer (opus), worktree | 85K | 16 calls, 3.4 min |
| 2026-11-04 | Round 2 close docs pass: indexes corrected, library index added, mechsim contract table | docs-indexer (haiku) | 71K | 23 calls, 1.5 min |
| 2026-11-04 | M1-S: wiring, contact environment, watchdog type, chip definitions, DES-029/041/042 | schema-author (opus), worktree | 170K | 48 calls, 10 min |
| 2026-11-04 | M1-S stage review | general-purpose (fable) | 117K | 19 calls, 3.5 min; pass, 2 medium carried to wave 1, vocabulary ruling |
| 2026-11-04 | M1-S invariant audit | invariant-auditor (sonnet) | 29K | 6 calls, 0.5 min; clean |
| 2026-11-04 | M1-V: peripheral-mode vocabulary unified, gantry pin table, rail-consumer rule | schema-author (opus), resumed | ~14K | 9 calls, 3 min |
| 2026-11-04 | Inline: heartbeat field removal across header, runtime, reference firmware; wave 1 worktrees | inline | ~25K | |
| 2026-11-04 | M1-F: recorder archives, exports, run-twice compare, 24 REC rows | module-writer (opus), worktree | 156K | 29 calls, 15 min |
| 2026-11-04 | Lost wave 1 first dispatch (container restart, ten writers in their first minutes) | module-writer x10 | ~150K est. | no output survived |
| 2026-11-04 | M1-H: gantry firmware G1 to G4, channel map, 62 host tests | firmware-builder (opus), worktree | 175K | 48 calls, 19 min |
| 2026-11-04 | M1-A: builder, neutral model, wiring table, channels header, 93 tests | module-writer (opus), worktree | 240K | 54 calls, 19 min |
| 2026-11-04 | M1-I: runner, driver protocol, scoring statistics, CLI, 23 TST and 18 SCO rows | module-writer (opus), worktree | 243K | 52 calls, 19 min |
| 2026-11-04 | M1 review: builder and runner (A, I) | general-purpose (fable) | 208K | 47 calls, 8 min; 0 algorithm defects, 1 cross-brief medium, 3 low |
| 2026-11-04 | M1 review: recorder and firmware (F, H) | general-purpose (fable) | 177K | 27 calls, 10 min; accept both; dir/enable source ruling |
| 2026-11-04 | M1-I fix round: baseline round trip, CLI kinds, tst rows | module-writer (opus), resumed | ~20K | ~10 calls |
| 2026-11-04 | M1-H fix round: single dir/enable source, fw rows, markers | firmware-builder (opus), resumed | ~10K | 9 calls, 5 min |
| 2026-11-04 | M1-A fix round: run step decoupled, module order, rigid_members (BLD-033), gantry placeholder geometry | module-writer (opus), resumed | ~36K | 24 calls, 13 min |
