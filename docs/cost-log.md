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
