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
