# Session Cost Log

One row per main task and one row per agent run. Agent tokens come from the task notification. Inline estimates are
from file reads. Archived rounds move to `docs/cost-log-archive.md` at round close. Reference per-run sizes by agent
type live in the root `CLAUDE.md` dispatch table once measured.

| Date | Task | Mode | Tokens (est.) | Notes |
|------|------|------|:-------------:|-------|
| 2026-11-05 | Round 3 close docs pass: 16 contract files swept, 59 index lines corrected, two HISTORY.md files | docs-indexer (haiku) | 97K | 11 calls, 3 min |
| 2026-11-05 | Round 3 close: rulings applied to spec rows, cost-log archive, queue, root dispatch table, PR | inline | ~30K | coordinator |
| 2026-11-10 | Resume: README python3-dev row, round/04-milestone-2 cut, milestone 2 plan assembled for sign-off | inline | ~90K | coordinator; 8 calls, spec and stub inventory by grep |
