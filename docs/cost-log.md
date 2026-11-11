# Session Cost Log

One row per main task and one row per agent run. Agent tokens come from the task notification. Inline estimates are
from file reads. Archived rounds move to `docs/cost-log-archive.md` at round close. Reference per-run sizes by agent
type live in the root `CLAUDE.md` dispatch table once measured.

| Date | Task | Mode | Tokens (est.) | Notes |
|------|------|------|:-------------:|-------|
| 2026-11-05 | Round 3 close docs pass: 16 contract files swept, 59 index lines corrected, two HISTORY.md files | docs-indexer (haiku) | 97K | 11 calls, 3 min |
| 2026-11-05 | Round 3 close: rulings applied to spec rows, cost-log archive, queue, root dispatch table, PR | inline | ~30K | coordinator |
| 2026-11-10 | Resume: README python3-dev row, round/04-milestone-2 cut, milestone 2 plan assembled for sign-off | inline | ~90K | coordinator; 8 calls, spec and stub inventory by grep |
| 2026-11-10 | M2-W: Windows CI, presets, MuJoCo zip pin, MSVC warnings; firmware green, cpp and python blocked on the POSIX-only native runtime | general-purpose (opus) | 117K | 52 calls, 23 min; continued as M2-W2 |
| 2026-11-10 | M2-E: gantry firmware G5 to G14, lift axis, 56 host tests, 9 FW stubs converted (wip/04-e 5bf3179) | firmware-builder (opus), worktree | 218K | 50 calls, 28 min; builds queued on the lock |
| 2026-11-10 | M2-C: bridge slip, payload attach and release, backlash; PHY-006 converted (wip/04-c f29a552) | module-writer (opus), worktree | 216K | 71 calls, 30 min; 2 foreign fixtures expect refused backlash, fixed at merge-back |
