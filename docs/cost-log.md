# Session Cost Log

One row per main task and one row per agent run. Agent tokens come from the task notification. Inline estimates are
from file reads. Archived rounds move to `docs/cost-log-archive.md` at round close. Reference per-run sizes by agent
type live in the root `CLAUDE.md` dispatch table once measured.

| Date | Task | Mode | Tokens (est.) | Notes |
|------|------|------|:-------------:|-------|
| 2026-11-02 | Planning session: workflow, architecture rulings, `docs/rulings/2026-11-02-planning-rulings.md` | inline | ~120K | Coordinator session; two doc reads, 14 web lookups |
| 2026-11-02 | ADR set 0001-0012 + arch doc edits, `docs/adr/` | general-purpose (opus) | 102K | 17 calls, 4.5 min; first reference size for a docs-writer run |
| 2026-11-02 | 0c brief A: scaffolding, pyproject, CMake presets, checker, CI | general-purpose (opus), worktree | 106K | 27 calls, 7.5 min; builds included |
| 2026-11-02 | 0c brief B: six agents, hooks, five commands, `.claude/` | general-purpose (opus), worktree | 93K | 18 calls, 4 min |
| 2026-11-02 | 0c brief C: root and per-dir `CLAUDE.md`, `docs/milestones.md` | general-purpose (opus), worktree | 107K | 20 calls, 3 min |
| 2026-11-02 | 0c merge-back, verification, ruff fix | inline | ~15K | cherry-pick x3, full suite, C++ debug preset |
| 2026-11-02 | 0c stage review from plan and diff | general-purpose (fable) | 136K | 19 calls, 5.5 min; 0 blockers, 5 should-fix, 8 nits |
| 2026-11-02 | 0c invariant-auditor smoke run and definition feedback | general-purpose (sonnet) | 69K | 6 calls, 0.5 min |
