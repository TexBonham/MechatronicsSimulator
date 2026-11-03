# Session Cost Log archive

Rounds closed, rows moved here at round close. Format as `docs/cost-log.md`.

## Round 1: phases 0a, 0c, 0b and the calibration spike (2026-11-02 to 2026-11-03)

Totals: phase 0a ~220K; phase 0c ~700K; phase 0b round 1 ~855K; spike ~1.16M (p1 ~290K, p2a ~450K, p2b ~420K).

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
| 2026-11-03 | 0b S0: traceability tool, spec template, stub conventions | module-writer (opus), worktree | 107K | 14 calls, 5 min; first module-writer reference size |
| 2026-11-03 | 0b S1: ENG KER SIG CTL FW specs, 152 stubs, ADR 0013 draft | general-purpose (opus), worktree | 143K | 29 calls, 7 min |
| 2026-11-03 | 0b S2: LIB MAT DES BLD PHY specs, 158 stubs | general-purpose (opus), worktree | 120K | 24 calls, 6 min |
| 2026-11-03 | 0b S3: TST REC INT SCO FAC VIE INV ACC specs, 224 stubs | general-purpose (opus), worktree | 141K | 29 calls, 8 min |
| 2026-11-03 | 0b merge-back, matrix regeneration, ENG-011 fix, verification | inline | ~12K | cherry-pick x4, full suite |
| 2026-11-03 | 0b cross-spec stage review | general-purpose (fable) | 182K | 22 calls, 6.5 min; 0 blockers, 12 should-fix |
| 2026-11-03 | 0b invariant audit | invariant-auditor (sonnet) | 25K | 5 calls, 0.4 min; caught stale matrix |
| 2026-11-03 | 0b fix pass: 15 review items, 12 new IDs | general-purpose (opus) | 137K | 27 calls, 5 min |
| 2026-11-03 | Spike p1: math core and integrator, 7 ENG IDs, 17 Catch2 tests | module-writer (opus), worktree | 110K | 21 calls, 11 min; first measured engine package |
| 2026-11-03 | Spike p1 stage review | general-purpose (fable) | 108K | 12 calls, 5 min; 1 blocker (RHS term missing, brief's formula), 2 should-fix, 3 nits |
| 2026-11-03 | Spike p1 invariant audit | invariant-auditor (sonnet) | 36K | 7 calls, 0.6 min; 4 minor |
| 2026-11-03 | Spike p1 fix round: RHS term, physics-derived tests, ENG-049/050, auditor items | module-writer (opus), resumed | ~15K | 7 calls, 2.5 min; delta of the cumulative 124K reported on resume |
| 2026-11-03 | Spike p1 re-review (ENG-003 focus) | general-purpose (fable), resumed | ~11K | 5 calls, 1.3 min; ready, 1 item to 2a |
| 2026-11-03 | Spike p2a: rigid multibody, 6 joints, loop constraints, 11 ENG IDs, 44 tests total | module-writer (opus), worktree | 220K | 50 calls, 40 min; largest engine package |
| 2026-11-03 | Spike p2a stage review | general-purpose (fable) | 165K | 24 calls, 10.5 min; 0 algorithm defects, 3 should-fix (spec rows, tags, CI time), 2 nits |
| 2026-11-03 | Spike p2a invariant audit | invariant-auditor (sonnet) | 35K | 7 calls, 0.5 min; 1 major spec gap, 3 minor |
| 2026-11-03 | Spike p2a fix round: spec rows, ENG-050 widened, tags, 2 000-step four-bar, -O1 sanitize preset | module-writer (opus), resumed | ~15K | 14 calls, 25 min; delta of cumulative 235K |
| 2026-11-03 | Spike p2b: joint springs, lumped beam, six analytical references, 7 ENG IDs, 61 tests total | module-writer (opus), worktree | 188K | 45 calls, 26 min |
| 2026-11-03 | Spike p2b invariant audit | invariant-auditor (sonnet) | 35K | 7 calls, 0.4 min; 1 major spec row, 1 minor |
| 2026-11-03 | Spike p2b reference-validator first run | reference-validator (sonnet) | 30K | 8 calls, 1.8 min; 6/6 pass, ENG-040 N=4 caveat, 5 definition items |
| 2026-11-03 | Spike p2b stage review | general-purpose (fable) | 140K | 19 calls, 7.4 min; 0 algorithm defects, 2 medium (raw assertion, spec row), 3 low, style |
| 2026-11-03 | Spike p2b fix round: raw assertions, spec rows, roller field, style | module-writer (opus), resumed | ~15K | 7 calls, 5 min; delta of cumulative 203K |
| 2026-11-03 | Spike merge-backs, verification x3, cost model in root CLAUDE.md | inline | ~40K | three cherry-pick merges, full builds |
