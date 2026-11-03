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

## Round 2: phase 1, contracts and schemas (2026-11-03 to 2026-11-04)

Totals: phase 1 ~2.6M (seven briefs 118K-210K each with one fix round each, five Fable reviews 130K-184K, four audits, inline ~90K) against a 1.4M estimate; overrun from one review per batch instead of three in total, fix rounds, and schema briefs at 170K-210K.

| Date | Task | Mode | Tokens (est.) | Notes |
|------|------|------|:-------------:|-------|
| 2026-11-03 | Round close: markers and indexes for two long tooling files, tools/ and tests/ CLAUDE.md | docs-indexer (haiku) | 57K | 15 calls, 1.7 min; first docs-indexer reference size |
| 2026-11-03 | Round close inline: cost-log archive, root CLAUDE.md trim, two agent definitions | inline | ~10K | |
| 2026-11-03 | Phase 1 C7: ADR 0015 facade protocol, message models, conformance stubs | schema-author (opus), worktree | 123K | 26 calls, 6 min; first schema-author reference size |
| 2026-11-03 | Phase 1 C6: board interface header, firmware presets, include check, reference stub, 26 host tests | firmware-builder (opus), worktree | 118K | 27 calls, 8 min; first firmware-builder reference size |
| 2026-11-03 | Phase 1 inline: units module, ADR 0014, traceability firmware scan | inline | ~25K | |
| 2026-11-03 | Phase 1 C1: neutral physics model, schema, pendulum example, 8 BLD stubs converted | schema-author (opus), worktree | 169K | 39 calls, 13 min |
| 2026-11-03 | Phase 1 C2: component types, 8 catalog parts, 6 materials, two schemas, 28 stubs converted | schema-author (opus), worktree | 175K | 35 calls, 14 min |
| 2026-11-03 | Phase 1 batch A stage review (C1, C2, C7) | general-purpose (fable) | 184K | 37 calls, 7 min; 0 blockers beyond matrix regen, 9 should-fix, nits |
| 2026-11-03 | Phase 1 C2 fix round: CatalogPart.resolve contract, single dimension error, $id | schema-author (opus), resumed | ~17K | 9 calls, 2.3 min |
| 2026-11-03 | Phase 1 C7 fix round: ADR open items, NonEmptyText | schema-author (opus), resumed | ~3K | 3 calls, 0.5 min |
| 2026-11-03 | Phase 1 C1 fix round: joint tiers, unit tolerances, node damping, ContactModel, names | schema-author (opus), resumed | ~29K | 18 calls, 4 min |
| 2026-11-03 | Phase 1 C5: core runtime interfaces, null implementations, 101 tests, three contract files | module-writer (opus), worktree | 178K | 51 calls, 27 min |
| 2026-11-03 | Phase 1 batch A invariant audit (landed commits) | invariant-auditor (sonnet) | 30K | 9 calls, 0.7 min; clean, 2 minor notes |
| 2026-11-03 | Phase 1 batch B stage review (C5, C6) | general-purpose (fable) | 167K | 22 calls, 5.3 min; 0 blockers, 7 should-fix, 2 design recommendations |
| 2026-11-03 | Phase 1 C6 fix round: header v2 layout, event log, include check hardening, spec rows | firmware-builder (opus), resumed | ~29K | 13 calls, 5 min |
| 2026-11-03 | Phase 1 C3: design file contract, CAD import interface, catalog registry, gantry and pendulum designs, 27 DES stubs converted | schema-author (opus), worktree | 205K | 37 calls, 16 min |
| 2026-11-03 | Phase 1 C5 fix round: rational tick time base, PreparedModel load, adapter surface, pin contract, 109 tests | module-writer (opus), resumed | ~47K | 24 calls, 19 min |
| 2026-11-03 | Phase 1 inline: batch merges, FW stub removal, verification | inline | ~20K | |
| 2026-11-03 | Phase 1 C3 stage review | general-purpose (fable) | 151K | 32 calls, 6 min; 1 mechanical blocker (matrix), 2 medium, 5 low, 4 owner items |
| 2026-11-03 | Phase 1 batch B invariant audit (landed commit) | invariant-auditor (sonnet) | 51K | 10 calls, 0.8 min; clean, 2 minor |
| 2026-11-03 | Phase 1 C3 fix round: heartbeat routing, DES-041, supports, style | schema-author (opus), resumed | ~30K | ~14 calls |
| 2026-11-03 | Phase 1 C4: test case and baseline file kinds, G3, P1, baseline example, 13 stubs converted | schema-author (opus), worktree | 210K | 42 calls, 16 min |
| 2026-11-03 | Phase 1 C4 stage review | general-purpose (fable) | 130K | 30 calls, 5.6 min; 1 mechanical blocker (matrix), 1 high (hook), 3 medium, owner items |
| 2026-11-03 | Phase 1 C4 fix round: inclusive criteria, P1 damped period, SCO-020, spec rows | schema-author (opus), resumed | ~25K | ~12 calls |
| 2026-11-03 | Phase 1 waves 2 and 3 invariant audit | invariant-auditor (sonnet) | 42K | 9 calls, 0.7 min; clean |
