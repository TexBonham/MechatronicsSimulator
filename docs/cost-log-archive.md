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

## Round 3: milestone 1 (2026-11-04 to 2026-11-05)

Wave 0 (contracts), wave 1 (ten briefs in worktrees), wave 2 (integration X1 to X7); gate passed 2026-11-05.
Runs far above their peers: integration writers M1-X1 (312K + 32K fix, 166 calls), M1-X3 (345K, 150), M1-X5 (321K, 126)
against the module-writer reference 110K-220K; cause: each integration brief spans five packages and discovered
a design defect (sensor placement, stepper hold, explicit-spring stability) mid-run. Reviews of grouped briefs ran
240K-327K against the 108K-182K reference, in proportion to diff size. Recorded in the root dispatch table.

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
| 2026-11-04 | M1-B: kernel implementation, coupling table, in-memory sink, 58 cases | module-writer (opus), worktree | 194K | 56 calls, 58 min; builds contended with concurrent worktrees |
| 2026-11-04 | M1-E: native controller runtime, host build, shim, channel map, 30 tests | module-writer (opus), worktree | 170K | 49 calls, 62 min; builds contended |
| 2026-11-04 | M1-D: signal built-ins, SignalLayer party, ASCII operator link, 26 SIG rows, 141 core tests | module-writer (opus), worktree | 269K | 87 calls, 62 min; over the 150 cap region, builds contended |
| 2026-11-04 | M1-C: MuJoCo bridge, MJCF translation and C++ stepping adapter, 3.9.0 pinned, 123 core tests | module-writer (opus), worktree | 279K | 78 calls, 64 min; builds contended |
| 2026-11-04 | M1-J: engine package 7 transmissions, 14 cases, derived bounds | module-writer (opus), worktree | 183K | 41 calls, 66 min; builds contended |
| 2026-11-04 | M1 review: kernel and native runtime (B, E) | general-purpose (fable) | 159K | 21 calls, 5 min; 2 high (E), 2 medium each, 4 rulings |
| 2026-11-04 | M1-G: pybind11 bindings, scikit-build wiring, `mechsim/_core.py`, 26 binding tests | module-writer (opus), worktree | 208K | 60 calls, 72 min; uv sync 4m24s cold, 51s warm |
| 2026-11-04 | M1-B fix round: role-defaulted rates, physics-boundary end checks, sink_errors, line index | module-writer (opus), resumed | ~19K | 14 calls, 6 min |
| 2026-11-04 | M1 review: MuJoCo bridge and signals (C, D) | general-purpose (fable) | 240K | 36 calls, 11 min; C blocked on transmission mapping, D accepted with 3 items to integration |
| 2026-11-04 | M1-E fix round: six rulings, header 2.2, dir/enable from digital_out, held levels, reload on reset | module-writer (opus), resumed | ~39K | 24 calls, 9 min |
| 2026-11-04 | Lost to container restart: G/J review and C fix round, both just dispatched | general-purpose (fable), module-writer | ~5K | re-dispatched with the same briefs |
| 2026-11-04 | M1 review: bindings and transmissions (G, J) | general-purpose (fable) | 198K | 21 calls, 7 min; both accept with fixes, 1 high (G engine names flat in `_core`) |
| 2026-11-04 | M1-C fix round: reflection mapping, efficiency by power direction, load torque (PHY-033), pair filter, kWorldBody | module-writer (opus), resumed | ~58K | 21 calls, 7 min |
| 2026-11-04 | M1-E re-review after fix round | general-purpose (fable), resumed | ~32K | 8 calls, 2 min; accept |
| 2026-11-04 | M1-C re-review after fix round | general-purpose (fable), resumed | ~35K | 8 calls, 2 min; accept, 2 medium to integration and gate |
| 2026-11-04 | M1-J fix round: transmission documentation items | module-writer (opus), resumed | ~10K | 9 calls, 6 min |
| 2026-11-04 | M1-G fix round: engine submodule and boundary rule, lumped beam bound, 11 edge cases, cache-keys | module-writer (opus), resumed | ~24K | 12 calls, 7 min |
| 2026-11-04 | Invariant audit: wave 1 stage commits (9fc0980..d4c7459) | invariant-auditor (sonnet) | 30K | 10 calls, 1 min; all nine pass, 2 low notes to integration |
| 2026-11-04 | M1-X1: core wiring, coupling crossings, pins target, load-torque convention, catalog alignment, kernel and signal bindings, KER-023 | module-writer (opus), worktree | 312K | 135 calls, 45 min; MuJoCo adapter binding left open, fix round sent |
| 2026-11-04 | M1-X1 fix round: MuJoCo adapter bound with libmujoco in the wheel, inverse heartbeat test, negative-ratio cases, ker.md rows | module-writer (opus), resumed | ~32K | 31 calls, 14 min |
| 2026-11-04 | M1 review: core wiring (X1) | general-purpose (fable) | 198K | 22 calls, 5 min; accept with 5 minor fixes, applied inline by the coordinator |
| 2026-11-04 | M1-X2: CoreDriver, PreparedModel assembly, rotor inertia (BLD-034), coupling mirror (REC-028), CLI entry, P1 green and run-twice identical | module-writer (opus), worktree | 286K | 109 calls, 28 min; gantry blocked on sensor placement in the design schema |
| 2026-11-04 | M1-X3: sensor placement (DES-043), open-parameter defaults (DES-044), gantry cases G1, G2, G4, generated channel header, gantry end to end; G2 and G3 green | module-writer (opus), worktree | 345K | 150 calls, 35 min; G1 and G4 blocked on the stepper hold model |
| 2026-11-04 | M1-X4: stepper hold law, state-code names (DES-045), status message fields, G1 to G4 re-run | module-writer (opus), resumed | ~70K | 50 calls, 23 min; explicit hold spring unstable at 1 ms on the lift (omega dt 2.3) |
| 2026-11-04 | M1-X5: joint drives in PhysicsInputs (KER-027, PHY-034), implicit hold in the MuJoCo adapter, stepper publishes its drive; G2, G3 green, G1, G4 on ringing | module-writer (opus), worktree | 321K | 126 calls, 52 min; UART loss root cause found |
| 2026-11-04 | M1-X6: no-motion criterion (SCO-026), UART and counter accumulation at unequal rates (SIG-038), baselines; G1 to G4 and P1 green, run twice identical | module-writer (opus), resumed | ~45K | ~40 calls, 20 min |
| 2026-11-04 | Invariant audit: wave 2 (ae4f7c7..1826496) | invariant-auditor (sonnet) | 26K | 8 calls, 2 min; all pass, stale matrix only |
| 2026-11-04 | Reference validation: milestone 1 (C++ 8, Python 9, MuJoCo 26, P1) | reference-validator (sonnet) | 42K | 12 calls, 3 min; 0 failed; two T4 rows to set (BLD-010, SIG-011) |
| 2026-11-04 | M1 review: wave 2 (X2 to X6) | general-purpose (fable) | 327K | 35 calls, 12 min; accept with fixes: 3 major (matrix, typed errors, approximation lines), 4 minor, 4 owner questions |
| 2026-11-04 | M1-X7: review fixes (typed driver errors, approximation lines in scores, stepper loss of synchronism, SIG-037 wiring) | module-writer (opus), worktree | 173K | 75 calls, 23 min |
| 2026-11-04 | Wave 2 merge-back: one stage commit (4afdf8f) after a three-stage attempt failed on a never-green intermediate state; host build dir keyed by firmware tree (inline fix) | inline | ~40K | coordinator |

## Round 4 (milestone 2), 2026-11-10 to 2026-11-11: 60 rows, about 6.33M tokens

| Date | Task | Mode | Tokens (est.) | Notes |
|------|------|------|:-------------:|-------|
| 2026-11-10 | Resume: README python3-dev row, round/04-milestone-2 cut, milestone 2 plan assembled for sign-off | inline | ~90K | coordinator; 8 calls, spec and stub inventory by grep |
| 2026-11-10 | M2-W: Windows CI, presets, MuJoCo zip pin, MSVC warnings; firmware green, cpp and python blocked on the POSIX-only native runtime | general-purpose (opus) | 117K | 52 calls, 23 min; continued as M2-W2 |
| 2026-11-10 | M2-E: gantry firmware G5 to G14, lift axis, 56 host tests, 9 FW stubs converted (wip/04-e 5bf3179) | firmware-builder (opus), worktree | 218K | 50 calls, 28 min; builds queued on the lock |
| 2026-11-10 | M2-C: bridge slip, payload attach and release, backlash; PHY-006 converted (wip/04-c f29a552) | module-writer (opus), worktree | 216K | 71 calls, 30 min; 2 foreign fixtures expect refused backlash, fixed at merge-back |
| 2026-11-10 | M2-G: chip fields, four catalog types, front_end category, schemas regenerated (wip/04-g ab1f65e) | schema-author (opus), worktree | 144K | 51 calls, 37 min; one foreign chip fixture to fix at merge-back |
| 2026-11-10 | M2-A1: sensor fidelity, fault modes, debounce, tangent detent, 57 signal cases (wip/04-a1 86eecaf) | module-writer (opus), worktree | 213K | 69 calls, 32 min; one foreign controller test, binding gap for fidelity |
| 2026-11-10 | Invariant audit: wave 1a (G, C, A1) | invariant-auditor (sonnet) | 37K | 6 calls, 0.5 min; clean, one accepted minor |
| 2026-11-10 | M2-D: suite file, regression diff and report, interval and no_overlap criteria, overrides and part substitution (wip/04-d 525fff3) | module-writer (opus), worktree | 232K | 81 calls, 31 min; 3 foreign invariant-test failures on suite.yaml, fixed at merge-back |
| 2026-11-10 | Invariant audit: wave 1b (D, E) | invariant-auditor (sonnet) | ~35K | clean, one accepted minor |
| 2026-11-10 | M2 review: G and C | general-purpose (fable) | ~120K | 16 calls; G accept, C accept with fixes (1 medium, 5 low, 2 rulings) |
| 2026-11-10 | M2 review: A1 | general-purpose (fable) | 127K | 16 calls; accept with fixes (1 high: predicted-lead detent, 1 medium, 3 low) |
| 2026-11-10 | M2 review: D and E | general-purpose (fable) | 201K | 28 calls; both accept, 2 owner rulings, event recording owed to X1 |
| 2026-11-10 | M2-G stage landed: merge wip/04-g, chip fixture fix, matrix regenerated, full suite 3330 passed | inline | ~15K | coordinator |
| 2026-11-10 | M2-C fix round: six review items, rulings 9 and 10 (wip/04-c e2ead4a) | module-writer (opus), resumed | ~15K | 11 calls |
| 2026-11-10 | M2-A1 fix round: predicted-lead detent, closed-form lag, rulings 11 to 13 (wip/04-a1 5b166b9) | module-writer (opus), resumed | ~16K | 14 calls |
| 2026-11-10 | M2-C re-review | general-purpose (fable), resumed | ~11K | 1 call; accept |
| 2026-11-10 | M2-A1 re-review | general-purpose (fable), resumed | ~14K | 3 calls; accept |
| 2026-11-10 | M2-D stage landed: merge wip/04-d, suite file in the invariant tier and YAML hook, full suite 3391 passed; E backed out to land with X1 (unbound E-stop status channel faults the firmware) | inline | ~25K | coordinator |
| 2026-11-10 | M2-W2: native runtime on Windows (CreateProcess, LoadLibrary), INV-020, cpp and python green on windows-2022 twice (wip/04-w 12a171c) | general-purpose (opus), resumed | ~53K | 37 calls total for W and W2, 58 min |
| 2026-11-10 | Invariant audit: W and W2 | invariant-auditor (sonnet) | 31K | 4 calls; clean, two style minors |
| 2026-11-10 | M2 review: W and W2 | general-purpose (fable) | ~45K | 9 calls; accept with minor fixes (x64 guard inline, helper dedup to B) |
| 2026-11-10 | M2-C and M2-A1 stage landed: merges, driver fixtures to self-locking, 6 clang-tidy fixes in tests, debug-sanitize 278 passed, full suite 3406 passed | inline | ~40K | coordinator |
| 2026-11-10 | M2-W stage landed: merge wip/04-w, x64 guard on the MSVC intrinsic, debug-sanitize 278 passed, invariant tier 159 passed; Windows CI green on the wip branch twice | inline | ~15K | coordinator |
| 2026-11-10 | M2-F1: facade service, WebSocket server, viewer module, conformance suite, CLI through the facade (wip/04-f1 112de83) | module-writer (opus), worktree | 258K | 81 calls, 35 min; 7 import-rule test failures, ruling 15 |
| 2026-11-10 | M2 review: F1 | general-purpose (fable) | ~110K | 19 calls; accept after one fix round (1 major: reader seam, 8 minor) |
| 2026-11-10 | M2-A2: load cell, amplifier, IMU, brake, power-loss fault, fidelity binding, front_end stage; 308 ctest, full suite green (wip/04-a2 1e03870) | module-writer (opus), worktree | ~250K | 100 calls, 65 min; corrected the brief reference arithmetic (635 counts, not 318) |
| 2026-11-10 | M2-B: freeze and slow-down scheduling, latency, brownout from chip data, physics fault scheduling, joint and payload series, Windows helper dedup; 294 ctest, full suite green (wip/04-b 37ba56f) | module-writer (opus), worktree | ~235K | 91 calls, 70 min; found the ctest-argument quoting bug in locked_build.sh (fixed inline) |
| 2026-11-10 | M2-F1 fix round: reader under mechsim/record, one answer per request, strict version, end reasons, conformance cases (wip/04-f1 8e960e3) | module-writer (opus), resumed | ~35K | 26 calls |
| 2026-11-10 | M2-F1 re-review | general-purpose (fable), resumed | ~30K | 8 calls; accept; ADR 0015 events row to align at merge-back |
| 2026-11-10 | Invariant audit: wave 2 (A2, B, F1) | invariant-auditor (sonnet) | ~35K | clean, three minors (tier default, archive format version, swallowed parse error) |
| 2026-11-10 | M2 review: A2 and B | general-purpose (fable) | 138K | 18 calls; A2 accept, B accept with rulings 18 to 20 |
| 2026-11-10 | M2-F1 stage landed: merge wip/04-f1, ruling 15 in the import test and tool contract, TST-034 stub dropped, ADR 0015 events row aligned; full suite 3463 passed | inline | ~20K | coordinator |
| 2026-11-10 | M2-B fix round: column names and units attrs, archive format 2, window checks at start (wip/04-b e499c86) | module-writer (opus), resumed | ~34K | 21 calls |
| 2026-11-10 | M2-B re-review | general-purpose (fable), resumed | ~20K | 3 calls; accept |
| 2026-11-10 | M2-F2: browser client, serve command with static route, vendored three.js r170, browser-free checks; VIE-001, 012 converted (wip/04-f2 bb25d8f) | module-writer (opus), worktree | ~140K | 52 calls, 30 min; one foreign import-set test to fix at merge-back; no list_runs request yet |
| 2026-11-10 | Invariant audit: F2 | invariant-auditor (sonnet) | ~25K | clean |
| 2026-11-10 | M2 review: F2 | general-purpose (fable) | ~65K | 14 calls; accept, 2 medium to X2 (advance kind, cursor sample), coordinator import-set fix |
| 2026-11-10 | M2-A2 and M2-B stage landed: merges, bindings contract merged, 8 clang-tidy fixes, optional guard in the load cell, debug-sanitize 327 passed, full suite 3511 passed | inline | ~45K | coordinator |
| 2026-11-10 | M2-F2 stage landed: merge wip/04-f2, CLI import set includes the server module; full suite 3551 passed | inline | ~10K | coordinator |
| 2026-11-10 | M2-X1: design, schema, builder payload, wiring, channel header, G5 to G8 case files; G2 to G4 pass, G1 and G5 to G8 blocked (brake drive refused, welded payload unstable) (wip/04-x1 d774e4f) | module-writer (opus), worktree | 317K | 142 calls, 33 min; cap reached, handed off to X2 |
| 2026-11-11 | M2-X2: drive combination (ruling 23), per-axis step scale, estop auto-close, G8 case; G1 to G4 and G8 pass, G5 to G7 diverge (wip/04-x1 9030952) | module-writer (opus), fresh in the X1 worktree | ~255K | 123 calls, 46 min; cap reached, stepper runaway and payload build left |
| 2026-11-11 | M2-A3: stepper loss-of-synchronism rule (SIG-039), drive torque apportioning, load cell joint_reaction; G7 no longer diverges (wip/04-a3 fa102dc) | module-writer (opus), worktree from X2 WIP | ~198K | 70 calls, 20 min |
| 2026-11-11 | M2-X3: payload as fixed child, joint_reaction senses, rail ref, G5 to G11 cases, suite and baseline to G11, 20 ACC/FW/TST stubs converted; G1 to G9 and G11 pass, G10 one criterion (wip/04-x1 db7e1de) | module-writer (opus), fresh in the X1 worktree | ~293K | 145 calls, 43 min; cap reached |
| 2026-11-11 | Gate audit: integration wave (X1, X2, A3, X3) | invariant-auditor (opus) | ~60K | 2 major (stand-in payload body, event argument kinds), 2 minor; to X4 |
| 2026-11-11 | M2 review: integration wave (X1, X2, A3, X3) | general-purpose (fable) | ~120K | 18 calls; accept with fixes (1 high: hold criteria by construction; 5 medium; rulings 28 to 31) |
| 2026-11-11 | M2-C2: bridge drive mapping exact backward Euler (runaway fixed), brake soft slip drive; G3 and G7 pass, G10 needs frame-side encoders (wip/04-c2 6032236) | module-writer (opus), worktree from X3 WIP | ~200K | est.; report pending usage |
| 2026-11-11 | M2 review: C2 (physics derivation of the drive mapping) | general-purpose (fable) | 116K | 11 calls; accept with one medium (envelope throw at Z > 400), 4 low |
| 2026-11-11 | M2-C2 fix round: envelope throw at Z > 400, spec rows, derivation (wip/04-c2 0814bb6) | module-writer (opus), resumed | ~14K | 7 calls |
| 2026-11-11 | M2-C2 re-review | general-purpose (fable), resumed | ~25K | 4 calls; accept, low finding withdrawn |
| 2026-11-11 | M2-X4: event argument kinds, brake error, fault ref, export labels, G13 and G14, review fixes; G11, G13, G14 pass, G7 and G9 wait on C2 (wip/04-x1 317e419) | module-writer (opus), fresh in the X1 worktree | ~248K | 147 calls, 39 min |
| 2026-11-11 | M2-A4: linear encoder, load cell without target; 347 ctest (wip/04-a4 597e038) | module-writer (opus), worktree | ~100K | 24 calls, 6 min |
| 2026-11-11 | M2-X5: frame-side linear encoders, stand-in payload removed, G7, G9, G10 green, suite 13 of 13, LIB-031 types, list_runs and advance (wip/04-x1 88aa526) | module-writer (opus), fresh in the X1 worktree | ~200K | 96+ calls; report pending usage |
| 2026-11-11 | Gate audit: integration wave part 2 (X4, A4, X5) | invariant-auditor (opus) | ~55K | first-part majors closed; 1 R-18 blocker (BLD-035 text), 2 minor; to the X5 fix round |
| 2026-11-11 | M2 review: integration wave part 2 (X4, A4, X5) | general-purpose (fable) | 142K | 19 calls; accept, 3 low; first-part high closed |
| 2026-11-11 | M2-X5 fix round: BLD-035 row, catalog-driven core type, linear_encoder comment, load-cell target errors, tidy, ruff, mypy (wip/04-x1 a0e2fd3) | module-writer (opus), resumed | ~45K | est. |
| 2026-11-11 | Integration-wave stage landed: wip/04-x1 merged twice (E, X1 to X5, A3, A4, C2), debug-sanitize 347 passed, firmware 92, full suite 3707 passed with G1 to G11, G13, G14 in the acceptance tier | inline | ~60K | coordinator |
| 2026-11-11 | Reference validation: milestone 2 gate (C++ 15 + 12 MuJoCo, Python 9; baselines) | reference-validator (sonnet) | 41K | 14 calls, 9 min; 0 misses; MuJoCo reference cases now labelled for ctest (inline) |
| 2026-11-11 | Fable gate review: milestone 2 | general-purpose (fable) | ~110K | 13 calls; pass with owner confirmations; CI on the merged branch blocking until the owner pushes |
| 2026-11-11 | M2-F3: client on list_runs and advance, cursor-sample narrowing (wip/04-f3 dd53bd5); review accept with one client fix, applied inline (span local to the request) | module-writer (opus) + general-purpose (fable) | ~90K + ~64K | 23 + 6 calls |
| 2026-11-11 | Docs close pass: markers in 13 files, 15 index tables, 3 HISTORY.md files, root dispatch table (wip/04-docs 4e0fc9c) | docs-indexer (haiku) | 163K | 24 calls, 5 min; contract files grew with index tables, accepted (queue item 3) |
