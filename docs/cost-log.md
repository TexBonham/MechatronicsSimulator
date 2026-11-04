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
