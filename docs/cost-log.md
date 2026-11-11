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
