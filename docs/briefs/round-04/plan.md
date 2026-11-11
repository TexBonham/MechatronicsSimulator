# Round 4 plan: milestone 2

Signed off by the owner 2026-11-10 ("go"), with rulings and clarifications retained in `docs/queue.md` (Round 4 open
rulings) and answered as the owner is available. Branch `round/04-milestone-2` from main at 46cbfe6.

## Scope

In: fault fan-out and sensor fidelity (SIG-002 to 008, 010, 014, 019, 020, 022, 034, 036, TST-014 to 020, 037); slip,
backlash and payload faults on the bridge (PHY-006, PHY-016, ENG-035); controller freeze, brownout, latency and
slow-down (CTL-011 to 013, 020, 021, KER-021); gantry firmware G5 to G14 behaviour with the lift axis (FW-017 to 021,
024 to 028); load cell, amplifier, IMU, brake catalog types (LIB-019, 025, 027, 031); chip data beyond pin names;
regression diff and reports (TST-008, SCO-009 to 012, 021, 025, FAC-012); sweep overrides and part substitution
(LIB-016, 017); facade service and conformance suite (FAC-001, 005 to 011, 014 to 017); viewer module and browser
client (VIE-001 to 009, 012, 014 to 017); stepper sinusoidal detent; recorded joint positions and drive torques; two
actuators on one shaft; Windows CI (R1); cases G5 to G11, G13, G14 (ACC-009 to 024, 027, 028).

Proposed re-tag to M3 (open ruling 4): ENG-016 to 021, LIB-009. If kept, brief P.

## Build concurrency (applied in wave 0)

- `CMakePresets.json`: `debug` preset = Debug, ASan, UBSan, no clang-tidy; `jobs: 2` on every build and test preset,
  core and firmware. `debug-sanitize` is the landing and CI preset.
- `tools/locked_build.sh <core|firmware|sync> <preset> [ctest args]`: configure, build, ctest under
  `flock /tmp/mechsim-build.lock`. Every brief builds through it; `uv sync` in a fresh worktree too.
- Writers iterate on `debug`, one full `debug` ctest at the end; the coordinator runs `debug-sanitize` once per stage
  commit. Waves hold at most two C++ writers.

## Briefs

Worktrees `git worktree add ../MechatronicsSimulator-04-<letter> -b wip/04-<letter>`. Caps are tool calls.

| Brief | Agent | Owns | Spec IDs | Cap | Cost |
|---|---|---|---|---|---|
| G chip and catalog data | schema-author, Opus | `mechsim/design/chip.py`, `mechsim/library/schema.py`, catalog YAML load cell, amplifier, IMU, brake, `chips/stm32_placeholder/chip.yaml`, `schemas/`, their tests | DES-029, LIB-019, 025, 027 | 120 | 150K-210K |
| W Windows CI | general-purpose, Opus, on CI | `.github/workflows/*.yml`, `core/cmake/MuJoCo.cmake`, Windows presets, `pyproject.toml` build section, `.gitattributes`, README setup | R1 | 120 | 150K-250K |
| A1 sensor fidelity and faults | module-writer, Opus | `core/signals/` sensor and stepper files, SIG stub tests, `core/signals/CLAUDE.md` | SIG-002, 003, 005 to 008, 010, 034, 036; sinusoidal detent | 150 | 220K-280K |
| C physics faults | module-writer, Opus | `core/physics/mujoco/`, `mechsim/physics/mujoco/`, PHY stub tests | PHY-006, 016, ENG-035 bridge side, TST-015, 020 delivery | 150 | 250K-300K |
| D runner and scoring | module-writer, Opus | `mechsim/testing/runner.py`, `mechsim/scoring/`, `mechsim/library/registry.py`, TST, SCO, LIB stub tests | TST-008, 014 to 020, 037; SCO-009 to 012, 021, 025; LIB-016, 017 | 150 | 200K-250K |
| E gantry firmware | firmware-builder, Opus | `firmware/app/gantry/`, `firmware/tests/`, fw.md open parameters | FW-017 to 021, 024 to 028; lift axis | 150 | 175K-250K |
| B controller and kernel | module-writer, Opus | `core/controller/`, `core/kernel/` record path, `mechsim/record/`, CTL, KER, REC stub tests | CTL-001, 011 to 014, 018, 020, 021; KER-021; SIG-022; joint position and drive torque series | 150 | 200K-250K |
| A2 power, safety, new sensors | module-writer, Opus, after A1 | `core/signals/` safety, amplifier, load cell, IMU, brake; reference tests | SIG-014, 019, 020; LIB-019, 021, 027 | 150 | 220K-280K |
| F1 facade service and viewer module | module-writer, Opus | `mechsim/facade/`, `mechsim/viewer/`, conformance suite, FAC and VIE unit tests, dependency line | FAC-001, 005 to 012, 014 to 017; VIE-003 to 009, 014 to 017 | 150 | 300K-350K |
| F2 browser client | module-writer, Opus, after F1 | `web/` | VIE-001, 002, 012, 015 client side | 120 | 200K-250K |
| X1 integration G5 to G8 | module-writer, Opus | `designs/gantry/`, G5 to G8, ACC stubs, `mechsim/builder/`, cross-package fixes | ACC-009 to 016, PHY-016; builder two actuators on one shaft; payload and end-effector catalog types (LIB-031) | 150 | 300K-350K |
| X2 integration G9 to G11 | module-writer, Opus, after X1 | as X1 plus G9 to G11, baseline | ACC-017 to 024 | 150 | 300K-350K |
| X3 integration G13, G14, fixes | module-writer, Opus, after X2 | as X1 plus G13, G14, `baseline.yaml` | ACC-027, 028 | 150 | 250K-300K |
| P engine collision and contact (if kept) | module-writer, Opus | `core/engine/` collision and contact, ENG reference stubs | ENG-016 to 021, ENG-035 engine side | 150 | 250K-300K |

Test commands: C++ briefs `tools/locked_build.sh core debug -R '<regex>'` filtered, then once
`tools/locked_build.sh core debug && uv run pytest -q`. Python briefs filtered `uv run pytest -q <files>`, then once
`uv run pytest -q`. Firmware `tools/locked_build.sh firmware host -R gantry`, then once the full host preset and
`uv run pytest -q tests/unit/test_fw_channels.py`. Integration: one case via `uv run mechsim run`, then
`uv run pytest -q tests/acceptance && uv run pytest -q`. W: push `wip/04-w`, `gh run watch`.

## Waves

| Wave | Briefs | Lock |
|---|---|---|
| 0 | presets, lock script, README, queue (inline); then G and W | G one sync; W on CI |
| 1 | A1, C, D, E | A1 and C share the lock |
| 2 | B, A2, F1 | B and A2 share the lock |
| 3 | F2, X1, P if kept | X1 the C++ consumer |
| 4 | X2 then X3 | one at a time |
| Gate | reference-validator (Sonnet), invariant-auditor (Opus), Fable gate review, owner | |

Per stage: oracle tests green in the worktree; Fable review from spec and diff only; invariant-auditor beside it;
fix round on resume; re-review; merge-back; one `debug-sanitize` build; stage commit. Pairing: G with W, A1 with C,
D with E, B with A2, F1 with F2, the X wave grouped. Owner-reviewed commits: the prep commit, G (contracts and
schemas), F1 (new facade and viewer contracts), the gate commit, one sampled stage commit in five.

Estimated round cost 5.5M to 6.1M tokens without P. Usage reserve (W-04): owner supplies the figure.

## Assumptions (as posted at sign-off)

1. `debug` keeps ASan and UBSan, drops clang-tidy; `jobs: 2` in the presets; lock covers core, firmware, `uv sync`.
2. Windows CI: `windows-2022`, MSVC, `release` and plain debug presets (no UBSan on MSVC), Python job on Windows,
   clang-tidy Linux only, MuJoCo Windows zip hash pinned by W. macOS out (R1 names Windows; CMake cannot extract the
   dmg). W corrects the README line naming macOS.
3. Gantry gains a load cell with amplifier on the hook, read through ADC, for G5 and G6. Overload limit = rated load
   plus 15 percent of the 3 kg placeholder.
4. G9 hold: power-off brake on the lift (open ruling 1). Brake type ships regardless.
5. G8 heartbeat: firmware digital output into the watchdog gating the motor rail; placeholders heartbeat 10 ms,
   timeout 50 ms, relay cutoff delay as the catalog placeholder.
6. G9 drops both rails; brownout resets the firmware; chip placeholder brownout threshold 2.0 V.
7. G10 encoder mismatch threshold 2 mm; encoders frame side.
8. Sinusoidal detent with the implicit drive kept: tangent stiffness and adjusted rest per tick; pull-out evaluated
   against the sinusoid.
9. Payload change on the bridge via the existing adapter operations; backlash as a dead band of two one-sided limits,
   recorded as an approximation; slip via the missed-step path with the bridge reporting `slip`.
10. Facade service on `websockets` and asyncio, in process, runs on a worker thread, localhost, per-session token.
    Browser client plain JavaScript, three.js vendored under `web/vendor/` at a pinned version, no build step.
11. ENG-016 to 021 and LIB-009 re-tagged M3 unless kept (brief P).
12. CTL-020 and CTL-021 unit tier only.
13. All cases rigid (BLD-033), milestone 1 placeholder geometry.
14. Three integration briefs; a wave whose intermediate states are never green lands as one commit.
15. New-type reference tests compare against closed-form responses derived in the brief.
16. The existing fault kinds are the complete milestone 2 fault vocabulary.
17. Writers Opus, firmware-builder Opus, invariant-auditor Sonnet per stage and Opus at the gate, docs-indexer Haiku.
