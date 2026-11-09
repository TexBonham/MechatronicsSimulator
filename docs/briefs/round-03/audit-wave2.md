# Invariant audit: milestone 1 wave 2

Read-only. Worktree ../MechatronicsSimulator-m1-x2, branch wip/m1-x2 (wave 2 WIP commits on top of the landed branch). Diff range: `git diff ae4f7c7..1826496` in the worktree ../MechatronicsSimulator-m1-x2
(five WIP commits: driver, gantry end to end, stepper hold, stepper through physics, no-motion criterion).
Call cap 30. Model: Sonnet (stage audit).

Audit against the nine invariants in the root CLAUDE.md and the spec-maintenance rule (R-18):
1. Controller never sees true state: every physics-to-controller path crosses the signal layer's pin tables; look for
   any include or call from core/controller into core/physics or core/engine, and for the signal layer handing
   physics outputs through to pin inputs unprocessed.
2. Nothing design-specific in code: grep core/ and mechsim/ for gantry, trolley, hoist, hook, lead-screw constants,
   pin numbers (PA9, PA10), 40 cm, 25 cm.
3. Data files are the source of truth: any YAML added validates (`uv run pytest -q tests/invariant -k schema`).
4. Determinism: unordered containers iterated in output paths, time-of-day or address-dependent behavior, threads.
5. Engine and emulator boundaries: `python3 tools/check_imports.py` plus a manual look at core/bindings exposing
   engine types only under `_core_ext.engine`, and MuJoCo symbols confined to core/physics/mujoco and
   mechsim/physics/mujoco.
6. Fidelity tiers additive: no tier flag removes behavior.
7. Failures are errors: every catch block either rethrows, wraps into the typed error, or ends the run; no swallowed
   exception, no silent approximation in the bridge (mapping report lines for every drop).
8. Board interface only coupling: firmware/app includes nothing outside board_interface (the include check target).
9. Headless core: no Qt, display or plotting import outside mechsim/viewer and web/.
R-18: every behavior the diff adds cites a requirement ID in a test marker or Catch2 tag, and the spec rows it
converts exist; `python3 tools/traceability.py` passes.

Report findings by file:line and severity (blocker, high, medium, low), one line each, then a one-line verdict per
invariant. Numbers once. No praise.
