# Stage review: M1-G (core bindings) and M1-J (engine package 7, transmissions)

You are the Fable stage reviewer (W-02). Review from the spec and the diff only. Do not read any writer
summary, report, cost log, HISTORY.md, or rulings file. Change nothing: no edits, no commits, no test edits.

## Inputs

Two worktrees, both branched from 2eb0a2e on `round/03-milestone-1`:

| Brief | Worktree | Commit | Diff |
|---|---|---|---|
| M1-G bindings | ../MechatronicsSimulator-m1-g | 6ee02a2 | `git -C ../MechatronicsSimulator-m1-g diff 2eb0a2e..6ee02a2` |
| M1-J transmissions | ../MechatronicsSimulator-m1-j | 241f8ee | `git -C ../MechatronicsSimulator-m1-j diff 2eb0a2e..241f8ee` |

Specs: `docs/spec/eng.md` (ENG-043, ENG-044, ENG-048 and any other ID the J diff cites), `docs/spec/ker.md`
(KER rows the G diff cites, notably KER-019), plus `docs/spec/sig.md`, `ctl.md`, `rec.md`, `phy.md` for any ID
the G bindings expose. Contracts: `core/engine/CLAUDE.md`, `core/bindings/CLAUDE.md` (new in G),
`core/kernel/CLAUDE.md`, `mechsim/CLAUDE.md`. Root `CLAUDE.md` for invariants and the C++ rules. ADR 0013
(`docs/adr/0013-*.md`) for the integrator contract the transmissions must respect.

## What to check

M1-J (physics, highest weight):
1. Derive independently, from ENG-043/044/048 and the integrator contract, what the belt-pulley and lead-screw
   elements must do: ratio and lead mapping of shaft torque to joint force, efficiency direction (forward vs
   back-driving), stiffness as an implicit spring in the `(H + hD + h²K)` system, backlash dead band, slip, the
   self-locking hold with zero motor torque, and the sign of the published load torque. Compare with the code
   in `core/engine/src/transmission.cpp` and the provider hook in `multibody_provider.cpp`. Any sign, unit or
   ratio-direction mismatch is a blocker.
2. Check the tests in `core/engine/tests/test_transmissions.cpp` derive their expected values from physics, not
   from the implementation. Flag any test whose bound is just the implementation's own output.
3. Determinism (invariant 4), no raw owning pointers, braces on all control flow, no ternaries, no comments
   unless the why is non-obvious, warnings-as-errors hygiene.
4. Spec maintenance (R-18): every behavior the code adds has a cited ID; rows the diff edits in `eng.md` are
   accurate; the removed stubs in `tests/unit/test_eng_stubs.py` match exactly the IDs the Catch2 tags now cover.

M1-G (bindings):
5. Every type the Python side will need for the wave-2 integration is exposed: CoSimKernel, Party hooks
   trampolined (the five hooks incl. `read_inputs`/`write_outputs`), EndTriggers, EndReason, RunReport fields
   incl. `party_rates` and `sink_errors` if present in the base commit's headers (they may not be; note it),
   CouplingValue, TickRecord.coupling, InMemoryRecorderSink.drain, SignalLayer and components, NativeControllerRuntime,
   ChannelMap, PhysicsAdapter and PreparedModel, engine model types. Report gaps as findings, not as blockers,
   unless the binding is wrong rather than missing.
6. Error translation in `errors.cpp`: each core exception class maps to a distinct Python exception, and the
   PartyFault wrapping (ADR 0010, invariant 7) survives the crossing with its party name and tick.
7. Ownership across the binding (no raw owning pointers across any API, invariant 5 boundary: only the neutral
   model crosses; nothing engine-specific leaks into Python names).
8. `pyproject.toml` scikit-build wiring and `.github/workflows/python.yml` change: CI still builds and runs the
   binding tests; `mechsim/_core.py` import path and failure message. Note whether `[tool.uv] cache-keys` is set.
9. `tests/unit/test_bindings.py`: cases cover edges (empty runs, faults, wrong types, lifetime of objects that
   outlive the kernel).

## Build and test (one run each, do not iterate)

Builds are contended by other worktrees. Use `-j2`.
- J: `cd ../MechatronicsSimulator-m1-j && cmake --preset debug-sanitize >/dev/null && cmake --build --preset debug-sanitize -j2 2>&1 | tail -5 && ctest --preset debug-sanitize -R transm 2>&1 | tail -5`
- G: `cd ../MechatronicsSimulator-m1-g && uv run pytest -q tests/unit/test_bindings.py 2>&1 | tail -5`
  (if the extension is not built, `uv sync --reinstall-package mechsim` first; it takes up to 5 minutes).
If a build fails, report the first error and stop building; it is a finding.

Call cap: 30 tool calls.

## Report format

For each brief: verdict (accept / accept with fixes / block), then findings as a table
`| Severity (blocker/high/medium/low) | File:line | Finding | Required change |`. Then a short list of
cross-brief integration items the wave-2 integration brief must handle. Numbers of record (test counts, timing)
once. No praise, no summary of what the code does.
