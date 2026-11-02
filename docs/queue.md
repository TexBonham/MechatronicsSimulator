# Dispatch queue

State for the coordinating session. A new session reads this, then `docs/rulings/`, then the two source docs.

## Current checkpoint

Phase 0a complete pending owner gate review of the ADR set (commit candidate below). Nothing dispatched.

## Next

1. Owner gate review: ADRs 0001-0012 and the architecture doc edits.
2. Phase 0c plan for sign-off: scaffolding (`core/` CMake tree, `mechsim/` packages, `pyproject.toml`, CI with sanitizers), the six agent definitions, hooks (import boundary, schema validation), slash commands, root and per-directory `CLAUDE.md`.
3. Phase 0b plan for sign-off: spec template, one spec per module, test stubs, traceability test; calibration spike brief (engine packages 1 and 2, pendulum and loaded-beam references) alongside.
4. Phase 1: contracts and schemas, facade protocol ADR, own-engine integrator ADR.

## In flight

None.

## Parked

- Chrono adapter status (ADR 0004 open item): decide with the phase 1 contracts.
- C++ test markers for requirement citations (ADR 0008 open item): phase 0c.
