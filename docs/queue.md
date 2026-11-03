# Dispatch queue

State for the coordinating session. A new session reads this, then `docs/rulings/`, then the two source docs.

## Current checkpoint

Phase 0a and 0c committed and pushed; first CI run on the 0c push not yet observed. Nothing dispatched.

## Next

1. Confirm the first CI run (python and cpp workflows) is green on the 0c commit.
2. Phase 0b plan for sign-off: spec template, one spec per module, test stubs, traceability test; calibration spike brief (engine packages 1 and 2, pendulum and loaded-beam references) alongside.
3. Phase 1: contracts and schemas, facade protocol ADR, own-engine integrator ADR.

## In flight

None.

## Parked

- C++ strict expected-failure convention for Catch2 stubs (ADR 0008 open item; `[!shouldfail]` is the candidate): rule in the phase 0b plan.
- Stage-review nits carried to the next docs-indexer pass: hooks and `tests/` outside mypy scope; engine package working names in `docs/milestones.md` unruled; `INV-nnn` numbering assumed equal to invariant numbers until the spec exists.

- Chrono adapter status (ADR 0004 open item): decide with the phase 1 contracts.
- C++ test markers for requirement citations (ADR 0008 open item): phase 0c.
