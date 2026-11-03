# Dispatch queue

State for the coordinating session. A new session reads this, then `docs/rulings/`, then the two source docs.

## Current checkpoint

Phase 0b round 1 (18 specs, 548 requirements, strict-xfail stubs, traceability tool, ADR 0013 proposed) staged and
reviewed; awaiting owner gate review and commit. Nothing dispatched.

## Next

1. Owner gate review of round 1: the specs (read `eng.md`, `phy.md`, `int.md` most carefully), the three boundary
   rulings below, and ADR 0013 (own-engine integrator), which must be Accepted or changed before the spike.
2. Round 2, the calibration spike: engine packages 1 and 2 against the ENG spec's "spike" rows, converting their stubs,
   through module-writer, logging tokens, calls and defects per package. Brief to be posted for sign-off after the gate.
3. Phase 1: contracts and schemas, facade protocol ADR.

## Rulings made at merge-back, confirmed by the owner 2026-11-03 with the round 1 commit

- Steppers and drivers are signal-layer models (SIG-032 to SIG-036); transmissions, ropes and lumped beams are
  engine-side neutral-model elements (ENG-043 to ENG-048). Shaft torque crosses out, load torque crosses back.
- ENG-011: the sixth joint type is the free joint (six degrees of freedom) for unattached bodies.
- Flexible-member tuning: the builder tunes (BLD-019); integrity imports FEA results into the design file (INT-034).
- Tier 1 integrity reads live loads through the kernel; everything else reads the recording (REC-001 carve-out).

## Parked

- C++ strict expected-failure convention for Catch2 stubs (ADR 0008 open item; `[!shouldfail]` is the candidate): rule in the phase 0b plan.
- Stage-review nits carried to the next docs-indexer pass: hooks and `tests/` outside mypy scope; engine package working names in `docs/milestones.md` unruled; `INV-nnn` numbering assumed equal to invariant numbers until the spec exists.

- Chrono adapter status (ADR 0004 open item): decide with the phase 1 contracts.
- C++ test markers for requirement citations (ADR 0008 open item): phase 0c.
