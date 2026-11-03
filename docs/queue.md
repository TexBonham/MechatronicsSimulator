# Dispatch queue

State for the coordinating session. A new session reads this, then `docs/rulings/`, then the two source docs.

## Current checkpoint

Round 1 closed (phases 0a, 0c, 0b, calibration spike). Round-close hygiene staged: cost-log rows archived to
`docs/cost-log-archive.md`, root `CLAUDE.md` under 8 000 characters, section markers and indexes for the two long
tooling files, agent definitions updated from the spike's lessons. Nothing dispatched.

## Next

1. Phase 1 plan for sign-off: neutral physics model and schema, board interface header, kernel party interface, facade
   protocol ADR, pydantic models and generated schemas, remaining engine packages sequenced on the measured cost model.
2. Round 2 branch: per R2, cut a fresh branch off main after the round 1 PR lands (owner decides when to open the PR).

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
