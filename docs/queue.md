# Dispatch queue

State for the coordinating session. A new session reads this, then `docs/rulings/`, then the two source docs.

## Current checkpoint

Calibration spike complete: packages 1, 2a and 2b staged in the main checkout as the closing commit, pending owner gate
review. Engine has 62 Catch2 tests, 7 in the reference tier (`ctest --preset debug-sanitize -L reference`). Cost model
written into the root `CLAUDE.md`. Nothing dispatched.

## Next

1. Owner gate review of the spike's closing commit; one ruling: keep N = 4 as the general lumped-beam default with
   pinned-pinned members at N = 8 (as now in the spec), or raise the default to 8.
2. Phase 1 plan for sign-off: contracts and schemas (neutral physics model, board interface header, kernel party
   interface, facade protocol ADR), plus the remaining engine packages planned on the measured cost model.
3. Docs pass: fold the reference-validator's definition feedback (C++ tier command, numbers for passing cases) and the
   stage reviewers' nits into the agent files; round-close hygiene (archive cost-log rows, sweep CLAUDE.md files).

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
