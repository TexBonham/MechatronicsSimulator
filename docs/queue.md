# Dispatch queue

State for the coordinating session. A new session reads this, then `docs/rulings/`, then the two source docs.

## Current checkpoint

Spike package 1 (math core and integrator) landed as a stage commit after one fix round (the brief's step formula
lacked the -h^2 K v term; reviewer caught it). Package 2a (rigid multibody) dispatched to module-writer in worktree
`../MechatronicsSimulator-spike-p2a` on branch `wip/spike-p2a`.

## Next

1. Package 2a lands: Fable stage review, invariant audit, reference-validator, merge-back, stage commit. Carries the
   reviewer's symmetry-check item: relative tolerance, finiteness of H first, LLT from the lower self-adjoint view.
2. Package 2b (lumped beam and references: ENG-022, 036 to
   041), each the same way; 2b's commit comes back to the owner as the spike's closing commit.
3. Record per-package tokens, calls, fix rounds and defects in `docs/cost-log.md`; write the cost-model section into
   the root `CLAUDE.md` at spike close.
4. Phase 1: contracts and schemas, facade protocol ADR.

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
