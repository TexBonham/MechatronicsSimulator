# Dispatch queue

State for the coordinating session. A new session reads this, then `docs/rulings/`, then the two source docs.

## Current checkpoint

Round 1 merged to main (PR #1, merge commit feda82d). Round 2 on branch `round/02-phase-1`. ADR 0014 (MuJoCo bridge
split) accepted. Phase 1 wave 1 (C1 neutral model, C2 component and material files, C5 core runtime interfaces, C6
board interface, C7 facade protocol ADR) dispatched in worktrees.

## Next

1. Wave 1 lands: Fable stage review per brief, invariant audit, merge-back, stage commits.
2. Wave 2: C3 design file and CAD import contract (needs C2). Wave 3: C4 test case and baseline files (needs C3).
3. Phase 1 closing commit comes back to the owner with ADR 0015 (facade protocol) and every schema.
4. Milestone 1 plan on the measured cost model; engine package 7 (transmissions) first.

## Commit trailers (owner ruling 2026-11-03)

Owner-reviewed commits carry `Reviewed-by: Tex Bonham <bo1750te0130@gmail.com>`. Model-gated stage commits carry
`Stage-review: Fable stage reviewer, invariant-auditor`. PR bodies carry no generated-by lines.

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
