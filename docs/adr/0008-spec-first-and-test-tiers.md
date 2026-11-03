# ADR 0008: Spec before code, strict-xfail stubs, traceability, spec maintenance

Date: 2026-11-02
Status: Accepted
Closes rulings: R-18, T1, T2, T3, T4
Supersedes: none

## Context
Most code will be written by agents and reviewed from the spec and the diff. The architecture doc's Validation paragraph
and the requirements doc's acceptance tests (G1 to G16) define what "correct" means, but nothing ties code to those
requirements line by line. Without that tie, behavior drifts from the documents and review has nothing fixed to check
against.

## Decision

| Aspect | Decision |
|---|---|
| Spec location | One spec per module in `docs/spec/` |
| Spec format (T2) | Numbered requirements, each traced to the source docs (R-18) |
| Stubs (T3) | Every requirement has a test stub committed as a strict expected failure before any code |
| Traceability test | Fails on any requirement without a test, and on any test citing an unknown requirement |
| Tolerances (T4) | Stated per test in the spec; changing one is a reviewed spec edit |
| Behavior changes | Cite requirement IDs and carry their spec edit in the same commit |
| Traceability matrix | Regenerated in CI |
| Stage review | Rejects a diff whose behavior is not covered by the spec as edited |

Test tiers (T1):

| Tier | Content |
|---|---|
| Unit | Per-module unit tests |
| Analytical reference | Results checked against closed-form references within spec tolerances |
| Acceptance | G1 to G16 plus the pendulum |
| Invariant | Import boundaries, schema validity of every shipped YAML file, run-twice determinism |

## Consequences
- No code lands for a requirement before its strict-xfail stub; a stub that starts passing without being converted fails
  the run.
- Spec and code cannot drift silently: CI fails on untraced requirements or tests, and review checks the spec edit.
- Tolerances are visible and changed only by review, never by editing a test.
- Several invariants become CI gates through the invariant tier (invariant 3 by schema validity, invariant 4 by run-twice determinism, invariants 5 and 9 by import boundaries).
- Milestone placement: specs and stubs are written in phase 0b, before phase 1 contracts.

## Open items
- How strict expected failures and requirement citations are expressed for the C++ tests in `core/`: not ruled; settle
  in phase 0c scaffolding.
- Spec template layout: phase 0b.
