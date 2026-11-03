# ADR 0002: In-house collision detection and v1 contact scope

Date: 2026-11-02
Status: Accepted
Closes rulings: R-05, R-06
Supersedes: Architecture doc, Technology recommendation, "Contact and collisions" row as the v1 contact implementation
(MuJoCo soft convex contact)

## Context
With an own engine (ADR 0001), collision detection and contact must be provided by the engine rather than inherited from
MuJoCo. Decisions log row 28 requires floor contact plus collisions such as a hook catching an obstacle and a carriage
hitting a hard stop. The architecture doc's Data and files paragraph already produces collision meshes by convex
decomposition (CoACD).

## Decision

| Aspect | Decision |
|---|---|
| Collision detection | Written in-house, inside the own engine |
| Reference collision library | Used only inside the test suite, as a differential oracle; never linked into the core runtime |
| Convex decomposition | Stays a library |
| v1 contact geometry | Primitives and convex hulls |
| v1 contact model | Soft contact with implicit integration |
| Mesh contact | A later tier, not in v1 |

## Consequences
- Imported meshes reach contact only as convex hulls from decomposition; designs needing true mesh contact are out of v1
  scope and are flagged, not special-cased (invariant 2).
- The Decisions log row 28 cases (floor, obstacle, hard stop) must be expressible with primitives and hulls.
- The reference collision library is a test-only dependency; the invariant tier's import-boundary test (ADR 0008) must
  keep it out of `core/` and `mechsim/`.
- Contact solver maturity gates the own engine becoming default (ADR 0004); the risk is tracked in the architecture doc's
  Risks table.

## Open items
- Which reference collision library serves as the oracle: named when the collision package is briefed (phase 1 or
  later).
- Contact solver and implicit integration detail: the own-engine integrator ADR (ADR 0001).
- Mesh contact tier: no milestone ruled.
