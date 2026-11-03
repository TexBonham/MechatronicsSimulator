# ADR 0009: Checks as plugins; v1 check set and milestone placement

Date: 2026-11-02
Status: Accepted
Closes rulings: R-19, R-20
Supersedes: Architecture doc, Repository starting point, `structural/` module ("beam and plate checks, safety factors,
load-case export"), renamed `integrity/` with the scope below

## Context
The architecture doc's `structural/` module computed beam and plate checks and safety factors from member loads. The
requirements doc's milestone 3 calls for structural checks and milestone 4 for FEA load export. A general product
(R-01) needs a check set that grows without code changes per design, and a scope that does not drift into
finite-element structural analysis, which the engine and external tools cover.

## Decision
Check model (R-19):

| Aspect | Decision |
|---|---|
| Form | Checks are plugins, like component types |
| Definition | Data-driven |
| Tiers | Tiered, as component plugins are (ADR 0005) |
| Verification | Each check ships with an analytical reference |
| Module scope | Mechanical integrity, not structural analysis |
| Module | `mechsim/integrity/` |

v1 check set and placement (R-20):

| Milestone | Checks |
|---|---|
| 3 | Strength, deflection, buckling, fatigue, fastener, impact, resonance screening |
| 4 | Bearing life |
| 4 | Build-time design rules: support spacing, cantilever ratio, clearance, misalignment binding, tip-over |
| Seams only | Thermal, wear |

## Consequences
- A new check is a plugin class, its schema entry and a reference test; the architecture doc's "How to extend" table
  gains that row.
- Every check's correctness is anchored to an analytical result within a spec tolerance (ADR 0008).
- Build-time design rules run on the design before a run, so they can fail a design without simulating it.
- Thermal and wear are slots in the data model only; no implementation in v1.
- In-loop evaluation of a subset of these checks during a run is ADR 0010.

## Open items
- Minimum safety factor and the G12 limits: owed by the gantry team (parameters until ruled).
- Thermal and wear check implementations: roadmap, no milestone.
- The architecture doc's Seams table lists a thermal seam but no wear seam: add when the Seams table is next revised.
