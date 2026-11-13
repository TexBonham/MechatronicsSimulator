# ADR 0010: In-loop detection and modeled consequences of structural failure

Date: 2026-11-02
Status: Accepted
Closes rulings: R-21
Supersedes: none

## Context
The architecture doc computes stress from member loads after the fact (Technology recommendation verdict; `structural/`
in the layout). The requirements doc's milestone 3 adds rope swing, flexible members and structural checks. A member that
yields, buckles or loses a fastener mid-move changes what the controller sees, so post-run checks alone cannot show
whether the firmware handles the failure.

## Decision
Tier 1, detection:

| Aspect | Decision |
|---|---|
| Where | A kernel party, evaluated every tick |
| Inputs | Live loads from physics |
| Quantities | Member stress, buckling margin, fastener load, deflection |
| On a limit crossing | Raise a timestamped event; fail the case; optionally halt the run |

Tier 2, modeled consequences:

| Failure | Model change |
|---|---|
| Yield | Sets a permanent spring offset |
| Buckling | Collapses stiffness |
| Fastener break | Disables its constraint |
| Rope snap | Drops the load |

| Milestone | Scope |
|---|---|
| 3 | Tier 1 and multi-segment beams |
| 3 and 4 | Tier 2, spread across both |

Interface requirement: physics publishes live member end-loads every tick.

## Consequences
- Every adapter that carries milestone 3 or later publishes member end-loads every tick (ADR 0003); the own engine is
  designed for it from the start (ADR 0001).
- Tier 2 requires the physics interface to accept model changes mid-run: spring offsets, stiffness changes, constraint
  removal and load release.
- Failure events are part of the recording and must be deterministic (invariant 4).
- The tier 1 party follows the plugin tiers of ADR 0005 and ADR 0009: a check may start on the Python slow path and be
  promoted to C++.
- Rope snap depends on the rope model of the milestone 3 ADR (ADR 0003, P4), now ADR 0016.

## Open items
- Split of tier 2 failure modes between milestones 3 and 4: not ruled; set in the milestone 3 plan. Closed by the
  2026-11-11 amendment.
- Rope model: milestone 3 ADR. Closed: ADR 0016.
- Where the optional halt is configured (test case or design) and its default: not ruled; phase 1 schemas. Closed by
  the 2026-11-11 amendment.
- A moving load on a flexible member (round 5 ruling 63, amended 2026-11-13, brief M3-E7). Decision for milestone 3:
  load-path coupling. A flexible member names the joints it carries; each step the engine moves the carried joint's
  in-plane transverse force and bending moment (its previous step's reaction, the static weight on the first step)
  onto the two member nodes around the joint's foot point by the lever rule and takes the same off the member's
  parent, so end loads, stress and deflection see the load. Approximation: the carried body moves with the parent
  and does not ride the deflected member; the position error, the member's deflection at the carried body, is
  published per member, and out-of-plane bending stays rigid (ruling 65). Milestone 4 alternative: full coupling,
  the carried body riding the deflected member through a sliding constraint between node bodies.

## Amendment 2026-11-11: open items closed (round 5 rulings 3 and 4)
Decisions, sourced to the round 5 rulings in `docs/queue.md` at their planned defaults; the owner confirms at the
milestone 3 gate.

| Item | Decision | Source |
|---|---|---|
| Tier 2 split, milestone 3 | Yield: permanent spring offset (ENG-031). Rope snap: load release (ENG-034), with the rope model and snap rule of ADR 0016 | Round 5 ruling 3 |
| Tier 2 split, milestone 4 | Buckling: stiffness collapse (ENG-032). Fastener break: constraint disable (ENG-033). The engine calls land in milestone 3 with the other mid-run changes; their use by integrity is milestone 4 | Round 5 ruling 3 |
| Halt configuration | Test-case field `integrity.halt_on_crossing`, boolean, default false; true halts the run at the crossing, false lets it continue (INT-026) | Round 5 ruling 4 |
| Case result on a crossing | The case fails on a tier 1 crossing whether or not the run halts (INT-025) | Round 5 ruling 4; Decision, Tier 1 |

The open items "Split of tier 2 failure modes", "Rope model" (ADR 0016) and "Where the optional halt is configured"
are closed by this amendment.
