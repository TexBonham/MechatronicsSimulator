# ADR 0013: Own-engine integrator

Date: 2026-11-03
Status: Accepted (owner ruling 2026-11-03)
Closes rulings: P2 (own-engine half); ADR 0001 open item "Own-engine integrator"; ADR 0002 open item "Contact solver and
implicit integration detail"
Supersedes: none

## Context
P2 makes the integrator a per-adapter choice and leaves the own engine's integrator to an engine ADR, which must exist
before engine package 1 is briefed (ADR 0001, Open items). The MuJoCo adapter runs implicitfast at 1 ms (ADR 0003, P2).
The constraints on the own engine's choice:

| Constraint | Source |
|---|---|
| Configured physics step (1 ms default), equal to the 1 kHz control tick by default | ADR 0011, C4; Architecture doc, Data flows |
| Stiff lumped springs: flexible members as multi-segment lumped beams | R-13; ADR 0010 |
| Soft contact with implicit integration | R-06; ADR 0002 |
| Kinematic loops closed by bilateral constraints, with drift reported as an error | ADR 0001 (package 2); invariant 7 |
| Bitwise determinism on the same machine | Invariant 4 |
| Faster than real time preferred for batches | Requirements doc, Physics and structural analysis requirements |
| Mid-run stiffness changes and constraint removal (tier 2 failure) | ADR 0010 |

A lumped beam segment of stiffness k and mass m has a natural frequency sqrt(k/m), and k grows and m shrinks as the
segment count N rises, so the stiffest segment mode scales roughly with N squared. The stability argument is therefore
made against the configured physics step h (KER-004; 1 ms by default, but run configuration) and the segment count of
each lumped beam (BLD-018), not against a fixed 1 ms: for short steel or aluminium segments at the default step and a
realistic N, omega times h can exceed the explicit stability limit (about 2) by an order of magnitude, and a smaller
step or a finer segmentation moves the limit, not the shape of the problem. Soft contact has the same shape: a stiff
spring-damper normal force. The integrator must therefore stay stable for stiff spring-damper and contact terms at the
configured step and segment count without shrinking the step.

## Options

**A. Semi-implicit Euler with implicit treatment of spring-damper and contact forces (the same semi-implicit shape
as MuJoCo's implicitfast).** Velocities are updated with the stiffness and damping Jacobians of springs, dampers and contacts folded into
the mass matrix (one linearized implicit step), positions then follow from the new velocities. Constraints, including
loop closures and limits, are solved at the velocity level each step with position stabilization. One linear solve per
step on a fixed sparsity pattern; no Newton loop, so the cost per step is fixed and the arithmetic sequence is the same
on every run. Damps high-frequency segment modes numerically, which lowers the apparent first mode slightly for coarse
segmentations; the beam references (ENG-039, ENG-041) bound that error.

**B. Fully implicit (backward) Euler with Newton iterations.** Solves the nonlinear implicit step to a residual
tolerance. Unconditionally stable and handles large rotations and nonlinear springs (plastic offsets, collapsing
stiffness) exactly at the step. Costs several factorizations per step, and the iteration count varies with state, so
determinism needs a fixed iteration cap and a fixed convergence test; a non-converged step must end the run (ENG-025).
Strong numerical damping, larger than option A.

**C. Newmark or generalized-alpha.** The structural-dynamics standard: second-order accurate with user-controlled
high-frequency dissipation, so beam modes keep their frequency and amplitude better than either Euler variant. Designed
for linear or mildly nonlinear structures; combining it with velocity-level contact and constraint impulses is less
established, and with nonlinear terms it needs Newton iterations like option B. Best fit if structural fidelity of the
lumped beams becomes the deciding requirement.

**D. Explicit RK4.** Fourth-order accurate and simple, but conditionally stable: with stiff lumped segments and soft
contact at the default step it either diverges or forces substeps of tens of microseconds, defeating the configured step and the
faster-than-real-time goal. Contact and limit events are discontinuous, which voids its order advantage. Rejected for
contact and stiff members; it remains useful as a test oracle for smooth, non-stiff cases (free rigid pendulum).

| Criterion | A. Semi-implicit, implicit springs and contact | B. Implicit Euler, Newton | C. Newmark or generalized-alpha | D. Explicit RK4 |
|---|---|---|---|---|
| Stable at the configured step and segment count, with stiff segments and soft contact | Yes | Yes | Yes | No |
| Cost per step | One linear solve, fixed | Several solves, variable | One to several solves | Four force evaluations, but substeps needed |
| Determinism | Fixed sequence | Needs fixed cap and test | Needs fixed cap if nonlinear | Fixed sequence |
| Accuracy of beam first mode | First order, mild damping | First order, more damping | Second order, tunable damping | High, when stable |
| Fit with velocity-level contact and loop constraints | Native | Workable | Less established | Poor (events) |
| Proven at this scale and step | MuJoCo implicitfast (same shape, soft constraints) | Common in multibody codes | Common in structural codes | Not for contact |

## Decision
Ruled by the owner on 2026-11-03:

| Item | Ruling |
|---|---|
| Integrator | Option A: semi-implicit Euler with implicit treatment of spring, damper and contact forces, at the configured step. Chosen for fixed per-step cost and determinism without a convergence loop. |
| Position stabilization | Post-step projection of positions onto the constraint manifold, not Baumgarte correction, to avoid stabilization-induced jitter. |
| Constraint solver | Projected Gauss-Seidel at the velocity level with a fixed iteration cap; the cap is the solver iteration limit of ENG-025 and a non-converged step ends the run. |
| Later tier | Option C (Newmark or generalized-alpha) is reserved as a later integrator tier behind the same interface (ENG-003) if the lumped beam references cannot be met by refining the segmentation. |
| Thresholds and tolerances | The drift threshold (ENG-024) and beam reference tolerances (ENG-036 to ENG-042) take the proposed values in `docs/spec/eng.md` as current values; each is a spec parameter changed only by a reviewed spec edit (T4). |
| Lumped beam segments | Default four segments per flexible member, overridable per member in the design file (BLD-018). |

Option A has the same semi-implicit shape as MuJoCo's implicitfast, which keeps the differential oracle (ADR 0004)
close to comparing like with like; MuJoCo's constraints are soft rather than velocity-level with projection, which the
differential comparison must account for. Option D stays a test oracle for smooth, non-stiff cases only.

## Consequences
- Engine package 1 is briefed against option A once ruled; the integrator sits behind an engine-internal interface so a
  later tier can be added without changing callers (invariant 6 applies to any added tier).
- The beam first-mode references (ENG-039, ENG-041) measure the numerical damping; their tolerances decide whether
  option C is ever needed.
- Constraint drift is bounded by position stabilization and reported every step (ENG-024, ENG-026); drift past the
  threshold ends the run.
- Mid-run stiffness changes (ENG-032) update the implicit Jacobian at the next step; no restart is needed.
- No Newton loop means no convergence-dependent branch in the step; the solver iteration limit (ENG-025) applies only to
  the constraint solver.

## Open items
- None for the integrator. The projection step's convergence test and ordering relative to the contact solve are
  engine package 1 design details, fixed in that package's brief.
