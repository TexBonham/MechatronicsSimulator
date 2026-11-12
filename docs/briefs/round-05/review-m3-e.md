# Review: M3-E1 (collision and contact) and M3-E2 (members, ropes, mid-run changes, belt compliance)

Reviewer: Fable (general-purpose). Read-only; spec, ADRs and diffs only; never a writer's report; no builds or
runs; call cap 50. This is physics review.

| Brief | Worktree | Diff | Spec rows | Brief |
|---|---|---|---|---|
| E1 | `/home/tex/dev/MechatronicsSimulator-05-e1` | `git -C <worktree> diff becf97e...wip/05-e1` (2e92c0d) | ENG-016 to 021, 035 (`docs/spec/eng.md`, tolerances marked proposed), ADR 0002, R-05, R-06 | `m3-e1.md` |
| E2 | `/home/tex/dev/MechatronicsSimulator-05-e2` | `git -C <worktree> diff becf97e...wip/05-e2` (a3638a4) | ENG-022, 030 to 034, 042, 043, 047, PHY-004, 007, 008, ACC-037 (proposal), ADR 0010, 0013, 0016, round 5 rulings 2, 10, 14, 15, 16, 21 | `m3-e2.md` |

Read first: root `CLAUDE.md`, `core/engine/CLAUDE.md` as each diff leaves it, the briefs, the rows and ADRs, then the
diffs. Both edited `multibody_provider.cpp`; their listed line ranges do not overlap, but say whether the two
merge cleanly in meaning (E1's contact terms and E2's rope terms both add to `DynamicsTerms`).

Check, in this order:
1. Ruling 21 (the finding that matters most): E2 reports that the implicit spring treatment loses about pi omega h
   of amplitude per period (1 percent per period for the rope swing at 1 ms; the gantry belt at omega h = 0.32 runs
   3.2 percent low in frequency and heavily damped), so ACC-037's numbers only hold at a 50 microsecond step.
   Derive the amplitude decay of the engine's step for a linear spring-mass (the `(H + hD + h^2 K) dv = ...` form in
   the contract) and confirm or correct E2's number; then judge the proposal: stiffness and damping terms by the
   implicit midpoint (trapezoidal) rule as the default (second order, no numerical damping for linear springs,
   unconditionally stable), with semi-implicit Euler selectable. State what it changes for contact (E1's penalty
   springs: midpoint can ring on stiff contacts; say whether contacts should keep backward Euler while structural
   springs use midpoint, and how that split is expressed in `DynamicsTerms`), for the stepper drive on the native
   adapter (PHY-034 exactness claims), and for the spike references (ENG-036 to 041, which were passed with the
   current scheme). Recommend one of: adopt midpoint for structural and rope springs only; adopt it for all; keep
   backward Euler and substep ropes and beams. This recommendation goes to the owner as the ADR 0013 amendment.
2. E1 physics: GJK and EPA correctness on the closed-form set, the single contact point for box-box and hull-hull
   (a box resting on a box tips without a manifold: is that acceptable for milestone 3, where the gantry's contacts
   are payload-on-floor and limit stops?), the implicit contact terms (K and D blocks per contact, never adhesive,
   friction regularization and its threshold), the four references' derivations and the proposed tolerances, the
   weld as a penalty spring (E1 could not touch the Constraints section): is a penalty weld acceptable for the
   payload, or must it become a hard constraint (E2 added constraint disable; the merge could unify them)?
3. E2 physics: member end-loads from the chain (root moment P L, shear P), the mid-run API, the Euler buckling
   eigenvalue test (asserted at 2 percent against a 5 percent proposal: which is right at N = 8?), the rope chain
   (translation-joint nodes, tension-only springs, slack rule fixed per step, geometric stiffness, snap), the
   added-mass closed form the pendulum test uses, the attachment damping (ruling 14), the snatch flag (E2 used the
   rope's fundamental sqrt(EA / (L M)), not the segment frequency; judge which ruling 15 should mean), the belt
   compliance reference, the root-spring offset relation (E2 says (L - l/2) phi, not offset times L: who is right).
4. Invariants 4, 5, 7 and style; ownership (E1's binding registration lines; E2's new `JointType::Translation` not
   bound); R-18.
5. What brief N (native adapter) must know from these two diffs: the per-segment tension to `RopeTension` mapping,
   the Translation joint binding, the clamped-clamped layout (ruling 16), contact parameters from the neutral
   `ContactModel`, the payload attach API.

Report, under 600 words: verdicts, the ruling 21 derivation and recommendation first, findings by severity as
`file:line, one sentence, spec row`, owner rulings needed, cost note.
