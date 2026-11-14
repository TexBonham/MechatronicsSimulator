# Review: M3-B3 (the rope payload starts at its static stretch) with M3-E8 (a rope segment taut within the step)

Reviewer: Fable (general-purpose), fresh. Read-only; spec and diff only; never the writers' reports; no builds or
runs; call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-05-e8`, branch `wip/05-e8`, commit 7951896, which
contains B3 (a28619a). Diffs: B3 `git -C <worktree> diff c0cc5f0 a28619a`, E8 `git -C <worktree> diff a28619a
7951896` (skip brief files, `docs/queue.md`). Briefs `docs/briefs/round-05/m3-b3.md` and `m3-e8.md` in that
worktree; ADR 0016 (the Slack rule row as amended), ADR 0013; rows BLD-032, ENG-025, ENG-047 and the rope rows in
`docs/spec/eng.md`, PHY-036, TST-044; `core/engine/include/mechsim/engine/rope.hpp` (Taut set), the step in
`core/engine/src/multibody_provider.cpp`, `core/engine/tests/test_rope_taut.cpp`,
`tests/reference/test_rope_payload_start.py`, `mechsim/builder/payload.py`; round 5 rulings 21, 66, 75, 86 to 88
in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`.

The defect (ruling 86, major 1): a 1 kg payload hanging on its rope at rest length put 6011 N into the rope at
12 ms (peak 26229 N) where a release at rest length gives at most 2 m g. B3 traced it (ruling 87): a segment's
slack or taut state was fixed from the start of the step and equality counted as slack, so a rope at rest length
held nothing in its first step and the load fell onto it one segment per step. B3 starts a hanging payload at
its static stretch (a builder change); E8 changes the engine's rule so a segment's state within a step is
consistent with the step's end state.

Check: (1) B3: the static stretch the builder places the payload at: derive it (the load's weight plus each
segment's share of the rope's own weight, on EA / l per segment; is the node spacing stretched too, or only the
load end) and compare with what `payload.py` computes; is it generic (any rope, any design, invariant 2) and
written as data the adapter reads, not a special case; what happens for a payload that is not hanging at t = 0
(attached later, or on a slack rope by the case): does the start stay unchanged; the oracle in
`test_rope_payload_start.py` (peak 9.90659 N against static 9.90657 N: what tolerance, and does it bracket the
scheme's own transient, ruling 21); the BLD-032 row says what the start is. (2) E8's rule as stated in `rope.hpp`:
taut exactly when the end-of-step length is at or above rest length; a segment going taut within the step pulls
with its spring alone from rest length, its damper from the next step that starts taut: is the end-of-step force
then k times the end stretch (no accumulated fall) and is the damper's omission in the first taut step justified
(the header's two-cycle argument: check it); the active-set iteration: deterministic order (invariant 4), every
inconsistent segment flipped at once, the cap `MultibodyConfig::iteration_cap`, reaching it a
`SolverNonConvergenceError` that leaves the state as it was (ENG-025, invariant 7), never a silent acceptance;
the convergence argument (one segment: two solves; a chain meeting at under 90 degrees: an M-matrix linear
complementarity problem, the primal-dual active set method monotone from the second solve, at most n + 2 solves;
bends past 90 degrees outside the proof): is the M-matrix claim right for the chain's W = J M^-1 J^T with the
stated off-diagonals, and does the test "does not settle within the cap" exercise the cap honestly (cap 1 on a
drop) rather than a constructed non-convergence; the interaction with the attachment's friction (start-of-step
direction and rate, not the set) and a mid-run rest length change (judged against the new rest length). (3) The
oracles in `test_rope_taut.cpp`: backward Euler's one-degree-of-freedom map rho^j (z_0 cos j theta + (v_0 /
omega) sin j theta), rho = (1 + x^2)^(-1/2), theta = atan x: derive it and check the asserted 1e-9 relative for
one segment; the release bracket at most 2 M g and the drop bracket M g (1 + sqrt(1 + 2 d k / (M g))) from the
energy argument; the eight-segment brackets (M = m + mu L plus 2 mu L g): is "a step response at most doubles a
static share" a bound or a heuristic; the speed at going taut within 1.5 g h of sqrt(2 g d): derive; the energy
oracle (never rising by more than 1e-12 J per step); bit-identity of a rope that stays taut (the test asserts one
pass per step: is that the same as bit-identical outputs, or should the reference values be compared); (4) the
ENG-047 row and the rope tolerance row state the rule, the cap and every bracket the tests assert (T4: tolerances
in the spec, not only in the test); the ADR 0016 Slack rule row says the same as the code; the engine `CLAUDE.md`
describes the step order with the set; `rope_passes` in `MultibodyOutputs` named and documented; (5) invariants 2,
4, 5, 7; R-18 (spec edit in the same commit as the behaviour); no gantry-specific value in either change; the
builder's start and the engine's rule together: does a payload at static stretch on the new rule start with one
pass and no transient (both fixes active at once: any double counting).

Report, under 400 words: verdict per brief (accept, accept with a fix round, reject), findings by severity as
`file:line, one sentence, row`, your reading of the convergence argument and of the brackets, and whether the
ADR 0016 sentence is right as committed (the owner confirms every ADR change at the gate).
