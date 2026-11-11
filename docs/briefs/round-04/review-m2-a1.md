# Review: M2-A1 (sensor fidelity, sensor faults, sinusoidal detent)

Reviewer: Fable (general-purpose). Read-only. Review from the spec rows and the diff only; never read the writer's
report or transcript. Call cap 30. Do not build or run tests (one machine build slot, held by writers); read the
test code and judge whether each asserted number is derived from physics or the spec.

Worktree `/home/tex/dev/MechatronicsSimulator-04-a1`, diff `git -C <worktree> diff round/04-milestone-2...wip/04-a1`
(commit 86eecaf). Spec rows SIG-002, 003, 005, 006, 007, 008, 010, 033, 034, 036 in `docs/spec/sig.md` (including
the writer's edits to SIG-033 and the Open parameters table, which are part of the diff), PHY-034 in
`docs/spec/phy.md`. Brief of record `docs/briefs/round-04/m2-a1.md`. Read first: root `CLAUDE.md`, the brief, the
rows, `core/signals/CLAUDE.md`, then the diff.

Check:
1. Each row implemented as written and cited in a test; spec edits in the same diff (R-18).
2. Physics: the detent tangent linearization (K_t = K_hold cos(pi delta / (2 step)), rest r = theta + tau(delta) /
   K_t, clamp at 0.05 K_hold) against the implicit drive of PHY-034; the writer records that the steady lag at one
   step per tick is -8.6e-3 rad against 5.2e-3 rad for the continuous sinusoid (a sign flip, not just a magnitude
   error). Derive the one-step-per-tick case yourself and say whether this scheme is acceptable for the gantry's
   speeds (lift at 0.2 m/s with 80000 steps per metre at 1 kHz is 16 steps per tick), or whether the rest must be
   predicted from the commanded velocity, or the sinusoid must be evaluated at the predicted delta. This is the
   finding that matters most.
3. Noise, delay, sample-rate and debounce semantics and bounds (the N = 20000 bounds, `periods_ceil` rounding,
   the N-period pulse rule change), stuck versus dropped on levels and counters, forced level on digital only,
   overlapping injections refused, SIG-005 generator discipline (no draw when noise is 0).
4. Invariants 1, 4, 5, 7 and the style rules.
5. Ownership and cross-brief: the writer reports `core/controller/tests/test_controller_party.cpp:196` now fails
   on the old drive rest, that the Python binding `make_signal_component` lacks a fidelity parameter (SIG-002, 003
   Python stubs stay xfail), and that the catalog encoder's 10 kHz sample rate exceeds the 1 kHz layer. Say which
   of these the fix round should take and which belong to A2 or the integration wave.

Report, under 350 words: verdict (accept, accept with fixes, blocked), findings by severity each as `file:line,
one sentence, spec row`, owner rulings needed, cost note.
