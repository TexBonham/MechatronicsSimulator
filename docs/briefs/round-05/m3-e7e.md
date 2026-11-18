# M3-E7e: fix round on E7c and E7d (paired review at 63cd21b: accept both)

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-e7` on `wip/05-e7` at 63cd21b
(built and synced). Brief I5 runs concurrently in the integrity and scoring packages in another worktree. Call cap
90 with the handoff rule (at 80 calls stop adding scope, commit what is green, report what is left). Read first:
`docs/briefs/round-05/m3-e7c.md`, `m3-e7d.md`, `core/engine/include/mechsim/engine/member_loads.hpp` (all header
derivations), `core/engine/src/multibody_provider.cpp` around the validity base (about line 1765) and the frozen
added-mass block (about line 504), `core/engine/tests/test_carried_validity.cpp`, `test_reference_carried_mass.cpp`,
rows ENG-052, PHY-035, round 5 rulings 63, 67 to 70, 73 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`.
Builds in the foreground through `tools/locked_build.sh` (600000 ms timeout; no detached jobs; ctest filter in
plain double quotes). Iterate on `core debug -R "carri|member|energy"`; end with ONE
`tools/locked_build.sh core debug-sanitize -R "member|beam|carri|energy|native"` waited for in the foreground
(clang-tidy clean), then ruff check, ruff format --check, mypy, `python3 tools/check_imports.py`; no full pytest
run of your own. Commit as you go; final message "WIP M3-E7e: validity base, moving carried mass", no attribution
trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`. In spec table cells write abs(x), never
bare pipe characters (they break the traceability parser; the ENG-052 row was fixed for that on the integration
branch: apply the same edit here, line 84, "abs(r) <= 1/2").

## Ownership

`core/engine/**`, `core/physics/native/**` and `core/bindings/src/bind_physics.cpp` only if an output changes,
`mechsim/physics/native/translate.py` and the run-end lines block of `mechsim/testing/core_driver.py` (the
wording of the lines), their tests, `docs/spec/{eng,phy}.md` rows, `docs/spec/traceability.md`, the contracts.

## Items

1. Major (ENG-052, PHY-035): the validity flag compares the carried deflection with twice the running peak of
   the instantaneous quasi-static deflection u_qs, so one impulse-like load raises the base for the rest of the
   run and a later resonance is not flagged (`test_carried_validity.cpp` about line 375: a matched stop stays
   unflagged for about a hundred periods). Derive and implement a base that forgets: the response of a linear
   member to a load history is bounded by twice the largest quasi-static deflection over the member's memory,
   which for a damped member is a few decay times and for an undamped one must be chosen: use the peak of u_qs
   over a sliding window of a stated number of the member's loaded first periods (derive the number from the
   step-response argument: after a load is removed the free ringing it left is at most its own u_qs, so a window
   of N periods bounds what an older load can legitimately contribute once the member's damping, including the
   scheme's, has reduced it below the margin; state N and the reasoning), computed deterministically with a
   fixed-size ring buffer. State in the row and in the run-end line exactly what "no flag" then bounds: the
   deflection is within 2.04 times the windowed peak quasi-static value. Also from the review, minor: a single
   load reversal at half a period legitimately gives a ratio up to 4 without resonance (a conservative false
   flag) and gravity sag opposite to the load shrinks the base: state both in the header and the row; do not
   special-case them. Tests: the review's case (an impulse, then a matched resonance long after) is now flagged
   within the predicted time; the earlier five tests keep their results.
2. Major, gate list (ENG-052): the added-mass block is frozen at the start of the step and the step advances
   B_n (v_(n+1) - v_n), which carries velocity, not momentum, across a change of the block: as a carriage moves
   toward mid-span the ringing amplitude scales as M_eff^(+1/4) where the physics gives M_eff^(-1/4). Make the
   step carry the added mass's momentum: B_n v_(n+1) = (the momentum B_(n-1) v_n carried from the previous step)
   + h (forces), or the equivalent form that fits the solve; derive it in the header, including what is still
   dropped (the convective terms of the moving point, about 2 c kappa / omega relative: state the size for a
   carriage speed c). Oracle (derive in the test header): a member ringing in its first mode while a carried mass
   moves slowly from a support to mid-span: the action E / omega of the mode is an adiabatic invariant, so the
   amplitude scales as (M_eff)^(-1/4) with the loaded modal mass; assert the amplitude ratio between two
   positions within a tolerance you derive from the non-adiabatic correction for the chosen speed, at
   m_c = m_beam and m_c = 10 m_beam; a standing carriage is unchanged to the last bit; the energy oracle holds
   with the carriage standing; with it moving the energy change equals the work the prescribed motion does
   (state it) within tolerance.
3. Minor: a test that the member's published end loads include the added mass's inertial force -m_c u'' (a
   ringing loaded member: the sum of end shears equals the static weight plus (m_c + lumped beam mass terms)
   times the measured accelerations); the ENG-052 row states the loaded frequency holds for a parent fixed in the
   bending plane and that on a free parent one-way coupling lowers omega^2 by (1 - r0); the row states that the
   carried body's rotary inertia about the bending axis is omitted, with its relative size
   (kappa d)^2 cot^2(kappa s) for a rigid carriage at offset d, and that descendants with their own compliance
   are counted rigid.
4. On the gantry (read only): G15 at 10 and at 20 substeps: each rail's peak deflection, dynamic ratio and flag
   through the stop (the review asks whether a second step size reproduces the ratio).

## Report

Under 250 words: commits, filtered test counts and the sanitizer build result, the window length and its
derivation in one sentence, the momentum form, the adiabatic oracle's measured ratios against (M_eff)^(-1/4), the
gantry figures at both step sizes, open issues.
