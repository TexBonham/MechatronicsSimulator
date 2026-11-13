# M3-E7d: a carried body's mass loads the member's inertia (ruling 70 b)

Why: the one-way carried-joint pair (ENG-052, ruling 63) puts the carried body's weight and inertial reaction on
the member's nodes but not its mass. The member therefore deflects correctly under a static load, but rings at
its unloaded frequency: the gantry's rail_x rings near 50 Hz where, loaded with its carriage, it would sit near
15 Hz, and that false coincidence with rail_y is what let one rail pump the other at the stop (ruling 69). The
missing term is the inertial part of the back-reaction: a body of mass m_c that rides a point of the member whose
deflection is u feels m_c u'' in addition to what the rigid parent gives it. Ruling 70 (b): add that term to the
member, keep the position coupling one-way.

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-e7` on `wip/05-e7`, after brief
E7c (validity flag) is committed there. Call cap 110 with the handoff rule (at 100 calls stop adding scope,
commit what is green, report what is left). Read first: `docs/briefs/round-05/m3-e7.md`, `m3-e7b.md`, `m3-e7c.md`,
`core/engine/include/mechsim/engine/member_loads.hpp` (all header derivations), `core/engine/CLAUDE.md`
(carried loads, the mass matrix assembly, the step order), `core/engine/tests/test_carried_*.cpp`,
`energy_helpers.hpp`, rows ENG-052, PHY-035, round 5 rulings 63, 65, 67 to 70 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Builds in the foreground through `tools/locked_build.sh`
(600000 ms timeout; no detached jobs; ctest filter in plain double quotes). Iterate on `core debug -R
"carri|member|energy"`; end with ONE `tools/locked_build.sh core debug-sanitize -R
"member|beam|carri|energy|native"` waited for in the foreground (clang-tidy clean), then ruff check, ruff format
--check, mypy, `python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final message
"WIP M3-E7d: carried mass on the member", no attribution trailer, no push. Do not edit `docs/queue.md` or
`docs/cost-log.md`.

## Ownership

`core/engine/**`, `core/physics/native/**` only if a field must cross, `mechsim/physics/native/translate.py` (the
approximation line), their tests, `docs/spec/{eng,phy}.md` rows, `docs/spec/traceability.md`, the contracts, and
one sentence added to the ADR 0010 bullet E7 wrote. Not yours: integrity, scoring, testing, `designs/`,
`test_cases/`.

## The model (derive it in the header before coding; the form below is the intent, not a formula to copy)

For each carried joint, with m_c the mass of the carried subtree and the lever weights w_k, w_{k+1} of the foot
point on the two adjacent nodes: the member's equations gain the inertia of a point mass m_c attached at the
foot point for motion in the bending direction relative to the parent, that is the consistent 2 by 2 block
m_c [w_k^2, w_k w_{k+1}; w_k w_{k+1}, w_{k+1}^2] on the two nodes' in-plane transverse coordinates, mapped
through the nodes' Jacobians into the engine's generalized mass matrix. It adds inertia only: no added weight
(the rerouted reaction already carries m_c g) and no added inertia for the parent's own motion (the parent's
solve already holds the carried subtree). Check these three statements by the tests, do not assume them. The
weights move with the carriage; say how the block is treated within a step (frozen at the start-of-step
position) and what that costs. Rotary inertia of the carried body about the bending axis: state whether it is
included (the moment channel) and why. What this does to the lag of ruling 67: re-derive the recursion and the
stability boundary with the added mass (the fixed-in-plane parent stays feed-forward; say what happens to
r > (1 + x^2) / 2 on a free parent) and update the refusal accordingly. What it does to E7c's quasi-static
reference and flag: unchanged in form; say so or correct it.

## Tests (derived oracles)

1. Frequency of a loaded member, undamped, small amplitude, eight and sixteen segments: a point mass m_c at
   mid-span of a pinned-pinned beam with distributed mass: omega^2 = k_mid / (m_c + 0.4857 m_beam) with
   k_mid = 48 E I / L^3 (the standard one-mode result; derive the factor or cite it in the header) within 2
   percent at m_c / m_beam = 0, 1 and 10; a cantilever with a tip mass: k = 3 E I / L^3, effective beam mass
   0.2357 m_beam; a clamped-clamped beam with a centre mass: k = 192 E I / L^3, 0.3714 m_beam. Measure the
   frequency by zero crossings over at least twenty periods at a step where atan(x) / x is within 0.1 percent.
2. Statics unchanged: every static oracle of `test_reference_carried.cpp` still holds to its stated tolerance
   (no added weight): run them, do not weaken any.
3. The parent is unchanged: a free parent accelerated by a force moves as before with the carriage on a rigid
   member (total mass m_p + m_c, not m_p + 2 m_c), within 1e-9.
4. Energy: E6's oracle with the member spring energy and the added-mass kinetic energy holds on the carried rigs
   with no drive and no input.
5. The two-member rig of E7c (one member's parent carried by another, a stop): with the mass on the members the
   carrying member's frequency drops as item 1 predicts and the E7c flag is not raised where the frequencies no
   longer coincide; make a second rig where the loaded frequencies do coincide and show the flag still catches
   it.
6. Determinism (run twice identical); a model whose members carry nothing is bit-identical to before.
7. On the gantry (read only; merge `wip/05-x1a` first if brief I4 has been merged there, otherwise run the cases
   as they are): report each rail's loaded first frequency against the item 1 estimate, and G15's per-rail peak
   deflection, pair work, dynamic ratio and flag through the stop, before and after this change.

## Report

Under 300 words: commits, filtered test counts and the sanitizer build result, the derived block and its
treatment within a step, the re-derived stability boundary, each oracle's measured value against its closed form,
the gantry's per-rail figures before and after, the ADR sentence, open issues.
