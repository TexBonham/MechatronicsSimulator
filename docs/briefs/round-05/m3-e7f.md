# M3-E7f: the carried-load validity flag reads near 1 on a settled loaded rail (review of E7e at b7221c9)

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-e7` on `wip/05-e7` at b7221c9
(built and synced). Brief I5 is finishing in the integrity and scoring packages in another worktree: do not edit
those. Call cap 90 with the handoff rule (at 80 calls stop adding scope, commit what is green, report what is
left). Read first: `docs/briefs/round-05/m3-e7c.md`, `m3-e7e.md`, the validity derivation in
`core/engine/include/mechsim/engine/member_loads.hpp`, `core/engine/src/member_loads.cpp` (the window, about line
207) and `multibody_provider.cpp` (where u_qs and the ratio are formed), `core/engine/tests/test_carried_validity.cpp`,
rows ENG-052, PHY-035, round 5 rulings 69, 73, 74 in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Builds in
the foreground through `tools/locked_build.sh` (600000 ms timeout; no detached jobs; ctest filter in plain double
quotes). Iterate on `core debug -R "carri|member|energy"`; end with ONE
`tools/locked_build.sh core debug-sanitize -R "member|beam|carri|energy|native"` waited for in the foreground
(clang-tidy clean), then ruff check, ruff format --check, mypy, `python3 tools/check_imports.py`; no full pytest
run of your own. Commit as you go; final message "WIP M3-E7f: validity flag on a settled loaded member", no
attribution trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`. In spec table cells write
abs(x), never bare pipe characters.

## What is wrong

On the gantry's G15 (hold still for 8 s from straight rails with the payload on its rope, then the
maximum-acceleration move) the flag now fires with the rails' peak deflections near their static values: at 10
substeps rail_y ratio 2.59 from 8.33 s (rail_x 1.71, not flagged); at 20 substeps rail_x 2.57 from 9.36 s and
rail_y 2.87 from 8.16 s. The review's reading: for these rails the window is the 100-period cap, 7 to 9 s at 11
to 15 Hz, so the start-of-run values age out at about 8 to 9 s; after that a ratio above 2 at near-static
deflection means the hold's abs(u_qs) = abs(d_g + C_f F + C_m M) is under half the deflection at the foot. The
code gives no reason for that on a settled rail; the candidates are cancellation inside the signed sum (a moment
from an axial drive or brake force at an offset, or a centre-of-mass offset, opposing the weight term) and a
load reaching the nodes outside the three terms.

## Items

1. Diagnose first, on the gantry (read only): at 7.9 s of G15 print for each rail d_g, C_f F, C_m M, their sum,
   the foot deflection, the window's base and the ratio; repeat at 10 and 20 substeps. State the cause in one
   paragraph. If it shows a defect in the reroute or the published loads (a load reaching the nodes that the
   three terms do not account for), stop and report that with the numbers: it outranks the rest of this brief.
2. Major (ENG-052): no validity rig has gravity on. Add the gate test before changing the definition, at two
   step sizes (1 ms and 0.1 ms) and at eight and sixteen segments: a member under gravity with a standing carried
   load, released from straight, with node damping; after the settle time it reads a ratio of 1 within the
   chain's tolerance and stays there after the window has turned over at least twice; then a slow move of the
   carriage across the span keeps the ratio within the dynamic term you derive for that speed. It must fail on
   the current code if the gantry's symptom is the definition's.
3. The definition (ruling 74; derive each choice in the header, replacing what it supersedes): (a) the window is
   in physical time: its length comes from the member's node damping only (not from the scheme's own decay
   pi x, which made the memory depend on the step), with a fixed cap in periods stated in the row; say what an
   undamped member gets; (b) the ratio is formed relative to the member's static shape: subtract the own-weight
   deflection at the foot from both the deflection sample and u_qs; (c) the base has a floor: the static
   deflection of the carried weight at the current foot point (so a hanging load at rest reads 1, never a ratio
   against a cancelled sum); (d) evaluation starts at a stated time: an adapter document key (seconds, default
   0) from which the ratio and flag are evaluated and the window starts fresh, so a release from straight is not
   judged; `CoreDriver` sets it from the case's `integrity.evaluate_from` when that field exists in your tree (if
   it does not yet, add the key and its plumbing to the adapter and say in the report that the driver wiring is
   one line for the integration branch). The five E7c tests and the E7e impulse test keep their results or you
   say exactly why one changes; the bit-identity of a run that carries nothing stays.
4. Re-run G15 at 10 and 20 substeps with the evaluation start at 8 s: each rail's peak deflection, ratio and
   flag through the move and the stop; the ratio must now agree between the two step sizes within a tolerance
   you state; if a rail is still flagged, say whether the excess is physical (give the forcing: the rope's bounce
   frequency against the rail's loaded frequency) or the definition's.
5. Rows ENG-052 and PHY-035 and the run-end line state the definition and what "no flag" bounds.

## Report

Under 250 words: commits, filtered test counts and the sanitizer build result, item 1's numbers and cause, the
gate test's readings before and after, the definition in four lines, G15's per-rail figures at both step sizes,
open issues.
