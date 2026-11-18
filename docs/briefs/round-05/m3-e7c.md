# M3-E7c: the carried-load coupling says when its result is not valid (ruling 69, option A1, owner 2026-11-13)

Why: on the gantry's G15 emergency stop, rail_y ringing at 50 Hz drives rail_x at 49.7 Hz through the one-way
carried-joint coupling (ruling 63): rail_x stores 10.7 J where about 0.06 J is available (peak deflection 16.8 mm
against a settled 1.24 mm). The rerouted pair does work on the carrying member that the source never loses (the
carried body does not ride the deflection), so at matched frequencies it pumps, bounded only by damping. The owner
ruled A1: keep one-way coupling in milestone 3, publish what the pair does, and flag the result as not valid when
it is; full coupling is milestone 4.

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-e7` on `wip/05-e7` (first step:
`git merge wip/05-x1a` to bring in the gantry cases and the other landed work; resolve
`docs/spec/traceability.md` by regenerating with `uv run --no-sync python tools/traceability.py --write`; then
`tools/locked_build.sh sync`). Brief I4 runs concurrently in `mechsim/integrity/`, `core/integrity/`,
`mechsim/scoring/`: do not edit those. Call cap 80 with the handoff rule (at 70 calls stop adding scope, commit
what is green, report what is left). Read first: `docs/briefs/round-05/m3-e7.md`, `m3-e7b.md`,
`core/engine/include/mechsim/engine/member_loads.hpp` (header derivations), `core/engine/CLAUDE.md` (carried
loads), `mechsim/testing/CLAUDE.md` (run-end lines), rows ENG-052, PHY-035, round 5 rulings 45, 63, 65, 67 to 69
in `/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Builds in the foreground through `tools/locked_build.sh`
(600000 ms timeout; no detached jobs; ctest filter in plain double quotes). Iterate on `core debug -R
"carri|member|energy"`; end with ONE `tools/locked_build.sh core debug-sanitize -R "member|beam|carri|energy|native"`
waited for in the foreground (clang-tidy clean), then ruff check, ruff format --check, mypy,
`python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final message "WIP M3-E7c:
carried-load validity", no attribution trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`.

## Ownership

`core/engine/**`, `core/physics/native/**`, `core/bindings/src/bind_physics.cpp`, `mechsim/physics/native/**`,
`mechsim/testing/core_driver.py` (the run-end lines block only), their tests, `docs/spec/{eng,phy}.md` rows,
`docs/spec/traceability.md`, the contracts. Not yours: integrity, scoring, `designs/`, `test_cases/`.

## Items

1. Publish what the pair does, per carrying member, cumulative over the run (engine output, adapter accessor as
   values, binding): (a) the net work of the rerouted pair, the sum over steps of the node forces times the node
   displacements minus the same forces times the displacement of the parent at the same points (derive the
   discrete form consistent with the scheme and state it in the header: for a static load P that deflects the
   member by u this is P u, twice the stored elastic energy, the rest dissipated: say so); (b) the peak rerouted
   in-plane force and, from the chain's static compliance at the foot point at that time, the quasi-static
   deflection it would cause; (c) the peak carried deflection (already published).
2. The validity flag (derive in the header, this is the rule the row states): a linear member answering a load
   that is applied no faster than suddenly deflects at most twice its quasi-static value (the undamped step
   response); a carried deflection above 2 u_qs, u_qs the largest quasi-static deflection of item 1(b) up to that
   time, means the member is being driven near its own frequency by the carried system, which is exactly the
   case where the one-way pair's missing back-reaction matters. Flag the member `unresolved` from the first step
   where the dynamic ratio exceeds 2 by more than the chain's discretisation error (state the margin and why),
   and keep the peak ratio. No clamp, no change to the dynamics: the flag is information.
3. Run-end lines: `CoreDriver` adds per carrying member the pair's work, the peak dynamic ratio and, when
   flagged, a line that says the one-way carried-load result is not valid for that member from the flagged time
   and why (ruling 69), next to the carried-deflection line; the kernel-visible outputs carry the flag so the
   tier 1 party or a later brief can read it (if that needs a kernel type, do not edit `core/kernel/`: expose it
   through the adapter accessor only and say so in the report).
4. Tests, derived: a static load: work P u within the chain's tolerance, ratio 1, no flag; a suddenly applied
   load on an undamped member: ratio approaches 2 and is not flagged (the margin holds); a carried force
   oscillating at the member's first frequency for many periods: flagged, at about the step your derivation
   predicts for the amplitude growth of a resonantly driven undamped oscillator (u grows as F0 omega t / (2 k));
   the same force at half the frequency: not flagged; a two-member rig like the finding (one member's parent
   carried by another, near-equal frequencies, a stop): flagged on the carrying member; determinism.
5. On the gantry (read only: run `test_cases/gantry/G15.yaml` and `G12.yaml` on native): report for each rail the
   work, the peak ratio, whether and when it is flagged. Do not edit the cases.
6. Spec rows ENG-052 and PHY-035 state the flag and its rule; fix the ENG-052 wording the E7b review noted (the
   row says mode coupling is dropped where the code keeps it; state that the moment channel and the travel of
   the carried joint are not covered by the stability check, ruling 68).

## Report

Under 250 words: commits, filtered test counts and the sanitizer build result, the discrete work formula, the
flag's margin, each test's measured value against its derivation, G12's and G15's per-rail figures, fields added,
open issues.
