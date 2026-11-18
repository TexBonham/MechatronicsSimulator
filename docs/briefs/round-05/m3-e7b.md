# M3-E7b: fix round on E7 (review of 5ab0704): the lagged pair's stability on a free parent

Fresh agent (the E7 transcript is long). Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-e7`
on `wip/05-e7` at 5ab0704. Brief X1d runs concurrently on the gantry data in another worktree. Call cap 70 with the
handoff rule (at 60 calls stop adding scope, commit what is green, report what is left). Read first:
`docs/briefs/round-05/m3-e7.md`, `core/engine/include/mechsim/engine/member_loads.hpp` (the header derivation),
`core/engine/tests/test_energy.cpp`, `energy_helpers.hpp`, `test_carried_loads.cpp`, rows ENG-052, PHY-035.
Builds in the foreground through `tools/locked_build.sh` (600000 ms timeout; no detached jobs; a ctest filter in
plain double quotes: `-R "member|beam|carri|energy|native"`); the worktree is built. Iterate on `core debug`; end
with ONE `tools/locked_build.sh core debug-sanitize -R "member|beam|carri|energy|native"` (clang-tidy clean), then
ruff check, ruff format --check, mypy, `python3 tools/check_imports.py`; no full pytest run of your own. Commit as
you go; final message "WIP M3-E7b: carried-load stability", no attribution trailer, no push. Do not edit
`docs/queue.md` or `docs/cost-log.md`.

## Ownership

As E7, plus `mechsim/testing/core_driver.py` for item 3 only (one block beside the existing run-end lines).

## Items

1. Major (ENG-052): the header claims the lagged pair "converges for any mass ratio". The review's derivation:
   the parent's momentum row is untouched; the lag sits in the member's bending mode as
   alpha u''_n - beta u''_(n-1) + k u = 0 with beta / alpha = r = m_c / (m_p + m_c) (m_c the carried subtree's
   mass, m_p the parent's effective mass along the member's bending direction). The lag is a negative damping: an
   undamped member on a parent that is free in the bending plane grows when m_c > m_p (about 8 percent per step
   at r = 0.9, k h^2 / alpha = 1e-3). Do this: (a) derive the per-step growth factor of that mode under the
   engine's scheme as a function of r and x = omega h, with the member's node damping, and state the stability
   boundary in the header (replace the wrong claim); (b) at load, for each carried joint, compute whether the
   parent can move along the member's in-plane transverse direction at the load configuration (the inverse
   inertia seen there, from the mass matrix: zero means fixed in the plane, pure feed-forward, always safe) and,
   when it can, the ratio r; refuse the model (`InvalidModelError` naming the member, the joint and r) where your
   derived boundary says an undamped member can grow (ruling 67: refuse, do not clamp or filter); below the
   boundary run and publish r and the worst-case growth or decay factor as an output the translation prints; (c)
   say in the row that the check is made at the load configuration and that full (implicit) coupling is milestone
   4.
2. Major (ENG-052): the only moving-parent rig has r = 2 / 7, and the per-step energy inequality holds for any
   applied force, so it does not test the lag. Add: a free-parent rig just below the boundary, undamped, run for
   at least a thousand member periods with the member's modal amplitude bounded by your derived factor (assert the
   envelope, not a per-step inequality); a rig above the boundary is refused; a rig above the boundary with the
   check disabled through a test-only seam (or the recursion evaluated directly) shows the growth rate your
   derivation predicts within 5 percent, so the derivation is tested and not only the refusal.
3. Minor (PHY-035): the translation line promises the run's largest deflection at a carried joint, but nothing
   outside tests reads `carried_deflection()`: at the end of a run `CoreDriver` adds one line per carrying member
   with that value (beside the driven-mode lines), with a test through a real run.
4. Minor (PHY-035): a committed test that a model whose members carry nothing records identically with the
   field absent and with an empty list (the no-carry path inert), replacing the scratch check.
5. Minor (ENG-052): the tolerance cell states the chain's error for an off-centre load,
   -L^2 / (4 N^2 a b) at a segment boundary, so the 2 percent row is a statement about position: at eight
   segments it holds only for a load within about [0.27 L, 0.73 L].

## Report

Under 200 words: commits, filtered test counts and the sanitizer build result, the derived stability boundary and
growth factor, the refusal's threshold, the three rigs' results against the derivation, each item done or not,
open issues.
