# Invariant audit: W and W2

invariant-auditor (Sonnet). Read-only; no builds or tests. Worktree `/home/tex/dev/MechatronicsSimulator-04-w`,
branch `wip/04-w` head 12a171c, diff against `git merge-base round/04-milestone-2 wip/04-w`. Nine invariants (root
`CLAUDE.md`) and R-18. Invariant 5 (MuJoCo only under `core/physics/mujoco/`, CI grep) and 8 (firmware includes)
matter most here; the Windows platform code lives in `core/controller/src/`. Report under 200 words by `file:line`,
severity, invariant.
