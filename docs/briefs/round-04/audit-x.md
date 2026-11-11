# Invariant audit: integration wave (gate-grade)

invariant-auditor, Opus (gate). Read-only; no builds or tests. Worktree `/home/tex/dev/MechatronicsSimulator-04-x1`,
branch `wip/04-x1`, diff `git -C <worktree> diff 58725e9...db7e1de` (X1, X2, A3, X3). Nine invariants (root
`CLAUDE.md`) and R-18. Invariant 2 matters most here (gantry-specific knowledge may appear only under `designs/`,
`test_cases/`, `firmware/`; the builder, wiring, scoring and signal layer must stay generic: look for gantry
instance names, axis counts, lift assumptions in `mechsim/` and `core/`), then 3 (every YAML validates; schemas
regenerated), 1 (controller sees pins and rails only), 4 (determinism; no clock or RNG outside the seeded paths).
Report under 300 words by `file:line`, severity, invariant.
