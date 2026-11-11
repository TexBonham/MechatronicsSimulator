# Invariant audit: wave 2 (A2, B, F1)

invariant-auditor (Sonnet). Read-only; no builds or tests. Diffs against `round/04-milestone-2`:
A2 `/home/tex/dev/MechatronicsSimulator-04-a2` `wip/04-a2` 1e03870; B `/home/tex/dev/MechatronicsSimulator-04-b`
`wip/04-b` 37ba56f; F1 `/home/tex/dev/MechatronicsSimulator-04-f1` `wip/04-f1` (its head after the fix round; use
the branch tip). Nine invariants (root `CLAUDE.md`) and R-18. For F1, invariant 9 under ruling 15 (queue):
`mechsim.viewer` may be imported under `mechsim/facade/`; no Qt or OpenGL anywhere. For B, invariant 1 (controller
party receives pins and rails only). Report per brief under 200 words by `file:line`, severity, invariant.
