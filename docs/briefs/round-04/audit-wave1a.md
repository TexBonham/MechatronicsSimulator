# Invariant audit: wave 1a (G, C, A1)

invariant-auditor (Sonnet). Read-only; do not build or run tests. Three diffs against `round/04-milestone-2`:

| Brief | Worktree | Branch | Commit |
|---|---|---|---|
| G | `/home/tex/dev/MechatronicsSimulator-04-g` | `wip/04-g` | ab1f65e |
| C | `/home/tex/dev/MechatronicsSimulator-04-c` | `wip/04-c` | f29a552 |
| A1 | `/home/tex/dev/MechatronicsSimulator-04-a1` | `wip/04-a1` | 86eecaf |

`git -C <worktree> diff round/04-milestone-2...<branch>` for each. Audit against the nine invariants in the root
`CLAUDE.md` and the spec-maintenance rule (R-18: a behaviour change cites its row and carries the spec edit in the
same diff). Report per brief: pass or findings by `file:line`, severity, invariant number. Under 250 words.
