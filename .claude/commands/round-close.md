---
description: "Close the round: archive cost-log rows, sweep stale CLAUDE.md sections, update the queue, verify CI, present one commit candidate."
---

Close the current round. Prepare everything; commit nothing without the owner's go-ahead.

1. Cost log: move this round's rows from `docs/cost-log.md` to `docs/cost-log-archive.md` under a dated round heading. Leave the header and any rows of work still in flight. Flag runs far above their peers in the dispatch table.
2. Contracts: list the `CLAUDE.md` files this round's agents touched (`git log --name-only <round-base>..HEAD -- '*CLAUDE.md'`). Dispatch the docs-indexer in sweep mode on them to move stale, superseded or history sections into each folder's `HISTORY.md`. Confirm no `CLAUDE.md` holds cost-log rows.
3. Update `docs/queue.md`: mark the round's items done, write the next brief, carry open questions forward.
4. Verify CI green with a command, not from memory: `gh run list --branch "$(git branch --show-current)" --limit 3`. If red or pending, stop and report.
5. Sweep `git status --ignored` for any file type new to the repo.
6. Present one commit candidate: the file list, a report of what the diff contains, and the commit message. Wait for explicit approval; "wrap up" means prepare, not commit.
