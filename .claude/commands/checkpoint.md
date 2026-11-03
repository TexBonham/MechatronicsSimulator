---
description: "Pause protocol (W-03): finish in-flight briefs, park worktrees, update log and queue, confirm pushed and green."
---

Bring the pipeline to a checkpoint (W-03). Start no new brief.

1. Let in-flight briefs finish; dispatch nothing new.
2. For each worktree (`git worktree list`): merge it back if its stage is verified and authorized, otherwise park it and add a queue note naming the branch, its state and the next step.
3. Update `docs/cost-log.md` with every finished run and `docs/queue.md` with the next brief written in full.
4. Confirm the stage commit is pushed (`git status -sb` shows no ahead count) and CI is green (`gh run list --branch "$(git branch --show-current)" --limit 1`). Any commit beyond an authorized stage commit comes back for review first.
5. Report the checkpoint in exactly five lines: branch and last commit; CI state; worktrees merged or parked; cost-log and usage state against the 90 percent reserve; next brief in the queue.
