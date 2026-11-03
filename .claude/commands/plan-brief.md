---
description: "Assemble a dispatch plan for sign-off. Never dispatches."
argument-hint: <scope or queue item>
---

Assemble a dispatch plan for: $ARGUMENTS

1. Read `docs/queue.md`, the current plan, and the root `CLAUDE.md` dispatch table. Read source only through `CLAUDE.md` tables and section markers.
2. List the briefs. For each brief give: agent and model (W-01 defaults; say where a brief overrides), files owned (exact paths; no file owned by two concurrent briefs), spec requirement IDs, call cap (at most about 120 to 150, at most about six small items per brief), the exact filtered test command and the one full-suite command, and the expected cost from the dispatch table's reference sizes.
3. Concurrency: mark which briefs run in parallel, each writer in its own worktree (`git worktree add ../<repo>-<round>-<letter> -b wip/<round>-<letter>`) with its own build directory; sequence only for same-file edits, consumed outputs or machine limits.
4. Review steps per stage: oracle tests, Fable review from spec and diff only, invariant-auditor alongside (Opus at gates), then the commit candidate. Name which commits come back to the owner (gate commits, ADRs, contracts, one sampled stage commit in five).
5. Check the usage reserve (W-04): state current usage against the 90 percent stop if known.
6. End with an explicit **Assumptions** list: everything inferred, defaulted or treated as settled rather than decided by the owner.
7. Post the plan as a plain chat message with no question dialog in the same turn. Wait for an explicit dispatch go-ahead. A reply that modifies the plan is scope input: fold it in, re-post, wait again.
8. Never dispatch an agent from this command.
