---
description: "Audit the current diff (invariants, import boundaries, full suite once) and summarize by severity. Changes nothing."
---

Audit the current working-tree diff. Change nothing.

1. Capture the scope: `git status --short`, `git diff --stat`, `git diff --staged --stat`. Stop if both diffs are empty.
2. Dispatch the invariant-auditor on `git diff` plus `git diff --staged`. Give it the diff range, the spec files for the cited requirement IDs, any files under active rework, and a call cap of 40. Do not pass it any writer summary.
3. Run the import checker once over the changed Python files: `python3 tools/check_imports.py $(git diff --name-only --diff-filter=d HEAD -- '*.py')`. If the checker is absent, say so.
4. Run the full test suite exactly once: `uv run pytest -q`, plus `ctest --preset <host>` if `core/` or `firmware/` changed.
5. Summarize in one table ordered blocker, major, minor: `file:line`, severity, invariant or requirement ID, evidence. Quote each test count once, as the current state.
6. Stop. Fixes go through `/plan-brief`.
