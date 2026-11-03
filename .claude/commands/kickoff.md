---
description: "Check the repo against the global kickoff checklist and report gaps; fixes nothing."
---

Audit this repository against the kickoff checklist in the global rules. Read only; fix nothing without sign-off.

1. Root `CLAUDE.md`: confirm it holds tables, not prose, for key constants with exact values; data layout and field maps (byte offsets where binary); a "Where to Find X" jump table (task, then files to edit in order); the full enum, opcode or slot tables for the domain; the dispatch table with reference per-run sizes; and a one-line pointer to `docs/cost-log.md`.
2. Confirm no `CLAUDE.md` at any level holds cost-log rows (W-07): `grep -rln "| Tokens" --include=CLAUDE.md .`.
3. Per-directory contracts: for each major subsystem folder (`mechsim/*/`, `core/*/`, `firmware/`, `schemas/`, `tools/`), confirm a `CLAUDE.md` with constructor signatures, key methods (one line each) and conventions. List folders that lack one.
4. Section markers: list source files over 500 lines (`find mechsim core firmware tools -name '*.py' -o -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.hpp' | xargs wc -l | sort -n`). For each, check for `===== ` markers and a matching line index in its directory's `CLAUDE.md`.
5. Confirm `docs/cost-log.md` and `docs/cost-log-archive.md` exist with the standard header.
6. Report as a gap table: item, location, status (present, missing, stale), suggested owner (docs-indexer or inline). Then stop. Any fix is a brief that goes through `/plan-brief`.
