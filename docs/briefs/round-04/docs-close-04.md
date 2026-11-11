# Docs-indexer: round 4 close pass

Agent: docs-indexer (Haiku). Worktree `../MechatronicsSimulator-04-docs` on branch `wip/04-docs` from the round branch
at f01c3a7. Read first: root `CLAUDE.md`, the global rules in it about contract files and section markers, `docs/queue.md`
item 3 under Next (contract files over 4 500 characters), this brief. Call cap 40. No builds; run only
`uv run --no-sync pytest -q tests/unit/test_traceability_tool.py` if you touch a file it indexes (you should not).
Commit once as WIP ("WIP docs: round 4 close pass"); do not push. Change no code, no spec row, no test.

1. Section markers and line indexes: for every file over 500 lines changed in round 4 (`git diff --stat 46cbfe6...HEAD`,
   filter by `wc -l`), check that `// ===== Name =====` or `# ===== Name =====` markers exist above each major
   region and that the directory `CLAUDE.md` "Section index" line numbers are current; fix stale numbers.
   Candidates: `core/signals/src/{signal_layer,stepper,sensors,safety}.cpp`, `core/physics/mujoco/src/mujoco_adapter.cpp`,
   `core/controller/src/{controller_party,native_controller_runtime}.cpp`, `core/kernel/src/physics_party.cpp`,
   `mechsim/testing/{core_wiring,core_driver,runner,schema}.py`, `mechsim/facade/{server,service}.py`,
   `mechsim/builder/*.py`, `mechsim/design/schema.py`, `firmware/app/gantry/gantry.c`, the large test files.
2. Contract sweep (round-close hygiene): in every `CLAUDE.md` touched this round, move history, superseded
   statements and "milestone 1" caveats that no longer hold into that folder's `HISTORY.md` (create it if absent);
   keep only current contract text. Report the character count of each contract file before and after; the aim
   is every contract under 4 500 characters where the content allows (queue item 3 lists the ones over).
3. Root `CLAUDE.md`: the "Agents and dispatch" table gains the round 4 measured sizes from `docs/cost-log.md`
   (module-writer 100K to 320K per brief, 24 to 150 calls, integration briefs at the 150 cap; schema-author 144K /
   51; firmware-builder 218K / 50; Fable stage reviews 45K to 225K / 8 to 23; re-reviews 11K to 30K; Opus gate
   audits 55K to 62K; Sonnet audits 25K to 37K; reference-validator 41K / 14); the "Run the C++ build" row names the
   `debug` preset for iteration and `tools/locked_build.sh`; the "Phases and milestones" table marks M2 as gated
   when the owner confirms (leave "gate pending owner review 2026-11-11").
4. `docs/briefs/README.md`: a row for round 4 listing the briefs in `round-04/`.

Report under 200 words: files touched, contract sizes before and after, index lines corrected.
