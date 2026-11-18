# M3-RM: the MuJoCo removal commit (ADR 0004 close; owner decision 1b, after X2i lands)

Why: the owner retires the MuJoCo bridge once X2i has landed (gate decision 1b, 2026-11-15). `docs/briefs/round-05/
mujoco-removal.md` is the written plan, extended by the gate review's additions (recorded in the gate report in
`docs/queue.md`); this brief executes it as one commit on its own branch, verified whole before landing. PHY-026:
after retirement no runtime code path imports MuJoCo; PHY-025: the bridge baselines stay in git as fixed files;
PHY-024 and the differential tests are replaced as the plan says.

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-rm` on `wip/05-rm`, cut from
`round/05-milestone-3` after X2i lands (no build: `tools/locked_build.sh sync` first; expect CMake to reconfigure
when the MuJoCo fetch leaves the tree). Call cap 120 with the handoff rule (at 110 calls stop adding scope, commit
what is green, report what is left by item). Read first: `mujoco-removal.md` whole; the gate report's removal
additions in `docs/queue.md` (`engine_artifact` in `core/kernel` `physics_adapter.hpp`, the `<mujoco/>` literals in
the kernel and native adapter tests, `mechsim/physics/__init__.py`, `mechsim/testing/core_wiring.py`, the determinism
test's docstrings, the README build notes and MuJoCo pin, `core/physics/README.md`, the `CLAUDE.md` files naming the
bridge, `.claude/agents/invariant-auditor.md`, `designs/gantry/design.yaml` line about the bridge, the PHY-015 stub);
ADR 0004; rows PHY-011, PHY-015, PHY-020 to PHY-026, TST-040, INV-020; the CI workflows; `docs/milestones.md`
retirement table. Builds and runs in the FOREGROUND only through `tools/locked_build.sh` with the Bash `timeout`
parameter at 600000 (`core debug`, then ONE `core debug-sanitize` whole; `firmware host` once; `run uv run pytest`
filtered, then the two wide runs `-m "not long_run"` and `-m long_run` once each at the end). Lints at the end.
COMMIT AFTER EACH ITEM; final message "WIP M3-RM: MuJoCo retired", no attribution trailer, no push. Do not edit
`docs/queue.md` or `docs/cost-log.md`. Curly braces on every control flow; no ternary operators; in spec table cells
write abs(x), never bare pipe characters.

## Ownership

Everything the plan names: `core/physics/mujoco/**` (deleted), the CMake targets, presets and the fetch, the CI steps
(the MuJoCo include-boundary step, the libmujoco install), the wheel and `pyproject.toml` dependency, the schema enum
value `mujoco` for `physics.adapter` (regenerated schemas), `mechsim/physics/mujoco/**` (deleted), the bridge-only
tests and the `-mujoco` test variants, the differential tests' replacement (PHY-021 to PHY-024 as the plan states:
the fixed bridge baselines stay as the comparison of record, the native baseline compared against them by the
companion test, PHY-024 retired or restated as the plan says), the run-identity suffix rule (TST-040), the
contracts and READMEs that name the bridge, `docs/milestones.md`, `docs/adr/0004-*.md` status (Decision implemented;
the owner reviews the ADR edit), the invariant auditor's agent file, the PHY-015 stub. Not yours: any case criterion,
`bridge_baselines/**` (they stay), the native adapter's behaviour, `docs/queue.md`, `docs/cost-log.md`.

## Items, in order

1. Inventory: `git grep -n -i mujoco` over the tree, every hit classified against the plan (delete, restate, keep as
   history); anything the plan lacks is added to it in the same commit.
2. Code and build: delete the bridge packages and their tests; remove the fetch, the targets, the presets' MuJoCo
   options, the `pyproject.toml` dependency and the wheel's libmujoco handling; `physics.adapter` keeps `native` only
   (a document naming `mujoco` is refused with an error naming the retired adapter: invariant 7); the run-identity
   suffix rule restated; `check_imports.py` and the import-boundary hook updated so the removed boundary stays
   enforced as "no MuJoCo import anywhere" (PHY-026 as the invariant tier's test).
3. Tests and spec: PHY-011, PHY-015, PHY-020 to PHY-026, TST-040, INV-020 restated as the plan says (R-18: rows in
   the same commit); the companion miss-list test keeps the fixed bridge baselines as the record; traceability
   regenerated.
4. Docs: READMEs, contracts, `docs/milestones.md` (retirement table: retired at <commit>, criteria as decided by the
   owner, the four G10 misses and E9 recorded), ADR 0004's status, the agent file.
5. Verification: `core debug-sanitize` whole, `firmware host`, both wide Python runs, lints, `check_imports.py`;
   `git grep -i mujoco` at the end lists only history and the fixed baselines' provenance lines.

## Report

Under 300 words: commits; the inventory counts (deleted, restated, kept with why); what `physics.adapter=mujoco`
now does; the verification counts; the final grep's residue; open issues.
