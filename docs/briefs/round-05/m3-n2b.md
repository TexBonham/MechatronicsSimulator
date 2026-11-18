# M3-N2b: verify and finish the native adapter fix round

Handoff from brief N2 (`docs/briefs/round-05/m3-n2.md`), which reached its call cap with all six items coded and
committed on `wip/05-n` (23c5684 item 1, verified; 30c9536 items 2 to 6, the last C++ edits never compiled). Fresh
agent, same worktree `../MechatronicsSimulator-05-n`, same ownership as N2, same out-of-scope rule (the adapter's
joint friction, limit and drive terms and `core/engine/` are brief E4 and N3). Agent: module-writer (Opus). Call cap
80; at the cap commit WIP and list the rest.

Read first: `docs/briefs/round-05/m3-n2.md` (the items and their oracles), the two native `CLAUDE.md` contracts,
`core/kernel/CLAUDE.md`, then `git show --stat 30c9536` to see what the last commit touched. Do not re-derive what
N2 built; build it, run it, fix what fails.

Unverified in 30c9536, by N2's own handoff: the lumped members' node bodies at load, the `body_index` range checks,
the JSON hardening and their Catch2 tests; the item 5 tests and item 6 conversions on the new build;
`tests/reference/test_phy_native_beams.py` (the four beam values were never measured); G3 and P1 on native through
`--set physics.adapter=native` (G3 now builds flexible rails on native).

## Steps

1. `tools/locked_build.sh core debug -R "'native|Native|kernel|Kernel'"` in the foreground (600000 ms timeout,
   build and ctest as separate calls if needed; never wait on a background task without checking `ps`). Fix
   compile errors and failures. A test that fails because the implementation is wrong gets the implementation
   fixed; a test whose expectation is wrong gets the derivation corrected in the spec row first (T4), never the
   number tuned to the output.
2. `tools/locked_build.sh sync`, then the filtered pytest: `uv run --no-sync pytest -q tests/reference
   tests/unit/test_phy_stubs.py tests/unit/test_native_translation.py tests/acceptance/test_phy_stubs.py
   tests/invariant tests/unit/test_record.py tests/unit/test_runner_fakes.py`. Record the four beam reference
   values against their closed forms (cantilever deflection and first frequency at eight segments, clamped-clamped
   at sixteen; 2 percent each).
3. P1 and G3 on the native adapter through the CLI with `--set physics.adapter=native`: every criterion's value.
   If G3 no longer passes with flexible rails, do not tune the design or the case (not yours): report the failing
   criterion, its value and limit, and what moved against N's rigid-rail values (final_x 0.4000250 m, final_y
   0.2500250 m, ready_after_homing 5.245 s).
4. Small additions inside N2's ownership plus two named files: `mechsim/physics/mujoco/translate.py` refuses a
   clamped far end with a typed error instead of mapping it as free (one branch and its test);
   `mechsim/facade/service.py` `run_id_of` follows the TST-040 identity rule (the adapter suffix) with its test
   and the `mechsim/facade/CLAUDE.md` row.
5. Known and not yours: on native a joint drive on a belt shaft gives zero motion (N2 saw drive torque 0.01 n,
   velocity 0, from `driven()` in `tests/invariant/test_phy_stubs.py`). Write the smallest failing test that
   shows it as a strict xfail citing PHY-034 with the reason "brief N3", so N3 starts from it. Do not fix it.
6. End: one `tools/locked_build.sh core debug-sanitize`, one full `tools/locked_build.sh run uv run --no-sync
   pytest -q`, `uv run --no-sync ruff check .`, `uv run --no-sync ruff format --check .`, `uv run --no-sync mypy`,
   `python3 tools/check_imports.py`. Commit "WIP M3-N2b: native adapter fix round verified", no attribution
   trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`.

## Report

Under 250 words: commits, what failed on first build and how it was fixed, ctest counts on `debug-sanitize`, full
pytest counts, the four beam values against closed forms, P1 and G3 on native by criterion, open issues.
