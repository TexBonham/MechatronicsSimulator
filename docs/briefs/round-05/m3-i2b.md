# M3-I2b: fix round on the tier 1 integrity party (review of 81b2943), plus the facade run id

Fresh agent (the I2 transcript is at 149 calls). Agent: module-writer (Opus). Worktree
`../MechatronicsSimulator-05-i2` on `wip/05-i2` at 81b2943. First step: `git merge round/05-milestone-3` (N3 has
landed there; expect a conflict only in `docs/spec/traceability.md`, resolve by regenerating with
`uv run --no-sync python tools/traceability.py --write`). Call cap 110. Read first: root `CLAUDE.md`,
`core/integrity/CLAUDE.md`, `core/kernel/CLAUDE.md`, `mechsim/testing/CLAUDE.md`, `mechsim/integrity/CLAUDE.md`,
`mechsim/facade/CLAUDE.md`, rows INT-023 to 028, 031, KER-028, TST-040, TST-041, FAC-009 and the INT tolerance
rows, round 5 rulings 4, 34, 35, 40, 50 to 54 (`docs/queue.md`), the original brief `docs/briefs/round-05/m3-i2.md`
for the oracles. Builds in the foreground through `tools/locked_build.sh` (600000 ms timeout; build and ctest as
separate calls; another agent shares the lock; never wait on a background task without checking `ps`): iterate
with `core debug -R "'integrity|Integrity|model change|physics party'"` and filtered pytest
(`tests/unit/test_integrity_tier1.py tests/unit/test_int_stubs.py tests/acceptance/test_integrity_determinism.py`
plus the files you touch); at the end one `core debug-sanitize`, `sync`, one full
`tools/locked_build.sh run uv run --no-sync pytest -q`, ruff check, ruff format --check, mypy,
`python3 tools/check_imports.py`. Commit as you go, final message "WIP M3-I2b: integrity fix round", no attribution
trailer, no push.

## Ownership

`core/integrity/**`, `core/kernel/**` (tests and the physics party's model-change path only),
`mechsim/integrity/{tier1,config}.py` and `schemas/integrity.json` (item 4 only), `mechsim/testing/` (driver end
reason, `core_driver.py`, schema), `mechsim/record/` and `mechsim/scoring/` only where an end reason is
enumerated, `mechsim/facade/` (end reason, item 6), `schemas/` regenerated, their tests, `docs/spec/{int,ker,tst,
fac,rec,sco}.md` rows you change, `docs/spec/traceability.md`, the contracts. Not yours: `core/engine/`,
`core/physics/`, `mechsim/physics/`, `designs/`, `test_cases/`.

## Items

1. Halt (rulings 50, 51; INT-026, TST-041, invariant 7): `integrity.halt_on_crossing` is the test case's field,
   default false, whether the section is absent or empty; remove the fallback to `integrity.yaml` and the yaml
   field if it exists only for that; make the INT and TST rows agree. A halted run ends with a real end reason
   `integrity_halt` carried through `mechsim/testing/driver.py` `EndReason`, the archive, the facade's
   `RunEndReason` (protocol and schema regenerated) and scoring (the case fails, as it does on any crossing); the
   halt line (member or fastening, quantity, value, limit, time) is a report line, not an approximation. Tests:
   absent section, empty section, false, true; the end reason in the archive and through the facade; a crossing
   without halt runs to its end time and fails.
2. Kernel tests for KER-028 in the kernel's own test files, one case each: spring offset, stiffness change,
   disable, release, a refused kind (typed error, run ends), an unwired request signal (no effect, no error),
   several requests in one tick applied once and in order, the same request not re-applied on the next tick
   (ruling 40's exactly-once rule read from `PhysicsOutputs.model_changes`).
3. Tolerances into the spec (ruling 54, T4): the INT tolerance row carries the yield oracle's derived bound 2.3e-4
   relative with its derivation in one clause, and the assertion uses it; crossing ticks exact; cross-check
   against the tier 2 Python strength check 1e-9 relative; rope-snap free fall 1e-9. Add the missing tests: a
   value exactly at the limit is not a crossing and the next representable value above is (INT-024); the yield
   offset under a rising peak (one `model_change` per change, monotone), under a larger reversed peak (the offset
   takes the new peak's sign, stated in the row as the ruling 35 placeholder), and no change while the load stays
   below the running peak.
4. Rope snap gating (ruling 53, INT-031): an `integrity.yaml` tier 2 entry enables it, absent means no snap;
   schema regenerated; tests for both.
5. Driver wiring: (a) build the limits table only when the adapter is native, so an unrated fastener does not fail
   a bridge run; the bridge keeps its approximation line (rulings 29, 34); (b) remove the string sentinel and the
   `designs_root != DESIGNS_ROOT` branch that keep the FAC-009 identity test passing: make the default factory
   honest (the default driver is `CoreDriver`; where the design folder is found is an explicit argument with one
   documented default) and restate the FAC-009 assertion to what the row requires; (c) `tier1.py`'s spring order
   duplicates the adapter's: read the count and order from the prepared native model (or assert equality against
   it at build) and make an unknown joint type an error, not a zero; (d) the determinism acceptance test runs long
   enough to cross a member quantity and a yield change as well as a fastener, twice, identical (INT-027,
   invariant 4).
6. Facade run id (TST-040, carried from N3): the run request carries the case's `sets`
   (`RunCasePayload`, protocol and schema regenerated) and `mechsim/facade/server.py` passes them to `run_id_of`,
   so a run with `--set physics.adapter=native` gets its `-native` suffix through the facade; one test through
   the facade and one for the default (no suffix).

## Report

Under 250 words: commits, ctest count on `debug-sanitize`, full pytest counts, each item done or not with the test
that shows it, every protocol or schema field added (the owner reviews contracts), open issues.
