# M3-D2c: finish the D2 fix round (D2b stopped at its cap)

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1a` on `wip/05-x1a` at aba358c
(built and synced). There is one uncommitted file left by D2b, `tests/unit/test_damping_run.py` (4 pass, 1
fails): it is yours. Call cap 70 with the handoff rule (at 60 calls stop adding scope, commit what is green,
report what is left). Read first: `docs/briefs/round-05/m3-d2b.md` (the items and ruling 80; you finish what is
listed below), `review-m3-d2.md`, `mechsim/builder/run_damping.py`, `mechsim/builder/structure.py`
(`_member_system`), `mechsim/testing/core_driver.py` (`steps_before`, the run-end lines), rows DES-047, BLD-040,
BLD-041, LIB-035, PHY-036, PHY-037. Filtered pytest in the foreground; no C++; ruff check, ruff format --check,
mypy, `python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final message "WIP
M3-D2c: damping fix round finished", no attribution trailer, no push. Do not edit `docs/queue.md` or
`docs/cost-log.md`. No conditional (ternary) expressions. In spec table cells write abs(x), never bare pipe
characters. Ownership as D2b.

## Items

1. The failing test in `tests/unit/test_damping_run.py`: the run line prints a stated ratio of 0.00999796 for the
   x belt where the data says 0.01, because the run model's driven mass differs from the build-time mass by
   about 1 g (the rope's end fitting or a run-time body). The line must print three things without ambiguity: the
   ratio stated in the data (exactly as stated), the ratio realised at build time, and the ratio realised with
   the run's bodies and payload. Fix the line and the test to that; say in the report where the 1 g comes from.
2. D2b item 3, the missing oracle: the logarithmic decrement of a clamped-clamped member at sixteen segments
   through the native adapter, zeta 0.01 and 0.05, against 2 pi zeta / sqrt(1 - zeta^2) within 2 percent and
   against the scheme-corrected value (times 1 + x / (2 zeta), x = omega h) within 0.2 percent. D2b's test that
   the damping's omega equals the recorded first frequency "passed before the change too": make it a test that
   would have failed with a second implementation (assert the damping and the calibration call the same
   function, or perturb one path in the test).
3. D2b item 4: the scheme-corrected decrement asserted within 0.2 percent and the plain closed form within 2
   percent for the cantilever and pinned-roller cases too (BLD-040 states both tolerances and why, T4); the
   PHY-036 row says the rope's before-evaluation line reads the recorded per-step `max_tension`, so an
   exceedance inside a substep that the engine held gets no line.
4. D2b item 2: a boundary test through a real run at 1 kHz with one substep for starts of 0.01 s and 8 s (the
   step that begins exactly at the start is not "before"). The engine's own count at 10 substeps is being fixed
   in another worktree (integer step counts): do not work around it; write the 10-substep case as a test that
   passes once the driver and the engine agree, and mark it with the suite's strict expected-failure marker and
   the reason "engine evaluation start by step count, brief E7g follow-up" if it fails today.
5. D2b item 5: `docs/gantry-team-inputs.md`: the belt row gets its numeric range and source like the rail row;
   each rail and belt row gives the stated 0.01, the build-time realised ratio and the loaded realised ratio
   with the design load.
6. D2b item 6: the tier 1 party's hogging under-read lines (`tier1.trace_notes`) reach the run notes where the
   driver collects the party's results; one test through a real run (present when its condition holds, absent
   otherwise).
7. Rows DES-047, BLD-040, BLD-041, LIB-035, PHY-036, PHY-037 match the code as it now is (ruling 80: the anchor
   is the unloaded mode and the build-time mass; what a run prints); traceability regenerated; the builder,
   testing and native contracts updated (`run_damping.py` is new: add it to the builder contract's index).

## Report

Under 150 words: commits, filtered test counts (state them, do not pipe them away), the source of the 1 g, the
clamped-clamped readings, the boundary tests, each item done or not.
