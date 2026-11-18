# Milestone 3 gate inputs (coordinator, 2026-11-14, at 66d8b26)

## Reference tier (reference-validator, Sonnet)

C++ `debug-sanitize -L reference`: 61 of 61 passed, 794.6 s test time (the heavy rope cases 755, 310 and 146 s).
Python `tests/reference` on native (the default adapter): 105 passed, 4 xfailed (strict stubs), 0 failed, 176 s.
Python `tests/reference/test_phy_stubs.py` (pendulum on both adapters): 8 passed; period mujoco 1.159849 s and
native 1.159849 s against the closed form 1.1596536 s, identical. No tolerance miss. Determinism: identical printed
values across the full-tier run and the separate run (not a formal run-twice check; the invariant tier's run-twice
test passed in the landing verification). Native against bridge: position 0.0 (limit 2.5e-7), orientation 4.4e-16,
angular velocity 2.1e-15, linear velocity 3.8e-17, reaction 1.99e-4 N (limit 1.861e-3), generalized force 6.65e-5 N m
(limit 1.136e-2). Values of note, all passing: BLD-015 1.0e-4 m against 9.9609e-5; BLD-016 51.2319 against 51.3709
rad/s; BLD-017 1.0e-4 against 9.8437e-5 m and 143.810 against 142.828 rad/s; BLD-019 clamped-free 46.1087 against
46.0761 rad/s, pinned-roller 129.429 against 128.714; cantilever first mode 28.336 against 28.435 Hz; clamped-clamped
180.309 against 181.377 Hz; chain statics 7.03125e-5 against 7.0312498e-5 m where the continuum gives 7.1429e-5;
pinned-pinned buckling margin 52.6379 and first frequency 45.5232 rad/s exact; fatigue damages, fastener margin and
impact stresses exact; hanging rope tension 24.525 N exact, settled load end 9.8164378 N exact; rope pendulum
period 2.006067 against 2.006364 s; friction stop after 1834 steps within [1824.98, 1835.10]; damping cases pass
with the largest scheme-corrected residual +0.091 percent (clamped-clamped N 16 zeta 0.01). Validator's minors: most
Python reference tests print closed form and measured but not the asserted limit; only BLD-015 to 019 and LIB-032
print a requirement ID (the `req` markers were not checked); damping tests print raw (+0.5 percent) and corrected
values and the validator did not check which is asserted. Not run: the reference executable's per-test printed values
under the sanitizer (budget), the `release` preset run and its bitwise comparison with `debug-sanitize`.

## Invariant audit of the round (invariant-auditor, Opus; `git diff cf5c74f 66d8b26`, 408 files)

No violations. Invariants 1, 2, 4, 5, 7, 8, 9 hold with evidence; 6 not checked in depth; 3 and R-18's traceability
were not confirmed by the auditor's own test run (it waited on the build lock) and are confirmed instead by the
landing verification at the same code (the invariant tier in the fast run: 4722 passed). Two optional minors:
`tests/conftest.py:22-33` the run cache key has no content hash of the case file (in-place edits at one path within
a session would share a run; edited copies go through `run_case` as documented); `mechsim/scoring/score.py:160` the
criterion `except` catches a bare `KeyError` (status "error", never a silent number). Style rules on changed lines
not checked at the diff's size. The pre-landing Sonnet audit of the X2 branch (c0cc5f0..1e614d5) was clean.

## Landing verification (coordinator, wip/05-x2 at 1e614d5, content-identical to 66d8b26 outside docs)

ctest debug-sanitize 622 of 622; `pytest -m "not long_run"` 4722 passed, 89 xfailed, 874 s; `pytest -m long_run`
36 passed, 4 xfailed, 3558 s; ruff, ruff format, mypy, check_imports clean. CI on the round branch at 66d8b26:
running at the time of writing (checked before the gate report).
