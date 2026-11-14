# M3-D2b: fix round on D2 (review of 578fcca: no algorithm defect)

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1a` on `wip/05-x1a`, after brief
I5d is committed there (built and synced). Call cap 60 with the handoff rule (at 50 calls stop adding scope,
commit what is green, report what is left). Read first: `docs/briefs/round-05/m3-d2.md`, `review-m3-d2.md`,
`mechsim/builder/structure.py` (`node_damping`, the clamped-clamped omega_1 near line 134, `clamped_calibration`),
`mechsim/builder/physics.py` near line 804, `mechsim/testing/core_driver.py` near line 612, rows DES-047,
BLD-040, BLD-041, PHY-036, PHY-037, round 5 rulings 78, 80 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Filtered pytest in the foreground; no C++; ruff check, ruff
format --check, mypy, `python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final
message "WIP M3-D2b: damping fix round", no attribution trailer, no push. Do not edit `docs/queue.md` or
`docs/cost-log.md`. No conditional (ternary) expressions. In spec table cells write abs(x), never bare pipe
characters.

## Ownership

As D2 (`mechsim/design/schema.py`, `mechsim/builder/**`, `mechsim/library/**`, `mechsim/physics/native/translate.py`,
`designs/**` only for item 5's wording, `docs/gantry-team-inputs.md`, the two lines of
`mechsim/testing/core_driver.py` for item 2 and the run-end line of item 1, their tests, the DES, BLD, LIB, PHY
rows, traceability, contracts).

## Ruling 80 (settled before the cases run; do not change the placeholder values)

The damping ratio stated in the data is anchored where D2 put it: a member's at its unloaded first mode, a
transmission's at the driven subtree's build-time mass. That is the definition, stated in the rows. What a run
realises is lower when the member or the belt carries more (the gantry's rails realise 0.0054 and 0.0072 loaded):
every run prints the realised ratio truthfully, and `docs/gantry-team-inputs.md` gives both figures per rail and
belt. The placeholders stay 0.01.

## Items

1. Medium (BLD-041, PHY-037): the row and the builder claim the per-run line prints the realised ratio, but a
   payload hung on a rope or releasable is parented to the world, outside the driven subtree's mass, so the line
   prints the build-time 0.01. Make the claim true: at the start of a run the driver knows the case payload's
   mass and which joints carry it; the run line for each damped transmission and member prints the stated ratio,
   the build-time realised ratio and the realised ratio with the run's payload attached (zeta sqrt(m / (m + m_p))
   for a transmission; zeta omega_loaded / omega_1 for a member, with the loaded frequency from the one-mode
   estimate sqrt(k / (m_c + f m_beam)) for its supports: say it is an estimate); or, if the driver cannot know
   which transmission carries the payload, the line says exactly that the figure excludes run-time payloads and
   gives the formula. Whichever you implement, the row says what is printed. Test through a real run.
2. Medium (PHY-036): `core_driver.py` near line 612 decides "before the evaluation start" with the float test
   `t - step < start`, which counts the step begun exactly at the start as before at 1 kHz for starts of 0.01 s
   and 8 s, while the engine tests its own time: the line can be off by one step. Use one rule shared with the
   engine's semantics (integer step counts from the run's rate, or the same comparison on the same quantity) and
   test the boundary step at both those start times.
3. Medium (BLD-040): the clamped-clamped omega_1 in `structure.py` near line 134 is a second implementation
   beside `clamped_calibration`. One implementation; a test that the frequency the damping uses equals the
   calibrated chain's recorded first frequency for clamped-clamped (and for the other support pairs) within
   1e-9; and the missing decrement oracle for a clamped-clamped member at sixteen segments (zeta 0.01 and 0.05,
   as the other two).
4. Low: the scheme-corrected decrement check in `tests/reference/test_member_damping.py` near line 141 is at 2
   percent, which cannot tell the scheme's 0.5 percent from an error: assert the scheme-corrected value (closed
   form times 1 + x / (2 zeta)) within 0.2 percent, and keep the plain closed form at 2 percent (BLD-040 row
   states both, T4); the PHY-036 row says a substep-only exceedance that the engine held gets no line (the line
   reads recorded per-step `max_tension`); `flexible.damping_ratio` and the catalog `damping_ratio` given as an
   open parameter still get the range check [0, 1) when resolved (DES-047, LIB-035).
5. `docs/gantry-team-inputs.md`: the belt row gets its numeric range and source like the rail row; each rail and
   belt row gives the stated 0.01, the build-time realised ratio and the loaded realised ratio with the design
   load.

## Report

Under 150 words: commits, filtered test counts, what the run line prints (one example), the boundary-step test,
the clamped-clamped oracle's readings, each item done or not.
