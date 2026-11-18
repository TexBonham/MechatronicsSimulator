# Review: M3-N3 (native adapter on the engine's limits, friction and drive)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never the writer's report; no builds or runs;
call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-05-n3`, branch `wip/05-n3`, commit 8e4d3f4, diff `git -C
<worktree> diff 3922340 8e4d3f4`. Brief `docs/briefs/round-05/m3-n3.md`; rows PHY-003, 006, 010, 012, 018, 019,
021, 027, 029, 030, 033, 034 (`docs/spec/phy.md`), BLD-017, BLD-019, the new kernel row in `docs/spec/ker.md`;
round 5 rulings 21, 32, 37 to 46 (`docs/queue.md`); contracts `core/physics/native/CLAUDE.md`,
`mechsim/physics/native/CLAUDE.md`, `core/engine/CLAUDE.md` (joint limits, joint friction, `set_joint_drive`,
`clear_joint_drive`, `scheme_decay`, transmission efficiency).

Check: (1) the swap: no penalty limit, damper stick or adapter-side drive force left in `native_adapter.cpp`;
limits and friction translated into the engine model; each tick's drives through `set_joint_drive` with the
end-of-step rest, `clear_joint_drive` when a drive is disabled or absent that tick, additive on model stiffness
(ruling 42); stale `e.inputs` cleared; `substeps` either produced and tested or removed; (2) the closed-form tests:
derive each expected value yourself (creep v_s L / F; limit rest reaction; sliding-stop bounds from the frozen-rate
scheme; drive sag and lag; lead screw; backlash dead band; gear ratio) and say whether each assertion is derived or
fitted (T4); two points to judge in particular: (a) the brief and ruling 39 say a drive through a transmission with
efficiency below one is lossless at zero shaft speed, so static sag is exact, but the tests assert only a band
[n eta, n / eta] for the belt and gear static sag: read the engine's efficiency rule in
`core/engine/src/transmission.cpp` (do not review the engine otherwise) and say whether the band is the engine
failing ruling 39 (an engine fix is owed), the test set up wrongly, or the ruling unachievable as worded; (b) the
lead screw lock test uses a large shaft inertia: read how the engine holds a self-locking screw and say whether a
lock that fails at small inertia is an engine defect against ENG-044 or a step-size limit that should be published
as an approximation line; (3) approximation lines against ruling 45: x = omega h, kept fraction
(1 + x^2)^(-pi / atan x), frequency ratio atan(x) / x, unresolved flag above 0.2, the numbers at x = 0.1 and 1.3;
the friction creep line per joint; (4) BLD-019: calibrated clamped-clamped member below sixteen segments is a build
error, calibration applies at sixteen or more; the edit to `tests/unit/test_builder_structure.py` was outside the
writer's list, say whether it is in substance right; (5) beam references: clamped end moments P L / 8, simply
supported deflection and frequency within 2 percent at eight segments; (6) PHY-010 run-twice cases use a real joint
drive and a member; (7) the spec rows changed (PHY tolerance rows, KER row for ruling 40) match the code and carry
derived tolerances; contracts current; (8) invariants 2, 4, 5, 7; style (braces, no ternary, no raw owning
pointers); R-18. Not done by the writer and going to a follow-up, so do not flag as missing: the `<member>.node<k>`
pose series and the `CoreDriver` archive test (needs `mechsim/record`), and the facade `sets` to `run_id_of`
(needs the protocol payload).

Report, under 400 words: verdict, findings by severity as `file:line, one sentence, spec row`, your answer on
(a) and (b), what the follow-up brief must carry, owner rulings needed, cost note.
