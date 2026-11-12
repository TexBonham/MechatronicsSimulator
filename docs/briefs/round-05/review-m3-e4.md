# Review: M3-E4 (engine joint limits, joint friction, per-step joint drive)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never the writer's report; no builds or runs;
call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-05-e4`, branch `wip/05-e4`, commit c072dee, diff `git -C
<worktree> diff a1106b9 c072dee`. Brief `docs/briefs/round-05/m3-e4.md` (the oracles are derived there); rows
ENG-013, 014, 021, 025, 049, 050, 051 and the new tolerance row "Joint limit, friction and drive tests" in
`docs/spec/eng.md`; ADR 0013; round 5 rulings 14, 21, 37, 38 (`docs/queue.md`); contracts `core/engine/CLAUDE.md`,
`core/bindings/CLAUDE.md`.

Check: (1) limits: one-sided rows in the velocity-level projected Gauss-Seidel pass (multiplier clamped to push
only), zero restitution, the position treatment (the writer says a limit engages within h times a velocity
tolerance of its bound and a projection then puts the coordinate exactly on it: derive whether that projection can
add energy or fight a joint spring, a transmission's reflected row or a loop closure on the same DOF, and whether
it is consistent with how the engine stabilises loop closures under ADR 0013); the release case (no holding
force); both bounds, lock (lower equal to upper), rejection cases; rows appended in joint order (determinism);
non-convergence ends the run (ENG-025); (2) friction: the regularised law frozen at the start-of-step rate, the
creep fixed point v_s L / F, and the sliding test's bounds: stopping time allowed up to t* + 0.0133 s and distance
within 5.0e-4 m of 0.25 m. Derive the frozen-rate scheme's error yourself and say whether those bounds are derived
or fitted (T4); chatter or sign reversal at the slip threshold; (3) drive: `set_joint_drive` semantics (from the
next step, persistent, no event), exact sag and lag, behaviour under a transmission (the writer tested only a
drive on the shaft joint: say whether a drive on the driven output joint can occur through the native adapter and
needs a test), zero stiffness; (4) the writer notes every sprung joint DOF now publishes `scheme_decay`, beam nodes
included, at the cost of one mass-matrix factorisation per step: judge the cost against the fixed per-step budget
of ADR 0013 and say whether it should be computed only when a stiffness or the configuration changes, or only for
driven DOFs; (5) the writer throws `InvalidModelError` for bad friction values where the brief cited ENG-050
(configuration): say which row governs; (6) bindings: value semantics, typed errors, bad-argument edges tested;
(7) nothing of the reviewed engine (contact, members, ropes, mid-run changes) altered in substance; invariants 4,
5, 7; style (braces, no ternary, no raw owning pointers); R-18.

Report, under 400 words: verdict, findings by severity as `file:line, one sentence, spec row`, what brief N3 must
know to swap the adapter onto these, owner rulings needed, cost note.
