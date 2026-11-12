# Review: M3-E3 (merge of E1 and E2, bindings, clamped far end)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never the writer's report; no builds or runs;
call cap 35. Worktree `/home/tex/dev/MechatronicsSimulator-05-e3`, branch `wip/05-e3`, head 0cdb0d0 (an empty
commit over ea37b2d). E1 (5f8878d) and E2 (ca0cd72) were each reviewed and accepted; do not re-review them. The
work under review is (a) the merge commit ee42996, which also carries E3's new work: read it as the combined diff
`git -C <worktree> show ee42996 --cc` (hunks that differ from both parents) and (b) `git -C <worktree> diff ee42996
ea37b2d` (clang-tidy fixes). Brief `docs/briefs/round-05/m3-e3.md`; rows in `docs/spec/eng.md` touched by the diff
(ENG-016 to 021, 030 to 035, 042 and the rope and member rows); ADR 0010 (model-change events), ADR 0016 (rope);
round 5 rulings 14 to 17, 20 to 23 (`docs/queue.md`); contracts `core/engine/CLAUDE.md`, `core/bindings/CLAUDE.md`.

Check: (1) the merge resolution: exactly one `ModelChange` / `ModelChangeEvent` definition; the writer dropped E1's
`PayloadReleased` so a payload release goes through `release_load(LoadSource::Payload, ...)` and emits
`LoadReleased` with source Payload, a second release throws, and E1's event clearing in `record_outputs` was
removed: say whether the event set still satisfies ADR 0010 and the ENG rows, who drains the event queue now, and
whether an event can be lost or delivered twice across a step; (2) nothing of E1's reviewed contact code
(GJK/EPA cap to `SolverNonConvergenceError`, `CollisionLimits`, `WeldAttachment`) or E2's (friction floor, rope on
`JointType::Translation`, `scheme_decay`, ENG-042) was altered in substance by the resolution; (3) the clamped far
end (ruling 16): the roller's Point constraint becomes a Hinge about the beam x axis (two more rows), axial slide
free. Judge it: does it lock both bending rotations, is leaving twist about x and the axial slide free right for a
clamped-clamped rail under an eccentric load, and should the support condition lock torsion; derive the chain
error the writer states, (1 - 4/N^2) on P L^3 / (192 E I) for deflection (-6.25 percent at N = 8, -1.56 percent at
N = 16) and +2.49 / +0.61 percent on the first frequency 22.37 / (2 pi) sqrt(E I / (rho A L^4)), and say whether
the reference asserting 2 percent at N = 16 with N = 8 checked against the chain values is the right oracle, and
how rulings 17 (default 8) and 20 (calibration; 2 percent for N >= 8) should be reworded; (4) bindings:
`JointType::Translation`, `add_rope`, `RopeOutput`, the mid-run calls (`set_attachment`, `set_body_mass`,
`release_load`), events, `CollisionLimits`; no raw owning pointers across the API, value semantics, errors typed;
`tests/unit/test_engine_bindings.py` covers each with edges; (5) the clang-tidy commit changes no behaviour
(`contact.cpp`, `collision.cpp`, the four test files: tolerances or expected values must not have moved);
(6) invariants 4, 5, 7, style (braces, no ternary), R-18 (spec rows in the same diff), T4 (no tolerance changed
outside the spec).

Report, under 400 words: verdict, findings by severity as `file:line, one sentence, spec row`, owner rulings
needed, cost note.
