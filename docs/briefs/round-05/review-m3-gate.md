# Gate review: milestone 3

Reviewer: Fable (general-purpose), fresh. Read-only; call cap 45. Checkout `/home/tex/dev/MechatronicsSimulator` on
`round/05-milestone-3` at the X2 landing commit 66d8b26. This is the milestone gate, not a diff review: the stage
reviews of record (E1 with E2, M with D, N, I1 with R, I2, E3 to E7g, I3 to I5d, D2 to D2d, X1a to X1h, X2 to X2h,
N5, B3 with E8) and their audits are in `docs/cost-log.md` rows and the briefs under `docs/briefs/round-05/`; the
reference-validator's report (both adapters) and the Opus invariant audit are given to you in the prompt. Read
`docs/briefs/round-05/plan.md` (scope, assumptions, the owner's amendment: milestone 3 retires MuJoCo in favour of
the own engine), `docs/queue.md` (round 5 rulings 1 to 95 with their planned defaults and status), `docs/milestones.md`
(the M3 gate row: G12, G15, G16 with G12 limits and the minimum safety factor not set; the MuJoCo retirement criteria
table), `docs/adr/0016-rope-model.md` and `docs/adr/README.md` (ADRs the owner reviews), `test_cases/gantry/{suite,
baseline}.yaml` and `README.md` (the evaluation start's derivation, the misses of record), `tests/acceptance/
test_phy_stubs.py` (PHY-024 and its companion), `docs/spec/traceability.md` (which rows are still strict-xfail
stubs: compare against the plan's "In" scope), `docs/gantry-team-inputs.md`, `docs/briefs/round-05/mujoco-removal.md`.

Answer: (1) Does the round branch meet the milestone 3 gate as `docs/milestones.md` states it: G12, G15 and G16 in
the suite on the native adapter with their results of record (G12's settle time and G15's rail_y not-valid flag
and rail_x safety factor are recorded, not passed: rulings 64, 81, 82; is the gate statement honest about them, and
does any case pass only through a placeholder that hides the behaviour); the suite's evaluation start (9 s, derived
once): what it hides and what it must not (rulings 86, 87, 91, 93: the rope start-up blow is fixed in the engine, the
belts' and rails' unloaded start is milestone 4's static start). (2) Retirement (ADR 0004): per criterion, met or
not with its test and numbers; the misses of record and their classification (explained with a named mechanism the
code applies, or a finding); the E9 release artefact (ruling 94) and G10's remainder as findings: state for the
owner what the evidence shows and does not show, in a form the owner can decide on; and whether the removal plan is
complete enough to execute as one commit after the decision. (3) Which plan-scope rows remain stubs or unproven,
and which block the gate versus carry to M4 (list by ID). (4) Rulings 1 to 95: which change milestone 1 or 2
behaviour or the architecture (the rope rule in the engine, ruling 87; the taut set, 89; the sizing span rule, 93;
the integrator decision, 21; the attachment, 14; the member frame, 24; the not-valid results, 69, 72; damping as
design data, 80; the fast-tier moves, 90) and so need the owner's explicit confirmation before the gate commit;
which are placeholders the gantry team owes; which are process. (5) ADR 0016 as amended (rulings 14, 15, 22, 87,
89): consistent with the code as the ENG-047 row states it. (6) Anything in `docs/gantry-team-inputs.md` the cases
depend on silently. (7) The usage against the plan's estimate from the cost log (sum the round 5 rows, 2026-11-11
to 2026-11-14) with the three largest overruns named.

Report, under 700 words: gate verdict (pass, pass with owner confirmations, fail), the blocking items, the
retirement statement for the owner, the M4 carry list by ID, the owner confirmation list grouped (architecture,
batch, gantry team, process), cost note.
