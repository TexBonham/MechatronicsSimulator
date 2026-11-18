# Review: M3-I2 (tier 1 integrity party, tier 2 yield and rope snap)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never the writer's report; no builds or runs;
call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-05-i2`, branch `wip/05-i2`, commit 81b2943, diff `git -C
<worktree> diff fac0e26 81b2943`. Brief `docs/briefs/round-05/m3-i2.md` (the oracles are derived there); rows
INT-023 to 028, 031 (`docs/spec/int.md`), KER-028 and the kernel physics rows (`docs/spec/ker.md`), TST-041
(`docs/spec/tst.md`); round 5 rulings 3, 4, 26, 27, 34, 35, 40 (`docs/queue.md`); contracts
`core/integrity/CLAUDE.md`, `core/kernel/CLAUDE.md`, `core/bindings/CLAUDE.md`, `mechsim/integrity/CLAUDE.md`,
`mechsim/testing/CLAUDE.md`.

Check: (1) limits table (`mechsim/integrity/tier1.py`): built only from the build result and `integrity.yaml`,
nothing design-specific (invariants 2, 3); SI; which limit each quantity gets and whether a design-load or safety
factor is applied (the writer says the party evaluates live loads with no design-load factor: say whether INT-023
to 025 require one); (2) the party (`core/integrity/`): a kernel `Party` at the physics rate that reads only
physics outputs (member end loads, rope tension, fastening forces) and never engine internals (invariant 5); its
place in the tick order and whether its result is deterministic (invariant 4: ordered containers, event order by
table order); the stress, buckling, fastener and deflection formulas against the tier 2 Python checks (derive the
peak bending stress from end loads yourself; two-axis sections, ruling 25); (3) crossings: exactly one event per
(member or fastening, quantity) per run (ruling 34), raised on the first tick at or past the limit (derive the
tick from the brief's ramp oracles: is "event at tick 11 for t* = 10.5 ms" the right side of the boundary, and is
a value exactly at the limit a crossing by the spec row); events reach the recording as `integrity` events (ruling
26); (4) halt: `integrity.halt_on_crossing` in the case schema, default false; the writer made the
`integrity.yaml` value apply when the case leaves the field unset: say whether that contradicts ruling 4 (test-case
field, default false); a halted run ends with end reason `end_time` plus a report line because the real end reason
enums are outside the brief: say whether invariant 7 (failures are errors, the run ends with a report) is met in
the meantime and what the follow-up must add; (5) tier 2: the yield offset (peak |M| - M_y) / k at the nearest end
node with M_y = sigma_y I / c and k = E I / l (ruling 35): applied once per exceedance or ratcheting each tick,
monotone, emitted as a `model_change` event exactly once per change (ruling 40); rope snap routed to the engine's
release with the load in free fall after; the yield oracle's tolerance (asserted 1e-3 against a derived bound of
2.3e-4: say whether the row should carry the derived bound, T4); the writer says yield acts on planar bending
about z only: say whether the spec row states that limit; (6) the kernel change: the physics party's model-change
input and row KER-028 against the kernel contract (owner-reviewed contract: list every signature that changed);
(7) `CoreDriver` now loads tier 1 from the design folder by default and a FAC-009 test was edited for it: say
whether the edit is right in substance; on the bridge the party is absent and an approximation line says so
(rulings 29, 34); (8) the determinism acceptance test; conversions of the INT stubs (no assertion weaker than the
row); (9) `core/CMakeLists.txt` gained one line outside the ownership list; style (braces, no ternary, no raw
owning pointers); R-18.

Report, under 400 words: verdict, findings by severity as `file:line, one sentence, spec row`, what brief X1 must
know to use tier 1 on the gantry (G15), owner rulings needed, cost note.
