# Review: M3-I1 (integrity plugins, tier 2 checks, FAC-013)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never the writer's report; no builds or runs;
call cap 40. Worktree `/home/tex/dev/MechatronicsSimulator-05-i1`, branch `wip/05-i1`, commit d26c5f8 (first pass
2b4957a plus fix round), diff `git -C <worktree> diff 3e65867 d26c5f8`. Brief `docs/briefs/round-05/m3-i1.md`;
rows INT-001 to 008, 010 to 012, 019 to 022, 036, 037 (`docs/spec/int.md`), FAC-013 (`fac.md`); ADR 0009 and
ADR 0010; round 5 rulings 3, 4, 5, 6, 19, 24, 25, 26, 27 (`docs/queue.md`); the contracts in the diff
(`mechsim/integrity`, `testing`, `facade` CLAUDE.md files); R's shapes in `mechsim/testing/driver.py`
(`IntegrityResults`, `MemberIntegrity`, `RopeIntegrity`, `MemberSection`).

Check: (1) the plugin interface and registry, configuration from `integrity.yaml` beside the design (ruling 27),
tiers additive with the test that shows a higher tier never changes a lower tier's numbers (INT-004); (2) strength:
the two-axis stress |N|/A + |M_y| c_z/I_y + |M_z| c_y/I_z in the member frame of ruling 24 from the recorded end
loads, shear, the safety factor against the process- and direction-dependent strength through
`IntegrityMaterialView` with the spreads, the single-axis fallback flagged as an approximation; the cantilever
reference derivation; (3) deflection: which source (chain node poses or end-loads with E I) and whether the choice is
right for a lumped beam with eight segments; the 1e-3 tolerance derivation; (4) buckling effective length factors
per end condition; fatigue: the rainflow algorithm stated and correct (ASTM E1049 three-point or four-point; say
which), Miner's sum, the sinusoid reference count; (5) fastener, impact (the impact factor from configuration),
resonance screening (excitation frequencies from the recorded signals: stepper step rate, commanded speed; the
band); (6) INT-019 to 022: the design-load rule from the design file only, the flags, the per-member and per-rope
table in R's shapes written beside the archive and returned through the facade with the CSV and JSON load-case
export (FAC-013); (7) the wiring: `core_services` passes the service, no file means no table and every existing
run unchanged; the `cast` the writer notes because `IntegrityCheck` in driver.py returns `IntegrityResults` not
`| None` (say whether the protocol should change); (8) the writer reports a facade run of a pendulum copy with one
flexible member ends with "no recorded end-loads (columns [])": from the diff and R's recorder (REC-004,
`mechsim/testing/core_driver.py`), say whether the bridge path records member end-loads for such a design and what
the FAC-013 conformance test needs; (9) invariants 2, 3, 6, 7, 9, style, ownership, R-18 (every behaviour change
with its spec row), tolerance rows stated in the spec (T4).

Report, under 400 words: verdict, findings by severity as `file:line, one sentence, spec row`, owner rulings
needed, cost note.
