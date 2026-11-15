# Gate: invariant audit (milestone 3, Opus per W-01)

Agent: invariant-auditor, model Opus (the gate overrides the frontmatter). Read-only; call cap 14. Checkout
`/home/tex/dev/MechatronicsSimulator` on `round/05-milestone-3`; the diff is the whole round, `git diff cf5c74f
66d8b26` (cf5c74f is main at the round's cut). Audit against the nine invariants and R-18 with emphasis on:
invariant 1 (the controller never sees true state: the native adapter, the integrity party, the new scoring
quantities read the record, never the engine), 2 (nothing gantry-specific in code: `mechsim/**`, `core/**`,
`tests/conftest.py`; the test-local weaker stepper part of ruling 95 lives in a test, not the catalog), 3 (every
YAML validates; schemas regenerated for the new quantity refs and criteria), 4 (determinism: the taut-set
iteration's deterministic order; the run cache keyed so two tests never share a run that differs), 5 (engines and
emulators behind their adapters: `python3 tools/check_imports.py`), 7 (failures are errors: the taut-set cap,
the unevaluable sizing criteria), 9 (core headless). Run the traceability test and the schema-validity invariant
test once each. Report findings by `file:line`, severity, invariant or row; then one line per invariant: holds,
or the finding numbers.
