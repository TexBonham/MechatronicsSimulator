# M3-X1h: the belt's driven mass includes member chains; G12 and G15 baseline entries

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x1a` on `wip/05-x1a` at f8f4836
(built and synced). Call cap 60 (at 50 stop adding scope, commit what is green, report what is left). Read first:
`docs/briefs/round-05/m3-x1g.md` (the no-tuning rule), `mechsim/builder/physics.py` (the BLD-041 conversion and
the subtree mass it uses), `mechsim/builder/run_damping.py`, `mechsim/physics/native/translate.py`
(`_subtree_mass` and the transmission mode line), rows BLD-041, PHY-037, SCO-009, round 5 rulings 78, 80, 82 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Filtered pytest and single cases through the CLI in the
foreground with timeouts; no C++; ruff check, ruff format --check, mypy, `python3 tools/check_imports.py`; no full
pytest run of your own. Commit as you go; final message "WIP M3-X1h: driven mass with member chains, baseline",
no attribution trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`. No conditional (ternary)
expressions. In spec table cells write abs(x), never bare pipe characters.

## Ownership

`mechsim/builder/physics.py` and `run_damping.py` (the driven mass only), `mechsim/physics/native/translate.py`
(`_subtree_mass` and the line that prints it), their tests, `test_cases/gantry/baseline.yaml`,
`docs/gantry-team-inputs.md` (the realised figures), rows BLD-041 and PHY-037, `docs/spec/traceability.md`.

## Items

1. Defect (BLD-041): the mass a compliant transmission's damping is anchored to, and the mass the translation's
   mode line prints, is the driven joint's body subtree without the lumped member chains parented inside it: on
   the gantry the x belt is anchored to 2.44 kg (32.2 Hz) while the x carriage also carries rail_y's 2.03 kg
   beam and rings at 22.6 Hz, so the belt realises 0.0074 where the data states 0.01 at build time. The driven
   mass is everything rigidly riding the driven joint: the body subtree and every lumped member rooted on a body
   of that subtree (its whole chain mass), recursively through bodies that members carry. One function, used by
   the conversion, the run-damping line and the translation's mode line. Tests: a small data-only design with a
   member on a belt-driven carriage: the damping equals 2 zeta sqrt(k m) with the member's mass in m, the mode
   line prints the same mass and frequency, and the measured decrement of the belt mode (the member built stiff
   enough that its own modes are well above) matches the stated ratio within the rows' tolerances; a design
   without members is unchanged to the last bit. Ruling 80 stands: the anchor is the build-time mass; this fixes
   what the build-time mass is. The placeholder values do not change.
2. With item 1 in, print the gantry's three belts (damping, mass, frequency, time constant) and update the
   realised figures in `docs/gantry-team-inputs.md`.
3. G12 and G15 baseline entries (left by X1g, SCO-009): regenerate them on native after item 1; report every
   criterion's value before and after item 1 (the belt damping on x rises, so small moves are expected: give
   them; a criterion that changes verdict is reported, not adjusted). The acceptance tests for ACC-025, 026,
   029, 030 and the run-twice tests for G12 and G15 keep their states (run them: they take several minutes each;
   foreground, 600000 ms timeout).

## Report

Under 150 words: commits, filtered test counts, the three belts' figures, G12's and G15's criteria before and
after, any verdict that changed.
