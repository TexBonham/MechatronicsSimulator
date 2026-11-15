# M3-X2h: merge X2g, regenerate the native baseline after E8, the misses of record restated (ruling 94)

Why: X2g (ruling 94) explained G7's and G9's held-position difference by two engine mechanisms, one of which (the
rope rule before E8) is gone from the tree: with E8 merged, native G7 records 1.821 mm against the bridge's
2.177 mm, inside the 0.436 mm per-axis tolerance. The native `baseline.yaml` was written before E8 and is stale
for every case with a rope. One regeneration of the whole suite after E8, compared entry by entry, is the
record; the companion miss list and the retirement table follow it.

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-x2` on `wip/05-x2` (X2f landed;
the extension is current unless the merge below changes C++, which it does not). Call cap 90 with the handoff
rule (at 80 calls stop adding scope, commit what is green, report what is left by item). Read first:
`docs/briefs/round-05/m3-x2d.md` and `m3-x2g.md` (their rules hold: no tuning, nothing of MuJoCo deleted, bridge
baselines never regenerated), `tests/acceptance/test_phy_stubs.py` (the companion miss list, the PHY-024 row),
`test_cases/gantry/README.md`, `docs/milestones.md` (criterion 2 row), round 5 rulings 86, 90 to 94 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Builds and runs in the FOREGROUND only through
`tools/locked_build.sh run ...` with a 600000 ms timeout (the suite regeneration runs 19 cases, about 21 minutes:
one call); filtered pytest; no wide run. Lints at the end: `uv run ruff check .`, `uv run ruff format --check .`,
`uv run mypy`, `python3 tools/check_imports.py`. COMMIT AFTER EACH ITEM; final message "WIP M3-X2h: native
baseline after E8", no attribution trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`. No
conditional (ternary) expressions. Curly braces on every control flow. In spec table cells write abs(x), never
bare pipe characters.

## Ownership

`test_cases/gantry/baseline.yaml` (regenerated once, whole), `tests/acceptance/test_phy_stubs.py` (the list of
record and its causes), `test_cases/gantry/README.md`, `docs/milestones.md` criterion 2 row, the PHY-024 row,
`docs/spec/traceability.md`, `docs/briefs/round-05/mujoco-removal.md` where it states the misses. Not yours:
`core/**`, scoring, any case criterion, `bridge_baselines/**`.

## Items, in order

0. `test_cases/gantry/G16.yaml`: its header comment still says the sizing criteria judge a "window"; reword
   it to the span rule of ruling 93 (between rest samples from the last rest at or before the evaluation start) in
   one or two sentences; nothing else in the case changes (the case hash changes once, absorbed by item 2).
1. `git merge --no-edit wip/05-x2g` into `wip/05-x2`; resolve conflicts keeping both sides' intent (X2f's G16 and
   scoring rows, X2g's PHY-024 derivation, README notes and milestones row; `docs/spec/traceability.md` by
   regenerating it with `python3 tools/traceability.py` or the project's documented command). Run the scoring unit
   tests and `tests/acceptance/test_phy_stubs.py -k "not 023"` filtered to confirm the merged tree is consistent.
2. Regenerate the native baseline once: `mechsim suite test_cases/gantry --write-baseline` on the default adapter
   (and the pendulum suite if it has a baseline; the rope pendulum case if one is baselined), after confirming
   with `git log` that the tree holds E8 (the rope taut-set rule). Compare with the previous file entry by entry:
   every entry whose status or measured value changed is listed in the report and in the README's baseline note
   with old value, new value and cause (E8's rule for rope cases; X2f's new G16 criteria; anything else explained
   or reported as a finding); hash-only changes counted. A status change from pass to anything else is a finding
   to report before anything else.
3. The misses of record: run the PHY-024 comparison; the companion test's sorted list becomes what the comparison
   now prints (expected: G7 `hook_held` and G9 `load_held` inside their tolerance, the four G10 entries left:
   `stall_fault`, `slip_fault` the finding, `stall_rehomed`, `slip_carriage_stopped` its consequences), each with
   its cause from rulings 90 and 94; `test_phy_024` stays a strict expected failure while any miss remains. If the
   list differs from the expectation, report why before editing it.
4. `docs/milestones.md` criterion 2, the README and `mujoco-removal.md` restated: per-axis tolerances, the misses
   left with causes, G7 and G9 explained (ruling 94's two mechanisms, 0.357 mm at the head), the efficiency-damping
   release artefact recorded as a finding for the owner (ruling 94, E9), G10's remainder as a finding with X2g's
   measurements. Nothing marked met without evidence.

5. The in-motion demonstration (ruling 95; X2f's draft is in the scratchpad at
   `/tmp/claude-1000/-home-tex-dev-MechatronicsSimulator/e12b8ab0-2af1-4dae-a612-eacb9e002661/scratchpad/item2_in_motion_draft.py`,
   and X2f built the weak x part at factor 0.05 through the run's registry): a `long_run` test that writes a test-local copy
   of G16's nominal case with `integrity.evaluate_from: 0 s` into its tmp dir, runs it with the weak x part (the
   derivation in the header: the x motor's demand in homing and at cruise by ACC-031's derivation against the
   scaled curve, at least 20 percent short at cruise), and asserts: stall events during motion (homing or the
   move), the span count for `x_motor` at least one, its windowed margin negative, the report's lines saying so;
   the same test (or a sibling) copies G10 with the start at 0 and asserts its stalled motor's span count at
   least one. The suite's own cases keep their 9 s start. Ownership adds `tests/acceptance/test_acc_stubs.py` for
   this test and ACC-032's conversion sentence.
6. Run `tests/acceptance/test_acc_stubs.py -k "test_acc_032" -m long_run -s` once (X2f's result was cut off by its
   output filter) and quote its printed lines.

## Report

Under 300 words: commits; the merge's conflicts and how resolved; item 5's factor, stall times and counts; ACC-032's lines; the baseline comparison (entries changed with
old, new, cause; any status change); the misses of record as the comparison prints them; the retirement table's
criterion 2 wording; lints; open issues.
