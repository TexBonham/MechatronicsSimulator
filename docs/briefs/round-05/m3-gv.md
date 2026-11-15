# M3-GV: real components for the gantry team's open slots (owner decision 4, 2026-11-15; plan signed off 2026-11-15)

Why: the gantry team has not supplied most structural, motor, transmission, rope, sensor and timing values; the suite
runs on flagged placeholders, so G12's settle, G15's safety factor and G16's margins are results against invented
data. The owner ruled that the coordinator may put real, datasheet-backed components, materials and sections into
the existing data slots, flagged replaceable, so every case is testable, wherever the data can be obtained without
the team. CAD import is milestone 4: section parameters in the design file stand in for geometry.

Fresh agent. Agent: module-writer (Opus). Worktree `../MechatronicsSimulator-05-gv` on `wip/05-gv`, cut from
`round/05-milestone-3` after X2i lands (no build: `tools/locked_build.sh sync` first). Call cap 120 with the handoff
rule (at 110 calls stop adding scope, commit what is green, report what is left by item). Read first:
`docs/gantry-team-inputs.md` (every slot, where it lives, what is answered), `designs/gantry/design.yaml`,
`designs/gantry/integrity.yaml`, `mechsim/library/CLAUDE.md` and the catalog files `stepper_behavioral.yaml`,
`belt_pulley.yaml`, `rope.yaml`, `hook.yaml`, `load_cell.yaml`, `load_cell_amplifier.yaml`, `linear_encoder.yaml`,
`mechsim/materials/CLAUDE.md` and `mechsim/materials/data/aluminium_6063_t5_extrusion.yaml`, `mechsim/builder/CLAUDE.md`
(`rail_calibration`, BLD-019: a calibrated member needs a recorded deflection and first frequency), `test_cases/gantry/
README.md` (the evaluation start's derivation from rail_x's damping and frequency; G6's bound derivation), rows LIB-016,
LIB-017, LIB-020, LIB-034, BLD-019, BLD-041, DES-047, ACC-026, ACC-029, ACC-030, ACC-038, round 5 rulings 6, 7, 35, 64,
78, 80, 83, 87, 94, 98 in `docs/queue.md`. Sources: the manufacturer's or distributor's published datasheet for every
value, found by web search (`WebSearch`, `WebFetch`); record the URL and the figure beside each value in
`docs/gantry-team-inputs.md`; a value you cannot source stays the placeholder it is, flagged. Builds and runs in the
FOREGROUND only through `tools/locked_build.sh` with the Bash `timeout` parameter at 600000; single cases through
the CLI into the scratchpad `/tmp/claude-1000/-home-tex-dev-MechatronicsSimulator/e12b8ab0-2af1-4dae-a612-eacb9e002661/
scratchpad/gv/`; filtered pytest; the whole-suite baseline regeneration is one call (about 21 minutes); the only wide
run is one `-m "not long_run"` at the end. Lints at the end. COMMIT AFTER EACH ITEM; final message "WIP M3-GV: real
components in the gantry's slots", no attribution trailer, no push. Do not edit `docs/queue.md` or
`docs/cost-log.md`. No code changes: if a slot needs a field the schema lacks, that is a finding, not a schema edit.
In spec table cells write abs(x), never bare pipe characters.

## Ownership

`designs/gantry/design.yaml`, `designs/gantry/integrity.yaml`, new or changed PARTS in the catalog files (a new part
entry beside the generic one, never a change to a generic part's values; the design then names the new part),
`mechsim/materials/data/*.yaml` only for a new sourced material, `docs/gantry-team-inputs.md`, `test_cases/gantry/
README.md` (derivations), `test_cases/gantry/baseline.yaml` (regenerated once), `test_cases/gantry/G6.yaml` and
`G12.yaml` ONLY for the derived evaluation start and G6's bound (both derived in the README, never adjusted to a
result), the strict expected failures in `tests/acceptance/test_acc_stubs.py` that flip for a stated physical reason
(converted, reason kept), their ACC rows, `docs/spec/traceability.md`, `docs/milestones.md` G12, G15, G16 rows. Not
yours: any code, any requirement value (safety factor 1.5, 1 cm, 0.1 rad, 5 s, 10 lb, overload 5.216 kg), any other
case criterion, `bridge_baselines/**`, firmware.

## Items, in order

1. Rails: a 2040 aluminium T-slot extrusion (6063-T5, the library material) as both rails: section area, second
   moments about both axes and fibre distances from the extrusion datasheet; the two rails may differ if the
   design's bodies do (rail_y's box is 40 x 60 mm: say what you choose and why); `rail_calibration` becomes each
   rail's closed-form midspan deflection under a stated load and first frequency for ITS section and end
   conditions (BLD-019), computed in the README; rail damping ratio from a cited source for bolted aluminium
   frames (state the range and the value). Keep the printed-PETG material entries in the library untouched.
2. Steppers: two real parts, one NEMA 17 class and one NEMA 23 class (for example StepperOnline 17HS19-2004S1 and
   23HS22-2804S, or the equivalents whose datasheets you can reach): holding torque, rotor inertia, phase
   resistance and inductance, rated current, max voltage, mass, step angle, and the pull-out torque curve read from
   the published graph at the catalog's speed points (LIB-034: the curve starts at the holding torque; if the
   published curve does not, say so and anchor it as the row allows). The design's motors name the NEMA 23 part;
   G16's sweep names both.
3. Belts and pulleys: GT2 6 mm belt with Gates' or an equivalent published stiffness per unit length (or the tensile
   modulus and section from which it follows), tensile rating, and an efficiency from a cited source; the 20-tooth
   pulley's pitch diameter. One new part; the design's three belts name it.
4. Rope and hook: a 1.5 or 2 mm 7x19 stainless cable (rated breaking strength, EA from the cable's effective modulus
   and section, mass per length) and a real swivel hook or shackle mass as `end_fitting_mass`; cite both. The
   design's `payload_rope` and hook name them.
5. Load cell and amplifier: a 5 kg single-point cell (rated output, overload) and an HX711-class amplifier (sample
   rate, resolution, filter) as new parts with their sourced values; the counts per kg and zero in the design
   follow from them (the README shows the arithmetic); G6's overload bound re-derived in the README with the real
   filter (ruling 35), `G6.yaml` edited only where the derivation changes the derived bound.
6. Encoders, fasteners, impact: a real magnetic linear encoder strip (counts per metre from its resolution), M5
   class 8.8 bolt load ratings for the mounts (steel_8_8 in the library), impact parameters left as placeholders
   unless a source exists; mismatch threshold, heartbeat and watchdog stay the firmware's defaults, flagged.
7. Re-derivations: the suite's evaluation start from rail_x's new damping and frequency (the README's method, five
   loaded time constants, derived once and not adjusted; edit `suite`-wide cases only where the start is a case
   field); G12's swing stays physical (no anti-sway); then one whole-suite run with `--write-baseline` on native;
   compare with the previous baseline entry by entry: every status or value change listed with cause; which strict
   expected failures now pass for a stated physical reason (convert them, keeping the reason as history), which
   still fail and why (G12's settle, G15's safety factor and validity, ACC-038).
8. `docs/gantry-team-inputs.md` rewritten row by row: value, source URL, "replaceable" flag; `docs/milestones.md`
   G12, G15, G16 rows; the README. Then the one wide `-m "not long_run"` run and the lints.

## Report

Under 350 words: commits; per slot the chosen part or value with its source; the re-derived evaluation start and G6
bound; the baseline comparison (statuses changed, values moved by case, causes); the expected failures that flipped
and the ones that stand; what could not be sourced; the wide run's result; lints; open issues.
