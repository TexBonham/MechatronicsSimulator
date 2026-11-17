# Milestones

Sources: requirements doc "Instructions for the planning session" (milestone plan), architecture doc "Repository starting point" and Risks, rulings 2026-11-02, ADRs 0001, 0004, 0009, 0010, 0011, 0012. Phase sequence before milestone 1 (W-06): 0a rulings and ADRs; 0c scaffolding; 0b spec and stubs with the calibration spike; phase 1 contracts, schemas, facade protocol.

## Milestones 1 to 5

| | M1 | M2 | M3 | M4 | M5 |
|---|---|---|---|---|---|
| Goal | Rigid gantry model, controller runtime, tests G1-G4 | Sensors, fault injection, safety hardware | Rope swing, flexible members, structural checks | CAD import, FEA load export, Simscape and Fusion cross-checks | A second, different design proves the app is general |
| Test cases | G1-G4; pendulum references (R-03) | G5-G11, G13, G14 | G12, G15, G16 (torque margin and missed steps per motor and part carried by the report, SCO-031, SCO-032, judged from the last rest at or before the evaluation start, 1 s since brief M3-GV (9 s before), with a before-span note per motor, rulings 91, 93) | Acceptance criteria 3 and 4 (G3, G12, G15 cross-checks) | Acceptance criterion 5 (two-joint arm, R-03) |
| Engine | MuJoCo adapter (ADR 0004) | MuJoCo adapter (ADR 0004) | Own engine (native adapter); MuJoCo retired at the gate, 2026-11-15 (ADR 0004) | Own engine | Own engine |
| Library components | Limit switches, encoders, step/dir drivers, UART operator link, power rail with E-stop relay | not set | not set | not set | Library as shipped, no new code (acceptance criterion 5) |
| Integrity checks | None | None | Strength, deflection, buckling, fatigue, fastener, impact, resonance screening (ADR 0009); in-loop tier 1 and multi-segment beams; part of tier 2 (ADR 0010) | Bearing life; build-time rules: support spacing, cantilever ratio, clearance, misalignment binding, tip-over (ADR 0009); rest of tier 2 (ADR 0010) | None |
| Other scope | Native controller runtime; designs/gantry and designs/pendulum; record, score, replay from archive; recorder streaming seam (invariant 9) | Fault fan-out, facade service and browser client, Windows CI (R1): done | Rope model ADR 0016 (P4, round 5 ruling 2); transmission (belt) compliance in the physics model (round 4 ruling 35), G6 re-derived | Emulated mode on Renode; Renode transport ADR (C6); I2C and SPI peripheral modes in the chip vocabulary and signal layer (round 4 ruling 2), serial sensors drop their analog stand-in | designs/<name>/ and test_cases/<name>/, data only |
| Gate | G1-G4 and pendulum references pass on the MuJoCo adapter; Windows joins CI (R1) | Listed G cases pass | Listed G cases pass; G12 limits and minimum safety factor not set (owed by gantry team). On the sourced parts (brief M3-GV, 2026-11-15): G15 safety factors pass (rail_x 37.96, rail_y 24.04, neither rail flagged) and both settled-deflection criteria pass against targets re-derived for the 2040 section (brief M3-GV2: 1.4541401e-5 and 5.2288134e-5 m, measured 1.45415e-5 and 5.22721e-5 m; ACC-029 and ACC-030 converted on the 10-substep record, ruling 103); G12 settle still fails (no swing damping); G16 passes nominally, and its NEMA 17 lift run stalls at 7.43 s (margin -0.308 N m) on the published curve's 60 rpm point (0.30 N m); design finding (ruling 99 (e)): the lift belt carries 55.5 N in G15 (63.5 N on the sizing rule) against the GT2 6 mm working tension 26.2 N, no GT2 width up to 12 mm covers it, the 6 mm belt stays flagged (`docs/gantry-team-inputs.md`) | Criteria 3 and 4 as analytical checks (architecture doc, Validation); tolerances not set | Second design builds and tests with library and data files only |

Tier 2 split between M3 and M4: M3 yield offset and rope snap, M4 buckling collapse and fastener break (ADR 0010
amendment 2026-11-11, round 5 ruling 3; owner confirms at the gate).

## Engine packages in parallel

Built alongside milestone 1, which ran on the MuJoCo adapter (ADR 0001, ADR 0005 consequences; retired 2026-11-15).

| # | Package | Status |
|---|---|---|
| 1 | Math core and integrators | Calibration spike (R-12), before phase 1 |
| 2 | Rigid multibody: six joint types, kinematic loops | Calibration spike (R-12), before phase 1 |

ADR 0001 names packages 1 and 2 only. "Engine package breakdown beyond packages 1 and 2: after the spike, using its cost model" (ADR 0001 open item). Working names from the phase 0c brief, unnumbered and not ruled: constraints and limits; collision detection; contact solver; flexible members; cables and ropes; transmissions and actuators; failure coupling; determinism and solver health; debug viewer; bindings and packaging; validation suite.

Spike references: pendulum and loaded beam. Output: the project's measured cost model.

## MuJoCo adapter retirement criteria (ADR 0004)

Retired on 2026-11-15 at 29a41d0 (brief M3-RM; owner gate decision 1b, its condition met when brief M3-X2i landed).
Criteria as decided by the owner: criterion 1 met; criterion 2 decided, the four G10 misses below stand as the record
and E9 (the efficiency-damping release artefact, bounded and published per run) is milestone 4's item. The rows below
are the record as it stood at the gate.

| Criterion | | Status (briefs M3-X2b, M3-X2c, 2026-11-14) | Evidence |
|---|---|---|---|
| 1 | Own engine passes the analytical reference suite | Met | PHY-023, `tests/acceptance/test_phy_stubs.py::test_phy_023`: the reference tier enumerated and run on native in a fresh interpreter, 94 passed, 4 expected failures (other modules' unconverted stubs), the 2 bridge-only variants left out; every bridge-parametrized test has its native variant. PHY-021 differential (`tests/reference/test_phy_stubs.py::test_phy_021`) passed; both retired with the bridge, the tier now runs whole on native |
| 2 | Own engine matches the gantry baselines | Not met: four misses of record, all G10, now explained (brief M3-X2i: a model difference at the homing latch and its consequences); the owner decides whether they stand in the way of retirement (ruling 83); E9 bounded and published per run (brief M3-X2i) | PHY-024, `tests/acceptance/test_phy_stubs.py`: `test_phy_024`, a strict expected failure asserting no miss, and `test_phy_024_misses_of_record`, which passes only while the misses equal the list of record (brief M3-X2d). Each criterion is compared at its own axis's tolerance (differential tolerance row, `docs/spec/phy.md`): positions one full step (0.2 mm) plus that axis's belt stretch at rest in that case (lift: (payload + the build's driven mass, 0.4064 kg) g on 1e5 N/m, 0.240 mm empty, 0.436 mm with 2 kg, 0.779 mm with 5.5 kg; x and y 0.2 mm, no load at rest); event times 3 ms; counts exact. Four misses of record, all G10, after the native baseline's regeneration with E8 (brief M3-X2h; 15 values in the rope cases G5, G6, G7, G9, G12, G15 moved, no status changed): G10 `stall_fault` (8 ms) and `slip_fault` (20 ms) explained, the firmware's mismatch rebuilt from both archives (brief M3-X2i): native's counts run ahead by the homing settle (counts zeroed at the latch, then the compliant belt lets go of the homing tension: 2.89 and 2.80 counts), the drive's belt stretch (2.99 counts) and the coast (-0.02 and +0.26 counts), so the crossing comes 3.44 and 9.93 ms earlier and the stop phase v / a, v rising at a, shortens by as much: 6.9 and 19.9 ms predicted against 8 and 20 ms, the rest count and tick quantisation; a model difference (compliant against rigid x belt), not an engine fault; G10 `stall_rehomed` (14 ms) and `slip_carriage_stopped` (0.73 mm against 0.2 mm, which passed before only under the pooled 0.783 mm) its consequences. G7 `hook_held` and G9 `load_held` explained (brief M3-X2g, ruling 94) and no longer misses: the drive is cleared (no rotor damping); on native (a) the belt's efficiency loss, an implicit shaft damping frozen at the step's start rate (ruling 47), holds the released rotor about 2 ms, and (b) before E8 the rope went slack and did not take up again; a three-mass backward-Euler model predicted 2.180 and 1.140 mm against 2.177 and 1.146 observed; with E8 native holds 1.821 mm against the bridge's 2.177 mm, 0.357 mm at the head, inside 0.436 mm. E9 (ruling 94): mechanism (a) bounded and published (brief M3-X2i): per step the frozen damping removes abs(omega_end) / max(abs(omega_start), omega_s) times the loss, above 2 only for a shaft leaving rest; a released rotor lags at most 1 + ln(beta^2 a h / omega_s) / ln((1 + beta) / (2 beta)) + 1/2 steps plus a logarithmic tail (G7: 4.6 steps bound, 3.2 to 3.6 by the recurrence); each native run states the largest ratio per transmission (G7 lift belt 19.97 at 15.033 s); the end-rate form is milestone 4 item E9. G6 `lift_stopped` (0.600 mm) is within its own 5.5 kg lift tolerance. G1 to G5, G8, G11, G13, G14 and P1, P2 within tolerance. Native baseline `test_cases/gantry/baseline.yaml` against `test_cases/gantry/bridge_baselines/`; causes in `test_cases/gantry/README.md` |
| After retirement | Comparison recordings kept as fixed baselines (YAML scoring summaries in git, D4); MuJoCo no longer a runtime dependency | Done 2026-11-15 (brief M3-RM): baselines in place, MuJoCo removed from the build, the wheel, the runtime and CI; `physics.adapter: mujoco` refused naming the retired adapter | PHY-025, `tests/unit/test_phy_stubs.py::test_phy_025`: 13 gantry and 2 pendulum fixed bridge baselines, generated once. PHY-026, `tests/invariant/test_phy_stubs.py`: nothing anywhere imports MuJoCo or the bridge package (import checker), no source includes a MuJoCo header, a run loads no MuJoCo module and the extension registers no MuJoCo adapter |

Gated on the criteria, not a date.

## Open values owed by the gantry team

From `docs/rulings/2026-11-02-planning-rulings.md`, "Still open":

| Value | Where it bites (source) |
|---|---|
| STM32 part, after the peripheral smoke test | M4 emulated mode (architecture doc Risks) |
| Encoder placement | Design parameter, both supported (Decisions log row 14) |
| Flexible members, sections, static or dynamic 1 cm target | M3; planning default: rails flexible, static target, dynamic reported (Decisions log row 17) |
| Workspace dimensions and rated load | G15, G16 use rated load; design-load rule |
| Swing angle and settle time limits for G12; minimum safety factor | G12 and G15 pass criteria |
| New-target-during-move and lift-while-moving policies | G13 "defined policy" |
