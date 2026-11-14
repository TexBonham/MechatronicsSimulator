# Milestones

Sources: requirements doc "Instructions for the planning session" (milestone plan), architecture doc "Repository starting point" and Risks, rulings 2026-11-02, ADRs 0001, 0004, 0009, 0010, 0011, 0012. Phase sequence before milestone 1 (W-06): 0a rulings and ADRs; 0c scaffolding; 0b spec and stubs with the calibration spike; phase 1 contracts, schemas, facade protocol.

## Milestones 1 to 5

| | M1 | M2 | M3 | M4 | M5 |
|---|---|---|---|---|---|
| Goal | Rigid gantry model, controller runtime, tests G1-G4 | Sensors, fault injection, safety hardware | Rope swing, flexible members, structural checks | CAD import, FEA load export, Simscape and Fusion cross-checks | A second, different design proves the app is general |
| Test cases | G1-G4; pendulum references (R-03) | G5-G11, G13, G14 | G12, G15, G16 (torque margin and missed steps per motor and part carried by the report, SCO-031, SCO-032) | Acceptance criteria 3 and 4 (G3, G12, G15 cross-checks) | Acceptance criterion 5 (two-joint arm, R-03) |
| Engine | MuJoCo adapter (ADR 0004) | MuJoCo adapter (ADR 0004) | MuJoCo adapter until retirement criteria met, then own engine (ADR 0004 open item) | Same rule | Same rule |
| Library components | Limit switches, encoders, step/dir drivers, UART operator link, power rail with E-stop relay | not set | not set | not set | Library as shipped, no new code (acceptance criterion 5) |
| Integrity checks | None | None | Strength, deflection, buckling, fatigue, fastener, impact, resonance screening (ADR 0009); in-loop tier 1 and multi-segment beams; part of tier 2 (ADR 0010) | Bearing life; build-time rules: support spacing, cantilever ratio, clearance, misalignment binding, tip-over (ADR 0009); rest of tier 2 (ADR 0010) | None |
| Other scope | Native controller runtime; designs/gantry and designs/pendulum; record, score, replay from archive; recorder streaming seam (invariant 9) | Fault fan-out, facade service and browser client, Windows CI (R1): done | Rope model ADR 0016 (P4, round 5 ruling 2); transmission (belt) compliance in the physics model (round 4 ruling 35), G6 re-derived | Emulated mode on Renode; Renode transport ADR (C6); I2C and SPI peripheral modes in the chip vocabulary and signal layer (round 4 ruling 2), serial sensors drop their analog stand-in | designs/<name>/ and test_cases/<name>/, data only |
| Gate | G1-G4 and pendulum references pass on the MuJoCo adapter; Windows joins CI (R1) | Listed G cases pass | Listed G cases pass; G12 limits and minimum safety factor not set (owed by gantry team) | Criteria 3 and 4 as analytical checks (architecture doc, Validation); tolerances not set | Second design builds and tests with library and data files only |

Tier 2 split between M3 and M4: M3 yield offset and rope snap, M4 buckling collapse and fastener break (ADR 0010
amendment 2026-11-11, round 5 ruling 3; owner confirms at the gate).

## Engine packages in parallel

Built alongside milestone 1, which runs on the MuJoCo adapter (ADR 0001, ADR 0005 consequences).

| # | Package | Status |
|---|---|---|
| 1 | Math core and integrators | Calibration spike (R-12), before phase 1 |
| 2 | Rigid multibody: six joint types, kinematic loops | Calibration spike (R-12), before phase 1 |

ADR 0001 names packages 1 and 2 only. "Engine package breakdown beyond packages 1 and 2: after the spike, using its cost model" (ADR 0001 open item). Working names from the phase 0c brief, unnumbered and not ruled: constraints and limits; collision detection; contact solver; flexible members; cables and ropes; transmissions and actuators; failure coupling; determinism and solver health; debug viewer; bindings and packaging; validation suite.

Spike references: pendulum and loaded beam. Output: the project's measured cost model.

## MuJoCo adapter retirement criteria (ADR 0004)

| Criterion | | Status (briefs M3-X2b, M3-X2c, 2026-11-14) | Evidence |
|---|---|---|---|
| 1 | Own engine passes the analytical reference suite | Met | PHY-023, `tests/acceptance/test_phy_stubs.py::test_phy_023`: the reference tier enumerated and run on native in a fresh interpreter, 94 passed, 4 expected failures (other modules' unconverted stubs), the 2 bridge-only variants left out; every bridge-parametrized test has its native variant. PHY-021 differential (`tests/reference/test_phy_stubs.py::test_phy_021`) passes |
| 2 | Own engine matches the gantry baselines | Not met; the owner decides at the gate whether the explained model differences satisfy it (ruling 83) | PHY-024, `tests/acceptance/test_phy_stubs.py`: `test_phy_024`, a strict expected failure asserting no miss, and `test_phy_024_misses_of_record`, which passes only while the misses equal the list of record (brief M3-X2d). Each criterion is compared at its own axis's tolerance (differential tolerance row, `docs/spec/phy.md`): positions one full step (0.2 mm) plus that axis's belt stretch at rest in that case (lift: (payload + the build's driven mass, 0.4064 kg) g on 1e5 N/m, 0.240 mm empty, 0.436 mm with 2 kg, 0.779 mm with 5.5 kg; x and y 0.2 mm, no load at rest); event times 3 ms; counts exact. Six misses of record: G7 `hook_held` and G9 `load_held` (bridge 2.177 mm, native 1.146 mm, tolerance 0.436 mm) explained, a modelling difference (brief M3-X2g): the drive is cleared (no rotor damping); on native the belt's efficiency loss, an implicit shaft damping frozen at the step's start rate (ruling 47), holds the released rotor about 2 ms, and the rope goes slack and, before E8, did not take up again, so the brake stopped the 0.4074 kg carriage alone; a three-mass backward-Euler model of these terms predicts 2.180 and 1.140 mm (1.040 mm against 1.031 observed). Since E8 native records 1.821 mm, 0.357 mm from the bridge, inside 0.436 mm: the two misses remain only because `baseline.yaml` predates E8 (owner: regenerate the native G7 and G9 entries); G10 `stall_fault` (8 ms) and `slip_fault` (20 ms) a finding: the x belt's stretch under the drive torque (0.0496 mm) predicts 3.4 and 9.4 ms, the stop phase and the coast during the stall not yet accounted (M3-X2g item 2 not finished; the head reproduces 9.153 and 31.407 s on native, 9.161 and 31.427 s on the bridge); G10 `stall_rehomed` (14 ms) and `slip_carriage_stopped` (0.73 mm against 0.2 mm, which passed before only under the pooled 0.783 mm) follow from them. G6 `lift_stopped` (0.600 mm) is within its own 5.5 kg lift tolerance. G1 to G5, G8, G11, G13, G14 and P1, P2 within tolerance. Native baseline `test_cases/gantry/baseline.yaml` against `test_cases/gantry/bridge_baselines/`; causes in `test_cases/gantry/README.md` |
| After retirement | Comparison recordings kept as fixed baselines (YAML scoring summaries in git, D4); MuJoCo no longer a runtime dependency | Baselines in place; MuJoCo still a dependency (removal is a separate commit after the owner's decision, `docs/briefs/round-05/mujoco-removal.md`) | PHY-025, `tests/unit/test_phy_stubs.py::test_phy_025`: 13 gantry and 2 pendulum fixed bridge baselines, generated once. PHY-026, `tests/invariant/test_phy_stubs.py` (two tests): no runtime module imports MuJoCo at module level and a native run loads no bridge module; both pass |

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
