# Milestones

Sources: requirements doc "Instructions for the planning session" (milestone plan), architecture doc "Repository starting point" and Risks, rulings 2026-11-02, ADRs 0001, 0004, 0009, 0010, 0011, 0012. Phase sequence before milestone 1 (W-06): 0a rulings and ADRs; 0c scaffolding; 0b spec and stubs with the calibration spike; phase 1 contracts, schemas, facade protocol.

## Milestones 1 to 5

| | M1 | M2 | M3 | M4 | M5 |
|---|---|---|---|---|---|
| Goal | Rigid gantry model, controller runtime, tests G1-G4 | Sensors, fault injection, safety hardware | Rope swing, flexible members, structural checks | CAD import, FEA load export, Simscape and Fusion cross-checks | A second, different design proves the app is general |
| Test cases | G1-G4; pendulum references (R-03) | G5-G11, G13, G14 | G12, G15, G16 | Acceptance criteria 3 and 4 (G3, G12, G15 cross-checks) | Acceptance criterion 5 (two-joint arm, R-03) |
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

| Criterion | |
|---|---|
| 1 | Own engine passes the analytical reference suite |
| 2 | Own engine matches the gantry baselines |
| After retirement | Comparison recordings kept as fixed baselines (YAML scoring summaries in git, D4); MuJoCo no longer a runtime dependency |

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
