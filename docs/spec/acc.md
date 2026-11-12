# Spec: Acceptance (ACC)

## Purpose

The acceptance requirements the simulator is judged by: every pass criterion of the reference design's suite G1 to G16
(the gantry, the first test case and never the design target), the pendulum references of milestone 1, and the six
acceptance criteria of the requirements doc, with criteria 3 and 4 rewritten as analytical checks. Each is run through
the test runner (TST) and judged by scoring (SCO); the cases and their values live as data under `designs/` and
`test_cases/`, never in code.

"Architecture doc" below is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`;
"Requirements doc" is `docs/Mechatronic Simulator — Requirements.md`. Requirements labelled "Reference design (gantry)"
are the gantry suite; values the gantry team has not set are open parameters.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| TST | ACC-001 to ACC-045 | TST-024, TST-025, TST-026, TST-032 |
| SCO | ACC-001 to ACC-036 | SCO-008, SCO-020 to SCO-026 |
| INT | ACC-029, ACC-030, ACC-040 to ACC-043 | INT-020, INT-021, INT-032 |
| INV | ACC-035, ACC-044, ACC-045 | INV-002, INV-004, INV-014 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| ACC-001 | Reference design (gantry), G1: when target (40, 25) is sent at power on before homing, no actuator shall move. | Requirements doc, Test cases, G1; M1-X6 coordinator ruling 2 (no step pulse on any axis for the whole run and every axis within its power-on hold band), for owner confirmation | M1 | acceptance | 1 mm hold band (placeholder, owed by gantry team) |
| ACC-002 | Reference design (gantry), G1: the refusal shall be reported with status "not homed". | Requirements doc, Test cases, G1 | M1 | acceptance | not set (owed by gantry team) |
| ACC-003 | Reference design (gantry), G2: after homing, each axis's origin shall lie within the tolerance of its true origin. | Requirements doc, Test cases, G2 | M1 | acceptance | 1 cm |
| ACC-004 | Reference design (gantry), G2: after homing, the status shall be "ready". | Requirements doc, Test cases, G2 | M1 | acceptance | not set (owed by gantry team) |
| ACC-005 | Reference design (gantry), G3: after target (40, 25) is sent following homing, the hook's final position shall lie within the tolerance of the target. | Requirements doc, Test cases, G3 | M1 | acceptance | 1 cm |
| ACC-006 | Reference design (gantry), G3: no travel limit shall be exceeded during the move. | Requirements doc, Test cases, G3 | M1 | acceptance | not set (owed by gantry team) |
| ACC-007 | Reference design (gantry), G4: for a target outside the workspace, no actuator shall move. | Requirements doc, Test cases, G4; M1-X6 coordinator ruling 2 (as ACC-001, from the target on), for owner confirmation | M1 | acceptance | 1 mm hold band (placeholder, owed by gantry team) |
| ACC-008 | Reference design (gantry), G4: the target shall be rejected with a reported fault, not clipped. | Requirements doc, Test cases, G4 | M1 | acceptance | not set (owed by gantry team) |
| ACC-009 | Reference design (gantry), G5: with a load over the limit attached before a move, a lift command shall be ignored. | Requirements doc, Test cases, G5 | M2 | acceptance | not set (owed by gantry team) |
| ACC-010 | Reference design (gantry), G5: with a load over the limit attached, a lower command shall work. | Requirements doc, Test cases, G5 | M2 | acceptance | not set (owed by gantry team) |
| ACC-011 | Reference design (gantry), G6: when the load becomes overweight mid-lift, the lift shall stop. | Requirements doc, Test cases, G6 | M2 | acceptance | not set (owed by gantry team) |
| ACC-012 | Reference design (gantry), G6: after an overweight stop, the load shall be able to be set down. | Requirements doc, Test cases, G6 | M2 | acceptance | not set (owed by gantry team) |
| ACC-013 | Reference design (gantry), G7: when the E-stop is pressed mid-move with a load, motor power shall be cut within the relay delay. | Requirements doc, Test cases, G7 | M2 | acceptance | not set (owed by gantry team: relay delay) |
| ACC-014 | Reference design (gantry), G7: after the E-stop, the hook shall not drop. | Requirements doc, Test cases, G7 | M2 | acceptance | not set (owed by gantry team) |
| ACC-015 | Reference design (gantry), G7: after the E-stop, motion shall resume only after a reset and a new command. | Requirements doc, Test cases, G7 | M2 | acceptance | not set (owed by gantry team) |
| ACC-016 | Reference design (gantry), G8: when the controller freezes mid-move, the motors shall stop within the heartbeat timeout. | Requirements doc, Test cases, G8 | M2 | acceptance | not set (owed by gantry team: heartbeat timeout) |
| ACC-017 | Reference design (gantry), G9: after power is lost and restored, no motion shall occur until the gantry is homed. | Requirements doc, Test cases, G9 | M2 | acceptance | not set (owed by gantry team) |
| ACC-018 | Reference design (gantry), G9: during the power loss and restore, the load shall not drop. | Requirements doc, Test cases, G9 | M2 | acceptance | not set (owed by gantry team) |
| ACC-019 | Reference design (gantry), G10: on a motor stall or belt slip during a move, motion shall stop. | Requirements doc, Test cases, G10 | M2 | acceptance | not set (owed by gantry team) |
| ACC-020 | Reference design (gantry), G10: on a motor stall or belt slip during a move, an encoder-mismatch fault shall be reported. | Requirements doc, Test cases, G10 | M2 | acceptance | not set (owed by gantry team) |
| ACC-021 | Reference design (gantry), G10: after a stall or slip fault, re-homing shall be required before further motion. | Requirements doc, Test cases, G10 | M2 | acceptance | not set (owed by gantry team) |
| ACC-022 | Reference design (gantry), G11: when a limit switch trips mid-workspace, motion shall stop. | Requirements doc, Test cases, G11 | M2 | acceptance | not set (owed by gantry team) |
| ACC-023 | Reference design (gantry), G11: when a limit switch trips mid-workspace, a fault shall be reported. | Requirements doc, Test cases, G11 | M2 | acceptance | not set (owed by gantry team) |
| ACC-024 | Reference design (gantry), G11: after a mid-workspace limit trip, re-homing shall be required before further motion. | Requirements doc, Test cases, G11 | M2 | acceptance | not set (owed by gantry team) |
| ACC-025 | Reference design (gantry), G12: after a fast move and after an E-stop, the peak load swing angle shall be within its limit. | Requirements doc, Test cases, G12 | M3 | acceptance | not set (owed by gantry team) |
| ACC-026 | Reference design (gantry), G12: after a fast move and after an E-stop, the swing settle time shall be within its limit. | Requirements doc, Test cases, G12 | M3 | acceptance | not set (owed by gantry team) |
| ACC-027 | Reference design (gantry), G13: a lift command while the gantry is moving shall be handled by the defined policy (queue or reject) in every run. | Requirements doc, Test cases, G13 | M2 | acceptance | not set (owed by gantry team) |
| ACC-028 | Reference design (gantry), G14: under rapid-fire and conflicting commands, no two actuator commands shall overlap. | Requirements doc, Test cases, G14 | M2 | acceptance | not set (owed by gantry team) |
| ACC-029 | Reference design (gantry), G15: under maximum acceleration with rated load and an E-stop with a swinging load, every member shall stay above the minimum safety factor. | Requirements doc, Test cases, G15 | M3 | acceptance | not set (owed by gantry team) |
| ACC-030 | Reference design (gantry), G15: under the same worst cases, member deflection shall stay under the target. | Requirements doc, Test cases, G15 | M3 | acceptance | 1 cm |
| ACC-031 | Reference design (gantry), G16: a sweep of NEMA 17 against NEMA 23 with the rated load shall report the torque margin per motor. | Requirements doc, Test cases, G16 | M3 | acceptance | not set (owed by gantry team) |
| ACC-032 | Reference design (gantry), G16: the same sweep shall report any missed steps per motor. | Requirements doc, Test cases, G16 | M3 | acceptance | not set (owed by gantry team) |
| ACC-033 | Pendulum design, rigid link: the small-amplitude swing period shall match the damped small-angle period for the link, T0 / sqrt(1 - zeta^2) with T0 = 2 pi sqrt(I / (m g d)) (2 pi sqrt(L / g) for a point mass) and zeta the damping ratio of the link's joint damping, within the period tolerance. | Ruling R-03; Architecture doc, Technology recommendation, Validation; Repository starting point, milestone 1 paragraph | M1 | acceptance | 0.1 % of the damped period (owner confirmation pending) |
| ACC-034 | Pendulum design, rigid link: the swing amplitude decay shall match the closed-form decay for the link's stated joint damping within the decay tolerance. | Ruling R-03; Architecture doc, Technology recommendation, Validation; Repository starting point, milestone 1 paragraph | M1 | acceptance | not set (owed by owner) |
| ACC-035 | Acceptance criterion 1: the simulator shall build the reference design from data files or an imported URDF with no design-specific code. | Requirements doc, Outputs and acceptance criteria, criterion 1; Architecture doc, Repository starting point, invariant 2 | M1 | acceptance | n/a |
| ACC-036 | Acceptance criterion 2: the simulator shall run G1 to G16 in one batch and produce the pass/fail report. | Requirements doc, Outputs and acceptance criteria, criterion 2 | M3 | acceptance | n/a |
| ACC-037 | Acceptance criterion 3, rewritten: the rope pendulum's small-amplitude period shall match the chain's closed-form period with the rope's added mass and static stretch derived (ADR 0016), and its swing amplitude shall match the closed-form decay, within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 3; Architecture doc, Decisions log rows 24, 48; Technology recommendation, Validation; ADR 0016 | M3 | acceptance | proposed: 0.1 % of the period; 2 % of the amplitude over 20 periods (owner) |
| ACC-038 | Acceptance criterion 3, rewritten: the stepper's pull-out torque shall match its published curve within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 3; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M2 | acceptance | not set (owed by owner) |
| ACC-039 | Acceptance criterion 3, rewritten: the settle time of a step response shall match its closed-form value within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 3; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M3 | acceptance | not set (owed by owner) |
| ACC-040 | Acceptance criterion 4, rewritten: a cantilever beam's static deflection shall match the closed-form value within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 4; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M3 | acceptance | not set (owed by owner) |
| ACC-041 | Acceptance criterion 4, rewritten: a cantilever beam's first natural frequency shall match the closed-form value within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 4; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M3 | acceptance | not set (owed by owner) |
| ACC-042 | Acceptance criterion 4, rewritten: a simply supported beam's static deflection shall match the closed-form value within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 4; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M3 | acceptance | not set (owed by owner) |
| ACC-043 | Acceptance criterion 4, rewritten: a simply supported beam's first natural frequency shall match the closed-form value within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 4; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M3 | acceptance | not set (owed by owner) |
| ACC-044 | Acceptance criterion 5: a second, different design (the two-joint arm) shall be built and tested using only the library and data files. | Requirements doc, Outputs and acceptance criteria, criterion 5; Ruling R-03 | M5 | acceptance | n/a |
| ACC-045 | Acceptance criterion 6: rerunning any case with the same seed shall give identical results. | Requirements doc, Outputs and acceptance criteria, criterion 6; Architecture doc, Repository starting point, invariant 4 | M1 | acceptance | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Reference design (gantry): workspace dimensions | not set | gantry team |
| Reference design (gantry): rated load | not set | gantry team |
| Reference design (gantry): overweight limit (G5, G6) | not set | gantry team |
| Reference design (gantry): E-stop relay delay (G7) | not set | gantry team |
| Reference design (gantry): watchdog heartbeat timeout (G8) | not set | gantry team |
| Reference design (gantry): swing angle limit and settle time limit (G12) | not set | gantry team |
| Reference design (gantry): lift-while-moving policy (G13) and new-target-during-move policy (G14) | not set | gantry team |
| Reference design (gantry): minimum safety factor (G15) | not set | gantry team |
| Reference design (gantry): 1 cm deflection target static or dynamic (G15) | not set | gantry team |
| Reference design (gantry): motion threshold for "no actuator motion" (G1, G4, G9) | not set | gantry team |
| Pendulum period tolerance (ACC-033) | 0.1 % of the damped period, consistent with ENG-036 (owner ruling 2026-11-04) | owner |
| Pendulum swing decay tolerance (ACC-034) | not set; proposed 2 % of the closed-form amplitude envelope over 20 periods | owner |
| Rope pendulum tolerances (ACC-037) | not set; proposed 0.1 % of the closed-form period and 2 % of the closed-form amplitude over 20 periods (ADR 0016) | owner |
| Analytical tolerances of criteria 3 and 4 (ACC-038 to ACC-043) | not set | owner |
| Design-load rule of the reference design | rated load plus 15 % (hoist, trolley); rated load not set | gantry team |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Hardware measurements replacing the analytical checks once the gantry is built | Roadmap (Architecture doc, Technology recommendation, Validation) |
| Simscape Multibody and Fusion Simulation comparisons | Replaced by ACC-037 to ACC-043 (Decisions log row 24) |
| Gantry firmware behavior that makes these pass | `firmware/` |
| Evaluating each criterion | SCO |
