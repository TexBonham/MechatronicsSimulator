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
| ACC-025 | Reference design (gantry), G12: after a fast move and after an E-stop, the peak load swing angle shall be within its limit. | Requirements doc, Test cases, G12; brief M3-X1a (native, rated load on the rope, 100 microseconds) | M3 | acceptance | 0.1 rad placeholder (`swing_limit_angle`, owed by gantry team); measured peaks also within the derived bounds 2 a / g (first ramp, from the hook's recorded motion), 7 a / g (whole move), v / sqrt(g L) (E-stop) on top of the free swing present, L = 0.37 m (hook fitting 0.02 m, rope 0.3 m, half the payload 0.05 m; brief M3-I4); first ramp a priori (brief M3-X1f): the hook's recorded acceleration within a_max = a p(zeta), p(zeta) the first maximum of a second-order follower's acceleration when its input's acceleration steps to a (belt spring with its damper in parallel, BLD-041: 1 - exp(-zeta w t) (cos w_d t - zeta / sqrt(1 - zeta^2) sin w_d t), 2 at zeta = 0), zeta the smallest ratio the run prints for the axis belt, and the swing within R0 + 2 a_max / g |
| ACC-026 | Reference design (gantry), G12: after a fast move and after an E-stop, the swing settle time shall be within its limit. | Requirements doc, Test cases, G12; brief M3-X1a | M3 | acceptance | 5 s placeholder (`swing_settle_time`) into a 0.01 rad band placeholder (`swing_settle_band`), both owed by gantry team; the model has no swing damping, so the settle fails (M3-X1a finding) |
| ACC-027 | Reference design (gantry), G13: a lift command while the gantry is moving shall be handled by the defined policy (queue or reject) in every run. | Requirements doc, Test cases, G13 | M2 | acceptance | not set (owed by gantry team) |
| ACC-028 | Reference design (gantry), G14: under rapid-fire and conflicting commands, no two actuator commands shall overlap. | Requirements doc, Test cases, G14 | M2 | acceptance | not set (owed by gantry team) |
| ACC-029 | Reference design (gantry), G15: under maximum acceleration with rated load and an E-stop with a swinging load, every member shall stay above the minimum safety factor. | Requirements doc, Test cases, G15; ruling 52 (the case carries the design load); brief M3-X1a | M3 | acceptance | 1.5 (`minimum_safety_factor`, gantry team 2026-11-11); a result of record needs ruling 73's three conditions (brief M3-X1f): no member flagged (ruling 69) and the verdict unchanged with stress scaled by 2.04 over the member's peak dynamic ratio, the verdicts reproduced at 10, 20 and 40 substeps (no agreement tolerance; the dynamic ratio is reported, ruling 77), the loaded first frequency against sqrt(k / (m_c + f m_beam)) reported; peak stress between the continuum static stress less 2 percent and 2.04 times it |
| ACC-030 | Reference design (gantry), G15: under the same worst cases, member deflection shall stay under the target. | Requirements doc, Test cases, G15; brief M3-X1a (dynamic node-pose peak, the conservative reading) | M3 | acceptance | 1 cm; the same three conditions as ACC-029 with deflection scaled; peak at most 2.04 times the static value, settled within 2 percent of the closed form |
| ACC-031 | Reference design (gantry), G16: a sweep of NEMA 17 against NEMA 23 with the rated load shall report the torque margin per motor. | Requirements doc, Test cases, G16 | M3 | acceptance | not set (owed by gantry team) |
| ACC-032 | Reference design (gantry), G16: the same sweep shall report any missed steps per motor. | Requirements doc, Test cases, G16 | M3 | acceptance | not set (owed by gantry team) |
| ACC-033 | Pendulum design, rigid link: the small-amplitude swing period shall match the damped small-angle period for the link, T0 / sqrt(1 - zeta^2) with T0 = 2 pi sqrt(I / (m g d)) (2 pi sqrt(L / g) for a point mass) and zeta the damping ratio of the link's joint damping, within the period tolerance. | Ruling R-03; Architecture doc, Technology recommendation, Validation; Repository starting point, milestone 1 paragraph | M1 | acceptance | 0.1 % of the damped period (owner confirmation pending) |
| ACC-034 | Pendulum design, rigid link: the swing amplitude decay shall match the closed-form decay for the link's stated joint damping within the decay tolerance. | Ruling R-03; Architecture doc, Technology recommendation, Validation; Repository starting point, milestone 1 paragraph | M1 | acceptance | not set (owed by owner) |
| ACC-035 | Acceptance criterion 1: the simulator shall build the reference design from data files or an imported URDF with no design-specific code. | Requirements doc, Outputs and acceptance criteria, criterion 1; Architecture doc, Repository starting point, invariant 2 | M1 | acceptance | n/a |
| ACC-036 | Acceptance criterion 2: the simulator shall run G1 to G16 in one batch and produce the pass/fail report. | Requirements doc, Outputs and acceptance criteria, criterion 2 | M3 | acceptance | n/a |
| ACC-037 | Acceptance criterion 3, rewritten: the rope pendulum's small-amplitude period shall match the chain's closed-form period with the rope's added mass and static stretch derived (ADR 0016), and its swing amplitude shall match the closed-form decay, within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 3; Architecture doc, Decisions log rows 24, 48; Technology recommendation, Validation; ADR 0016 | M3 | acceptance | proposed: 0.1 % of the period; amplitude over the period after 20 periods within 2 % of the release amplitude (owner) |
| ACC-038 | Acceptance criterion 3, rewritten: the stepper's pull-out torque shall match its published curve within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 3; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M2 | acceptance | not set (owed by owner) |
| ACC-039 | Acceptance criterion 3, rewritten: the settle time of a step response shall match its closed-form value within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 3; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M3 | acceptance | proposed: settle time between 0.98 t_s - pi / omega_d and 1.02 t_s, t_s = ln(50) / (zeta omega_n) continuous, at a step where backward Euler's correction is inside the 2 % (owner) |
| ACC-040 | Acceptance criterion 4, rewritten: a cantilever beam's static deflection shall match the closed-form value within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 4; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M3 | acceptance | proposed: 2 % of w L^4 / (8 E I), mean node-pose deflection over a final window of the settled run (owner) |
| ACC-041 | Acceptance criterion 4, rewritten: a cantilever beam's first natural frequency shall match the closed-form value within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 4; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M3 | acceptance | proposed: 2 % of the closed-form first frequency, step with atan(x) / x within 0.1 % (owner) |
| ACC-042 | Acceptance criterion 4, rewritten: a simply supported beam's static deflection shall match the closed-form value within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 4; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M3 | acceptance | proposed: 2 % of 5 w L^4 / (384 E I), mean node-pose deflection over a final window of the settled run (owner) |
| ACC-043 | Acceptance criterion 4, rewritten: a simply supported beam's first natural frequency shall match the closed-form value within tolerance. | Requirements doc, Outputs and acceptance criteria, criterion 4; Architecture doc, Decisions log row 24; Technology recommendation, Validation | M3 | acceptance | proposed: 2 % of the closed-form first frequency, step with atan(x) / x within 0.1 % (owner) |
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
| ACC-037 tolerances and step | Proposed (ADR 0016): period within 0.1 % of T_N, amplitude after 20 periods within 2 % of the release amplitude; they hold at a 50 microsecond physics step, which the case states (round 5 ruling 21, ruling 45: x = omega h = 1.57e-4, backward Euler keeps (1 + x^2)^(-pi / atan x) of the swing per period, 1 % lost over 20 periods, ruling 61). Case `test_cases/rope_pendulum/R1_period.yaml` (design `designs/rope_pendulum`, native adapter, rope rest length 1 m, M = 1 kg, m_r = 0.035 kg, N = 8, T_N = 2.0004181 s); the oracle lives in the case (ruling 60): `period` (SCO-027, period about the mean of `pose:bob.x`, which swings about -L sin(0.02)) within 0.1 % of T_N, `amplitude_after_20_periods` (half the peak-to-peak swing from 40 s) within 2 % of L sin(0.02) = 0.0199990659 m. A rope 5 % longer placed at its own loaded length lengthens the period by 2.5 % and fails `period`. | owner |
| Analytical tolerances of criteria 3 and 4 (ACC-038 to ACC-043) | ACC-038 not set. Proposed (briefs X1b, X1c; derivations in `tests/acceptance/test_acc_stubs.py` and the case files; the oracle lives in each case, two-sided, ruling 60, and a perturbed run of each case must fail its criterion). ACC-039 (`test_cases/rope_spring/S1_settle.yaml`, mass on a one-segment rope, M = 1.0175 kg, k = 500 kN/m, c = 50 N s/m): judged against the continuous closed form t_s = ln(50) / (zeta omega_n) = 2 M ln(50) / c = 0.159219 s (ruling 59; the exact envelope crossing ln(50 / sqrt(1 - zeta^2)) / (zeta omega_n) is 0.016 % later). The scorer's settle time is the first sample after the last exit from the 2 % band; the last exit falls at a peak under the envelope, at most half a damped period pi / omega_d = 4.484 ms before the envelope enters the band, so the band is [0.98 t_s - pi / omega_d, 1.02 t_s] = [0.151551 s, 0.162404 s] (criteria `settle_not_early`, `settle_not_late`). The case runs at a 1 microsecond step: backward Euler decays each mode s at sigma_h = ln abs(1 - h s) / h = zeta omega_n (1 + k h / c) to first order (ruling 45), 1.0 % faster here (k / c = 1e4 /s), inside the 2 %; the corrected value 0.157647 s is the published approximation line. A rope 5 % longer lowers k and c by 1 / 1.05; the static stretch grows 5 % (2.5 band widths) and the run never settles about the stated target. ACC-040, ACC-042 (`test_cases/cantilever_beam/B1_cantilever.yaml`, `test_cases/simple_beam/B2_simply_supported.yaml`, a 10 mm square bar of steel_8_8_fastener, E 205 GPa, under its own weight, N = 8): `settled_deflection`, the mean of the node-pose deflection (INT-006, SCO-028) over the final second (B1, 5 to 6 s) or half second (B2, 2.5 to 3 s), within 2 % of w L^4 / (8 E I) = 5.634768 mm and 5 w L^4 / (384 E I) = 0.586955 mm (rulings 31, 32; the chain under the distributed load reads 0.78 % and 0.625 % low); a second moment 5 % larger lowers the deflection 4.8 % and fails it. The peak over the run (the integrity table) includes the release overshoot, about twice static. ACC-041, ACC-043: `first_frequency`, the frequency about the mean of the deflection (SCO-027) from 0.25 s to 1.75 s (B1, twelve periods; the second mode, x = 0.33, keeps 0.37 per period, under 1e-5 by 0.25 s) and from 0.1 s to 0.7 s (B2, thirteen periods; no antisymmetric mode under a uniform load, the third keeps 0.37 per period), within 2 % of 1.8751^2 / (2 pi) sqrt(E I / (rho A L^4)) = 8.255100 Hz and (pi / 2) sqrt(E I / (rho A L^4)) = 23.172418 Hz, at the case's step (1 ms, 0.25 ms) where atan(x) / x is within 0.1 % (x = 0.052, 0.036). | owner |
| Design-load rule of the reference design | rated load plus 15 % (hoist, trolley); rated load not set | gantry team |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Hardware measurements replacing the analytical checks once the gantry is built | Roadmap (Architecture doc, Technology recommendation, Validation) |
| Simscape Multibody and Fusion Simulation comparisons | Replaced by ACC-037 to ACC-043 (Decisions log row 24) |
| Gantry firmware behavior that makes these pass | `firmware/` |
| Evaluating each criterion | SCO |
