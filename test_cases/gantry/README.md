Gantry test suite: G1 to G16. Every case runs on the native adapter, the default since M3-X2 (round 5 ruling 1);
milestone 2 shipped G1 to G11, G13 and G14 on the bridge, whose values of record are the fixed baselines in
`bridge_baselines/` (PHY-025); `suite.yaml` runs all of them; every open parameter has a placeholder default
(DES-044) until the gantry team rules.

## Evaluation start (TST-044, brief M3-X2)

One start for every case, derived once and not adjusted to any result: a native run starts from straight rails and
the release rings for a few loaded time constants of the slowest rail, so integrity results (peaks, safety factors,
tier 1 crossings, the validity flag, rope snap) count from five of them, rounded up to a whole second. Stiffness-
proportional damping gives zeta = beta omega / 2 for every mode, so a member stated zeta_0 at its unloaded first mode
omega_0 and realised zeta_r on its loaded mode has omega_c = omega_0 zeta_r / zeta_0 and time constant tau = 1 /
(zeta_r omega_c) = zeta_0 / (zeta_r^2 omega_0). Re-derived by brief M3-GV from the sourced 2040 rails (zeta_0 = 0.02),
from the figures every run prints (the case payload on its rope is not counted, so every case prints the same):
rail_x f_0 = 902.104 Hz (omega_0 = 5668.09 rad/s), zeta_r = 0.00565055 (carried 1.68236 kg), omega_c = 1601.39
rad/s, tau = 0.1105 s; rail_y f_0 = 400.379 Hz (omega_0 = 2515.66 rad/s), zeta_r = 0.00822979 (carried 0.917467 kg),
omega_c = 1035.17 rad/s, tau = 0.1174 s, now the slowest. Five tau = 0.587 s: `integrity.evaluate_from: 1 s` in G2 to
G16. G1 ends at 2 s and could now carry it; it has no `evaluate_from` field and none is added (the start is edited only
where it is a case field). Before brief M3-GV (placeholder PETG rails, zeta_0 = 0.01, rail_x f_0 = 49.6973 Hz, zeta_r =
0.0042905, tau = 1.740 s) the start was 9 s. What a case loses: integrity during its first second is not judged (the
release transient); homing, the moves and the stops all count. Motion criteria keep their own windows.

| Case | Requirements doc test | Criteria (spec rows) |
|---|---|---|
| `G1.yaml` | Target (40, 25) at power on, before homing | no actuator motion (ACC-001); refusal reported (ACC-002) |
| `G2.yaml` | Home | each origin within 1 cm of the true origin (ACC-003); status reported (ACC-004) |
| `G3.yaml` | Target (40, 25) after homing | hook within 1 cm of the target (ACC-005); no travel limit exceeded (ACC-006) |
| `G4.yaml` | Target (75, 25) outside the 0.5 m workspace after homing | no actuator motion (ACC-007); rejected with a reported fault (ACC-008) |
| `G5.yaml` | Overweight load attached before a move | lift up refused, `refused` event, lift held (ACC-009); lowered within 1 cm (ACC-010); loaded in steps of at most 1 kg, finer near the top, to 5.5 kg (over the 5.216 kg limit, rated 10 lb + 15 %); passes on the placeholder NEMA 23 (ruling 31, M2-X7) |
| `G6.yaml` | Load becomes overweight mid-lift | rated 4.536 kg lifted, 5.5 kg at command + 2 s; lift stops under the derived 21.1 cm bound, 2 Hz amplifier latency included (ACC-011, ruling 35); set down within 1 cm (ACC-012) |
| `G7.yaml` | E-stop mid-move with a load | hook lifted to 5 cm first (under the 10 cm homing travel) so the brake holds it; `rail:motor_power` and relay status 0 from press + 20 ms, recorded 12.021 s (ACC-013); lift held (ACC-014); no steps from the next tick until the new command, `reset`, `home`, target reached after (ACC-015) |
| `G8.yaml` | Controller freeze mid-move | motor rail off and no step pulses from freeze + 70 ms (ACC-016) |
| `G9.yaml` | Power loss and restore mid-move with a load | hook lifted to 5 cm first; not_homed after the restore, target refused, no steps until `home` (ACC-017); lift held through the loss (ACC-018) |
| `G10.yaml` | Motor stall, then belt slip, mid-move | fault, no steps after the stop bound (ACC-019), `fault:encoder_mismatch` (ACC-020, ruling 30) from the frame-side linear encoders (ruling 32), target refused, re-home (ACC-021); carriage stopped within the slip bound |
| `G11.yaml` | Limit switch forced mid-workspace | stop (ACC-022), `fault:limit_trip` (ACC-023, ruling 30), target refused, re-home (ACC-024) |
| `G12.yaml` | Load swing after a fast move and after an E-stop at full speed (native, rated load on `payload_rope`, 100 microseconds) | peak swing within `swing_limit_angle` (ACC-025; the hook-to-payload swing angle, `swing:hook.payload.<axis>`, in the case and the test, against 2 a / g, 7 a / g and 2 a / g + v / sqrt(g L)); settle within `swing_settle_time` (ACC-026; fails on the model's zero swing damping, finding) |
| `G13.yaml` | Lift command while moving | `lift 0.3` mid-move refused with `fault:busy` and a `refused` event, no lift steps, lift held, move arrives (ACC-027; reject policy is the firmware placeholder) |
| `G14.yaml` | Fifty rapid-fire and conflicting commands | `no_overlap` on each driver's `motion_start`/`motion_end` (ACC-028, SCO-025); first move's interval within its cruise and slow-zone bounds (SCO-021); malformed refused |
| `G15.yaml` | Maximum acceleration to mid-travel with the design load, then an E-stop with a swinging load (native, tier 1, no halt) | every member's safety factor at least `minimum_safety_factor` (ACC-029); node-pose deflection under 1 cm (ACC-030); no tier 1 crossing; load case exported; since brief M3-GV (2040 rails): safety factors 37.96 and 24.04, peaks 0.0164 and 0.0692 mm, dynamic ratios 1.011 and 1.131, neither flagged; the settled-deflection targets (1.0976722e-3 and 7.384859e-4 m) are the placeholder section's closed form and fail: re-derived on the 2040 they are 1.4556e-5 and 5.2347e-5 m, measured 1.4541e-5 and 5.2272e-5 m |
| `G16.yaml` | Motor sizing: NEMA 17 against NEMA 23 at the rated load | sweeps of `instance:<motor>.part` (LIB-017), one motor substituted per run, the nominal run all NEMA 23; ready, no `fault:encoder_mismatch`, lifted and arrived within 1 cm; torque margin (`torque_margin:<motor>`, SCO-032) and missed steps (`missed_steps:<motor>`, SCO-031) per motor in the report, margin minimum above 0 and final count 0 within a quarter step, checked by ACC-031 and ACC-032 against their derivations (briefs M3-X2e, M3-X2f): both judged from the last rest at or before the 9 s evaluation start to a final rest (rulings 91, 93), what precedes it on a before-span note per motor; on native 2026-11-14 every run passes: nominal lift NEMA 23 least margin +0.873 N m at 7.976 s, x +1.163, y +1.163; lift NEMA 17 +0.0499 N m at 7.792 s, its one step lost at 0.065 s in the start-up transient and its -0.123 N m at 0.022 s on the before-span note (rest at 6.048 s); x and y NEMA 17 +0.337 and +0.339 N m; no motor loses a step within the span). Since brief M3-GV (sourced StepperOnline parts, start 1 s): nominal 23HS22-2804S margins lift +0.4385 N m at 7.927 s, x +0.7177, y +0.7184, no step lost; the 17HS19-2004S1 on x and y passes; on the lift it FAILS: pull-out at 7.43 s (margin -0.308 N m), 91.6 steps lost, `fault:encoder_mismatch`, not lifted, because the published curve dips to 0.30 N m at 60 rpm (read by eye) against about 0.33 N m of rated-load demand |

Windows derive from the gantry team's values of 2026-11-11 (0.05 m/s^2, 0.1 m/s, 0.02 m/s homing, 10 cm homing travel,
0.5 m workspace); each case header carries its derivation.

## Sourced components (brief M3-GV, owner decision 4)

Every value below is REPLACEABLE: a published datasheet's figure put in a gantry slot until the gantry team gives its
own; sources and URLs in `docs/gantry-team-inputs.md`.

Rails. Both rails are a 2040 aluminium T-slot extrusion (Alux 2040 V-slot: A = 285.30 mm^2, I = 45162 mm^4 about the
40 mm deep axis, 11680 mm^4 about the 20 mm one, 0.761 kg/m) on the library's `aluminium_6063_t5_extrusion` (E = 69
GPa, rho = 2690 kg/m^3; the datasheet's alloy is 6063 in temper T66, the library holds T5), 40 mm deep vertical, so
`second_moment_z` (the bending plane, ruling 65) is the deep axis's and `fibre_y` = 20 mm, `fibre_z` = 10 mm. rail_y's
box is 40 x 60 mm; the same 2040 is used (40 mm vertical as the box, one profile for the frame), the box stays the CAD
stand-in and the rails declare the extrusion's mass (rho A L = 0.38373 kg each) instead of the solid box's. Per rail,
L = 0.5 m, E I = 69e9 x 4.5162e-8 = 3116.18 N m^2, rho A = 0.767457 kg/m, calibration load P = 100 N at mid-span:

| Rail | Supports | Midspan deflection | First frequency |
|---|---|---|---|
| rail_x | clamped-clamped | P L^3 / (192 E I) = 0.0208923 mm | 4.7300^2 / (2 pi L^2) sqrt(E I / (rho A)) = 907.599 Hz |
| rail_y | pinned-roller | P L^3 / (48 E I) = 0.0835693 mm | pi / (2 L^2) sqrt(E I / (rho A)) = 400.372 Hz |

The built chains print 902.104 Hz (rail_x, 16 segments, clamped-clamped calibration layout owed, ruling 16) and
400.379 Hz (rail_y). Damping ratio 0.02 on both rails: the low end of 2 to 7 percent for an assembly with many bolted
joints (SimScale's table, which cites no origin; bolted steel at low amplitude 0.5 to 2 percent in the same table),
fixed in advance. At the 1 ms default step both first modes are unresolved (x = 5.67 and 2.52); G12 and G15 run at 100
microseconds (x = 0.567, 0.252).

G15's settled-deflection targets (ruling 99 (b), brief M3-GV2): the closed form for each rail's section and supports
at mid-travel under its own weight, its carried weight and the design load, the method of the G15 header, re-derived
whenever the section changes. E I = 69e9 x 4.5162e-8 = 3116.178 N m^2, w = rho A g = 2690 x 2.853e-4 x 9.81 = 7.528753
N/m (the section's, not a calibrated chain's), P = (5.2164 kg design load + 0.3 m x 0.011161 kg/m rope + 0.001 kg end
fitting) g = 51.21554 N, P_c = 1.68236 kg g = 16.50395 N (rail_y and all on it, the engine's carried mass), P_t =
0.917467 kg g = 9.000351 N (trolley, lift, hook); L = 0.5 m. rail_x (clamped-clamped): w L^4 / (384 E I) + (P_c + P)
L^3 / (192 E I) = 1.4541401e-5 m (was 1.0976722e-3 m on the placeholder section); rail_y (pinned-roller): 5 w L^4 /
(384 E I) + (P_t + P) L^3 / (48 E I) = 5.2288134e-5 m (was 7.384859e-4 m). The settled runs before the change read
1.45415e-5 and 5.22721e-5 m (+0.001 and -0.03 percent). Static moments (P_c + P) L / 8 + w L^2 / 12 = 4.389 N m and
(P_t + P) L / 4 + w L^2 / 8 = 7.762 N m, stresses M c / I with c = 0.02 m: 1.944 and 3.438 MPa.

Load cell chain. TAL220B 5 kg (1.0 mV/V rated output, 5 V excitation, ultimate overload 150 percent = 73.575 N)
through an HX711-class front end (gain 128 or 64, 10 or 80 SPS; RATE high, 80 SPS, is chosen for the overload
detection's latency). The cell gives 1.0e-3 x 5 V / 5 kg = 1e-3 V/kg; the firmware's 600 counts per kg on the
placeholder chip's 12-bit, 3.3 V ADC needs a gain of 600 x 3.3 / 4096 / 1e-3 = 483.398 (5.5 kg reads 3300 counts, full
scale 4096 / 600 = 6.83 kg), zero 0 counts. The HX711's own 24-bit digital path (where its gain of 128 would apply) is
not in the schema, a finding. G6's bound re-derived (ruling 35): the detection latency is one sample at 80 SPS (12.5
ms), the front end's first-order rise from the rated 4.536 kg reading to the 5.216 kg limit after the step to 5.5 kg
(tau = 1 / (2 pi 2 Hz) = 79.6 ms; the HX711 datasheet does not describe its digital filter, so 2 Hz stays ruling 35's
placeholder; its 50 ms settling at 80 SPS is from power-up, reset or a channel or gain change, not a step response),
t = tau ln(0.964 / 0.284) = 97.3 ms, and GANTRY_OVERLOAD_TICKS 2 ms: 111.8 ms, 0.1 m/s x 0.1118 s = 1.12 cm, so the
bound stays 0.1 + 0.0112 + 0.1 m = 21.1 cm; `G6.yaml` is unchanged.

## What runs where (INV-020, brief M3-X2d)

The acceptance checks of G2 to G16 read whole native runs of 30 to 130 s each and carry the `long_run` marker: CI
runs them, the G16 sweep and the G1 to G16 batch (ACC-036) in the Linux `python-long` job only. The main Linux and
Windows jobs run G1 (ACC-001, 002), G3 (run-twice determinism) and G8 (FW-025) end to end on native, G7 on the
bridge (BLD-039) and the pendulum. A case several test modules read runs once per session (`tests/conftest.py`).

## Native against the bridge (PHY-024, brief M3-X2d)

Each criterion is compared at its own axis's tolerance (`docs/spec/phy.md`, the differential tolerance row): one full
step, 0.2 mm, plus that axis's belt stretch at rest in that case (lift: (payload + the build's driven mass, 0.4064 kg) g on 1e5 N/m, 0.240 mm
empty, 0.436 mm with 2 kg, 0.779 mm with 5.5 kg; x and y carry no load at rest, 0.2 mm); event times 3 ms; counts
exact. Four misses of record, all G10, with their causes from the archives of both adapters (`tests/acceptance/
test_phy_stubs.py`, `MISSES_OF_RECORD`; a companion test asserts the list exactly; brief M3-X2h, after the native
baseline's regeneration with E8). G7 and G9 were misses until then and are kept in the table, explained and inside
their tolerance. Two findings for the owner at the gate: G10's remainder, and the efficiency-damping release artefact
below (ruling 94, E9: a milestone 4 engine item unless the owner wants it before retirement):

| Miss | Bridge | Native | Cause | Verdict |
|---|---|---|---|---|
| G7 `hook_held`, G9 `load_held` (no longer a miss) | 2.177 mm at 12.050 s | 1.821 mm at 12.050 s (baseline after E8; 1.146 mm at 12.044 s before E8) | Drive cleared from 12.022 s (SIG-033: K = D = 0), brake (0.5 N m) from 12.042 s. Bridge: rigid drive, payload welded, fall at 2.4074 g / 3.098 kg = 7.62 m/s^2 for 20 ms (1.524 mm), brake stop 0.656 mm: 2.180 mm. Native, two engine mechanisms: (a) the belt's efficiency loss is a shaft damping c = alpha L / max(omega, omega_s) frozen at the step's start rate (ruling 47, M3-E6): released from rest it holds the rotor about 2 ms (recorded 0.002, 0.038, 0.655, 3.86 rad/s against about 5 rad/s after one step at 0.95); (b) the rope goes slack at 12.030 s and, before E8, did not take up again in the window, so the brake stopped the rotor and the 0.4074 kg carriage alone. A three-mass backward-Euler model of these terms gives 1.140 mm (head rope: 1.811 mm) | Explained: 1.040 mm predicted against 1.031 mm before E8; no rotor damping (ruled out). After E8 the difference is 0.357 mm (model 0.369), inside 0.436 mm: not a miss. Mechanism (a) is a finding: on a release from rest the frozen-rate damping removes far more than the 5 percent loss it stands for in the first steps (reproduce: G7 on native, `lift_motor_spin` from 12.022 to 12.025 s; E9) |
| G10 `stall_fault`, `slip_fault` | 9.161 s, 31.427 s | 9.153 s, 31.407 s | The x belt's stretch under the drive torque at the fault, 0.0496 mm, puts the mismatch ahead: crossing 1.7 ms (stall, 28.2 mm/s) and 4.7 ms (slip, 10.5 mm/s) earlier, the event (detection + v / a) twice that: 3.4 ms and 9.4 ms | Finding: against 8 ms and 20 ms observed; the recomputed crossings move 2 and 5 ms as predicted; the stop phase (detection to the fault at zero commanded velocity) and the carriage's coast during the stall are not yet accounted; M3-X2g reproduced both adapters' event times (native 9.153 and 31.407 s, bridge 9.161 and 31.427 s) but did not rebuild the firmware's mismatch from the archives: still a finding, closed by a follow-up only if the owner asks |
| G10 `stall_rehomed` | 12.044 s | 12.030 s | Follows the stall stop (8 ms) and a homing 2 ms shorter (G2 `ready`) | Consequence of the finding |
| G10 `slip_carriage_stopped` | 21.16 mm | 20.43 mm | The stop starts 10 ms earlier; the commanded stop distance moves 2 v per second of detection, 2 x 0.036 m/s x 10 ms = 0.72 mm | Consequence of the finding |

Native baseline regenerated whole after brief M3-GV (the sourced parts and the 1 s start; one
`mechsim suite test_cases/gantry --write-baseline` run, 2026-11-15): 99 entries, none added or removed; 5 statuses
changed, all G15: `safety_factor_rail_x` fail -> pass (1.43609 -> 37.9641), `safety_factor_rail_y` and
`deflection_rail_y` error -> pass (24.0354; 6.915e-5 m; rail_y no longer flagged, dynamic ratio 1.131),
`settled_deflection_rail_x` pass -> fail (1.08056e-3 -> 1.45415e-5 m) and `settled_deflection_rail_y` error -> fail
(5.22721e-5 m): the stiffer, lighter aluminium rails; the two settled targets are the placeholder section's closed form
(not this brief's to change). 46 more entries changed value with their status kept; 48 changed only in run hashes.

| Case | Entries moved (before -> after) | Cause |
|---|---|---|
| G15 | `deflection_rail_x` 1.12699 -> 0.0163851 mm | 2040 rails: E I 3116 N m^2 against the placeholder's calibrated 64 N m^2 |
| G16 | margins lift 0.8730 -> 0.4385, x 1.1627 -> 0.7177, y 1.1628 -> 0.7184 N m; missed steps within 7e-4 of a step | the 23HS22-2804S's published curve (0.76 N m at 150 rpm against the placeholder's 1.205) |
| G12 | `estop_swing_x` 0.045081 -> 0.045452 rad, `move_swing_x` 0.013085 -> 0.012893, `move_swing_y` 0.010467 -> 0.010393, `move_settle_x` 5.207 -> 5.202 s (fail kept) | lighter stainless rope (EA 107 kN, 11.2 g/m), stiffer belts, new motors |
| G7, G9 | `hook_held` 1.8207 -> 1.8473 mm, `load_held` 1.8206 -> 1.8465 mm; `resumed_x`, `_y` under 9 um | belt 124 kN/m and efficiency 0.98 in the drop before the brake; rope |
| G5, G6 | `lowered` 99.605 -> 99.692 mm; `lift_stopped` 208.84 -> 209.08 mm (bound 211 mm), `set_down` 49.953 -> 49.988 mm; `lift_up_ignored` 0.22 -> 0.088 um | belt stiffness and efficiency; the load chain reads the same 600 counts/kg |
| G10 | `stall_fault` 9.153 -> 9.147 s, `stall_rehomed` 12.030 -> 12.022 s, `slip_carriage_stopped` 20.429 -> 20.434 mm | belt 124 kN/m: less stretch under the drive torque, the mismatch crosses earlier |
| G2, G3, G12 to G16 | `ready` 5.245 -> 5.246 s (G2, G3), 5.218 to 5.222 -> 5.224 to 5.226 s elsewhere; final positions within 0.03 um | homing on the new motors and belts |
| G1, G4, G13 | hold bands 52.0 -> 40.1 um (G1), G4 and G13 at 1e-13 m and below | start-up settling on the stiffer drive |

`baseline.yaml`: placeholder-geometry baselines for G1 to G16 on the native adapter (statuses of record on the draft
gantry), until CAD. `bridge_baselines/<case>.yaml`: the bridge's values of record for G1 to G11, G13 and G14,
generated once on 2026-11-14 and never regenerated (PHY-025; the native baseline is compared with them, PHY-024).

Native baseline regenerated whole after E8 (brief M3-X2h; `mechsim suite test_cases/gantry --write-baseline`, the
tree holding E8, the rope taut within the step): 99 entries, no status changed, no entry added or removed. 15
entries changed value, every one in a case whose payload hangs on the rope (G5, G6, G7, G9, G12, G15), cause E8's
taut-set rule; 84 changed only in their run hashes (archive and case hashes; G16's case hash from its header
comment). G16's values, regenerated after E8 by M3-X2f, did not move.

| Entry | Before (pre-E8) | After | Cause |
|---|---|---|---|
| G7 `hook_held` | 1.14624 mm at 12.044 s | 1.82075 mm at 12.050 s | E8: the rope takes up again after the slack at 12.030 s, so the brake stops the payload too (M3-X2g model 1.811 mm) |
| G9 `load_held` | 1.14618 mm at 12.024 s | 1.82062 mm at 12.030 s | E8, as G7 |
| G7 `resumed_x`, `resumed_y` | 400.01833, 250.01060 mm | 400.01961, 250.01478 mm | E8: the hanging payload's swing after the hold, under 5 um |
| G5 `lift_up_ignored` | 0.21761 um | 0.22108 um | E8 |
| G5 `lowered` | 99.60571 mm | 99.60519 mm | E8, under 1 um |
| G6 `lift_stopped`, `set_down` | 208.84585, 49.95560 mm | 208.84338, 49.95313 mm | E8, under 3 um |
| G12 `estop_swing_x` | 0.0449999 rad | 0.0450813 rad | E8: the rope's swing |
| G12 `move_swing_x`, `move_swing_y` | 0.0131140, 0.0105620 rad | 0.0130845, 0.0104670 rad | E8; `move_swing_x` worst 1 ms earlier |
| G12 `move_settle_x` | 5.208 s | 5.207 s | E8, one tick |
| G15 `deflection_rail_x` | 1.13384 mm at 16.548 s | 1.12699 mm at 16.550 s | E8: the hanging design load on the rail |
| G15 `settled_deflection_rail_x` | 1.080564 mm | 1.080562 mm | E8 |
| G15 `safety_factor_rail_x` | 1.42435 (fail) | 1.43609 (fail) | E8; the status stays fail (G12 limits not set, ADR 0009) |
