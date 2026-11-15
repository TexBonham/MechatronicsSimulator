Gantry test suite: G1 to G16. Every case runs on the native adapter, the default since M3-X2 (round 5 ruling 1);
milestone 2 shipped G1 to G11, G13 and G14 on the bridge, whose values of record are the fixed baselines in
`bridge_baselines/` (PHY-025); `suite.yaml` runs all of them; every open parameter has a placeholder default
(DES-044) until the gantry team rules.

## Evaluation start (TST-044, brief M3-X2)

One start for every case, derived once and not adjusted to any result: a native run starts from straight rails and
the release rings for a few loaded time constants of the slowest rail, so integrity results (peaks, safety factors,
tier 1 crossings, the validity flag, rope snap) count from five of them, rounded up to a whole second. From the
figures every run prints: rail_x stated ratio zeta_0 = 0.01 at its unloaded first mode f_0 = 49.6973 Hz
(omega_0 = 312.26 rad/s), realised zeta_r = 0.0042905 on its loaded mode (the lowest any case prints). Stiffness-
proportional damping gives zeta = beta omega / 2 for every mode, so the loaded mode is omega_c = omega_0 zeta_r /
zeta_0 = 133.97 rad/s and its time constant tau = 1 / (zeta_r omega_c) = zeta_0 / (zeta_r^2 omega_0) = 1.740 s
(rail_y: zeta_r = 0.0072418, 0.607 s). Five tau = 8.70 s: `integrity.evaluate_from: 9 s` in G2 to G15. G1 ends at
2 s and cannot carry it (a start past the last sample is an error, TST-044); its run on native records no tier 1
crossing. What a case loses: integrity during its first 9 s is not judged (homing ends by 5.25 s; G12 and G15 move
from 8 s, so their first second of the 2 s acceleration ramp is not judged, the second second at the same
acceleration is). Motion criteria keep their own windows. Before it, G5 and G6 cross tier 1 on the release at 0.011 to
0.022 s (rail_x and rail_y), recorded as `integrity_before_evaluation` events. On native the same transient
carries the payload rope above its 5000 N breaking tension (6011 N at 0.012 s, 26229 N at its peak before 9 s); the
start holds the snap back. Without it G5's rope snaps at 0.012 s, the load cell reads empty (7 counts against 3308
at 5.5 kg), `lift 0.4` is accepted and the lift ends at 0.4001 m (M3-X2c; that was N5's first G5 result).

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
| `G15.yaml` | Maximum acceleration to mid-travel with the design load, then an E-stop with a swinging load (native, tier 1, no halt) | every member's safety factor at least `minimum_safety_factor` (ACC-029); node-pose deflection under 1 cm (ACC-030); no tier 1 crossing; load case exported |
| `G16.yaml` | Motor sizing: NEMA 17 against NEMA 23 at the rated load | sweeps of `instance:<motor>.part` (LIB-017), one motor substituted per run, the nominal run all NEMA 23; ready, no `fault:encoder_mismatch`, lifted and arrived within 1 cm; torque margin (`torque_margin:<motor>`, SCO-032) and missed steps (`missed_steps:<motor>`, SCO-031) per motor in the report, margin minimum above 0 and final count 0 within a quarter step, checked by ACC-031 and ACC-032 against their derivations (briefs M3-X2e, M3-X2f): both judged from the last rest at or before the 9 s evaluation start to a final rest (rulings 91, 93), what precedes it on a before-span note per motor; on native 2026-11-14 every run passes: nominal lift NEMA 23 least margin +0.873 N m at 7.976 s, x +1.163, y +1.163; lift NEMA 17 +0.0499 N m at 7.792 s, its one step lost at 0.065 s in the start-up transient and its -0.123 N m at 0.022 s on the before-span note (rest at 6.048 s); x and y NEMA 17 +0.337 and +0.339 N m; no motor loses a step within the span) |

Windows derive from the gantry team's values of 2026-11-11 (0.05 m/s^2, 0.1 m/s, 0.02 m/s homing, 10 cm homing travel,
0.5 m workspace); each case header carries its derivation.

## What runs where (INV-020, brief M3-X2d)

The acceptance checks of G2 to G16 read whole native runs of 30 to 130 s each and carry the `long_run` marker: CI
runs them, the G16 sweep and the G1 to G16 batch (ACC-036) in the Linux `python-long` job only. The main Linux and
Windows jobs run G1 (ACC-001, 002), G3 (run-twice determinism) and G8 (FW-025) end to end on native, G7 on the
bridge (BLD-039) and the pendulum. A case several test modules read runs once per session (`tests/conftest.py`).

## Native against the bridge (PHY-024, brief M3-X2d)

Each criterion is compared at its own axis's tolerance (`docs/spec/phy.md`, the differential tolerance row): one full
step, 0.2 mm, plus that axis's belt stretch at rest in that case (lift: (payload + the build's driven mass, 0.4064 kg) g on 1e5 N/m, 0.240 mm
empty, 0.436 mm with 2 kg, 0.779 mm with 5.5 kg; x and y carry no load at rest, 0.2 mm); event times 3 ms; counts
exact. Six misses of record, with their causes from the archives of both adapters (`tests/acceptance/
test_phy_stubs.py`, `MISSES_OF_RECORD`; a companion test asserts the list exactly):

| Miss | Bridge | Native | Cause | Verdict |
|---|---|---|---|---|
| G7 `hook_held`, G9 `load_held` | 2.177 mm at 12.050 s | 1.146 mm at 12.044 s (record, before E8); 1.821 mm at 12.050 s at the head | Drive cleared from 12.022 s (SIG-033: K = D = 0), brake (0.5 N m) from 12.042 s. Bridge: rigid drive, payload welded, fall at 2.4074 g / 3.098 kg = 7.62 m/s^2 for 20 ms (1.524 mm), brake stop 0.656 mm: 2.180 mm. Native, two engine mechanisms: (a) the belt's efficiency loss is a shaft damping c = alpha L / max(omega, omega_s) frozen at the step's start rate (ruling 47, M3-E6): released from rest it holds the rotor about 2 ms (recorded 0.002, 0.038, 0.655, 3.86 rad/s against about 5 rad/s after one step at 0.95); (b) the rope goes slack at 12.030 s and, before E8, did not take up again in the window, so the brake stopped the rotor and the 0.4074 kg carriage alone. A three-mass backward-Euler model of these terms gives 1.140 mm (head rope: 1.811 mm) | Explained: 1.040 mm predicted against 1.031 mm; no rotor damping (ruled out). At the head the difference is 0.357 mm, inside 0.436 mm: the native baseline predates E8 and is stale |
| G10 `stall_fault`, `slip_fault` | 9.161 s, 31.427 s | 9.153 s, 31.407 s | The x belt's stretch under the drive torque at the fault, 0.0496 mm, puts the mismatch ahead: crossing 1.7 ms (stall, 28.2 mm/s) and 4.7 ms (slip, 10.5 mm/s) earlier, the event (detection + v / a) twice that: 3.4 ms and 9.4 ms | Finding: against 8 ms and 20 ms observed; the recomputed crossings move 2 and 5 ms as predicted; the stop phase (detection to the fault at zero commanded velocity) and the carriage's coast during the stall are not yet accounted (M3-X2g item 2 not finished); the head reproduces both adapters' event times |
| G10 `stall_rehomed` | 12.044 s | 12.030 s | Follows the stall stop (8 ms) and a homing 2 ms shorter (G2 `ready`) | Consequence of the finding |
| G10 `slip_carriage_stopped` | 21.16 mm | 20.43 mm | The stop starts 10 ms earlier; the commanded stop distance moves 2 v per second of detection, 2 x 0.036 m/s x 10 ms = 0.72 mm | Consequence of the finding |

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
