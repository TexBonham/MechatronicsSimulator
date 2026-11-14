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
| `G16.yaml` | Motor sizing: NEMA 17 against NEMA 23 at the rated load | sweeps of `instance:<motor>.part` (LIB-017), one motor substituted per run, the nominal run all NEMA 23; ready, no `fault:encoder_mismatch`, lifted and arrived within 1 cm; torque margin per motor computed and checked by the acceptance test from the recorded drive torque (ACC-031; on native 2026-11-14 every margin is positive: lift NEMA 17 +0.053 N m, NEMA 23 +0.877, x and y NEMA 17 +0.344, no lost step shown); the stepper's missed-step count is not recorded (ACC-032 open) |

Windows derive from the gantry team's values of 2026-11-11 (0.05 m/s^2, 0.1 m/s, 0.02 m/s homing, 10 cm homing travel,
0.5 m workspace); each case header carries its derivation.

`baseline.yaml`: placeholder-geometry baselines for G1 to G16 on the native adapter (statuses of record on the draft
gantry), until CAD. `bridge_baselines/<case>.yaml`: the bridge's values of record for G1 to G11, G13 and G14,
generated once on 2026-11-14 and never regenerated (PHY-025; the native baseline is compared with them, PHY-024).
