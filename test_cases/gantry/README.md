Gantry test suite: G1 to G16. Milestone 2 ships G1 to G11, G13 and G14; `suite.yaml` runs all of them; every open parameter has a placeholder default (DES-044)
until the gantry team rules.

| Case | Requirements doc test | Criteria (spec rows) |
|---|---|---|
| `G1.yaml` | Target (40, 25) at power on, before homing | no actuator motion (ACC-001); refusal reported (ACC-002) |
| `G2.yaml` | Home | each origin within 1 cm of the true origin (ACC-003); status reported (ACC-004) |
| `G3.yaml` | Target (40, 25) after homing | hook within 1 cm of the target (ACC-005); no travel limit exceeded (ACC-006) |
| `G4.yaml` | Target (75, 25) outside the 0.5 m workspace after homing | no actuator motion (ACC-007); rejected with a reported fault (ACC-008) |
| `G5.yaml` | Overweight load attached before a move | lift up refused, `refused` event, lift held (ACC-009); lowered within 1 cm (ACC-010); loaded in steps of at most 1 kg, finer near the top, to 5.5 kg (over the 5.216 kg limit, rated 10 lb + 15 %); the placeholder NEMA 17 stalls lowering 5.5 kg (open) |
| `G6.yaml` | Load becomes overweight mid-lift | rated 4.536 kg lifted, 5.5 kg at command + 2 s; lift stops under the derived 20.5 cm bound (ACC-011); set down within 1 cm (ACC-012); the placeholder NEMA 17 cannot hold the step (open) |
| `G7.yaml` | E-stop mid-move with a load | hook lifted to 5 cm first (under the 10 cm homing travel) so the brake holds it; `rail:motor_power` and relay status 0 from press + 20 ms, recorded 12.021 s (ACC-013); lift held (ACC-014); no steps from the next tick until the new command, `reset`, `home`, target reached after (ACC-015) |
| `G8.yaml` | Controller freeze mid-move | motor rail off and no step pulses from freeze + 70 ms (ACC-016) |
| `G9.yaml` | Power loss and restore mid-move with a load | hook lifted to 5 cm first; not_homed after the restore, target refused, no steps until `home` (ACC-017); lift held through the loss (ACC-018) |
| `G10.yaml` | Motor stall, then belt slip, mid-move | fault, no steps after the stop bound (ACC-019), `fault:encoder_mismatch` (ACC-020, ruling 30) from the frame-side linear encoders (ruling 32), target refused, re-home (ACC-021); carriage stopped within the slip bound |
| `G11.yaml` | Limit switch forced mid-workspace | stop (ACC-022), `fault:limit_trip` (ACC-023, ruling 30), target refused, re-home (ACC-024) |
| `G13.yaml` | Lift command while moving | `lift 0.3` mid-move refused with `fault:busy` and a `refused` event, no lift steps, lift held, move arrives (ACC-027; reject policy is the firmware placeholder) |
| `G14.yaml` | Fifty rapid-fire and conflicting commands | `no_overlap` on each driver's `motion_start`/`motion_end` (ACC-028, SCO-025); first move's interval within its cruise and slow-zone bounds (SCO-021); malformed refused |

Windows derive from the gantry team's values of 2026-11-11 (0.05 m/s^2, 0.1 m/s, 0.02 m/s homing, 10 cm homing travel,
0.5 m workspace); each case header carries its derivation.

`baseline.yaml`: placeholder-geometry baselines for G1 to G11, G13 and G14 (statuses of record on the draft gantry), until CAD.
