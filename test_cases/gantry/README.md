Gantry test suite: G1 to G16. Milestone 2 ships G1 to G11; `suite.yaml` runs G1 to G9 and G11 (G10 waits for the bridge slip fix); every open parameter has a placeholder default (DES-044)
until the gantry team rules.

| Case | Requirements doc test | Criteria (spec rows) |
|---|---|---|
| `G1.yaml` | Target (40, 25) at power on, before homing | no actuator motion (ACC-001); refusal reported (ACC-002) |
| `G2.yaml` | Home | each origin within 1 cm of the true origin (ACC-003); status reported (ACC-004) |
| `G3.yaml` | Target (40, 25) after homing | hook within 1 cm of the target (ACC-005); no travel limit exceeded (ACC-006) |
| `G4.yaml` | Target outside the workspace after homing | no actuator motion (ACC-007); rejected with a reported fault (ACC-008) |
| `G5.yaml` | Overweight load attached before a move | lift up refused, `refused` event, lift held (ACC-009); lowered within 1 cm (ACC-010); loaded in steps to 3.6 kg (stepper margin on the 40 mm placeholder lead) |
| `G6.yaml` | Load becomes overweight mid-lift | lift stops under the derived 25 cm bound (ACC-011); set down within 1 cm (ACC-012) |
| `G7.yaml` | E-stop mid-move with a load | `rail:motor_power` and relay status 0 from press + 20 ms, recorded 9.021 s (ACC-013); lift held (ACC-014); no steps from the next tick until the new command, `reset`, `home`, target reached after (ACC-015) |
| `G8.yaml` | Controller freeze mid-move | motor rail off and no step pulses from freeze + 70 ms (ACC-016) |
| `G9.yaml` | Power loss and restore mid-move with a load | not_homed after the restore, target refused, no steps until `home` (ACC-017); lift held through the loss (ACC-018) |
| `G10.yaml` | Motor stall, then belt slip, mid-move | fault, no steps after the stop bound (ACC-019), encoder_mismatch (ACC-020), target refused, re-home (ACC-021); `slip_carriage_stopped` fails: the bridge drive runs away under slip (open, M2-X3) |
| `G11.yaml` | Limit switch forced mid-workspace | stop (ACC-022), limit_trip fault (ACC-023), target refused, re-home (ACC-024) |

`baseline.yaml`: placeholder-geometry baselines for G1 to G9 and G11 (statuses of record on the draft gantry), until CAD.
