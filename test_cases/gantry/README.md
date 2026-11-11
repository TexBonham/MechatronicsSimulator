Gantry test suite: G1 to G16. Milestone 1 ships G1 to G4; every open parameter has a placeholder default (DES-044)
until the gantry team rules.

| Case | Requirements doc test | Criteria (spec rows) |
|---|---|---|
| `G1.yaml` | Target (40, 25) at power on, before homing | no actuator motion (ACC-001); refusal reported (ACC-002) |
| `G2.yaml` | Home | each origin within 1 cm of the true origin (ACC-003); status reported (ACC-004) |
| `G3.yaml` | Target (40, 25) after homing | hook within 1 cm of the target (ACC-005); no travel limit exceeded (ACC-006) |
| `G4.yaml` | Target outside the workspace after homing | no actuator motion (ACC-007); rejected with a reported fault (ACC-008) |
| `G5.yaml` | Overweight load attached before a move | lift up refused, `refused` event, lift held (ACC-009); lowered within 1 cm (ACC-010). Not in the suite yet: errors on the bridge (M2-X2) |
| `G6.yaml` | Load becomes overweight mid-lift | lift stops under the derived 25 cm bound (ACC-011); set down within 1 cm (ACC-012). Not in the suite yet |
| `G7.yaml` | E-stop mid-move with a load | relay status 0 from press + 20 ms (ACC-013); lift held (ACC-014); no steps until the new command, target reached after (ACC-015). Not in the suite yet: needs a relay reset input and the brake |
| `G8.yaml` | Controller freeze mid-move | no motor motion from freeze + 70 ms (ACC-016). Not in the suite yet |

`baseline.yaml`: placeholder baselines for G1 to G4, all not run, until CAD.
