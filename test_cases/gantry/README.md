Gantry test suite: G1 to G16. Milestone 1 ships G1 to G4; every open parameter has a placeholder default (DES-044)
until the gantry team rules.

| Case | Requirements doc test | Criteria (spec rows) |
|---|---|---|
| `G1.yaml` | Target (40, 25) at power on, before homing | no actuator motion (ACC-001); refusal reported (ACC-002) |
| `G2.yaml` | Home | each origin within 1 cm of the true origin (ACC-003); status reported (ACC-004) |
| `G3.yaml` | Target (40, 25) after homing | hook within 1 cm of the target (ACC-005); no travel limit exceeded (ACC-006) |
| `G4.yaml` | Target outside the workspace after homing | no actuator motion (ACC-007); rejected with a reported fault (ACC-008) |

`baseline.yaml`: placeholder baselines for G1 to G4, all not run, until CAD.
