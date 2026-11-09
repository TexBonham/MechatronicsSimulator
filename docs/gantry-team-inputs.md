# Inputs owed by the gantry team

Every item is a placeholder or an open ruling the simulator ran with at the milestone 1 gate (2026-11-05). Each has a
slot in a data file; a value drops in by editing the file named, never code. Placeholders are flagged in place.

| Area | Item | Placeholder today | Where it lives |
|---|---|---|---|
| Geometry | Fusion 360 model: bodies, mass properties, joint placements, motor shaft axes | primitive solids | `designs/gantry/design.yaml` (`cad_model`) |
| Geometry | Workspace X and Y travel, lift travel | 1 m, 0.8 m, 0.5 m | design open parameters |
| Structure | Rail extrusion section area and second moment; rail end conditions | 1.5e-4 m², 2e-8 m⁴; row 17 defaults | design open parameters, member declarations |
| Structure | Rail calibration: known deflection and first natural frequency per rail | none | `rail_calibration` |
| Structure | Body materials and process | aluminium 6063-T5 extrusion | `body_materials` |
| Structure | Deflection target static or dynamic (1 cm) | static, dynamic reported | `deflection_target_mode` |
| Loads | Rated load and overload limit (design-load rule: rated + 15 %) | 3 kg; overload not set | `rated_load`; fw.md FW-018, FW-019 |
| Loads | Minimum safety factor, and whether it lives in the design or the case | 2 | `minimum_safety_factor`, ADR 0009 |
| Loads | Swing angle limit and settle time (G12) | 0.1 rad, 5 s | `swing_limit_angle`, `swing_settle_time` |
| Loads | Power-loss hold: brake or self-locking screw | self-locking screw | `power_loss_hold` |
| Controller | New-target-during-move policy; lift-while-moving policy | reject; reject | `new_target_policy`, `lift_while_moving_policy`; fw.md FW-026, FW-027 |
| Controller | Acceleration and deceleration limits, cruise and homing speeds | 0.5 m/s², 0.2 m/s, 0.02 m/s | fw.md FW-022; `firmware/app/gantry/gantry_config.h` |
| Controller | Slow-down zones near targets and travel ends | 50 mm at 0.05 m/s | fw.md FW-023 |
| Controller | Encoder mismatch threshold; homing maximum travel | not set; 1.2 m | fw.md FW-020, FW-014 |
| Controller | Heartbeat period and watchdog timeout; position status period | not set; 100 ticks | fw.md FW-025, FW-029; catalog watchdog |
| Controller | Limit switch active level; driver direction and enable levels | high; high, high | fw.md open parameters |
| Controller | Lift commands (milestone 1 firmware never moves the lift) | held enabled, no steps | fw.md, milestone 2 |
| Sensors | Encoder placement (motor shaft or axis side); counts per metre; steps per metre | frame; 60 000; 80 000 | `encoder_placement`; fw.md |
| Motors | Stepper datasheet: holding torque, torque-speed curve, rotor inertia, viscous damping, rated current, max voltage, phase R and L | catalog placeholders | `mechsim/library/catalog/stepper_behavioral.yaml` |
| Transmissions | Lead screw lead, efficiency, stiffness, backlash, max axial load; belt efficiency, stiffness, max tension | catalog placeholders | `lead_screw.yaml`, `belt_pulley.yaml` |
| Components | Limit switch, driver, supply, relay, watchdog, UART link ratings | catalog placeholders | each catalog file's `placeholders` list |
| Components | Load cell type, if a weight sensor exists | none declared | `load_cell_type` |
| Electronics | STM32 part and target flags, after the peripheral smoke test | `chips/stm32_placeholder` | `chips/`, ADR 0011 |
| Electronics | Logic supply arrangement; ambient temperature; contact parameters | logic_psu; 298.15 K; 1e5 N/m, 1e3 N s/m, 0.5 | design open parameters |
| Acceptance | No-motion hold band (G1, G4) | 1 mm | `test_cases/gantry/G1.yaml`, `G4.yaml`; acc.md |
| Acceptance | Wording of "not homed" and "ready" on the operator link | codes exist, words to confirm | acc.md ACC-002, ACC-004; design `firmware.state_codes` |
| Acceptance | Travel-limit margin (G3); homing time budget (G2) | not set; case default | acc.md ACC-006; `G2.yaml` `homing_time` |

The G1 to G4 runs at the gate used the placeholders above, so values can be replaced one at a time and re-run with
`uv run mechsim suite test_cases/gantry --design designs/gantry/design.yaml`.
