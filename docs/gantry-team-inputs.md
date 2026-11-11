# Inputs owed by the gantry team

Every item is a placeholder or an open ruling the simulator ran with at the milestone 1 gate (2026-11-05). Each has a
slot in a data file; a value drops in by editing the file named, never code. Placeholders are flagged in place.

| Area | Item | Placeholder today | Where it lives |
|---|---|---|---|
| Geometry | Fusion 360 model: bodies, mass properties, joint placements, motor shaft axes | primitive solids | `designs/gantry/design.yaml` (`cad_model`) |
| Geometry | Workspace X and Y travel, lift travel | 1 m, 0.8 m, 0.5 m | design open parameters |
| Geometry | Lift carriage hang below the trolley (lift travel fits under rail_x) | 0.65 m (0.3 m struck rail_x at 0.21 m of travel, M2-X3) | `designs/gantry/design.yaml` `lift_axis` placement |
| Mechanics | Joint friction on the X and Y axes | 5 N Coulomb each (M2-X3: without it an E-stopped axis coasts to its far limit) | `designs/gantry/design.yaml` `x_axis`, `y_axis` |
| Loads | Overweight payload in G5 and G6, and the loading steps | 3.6 kg reached in steps of at most 1 kg (G5) or from 2.5 kg (G6): the 40 mm placeholder lead leaves the NEMA 17 placeholder too little margin for 4 kg (case headers) | `overweight_mass` in `test_cases/gantry/G5.yaml`, `G6.yaml` |
| Structure | Rail extrusion section area and second moment; rail end conditions | 1.5e-4 m², 2e-8 m⁴; row 17 defaults | design open parameters, member declarations |
| Structure | Rail calibration: known deflection and first natural frequency per rail | none | `rail_calibration` |
| Structure | Body materials and process | aluminium 6063-T5 extrusion | `body_materials` |
| Structure | Deflection target static or dynamic (1 cm) | static, dynamic reported | `deflection_target_mode` |
| Loads | Rated load and overload limit (design-load rule: rated + 15 %) | 3 kg; overload not set | `rated_load`; fw.md FW-018, FW-019 |
| Loads | Minimum safety factor, and whether it lives in the design or the case | 2 | `minimum_safety_factor`, ADR 0009 |
| Loads | Swing angle limit and settle time (G12) | 0.1 rad, 5 s | `swing_limit_angle`, `swing_settle_time` |
| Loads | Power-loss hold: brake or self-locking screw | brake: `lift_brake`, brake_24v_generic on the motor rail acting on the lift joint (round 4 ruling 1) | `power_loss_hold`; `designs/gantry/design.yaml` `lift_brake` |
| Controller | New-target-during-move policy; lift-while-moving policy | reject; reject | `new_target_policy`, `lift_while_moving_policy`; fw.md FW-026, FW-027 |
| Controller | Acceleration and deceleration limits, cruise and homing speeds | 0.5 m/s², 0.2 m/s, 0.02 m/s | fw.md FW-022; `firmware/app/gantry/gantry_config.h` |
| Controller | Slow-down zones near targets and travel ends | 50 mm at 0.05 m/s | fw.md FW-023 |
| Controller | Encoder mismatch threshold; homing maximum travel | not set; 1.2 m | fw.md FW-020, FW-014 |
| Controller | Heartbeat period and watchdog timeout; position status period | not set; 100 ticks | fw.md FW-025, FW-029; catalog watchdog |
| Controller | Limit switch active level; driver direction and enable levels | high; high, high | fw.md open parameters |
| Controller | Lift commands (milestone 1 firmware never moves the lift) | held enabled, no steps | fw.md, milestone 2 |
| Sensors | Encoder placement (motor shaft or axis side); counts per metre; steps per metre | frame; 60 000 per axis; 80 000 per axis (lift at a placeholder 40 mm/rev ratio; 300 000 and 400 000 at the TR8x8's 8 mm lead) | `encoder_placement`; fw.md |
| Motors | Stepper datasheet: holding torque, torque-speed curve, rotor inertia, viscous damping, rated current, max voltage, phase R and L | catalog placeholders | `mechsim/library/catalog/stepper_behavioral.yaml` |
| Transmissions | Lead screw lead, efficiency, stiffness, backlash, max axial load; belt efficiency, stiffness, max tension | catalog placeholders | `lead_screw.yaml`, `belt_pulley.yaml` |
| Components | Limit switch, driver, supply, relay, watchdog, UART link ratings | catalog placeholders | each catalog file's `placeholders` list |
| Components | Load cell type, if a weight sensor exists | load_cell_5kg_generic on the hook, reading the reaction at the case payload's fixed joint (ruling 24) | `load_cell_type`; `designs/gantry/design.yaml` `load_cell` |
| Components | Load cell amplifier part and gain; load scale and zero in counts | load_cell_amplifier_generic at gain 403 (1000 counts per kg on a 12-bit 3.3 V ADC); 1000 counts/kg, zero 0 | `load_amp` overrides; `gantry_config.h` `GANTRY_LOAD_COUNTS_PER_KG`, `GANTRY_LOAD_ZERO_COUNTS` |
| Controller | Overload limit, heartbeat period, watchdog timeout, encoder mismatch threshold (round 4 ruling 6) | 3.45 kg, 10 ms, 50 ms, 2 mm | `gantry_config.h`; catalog watchdog |
| Controller | Lift wait after the brake release output goes high (brief M2-X2) | 50 ms | `gantry_config.h` `GANTRY_BRAKE_RELEASE_WAIT_MS` |
| Electronics | MCU pins of the load ADC, E-stop status input and brake release output | PA11 adc_input, PB7 digital_input, PB8 digital_output | `chips/stm32_placeholder/chip.yaml`; design `pin_bindings` |
| Motors | Lift motor and lead sizing (round 4 ruling 31): the placeholder NEMA 17 on the 40 mm placeholder lead cannot lower a 4 kg overload or take a 1 to 4 kg step load | G5 and G6 use 3.6 kg reached in 1 kg steps | `designs/gantry/design.yaml` lift motor and `lift_screw`; `G5.yaml`, `G6.yaml` `overweight_mass` |
| Mechanics | Axis friction (round 4 ruling 31): keeps a coasting carriage inside G7's homing window | 5 N per axis placeholder | `axis_friction` open parameter; `G7.yaml` `homing_time` |
| Sensors | Encoder counts per metre on the frame side (round 4 ruling 32: the encoders sense x_axis, y_axis, lift_axis so belt slip and a stall both show as a step-count versus encoder mismatch) | 60 000 counts/m placeholder (`GANTRY_COUNTS_PER_M_*`), read by frame-side linear encoders (`linear_encoder_generic`, 1 / 60000 m per quadrature count) on the axis joints since brief M2-X5; the real scale and read head are owed | `designs/gantry/design.yaml` `x_encoder`, `y_encoder`, `lift_encoder` `senses.joint`; `gantry_config.h` |
| Cases | G5 to G8 payload masses, fault times, hold band and stop margins | see each case's `open_parameters` | `test_cases/gantry/G5.yaml` to `G8.yaml` |
| Electronics | STM32 part and target flags, after the peripheral smoke test | `chips/stm32_placeholder` | `chips/`, ADR 0011 |
| Electronics | Logic supply arrangement; ambient temperature; contact parameters | logic_psu; 298.15 K; 1e5 N/m, 1e3 N s/m, 0.5 | design open parameters |
| Acceptance | No-motion hold band (G1, G4) | 1 mm | `test_cases/gantry/G1.yaml`, `G4.yaml`; acc.md |
| Acceptance | Wording of "not homed" and "ready" on the operator link | codes exist, words to confirm | acc.md ACC-002, ACC-004; design `firmware.state_codes` |
| Acceptance | Travel-limit margin (G3); homing time budget (G2) | not set; case default | acc.md ACC-006; `G2.yaml` `homing_time` |

The G1 to G4 runs at the gate used the placeholders above, so values can be replaced one at a time and re-run with
`uv run mechsim suite test_cases/gantry --design designs/gantry/design.yaml`.
