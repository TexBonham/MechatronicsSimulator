# Inputs owed by the gantry team

Every item is a placeholder or an open ruling the simulator ran with at the milestone 1 gate (2026-11-05); rows marked
"answered" carry the gantry team's value of 2026-11-11 (PR #5 document), applied in brief M2-X6. Each has a
slot in a data file; a value drops in by editing the file named, never code. Placeholders are flagged in place.

| Area | Item | Placeholder today | Where it lives |
|---|---|---|---|
| Geometry | Fusion 360 model: bodies, mass properties, joint placements, motor shaft axes | primitive solids | `designs/gantry/design.yaml` (`cad_model`) |
| Geometry | Workspace X and Y travel, lift travel | answered: 0.5 m, 0.5 m, 0.5 m (gantry team 2026-11-11) | design open parameters; `GANTRY_WORKSPACE_*`, `GANTRY_LIFT_TRAVEL_UM` |
| Geometry | Lift carriage hang below the trolley (lift travel fits under rail_x) | 0.65 m (0.3 m struck rail_x at 0.21 m of travel, M2-X3) | `designs/gantry/design.yaml` `lift_axis` placement |
| Mechanics | Joint friction on the X and Y axes | 5 N Coulomb each (M2-X3: without it an E-stopped axis coasts to its far limit) | `designs/gantry/design.yaml` `x_axis`, `y_axis` |
| Loads | Overweight payload in G5 and G6, and the loading steps | 5.5 kg (over the 5.216 kg limit); G5 loads 1, 2, 3, 4, 4.5, 5, 5.25, 5.5 kg while standing (undamped step bound on the 0.40 N m hold); G6 lifts the rated 4.536 kg and steps to 5.5 kg mid-lift. Both FAIL on the placeholder NEMA 17 (see Motors row) | `overweight_mass` in `test_cases/gantry/G5.yaml`, `G6.yaml` |
| Structure | Rail length; rail section area and second moment; rail end conditions | length answered: 0.5 m per rail (gantry team 2026-11-11); section 1.5e-4 m², 2e-8 m⁴ and row 17 end conditions still placeholders | design bodies `rail_x`, `rail_y` (geometry, `flexible.section.length`), open parameters |
| Structure | Rail calibration: known deflection and first natural frequency per rail | none | `rail_calibration` |
| Structure | Body materials and process | answered: PETG, 3D printed lengthwise, print direction along the rail (gantry team 2026-11-11); `petg_printed` mechanical properties remain placeholder datasheet values (wide spread) until the team gives them | `body_materials`; every design body; `mechsim/materials/data/petg_printed.yaml` |
| Structure | Deflection target static or dynamic (1 cm) | static, dynamic reported | `deflection_target_mode` |
| Loads | Rated load and overload limit (design-load rule: rated + 15 %) | answered: 10 lb = 4.536 kg; overload 5.216 kg (gantry team 2026-11-11) | `rated_load`; `GANTRY_OVERLOAD_LIMIT_MILLIKG` 5216; fw.md FW-018, FW-019 |
| Loads | Minimum safety factor, and whether it lives in the design or the case | answered: 1.5, in the design (gantry team 2026-11-11) | `minimum_safety_factor`, ADR 0009 |
| Loads | Swing angle limit and settle time (G12) | 0.1 rad, 5 s | `swing_limit_angle`, `swing_settle_time` |
| Loads | Power-loss hold: brake or self-locking screw | answered: brake (gantry team 2026-11-11; round 4 ruling 1 confirmed): `lift_brake`, brake_24v_generic on the motor rail on the lift motor shaft | `power_loss_hold`; `designs/gantry/design.yaml` `lift_brake` |
| Controller | New-target-during-move policy; lift-while-moving policy | new target: answered, operator choice, reject or queue, default reject (gantry team 2026-11-11); lift-while-moving: reject, still open | `new_target_policy`, `lift_while_moving_policy`; fw.md FW-026, FW-027 |
| Controller | Acceleration and deceleration limits, cruise and homing speeds | answered: 0.05 m/s², 0.1 m/s, 0.02 m/s (gantry team 2026-11-11) | fw.md FW-022; `firmware/app/gantry/gantry_config.h` |
| Controller | Slow-down zones near targets and travel ends | answered: 50 mm at 0.05 m/s (gantry team 2026-11-11) | fw.md FW-023 |
| Controller | Encoder mismatch threshold; homing maximum travel | threshold not given, 2 mm placeholder; homing travel answered: 10 cm (gantry team 2026-11-11); a 0.1 m start still homes (G2), so the lift is raised only 5 cm in G7 and G9 to keep the re-home inside 10 cm | fw.md FW-020, FW-014 |
| Controller | Heartbeat period and watchdog timeout; position status period | heartbeat and timeout not given: 10 ms, 50 ms placeholders; status period answered: 100 ticks (gantry team 2026-11-11) | fw.md FW-025, FW-029; catalog watchdog |
| Controller | Limit switch active level; driver direction and enable levels | high; high, high | fw.md open parameters |
| Controller | Lift commands (milestone 1 firmware never moves the lift) | held enabled, no steps | fw.md, milestone 2 |
| Sensors | Encoder placement (motor shaft or axis side); counts per metre; steps per metre | placement answered: axis side, frame (gantry team 2026-11-11, ruling 32 confirmed); 60 000 counts/m and 80 000 steps/m per axis stay placeholders (lift on a 40 mm/rev belt now) | `encoder_placement`; fw.md |
| Motors | Stepper datasheet: holding torque, torque-speed curve, rotor inertia, viscous damping, rated current, max voltage, phase R and L | catalog placeholders | `mechsim/library/catalog/stepper_behavioral.yaml` |
| Transmissions | Lead screw: none, every axis is belt-driven, the lift included (gantry team 2026-11-11; `lift_belt` replaces `lift_screw`, ruling 26 moot for the gantry); belt efficiency, stiffness, max tension not given | catalog placeholders | `belt_pulley.yaml` |
| Components | Limit switch, driver, supply, relay, watchdog, UART link ratings | catalog placeholders | each catalog file's `placeholders` list |
| Components | Load cell type, if a weight sensor exists | none declared by the team (gantry team 2026-11-11); load_cell_5kg_generic kept as a placeholder: G5 and G6 need one or another overload detection. OWNER QUESTION: which overload detection the gantry uses | `load_cell_type`; `designs/gantry/design.yaml` `load_cell` |
| Components | Load cell amplifier part and gain; load scale and zero in counts | load_cell_amplifier_generic at gain 241.8 (600 counts per kg on a 12-bit 3.3 V ADC, full scale 6.8 kg so the 5.5 kg payload reads; 1000 counts/kg saturated at 4.1 kg, under the new limit); 600 counts/kg, zero 0 | `load_amp` overrides; `gantry_config.h` `GANTRY_LOAD_COUNTS_PER_KG`, `GANTRY_LOAD_ZERO_COUNTS` |
| Controller | Heartbeat period, watchdog timeout, encoder mismatch threshold (round 4 ruling 6) | 10 ms, 50 ms, 2 mm placeholders (not given, gantry team 2026-11-11); overload limit answered above | `gantry_config.h`; catalog watchdog |
| Controller | Lift wait after the brake release output goes high (brief M2-X2) | 50 ms | `gantry_config.h` `GANTRY_BRAKE_RELEASE_WAIT_MS` |
| Electronics | MCU pins of the load ADC, E-stop status input and brake release output | PA11 adc_input, PB7 digital_input, PB8 digital_output | `chips/stm32_placeholder/chip.yaml`; design `pin_bindings` |
| Motors | Lift motor sizing (round 4 ruling 31, re-checked gantry team 2026-11-11 on the 40 mm/rev lift belt) | the placeholder NEMA 17 (0.40 N m hold, c = 6.37 mm, 6.4 kg) cannot lower 5.5 kg (stalls 1 s into the lowering, encoder_mismatch, G5) nor hold a step to 5.5 kg mid-lift (G6); no starting mass under the limit meets the undamped step bound | `designs/gantry/design.yaml` lift motor; `G5.yaml`, `G6.yaml` |
| Mechanics | Axis friction (round 4 ruling 31): keeps a coasting carriage inside G7's homing window | 5 N per axis placeholder | `axis_friction` open parameter; `G7.yaml` `homing_time` |
| Sensors | Encoder counts per metre on the frame side (round 4 ruling 32: the encoders sense x_axis, y_axis, lift_axis so belt slip and a stall both show as a step-count versus encoder mismatch) | 60 000 counts/m placeholder (`GANTRY_COUNTS_PER_M_*`), read by frame-side linear encoders (`linear_encoder_generic`, 1 / 60000 m per quadrature count) on the axis joints since brief M2-X5; the real scale and read head are owed | `designs/gantry/design.yaml` `x_encoder`, `y_encoder`, `lift_encoder` `senses.joint`; `gantry_config.h` |
| Cases | G5 to G8 payload masses, fault times, hold band and stop margins | see each case's `open_parameters` | `test_cases/gantry/G5.yaml` to `G8.yaml` |
| Electronics | STM32 part and target flags, after the peripheral smoke test | part answered: NUCLEO-G474RE, STM32G474RE (gantry team 2026-11-11); `chips/stm32_placeholder` stands in until the milestone 4 smoke test; flags open | `stm32_part`; `chips/README.md`, ADR 0011 |
| Electronics | Logic supply arrangement; ambient temperature; contact parameters | placeholders accepted (gantry team 2026-11-11): logic_psu; 298.15 K; 1e5 N/m, 1e3 N s/m, 0.5 | design open parameters |
| Acceptance | No-motion hold band (G1, G4) | 1 mm | `test_cases/gantry/G1.yaml`, `G4.yaml`; acc.md |
| Acceptance | Wording of "not homed" and "ready" on the operator link | codes exist, words to confirm | acc.md ACC-002, ACC-004; design `firmware.state_codes` |
| Acceptance | Travel-limit margin (G3); homing time budget (G2) | not set; case default | acc.md ACC-006; `G2.yaml` `homing_time` |

The G1 to G4 runs at the gate used the placeholders above, so values can be replaced one at a time and re-run with
`uv run mechsim suite test_cases/gantry --design designs/gantry/design.yaml`.
