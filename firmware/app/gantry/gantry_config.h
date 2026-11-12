/* Gantry controller parameters. Every value marked PLACEHOLDER belongs to an open parameter of the gantry design
 * (docs/spec/fw.md, Open parameters; designs/gantry/design.yaml open_parameters) and is overridable at build time;
 * values marked "gantry team 2026-11-11" are the team's answers (docs/gantry-team-inputs.md).
 * Lengths in micrometres, speeds in micrometres per second, accelerations in micrometres per second squared.
 */
#ifndef GANTRY_CONFIG_H
#define GANTRY_CONFIG_H

#include "mechsim_board.h"
#include "channels.h"

/* Gantry team 2026-11-11: workspace_x, workspace_y 0.5 m each (FW-016). Targets in [0, limit] inclusive are accepted. */
#ifndef GANTRY_WORKSPACE_X_UM
#define GANTRY_WORKSPACE_X_UM 500000
#endif
#ifndef GANTRY_WORKSPACE_Y_UM
#define GANTRY_WORKSPACE_Y_UM 500000
#endif

/* PLACEHOLDER: per-axis step scale, 200 full steps at 16 microsteps on every axis, every axis belt-driven (gantry team
 * 2026-11-11, no lead screw): GT2 20-tooth pulley, 40 mm/rev, so 3200 / 0.04 m = 80000 steps/m on X, Y and the lift.
 * The step angle and pulley stay placeholders. */
#ifndef GANTRY_STEPS_PER_M_X
#define GANTRY_STEPS_PER_M_X 80000
#endif
#ifndef GANTRY_STEPS_PER_M_Y
#define GANTRY_STEPS_PER_M_Y 80000
#endif
#ifndef GANTRY_STEPS_PER_M_LIFT
#define GANTRY_STEPS_PER_M_LIFT 80000
#endif
/* PLACEHOLDER: counts per metre of the frame-side linear encoders (encoder_placement axis side, gantry team
 * 2026-11-11, ruling 32). */
#ifndef GANTRY_COUNTS_PER_M_X
#define GANTRY_COUNTS_PER_M_X 60000
#endif
#ifndef GANTRY_COUNTS_PER_M_Y
#define GANTRY_COUNTS_PER_M_Y 60000
#endif
#ifndef GANTRY_COUNTS_PER_M_LIFT
#define GANTRY_COUNTS_PER_M_LIFT 60000
#endif

/* PLACEHOLDER: after the brake release output goes high the lift issues no steps for this long, so it never steps
 * against a brake still engaged (the catalog brake's release delay 30 ms, plus margin) (FW-018, FW-024). */
#ifndef GANTRY_BRAKE_RELEASE_WAIT_MS
#define GANTRY_BRAKE_RELEASE_WAIT_MS 50
#endif

/* Gantry team 2026-11-11: acceleration and deceleration limit 0.05 m/s^2 (FW-022), cruise speed 0.1 m/s. */
#ifndef GANTRY_MAX_ACCEL_UM_S2
#define GANTRY_MAX_ACCEL_UM_S2 50000
#endif
#ifndef GANTRY_MAX_SPEED_UM_S
#define GANTRY_MAX_SPEED_UM_S 100000
#endif
/* Gantry team 2026-11-11: slow-down zone at each travel end, 50 mm, and its speed cap 0.05 m/s (FW-023). */
#ifndef GANTRY_SLOW_ZONE_UM
#define GANTRY_SLOW_ZONE_UM 50000
#endif
#ifndef GANTRY_SLOW_SPEED_UM_S
#define GANTRY_SLOW_SPEED_UM_S 50000
#endif
/* Gantry team 2026-11-11: homing approach speed 0.02 m/s, and the travel after which a missing switch faults the
 * homing, 10 cm (FW-014). */
#ifndef GANTRY_HOMING_SPEED_UM_S
#define GANTRY_HOMING_SPEED_UM_S 20000
#endif
#ifndef GANTRY_HOMING_MAX_TRAVEL_UM
#define GANTRY_HOMING_MAX_TRAVEL_UM 100000
#endif

/* PLACEHOLDER: electrical levels of the switches and drivers. */
#ifndef GANTRY_SWITCH_ACTIVE_LEVEL
#define GANTRY_SWITCH_ACTIVE_LEVEL MECHSIM_LEVEL_HIGH
#endif
#ifndef GANTRY_DIR_POSITIVE_LEVEL
#define GANTRY_DIR_POSITIVE_LEVEL MECHSIM_LEVEL_HIGH
#endif
#ifndef GANTRY_DIR_NEGATIVE_LEVEL
#define GANTRY_DIR_NEGATIVE_LEVEL MECHSIM_LEVEL_LOW
#endif
#ifndef GANTRY_ENABLE_ACTIVE_LEVEL
#define GANTRY_ENABLE_ACTIVE_LEVEL MECHSIM_LEVEL_HIGH
#endif
#ifndef GANTRY_ENABLE_INACTIVE_LEVEL
#define GANTRY_ENABLE_INACTIVE_LEVEL MECHSIM_LEVEL_LOW
#endif

/* Gantry team 2026-11-11: ticks between `position x y` status lines in READY and MOVING (FW-029). */
#ifndef GANTRY_STATUS_PERIOD_TICKS
#define GANTRY_STATUS_PERIOD_TICKS 100u
#endif

/* Operator choice (gantry team 2026-11-11): new target while MOVING (FW-026), reject or hold one queued target;
 * default reject. */
#define GANTRY_POLICY_REJECT 0
#define GANTRY_POLICY_QUEUE 1
#ifndef GANTRY_NEW_TARGET_POLICY
#define GANTRY_NEW_TARGET_POLICY GANTRY_POLICY_REJECT
#endif

/* PLACEHOLDER: lift while X or Y (or the lift) is MOVING (FW-027 owed by the gantry team), same shape as FW-026. It
 * follows the new-target policy unless set on its own; both share the one queue slot. */
#ifndef GANTRY_LIFT_WHILE_MOVING_POLICY
#define GANTRY_LIFT_WHILE_MOVING_POLICY GANTRY_NEW_TARGET_POLICY
#endif

/* Gantry team 2026-11-11: lift travel 0.5 m from the lift home (min) switch, positive up. */
#ifndef GANTRY_LIFT_TRAVEL_UM
#define GANTRY_LIFT_TRAVEL_UM 500000
#endif

/* Channels of the load ADC, the E-stop status input (1 = motor rail live) and the brake release output (1 = released),
 * each the generated MECHSIM_CH_<SIGNAL> of the design's binding. */
#ifndef GANTRY_CH_LOAD_ADC
#define GANTRY_CH_LOAD_ADC MECHSIM_CH_LOAD_AMP_OUT
#endif
#ifndef GANTRY_CH_ESTOP_STATUS_IN
#define GANTRY_CH_ESTOP_STATUS_IN MECHSIM_CH_ESTOP_STATUS
#endif
#ifndef GANTRY_CH_BRAKE_RELEASE_OUT
#define GANTRY_CH_BRAKE_RELEASE_OUT MECHSIM_CH_LIFT_BRAKE_RELEASE
#endif
/* The design already binds the watchdog heartbeat, so this aliases the generated channel. */
#ifndef GANTRY_CH_HEARTBEAT_OUT
#define GANTRY_CH_HEARTBEAT_OUT MECHSIM_CH_WATCHDOG_HEARTBEAT
#endif

/* PLACEHOLDER (ruling 6): load cell scale in integer ADC counts, 600 per kg so the 12-bit ADC spans 6.8 kg (no weight
 * sensor declared by the gantry team, 2026-11-11). Gantry team 2026-11-11: overload limit = rated 10 lb (4.536 kg)
 * + 15 percent = 5.216 kg (FW-018, FW-019). Over the limit means strictly above it; mid-lift it must hold for
 * GANTRY_OVERLOAD_TICKS ticks. */
#ifndef GANTRY_LOAD_COUNTS_PER_KG
#define GANTRY_LOAD_COUNTS_PER_KG 600
#endif
#ifndef GANTRY_LOAD_ZERO_COUNTS
#define GANTRY_LOAD_ZERO_COUNTS 0
#endif
#ifndef GANTRY_OVERLOAD_LIMIT_MILLIKG
#define GANTRY_OVERLOAD_LIMIT_MILLIKG 5216

#endif
#ifndef GANTRY_OVERLOAD_TICKS
#define GANTRY_OVERLOAD_TICKS 2u
#endif

/* PLACEHOLDER (ruling 6): heartbeat toggle period, 10 ms at 1 kHz (FW-025). */
#ifndef GANTRY_HEARTBEAT_PERIOD_TICKS
#define GANTRY_HEARTBEAT_PERIOD_TICKS 10u
#endif

/* PLACEHOLDER (ruling 6): encoder against commanded position, 2 mm (FW-020), in counts per axis at its scale; over it
 * faults. */
#ifndef GANTRY_ENCODER_MISMATCH_UM
#define GANTRY_ENCODER_MISMATCH_UM 2000
#endif

/* Buffer sizes: longest accepted command line, queued transmit bytes and queued events across ticks. */
#ifndef GANTRY_LINE_MAX
#define GANTRY_LINE_MAX 48u
#endif
#ifndef GANTRY_TX_QUEUE_BYTES
#define GANTRY_TX_QUEUE_BYTES 512u
#endif
#ifndef GANTRY_EVENT_QUEUE_COUNT
#define GANTRY_EVENT_QUEUE_COUNT 64u
#endif

/* State codes (state_code), fault codes (fault_code and `fault <code>` lines), event codes (event_log). */
#define GANTRY_STATE_NOT_HOMED 1u
#define GANTRY_STATE_HOMING 2u
#define GANTRY_STATE_READY 3u
#define GANTRY_STATE_MOVING 4u
#define GANTRY_STATE_FAULT 5u

#define GANTRY_FAULT_NONE 0u
#define GANTRY_FAULT_NOT_HOMED 1u
#define GANTRY_FAULT_OUTSIDE_WORKSPACE 2u
#define GANTRY_FAULT_BUSY 3u
#define GANTRY_FAULT_MALFORMED 4u
#define GANTRY_FAULT_QUEUE_FULL 5u
#define GANTRY_FAULT_IN_FAULT 6u
#define GANTRY_FAULT_HOMING_FAILED 7u
#define GANTRY_FAULT_EVENT_OVERFLOW 8u
#define GANTRY_FAULT_TX_OVERFLOW 9u
#define GANTRY_FAULT_OVERLOAD 10u         /* refusal: lift up over the load limit (FW-018, FW-019) */
#define GANTRY_FAULT_ESTOP 11u            /* latched until `reset` with the rail live; refusal while it is not */
#define GANTRY_FAULT_ENCODER_MISMATCH 12u /* latched until `home` (FW-020) */
#define GANTRY_FAULT_LIMIT_TRIP 13u       /* latched until `home` (FW-021) */
#define GANTRY_FAULT_NOTHING_TO_RESET 14u /* refusal: `reset` outside FAULT */

#define GANTRY_EVENT_STATE 1u   /* arg: (from << 8) | to */
#define GANTRY_EVENT_COMMAND 2u /* arg: GANTRY_COMMAND_* */
#define GANTRY_EVENT_REFUSED 3u /* arg: fault code of the refusal */
#define GANTRY_EVENT_FAULT 4u   /* arg: latched fault code */
#define GANTRY_EVENT_ORIGIN 5u  /* arg: GANTRY_AXIS_* whose origin was set */
#define GANTRY_EVENT_MOTION_START 6u /* arg: GANTRY_AXIS_* whose profile started (FW-028) */
#define GANTRY_EVENT_MOTION_END 7u   /* arg: GANTRY_AXIS_* whose profile reached zero velocity (FW-028) */

#define GANTRY_COMMAND_TARGET 1u
#define GANTRY_COMMAND_HOME 2u
#define GANTRY_COMMAND_UNKNOWN 3u
#define GANTRY_COMMAND_LIFT 4u
#define GANTRY_COMMAND_RESET 5u

#define GANTRY_AXIS_X 0u
#define GANTRY_AXIS_Y 1u
#define GANTRY_AXIS_LIFT 2u
#define GANTRY_AXIS_COUNT 3u

#endif
