/* Gantry controller parameters. Every value marked PLACEHOLDER belongs to an open parameter of the gantry design
 * (docs/spec/fw.md, Open parameters; designs/gantry/design.yaml open_parameters) and is overridable at build time.
 * Lengths in micrometres, speeds in micrometres per second, accelerations in micrometres per second squared.
 */
#ifndef GANTRY_CONFIG_H
#define GANTRY_CONFIG_H

#include "mechsim_board.h"
#include "channels.h"

/* PLACEHOLDER: workspace_x, workspace_y (FW-016). Targets in [0, limit] inclusive are accepted. */
#ifndef GANTRY_WORKSPACE_X_UM
#define GANTRY_WORKSPACE_X_UM 1000000
#endif
#ifndef GANTRY_WORKSPACE_Y_UM
#define GANTRY_WORKSPACE_Y_UM 800000
#endif

/* PLACEHOLDER: GT2 20-tooth pulley (40 mm/rev), 200 full steps at 16 microsteps. */
#ifndef GANTRY_STEPS_PER_M
#define GANTRY_STEPS_PER_M 80000
#endif
/* PLACEHOLDER: 600 ppr quadrature (2400 counts/rev) on the pulley shaft; encoder_placement is open. */
#ifndef GANTRY_COUNTS_PER_M
#define GANTRY_COUNTS_PER_M 60000
#endif

/* PLACEHOLDER: acceleration and deceleration limit (FW-022), cruise speed. */
#ifndef GANTRY_MAX_ACCEL_UM_S2
#define GANTRY_MAX_ACCEL_UM_S2 500000
#endif
#ifndef GANTRY_MAX_SPEED_UM_S
#define GANTRY_MAX_SPEED_UM_S 200000
#endif
/* PLACEHOLDER: slow-down zone at each travel end and its speed cap (FW-023). */
#ifndef GANTRY_SLOW_ZONE_UM
#define GANTRY_SLOW_ZONE_UM 50000
#endif
#ifndef GANTRY_SLOW_SPEED_UM_S
#define GANTRY_SLOW_SPEED_UM_S 50000
#endif
/* PLACEHOLDER: homing approach speed, and the travel after which a missing switch faults the homing. */
#ifndef GANTRY_HOMING_SPEED_UM_S
#define GANTRY_HOMING_SPEED_UM_S 20000
#endif
#ifndef GANTRY_HOMING_MAX_TRAVEL_UM
#define GANTRY_HOMING_MAX_TRAVEL_UM 1200000
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

/* PLACEHOLDER: ticks between `position x y` status lines in READY and MOVING. */
#ifndef GANTRY_STATUS_PERIOD_TICKS
#define GANTRY_STATUS_PERIOD_TICKS 100u
#endif

/* PLACEHOLDER: new target while MOVING (FW-026 owed by the gantry team): reject, or hold one queued target. */
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

/* PLACEHOLDER (ruling 6): lift workspace from the lift home (min) switch, positive up. */
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

/* PLACEHOLDER (ruling 6): load cell scale in integer ADC counts, and the overload limit, rated 3 kg + 15 percent
 * (FW-018, FW-019). Over the limit means strictly above it; mid-lift it must hold for GANTRY_OVERLOAD_TICKS ticks. */
#ifndef GANTRY_LOAD_COUNTS_PER_KG
#define GANTRY_LOAD_COUNTS_PER_KG 1000
#endif
#ifndef GANTRY_LOAD_ZERO_COUNTS
#define GANTRY_LOAD_ZERO_COUNTS 0
#endif
#ifndef GANTRY_OVERLOAD_LIMIT_MILLIKG
#define GANTRY_OVERLOAD_LIMIT_MILLIKG 3450
#endif
#ifndef GANTRY_OVERLOAD_TICKS
#define GANTRY_OVERLOAD_TICKS 2u
#endif

/* PLACEHOLDER (ruling 6): heartbeat toggle period, 10 ms at 1 kHz (FW-025). */
#ifndef GANTRY_HEARTBEAT_PERIOD_TICKS
#define GANTRY_HEARTBEAT_PERIOD_TICKS 10u
#endif

/* PLACEHOLDER (ruling 6): encoder against commanded position, 2 mm at 60000 counts/m (FW-020); over it faults. */
#ifndef GANTRY_ENCODER_MISMATCH_COUNTS
#define GANTRY_ENCODER_MISMATCH_COUNTS 120
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
