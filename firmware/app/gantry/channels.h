/* Gantry board channel map, written by hand from designs/gantry/design.yaml pin_bindings (M1-H).
 *
 * Rule: each peripheral mode numbers its own channels from 0 in the order the pin table lists that mode's bindings
 * (pulse_output -> step_count[] axis, digital_output -> digital_out[], digital_input -> digital_in[],
 * counter_input -> counter_delta[]). The builder generates this file from the design by the same rule; integration
 * replaces this copy, so every channel the firmware uses is a macro here and nowhere else.
 *
 * The driver dir and enable pins are digital_output bindings, so they land on digital_out[], the single source of
 * truth for both; the native shim fills the stepper direction[] and enable[] arrays from them, so the firmware holds
 * those arrays low.
 */
#ifndef GANTRY_CHANNELS_H
#define GANTRY_CHANNELS_H

/* pulse_output, step_count[] axis index */
#define GANTRY_CH_X_STEP 0u    /* x_driver.step, PA0 */
#define GANTRY_CH_Y_STEP 1u    /* y_driver.step, PA3 */
#define GANTRY_CH_LIFT_STEP 2u /* lift_driver.step, PA6 */

/* digital_output, digital_out[] index */
#define GANTRY_CH_X_DIR 0u       /* x_driver.dir, PA1 */
#define GANTRY_CH_X_ENABLE 1u    /* x_driver.enable, PA2 */
#define GANTRY_CH_Y_DIR 2u       /* y_driver.dir, PA4 */
#define GANTRY_CH_Y_ENABLE 3u    /* y_driver.enable, PA5 */
#define GANTRY_CH_LIFT_DIR 4u    /* lift_driver.dir, PA7 */
#define GANTRY_CH_LIFT_ENABLE 5u /* lift_driver.enable, PA8 */
#define GANTRY_CH_HEARTBEAT 6u   /* watchdog.heartbeat, PB6 */

/* digital_input, digital_in[] index */
#define GANTRY_CH_X_MIN 0u    /* x_min_switch.out, PB0 */
#define GANTRY_CH_X_MAX 1u    /* x_max_switch.out, PB1 */
#define GANTRY_CH_Y_MIN 2u    /* y_min_switch.out, PB2 */
#define GANTRY_CH_Y_MAX 3u    /* y_max_switch.out, PB3 */
#define GANTRY_CH_LIFT_MIN 4u /* lift_min_switch.out, PB4 */
#define GANTRY_CH_LIFT_MAX 5u /* lift_max_switch.out, PB5 */

/* counter_input, counter_delta[] index */
#define GANTRY_CH_X_ENC_A 0u    /* x_encoder.channel_a, PC0 */
#define GANTRY_CH_X_ENC_B 1u    /* x_encoder.channel_b, PC1 */
#define GANTRY_CH_Y_ENC_A 2u    /* y_encoder.channel_a, PC2 */
#define GANTRY_CH_Y_ENC_B 3u    /* y_encoder.channel_b, PC3 */
#define GANTRY_CH_LIFT_ENC_A 4u /* lift_encoder.channel_a, PC4 */
#define GANTRY_CH_LIFT_ENC_B 5u /* lift_encoder.channel_b, PC5 */

#endif
