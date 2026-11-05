#define MECHSIM_REF_HEARTBEAT_CHANNEL 0u
#include "mechsim_board.h"

#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ != 201112L)
#error "firmware application code is C11 (FW-001)"
#endif

#ifndef MECHSIM_APP_READY_TICKS
#define MECHSIM_APP_READY_TICKS 10u
#endif
#if MECHSIM_APP_READY_TICKS < 1
#error "MECHSIM_APP_READY_TICKS must be at least 1"
#endif
#if MECHSIM_APP_TICK_HZ < 1
#error "MECHSIM_APP_TICK_HZ must be at least 1"
#endif

#define APP_STATE_NOT_HOMED 1u
#define APP_STATE_READY 2u

#define APP_FAULT_NONE 0u
#define APP_FAULT_EVENT_LOG_FULL 1u

#define APP_EVENT_TICK 1u
#define APP_EVENT_UART_RX 2u

typedef struct app_state {
    uint8_t mode;
    uint32_t ticks_to_ready;
    uint32_t tick_number;
    uint8_t heartbeat;
} app_state;

static app_state g_app;

void mechsim_app_init(void)
{
    g_app.mode = APP_STATE_NOT_HOMED;
    g_app.ticks_to_ready = 0u;
    g_app.tick_number = 0u;
    g_app.heartbeat = MECHSIM_LEVEL_LOW;
}

static void advance_mode(void)
{
    if (g_app.mode == APP_STATE_READY) {
        return;
    }
    g_app.ticks_to_ready = g_app.ticks_to_ready + 1u;
    if (g_app.ticks_to_ready >= (uint32_t)MECHSIM_APP_READY_TICKS) {
        g_app.mode = APP_STATE_READY;
    }
}

static void toggle_heartbeat(void)
{
    if (g_app.heartbeat == MECHSIM_LEVEL_LOW) {
        g_app.heartbeat = MECHSIM_LEVEL_HIGH;
    } else {
        g_app.heartbeat = MECHSIM_LEVEL_LOW;
    }
}

static void clear_outputs(mechsim_board_outputs *outputs)
{
    uint32_t index;
    for (index = 0u; index < (uint32_t)MECHSIM_AXIS_COUNT; index = index + 1u) {
        outputs->step_count[index] = 0u;
        outputs->direction[index] = MECHSIM_LEVEL_LOW;
        outputs->enable[index] = MECHSIM_LEVEL_LOW;
    }
    for (index = 0u; index < (uint32_t)MECHSIM_DIGITAL_OUT_COUNT; index = index + 1u) {
        outputs->digital_out[index] = MECHSIM_LEVEL_LOW;
    }
    for (index = 0u; index < (uint32_t)MECHSIM_UART_TX_BYTES; index = index + 1u) {
        outputs->uart_tx[index] = 0u;
    }
    for (index = 0u; index < (uint32_t)MECHSIM_EVENT_LOG_COUNT; index = index + 1u) {
        outputs->event_log[index].code = 0u;
        outputs->event_log[index].arg = 0u;
    }
    outputs->uart_tx_count = 0u;
    outputs->event_count = 0u;
    outputs->fault_code = APP_FAULT_NONE;
}

static void log_event(mechsim_board_outputs *outputs, uint8_t code, uint16_t arg)
{
    if (outputs->event_count >= (uint8_t)MECHSIM_EVENT_LOG_COUNT) {
        outputs->fault_code = APP_FAULT_EVENT_LOG_FULL;
        return;
    }
    outputs->event_log[outputs->event_count].code = code;
    outputs->event_log[outputs->event_count].arg = arg;
    outputs->event_count = (uint8_t)(outputs->event_count + 1u);
}

static uint16_t echo_length(uint16_t received)
{
    if (received > (uint16_t)MECHSIM_UART_TX_BYTES) {
        return (uint16_t)MECHSIM_UART_TX_BYTES;
    }
    return received;
}

void mechsim_app_tick(const mechsim_board_inputs *inputs, mechsim_board_outputs *outputs)
{
    uint16_t index;
    uint16_t length;

    advance_mode();
    toggle_heartbeat();
    g_app.tick_number = g_app.tick_number + 1u;
    clear_outputs(outputs);

    log_event(outputs, APP_EVENT_TICK, (uint16_t)(g_app.tick_number & 0xFFFFu));

    length = echo_length(inputs->uart_rx_count);
    for (index = 0u; index < length; index = (uint16_t)(index + 1u)) {
        outputs->uart_tx[index] = inputs->uart_rx[index];
        log_event(outputs, APP_EVENT_UART_RX, inputs->uart_rx[index]);
    }
    outputs->uart_tx_count = length;
    outputs->digital_out[MECHSIM_REF_HEARTBEAT_CHANNEL] = g_app.heartbeat;
    outputs->state_code = g_app.mode;
}

uint32_t mechsim_app_tick_hz(void)
{
    return (uint32_t)MECHSIM_APP_TICK_HZ;
}

uint32_t mechsim_app_board_interface_version(void)
{
    return (uint32_t)MECHSIM_BOARD_INTERFACE_VERSION;
}
