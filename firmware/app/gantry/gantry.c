/* Gantry reference controller, milestone 1: G1 to G4 (FW-013 to FW-016, FW-022, FW-023, FW-029, FW-030).
 * Integer fixed point throughout: positions and speeds in steps scaled by 2^16 (Q16), per tick.
 */
#include "mechsim_board.h"
#include "gantry_config.h"

#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ != 201112L)
#error "firmware application code is C11 (FW-001)"
#endif

#if MECHSIM_APP_TICK_HZ < 1
#error "MECHSIM_APP_TICK_HZ must be at least 1"
#endif
#if (GANTRY_WORKSPACE_X_UM < 1) || (GANTRY_WORKSPACE_Y_UM < 1) || (GANTRY_STEPS_PER_M < 1) || (GANTRY_COUNTS_PER_M < 1)
#error "gantry workspace and scale parameters must be positive"
#endif
#if (GANTRY_MAX_ACCEL_UM_S2 < 1) || (GANTRY_MAX_SPEED_UM_S < 1) || (GANTRY_SLOW_SPEED_UM_S < 1)
#error "gantry profile parameters must be positive"
#endif
#if (GANTRY_HOMING_SPEED_UM_S < 1) || (GANTRY_STATUS_PERIOD_TICKS < 1) || (GANTRY_SLOW_ZONE_UM < 0)
#error "gantry homing, status and zone parameters out of range"
#endif
#if GANTRY_LINE_MAX < 8
#error "GANTRY_LINE_MAX must hold the longest status line"
#endif

#define Q16_ONE 65536
#define UM_PER_M 1000000

// ===== State ==========================================================================================================

typedef struct gantry_axis {
    int32_t position_steps;
    int32_t move_start_steps;
    int32_t move_sign;
    int64_t move_total_q16;
    int64_t move_progress_q16;
    int64_t velocity_q16;
    int64_t homing_progress_q16;
    int32_t encoder_counts;
    uint8_t direction_level;
    uint8_t homed;
    uint32_t steps_this_tick;
} gantry_axis;

typedef struct gantry_limits {
    int64_t accel_q16;
    int64_t speed_q16;
    int64_t slow_speed_q16;
    int64_t homing_speed_q16;
    int64_t slow_zone_q16;
    int64_t homing_max_q16;
    int64_t workspace_q16[GANTRY_AXIS_COUNT];
} gantry_limits;

typedef struct gantry_app {
    uint8_t state;
    uint8_t latched_fault;
    uint8_t tick_fault;
    uint8_t heartbeat;
    uint8_t entering_fault;
    gantry_axis axis[GANTRY_AXIS_COUNT];
    gantry_limits limits;
    uint8_t has_queued_target;
    int32_t queued_target_steps[GANTRY_AXIS_COUNT];
    uint32_t status_ticks;
    uint8_t line[GANTRY_LINE_MAX];
    uint32_t line_length;
    uint8_t line_overflow;
    uint8_t tx_queue[GANTRY_TX_QUEUE_BYTES];
    uint32_t tx_head;
    uint32_t tx_count;
    mechsim_board_event event_queue[GANTRY_EVENT_QUEUE_COUNT];
    uint32_t event_head;
    uint32_t event_count;
} gantry_app;

static gantry_app g_app;

// ===== Fixed point ====================================================================================================

static int64_t speed_to_q16(int64_t um_per_s)
{
    return (um_per_s * (int64_t)GANTRY_STEPS_PER_M * Q16_ONE) / ((int64_t)UM_PER_M * (int64_t)MECHSIM_APP_TICK_HZ);
}

static int64_t accel_to_q16(int64_t um_per_s2)
{
    int64_t hz = (int64_t)MECHSIM_APP_TICK_HZ;
    int64_t value = (um_per_s2 * (int64_t)GANTRY_STEPS_PER_M * Q16_ONE) / ((int64_t)UM_PER_M * hz * hz);
    if (value < 1) {
        return 1;
    }
    return value;
}

static int64_t length_to_q16(int64_t um)
{
    return ((um * (int64_t)GANTRY_STEPS_PER_M + (UM_PER_M / 2)) / UM_PER_M) * Q16_ONE;
}

static int32_t um_to_steps(int64_t um)
{
    return (int32_t)((um * (int64_t)GANTRY_STEPS_PER_M + (UM_PER_M / 2)) / UM_PER_M);
}

static int64_t isqrt64(uint64_t value)
{
    uint64_t result = 0u;
    uint64_t bit = (uint64_t)1u << 62u;
    while (bit > value) {
        bit = bit >> 2u;
    }
    while (bit != 0u) {
        if (value >= result + bit) {
            value = value - (result + bit);
            result = (result >> 1u) + bit;
        } else {
            result = result >> 1u;
        }
        bit = bit >> 2u;
    }
    return (int64_t)result;
}

static int64_t min64(int64_t a, int64_t b)
{
    if (a < b) {
        return a;
    }
    return b;
}

static int64_t max64(int64_t a, int64_t b)
{
    if (a > b) {
        return a;
    }
    return b;
}

// ===== Queues =========================================================================================================

static void enter_fault(uint8_t code);

static void log_event(uint8_t code, uint16_t arg)
{
    uint32_t slot;
    if (g_app.event_count >= (uint32_t)GANTRY_EVENT_QUEUE_COUNT) {
        if (g_app.state != GANTRY_STATE_FAULT) {
            enter_fault(GANTRY_FAULT_EVENT_OVERFLOW);
        }
        return;
    }
    slot = (g_app.event_head + g_app.event_count) % (uint32_t)GANTRY_EVENT_QUEUE_COUNT;
    g_app.event_queue[slot].code = code;
    g_app.event_queue[slot].arg = arg;
    g_app.event_count = g_app.event_count + 1u;
}

static void send_bytes(const uint8_t *bytes, uint32_t length)
{
    uint32_t index;
    if (length > (uint32_t)GANTRY_TX_QUEUE_BYTES - g_app.tx_count) {
        if (g_app.state != GANTRY_STATE_FAULT) {
            enter_fault(GANTRY_FAULT_TX_OVERFLOW);
        }
        return;
    }
    for (index = 0u; index < length; index = index + 1u) {
        uint32_t slot = (g_app.tx_head + g_app.tx_count) % (uint32_t)GANTRY_TX_QUEUE_BYTES;
        g_app.tx_queue[slot] = bytes[index];
        g_app.tx_count = g_app.tx_count + 1u;
    }
}

// ===== Status lines ===================================================================================================

static uint32_t append_text(uint8_t *buffer, uint32_t length, const char *text)
{
    uint32_t index = 0u;
    while (text[index] != '\0') {
        buffer[length] = (uint8_t)text[index];
        length = length + 1u;
        index = index + 1u;
    }
    return length;
}

static uint32_t append_unsigned(uint8_t *buffer, uint32_t length, uint32_t value, uint32_t min_digits)
{
    uint8_t digits[10];
    uint32_t count = 0u;
    while ((value != 0u) || (count < min_digits) || (count == 0u)) {
        digits[count] = (uint8_t)('0' + (value % 10u));
        value = value / 10u;
        count = count + 1u;
    }
    while (count > 0u) {
        count = count - 1u;
        buffer[length] = digits[count];
        length = length + 1u;
    }
    return length;
}

static uint32_t append_metres(uint8_t *buffer, uint32_t length, int64_t um)
{
    uint64_t magnitude;
    if (um < 0) {
        buffer[length] = (uint8_t)'-';
        length = length + 1u;
        magnitude = (uint64_t)(-um);
    } else {
        magnitude = (uint64_t)um;
    }
    length = append_unsigned(buffer, length, (uint32_t)(magnitude / (uint64_t)UM_PER_M), 1u);
    buffer[length] = (uint8_t)'.';
    length = length + 1u;
    return append_unsigned(buffer, length, (uint32_t)(magnitude % (uint64_t)UM_PER_M), 6u);
}

static int64_t encoder_um(const gantry_axis *axis)
{
    return ((int64_t)axis->encoder_counts * UM_PER_M) / (int64_t)GANTRY_COUNTS_PER_M;
}

static void send_position(void)
{
    uint8_t line[48];
    uint32_t length = append_text(line, 0u, "current_position ");
    length = append_metres(line, length, encoder_um(&g_app.axis[GANTRY_AXIS_X]));
    line[length] = (uint8_t)' ';
    length = append_metres(line, length + 1u, encoder_um(&g_app.axis[GANTRY_AXIS_Y]));
    line[length] = (uint8_t)'\n';
    send_bytes(line, length + 1u);
}

/* The design's "ready / fault" message, named status: the state code and the fault code of this report. */
static void send_status(uint8_t fault)
{
    uint8_t line[20];
    uint32_t length = append_text(line, 0u, "status ");
    length = append_unsigned(line, length, (uint32_t)g_app.state, 1u);
    line[length] = (uint8_t)' ';
    length = append_unsigned(line, length + 1u, (uint32_t)fault, 1u);
    line[length] = (uint8_t)'\n';
    send_bytes(line, length + 1u);
}

// ===== State machine ==================================================================================================

static void set_state(uint8_t next)
{
    if (next == g_app.state) {
        return;
    }
    log_event(GANTRY_EVENT_STATE, (uint16_t)(((uint32_t)g_app.state << 8u) | next));
    g_app.state = next;
    g_app.status_ticks = 0u;
}

static void stop_axes(void)
{
    uint32_t index;
    for (index = 0u; index < GANTRY_AXIS_COUNT; index = index + 1u) {
        g_app.axis[index].velocity_q16 = 0;
        g_app.axis[index].move_total_q16 = 0;
        g_app.axis[index].move_progress_q16 = 0;
    }
}

static void enter_fault(uint8_t code)
{
    uint32_t index;
    /* Logging the fault can overflow the queue again; the first fault entered wins. */
    if (g_app.entering_fault != 0u) {
        return;
    }
    g_app.entering_fault = 1u;
    g_app.latched_fault = code;
    stop_axes();
    for (index = 0u; index < GANTRY_AXIS_COUNT; index = index + 1u) {
        g_app.axis[index].homed = 0u;
        g_app.axis[index].steps_this_tick = 0u;
    }
    g_app.has_queued_target = 0u;
    set_state(GANTRY_STATE_FAULT);
    log_event(GANTRY_EVENT_FAULT, code);
    send_status(code);
    g_app.entering_fault = 0u;
}

static void refuse(uint8_t code)
{
    g_app.tick_fault = code;
    log_event(GANTRY_EVENT_REFUSED, code);
    send_status(code);
}

static void start_move(const int32_t target_steps[GANTRY_AXIS_COUNT])
{
    uint32_t index;
    for (index = 0u; index < GANTRY_AXIS_COUNT; index = index + 1u) {
        gantry_axis *axis = &g_app.axis[index];
        int64_t delta = (int64_t)target_steps[index] - (int64_t)axis->position_steps;
        axis->move_start_steps = axis->position_steps;
        axis->move_progress_q16 = 0;
        axis->velocity_q16 = 0;
        if (delta < 0) {
            axis->move_sign = -1;
            axis->move_total_q16 = -delta * Q16_ONE;
            axis->direction_level = GANTRY_DIR_NEGATIVE_LEVEL;
        } else {
            axis->move_sign = 1;
            axis->move_total_q16 = delta * Q16_ONE;
            if (delta > 0) {
                axis->direction_level = GANTRY_DIR_POSITIVE_LEVEL;
            }
        }
    }
    set_state(GANTRY_STATE_MOVING);
}

static void start_homing(void)
{
    uint32_t index;
    g_app.latched_fault = GANTRY_FAULT_NONE;
    g_app.has_queued_target = 0u;
    stop_axes();
    for (index = 0u; index < GANTRY_AXIS_COUNT; index = index + 1u) {
        g_app.axis[index].homed = 0u;
        g_app.axis[index].homing_progress_q16 = 0;
        g_app.axis[index].direction_level = GANTRY_DIR_NEGATIVE_LEVEL;
    }
    set_state(GANTRY_STATE_HOMING);
}

// ===== Command parsing ================================================================================================

static uint8_t parse_metres(const uint8_t *text, uint32_t length, int64_t *um_out)
{
    uint32_t index = 0u;
    uint32_t digits = 0u;
    uint32_t fraction_digits = 0u;
    int64_t whole = 0;
    int64_t fraction = 0;
    int64_t negative = 0;
    if ((index < length) && ((text[index] == (uint8_t)'-') || (text[index] == (uint8_t)'+'))) {
        if (text[index] == (uint8_t)'-') {
            negative = 1;
        }
        index = index + 1u;
    }
    while ((index < length) && (text[index] >= (uint8_t)'0') && (text[index] <= (uint8_t)'9')) {
        if (whole < (int64_t)UM_PER_M) {
            whole = (whole * 10) + (int64_t)(text[index] - (uint8_t)'0');
        }
        digits = digits + 1u;
        index = index + 1u;
    }
    if ((index < length) && (text[index] == (uint8_t)'.')) {
        index = index + 1u;
        while ((index < length) && (text[index] >= (uint8_t)'0') && (text[index] <= (uint8_t)'9')) {
            if (fraction_digits >= 6u) {
                return 0u;
            }
            fraction = (fraction * 10) + (int64_t)(text[index] - (uint8_t)'0');
            fraction_digits = fraction_digits + 1u;
            index = index + 1u;
        }
    }
    if ((index != length) || ((digits + fraction_digits) == 0u)) {
        return 0u;
    }
    while (fraction_digits < 6u) {
        fraction = fraction * 10;
        fraction_digits = fraction_digits + 1u;
    }
    *um_out = (whole * UM_PER_M) + fraction;
    if (negative == 1) {
        *um_out = -*um_out;
    }
    return 1u;
}

static uint8_t token_is(const uint8_t *text, uint32_t length, const char *word)
{
    uint32_t index = 0u;
    while ((index < length) && (word[index] != '\0')) {
        if (text[index] != (uint8_t)word[index]) {
            return 0u;
        }
        index = index + 1u;
    }
    if ((index == length) && (word[index] == '\0')) {
        return 1u;
    }
    return 0u;
}

static uint8_t inside_workspace(int64_t x_um, int64_t y_um)
{
    if ((x_um < 0) || (y_um < 0) || (x_um > (int64_t)GANTRY_WORKSPACE_X_UM) || (y_um > (int64_t)GANTRY_WORKSPACE_Y_UM)) {
        return 0u;
    }
    return 1u;
}

static void handle_target(uint8_t parsed, int64_t x_um, int64_t y_um)
{
    int32_t target_steps[GANTRY_AXIS_COUNT];
    log_event(GANTRY_EVENT_COMMAND, GANTRY_COMMAND_TARGET);
    if (parsed == 0u) {
        refuse(GANTRY_FAULT_MALFORMED);
        return;
    }
    if (g_app.state == GANTRY_STATE_FAULT) {
        refuse(GANTRY_FAULT_IN_FAULT);
        return;
    }
    if (g_app.state == GANTRY_STATE_NOT_HOMED) {
        refuse(GANTRY_FAULT_NOT_HOMED);
        return;
    }
    if (g_app.state == GANTRY_STATE_HOMING) {
        refuse(GANTRY_FAULT_BUSY);
        return;
    }
    if (inside_workspace(x_um, y_um) == 0u) {
        refuse(GANTRY_FAULT_OUTSIDE_WORKSPACE);
        return;
    }
    target_steps[GANTRY_AXIS_X] = um_to_steps(x_um);
    target_steps[GANTRY_AXIS_Y] = um_to_steps(y_um);
    if (g_app.state == GANTRY_STATE_MOVING) {
        if (GANTRY_NEW_TARGET_POLICY != GANTRY_POLICY_QUEUE) {
            refuse(GANTRY_FAULT_BUSY);
            return;
        }
        if (g_app.has_queued_target != 0u) {
            refuse(GANTRY_FAULT_QUEUE_FULL);
            return;
        }
        g_app.queued_target_steps[GANTRY_AXIS_X] = target_steps[GANTRY_AXIS_X];
        g_app.queued_target_steps[GANTRY_AXIS_Y] = target_steps[GANTRY_AXIS_Y];
        g_app.has_queued_target = 1u;
        return;
    }
    start_move(target_steps);
}

static void handle_home(void)
{
    log_event(GANTRY_EVENT_COMMAND, GANTRY_COMMAND_HOME);
    if ((g_app.state == GANTRY_STATE_HOMING) || (g_app.state == GANTRY_STATE_MOVING)) {
        refuse(GANTRY_FAULT_BUSY);
        return;
    }
    start_homing();
}

static void dispatch_line(void)
{
    uint32_t starts[4];
    uint32_t lengths[4];
    uint32_t tokens = 0u;
    uint32_t index = 0u;
    uint32_t length = g_app.line_length;
    int64_t x_um = 0;
    int64_t y_um = 0;
    uint8_t parsed = 0u;

    if ((length > 0u) && (g_app.line[length - 1u] == (uint8_t)'\r')) {
        length = length - 1u;
    }
    while (index < length) {
        if (g_app.line[index] == (uint8_t)' ') {
            index = index + 1u;
        } else {
            if (tokens == 4u) {
                tokens = 5u;
                break;
            }
            starts[tokens] = index;
            while ((index < length) && (g_app.line[index] != (uint8_t)' ')) {
                index = index + 1u;
            }
            lengths[tokens] = index - starts[tokens];
            tokens = tokens + 1u;
        }
    }
    if (tokens == 0u) {
        return;
    }
    if ((tokens <= 4u) && (token_is(&g_app.line[starts[0]], lengths[0], "target") == 1u)) {
        if ((tokens == 3u) && (parse_metres(&g_app.line[starts[1]], lengths[1], &x_um) == 1u)
            && (parse_metres(&g_app.line[starts[2]], lengths[2], &y_um) == 1u)) {
            parsed = 1u;
        }
        handle_target(parsed, x_um, y_um);
        return;
    }
    if ((tokens == 1u) && (token_is(&g_app.line[starts[0]], lengths[0], "home") == 1u)) {
        handle_home();
        return;
    }
    log_event(GANTRY_EVENT_COMMAND, GANTRY_COMMAND_UNKNOWN);
    refuse(GANTRY_FAULT_MALFORMED);
}

static void receive_bytes(const mechsim_board_inputs *inputs)
{
    uint32_t index;
    uint32_t count = inputs->uart_rx_count;
    if (count > (uint32_t)MECHSIM_UART_RX_BYTES) {
        count = (uint32_t)MECHSIM_UART_RX_BYTES;
    }
    for (index = 0u; index < count; index = index + 1u) {
        uint8_t byte = inputs->uart_rx[index];
        if (byte == (uint8_t)'\n') {
            if (g_app.line_overflow != 0u) {
                log_event(GANTRY_EVENT_COMMAND, GANTRY_COMMAND_UNKNOWN);
                refuse(GANTRY_FAULT_MALFORMED);
            } else {
                dispatch_line();
            }
            g_app.line_length = 0u;
            g_app.line_overflow = 0u;
        } else if (g_app.line_length < (uint32_t)GANTRY_LINE_MAX) {
            g_app.line[g_app.line_length] = byte;
            g_app.line_length = g_app.line_length + 1u;
        } else {
            g_app.line_overflow = 1u;
        }
    }
}

// ===== Motion =========================================================================================================

static uint32_t advance_progress(int64_t *progress_q16, int64_t velocity_q16)
{
    int64_t before = *progress_q16 / Q16_ONE;
    *progress_q16 = *progress_q16 + velocity_q16;
    return (uint32_t)((*progress_q16 / Q16_ONE) - before);
}

static int64_t zone_speed_cap(const gantry_axis *axis, uint32_t index)
{
    const gantry_limits *limits = &g_app.limits;
    int64_t position_q16 = ((int64_t)axis->move_start_steps * Q16_ONE) + (axis->move_sign * axis->move_progress_q16);
    int64_t distance_q16;
    uint64_t squared;
    if (axis->move_sign > 0) {
        distance_q16 = (limits->workspace_q16[index] - limits->slow_zone_q16) - position_q16;
    } else {
        distance_q16 = position_q16 - limits->slow_zone_q16;
    }
    if (distance_q16 <= 0) {
        return limits->slow_speed_q16;
    }
    squared = (uint64_t)(limits->slow_speed_q16 * limits->slow_speed_q16)
        + (uint64_t)(2 * limits->accel_q16) * (uint64_t)distance_q16;
    return isqrt64(squared);
}

static int64_t stop_speed_cap(int64_t accel_q16, int64_t remaining_q16)
{
    uint64_t squared = (uint64_t)(accel_q16 * accel_q16) + (uint64_t)(8 * accel_q16) * (uint64_t)remaining_q16;
    return (isqrt64(squared) - accel_q16) / 2;
}

static uint8_t move_axis(gantry_axis *axis, uint32_t index)
{
    const gantry_limits *limits = &g_app.limits;
    int64_t remaining_q16 = axis->move_total_q16 - axis->move_progress_q16;
    int64_t velocity;
    uint32_t steps;
    if (remaining_q16 <= 0) {
        axis->velocity_q16 = 0;
        return 1u;
    }
    velocity = min64(axis->velocity_q16 + limits->accel_q16, limits->speed_q16);
    velocity = min64(velocity, max64(zone_speed_cap(axis, index), axis->velocity_q16 - limits->accel_q16));
    velocity = min64(velocity, stop_speed_cap(limits->accel_q16, remaining_q16));
    velocity = max64(velocity, limits->accel_q16);
    velocity = min64(velocity, remaining_q16);
    steps = advance_progress(&axis->move_progress_q16, velocity);
    axis->velocity_q16 = velocity;
    axis->steps_this_tick = steps;
    axis->position_steps = axis->position_steps + (axis->move_sign * (int32_t)steps);
    if (axis->move_progress_q16 >= axis->move_total_q16) {
        return 1u;
    }
    return 0u;
}

static void run_moving(void)
{
    uint8_t arrived = 1u;
    uint32_t index;
    for (index = 0u; index < GANTRY_AXIS_COUNT; index = index + 1u) {
        if (move_axis(&g_app.axis[index], index) == 0u) {
            arrived = 0u;
        }
    }
    if (arrived == 0u) {
        return;
    }
    stop_axes();
    set_state(GANTRY_STATE_READY);
    send_position();
    send_status(GANTRY_FAULT_NONE);
}

static void start_queued_target(void)
{
    if ((g_app.state == GANTRY_STATE_READY) && (g_app.has_queued_target != 0u)) {
        g_app.has_queued_target = 0u;
        start_move(g_app.queued_target_steps);
    }
}

static void run_homing(const mechsim_board_inputs *inputs)
{
    static const uint32_t switch_channel[GANTRY_AXIS_COUNT] = {MECHSIM_CH_X_MIN_SWITCH_OUT, MECHSIM_CH_Y_MIN_SWITCH_OUT};
    const gantry_limits *limits = &g_app.limits;
    uint8_t all_homed = 1u;
    uint32_t index;
    for (index = 0u; index < GANTRY_AXIS_COUNT; index = index + 1u) {
        gantry_axis *axis = &g_app.axis[index];
        if (axis->homed != 0u) {
            continue;
        }
        if (inputs->digital_in[switch_channel[index]] == GANTRY_SWITCH_ACTIVE_LEVEL) {
            axis->homed = 1u;
            axis->velocity_q16 = 0;
            axis->position_steps = 0;
            axis->encoder_counts = 0;
            log_event(GANTRY_EVENT_ORIGIN, (uint16_t)index);
            continue;
        }
        all_homed = 0u;
        if (axis->homing_progress_q16 >= limits->homing_max_q16) {
            enter_fault(GANTRY_FAULT_HOMING_FAILED);
            return;
        }
        axis->velocity_q16 = min64(axis->velocity_q16 + limits->accel_q16, limits->homing_speed_q16);
        axis->steps_this_tick = advance_progress(&axis->homing_progress_q16, axis->velocity_q16);
    }
    if (all_homed == 1u) {
        set_state(GANTRY_STATE_READY);
        send_status(GANTRY_FAULT_NONE);
    }
}

static void accumulate_encoders(const mechsim_board_inputs *inputs)
{
    /* Quadrature: the A and B edge counts together are the encoder's full count (four per line). */
    static const uint32_t channel_a[GANTRY_AXIS_COUNT] = {MECHSIM_CH_X_ENCODER_CHANNEL_A, MECHSIM_CH_Y_ENCODER_CHANNEL_A};
    static const uint32_t channel_b[GANTRY_AXIS_COUNT] = {MECHSIM_CH_X_ENCODER_CHANNEL_B, MECHSIM_CH_Y_ENCODER_CHANNEL_B};
    uint32_t index;
    for (index = 0u; index < GANTRY_AXIS_COUNT; index = index + 1u) {
        int64_t sum = (int64_t)g_app.axis[index].encoder_counts + (int64_t)inputs->counter_delta[channel_a[index]] +
                      (int64_t)inputs->counter_delta[channel_b[index]];
        if (sum > (int64_t)INT32_MAX) {
            sum = (int64_t)INT32_MAX;
        }
        if (sum < (int64_t)INT32_MIN) {
            sum = (int64_t)INT32_MIN;
        }
        g_app.axis[index].encoder_counts = (int32_t)sum;
    }
}

// ===== Outputs ========================================================================================================

static void clear_outputs(mechsim_board_outputs *outputs)
{
    uint32_t index;
    for (index = 0u; index < (uint32_t)MECHSIM_AXIS_COUNT; index = index + 1u) {
        outputs->step_count[index] = 0u;
        /* The pin-table digital_out channels own direction and enable; the native shim fills these from them. */
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
    /* Milestone 1 never moves the lift: its driver stays enabled with no steps, so the motor holds the lift. */
    outputs->digital_out[MECHSIM_CH_LIFT_DRIVER_ENABLE] = GANTRY_ENABLE_ACTIVE_LEVEL;
}

static void write_axis(mechsim_board_outputs *outputs, uint32_t index, uint32_t step_ch, uint32_t dir_ch, uint32_t en_ch)
{
    uint8_t enable = GANTRY_ENABLE_INACTIVE_LEVEL;
    if ((g_app.state == GANTRY_STATE_HOMING) || (g_app.state == GANTRY_STATE_READY)
        || (g_app.state == GANTRY_STATE_MOVING)) {
        enable = GANTRY_ENABLE_ACTIVE_LEVEL;
    }
    outputs->step_count[step_ch] = g_app.axis[index].steps_this_tick;
    outputs->digital_out[dir_ch] = g_app.axis[index].direction_level;
    outputs->digital_out[en_ch] = enable;
}

static void drain_queues(mechsim_board_outputs *outputs)
{
    uint32_t sent = 0u;
    uint32_t logged = 0u;
    while ((g_app.tx_count > 0u) && (sent < (uint32_t)MECHSIM_UART_TX_BYTES)) {
        outputs->uart_tx[sent] = g_app.tx_queue[g_app.tx_head];
        g_app.tx_head = (g_app.tx_head + 1u) % (uint32_t)GANTRY_TX_QUEUE_BYTES;
        g_app.tx_count = g_app.tx_count - 1u;
        sent = sent + 1u;
    }
    while ((g_app.event_count > 0u) && (logged < (uint32_t)MECHSIM_EVENT_LOG_COUNT)) {
        outputs->event_log[logged] = g_app.event_queue[g_app.event_head];
        g_app.event_head = (g_app.event_head + 1u) % (uint32_t)GANTRY_EVENT_QUEUE_COUNT;
        g_app.event_count = g_app.event_count - 1u;
        logged = logged + 1u;
    }
    outputs->uart_tx_count = (uint16_t)sent;
    outputs->event_count = (uint8_t)logged;
}

// ===== Entry points ===================================================================================================

void mechsim_app_init(void)
{
    uint32_t index;
    uint8_t *bytes = (uint8_t *)&g_app;
    for (index = 0u; index < (uint32_t)sizeof(g_app); index = index + 1u) {
        bytes[index] = 0u;
    }
    g_app.state = GANTRY_STATE_NOT_HOMED;
    g_app.heartbeat = MECHSIM_LEVEL_LOW;
    g_app.limits.accel_q16 = accel_to_q16(GANTRY_MAX_ACCEL_UM_S2);
    g_app.limits.speed_q16 = speed_to_q16(GANTRY_MAX_SPEED_UM_S);
    g_app.limits.slow_speed_q16 = speed_to_q16(GANTRY_SLOW_SPEED_UM_S);
    g_app.limits.homing_speed_q16 = speed_to_q16(GANTRY_HOMING_SPEED_UM_S);
    g_app.limits.slow_zone_q16 = length_to_q16(GANTRY_SLOW_ZONE_UM);
    g_app.limits.homing_max_q16 = length_to_q16(GANTRY_HOMING_MAX_TRAVEL_UM);
    g_app.limits.workspace_q16[GANTRY_AXIS_X] = length_to_q16(GANTRY_WORKSPACE_X_UM);
    g_app.limits.workspace_q16[GANTRY_AXIS_Y] = length_to_q16(GANTRY_WORKSPACE_Y_UM);
    for (index = 0u; index < GANTRY_AXIS_COUNT; index = index + 1u) {
        g_app.axis[index].direction_level = GANTRY_DIR_NEGATIVE_LEVEL;
    }
}

void mechsim_app_tick(const mechsim_board_inputs *inputs, mechsim_board_outputs *outputs)
{
    uint32_t index;
    g_app.tick_fault = GANTRY_FAULT_NONE;
    for (index = 0u; index < GANTRY_AXIS_COUNT; index = index + 1u) {
        g_app.axis[index].steps_this_tick = 0u;
    }
    clear_outputs(outputs);
    accumulate_encoders(inputs);
    start_queued_target();
    receive_bytes(inputs);
    if (g_app.state == GANTRY_STATE_HOMING) {
        run_homing(inputs);
    } else if (g_app.state == GANTRY_STATE_MOVING) {
        run_moving();
    }
    if ((g_app.state == GANTRY_STATE_READY) || (g_app.state == GANTRY_STATE_MOVING)) {
        g_app.status_ticks = g_app.status_ticks + 1u;
        if (g_app.status_ticks >= (uint32_t)GANTRY_STATUS_PERIOD_TICKS) {
            g_app.status_ticks = 0u;
            send_position();
        }
    }
    write_axis(outputs, GANTRY_AXIS_X, MECHSIM_CH_X_DRIVER_STEP, MECHSIM_CH_X_DRIVER_DIR, MECHSIM_CH_X_DRIVER_ENABLE);
    write_axis(outputs, GANTRY_AXIS_Y, MECHSIM_CH_Y_DRIVER_STEP, MECHSIM_CH_Y_DRIVER_DIR, MECHSIM_CH_Y_DRIVER_ENABLE);
    drain_queues(outputs);
    if (g_app.state == GANTRY_STATE_FAULT) {
        g_app.heartbeat = MECHSIM_LEVEL_LOW;
        outputs->fault_code = g_app.latched_fault;
    } else {
        if (g_app.heartbeat == MECHSIM_LEVEL_LOW) {
            g_app.heartbeat = MECHSIM_LEVEL_HIGH;
        } else {
            g_app.heartbeat = MECHSIM_LEVEL_LOW;
        }
        outputs->fault_code = g_app.tick_fault;
    }
    outputs->digital_out[MECHSIM_CH_WATCHDOG_HEARTBEAT] = g_app.heartbeat;
    outputs->state_code = g_app.state;
}

uint32_t mechsim_app_tick_hz(void)
{
    return (uint32_t)MECHSIM_APP_TICK_HZ;
}

uint32_t mechsim_app_board_interface_version(void)
{
    return (uint32_t)MECHSIM_BOARD_INTERFACE_VERSION;
}
