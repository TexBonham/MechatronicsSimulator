// Test application for the native shim: every output field is a fixed function of one input field, so a round trip
// shows each field crossed the board interface unchanged (CTL-003). The stepper direction and enable arrays stay low:
// the shim derives them from digital_out (fw.md ruling). Compile definitions vary the version symbol and
// the declared tick rate.
#include <cstddef>
#include <cstdint>

#include "mechsim_board.h"

#ifndef MIRROR_VERSION_OFFSET
#define MIRROR_VERSION_OFFSET 0U
#endif
#ifndef MIRROR_TICK_HZ
#define MIRROR_TICK_HZ 1000U
#endif

namespace {

std::uint8_t ticks_since_init = 0;
std::uint8_t init_count = 0;

}  // namespace

extern "C" {

void mechsim_app_init() {
    ticks_since_init = 0;
    init_count = static_cast<std::uint8_t>(init_count + 1U);
}

void mechsim_app_tick(const mechsim_board_inputs* inputs, mechsim_board_outputs* outputs) {
    ticks_since_init = static_cast<std::uint8_t>(ticks_since_init + 1U);
    for (std::size_t axis = 0; axis < MECHSIM_AXIS_COUNT; ++axis) {
        outputs->step_count[axis] = static_cast<std::uint32_t>(inputs->counter_delta[axis]);
        outputs->direction[axis] = MECHSIM_LEVEL_LOW;
        outputs->enable[axis] = MECHSIM_LEVEL_LOW;
    }
    for (std::size_t pin = 0; pin < MECHSIM_DIGITAL_OUT_COUNT; ++pin) {
        outputs->digital_out[pin] = inputs->digital_in[MECHSIM_DIGITAL_IN_COUNT - 1U - pin];
    }
    for (std::size_t byte = 0; byte < MECHSIM_UART_TX_BYTES; ++byte) {
        outputs->uart_tx[byte] = static_cast<std::uint8_t>(inputs->uart_rx[byte] ^ 0xFFU);
    }
    outputs->uart_tx_count = inputs->uart_rx_count;
    for (std::size_t entry = 0; entry < MECHSIM_EVENT_LOG_COUNT; ++entry) {
        outputs->event_log[entry].code = static_cast<std::uint8_t>(entry + 1U);
        outputs->event_log[entry].arg = inputs->adc_sample[entry];
    }
    outputs->event_count = static_cast<std::uint8_t>(MECHSIM_EVENT_LOG_COUNT);
    outputs->state_code = ticks_since_init;
    outputs->fault_code = init_count;
    // Trigger values on the last ADC channel make the application break the output bounds (overflow tests).
    if (inputs->adc_sample[MECHSIM_ADC_COUNT - 1U] == 0xFFFFU) {
        outputs->uart_tx_count = static_cast<std::uint16_t>(MECHSIM_UART_TX_BYTES + 1U);
    }
    if (inputs->adc_sample[MECHSIM_ADC_COUNT - 1U] == 0xFFFEU) {
        outputs->event_count = static_cast<std::uint8_t>(MECHSIM_EVENT_LOG_COUNT + 1U);
    }
}

uint32_t mechsim_app_tick_hz() {
    return MIRROR_TICK_HZ;
}

#ifndef MIRROR_NO_VERSION
uint32_t mechsim_app_board_interface_version() {
    return MECHSIM_BOARD_INTERFACE_VERSION + MIRROR_VERSION_OFFSET;
}
#endif

}  // extern "C"
