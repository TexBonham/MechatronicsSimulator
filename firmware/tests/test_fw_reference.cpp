#include <catch2/catch_test_macros.hpp>

#include <array>
#include <climits>
#include <cstdint>
#include <cstring>
#include <type_traits>

extern "C" {
#include "mechsim_board.h"
#define MECHSIM_REF_HEARTBEAT_CHANNEL 0u
}

namespace {

constexpr std::uint8_t state_not_homed = 1U;
constexpr std::uint8_t state_ready = 2U;
constexpr std::uint8_t fault_none = 0U;
constexpr std::uint8_t fault_event_log_full = 1U;
constexpr std::uint8_t event_tick = 1U;
constexpr std::uint8_t event_uart_rx = 2U;

mechsim_board_outputs run_tick(const mechsim_board_inputs& inputs)
{
    mechsim_board_outputs outputs{};
    std::memset(&outputs, 0xAA, sizeof(outputs));
    mechsim_app_tick(&inputs, &outputs);
    return outputs;
}

void fill_rx(mechsim_board_inputs& inputs, std::uint16_t count)
{
    for (std::uint16_t index = 0U; index < MECHSIM_UART_RX_BYTES; index = static_cast<std::uint16_t>(index + 1U)) {
        inputs.uart_rx[index] = static_cast<std::uint8_t>(index + 1U);
    }
    inputs.uart_rx_count = count;
}

void require_echo(const mechsim_board_outputs& outputs, std::uint16_t expected)
{
    REQUIRE(outputs.uart_tx_count == expected);
    for (std::uint16_t index = 0U; index < expected; index = static_cast<std::uint16_t>(index + 1U)) {
        REQUIRE(outputs.uart_tx[index] == static_cast<std::uint8_t>(index + 1U));
    }
}

}  // namespace

TEST_CASE("board header declares plain C structs of fixed-width fields", "[FW-003][FW-004][FW-009][FW-010][CTL-003]")
{
    static_assert(std::is_same_v<decltype(mechsim_board_inputs::digital_in), std::uint8_t[32]>);
    static_assert(std::is_same_v<decltype(mechsim_board_inputs::counter_delta), std::int32_t[8]>);
    static_assert(std::is_same_v<decltype(mechsim_board_inputs::adc_sample), std::uint16_t[16]>);
    static_assert(std::is_same_v<decltype(mechsim_board_inputs::uart_rx), std::uint8_t[64]>);
    static_assert(std::is_same_v<decltype(mechsim_board_inputs::uart_rx_count), std::uint16_t>);
    static_assert(std::is_same_v<decltype(mechsim_board_event::code), std::uint8_t>);
    static_assert(std::is_same_v<decltype(mechsim_board_event::arg), std::uint16_t>);
    static_assert(std::is_same_v<decltype(mechsim_board_outputs::step_count), std::uint32_t[8]>);
    static_assert(std::is_same_v<decltype(mechsim_board_outputs::direction), std::uint8_t[8]>);
    static_assert(std::is_same_v<decltype(mechsim_board_outputs::enable), std::uint8_t[8]>);
    static_assert(std::is_same_v<decltype(mechsim_board_outputs::digital_out), std::uint8_t[32]>);
    static_assert(std::is_same_v<decltype(mechsim_board_outputs::uart_tx), std::uint8_t[64]>);
    static_assert(std::is_same_v<decltype(mechsim_board_outputs::uart_tx_count), std::uint16_t>);
    static_assert(std::is_same_v<decltype(mechsim_board_outputs::state_code), std::uint8_t>);
    static_assert(std::is_same_v<decltype(mechsim_board_outputs::fault_code), std::uint8_t>);
    static_assert(std::is_same_v<decltype(mechsim_board_outputs::event_log), mechsim_board_event[16]>);
    static_assert(std::is_same_v<decltype(mechsim_board_outputs::event_count), std::uint8_t>);
    static_assert(std::is_standard_layout_v<mechsim_board_inputs> && std::is_trivially_copyable_v<mechsim_board_inputs>);
    static_assert(std::is_standard_layout_v<mechsim_board_outputs> && std::is_trivially_copyable_v<mechsim_board_outputs>);
    static_assert(std::is_same_v<decltype(&mechsim_app_init), void (*)()>);
    static_assert(
        std::is_same_v<decltype(&mechsim_app_tick), void (*)(const mechsim_board_inputs*, mechsim_board_outputs*)>);
    static_assert(std::is_same_v<decltype(&mechsim_app_tick_hz), std::uint32_t (*)()>);

    REQUIRE(MECHSIM_BOARD_INTERFACE_VERSION == 2U * 65536U + 1U);
    REQUIRE(MECHSIM_DIGITAL_IN_COUNT == 32U);
    REQUIRE(MECHSIM_COUNTER_COUNT == 8U);
    REQUIRE(MECHSIM_ADC_COUNT == 16U);
    REQUIRE(MECHSIM_UART_RX_BYTES == 64U);
    REQUIRE(MECHSIM_UART_TX_BYTES == 64U);
    REQUIRE(MECHSIM_AXIS_COUNT == 8U);
    REQUIRE(MECHSIM_DIGITAL_OUT_COUNT == 32U);
    REQUIRE(MECHSIM_EVENT_LOG_COUNT == 16U);
}

TEST_CASE("tick rate defaults to the header's 1 kHz", "[FW-010][CTL-002]")
{
    REQUIRE(MECHSIM_APP_TICK_HZ == 1000U);
    REQUIRE(mechsim_app_tick_hz() == 1000U);
}

TEST_CASE("init enters NOT_HOMED and READY follows the configured tick count", "[FW-010][FW-011][CTL-002][CTL-018]")
{
    mechsim_app_init();
    const mechsim_board_inputs inputs{};
    for (int tick = 1; tick < 10; ++tick) {
        REQUIRE(run_tick(inputs).state_code == state_not_homed);
    }
    REQUIRE(run_tick(inputs).state_code == state_ready);
    for (int tick = 0; tick < 100000; ++tick) {
        run_tick(inputs);
    }
    REQUIRE(run_tick(inputs).state_code == state_ready);
}

TEST_CASE("init after READY restarts from NOT_HOMED with the heartbeat low", "[FW-010][CTL-002][CTL-018]")
{
    mechsim_app_init();
    const mechsim_board_inputs inputs{};
    for (int tick = 0; tick < 11; ++tick) {
        run_tick(inputs);
    }
    REQUIRE(run_tick(inputs).state_code == state_ready);
    mechsim_app_init();
    const mechsim_board_outputs outputs = run_tick(inputs);
    REQUIRE(outputs.state_code == state_not_homed);
    REQUIRE(outputs.digital_out[0] == MECHSIM_LEVEL_HIGH);
    REQUIRE(outputs.event_log[0].arg == 1U);
}

TEST_CASE("heartbeat toggles every tick", "[FW-010][FW-011]")
{
    mechsim_app_init();
    const mechsim_board_inputs inputs{};
    std::uint8_t expected = MECHSIM_LEVEL_HIGH;
    for (int tick = 0; tick < 25; ++tick) {
        REQUIRE(run_tick(inputs).digital_out[0] == expected);
        if (expected == MECHSIM_LEVEL_HIGH) {
            expected = MECHSIM_LEVEL_LOW;
        } else {
            expected = MECHSIM_LEVEL_HIGH;
        }
    }
}

TEST_CASE("every output field is written each tick and steps are zero", "[FW-010][CTL-003]")
{
    mechsim_app_init();
    const mechsim_board_outputs outputs = run_tick(mechsim_board_inputs{});
    for (std::uint32_t axis = 0U; axis < MECHSIM_AXIS_COUNT; ++axis) {
        REQUIRE(outputs.step_count[axis] == 0U);
        REQUIRE(outputs.direction[axis] == MECHSIM_LEVEL_LOW);
        REQUIRE(outputs.enable[axis] == MECHSIM_LEVEL_LOW);
    }
    for (std::uint32_t pin = 0U; pin < MECHSIM_DIGITAL_OUT_COUNT; ++pin) {
        REQUIRE(outputs.digital_out[pin] == MECHSIM_LEVEL_LOW);
    }
    for (std::uint32_t index = 0U; index < MECHSIM_UART_TX_BYTES; ++index) {
        REQUIRE(outputs.uart_tx[index] == 0U);
    }
    for (std::uint32_t index = 1U; index < MECHSIM_EVENT_LOG_COUNT; ++index) {
        REQUIRE(outputs.event_log[index].code == 0U);
        REQUIRE(outputs.event_log[index].arg == 0U);
    }
    REQUIRE(outputs.uart_tx_count == 0U);
    REQUIRE(outputs.digital_out[0] == MECHSIM_LEVEL_HIGH);
    REQUIRE(outputs.state_code == state_not_homed);
    REQUIRE(outputs.fault_code == fault_none);
    REQUIRE(outputs.event_count == 1U);
}

TEST_CASE("UART echo with an empty receive buffer sends nothing", "[FW-011][CTL-004]")
{
    mechsim_app_init();
    mechsim_board_inputs inputs{};
    fill_rx(inputs, 0U);
    require_echo(run_tick(inputs), 0U);
}

TEST_CASE("UART echo returns a partial buffer in order", "[FW-011][CTL-004]")
{
    mechsim_app_init();
    mechsim_board_inputs inputs{};
    fill_rx(inputs, 1U);
    require_echo(run_tick(inputs), 1U);
    fill_rx(inputs, 3U);
    require_echo(run_tick(inputs), 3U);
}

TEST_CASE("UART echo returns a full buffer", "[FW-011][CTL-004]")
{
    mechsim_app_init();
    mechsim_board_inputs inputs{};
    fill_rx(inputs, MECHSIM_UART_RX_BYTES);
    require_echo(run_tick(inputs), MECHSIM_UART_RX_BYTES);
}

TEST_CASE("UART echo clamps a receive count beyond the buffer", "[FW-011][CTL-004]")
{
    mechsim_app_init();
    mechsim_board_inputs inputs{};
    fill_rx(inputs, MECHSIM_UART_RX_BYTES + 1U);
    require_echo(run_tick(inputs), MECHSIM_UART_TX_BYTES);
    fill_rx(inputs, UINT16_MAX);
    require_echo(run_tick(inputs), MECHSIM_UART_TX_BYTES);
}

TEST_CASE("UART bytes are echoed only on the tick that received them", "[FW-011][CTL-004]")
{
    mechsim_app_init();
    mechsim_board_inputs inputs{};
    fill_rx(inputs, 5U);
    require_echo(run_tick(inputs), 5U);
    fill_rx(inputs, 0U);
    require_echo(run_tick(inputs), 0U);
}

TEST_CASE("event log carries one tick event per tick numbered from init", "[FW-010][CTL-018]")
{
    mechsim_app_init();
    const mechsim_board_inputs inputs{};
    for (std::uint16_t tick = 1U; tick <= 3U; tick = static_cast<std::uint16_t>(tick + 1U)) {
        const mechsim_board_outputs outputs = run_tick(inputs);
        REQUIRE(outputs.event_count == 1U);
        REQUIRE(outputs.event_log[0].code == event_tick);
        REQUIRE(outputs.event_log[0].arg == tick);
        REQUIRE(outputs.fault_code == fault_none);
    }
}

TEST_CASE("tick event argument wraps at the uint16_t limit", "[FW-004][CTL-018]")
{
    mechsim_app_init();
    const mechsim_board_inputs inputs{};
    for (std::uint32_t tick = 1U; tick < 65535U; ++tick) {
        run_tick(inputs);
    }
    REQUIRE(run_tick(inputs).event_log[0].arg == 65535U);
    REQUIRE(run_tick(inputs).event_log[0].arg == 0U);
    REQUIRE(run_tick(inputs).event_log[0].arg == 1U);
}

TEST_CASE("event log fills to 16 and rejects the overflow", "[FW-010][CTL-018]")
{
    mechsim_app_init();
    mechsim_board_inputs inputs{};

    fill_rx(inputs, 14U);
    mechsim_board_outputs outputs = run_tick(inputs);
    REQUIRE(outputs.event_count == 15U);
    REQUIRE(outputs.fault_code == fault_none);

    fill_rx(inputs, 15U);
    outputs = run_tick(inputs);
    REQUIRE(outputs.event_count == MECHSIM_EVENT_LOG_COUNT);
    REQUIRE(outputs.fault_code == fault_none);
    REQUIRE(outputs.event_log[0].code == event_tick);
    for (std::uint32_t index = 1U; index < MECHSIM_EVENT_LOG_COUNT; ++index) {
        REQUIRE(outputs.event_log[index].code == event_uart_rx);
        REQUIRE(outputs.event_log[index].arg == index);
    }

    fill_rx(inputs, 16U);
    outputs = run_tick(inputs);
    REQUIRE(outputs.event_count == MECHSIM_EVENT_LOG_COUNT);
    REQUIRE(outputs.fault_code == fault_event_log_full);
    REQUIRE(outputs.event_log[15].arg == 15U);
    require_echo(outputs, 16U);

    fill_rx(inputs, MECHSIM_UART_RX_BYTES);
    outputs = run_tick(inputs);
    REQUIRE(outputs.event_count == MECHSIM_EVENT_LOG_COUNT);
    REQUIRE(outputs.fault_code == fault_event_log_full);
    require_echo(outputs, MECHSIM_UART_RX_BYTES);

    fill_rx(inputs, 0U);
    outputs = run_tick(inputs);
    REQUIRE(outputs.event_count == 1U);
    REQUIRE(outputs.fault_code == fault_none);
}

TEST_CASE("tick reads inputs without modifying them and is deterministic", "[FW-011][CTL-003][CTL-004]")
{
    mechsim_board_inputs inputs{};
    for (std::uint32_t pin = 0U; pin < MECHSIM_DIGITAL_IN_COUNT; ++pin) {
        inputs.digital_in[pin] = MECHSIM_LEVEL_HIGH;
    }
    for (std::uint32_t channel = 0U; channel < MECHSIM_ADC_COUNT; ++channel) {
        inputs.adc_sample[channel] = UINT16_MAX;
    }
    fill_rx(inputs, 7U);
    std::array<std::int32_t, 3> extremes{INT32_MIN, 0, INT32_MAX};

    std::array<mechsim_board_outputs, 3> first{};
    std::array<mechsim_board_outputs, 3> second{};
    for (std::array<mechsim_board_outputs, 3>* run : {&first, &second}) {
        mechsim_app_init();
        for (std::size_t tick = 0U; tick < extremes.size(); ++tick) {
            for (std::uint32_t channel = 0U; channel < MECHSIM_COUNTER_COUNT; ++channel) {
                inputs.counter_delta[channel] = extremes[tick];
            }
            const mechsim_board_inputs snapshot = inputs;
            (*run)[tick] = run_tick(inputs);
            REQUIRE(std::memcmp(&inputs, &snapshot, sizeof(inputs)) == 0);
        }
    }

    REQUIRE(std::memcmp(first.data(), second.data(), sizeof(first)) == 0);
    for (const mechsim_board_outputs& outputs : first) {
        require_echo(outputs, 7U);
        REQUIRE(outputs.step_count[0] == 0U);
    }
}
