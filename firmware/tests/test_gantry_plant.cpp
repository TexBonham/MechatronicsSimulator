// Simulated gantry plant for the firmware-level scenarios; included by test_gantry.cpp and test_gantry_variant.cpp.
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#include "mechsim_board.h"
}
#include "channels.h"
#include "gantry_config.h"

namespace gantry_test {

constexpr std::int64_t steps_per_m = 80000;
constexpr std::int64_t counts_per_m = 60000;
constexpr std::int64_t workspace_x_steps = 80000;
constexpr std::int64_t workspace_y_steps = 64000;
constexpr std::int64_t max_switch_margin_steps = 80;

struct Event {
    std::uint8_t code;
    std::uint16_t arg;
};

std::int64_t counts_of(std::int64_t steps)
{
    std::int64_t scaled = steps * counts_per_m;
    if (scaled < 0) {
        return -((-scaled + steps_per_m - 1) / steps_per_m);
    }
    return scaled / steps_per_m;
}

// Integrates step pulses into true position, emits encoder deltas on channel A and trips the switches at the ends.
class Plant {
public:
    Plant(std::int64_t x_steps, std::int64_t y_steps)
    {
        position[0] = x_steps;
        position[1] = y_steps;
        mechsim_app_init();
        std::memset(&last, 0, sizeof(last));
    }

    mechsim_board_outputs tick(const std::string& rx = std::string())
    {
        mechsim_board_inputs inputs{};
        REQUIRE(rx.size() <= MECHSIM_UART_RX_BYTES);
        std::memcpy(inputs.uart_rx, rx.data(), rx.size());
        inputs.uart_rx_count = static_cast<std::uint16_t>(rx.size());
        apply(inputs);
        mechsim_board_outputs outputs{};
        std::memset(&outputs, 0xAA, sizeof(outputs));
        mechsim_app_tick(&inputs, &outputs);
        check_invariants(outputs);
        last = outputs;
        tx.append(reinterpret_cast<const char*>(outputs.uart_tx), outputs.uart_tx_count);
        for (std::uint8_t index = 0U; index < outputs.event_count; ++index) {
            events.push_back(Event{outputs.event_log[index].code, outputs.event_log[index].arg});
        }
        ticks = ticks + 1;
        return outputs;
    }

    template <typename Predicate>
    std::int64_t run_until(Predicate predicate, std::int64_t max_ticks)
    {
        for (std::int64_t count = 1; count <= max_ticks; ++count) {
            if (predicate(tick())) {
                return count;
            }
        }
        return -1;
    }

    std::vector<std::string> take_lines()
    {
        std::vector<std::string> lines;
        std::size_t start = 0U;
        std::size_t end = tx.find('\n', start);
        while (end != std::string::npos) {
            lines.push_back(tx.substr(start, end - start));
            start = end + 1U;
            end = tx.find('\n', start);
        }
        tx = tx.substr(start);
        return lines;
    }

    std::vector<Event> take_events()
    {
        std::vector<Event> taken = events;
        events.clear();
        return taken;
    }

    void home()
    {
        tick("home\n");
        REQUIRE(run_until([](const mechsim_board_outputs& out) { return out.state_code == GANTRY_STATE_READY; },
                    200000)
            > 0);
        take_lines();
        take_events();
    }

    std::int64_t position[2]{};
    std::int64_t extra_counts[2]{};
    bool switches_work = true;
    bool on_min_switch_override = false;
    std::int64_t ticks = 0;
    std::string tx;
    std::vector<Event> events;
    mechsim_board_outputs last{};

private:
    void apply(mechsim_board_inputs& inputs)
    {
        static constexpr std::uint32_t step_channel[2] = {MECHSIM_CH_X_DRIVER_STEP, MECHSIM_CH_Y_DRIVER_STEP};
        static constexpr std::uint32_t dir_channel[2] = {MECHSIM_CH_X_DRIVER_DIR, MECHSIM_CH_Y_DRIVER_DIR};
        static constexpr std::uint32_t enc_channel[2] = {MECHSIM_CH_X_ENCODER_CHANNEL_A, MECHSIM_CH_Y_ENCODER_CHANNEL_A};
        static constexpr std::uint32_t min_channel[2] = {MECHSIM_CH_X_MIN_SWITCH_OUT, MECHSIM_CH_Y_MIN_SWITCH_OUT};
        static constexpr std::uint32_t max_channel[2] = {MECHSIM_CH_X_MAX_SWITCH_OUT, MECHSIM_CH_Y_MAX_SWITCH_OUT};
        static constexpr std::int64_t workspace[2] = {workspace_x_steps, workspace_y_steps};
        for (std::uint32_t axis = 0U; axis < 2U; ++axis) {
            std::int64_t before = counts_of(position[axis]);
            std::int64_t steps = static_cast<std::int64_t>(last.step_count[step_channel[axis]]);
            if (last.digital_out[dir_channel[axis]] == GANTRY_DIR_POSITIVE_LEVEL) {
                position[axis] = position[axis] + steps;
            } else {
                position[axis] = position[axis] - steps;
            }
            inputs.counter_delta[enc_channel[axis]] =
                static_cast<std::int32_t>(counts_of(position[axis]) - before + extra_counts[axis]);
            extra_counts[axis] = 0;
            if (switches_work && ((position[axis] <= 0) || on_min_switch_override)) {
                inputs.digital_in[min_channel[axis]] = GANTRY_SWITCH_ACTIVE_LEVEL;
            }
            if (switches_work && (position[axis] >= workspace[axis] + max_switch_margin_steps)) {
                inputs.digital_in[max_channel[axis]] = GANTRY_SWITCH_ACTIVE_LEVEL;
            }
        }
    }

    static void check_invariants(const mechsim_board_outputs& outputs)
    {
        REQUIRE(outputs.uart_tx_count <= MECHSIM_UART_TX_BYTES);
        REQUIRE(outputs.event_count <= MECHSIM_EVENT_LOG_COUNT);
        for (std::uint32_t index = outputs.uart_tx_count; index < MECHSIM_UART_TX_BYTES; ++index) {
            REQUIRE(outputs.uart_tx[index] == 0U);
        }
        for (std::uint32_t index = outputs.event_count; index < MECHSIM_EVENT_LOG_COUNT; ++index) {
            REQUIRE(outputs.event_log[index].code == 0U);
            REQUIRE(outputs.event_log[index].arg == 0U);
        }
        if (outputs.step_count[MECHSIM_CH_X_DRIVER_STEP] != 0U) {
            REQUIRE(outputs.digital_out[MECHSIM_CH_X_DRIVER_ENABLE] == GANTRY_ENABLE_ACTIVE_LEVEL);
        }
        if (outputs.step_count[MECHSIM_CH_Y_DRIVER_STEP] != 0U) {
            REQUIRE(outputs.digital_out[MECHSIM_CH_Y_DRIVER_ENABLE] == GANTRY_ENABLE_ACTIVE_LEVEL);
        }
        for (std::uint32_t axis = 0U; axis < MECHSIM_AXIS_COUNT; ++axis) {
            REQUIRE(outputs.direction[axis] == MECHSIM_LEVEL_LOW);
            REQUIRE(outputs.enable[axis] == MECHSIM_LEVEL_LOW);
            if ((axis != MECHSIM_CH_X_DRIVER_STEP) && (axis != MECHSIM_CH_Y_DRIVER_STEP)) {
                REQUIRE(outputs.step_count[axis] == 0U);
            }
        }
        for (std::uint32_t pin = 0U; pin < MECHSIM_DIGITAL_OUT_COUNT; ++pin) {
            if ((pin > MECHSIM_CH_WATCHDOG_HEARTBEAT) || (pin == MECHSIM_CH_LIFT_DRIVER_DIR)) {
                REQUIRE(outputs.digital_out[pin] == MECHSIM_LEVEL_LOW);
            }
            if (pin == MECHSIM_CH_LIFT_DRIVER_ENABLE) {
                REQUIRE(outputs.digital_out[pin] == GANTRY_ENABLE_ACTIVE_LEVEL);
            }
        }
    }
};

// Parses "current_position x y" in metres into micrometres.
bool parse_position(const std::string& line, std::int64_t& x_um, std::int64_t& y_um)
{
    if (line.rfind("current_position ", 0) != 0) {
        return false;
    }
    double x = 0.0;
    double y = 0.0;
    if (std::sscanf(line.c_str(), "current_position %lf %lf", &x, &y) != 2) {
        return false;
    }
    x_um = std::llround(x * 1e6);
    y_um = std::llround(y * 1e6);
    return true;
}

std::uint16_t transition(std::uint8_t from, std::uint8_t to)
{
    return static_cast<std::uint16_t>((static_cast<std::uint32_t>(from) << 8U) | to);
}

bool has_event(const std::vector<Event>& events, std::uint8_t code, std::uint16_t arg)
{
    for (const Event& event : events) {
        if ((event.code == code) && (event.arg == arg)) {
            return true;
        }
    }
    return false;
}

}  // namespace gantry_test
