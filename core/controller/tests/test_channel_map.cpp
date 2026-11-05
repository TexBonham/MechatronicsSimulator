#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "mechsim/controller/channel_map.hpp"
#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/controller/native_controller_runtime.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/pin_binding.hpp"

using mechsim::controller::AxisChannels;
using mechsim::controller::ChannelAssignment;
using mechsim::controller::ChannelMap;
using mechsim::controller::ControllerConfigError;
using mechsim::controller::PinSlot;
using mechsim::signals::PeripheralMode;
using mechsim::signals::PinBinding;
using mechsim::signals::PinBindingTable;

namespace {

// A hand-written table in the shape the builder produces: one assignment per bound signal.
ChannelMap hand_table() {
    ChannelMap map;
    map.add({"limit_x.level", PinSlot::digital_in, 0});
    map.add({"encoder_x.count", PinSlot::counter_delta, 1});
    map.add({"current_x.code", PinSlot::adc_sample, 15});
    map.add({"operator.rx", PinSlot::uart_rx, 0});
    map.add({"driver_x.step", PinSlot::step_count, 7});
    map.add({"driver_x.dir", PinSlot::digital_out, 6});
    map.add({"driver_x.enable", PinSlot::digital_out, 7});
    map.add({"lamp.level", PinSlot::digital_out, 31});
    map.add({"operator.tx", PinSlot::uart_tx, 0});
    map.assign_axis(7, {6, 7});
    return map;
}

PinBindingTable hand_bindings() {
    PinBindingTable table;
    table.add(PinBinding{"limit_x.level", "PA0", PeripheralMode::digital_input});
    table.add(PinBinding{"encoder_x.count", "PA8", PeripheralMode::counter_input});
    table.add(PinBinding{"current_x.code", "PC0", PeripheralMode::adc_input});
    table.add(PinBinding{"operator.rx", "PA10", PeripheralMode::uart_rx});
    table.add(PinBinding{"driver_x.step", "PB0", PeripheralMode::pulse_output});
    table.add(PinBinding{"driver_x.dir", "PB1", PeripheralMode::digital_output});
    table.add(PinBinding{"driver_x.enable", "PB2", PeripheralMode::digital_output});
    table.add(PinBinding{"lamp.level", "PB3", PeripheralMode::digital_output});
    table.add(PinBinding{"operator.tx", "PA9", PeripheralMode::uart_tx});
    return table;
}

}  // namespace

TEST_CASE("Slot capacities equal the pin contract arrays; each UART direction is one stream", "[CTL-006]") {
    CHECK(slot_capacity(PinSlot::digital_in) == 32);
    CHECK(slot_capacity(PinSlot::counter_delta) == 8);
    CHECK(slot_capacity(PinSlot::adc_sample) == 16);
    CHECK(slot_capacity(PinSlot::uart_rx) == 1);
    CHECK(slot_capacity(PinSlot::step_count) == 8);
    CHECK(slot_capacity(PinSlot::digital_out) == 32);
    CHECK(slot_capacity(PinSlot::uart_tx) == 1);
}

TEST_CASE("Each slot accepts exactly its peripheral modes", "[CTL-006]") {
    const std::vector<PeripheralMode> modes{PeripheralMode::digital_input, PeripheralMode::digital_output,
                                            PeripheralMode::counter_input, PeripheralMode::adc_input,
                                            PeripheralMode::pulse_output,  PeripheralMode::uart_rx,
                                            PeripheralMode::uart_tx};
    const std::vector<std::pair<PinSlot, std::vector<PeripheralMode>>> accepted{
        {PinSlot::digital_in, {PeripheralMode::digital_input}},
        {PinSlot::counter_delta, {PeripheralMode::counter_input}},
        {PinSlot::adc_sample, {PeripheralMode::adc_input}},
        {PinSlot::uart_rx, {PeripheralMode::uart_rx}},
        {PinSlot::step_count, {PeripheralMode::pulse_output}},
        {PinSlot::digital_out, {PeripheralMode::digital_output}},
        {PinSlot::uart_tx, {PeripheralMode::uart_tx}},
    };
    for (const auto& [slot, allowed] : accepted) {
        for (const PeripheralMode mode : modes) {
            bool expected = false;
            for (const PeripheralMode ok : allowed) {
                expected = expected || ok == mode;
            }
            CHECK(slot_accepts(slot, mode) == expected);
        }
    }
}

TEST_CASE("The channel map finds each signal's slot and channel in insertion order", "[CTL-006]") {
    const ChannelMap map = hand_table();
    REQUIRE(map.entries().size() == 9);
    CHECK(map.entries().front().signal == "limit_x.level");
    CHECK(map.entries().back().signal == "operator.tx");
    const std::optional<ChannelAssignment> found = map.find("current_x.code");
    CHECK(found.has_value());
    CHECK(found.value_or(ChannelAssignment{}).slot == PinSlot::adc_sample);
    CHECK(found.value_or(ChannelAssignment{}).channel == 15);
    CHECK_FALSE(map.find("absent.signal").has_value());
}

TEST_CASE("The channel map rejects bad assignments", "[CTL-006]") {
    ChannelMap map = hand_table();
    CHECK_THROWS_AS(map.add({"", PinSlot::digital_in, 1}), ControllerConfigError);
    CHECK_THROWS_AS(map.add({"extra.level", PinSlot::digital_in, 32}), ControllerConfigError);
    CHECK_THROWS_AS(map.add({"extra.code", PinSlot::adc_sample, 16}), ControllerConfigError);
    CHECK_THROWS_AS(map.add({"extra.rx", PinSlot::uart_rx, 1}), ControllerConfigError);
    CHECK_THROWS_AS(map.add({"extra.step", PinSlot::step_count, 8}), ControllerConfigError);
    CHECK_THROWS_AS(map.add({"limit_x.level", PinSlot::digital_in, 1}), ControllerConfigError);
    CHECK_THROWS_AS(map.add({"extra.level", PinSlot::digital_in, 0}), ControllerConfigError);
    CHECK(map.entries().size() == 9);
    map.add({"extra.level", PinSlot::digital_in, 31});
    CHECK(map.entries().size() == 10);
}

TEST_CASE("The channel map is checked against the pin-binding table", "[CTL-006]") {
    const PinBindingTable bindings = hand_bindings();
    CHECK_NOTHROW(hand_table().check_bindings(bindings));

    SECTION("a bound signal without an assignment") {
        PinBindingTable more = hand_bindings();
        more.add(PinBinding{"limit_y.level", "PA1", PeripheralMode::digital_input});
        CHECK_THROWS_AS(hand_table().check_bindings(more), ControllerConfigError);
    }
    SECTION("an assignment whose slot does not accept the bound mode") {
        ChannelMap wrong;
        wrong.add({"limit_x.level", PinSlot::digital_out, 0});
        PinBindingTable one;
        one.add(PinBinding{"limit_x.level", "PA0", PeripheralMode::digital_input});
        CHECK_THROWS_AS(wrong.check_bindings(one), ControllerConfigError);
    }
    SECTION("an assignment for a signal no pin binds") {
        ChannelMap extra = hand_table();
        extra.add({"spare.level", PinSlot::digital_in, 5});
        CHECK_THROWS_AS(extra.check_bindings(bindings), ControllerConfigError);
    }
}

TEST_CASE("The native runtime exposes the builder's channel table", "[CTL-006]") {
    mechsim::controller::NativeRuntimeConfig config;
    config.control_rate = mechsim::kernel::Rate(1'000);
    config.work_dir = "unused";
    config.channels = hand_table();
    const mechsim::controller::NativeControllerRuntime runtime(std::move(config));
    REQUIRE(runtime.channel_map().entries().size() == 9);
    const std::optional<ChannelAssignment> dir = runtime.channel_map().find("driver_x.dir");
    CHECK(dir.value_or(ChannelAssignment{}).channel == 6);
    CHECK(runtime.channel_map().axis(7) == std::optional<AxisChannels>(AxisChannels{6, 7}));
    CHECK(runtime.held_in_reset());
    CHECK_FALSE(runtime.frozen());
}

TEST_CASE("Stepper axes map to bound digital_out channels for direction and enable", "[CTL-006]") {
    ChannelMap map = hand_table();
    CHECK(map.axis(7) == std::optional<AxisChannels>(AxisChannels{6, 7}));
    CHECK_FALSE(map.axis(0).has_value());
    CHECK_FALSE(map.axis(8).has_value());
    CHECK_THROWS_AS(map.assign_axis(8, {6, 7}), ControllerConfigError);
    CHECK_THROWS_AS(map.assign_axis(7, {6, 31}), ControllerConfigError);
    CHECK_THROWS_AS(map.assign_axis(0, {6, 6}), ControllerConfigError);
    CHECK_THROWS_AS(map.assign_axis(0, {6, 5}), ControllerConfigError);
    CHECK_THROWS_AS(map.assign_axis(0, {0, 31}), ControllerConfigError);
    map.assign_axis(0, {31, 7});
    CHECK(map.axis(0) == std::optional<AxisChannels>(AxisChannels{31, 7}));
}

TEST_CASE("Direction and enable are derived from digital_out; unmapped axes read low", "[CTL-006]") {
    const ChannelMap map = hand_table();
    mechsim::controller::PinOutputs outputs;
    outputs.digital_out.at(6) = 1;
    outputs.digital_out.at(7) = 0;
    outputs.direction.fill(1);
    outputs.enable.fill(1);
    derive_axis_levels(map, outputs);
    for (std::size_t axis = 0; axis < 7; ++axis) {
        CHECK(outputs.direction.at(axis) == 0U);
        CHECK(outputs.enable.at(axis) == 0U);
    }
    CHECK(outputs.direction.at(7) == 1U);
    CHECK(outputs.enable.at(7) == 0U);
    outputs.digital_out.at(7) = 1;
    derive_axis_levels(map, outputs);
    CHECK(outputs.enable.at(7) == 1U);
}
