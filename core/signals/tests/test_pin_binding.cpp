#include <catch2/catch_test_macros.hpp>
#include <string>

#include "mechsim/signals/pin_binding.hpp"
#include "mechsim/signals/power_rail.hpp"

using mechsim::signals::PeripheralMode;
using mechsim::signals::PinBinding;
using mechsim::signals::PinBindingError;
using mechsim::signals::PinBindingTable;
using mechsim::signals::PowerRail;

TEST_CASE("The pin-binding table maps a dotted signal to one pin and mode, in insertion order", "[SIG-023]") {
    PinBindingTable table;
    table.add(PinBinding{"x_limit.level", "PA0", PeripheralMode::digital_input});
    table.add(PinBinding{"x_drv.step", "PB3", PeripheralMode::pulse_output});
    table.add(PinBinding{"link.rx", "PA10", PeripheralMode::uart_rx});
    REQUIRE(table.entries().size() == 3);
    CHECK(table.entries()[1].signal == "x_drv.step");
    const auto by_signal = table.find_signal("x_drv.step");
    REQUIRE(by_signal.has_value());
    CHECK(by_signal.value_or(PinBinding{}).pin == "PB3");
    CHECK(by_signal.value_or(PinBinding{}).mode == PeripheralMode::pulse_output);
    const auto by_pin = table.find_pin("PA10");
    CHECK(by_pin.value_or(PinBinding{}).signal == "link.rx");
    CHECK_FALSE(table.find_signal("y_limit.level").has_value());
    CHECK_FALSE(table.find_pin("PC0").has_value());
}

TEST_CASE("The pin-binding table rejects a duplicate pin, a duplicate signal and bad entries", "[SIG-023][SIG-024]") {
    PinBindingTable table;
    table.add(PinBinding{"x_limit.level", "PA0", PeripheralMode::digital_input});
    CHECK_THROWS_AS(table.add(PinBinding{"y_limit.level", "PA0", PeripheralMode::digital_input}), PinBindingError);
    CHECK_THROWS_AS(table.add(PinBinding{"y_limit.level", "PA0", PeripheralMode::adc_input}), PinBindingError);
    CHECK_THROWS_AS(table.add(PinBinding{"x_limit.level", "PA1", PeripheralMode::digital_input}), PinBindingError);
    CHECK_THROWS_AS(table.add(PinBinding{"level", "PA2", PeripheralMode::digital_input}), PinBindingError);
    CHECK_THROWS_AS(table.add(PinBinding{"a.b.c", "PA2", PeripheralMode::digital_input}), PinBindingError);
    CHECK_THROWS_AS(table.add(PinBinding{"y_limit.level", "", PeripheralMode::digital_input}), PinBindingError);
    CHECK(table.entries().size() == 1);
}

TEST_CASE("A power rail carries its name, voltage and on state", "[SIG-015]") {
    const PowerRail rail{"motor", 24.0, true};
    CHECK(rail.name == "motor");
    CHECK(rail.voltage == 24.0);
    CHECK(rail.on);
    const PowerRail off{};
    CHECK_FALSE(off.on);
    CHECK(off.voltage == 0.0);
}
