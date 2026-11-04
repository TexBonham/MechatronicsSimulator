#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>

#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/power_rail.hpp"

using mechsim::controller::ControllerConfigError;
using mechsim::controller::ControllerError;
using mechsim::controller::ControllerRuntime;
using mechsim::controller::FirmwareImage;
using mechsim::controller::NullControllerRuntime;
using mechsim::controller::PinInputs;
using mechsim::controller::PinOutputs;
using mechsim::kernel::Rate;
using mechsim::signals::PowerRail;

namespace controller = mechsim::controller;

static_assert(std::is_abstract_v<ControllerRuntime>);
static_assert(std::is_base_of_v<ControllerRuntime, NullControllerRuntime>);
static_assert(std::is_trivially_copyable_v<PinInputs>);
static_assert(std::is_trivially_copyable_v<PinOutputs>);

// The pin contract widths shared with the board interface C header (brief C6).
static_assert(std::tuple_size_v<decltype(PinInputs::digital_in)> == 32);
static_assert(std::tuple_size_v<decltype(PinInputs::counter_delta)> == 8);
static_assert(std::tuple_size_v<decltype(PinInputs::adc_sample)> == 16);
static_assert(std::tuple_size_v<decltype(PinInputs::uart_rx)> == 64);
static_assert(std::is_same_v<decltype(PinInputs::counter_delta)::value_type, std::int32_t>);
static_assert(std::is_same_v<decltype(PinInputs::adc_sample)::value_type, std::uint16_t>);
static_assert(std::is_same_v<decltype(PinInputs::uart_rx_count), std::uint16_t>);
static_assert(std::tuple_size_v<decltype(PinOutputs::step_count)> == 8);
static_assert(std::is_same_v<decltype(PinOutputs::step_count)::value_type, std::uint32_t>);
static_assert(std::tuple_size_v<decltype(PinOutputs::direction)> == 8);
static_assert(std::tuple_size_v<decltype(PinOutputs::enable)> == 8);
static_assert(std::tuple_size_v<decltype(PinOutputs::digital_out)> == 32);
static_assert(std::tuple_size_v<decltype(PinOutputs::uart_tx)> == 64);
static_assert(std::is_same_v<decltype(PinOutputs::uart_tx_count), std::uint16_t>);
static_assert(std::is_same_v<decltype(PinOutputs::state_code), std::uint8_t>);
static_assert(std::is_same_v<decltype(PinOutputs::fault_code), std::uint8_t>);
static_assert(std::tuple_size_v<decltype(PinOutputs::event_log)> == 16);
static_assert(std::is_same_v<decltype(controller::EventLogEntry::code), std::uint8_t>);
static_assert(std::is_same_v<decltype(controller::EventLogEntry::arg), std::uint16_t>);
static_assert(std::is_same_v<decltype(PinOutputs::event_count), std::uint8_t>);

namespace {

const PowerRail logic_rail{"logic", 3.3, true};

std::unique_ptr<NullControllerRuntime> ready_runtime() {
    auto runtime = std::make_unique<NullControllerRuntime>(Rate(1'000));
    runtime->load(FirmwareImage{"build/host/firmware", 1'000});
    runtime->reset(logic_rail);
    return runtime;
}

}  // namespace

TEST_CASE("The null runtime loads, resets with the rail up and ticks through the base interface",
          "[CTL-001][CTL-018]") {
    auto runtime = ready_runtime();
    CHECK(runtime->firmware_path() == "build/host/firmware");
    CHECK(runtime->resets() == 1);
    CHECK(runtime->supply_voltage() == 3.3);
    ControllerRuntime& base = *runtime;
    PinInputs inputs;
    inputs.digital_in[0] = 1;
    inputs.counter_delta[2] = 42;
    PinOutputs outputs;
    outputs.step_count[0] = 7;
    outputs.uart_tx_count = 1;
    outputs.digital_out[0] = 1;
    outputs.event_count = 2;
    outputs.fault_code = 9;
    base.tick(inputs, outputs);
    CHECK(runtime->ticks_run() == 1);
    CHECK(outputs.step_count[0] == 0);
    CHECK(outputs.uart_tx_count == 0);
    CHECK(outputs.digital_out[0] == 0);
    CHECK(outputs.event_count == 0);
    CHECK(outputs.fault_code == 0);
}

TEST_CASE("Load rejects a firmware whose declared tick rate differs from the control rate", "[CTL-002]") {
    NullControllerRuntime runtime(Rate(1'000));
    CHECK_THROWS_AS(runtime.load(FirmwareImage{"fw", 500}), ControllerConfigError);
    CHECK_THROWS_AS(runtime.load(FirmwareImage{"fw", 0}), ControllerConfigError);
    CHECK_THROWS_AS(runtime.load(FirmwareImage{"", 1'000}), ControllerConfigError);
    NullControllerRuntime sub_hertz(Rate(1, 2));
    CHECK_THROWS_AS(sub_hertz.load(FirmwareImage{"fw", 1}), ControllerConfigError);
    runtime.load(FirmwareImage{"fw", 1'000});
    CHECK(runtime.firmware_path() == "fw");
}

TEST_CASE("The null runtime refuses to run out of order", "[CTL-012]") {
    NullControllerRuntime runtime(Rate(1'000));
    PinOutputs outputs;
    CHECK_THROWS_AS(runtime.reset(logic_rail), ControllerError);
    runtime.load(FirmwareImage{"fw", 1'000});
    CHECK_THROWS_AS(runtime.tick(PinInputs{}, outputs), ControllerError);
}

// 64 bytes is the contract buffer; 65 in one tick is an overflow and a run error, never a truncation.
TEST_CASE("A UART receive count above the buffer is an error", "[CTL-003]") {
    auto runtime = ready_runtime();
    PinOutputs outputs;
    PinInputs full;
    full.uart_rx_count = 64;
    runtime->tick(full, outputs);
    PinInputs over;
    over.uart_rx_count = 65;
    CHECK_THROWS_AS(runtime->tick(over, outputs), ControllerError);
    CHECK(runtime->ticks_run() == 1);
}

TEST_CASE("A frozen runtime runs no tick and outputs nothing until thawed", "[CTL-011]") {
    auto runtime = ready_runtime();
    PinOutputs outputs;
    runtime->freeze(true);
    CHECK(runtime->frozen());
    outputs.digital_out[0] = 1;
    runtime->tick(PinInputs{}, outputs);
    CHECK(runtime->ticks_run() == 0);
    CHECK(outputs.digital_out[0] == 0);
    runtime->freeze(false);
    runtime->tick(PinInputs{}, outputs);
    CHECK(runtime->ticks_run() == 1);
}

TEST_CASE("Brownout takes the supply voltage and rejects non-physical values", "[CTL-013]") {
    auto runtime = ready_runtime();
    runtime->brownout(1.2);
    CHECK(runtime->supply_voltage() == 1.2);
    runtime->brownout(0.0);
    CHECK(runtime->supply_voltage() == 0.0);
    CHECK_THROWS_AS(runtime->brownout(-0.1), std::invalid_argument);
    CHECK_THROWS_AS(runtime->brownout(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
    CHECK(runtime->supply_voltage() == 0.0);
}
