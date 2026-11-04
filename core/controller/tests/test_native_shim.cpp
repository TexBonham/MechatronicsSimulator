// The shim against the mirror test application (fixtures/mirror_app.cpp), whose outputs are fixed functions of its
// inputs: step_count[i] = counter_delta[i] as u32, direction and enable arrays low, digital_out[i] = digital_in[31 - i],
// uart_tx[k] = uart_rx[k] xor 0xFF with the same count, event_log[i] = {i + 1, adc_sample[i]} with 16 entries,
// state_code = ticks since init, fault_code = inits seen by this copy of the library (1 after every reset, since each
// reset reopens a fresh copy). The runtimes here map axis i to digital_out channels 2i (direction) and 2i + 1 (enable),
// so the shim's derived direction[i] = digital_in[31 - 2i] and enable[i] = digital_in[30 - 2i].
#include <catch2/catch_test_macros.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/controller/native_controller_runtime.hpp"
#include "mechsim/signals/power_rail.hpp"
#include "native_test_support.hpp"

using mechsim::controller::ControllerConfigError;
using mechsim::controller::ControllerError;
using mechsim::controller::FirmwareImage;
using mechsim::controller::NativeControllerRuntime;
using mechsim::controller::PinInputs;
using mechsim::controller::PinOutputs;
using mechsim::signals::PowerRail;

namespace {

FirmwareImage mirror() {
    return FirmwareImage{std::string(native_test::mirror_app), 1'000};
}

PinOutputs mirrored(const PinInputs& inputs, std::uint8_t ticks_since_init, std::uint8_t inits) {
    PinOutputs expected;
    for (std::size_t axis = 0; axis < expected.step_count.size(); ++axis) {
        expected.step_count.at(axis) = static_cast<std::uint32_t>(inputs.counter_delta.at(axis));
        expected.direction.at(axis) = inputs.digital_in.at(31U - (2U * axis));
        expected.enable.at(axis) = inputs.digital_in.at(30U - (2U * axis));
    }
    for (std::size_t pin = 0; pin < expected.digital_out.size(); ++pin) {
        expected.digital_out.at(pin) = inputs.digital_in.at(31U - pin);
    }
    for (std::size_t byte = 0; byte < expected.uart_tx.size(); ++byte) {
        expected.uart_tx.at(byte) = static_cast<std::uint8_t>(inputs.uart_rx.at(byte) ^ 0xFFU);
    }
    expected.uart_tx_count = inputs.uart_rx_count;
    for (std::size_t entry = 0; entry < expected.event_log.size(); ++entry) {
        expected.event_log.at(entry) = {static_cast<std::uint8_t>(entry + 1U), inputs.adc_sample.at(entry)};
    }
    expected.event_count = 16;
    expected.state_code = ticks_since_init;
    expected.fault_code = inits;
    return expected;
}

// Every field distinct within its array and from its neighbours, so a swapped or shifted copy shows.
PinInputs distinct_inputs() {
    PinInputs inputs;
    for (std::size_t index = 0; index < inputs.digital_in.size(); ++index) {
        inputs.digital_in.at(index) = static_cast<std::uint8_t>(index + 1U);
    }
    for (std::size_t index = 0; index < inputs.counter_delta.size(); ++index) {
        inputs.counter_delta.at(index) = (static_cast<std::int32_t>(index) * 100'003) - 400'000;
    }
    for (std::size_t index = 0; index < inputs.adc_sample.size(); ++index) {
        inputs.adc_sample.at(index) = static_cast<std::uint16_t>(1'000U + (index * 257U));
    }
    for (std::size_t index = 0; index < inputs.uart_rx.size(); ++index) {
        inputs.uart_rx.at(index) = static_cast<std::uint8_t>(index + 64U);
    }
    inputs.uart_rx_count = 64;
    return inputs;
}

mechsim::controller::NativeRuntimeConfig mapped_config(const std::string& work) {
    mechsim::controller::NativeRuntimeConfig result = native_test::config(work);
    for (std::size_t axis = 0; axis < mechsim::controller::stepper_channels; ++axis) {
        const std::string name = "driver_" + std::to_string(axis);
        result.channels.add({name + ".dir", mechsim::controller::PinSlot::digital_out, 2U * axis});
        result.channels.add({name + ".enable", mechsim::controller::PinSlot::digital_out, (2U * axis) + 1U});
        result.channels.assign_axis(axis, {2U * axis, (2U * axis) + 1U});
    }
    return result;
}

std::unique_ptr<NativeControllerRuntime> running_mirror(const std::string& work) {
    auto runtime = std::make_unique<NativeControllerRuntime>(mapped_config(work));
    runtime->load(mirror());
    runtime->reset(native_test::logic_rail);
    return runtime;
}

}  // namespace

TEST_CASE("A tick copies every input field to the firmware and every output field back", "[CTL-003][CTL-005]") {
    auto runtime = running_mirror("mirror-fields");
    const PinInputs inputs = distinct_inputs();
    PinOutputs outputs;
    runtime->tick(inputs, outputs);
    CHECK(native_test::same_outputs(outputs, mirrored(inputs, 1, 1)));
    CHECK(outputs.step_count.at(0) == 0xFFF9'E580U);  // -400000 as two's-complement u32

    for (std::uint32_t seed = 1; seed <= 50; ++seed) {
        const PinInputs pattern = native_test::patterned_inputs(seed);
        runtime->tick(pattern, outputs);
        INFO("seed " << seed);
        CHECK(native_test::same_outputs(outputs, mirrored(pattern, static_cast<std::uint8_t>(seed + 1U), 1)));
    }
}

TEST_CASE("Direction and enable come from the mapped digital_out channels, not the stepper arrays", "[CTL-003][CTL-006]") {
    auto runtime = running_mirror("mirror-axes");
    PinInputs inputs;
    // Enable channel 2a + 1 of every axis is digital_out[2a + 1] = digital_in[30 - 2a]; direction only on axis 3.
    for (std::size_t axis = 0; axis < 8; ++axis) {
        inputs.digital_in.at(30U - (2U * axis)) = 1;
    }
    inputs.digital_in.at(31U - 6U) = 1;
    PinOutputs outputs;
    runtime->tick(inputs, outputs);
    for (std::size_t axis = 0; axis < 8; ++axis) {
        INFO("axis " << axis);
        CHECK(outputs.enable.at(axis) == 1U);
        std::uint8_t direction = 0;
        if (axis == 3U) {
            direction = 1;
        }
        CHECK(outputs.direction.at(axis) == direction);
    }

    NativeControllerRuntime unmapped(native_test::config("mirror-unmapped"));
    unmapped.load(mirror());
    unmapped.reset(native_test::logic_rail);
    unmapped.tick(inputs, outputs);
    CHECK(outputs.enable == std::array<std::uint8_t, 8>{});
    CHECK(outputs.direction == std::array<std::uint8_t, 8>{});
}

TEST_CASE("Counters reach the firmware as this tick's counts, never accumulated by the runtime", "[CTL-004]") {
    auto runtime = running_mirror("mirror-counters");
    PinInputs inputs;
    PinOutputs outputs;
    inputs.counter_delta.at(2) = 3;
    runtime->tick(inputs, outputs);
    CHECK(outputs.step_count.at(2) == 3U);
    inputs.counter_delta.at(2) = 0;
    runtime->tick(inputs, outputs);
    CHECK(outputs.step_count.at(2) == 0U);
    inputs.counter_delta.at(2) = -5;
    runtime->tick(inputs, outputs);
    CHECK(outputs.step_count.at(2) == 0xFFFF'FFFBU);
    inputs.digital_in.at(31U - 8U) = 1;
    runtime->tick(inputs, outputs);
    CHECK(outputs.direction.at(4) == 1U);
    runtime->tick(inputs, outputs);
    CHECK(outputs.direction.at(4) == 1U);
    CHECK(runtime->ticks_run() == 5);
}

TEST_CASE("A library built against another board interface version is rejected", "[CTL-003]") {
    NativeControllerRuntime runtime(native_test::config("mirror-version"));
    const std::uint32_t version = mechsim::controller::board_interface_version();
    CHECK(version == (2U * 65'536U) + 2U);
    CHECK_THROWS_AS(runtime.load(FirmwareImage{std::string(native_test::mirror_bad_version), 1'000}),
                    ControllerConfigError);
    CHECK_THROWS_AS(runtime.load(FirmwareImage{std::string(native_test::mirror_no_version), 1'000}),
                    ControllerConfigError);
    CHECK_THROWS_AS(runtime.reset(native_test::logic_rail), ControllerError);
}

TEST_CASE("A file that is not a loadable library is a configuration error", "[CTL-003]") {
    const std::filesystem::path junk = native_test::work_dir("junk") / "not_a_library.so";
    std::filesystem::create_directories(junk.parent_path());
    std::ofstream(junk) << "not an ELF file\n";
    NativeControllerRuntime runtime(native_test::config("junk"));
    CHECK_THROWS_AS(runtime.load(FirmwareImage{junk.string(), 1'000}), ControllerConfigError);
}

TEST_CASE("A mismatched declared rate from a prebuilt library is rejected", "[CTL-002]") {
    NativeControllerRuntime runtime(native_test::config("mirror-250"));
    CHECK_THROWS_AS(runtime.load(FirmwareImage{std::string(native_test::mirror_250hz), 1'000}), ControllerConfigError);
    NativeControllerRuntime slow(native_test::config("mirror-250", 250));
    slow.load(FirmwareImage{std::string(native_test::mirror_250hz), 250});
    slow.reset(native_test::logic_rail);
    PinOutputs outputs;
    slow.tick(PinInputs{}, outputs);
    CHECK(outputs.state_code == 1U);
}

TEST_CASE("Init runs only with the rail at or above the brownout threshold", "[CTL-012][CTL-013]") {
    NativeControllerRuntime runtime(mapped_config("mirror-brownout"));
    runtime.load(mirror());
    PinOutputs outputs;
    const PinInputs inputs = distinct_inputs();

    runtime.reset(PowerRail{"logic", 2.0, true});
    CHECK(runtime.held_in_reset());
    CHECK(runtime.inits() == 0);
    runtime.tick(inputs, outputs);
    CHECK(native_test::same_outputs(outputs, PinOutputs{}));
    CHECK(runtime.ticks_run() == 0);

    runtime.brownout(native_test::brownout_threshold_v);
    CHECK_FALSE(runtime.held_in_reset());
    CHECK(runtime.inits() == 1);
    runtime.tick(inputs, outputs);
    runtime.tick(inputs, outputs);
    CHECK(native_test::same_outputs(outputs, mirrored(inputs, 2, 1)));

    runtime.brownout(3.3);
    CHECK(runtime.inits() == 1);

    runtime.brownout(1.0);
    CHECK(runtime.held_in_reset());
    runtime.tick(inputs, outputs);
    CHECK(native_test::same_outputs(outputs, PinOutputs{}));
    runtime.brownout(2.0);
    CHECK(runtime.inits() == 1);

    runtime.brownout(3.3);
    CHECK(runtime.inits() == 2);
    runtime.tick(inputs, outputs);
    CHECK(native_test::same_outputs(outputs, mirrored(inputs, 1, 1)));
    CHECK(runtime.ticks_run() == 3);
    CHECK(runtime.supply_voltage() == 3.3);

    runtime.reset(PowerRail{"logic", 3.3, false});
    CHECK(runtime.held_in_reset());
    CHECK(runtime.supply_voltage() == 0.0);
    CHECK(runtime.inits() == 2);

    CHECK_THROWS_AS(runtime.brownout(-0.1), std::invalid_argument);
    CHECK_THROWS_AS(runtime.reset(PowerRail{"logic", -1.0, true}), std::invalid_argument);
}

TEST_CASE("A brownout before the first reset only records the voltage", "[CTL-013]") {
    NativeControllerRuntime runtime(native_test::config("mirror-early-brownout"));
    runtime.load(mirror());
    runtime.brownout(1.0);
    runtime.brownout(3.3);
    CHECK(runtime.inits() == 0);
    PinOutputs outputs;
    CHECK_THROWS_AS(runtime.tick(PinInputs{}, outputs), ControllerError);
}

TEST_CASE("A frozen controller is not ticked and its outputs hold", "[CTL-011]") {
    auto runtime = running_mirror("mirror-freeze");
    const PinInputs first = distinct_inputs();
    PinOutputs held;
    runtime->tick(first, held);

    runtime->freeze(true);
    CHECK(runtime->frozen());
    PinOutputs outputs;
    runtime->tick(native_test::patterned_inputs(7), outputs);
    runtime->tick(native_test::patterned_inputs(8), outputs);
    CHECK(runtime->ticks_run() == 1);

    // Levels hold: digital_out, derived direction and enable, state and fault codes.
    CHECK(outputs.digital_out == held.digital_out);
    CHECK(outputs.direction == held.direction);
    CHECK(outputs.enable == held.enable);
    CHECK(outputs.state_code == held.state_code);
    CHECK(outputs.fault_code == held.fault_code);
    CHECK(held.enable != std::array<std::uint8_t, 8>{});
    // Per-tick buffers empty: no step pulses, UART bytes or events while frozen (held.* were non-empty).
    CHECK(held.step_count != std::array<std::uint32_t, 8>{});
    CHECK(held.uart_tx_count == 64U);
    CHECK(held.event_count == 16U);
    CHECK(outputs.step_count == std::array<std::uint32_t, 8>{});
    CHECK(outputs.uart_tx_count == 0U);
    CHECK(outputs.event_count == 0U);
    CHECK(native_test::same_outputs(outputs, mechsim::controller::held_levels(held)));

    runtime->freeze(false);
    const PinInputs later = native_test::patterned_inputs(9);
    runtime->tick(later, outputs);
    CHECK(native_test::same_outputs(outputs, mirrored(later, 2, 1)));
    CHECK(runtime->ticks_run() == 2);
}

TEST_CASE("UART and event-log bounds are errors in both directions, never truncation", "[CTL-003]") {
    auto runtime = running_mirror("mirror-overflow");
    PinInputs inputs;
    PinOutputs outputs;

    inputs.uart_rx_count = 64;
    runtime->tick(inputs, outputs);
    CHECK(outputs.uart_tx_count == 64U);

    inputs.uart_rx_count = 65;
    CHECK_THROWS_AS(runtime->tick(inputs, outputs), ControllerError);
    CHECK(runtime->ticks_run() == 1);

    inputs.uart_rx_count = 0;
    inputs.adc_sample.at(15) = 0xFFFF;
    CHECK_THROWS_AS(runtime->tick(inputs, outputs), ControllerError);
    inputs.adc_sample.at(15) = 0xFFFE;
    CHECK_THROWS_AS(runtime->tick(inputs, outputs), ControllerError);

    runtime->freeze(true);
    inputs.uart_rx_count = 65;
    CHECK_THROWS_AS(runtime->tick(inputs, outputs), ControllerError);
}

TEST_CASE("Calls out of order are run errors", "[CTL-012]") {
    NativeControllerRuntime runtime(native_test::config("mirror-order"));
    PinOutputs outputs;
    CHECK_THROWS_AS(runtime.reset(native_test::logic_rail), ControllerError);
    runtime.load(mirror());
    CHECK_THROWS_AS(runtime.tick(PinInputs{}, outputs), ControllerError);
    runtime.reset(native_test::logic_rail);
    runtime.tick(PinInputs{}, outputs);
    CHECK(outputs.state_code == 1U);

    runtime.load(mirror());
    CHECK(runtime.ticks_run() == 0);
    CHECK_THROWS_AS(runtime.tick(PinInputs{}, outputs), ControllerError);
}

TEST_CASE("Runtimes loading one library keep separate firmware state", "[CTL-002]") {
    auto solo = running_mirror("mirror-solo");
    auto first = running_mirror("mirror-shared");
    auto second = running_mirror("mirror-shared");
    for (std::uint32_t tick = 0; tick < 20; ++tick) {
        const PinInputs inputs = native_test::patterned_inputs(tick);
        PinOutputs solo_out;
        PinOutputs first_out;
        PinOutputs second_out;
        solo->tick(inputs, solo_out);
        first->tick(inputs, first_out);
        second->tick(inputs, second_out);
        CHECK(native_test::same_outputs(first_out, solo_out));
        CHECK(native_test::same_outputs(second_out, solo_out));
    }
}

TEST_CASE("A reset reopens the library: same inputs after each reset give identical outputs", "[CTL-012]") {
    auto runtime = running_mirror("mirror-reset-determinism");
    std::vector<std::vector<PinOutputs>> runs;
    for (int run = 0; run < 3; ++run) {
        runtime->reset(native_test::logic_rail);
        std::vector<PinOutputs>& outputs = runs.emplace_back();
        for (std::uint32_t tick = 0; tick < 30; ++tick) {
            runtime->tick(native_test::patterned_inputs(tick), outputs.emplace_back());
        }
        runtime->brownout(1.0);
        runtime->brownout(3.3);
        runtime->tick(native_test::patterned_inputs(0), outputs.emplace_back());
    }
    CHECK(runs.at(0).front().fault_code == 1U);
    CHECK(runs.at(0).back().fault_code == 1U);
    CHECK(runs.at(0).back().state_code == 1U);
    for (std::size_t run = 1; run < runs.size(); ++run) {
        REQUIRE(runs.at(run).size() == runs.at(0).size());
        for (std::size_t tick = 0; tick < runs.at(0).size(); ++tick) {
            INFO("run " << run << " tick " << tick);
            CHECK(native_test::same_outputs(runs.at(run).at(tick), runs.at(0).at(tick)));
        }
    }
}

TEST_CASE("Configuration errors in the runtime settings", "[CTL-013]") {
    mechsim::controller::NativeRuntimeConfig unset = native_test::config("bad");
    unset.brownout_threshold_v.reset();
    CHECK_THROWS_AS(NativeControllerRuntime(unset), ControllerConfigError);
    CHECK(mechsim::controller::NativeRuntimeConfig{}.brownout_threshold_v ==
          mechsim::controller::placeholder_brownout_threshold_v);
    mechsim::controller::NativeRuntimeConfig bad = native_test::config("bad");
    bad.brownout_threshold_v = -1.0;
    CHECK_THROWS_AS(NativeControllerRuntime(bad), ControllerConfigError);
    mechsim::controller::NativeRuntimeConfig no_work = native_test::config("bad");
    no_work.work_dir.clear();
    CHECK_THROWS_AS(NativeControllerRuntime(no_work), ControllerConfigError);
}
