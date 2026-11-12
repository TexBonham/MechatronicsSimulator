// The reference stub (firmware/app/main_tick.c) built through the firmware tree's real host preset.
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/controller/native_controller_runtime.hpp"
#include "native_test_support.hpp"

using mechsim::controller::ControllerConfigError;
using mechsim::controller::FirmwareImage;
using mechsim::controller::NativeControllerRuntime;
using mechsim::controller::PinInputs;
using mechsim::controller::PinOutputs;

namespace fs = std::filesystem;

namespace {

bool throws_config_error_containing(NativeControllerRuntime& runtime, const FirmwareImage& image,
                                    const std::string& fragment) {
    try {
        runtime.load(image);
    } catch (const ControllerConfigError& error) {
        return std::string(error.what()).find(fragment) != std::string::npos;
    }
    return false;
}

std::uint8_t rx_byte(std::size_t tick, std::size_t index) {
    return static_cast<std::uint8_t>(((tick * 7U) + (index * 3U)) & 0xFFU);
}

// Inputs for tick `tick` (1-based since init) with `count` received bytes; every other field holds a distinct value
// the stub must ignore.
PinInputs stub_inputs(std::size_t tick, std::uint16_t count) {
    PinInputs inputs;
    for (std::size_t index = 0; index < inputs.digital_in.size(); ++index) {
        inputs.digital_in.at(index) = static_cast<std::uint8_t>((index + tick) % 2U);
    }
    for (std::size_t index = 0; index < inputs.counter_delta.size(); ++index) {
        inputs.counter_delta.at(index) = static_cast<std::int32_t>(index) - 4;
    }
    for (std::size_t index = 0; index < inputs.adc_sample.size(); ++index) {
        inputs.adc_sample.at(index) = static_cast<std::uint16_t>(100U + index);
    }
    for (std::size_t index = 0; index < inputs.uart_rx.size(); ++index) {
        inputs.uart_rx.at(index) = rx_byte(tick, index);
    }
    inputs.uart_rx_count = count;
    return inputs;
}

// Expected stub outputs, from main_tick.c: mode advances first, so tick n is READY (2) from n = 10
// (MECHSIM_APP_READY_TICKS) and NOT_HOMED (1) before; the heartbeat starts LOW and toggles before it is written, so
// digital_out[0] is n mod 2; event 0 is {1, n}, then one {2, byte} per echoed byte while fewer than 16 entries, and
// fault 1 once a byte finds the log full; the echo copies min(count, 64) bytes; every other output is 0.
PinOutputs expected_stub_outputs(std::size_t tick, std::uint16_t count) {
    PinOutputs expected;
    expected.digital_out.at(0) = static_cast<std::uint8_t>(tick % 2U);
    expected.state_code = 1;
    if (tick >= 10U) {
        expected.state_code = 2;
    }
    expected.event_log.at(0) = {1, static_cast<std::uint16_t>(tick)};
    std::size_t events = 1;
    for (std::size_t index = 0; index < count; ++index) {
        expected.uart_tx.at(index) = rx_byte(tick, index);
        if (events < expected.event_log.size()) {
            expected.event_log.at(events) = {2, rx_byte(tick, index)};
            ++events;
        } else {
            expected.fault_code = 1;
        }
    }
    expected.uart_tx_count = count;
    expected.event_count = static_cast<std::uint8_t>(events);
    return expected;
}

}  // namespace

TEST_CASE("The runtime builds the reference stub with the host preset and loads it", "[.native-build][CTL-005][CTL-007]") {
    const fs::path work = native_test::reference_work_dir();
    fs::remove_all(work);
    NativeControllerRuntime runtime(native_test::config("reference"));
    runtime.load(FirmwareImage{std::string(native_test::firmware_tree), 1'000});

    CHECK(runtime.library_path() ==
          work / "native-firmware" / "host" / mechsim::controller::native_library_name("mechsim_fw_app"));
    CHECK(fs::is_regular_file(runtime.library_path()));
    CHECK(runtime.build_log().find("--preset host") != std::string::npos);
    CHECK(runtime.build_log().find("-DMECHSIM_APP_TICK_HZ=1000") != std::string::npos);
    const fs::path tree = fs::weakly_canonical(fs::path(native_test::firmware_tree));
    const fs::path built = fs::weakly_canonical(runtime.library_path());
    CHECK(std::mismatch(tree.begin(), tree.end(), built.begin(), built.end()).first != tree.end());

    runtime.reset(native_test::logic_rail);
    PinOutputs outputs;
    runtime.tick(stub_inputs(1, 0), outputs);
    CHECK(native_test::same_outputs(outputs, expected_stub_outputs(1, 0)));
}

TEST_CASE("A failed host build is a configuration error carrying the tool output", "[CTL-007]") {
    const fs::path root = native_test::work_dir("broken");
    fs::remove_all(root);
    fs::create_directories(root / "firmware");
    NativeControllerRuntime runtime(native_test::config("broken-run"));

    SECTION("a tree without the presets file is rejected before any tool runs") {
        CHECK(throws_config_error_containing(runtime, FirmwareImage{(root / "firmware").string(), 1'000},
                                             "no CMakePresets.json"));
    }
    SECTION("a configure failure reports what CMake printed") {
        fs::copy_file(fs::path(native_test::firmware_tree) / "CMakePresets.json", root / "firmware" / "CMakePresets.json");
        fs::create_directories(root / "board_interface");
        fs::copy_file(fs::path(native_test::firmware_tree) / ".." / "board_interface" / "mechsim_board.h",
                      root / "board_interface" / "mechsim_board.h");
        std::ofstream(root / "firmware" / "CMakeLists.txt")
            << "cmake_minimum_required(VERSION 3.28)\nproject(broken LANGUAGES NONE)\n"
            << "message(FATAL_ERROR \"deliberate configure failure\")\n";
        CHECK(throws_config_error_containing(runtime, FirmwareImage{(root / "firmware").string(), 1'000},
                                             "deliberate configure failure"));
    }
    SECTION("a missing firmware path is rejected") {
        CHECK(throws_config_error_containing(runtime, FirmwareImage{(root / "absent").string(), 1'000},
                                             "neither a source tree nor a library"));
    }
}

TEST_CASE("A design tick rate other than the control rate is rejected before the build", "[CTL-002]") {
    const fs::path work = native_test::work_dir("rate-design");
    fs::remove_all(work);
    NativeControllerRuntime runtime(native_test::config("rate-design"));
    CHECK(throws_config_error_containing(runtime, FirmwareImage{std::string(native_test::firmware_tree), 500},
                                         "differs from the control rate"));
    CHECK(throws_config_error_containing(runtime, FirmwareImage{std::string(native_test::firmware_tree), 0},
                                         "differs from the control rate"));
    CHECK_FALSE(fs::exists(work / "native-firmware"));
}

TEST_CASE("A library declaring a tick rate other than the control rate is rejected", "[CTL-002]") {
    NativeControllerRuntime runtime(native_test::config("rate-library", 500));
    CHECK(throws_config_error_containing(runtime, FirmwareImage{native_test::reference_library().string(), 500},
                                         "firmware declares 1000 Hz"));
}

TEST_CASE("The reference stub sees inputs and publishes state, echo and event log each tick",
          "[CTL-002][CTL-003][CTL-012][CTL-018]") {
    NativeControllerRuntime runtime(native_test::config("stub"));
    runtime.load(FirmwareImage{native_test::reference_library().string(), 1'000});
    runtime.reset(native_test::logic_rail);

    // Receive counts per tick: none, 15 (log exactly full), 16 (one echo past the log: fault), the full buffer, then 3.
    const std::vector<std::uint16_t> counts{0, 15, 16, 64, 3, 3, 3, 3, 3, 3, 3, 3};
    PinOutputs outputs;
    for (std::size_t tick = 1; tick <= counts.size(); ++tick) {
        runtime.tick(stub_inputs(tick, counts.at(tick - 1U)), outputs);
        INFO("tick " << tick);
        CHECK(native_test::same_outputs(outputs, expected_stub_outputs(tick, counts.at(tick - 1U))));
    }
    CHECK(runtime.ticks_run() == counts.size());
    CHECK(runtime.inits() == 1);

    runtime.reset(native_test::logic_rail);
    runtime.tick(stub_inputs(1, 2), outputs);
    CHECK(native_test::same_outputs(outputs, expected_stub_outputs(1, 2)));
    CHECK(runtime.inits() == 2);
}

TEST_CASE("Two runs of the same library with the same inputs give bitwise identical outputs", "[CTL-002][CTL-004]") {
    constexpr std::size_t ticks = 500;
    std::vector<PinOutputs> first_run;
    std::vector<PinOutputs> second_run;
    for (std::vector<PinOutputs>* run : {&first_run, &second_run}) {
        NativeControllerRuntime runtime(native_test::config("run-twice"));
        runtime.load(FirmwareImage{native_test::reference_library().string(), 1'000});
        runtime.reset(native_test::logic_rail);
        for (std::size_t tick = 0; tick < ticks; ++tick) {
            PinOutputs outputs;
            runtime.tick(native_test::patterned_inputs(static_cast<std::uint32_t>(tick)), outputs);
            run->push_back(outputs);
        }
    }
    REQUIRE(first_run.size() == ticks);
    REQUIRE(second_run.size() == ticks);
    for (std::size_t tick = 0; tick < ticks; ++tick) {
        INFO("tick " << tick);
        CHECK(native_test::same_outputs(first_run.at(tick), second_run.at(tick)));
    }
}
