#include "native_test_support.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/controller/native_controller_runtime.hpp"
#include "mechsim/kernel/time.hpp"

namespace native_test {

using mechsim::controller::NativeRuntimeConfig;
using mechsim::controller::PinInputs;
using mechsim::controller::PinOutputs;

std::filesystem::path work_dir(std::string_view name) {
    return std::filesystem::path(MECHSIM_TEST_WORK_DIR) / name;
}

std::filesystem::path reference_work_dir() {
    return work_dir("reference");
}

std::filesystem::path reference_library() {
    std::filesystem::path library =
        reference_work_dir() / "native-firmware" / "host" / "libmechsim_fw_app_native.so";
    if (std::filesystem::is_regular_file(library)) {
        return library;
    }
    mechsim::controller::HostBuildRequest request;
    request.firmware_tree = std::filesystem::path(firmware_tree);
    request.work_dir = reference_work_dir();
    request.tick_hz = 1'000;
    request.cmake_command = std::string(cmake_command);
    return mechsim::controller::build_host_firmware(request).library;
}

NativeRuntimeConfig config(std::string_view work, std::int64_t control_hz) {
    NativeRuntimeConfig result;
    result.control_rate = mechsim::kernel::Rate(control_hz);
    result.work_dir = work_dir(work);
    result.brownout_threshold_v = brownout_threshold_v;
    result.cmake_command = std::string(cmake_command);
    return result;
}

bool same_outputs(const PinOutputs& a, const PinOutputs& b) {
    bool same = a.step_count == b.step_count && a.direction == b.direction && a.enable == b.enable &&
                a.digital_out == b.digital_out && a.uart_tx == b.uart_tx && a.uart_tx_count == b.uart_tx_count &&
                a.state_code == b.state_code && a.fault_code == b.fault_code && a.event_count == b.event_count;
    for (std::size_t index = 0; index < a.event_log.size(); ++index) {
        same = same && a.event_log.at(index).code == b.event_log.at(index).code &&
               a.event_log.at(index).arg == b.event_log.at(index).arg;
    }
    return same;
}

PinInputs patterned_inputs(std::uint32_t seed) {
    std::uint32_t state = seed;
    auto next = [&state]() {
        state = (state * 1'664'525U) + 1'013'904'223U;
        return state >> 8U;
    };
    PinInputs inputs;
    for (std::uint8_t& level : inputs.digital_in) {
        level = static_cast<std::uint8_t>(next() & 1U);
    }
    for (std::int32_t& delta : inputs.counter_delta) {
        delta = static_cast<std::int32_t>(next() % 2'001U) - 1'000;
    }
    for (std::uint16_t& sample : inputs.adc_sample) {
        sample = static_cast<std::uint16_t>(next() % 4'096U);
    }
    for (std::uint8_t& byte : inputs.uart_rx) {
        byte = static_cast<std::uint8_t>(next() & 0xFFU);
    }
    inputs.uart_rx_count = static_cast<std::uint16_t>(next() % 65U);
    return inputs;
}

}  // namespace native_test
