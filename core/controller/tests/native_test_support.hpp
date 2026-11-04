#pragma once

#include <cstdint>
#include <filesystem>
#include <string_view>

#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/controller/native_controller_runtime.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/power_rail.hpp"

namespace native_test {

inline constexpr std::string_view firmware_tree = MECHSIM_TEST_FIRMWARE_TREE;
inline constexpr std::string_view cmake_command = MECHSIM_TEST_CMAKE;
inline constexpr std::string_view mirror_app = MECHSIM_TEST_MIRROR_APP;
inline constexpr std::string_view mirror_bad_version = MECHSIM_TEST_MIRROR_BAD_VERSION;
inline constexpr std::string_view mirror_no_version = MECHSIM_TEST_MIRROR_NO_VERSION;
inline constexpr std::string_view mirror_250hz = MECHSIM_TEST_MIRROR_250HZ;

// Placeholder threshold for the tests; the chip's value is an open CTL parameter (owner, with chips/).
inline constexpr double brownout_threshold_v = 2.7;

inline const mechsim::signals::PowerRail logic_rail{"logic", 3.3, true};

[[nodiscard]] std::filesystem::path work_dir(std::string_view name);
// Work directory of the one real host build of the reference stub (the ctest fixture writes it).
[[nodiscard]] std::filesystem::path reference_work_dir();
// The reference stub's shared library; builds it when the fixture has not (the binary run on its own).
[[nodiscard]] std::filesystem::path reference_library();

[[nodiscard]] mechsim::controller::NativeRuntimeConfig config(std::string_view work, std::int64_t control_hz = 1'000);

// Field-by-field equality, padding excluded.
[[nodiscard]] bool same_outputs(const mechsim::controller::PinOutputs& a, const mechsim::controller::PinOutputs& b);

// Deterministic input pattern from a seeded linear congruential generator; uart_rx_count stays within the buffer and
// the last ADC channel stays below the mirror application's overflow triggers.
[[nodiscard]] mechsim::controller::PinInputs patterned_inputs(std::uint32_t seed);

}  // namespace native_test
