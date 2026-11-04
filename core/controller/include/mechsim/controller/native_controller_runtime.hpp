#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "mechsim/controller/channel_map.hpp"
#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/power_rail.hpp"

namespace mechsim::controller {

// Board interface version the runtime is compiled against (MECHSIM_BOARD_INTERFACE_VERSION).
[[nodiscard]] std::uint32_t board_interface_version() noexcept;

// Brownout threshold used until chips/ supplies the chip's value (CTL-013 open parameter); 2.7 V is a placeholder.
inline constexpr double placeholder_brownout_threshold_v = 2.7;

struct HostBuildRequest {
    std::filesystem::path firmware_tree;  // holds CMakePresets.json with the host preset
    std::filesystem::path work_dir;       // the run's working directory; the build goes to native-firmware/host
    std::uint32_t tick_hz = 0;            // the design's control rate, passed as MECHSIM_APP_TICK_HZ
    std::string app_target = "mechsim_fw_app";
    std::string cmake_command = "cmake";
};

struct HostBuildResult {
    std::filesystem::path library;  // lib<app_target>_native.so
    std::string log;                // tool output of every step
};

// Configures the tree with its host preset and builds the application's SHARED <app_target>_native target (CTL-007). Throws ControllerConfigError carrying the tool output on any failure.
[[nodiscard]] HostBuildResult build_host_firmware(const HostBuildRequest& request);

struct NativeRuntimeConfig {
    kernel::Rate control_rate;           // RunConfig::control_rate
    std::filesystem::path work_dir;      // the run's working directory
    // Chip's brownout threshold, V (CTL-013); unset is a ControllerConfigError, never "no brownout".
    std::optional<double> brownout_threshold_v = placeholder_brownout_threshold_v;
    ChannelMap channels;                 // builder's channel table (CTL-006)
    std::string app_target = "mechsim_fw_app";
    std::string cmake_command = "cmake";
};

class LoadedApplication;

// The host build of the firmware called through the board interface each tick (CTL-002 to CTL-007). load() takes a
// firmware tree (directory: built with the host preset first) or a host-built application library (file). Every
// reset and brownout recovery reopens a fresh private copy of the library, so no firmware static survives (CTL-012).
class NativeControllerRuntime final : public ControllerRuntime {
public:
    explicit NativeControllerRuntime(NativeRuntimeConfig config);
    ~NativeControllerRuntime() override;
    NativeControllerRuntime(const NativeControllerRuntime&) = delete;
    NativeControllerRuntime& operator=(const NativeControllerRuntime&) = delete;
    NativeControllerRuntime(NativeControllerRuntime&&) = delete;
    NativeControllerRuntime& operator=(NativeControllerRuntime&&) = delete;

    void load(const FirmwareImage& firmware) override;
    void reset(const signals::PowerRail& rail) override;
    void tick(const PinInputs& inputs, PinOutputs& outputs) override;
    void freeze(bool frozen) override;
    void brownout(double voltage) override;

    [[nodiscard]] const ChannelMap& channel_map() const noexcept;
    [[nodiscard]] const std::filesystem::path& library_path() const noexcept;  // the library as built or given
    [[nodiscard]] const std::string& build_log() const noexcept;
    [[nodiscard]] std::size_t ticks_run() const noexcept;  // firmware tick calls
    [[nodiscard]] std::size_t inits() const noexcept;      // firmware init calls
    [[nodiscard]] bool frozen() const noexcept;
    [[nodiscard]] bool held_in_reset() const noexcept;
    [[nodiscard]] double supply_voltage() const noexcept;

private:
    void start_firmware();

    double brownout_threshold_v_ = 0.0;
    NativeRuntimeConfig config_;
    std::unique_ptr<LoadedApplication> application_;
    std::filesystem::path library_path_;
    std::string build_log_;
    PinOutputs last_outputs_{};
    std::size_t ticks_ = 0;
    std::size_t inits_ = 0;
    bool was_reset_ = false;
    bool held_ = true;
    bool frozen_ = false;
    double supply_voltage_ = 0.0;
};

}  // namespace mechsim::controller
