#include "mechsim/controller/native_controller_runtime.hpp"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <type_traits>
#include <utility>

#include "mechsim/controller/channel_map.hpp"
#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/pins/pins.hpp"
#include "mechsim_board.h"

namespace mechsim::controller {

namespace {

namespace fs = std::filesystem;

template <typename CArray, typename StdArray>
constexpr bool same_shape() {
    return std::is_same_v<std::remove_extent_t<CArray>, typename StdArray::value_type> &&
           std::extent_v<CArray> == std::tuple_size_v<StdArray>;
}

static_assert(same_shape<decltype(mechsim_board_inputs::digital_in), decltype(PinInputs::digital_in)>());
static_assert(same_shape<decltype(mechsim_board_inputs::counter_delta), decltype(PinInputs::counter_delta)>());
static_assert(same_shape<decltype(mechsim_board_inputs::adc_sample), decltype(PinInputs::adc_sample)>());
static_assert(same_shape<decltype(mechsim_board_inputs::uart_rx), decltype(PinInputs::uart_rx)>());
static_assert(std::is_same_v<decltype(mechsim_board_inputs::uart_rx_count), decltype(PinInputs::uart_rx_count)>);
static_assert(same_shape<decltype(mechsim_board_outputs::step_count), decltype(PinOutputs::step_count)>());
static_assert(same_shape<decltype(mechsim_board_outputs::direction), decltype(PinOutputs::direction)>());
static_assert(same_shape<decltype(mechsim_board_outputs::enable), decltype(PinOutputs::enable)>());
static_assert(same_shape<decltype(mechsim_board_outputs::digital_out), decltype(PinOutputs::digital_out)>());
static_assert(same_shape<decltype(mechsim_board_outputs::uart_tx), decltype(PinOutputs::uart_tx)>());
static_assert(std::is_same_v<decltype(mechsim_board_outputs::uart_tx_count), decltype(PinOutputs::uart_tx_count)>);
static_assert(std::is_same_v<decltype(mechsim_board_outputs::state_code), decltype(PinOutputs::state_code)>);
static_assert(std::is_same_v<decltype(mechsim_board_outputs::fault_code), decltype(PinOutputs::fault_code)>);
static_assert(std::extent_v<decltype(mechsim_board_outputs::event_log)> == event_log_entries);
static_assert(std::is_same_v<decltype(mechsim_board_event::code), decltype(EventLogEntry::code)>);
static_assert(std::is_same_v<decltype(mechsim_board_event::arg), decltype(EventLogEntry::arg)>);
static_assert(std::is_same_v<decltype(mechsim_board_outputs::event_count), decltype(PinOutputs::event_count)>);
static_assert(MECHSIM_UART_RX_BYTES == uart_buffer_bytes && MECHSIM_UART_TX_BYTES == uart_buffer_bytes);

using InitFunction = void (*)();
using TickFunction = void (*)(const mechsim_board_inputs*, mechsim_board_outputs*);
using U32Function = std::uint32_t (*)();

std::string version_text(std::uint32_t version) {
    return std::to_string(version / 65536U) + "." + std::to_string(version % 65536U);
}

std::string rate_text(const kernel::Rate& rate) {
    return std::to_string(rate.numerator()) + "/" + std::to_string(rate.denominator()) + " Hz";
}

bool matches_rate(std::uint32_t hz, const kernel::Rate& rate) {
    return hz != 0 && kernel::Rate(hz) == rate;
}

void check_voltage(double voltage) {
    if (!std::isfinite(voltage) || voltage < 0.0) {
        throw std::invalid_argument("supply voltage must be finite and non-negative");
    }
}

#if defined(_WIN32)
std::string error_text(DWORD code) {
    LPWSTR buffer = nullptr;
    const DWORD length = ::FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, code, 0,
        reinterpret_cast<LPWSTR>(&buffer), 0, nullptr);
    std::string text = "error " + std::to_string(code);
    if (length > 0 && buffer != nullptr) {
        std::wstring message(buffer, length);
        while (!message.empty() && (message.back() == L'\n' || message.back() == L'\r' || message.back() == L' ')) {
            message.pop_back();
        }
        const int size = ::WideCharToMultiByte(CP_UTF8, 0, message.data(), static_cast<int>(message.size()), nullptr,
                                               0, nullptr, nullptr);
        text.assign(static_cast<std::size_t>(size), '\0');
        ::WideCharToMultiByte(CP_UTF8, 0, message.data(), static_cast<int>(message.size()), text.data(), size,
                              nullptr, nullptr);
    }
    if (buffer != nullptr) {
        ::LocalFree(buffer);
    }
    return text;
}
#endif

// dlopen hands back one handle per path, so two runtimes opening the same file would share the firmware's static
// state; each runtime therefore opens its own copy.
fs::path private_copy(const fs::path& library, const fs::path& work_dir) {
    static std::size_t instances = 0;
    ++instances;
#if defined(_WIN32)
    const fs::path dir = fs::absolute(work_dir) / "native-controller" /
                         (std::to_string(::GetCurrentProcessId()) + "-" + std::to_string(instances));
#else
    const fs::path dir = fs::absolute(work_dir) / "native-controller" /
                         (std::to_string(::getpid()) + "-" + std::to_string(instances));
#endif
    std::error_code error;
    fs::create_directories(dir, error);
    if (!error) {
        fs::copy_file(library, dir / library.filename(), fs::copy_options::overwrite_existing, error);
    }
    if (error) {
        throw ControllerConfigError("cannot stage " + library.string() + " in " + dir.string() + ": " +
                                    error.message());
    }
    return dir / library.filename();
}

template <typename Function>
Function resolve(void* handle, const char* name, const fs::path& library) {
#if defined(_WIN32)
    void* symbol = reinterpret_cast<void*>(::GetProcAddress(static_cast<HMODULE>(handle), name));
#else
    void* symbol = ::dlsym(handle, name);
#endif
    if (symbol == nullptr) {
        throw ControllerConfigError(library.string() + " does not export " + name);
    }
    return reinterpret_cast<Function>(symbol);
}

}  // namespace

// ===== LoadedApplication ====================================================

// Owns the dlopen handle and the private copy it was opened from.
class LoadedApplication {
public:
#if defined(_WIN32)
    LoadedApplication(const fs::path& library, const fs::path& work_dir)
        : copy_(private_copy(library, work_dir)), handle_(::LoadLibraryW(copy_.c_str())) {
        if (handle_ == nullptr) {
            const std::string message = "cannot load " + library.string() + ": " + error_text(::GetLastError());
            remove_copy();
            throw ControllerConfigError(message);
        }
#else
    LoadedApplication(const fs::path& library, const fs::path& work_dir)
        : copy_(private_copy(library, work_dir)), handle_(::dlopen(copy_.c_str(), RTLD_NOW | RTLD_LOCAL)) {
        if (handle_ == nullptr) {
            const char* reason = ::dlerror();
            std::string message = "cannot load " + library.string();
            if (reason != nullptr) {
                message += ": ";
                message += reason;
            }
            remove_copy();
            throw ControllerConfigError(message);
        }
#endif
        try {
            init = resolve<InitFunction>(handle_, "mechsim_app_init", library);
            tick = resolve<TickFunction>(handle_, "mechsim_app_tick", library);
            tick_hz = resolve<U32Function>(handle_, "mechsim_app_tick_hz", library);
            version = resolve<U32Function>(handle_, "mechsim_app_board_interface_version", library);
        } catch (...) {
            close();
            throw;
        }
    }
    ~LoadedApplication() { close(); }
    LoadedApplication(const LoadedApplication&) = delete;
    LoadedApplication& operator=(const LoadedApplication&) = delete;
    LoadedApplication(LoadedApplication&&) = delete;
    LoadedApplication& operator=(LoadedApplication&&) = delete;

    InitFunction init = nullptr;
    TickFunction tick = nullptr;
    U32Function tick_hz = nullptr;
    U32Function version = nullptr;

private:
    void close() noexcept {
        if (handle_ != nullptr) {
#if defined(_WIN32)
            ::FreeLibrary(static_cast<HMODULE>(handle_));
#else
            ::dlclose(handle_);
#endif
            handle_ = nullptr;
        }
        remove_copy();
    }
    void remove_copy() noexcept {
        std::error_code ignored;
        fs::remove_all(copy_.parent_path(), ignored);
    }

    fs::path copy_;
    void* handle_ = nullptr;
};

// ===== Pin copy =============================================================

namespace {

mechsim_board_inputs to_board(const PinInputs& inputs) {
    mechsim_board_inputs board{};
    std::ranges::copy(inputs.digital_in, std::begin(board.digital_in));
    std::ranges::copy(inputs.counter_delta, std::begin(board.counter_delta));
    std::ranges::copy(inputs.adc_sample, std::begin(board.adc_sample));
    std::ranges::copy(inputs.uart_rx, std::begin(board.uart_rx));
    board.uart_rx_count = inputs.uart_rx_count;
    return board;
}

void from_board(const mechsim_board_outputs& board, PinOutputs& outputs) {
    if (board.uart_tx_count > uart_buffer_bytes) {
        throw ControllerError("UART overflow: firmware sent " + std::to_string(board.uart_tx_count) +
                              " bytes in one tick, limit " + std::to_string(uart_buffer_bytes));
    }
    if (board.event_count > event_log_entries) {
        throw ControllerError("event log overflow: firmware raised " + std::to_string(board.event_count) +
                              " events in one tick, limit " + std::to_string(event_log_entries));
    }
    std::ranges::copy(board.step_count, outputs.step_count.begin());
    std::ranges::copy(board.direction, outputs.direction.begin());
    std::ranges::copy(board.enable, outputs.enable.begin());
    std::ranges::copy(board.digital_out, outputs.digital_out.begin());
    std::ranges::copy(board.uart_tx, outputs.uart_tx.begin());
    outputs.uart_tx_count = board.uart_tx_count;
    outputs.state_code = board.state_code;
    outputs.fault_code = board.fault_code;
    for (std::size_t index = 0; index < event_log_entries; ++index) {
        outputs.event_log.at(index).code = board.event_log[index].code;
        outputs.event_log.at(index).arg = board.event_log[index].arg;
    }
    outputs.event_count = board.event_count;
}

}  // namespace

// ===== NativeControllerRuntime ==============================================

std::uint32_t board_interface_version() noexcept {
    return MECHSIM_BOARD_INTERFACE_VERSION;
}

NativeControllerRuntime::NativeControllerRuntime(NativeRuntimeConfig config) : config_(std::move(config)) {
    if (!config_.brownout_threshold_v.has_value()) {
        throw ControllerConfigError("brownout threshold is unset; give the chip's value");
    }
    brownout_threshold_v_ = *config_.brownout_threshold_v;
    if (!std::isfinite(brownout_threshold_v_) || brownout_threshold_v_ < 0.0) {
        throw ControllerConfigError("brownout threshold must be finite and non-negative");
    }
    if (config_.work_dir.empty()) {
        throw ControllerConfigError("native runtime needs the run's working directory");
    }
}

NativeControllerRuntime::~NativeControllerRuntime() = default;

void NativeControllerRuntime::load(const FirmwareImage& firmware) {
    if (!matches_rate(firmware.tick_hz, config_.control_rate)) {
        throw ControllerConfigError("design tick rate " + std::to_string(firmware.tick_hz) +
                                    " Hz differs from the control rate " + rate_text(config_.control_rate));
    }
    if (firmware.path.empty()) {
        throw ControllerConfigError("firmware path is empty");
    }
    const fs::path source(firmware.path);
    fs::path library;
    std::string log;
    if (fs::is_directory(source)) {
        HostBuildResult built = build_host_firmware(HostBuildRequest{source, config_.work_dir, firmware.tick_hz,
                                                                     config_.app_target, config_.cmake_command});
        library = std::move(built.library);
        log = std::move(built.log);
    } else if (fs::is_regular_file(source)) {
        library = source;
    } else {
        throw ControllerConfigError("firmware " + firmware.path + " is neither a source tree nor a library");
    }

    auto application = std::make_unique<LoadedApplication>(library, config_.work_dir);
    const std::uint32_t version = application->version();
    if (version != board_interface_version()) {
        throw ControllerConfigError(library.string() + " was built against board interface " + version_text(version) +
                                    ", the runtime against " + version_text(board_interface_version()));
    }
    const std::uint32_t declared_hz = application->tick_hz();
    if (!matches_rate(declared_hz, config_.control_rate)) {
        throw ControllerConfigError("firmware declares " + std::to_string(declared_hz) +
                                    " Hz but the control rate is " + rate_text(config_.control_rate));
    }

    application_ = std::move(application);
    library_path_ = std::move(library);
    build_log_ = std::move(log);
    last_outputs_ = PinOutputs{};
    ticks_ = 0;
    inits_ = 0;
    was_reset_ = false;
    held_ = true;
    frozen_ = false;
}

void NativeControllerRuntime::reset(const pins::PowerRail& rail) {
    if (library_path_.empty()) {
        throw ControllerError("reset() before load()");
    }
    double voltage = 0.0;
    if (rail.on) {
        voltage = rail.voltage;
    }
    check_voltage(voltage);
    was_reset_ = true;
    supply_voltage_ = voltage;
    last_outputs_ = PinOutputs{};
    held_ = true;
    if (voltage >= brownout_threshold_v_) {
        start_firmware();
    }
}

void NativeControllerRuntime::tick(const PinInputs& inputs, PinOutputs& outputs) {
    if (!was_reset_) {
        throw ControllerError("tick() before reset()");
    }
    if (inputs.uart_rx_count > uart_buffer_bytes) {
        throw ControllerError("UART overflow: " + std::to_string(inputs.uart_rx_count) + " bytes in one tick, limit " +
                              std::to_string(uart_buffer_bytes));
    }
    if (held_) {
        outputs = PinOutputs{};
        return;
    }
    if (frozen_) {
        outputs = held_levels(last_outputs_);
        return;
    }
    const mechsim_board_inputs board_inputs = to_board(inputs);
    mechsim_board_outputs board_outputs{};
    application_->tick(&board_inputs, &board_outputs);
    ++ticks_;
    PinOutputs published;
    from_board(board_outputs, published);
    derive_axis_levels(config_.channels, published);
    last_outputs_ = published;
    outputs = published;
}

void NativeControllerRuntime::freeze(bool frozen) {
    frozen_ = frozen;
}

void NativeControllerRuntime::brownout(double voltage) {
    check_voltage(voltage);
    supply_voltage_ = voltage;
    if (voltage < brownout_threshold_v_) {
        held_ = true;
        last_outputs_ = PinOutputs{};
        return;
    }
    if (held_ && was_reset_) {
        start_firmware();
    }
}

void NativeControllerRuntime::start_firmware() {
    application_.reset();
    application_ = std::make_unique<LoadedApplication>(library_path_, config_.work_dir);
    application_->init();
    ++inits_;
    held_ = false;
}

const ChannelMap& NativeControllerRuntime::channel_map() const noexcept {
    return config_.channels;
}

const std::filesystem::path& NativeControllerRuntime::library_path() const noexcept {
    return library_path_;
}

const std::string& NativeControllerRuntime::build_log() const noexcept {
    return build_log_;
}

std::size_t NativeControllerRuntime::ticks_run() const noexcept {
    return ticks_;
}

std::size_t NativeControllerRuntime::inits() const noexcept {
    return inits_;
}

bool NativeControllerRuntime::frozen() const noexcept {
    return frozen_;
}

bool NativeControllerRuntime::held_in_reset() const noexcept {
    return held_;
}

double NativeControllerRuntime::supply_voltage() const noexcept {
    return supply_voltage_;
}

}  // namespace mechsim::controller
