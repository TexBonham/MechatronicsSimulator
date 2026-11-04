#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/power_rail.hpp"

namespace mechsim::controller {

// Fixed contract sizes, identical to the board interface C header (board_interface/, brief C6); not overridable.
inline constexpr std::size_t digital_in_channels = 32;
inline constexpr std::size_t counter_channels = 8;
inline constexpr std::size_t adc_channels = 16;
inline constexpr std::size_t uart_buffer_bytes = 64;
inline constexpr std::size_t stepper_channels = 8;
inline constexpr std::size_t digital_out_channels = 32;
inline constexpr std::size_t event_log_entries = 16;

// Polled semantics (CTL-004, ADR 0011 C3): levels latched at the sync point, counter deltas accumulated during the
// tick, bytes that arrived during the tick, nothing interrupts the tick. No rail field: the rail reaches the
// runtime through reset() and brownout() (CTL-013).
struct PinInputs {
    std::array<std::uint8_t, digital_in_channels> digital_in{};  // 0 or 1
    std::array<std::int32_t, counter_channels> counter_delta{};  // counts accumulated during the tick
    std::array<std::uint16_t, adc_channels> adc_sample{};
    std::array<std::uint8_t, uart_buffer_bytes> uart_rx{};
    std::uint16_t uart_rx_count = 0;
};

struct EventLogEntry {
    std::uint8_t code = 0;
    std::uint16_t arg = 0;
};

struct PinOutputs {
    std::array<std::uint32_t, stepper_channels> step_count{};  // pulses this tick
    std::array<std::uint8_t, stepper_channels> direction{};
    std::array<std::uint8_t, stepper_channels> enable{};
    std::array<std::uint8_t, digital_out_channels> digital_out{};
    std::array<std::uint8_t, uart_buffer_bytes> uart_tx{};
    std::uint16_t uart_tx_count = 0;
    std::uint8_t state_code = 0;  // CTL-018 controller state
    std::uint8_t fault_code = 0;  // CTL-018 fault
    std::array<EventLogEntry, event_log_entries> event_log{};  // CTL-018 fault log
    std::uint8_t event_count = 0;
};

// Native mode: the host build of the firmware; emulated mode: the .elf Renode loads (CTL-007, CTL-008).
// tick_hz is the firmware's declared tick rate (mechsim_app_tick_hz), read from the build by the runtime.
struct FirmwareImage {
    std::string path;
    std::uint32_t tick_hz = 0;
};

// Rate mismatch, overflowed buffer, call out of order: ends the run with a report (invariant 7).
class ControllerError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class ControllerConfigError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

// One runtime interface, native and emulated implementations (CTL-001, CTL-023). It sees pin inputs and rail state
// only, never physical state (CTL-014, invariant 1); this header includes no kernel physics type for that reason.
// A runtime is built with RunConfig::control_rate; the tick interval is 1 / control_rate (CTL-002).
class ControllerRuntime {
public:
    ControllerRuntime() = default;
    virtual ~ControllerRuntime() = default;
    ControllerRuntime(const ControllerRuntime&) = delete;
    ControllerRuntime& operator=(const ControllerRuntime&) = delete;
    ControllerRuntime(ControllerRuntime&&) = delete;
    ControllerRuntime& operator=(ControllerRuntime&&) = delete;

    // Throws ControllerConfigError unless the firmware's declared tick rate equals the control rate.
    virtual void load(const FirmwareImage& firmware) = 0;
    // Restart from the reset state; native mode then calls init once (CTL-002, CTL-012).
    virtual void reset(const signals::PowerRail& rail) = 0;
    // One control tick; native mode calls the tick entry point once, emulated mode runs one tick of virtual time
    // (CTL-008). outputs is overwritten. While frozen or held in brownout reset, outputs are zeroed and nothing runs.
    // uart_rx_count above uart_buffer_bytes throws ControllerError; it is never truncated.
    virtual void tick(const PinInputs& inputs, PinOutputs& outputs) = 0;
    virtual void freeze(bool frozen) = 0;  // CTL-011
    // Supply voltage, V; below the chip's brownout threshold the controller is held in reset (CTL-013).
    virtual void brownout(double voltage) = 0;
};

// Runs nothing and echoes nothing: tick() zeroes outputs and counts the ticks it would have run.
class NullControllerRuntime final : public ControllerRuntime {
public:
    explicit NullControllerRuntime(kernel::Rate control_rate);

    void load(const FirmwareImage& firmware) override;
    void reset(const signals::PowerRail& rail) override;
    void tick(const PinInputs& inputs, PinOutputs& outputs) override;
    void freeze(bool frozen) override;
    void brownout(double voltage) override;

    [[nodiscard]] const std::string& firmware_path() const noexcept;
    [[nodiscard]] std::size_t ticks_run() const noexcept;
    [[nodiscard]] std::size_t resets() const noexcept;
    [[nodiscard]] bool frozen() const noexcept;
    [[nodiscard]] double supply_voltage() const noexcept;

private:
    kernel::Rate control_rate_;
    bool loaded_ = false;
    std::string firmware_path_;
    std::size_t ticks_ = 0;
    std::size_t resets_ = 0;
    bool frozen_ = false;
    double supply_voltage_ = 0.0;
};

}  // namespace mechsim::controller
