#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "mechsim/kernel/time.hpp"
#include "mechsim/pins/pins.hpp"

namespace mechsim::controller {

using pins::adc_channels;
using pins::counter_channels;
using pins::digital_in_channels;
using pins::digital_out_channels;
using pins::event_log_entries;
using pins::EventLogEntry;
using pins::PinInputs;
using pins::PinOutputs;
using pins::stepper_channels;
using pins::uart_buffer_bytes;

// Native mode: a firmware source tree the runtime builds for the host, or an application library it built earlier;
// emulated mode: the .elf Renode loads (CTL-007, CTL-008). tick_hz is the design's control rate the firmware is
// built for; load also checks the rate the firmware declares (mechsim_app_tick_hz).
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
    virtual void reset(const pins::PowerRail& rail) = 0;
    // One control tick; native mode calls the tick entry point once, emulated mode runs one tick of virtual time
    // (CTL-008). outputs is overwritten. While frozen nothing runs, levels hold and per-tick buffers are empty
    // (held_levels, CTL-011); while held in brownout reset nothing runs and outputs are zeroed (CTL-013).
    // uart_rx_count above uart_buffer_bytes throws ControllerError; it is never truncated.
    virtual void tick(const PinInputs& inputs, PinOutputs& outputs) = 0;
    virtual void freeze(bool frozen) = 0;  // CTL-011
    // Supply voltage, V; below the chip's brownout threshold the controller is held in reset (CTL-013).
    virtual void brownout(double voltage) = 0;
};

// Outputs of a frozen controller (CTL-011): latched levels (direction, enable, digital_out, state and fault codes) as
// last published; per-tick buffers empty (no step pulses, no UART bytes, no events). Bit-banged pulses stop on a hang
// in milestone 1; timer-driven pulses that continue are an open CTL parameter.
[[nodiscard]] PinOutputs held_levels(const PinOutputs& last) noexcept;

// Runs nothing and echoes nothing: tick() zeroes outputs and counts the ticks it would have run.
class NullControllerRuntime final : public ControllerRuntime {
public:
    explicit NullControllerRuntime(kernel::Rate control_rate);

    void load(const FirmwareImage& firmware) override;
    void reset(const pins::PowerRail& rail) override;
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
