#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include "mechsim/controller/controller_runtime.hpp"

namespace mechsim::controller {

PinOutputs held_levels(const PinOutputs& last) noexcept {
    PinOutputs held;
    held.direction = last.direction;
    held.enable = last.enable;
    held.digital_out = last.digital_out;
    held.state_code = last.state_code;
    held.fault_code = last.fault_code;
    return held;
}

NullControllerRuntime::NullControllerRuntime(kernel::Rate control_rate) : control_rate_(control_rate) {}

void NullControllerRuntime::load(const FirmwareImage& firmware) {
    if (firmware.path.empty()) {
        throw ControllerConfigError("firmware path is empty");
    }
    if (firmware.tick_hz == 0 || !(kernel::Rate(firmware.tick_hz) == control_rate_)) {
        throw ControllerConfigError("firmware declares " + std::to_string(firmware.tick_hz) +
                                    " Hz but the control rate is " + std::to_string(control_rate_.numerator()) + "/" +
                                    std::to_string(control_rate_.denominator()) + " Hz");
    }
    loaded_ = true;
    firmware_path_ = firmware.path;
}

void NullControllerRuntime::reset(const signals::PowerRail& rail) {
    if (!loaded_) {
        throw ControllerError("reset() before load()");
    }
    ++resets_;
    supply_voltage_ = rail.voltage;
}

void NullControllerRuntime::tick(const PinInputs& inputs, PinOutputs& outputs) {
    if (resets_ == 0) {
        throw ControllerError("tick() before reset()");
    }
    if (inputs.uart_rx_count > uart_buffer_bytes) {
        throw ControllerError("UART overflow: " + std::to_string(inputs.uart_rx_count) + " bytes in one tick, limit " +
                              std::to_string(uart_buffer_bytes));
    }
    // The null runtime publishes all-zero outputs, so the last published levels are zero too.
    if (frozen_) {
        outputs = held_levels(PinOutputs{});
        return;
    }
    outputs = PinOutputs{};
    ++ticks_;
}

void NullControllerRuntime::freeze(bool frozen) {
    frozen_ = frozen;
}

void NullControllerRuntime::brownout(double voltage) {
    if (!std::isfinite(voltage) || voltage < 0.0) {
        throw std::invalid_argument("supply voltage must be finite and non-negative");
    }
    supply_voltage_ = voltage;
}

const std::string& NullControllerRuntime::firmware_path() const noexcept {
    return firmware_path_;
}

std::size_t NullControllerRuntime::ticks_run() const noexcept {
    return ticks_;
}

std::size_t NullControllerRuntime::resets() const noexcept {
    return resets_;
}

bool NullControllerRuntime::frozen() const noexcept {
    return frozen_;
}

double NullControllerRuntime::supply_voltage() const noexcept {
    return supply_voltage_;
}

}  // namespace mechsim::controller
