#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace mechsim::signals {

enum class PeripheralMode : std::uint8_t {
    digital_input,
    digital_output,
    counter_input,  // encoder interface or pulse counter: counts accumulated over the tick (CTL-004)
    adc_input,
    pulse_output,  // step pulses: count per tick
    uart_rx,
    uart_tx,
};

class PinBindingError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

// One entry: a dotted signal name bound to one MCU pin in one peripheral mode (SIG-023). Pin names are the chip's
// pin map names, opaque here.
struct PinBinding {
    std::string signal;
    std::string pin;
    PeripheralMode mode = PeripheralMode::digital_input;
};

// Every value crossing between the signal layer and the controller runtime crosses through an entry of this table
// (SIG-024, invariant 1). Signal names and pins are each unique; entries keep insertion order (determinism).
class PinBindingTable {
public:
    // Throws PinBindingError on a name that is not dotted, an empty pin, a duplicate signal or a duplicate pin.
    void add(PinBinding binding);

    [[nodiscard]] std::optional<PinBinding> find_signal(std::string_view signal) const;
    [[nodiscard]] std::optional<PinBinding> find_pin(std::string_view pin) const;
    [[nodiscard]] std::span<const PinBinding> entries() const noexcept;

private:
    std::vector<PinBinding> entries_;
};

}  // namespace mechsim::signals
