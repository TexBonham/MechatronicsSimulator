#include "mechsim/signals/pin_binding.hpp"

#include <optional>
#include <span>
#include <string_view>
#include <utility>

#include "mechsim/signals/signal_name.hpp"

namespace mechsim::signals {

void PinBindingTable::add(PinBinding binding) {
    if (!is_dotted_signal_name(binding.signal)) {
        throw PinBindingError("signal '" + binding.signal + "' is not a dotted instance.signal name");
    }
    if (binding.pin.empty()) {
        throw PinBindingError("signal '" + binding.signal + "' has an empty pin");
    }
    for (const PinBinding& entry : entries_) {
        if (entry.signal == binding.signal) {
            throw PinBindingError("signal '" + binding.signal + "' is already bound to pin '" + entry.pin + "'");
        }
        if (entry.pin == binding.pin) {
            throw PinBindingError("pin '" + binding.pin + "' is already bound to signal '" + entry.signal + "'");
        }
    }
    entries_.push_back(std::move(binding));
}

std::optional<PinBinding> PinBindingTable::find_signal(std::string_view signal) const {
    for (const PinBinding& entry : entries_) {
        if (entry.signal == signal) {
            return entry;
        }
    }
    return std::nullopt;
}

std::optional<PinBinding> PinBindingTable::find_pin(std::string_view pin) const {
    for (const PinBinding& entry : entries_) {
        if (entry.pin == pin) {
            return entry;
        }
    }
    return std::nullopt;
}

std::span<const PinBinding> PinBindingTable::entries() const noexcept {
    return entries_;
}

}  // namespace mechsim::signals
