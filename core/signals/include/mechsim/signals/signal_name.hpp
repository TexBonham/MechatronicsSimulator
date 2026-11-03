#pragma once

#include <stdexcept>
#include <string_view>

namespace mechsim::signals {

class SignalNameError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

// True for "instance.signal" (ADR 0012 D2, SIG-023): exactly two non-empty segments, each a letter or underscore
// followed by letters, digits or underscores.
[[nodiscard]] bool is_dotted_signal_name(std::string_view name) noexcept;

// True for a single segment: the instance part of a dotted name on its own.
[[nodiscard]] bool is_instance_name(std::string_view name) noexcept;

// The instance part of a dotted name; empty if the name is not dotted.
[[nodiscard]] std::string_view signal_instance(std::string_view name) noexcept;

}  // namespace mechsim::signals
