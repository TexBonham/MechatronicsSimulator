#include "mechsim/signals/signal_name.hpp"

#include <cstddef>
#include <string_view>

namespace mechsim::signals {

namespace {

bool is_identifier(std::string_view part) noexcept {
    if (part.empty()) {
        return false;
    }
    for (std::size_t i = 0; i < part.size(); ++i) {
        const char c = part[i];
        const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
        const bool digit = c >= '0' && c <= '9';
        if (letter) {
            continue;
        }
        if (!digit || i == 0) {
            return false;
        }
    }
    return true;
}

}  // namespace

bool is_dotted_signal_name(std::string_view name) noexcept {
    const std::size_t dot = name.find('.');
    if (dot == std::string_view::npos) {
        return false;
    }
    return is_identifier(name.substr(0, dot)) && is_identifier(name.substr(dot + 1));
}

std::string_view signal_instance(std::string_view name) noexcept {
    if (!is_dotted_signal_name(name)) {
        return {};
    }
    return name.substr(0, name.find('.'));
}

bool is_instance_name(std::string_view name) noexcept {
    return is_identifier(name);
}

}  // namespace mechsim::signals
