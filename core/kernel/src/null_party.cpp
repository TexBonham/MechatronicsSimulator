#include "mechsim/kernel/null_party.hpp"

#include <span>
#include <string>
#include <string_view>
#include <utility>

namespace mechsim::kernel {

NullParty::NullParty(std::string name, Rate rate) : name_(std::move(name)), rate_(rate) {}

std::string_view NullParty::name() const {
    return name_;
}

Rate NullParty::nominal_rate() const {
    return rate_;
}

std::span<const std::string> NullParty::published() const {
    return {};
}

std::span<const std::string> NullParty::consumed() const {
    return {};
}

void NullParty::start(const Rate& /*base_rate*/) {}

void NullParty::step(SimTime /*t*/, Duration /*dt*/) {}

void NullParty::on_fault(const PartyFault& /*fault*/) noexcept {}

}  // namespace mechsim::kernel
