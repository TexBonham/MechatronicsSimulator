#include "mechsim/kernel/party.hpp"

#include <string>
#include <utility>

namespace mechsim::kernel {

PartyFault::PartyFault(std::string party, SimTime time, std::string cause)
    : std::runtime_error(party + " at " + std::to_string(time.count()) + " ticks: " + cause),
      party_(std::move(party)),
      time_(time),
      cause_(std::move(cause)) {}

const std::string& PartyFault::party() const noexcept {
    return party_;
}

SimTime PartyFault::time() const noexcept {
    return time_;
}

const std::string& PartyFault::cause() const noexcept {
    return cause_;
}

}  // namespace mechsim::kernel
