#include "mechsim/kernel/kernel.hpp"

#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

void Kernel::set_end_triggers(const EndTriggers& triggers) {
    if (triggers.at_time.has_value() || !triggers.on_signal.empty() || !triggers.party_state.empty()) {
        throw KernelConfigError("this kernel supports no end triggers");
    }
}

}  // namespace mechsim::kernel
