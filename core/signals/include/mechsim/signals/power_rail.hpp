#pragma once

#include <string>

namespace mechsim::signals {

// State of one power rail each tick (SIG-015, SIG-022, REC-006). The controller runtime receives this and pin
// inputs, nothing else (CTL-014), so this header includes no physics type.
struct PowerRail {
    std::string name;
    double voltage = 0.0;  // V
    bool on = false;
};

}  // namespace mechsim::signals
