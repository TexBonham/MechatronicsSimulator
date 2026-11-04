#pragma once

#include <string>
#include <vector>

#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

// One named scalar series sample; name is the dotted signal or rail name (SIG-023), unit explicit (REC-018).
struct SeriesSample {
    std::string name;
    std::string unit;
    double value = 0.0;
};

// A timestamped event: injected fault, safety event, operator command or status, integrity event, controller fault
// log entry (REC-007 to REC-010).
struct RecordedEvent {
    SimTime time;
    std::string source;  // party or component instance name
    std::string kind;
    std::string detail;
};

// One recorder row per physics tick, written last in the tick (KER-009). Simulated time comes first (REC-011).
struct TickRecord {
    SimTime time;
    PhysicsOutputs physics;                     // REC-002, REC-003, REC-004
    std::vector<SeriesSample> pin_signals;      // both directions across the pin-binding table (REC-005)
    std::vector<SeriesSample> rails;            // REC-006
    std::vector<SeriesSample> actuator_efforts; // torques and forces applied to physics (REC-027)
    std::string controller_state;               // REC-007
    std::vector<RecordedEvent> events;          // REC-007 to REC-010
    std::vector<CouplingSample> coupling;       // every coupling variable at the row's sync point, name order (KER-022)
};

}  // namespace mechsim::kernel
