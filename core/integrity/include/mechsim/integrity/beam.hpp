#pragma once

#include <span>

#include "mechsim/integrity/limits.hpp"
#include "mechsim/kernel/physics_types.hpp"

namespace mechsim::integrity {

// Beam theory on one tick's member end-loads, the formulas of mechsim/integrity/beam.py (ruling 25, ruling 28).

// abs(N) / A + abs(M_y) c_z / I_y + abs(M_z) c_y / I_z at the start and the end, and at the interior point each bending
// plane resolves its interior load to (abs(N) there the larger end value); the largest of them.
[[nodiscard]] double peak_normal_stress(const kernel::MemberEndLoads& loads, const MemberLimits& member);

// abs(N) / A + abs(M_y) c_z / I_y + abs(M_z) c_y / I_z at one end.
[[nodiscard]] double end_stress(const kernel::Wrench& wrench, const MemberLimits& member);

// Largest compressive axial force over the two ends (>= 0): max(F_start.x, -F_end.x, 0).
[[nodiscard]] double compression(const kernel::MemberEndLoads& loads);

// pi^2 E min(I_y, I_z) / (K L)^2.
[[nodiscard]] double critical_load(const MemberLimits& member);

// Deflection from the published node poses (ruling 28, INT-006): the largest perpendicular distance of the chain's node
// origins and its far end (the last node plus l / 2 along its x) from the reference line. A clamped root's line is
// the root tangent (the parent pose times the member placement, x); a pinned root's the chord from the pin (node 0)
// to the far end. Throws LimitsError without nodes or with nodes outside the published bodies.
[[nodiscard]] double node_deflection(std::span<const kernel::BodyState> bodies, const MemberLimits& member);

// The moment the node spring nearest an end carries, by statics over the half segment between the end and the spring
// (its inertia neglected, ruling 35): start -M_z + (l / 2) F_y, end M_z + (l / 2) F_y, the spring's own sense.
[[nodiscard]] double start_node_moment(const kernel::Wrench& start, double segment_length);
[[nodiscard]] double end_node_moment(const kernel::Wrench& end, double segment_length);

}  // namespace mechsim::integrity
