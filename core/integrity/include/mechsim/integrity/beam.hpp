#pragma once

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

// Peak static deflection magnitude over the 201-point grid (plus the supports), I1's end-load formula (ruling 28).
// Throws LimitsError when the supports leave the member free.
[[nodiscard]] double static_deflection(const kernel::MemberEndLoads& loads, const MemberLimits& member);

// The moment the node spring nearest an end carries, by statics over the half segment between the end and the spring
// (its inertia neglected, ruling 35): start -M_z + (l / 2) F_y, end M_z + (l / 2) F_y, the spring's own sense.
[[nodiscard]] double start_node_moment(const kernel::Wrench& start, double segment_length);
[[nodiscard]] double end_node_moment(const kernel::Wrench& end, double segment_length);

}  // namespace mechsim::integrity
