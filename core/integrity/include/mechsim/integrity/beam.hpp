#pragma once

#include <span>
#include <vector>

#include "mechsim/integrity/limits.hpp"
#include "mechsim/kernel/physics_types.hpp"

namespace mechsim::integrity {

// Beam theory on one tick's member end-loads, the formulas of mechsim/integrity/beam.py (ruling 25, ruling 28).

// Bending moment about member z at every node spring from the published node poses (INT-005, INT-023): k times the
// rotation of each node body relative to the one before it (before a clamped root's node 0 the root frame, the parent
// pose times the member placement; a pinned root's node 0 is the spring-free pin segment), in the spring's sense, root
// to tip, spring j at x = (j + 1/2) l. Throws LimitsError without nodes or with nodes outside the published bodies.
[[nodiscard]] std::vector<double> node_moments(std::span<const kernel::BodyState> bodies, const MemberLimits& member);

// abs(N) / A + abs(M_y) c_z / I_y + abs(M_z) c_y / I_z at the start and the end and, for a member with nodes, at each
// node spring: abs(N) the larger end value, M_z the spring's moment (`node_moments`), M_y interpolated linearly
// between the end moments (the chain is rigid out of its bending plane, ruling 65); the largest of them. The formula
// of mechsim/integrity/beam.py `member_normal_stress`.
[[nodiscard]] double peak_normal_stress(const kernel::MemberEndLoads& loads, std::span<const kernel::BodyState> bodies,
                                        const MemberLimits& member);

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
