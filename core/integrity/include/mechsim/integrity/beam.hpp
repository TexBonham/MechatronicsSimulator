#pragma once

#include <optional>
#include <span>
#include <vector>

#include "mechsim/integrity/limits.hpp"
#include "mechsim/kernel/physics_types.hpp"

namespace mechsim::integrity {

// Beam theory on one tick's member end-loads, the formulas of mechsim/integrity/beam.py (ruling 25, ruling 28).

// Bending moment about member z at every node spring from the published node poses (INT-005, INT-023): k times the
// rotation of each node body relative to the one before it (before a clamped root's node 0 the root frame, the parent
// pose times the member placement; a pinned root's node 0 is the spring-free pin segment), in the spring's sense, root
// to tip, spring j at x = (j + 1/2) l. `offsets` (empty, or one per spring, rad): each spring's permanent rest offset
// (the yield offset, INT-028); the spring carries k times its elastic rotation, the total minus the offset. Throws
// LimitsError without nodes, with nodes outside the published bodies or with another offset count.
[[nodiscard]] std::vector<double> node_moments(std::span<const kernel::BodyState> bodies, const MemberLimits& member,
                                               std::span<const double> offsets = {});

// A carried joint's load as the engine applies it in a step (ENG-052): its foot point s along the member (m), the
// in-plane force F = F . y (N) and the couple about the foot point C = M_p . z (N m), member frame.
struct CarriedAt {
    double position = 0.0;
    double force = 0.0;
    double couple = 0.0;
};

// (s, F . y, C) of each joint the chain lists in `carried`, from one tick's poses and joint reactions: the load is the
// negated reaction (parent on child, world-aligned, about the child origin r_c), s = (r_c - o) . x in the root frame
// (o, x, y, z: the parent's pose times the member placement), C = (-M + (r_c - p) x (-R)) . z, p = o + s x. The engine
// places the pair at the step's start, so the party passes the previous tick's outputs. Throws LimitsError for a body
// or joint outside the published ones, or a chain without nodes.
[[nodiscard]] std::vector<CarriedAt> carried_at(std::span<const kernel::BodyState> bodies,
                                                std::span<const kernel::JointForce> joints, const MemberLimits& member);

// A moment read at a carried load: value N m (spring sense), position m along the member.
struct CarriedMoment {
    double value = 0.0;
    double position = 0.0;
};

// The rule of rulings 71 and 75 (derivation and end conditions: mechsim/integrity/beam.py, section "Moment under a
// carried load"; the same algorithm as its `row_reads`): loads sorted by s and grouped into clusters whose lever
// segments differ by at most 2; per cluster the moment just left of its first load (line A through the clean samples
// k - 1 and k, or a pinned root's start line) and just right of its last (line B through k + 2 and k + 3, or the end's
// line). `bound` the largest U of a cluster the rule does not resolve (a couple among several loads, three or more
// loads, a clamped root's stub), 0 when every cluster is exact.
struct CarriedReads {
    std::vector<CarriedMoment> reads;
    double bound = 0.0;
};
[[nodiscard]] CarriedReads carried_reads(const kernel::MemberEndLoads& loads, std::span<const double> moments,
                                         const MemberLimits& member, std::span<const CarriedAt> carried);

// abs(N) / A + abs(M_y) c_z / I_y + abs(M_z) c_y / I_z at the start and the end and, for a member with nodes, at each
// node spring: abs(N) the larger end value, M_z the spring's moment (`node_moments` under `offsets`), M_y
// interpolated linearly between the end moments (the chain is rigid out of its bending plane, ruling 65); on a member
// whose chain carries a joint also at each `carried_reads` read of the `carried` loads; the largest of them, plus the
// reads' bound times c_y / I_z (conservative, ruling 75). The formula of mechsim/integrity/beam.py
// `member_normal_stress` with the strength check's bound.
[[nodiscard]] double peak_normal_stress(const kernel::MemberEndLoads& loads, std::span<const kernel::BodyState> bodies,
                                        const MemberLimits& member, std::span<const double> offsets = {},
                                        std::span<const CarriedAt> carried = {});

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
