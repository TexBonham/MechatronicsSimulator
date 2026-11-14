#include "mechsim/integrity/beam.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "mechsim/integrity/limits.hpp"
#include "mechsim/kernel/physics_types.hpp"

namespace mechsim::integrity {

double end_stress(const kernel::Wrench& wrench, const MemberLimits& member) {
    return std::abs(wrench.force[0]) / member.area + std::abs(wrench.moment[1]) * member.c_z / member.i_y +
           std::abs(wrench.moment[2]) * member.c_y / member.i_z;
}

double compression(const kernel::MemberEndLoads& loads) {
    return std::max({loads.start.force[0], -loads.end.force[0], 0.0});
}

double critical_load(const MemberLimits& member) {
    const double kl = member.effective_length_factor * member.length;
    return std::numbers::pi * std::numbers::pi * member.elastic_modulus * std::min(member.i_y, member.i_z) / (kl * kl);
}

namespace {

kernel::Vec3 rotate(const kernel::Quat& q, const kernel::Vec3& v) {
    const double tx = 2.0 * (q[2] * v[2] - q[3] * v[1]);
    const double ty = 2.0 * (q[3] * v[0] - q[1] * v[2]);
    const double tz = 2.0 * (q[1] * v[1] - q[2] * v[0]);
    return {v[0] + q[0] * tx + (q[2] * tz - q[3] * ty), v[1] + q[0] * ty + (q[3] * tx - q[1] * tz),
            v[2] + q[0] * tz + (q[1] * ty - q[2] * tx)};
}

kernel::Quat multiply(const kernel::Quat& a, const kernel::Quat& b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

kernel::Vec3 plus(const kernel::Vec3& a, const kernel::Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

kernel::Vec3 minus(const kernel::Vec3& a, const kernel::Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

kernel::Vec3 scaled(const kernel::Vec3& a, double k) {
    return {a[0] * k, a[1] * k, a[2] * k};
}

double dot(const kernel::Vec3& a, const kernel::Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double distance_from_line(const kernel::Vec3& point, const kernel::Vec3& origin, const kernel::Vec3& direction) {
    const kernel::Vec3 r = minus(point, origin);
    const kernel::Vec3 off = minus(r, scaled(direction, dot(r, direction)));
    return std::sqrt(dot(off, off));
}

// Rotation of `current` relative to `previous` about their common z (the chain's bending axis): the relative
// quaternion conj(previous) current in its w >= 0 half, 2 atan2(r_z, r_w).
double bending_angle(const kernel::Quat& previous, const kernel::Quat& current) {
    const kernel::Quat inverse{previous[0], -previous[1], -previous[2], -previous[3]};
    const kernel::Quat relative = multiply(inverse, current);
    double sign = 1.0;
    if (relative[0] < 0.0) {
        sign = -1.0;
    }
    return 2.0 * std::atan2(sign * relative[3], sign * relative[0]);
}

}  // namespace

double node_deflection(std::span<const kernel::BodyState> bodies, const MemberLimits& member) {
    if (!member.nodes.has_value()) {
        throw LimitsError("member '" + member.name + "': no node poses (built rigid); deflection not evaluated");
    }
    const NodeChain& chain = member.nodes.value();
    if (chain.count == 0 || chain.first_body + chain.count > bodies.size()) {
        throw LimitsError("member '" + member.name + "': nodes " + std::to_string(chain.first_body) + " + " +
                          std::to_string(chain.count) + " outside the " + std::to_string(bodies.size()) +
                          " published bodies");
    }
    const kernel::BodyState& last = bodies[chain.first_body + chain.count - 1];
    const kernel::Vec3 tip = plus(last.position, rotate(last.orientation, {0.5 * member.segment_length, 0.0, 0.0}));
    kernel::Vec3 origin{};
    kernel::Vec3 direction{};
    if (chain.root == SupportCondition::pinned) {
        origin = bodies[chain.first_body].position;
        const kernel::Vec3 chord = minus(tip, origin);
        const double length = std::sqrt(dot(chord, chord));
        if (!(length > 0.0)) {
            throw LimitsError("member '" + member.name + "': its end nodes coincide; no chord");
        }
        direction = scaled(chord, 1.0 / length);
    } else {
        kernel::Vec3 parent_position{};
        kernel::Quat parent_orientation{1.0, 0.0, 0.0, 0.0};
        if (chain.parent_body.has_value()) {
            if (chain.parent_body.value() >= bodies.size()) {
                throw LimitsError("member '" + member.name + "': parent body " +
                                  std::to_string(chain.parent_body.value()) + " outside the published bodies");
            }
            parent_position = bodies[chain.parent_body.value()].position;
            parent_orientation = bodies[chain.parent_body.value()].orientation;
        }
        origin = plus(parent_position, rotate(parent_orientation, chain.position));
        direction = rotate(multiply(parent_orientation, chain.orientation), {1.0, 0.0, 0.0});
    }
    double peak = distance_from_line(tip, origin, direction);
    for (std::size_t k = 0; k < chain.count; ++k) {
        peak = std::max(peak, distance_from_line(bodies[chain.first_body + k].position, origin, direction));
    }
    return peak;
}

std::vector<double> node_moments(std::span<const kernel::BodyState> bodies, const MemberLimits& member,
                                 std::span<const double> offsets) {
    if (!member.nodes.has_value()) {
        throw LimitsError("member '" + member.name + "': no node poses (built rigid); no node moments");
    }
    const NodeChain& chain = member.nodes.value();
    if (chain.count == 0 || chain.first_body + chain.count > bodies.size()) {
        throw LimitsError("member '" + member.name + "': nodes " + std::to_string(chain.first_body) + " + " +
                          std::to_string(chain.count) + " outside the " + std::to_string(bodies.size()) +
                          " published bodies");
    }
    kernel::Quat previous = bodies[chain.first_body].orientation;
    std::size_t first = 1;
    if (chain.root != SupportCondition::pinned) {
        kernel::Quat parent{1.0, 0.0, 0.0, 0.0};
        if (chain.parent_body.has_value()) {
            if (chain.parent_body.value() >= bodies.size()) {
                throw LimitsError("member '" + member.name + "': parent body " +
                                  std::to_string(chain.parent_body.value()) + " outside the published bodies");
            }
            parent = bodies[chain.parent_body.value()].orientation;
        }
        previous = multiply(parent, chain.orientation);
        first = 0;
    }
    std::vector<double> moments;
    moments.reserve(chain.count - first);
    if (!offsets.empty() && offsets.size() != chain.count - first) {
        throw LimitsError("member '" + member.name + "': " + std::to_string(offsets.size()) + " spring offsets for " +
                          std::to_string(chain.count - first) + " springs");
    }
    for (std::size_t k = first; k < chain.count; ++k) {
        const kernel::Quat& current = bodies[chain.first_body + k].orientation;
        double rotation = bending_angle(previous, current);
        if (!offsets.empty()) {
            rotation = rotation - offsets[k - first];
        }
        moments.push_back(member.node_stiffness * rotation);
        previous = current;
    }
    return moments;
}

namespace {

constexpr double couple_floor = 1e-9;

struct Line {
    double x = 0.0;
    double m = 0.0;
    double slope = 0.0;
};

double at(const Line& line, double s) {
    return line.m + line.slope * (s - line.x);
}

Line through(const std::vector<double>& x, const std::vector<double>& m, std::size_t first) {
    return Line{x[first + 1], m[first + 1], (m[first + 1] - m[first]) / (x[first + 1] - x[first])};
}

struct Placed {
    double s = 0.0;
    std::size_t k = 0;
    double force = 0.0;
    double couple = 0.0;
};

kernel::Vec3 cross(const kernel::Vec3& a, const kernel::Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

}  // namespace

std::vector<CarriedAt> carried_at(std::span<const kernel::BodyState> bodies,
                                  std::span<const kernel::JointForce> joints, const MemberLimits& member) {
    if (!member.nodes.has_value()) {
        throw LimitsError("member '" + member.name + "': no node poses (built rigid); no carried loads");
    }
    const NodeChain& chain = member.nodes.value();
    kernel::Vec3 parent_position{};
    kernel::Quat parent_orientation{1.0, 0.0, 0.0, 0.0};
    if (chain.parent_body.has_value()) {
        if (chain.parent_body.value() >= bodies.size()) {
            throw LimitsError("member '" + member.name + "': parent body " +
                              std::to_string(chain.parent_body.value()) + " outside the published bodies");
        }
        parent_position = bodies[chain.parent_body.value()].position;
        parent_orientation = bodies[chain.parent_body.value()].orientation;
    }
    const kernel::Vec3 origin = plus(parent_position, rotate(parent_orientation, chain.position));
    const kernel::Quat turn = multiply(parent_orientation, chain.orientation);
    const kernel::Vec3 x = rotate(turn, {1.0, 0.0, 0.0});
    const kernel::Vec3 y = rotate(turn, {0.0, 1.0, 0.0});
    const kernel::Vec3 z = rotate(turn, {0.0, 0.0, 1.0});
    std::vector<CarriedAt> out;
    out.reserve(chain.carried.size());
    for (const CarriedJoint& joint : chain.carried) {
        if (joint.body >= bodies.size() || joint.joint >= joints.size()) {
            throw LimitsError("member '" + member.name + "': carried body " + std::to_string(joint.body) +
                              " or joint " + std::to_string(joint.joint) + " outside the published outputs");
        }
        const kernel::Vec3& child = bodies[joint.body].position;
        const kernel::Wrench& reaction = joints[joint.joint].reaction;
        const kernel::Vec3 force = scaled(reaction.force, -1.0);
        const double s = dot(minus(child, origin), x);
        const kernel::Vec3 foot = plus(origin, scaled(x, s));
        const kernel::Vec3 moment = plus(scaled(reaction.moment, -1.0), cross(minus(child, foot), force));
        out.push_back(CarriedAt{s, dot(force, y), dot(moment, z)});
    }
    return out;
}

CarriedReads carried_reads(const kernel::MemberEndLoads& loads, std::span<const double> moments,
                           const MemberLimits& member, std::span<const CarriedAt> carried) {
    CarriedReads out;
    if (carried.empty() || !member.nodes.has_value()) {
        return out;
    }
    const std::size_t springs = moments.size();
    std::vector<double> x;
    std::vector<double> m;
    x.reserve(springs + 2);
    m.reserve(springs + 2);
    x.push_back(0.0);
    m.push_back(-loads.start.moment[2]);
    for (std::size_t j = 0; j < springs; ++j) {
        x.push_back((static_cast<double>(j) + 0.5) * member.segment_length);
        m.push_back(moments[j]);
    }
    x.push_back(member.length);
    m.push_back(loads.end.moment[2]);
    const std::size_t last = x.size() - 1;
    std::vector<Placed> placed;
    placed.reserve(carried.size());
    for (const CarriedAt& load : carried) {
        const double s = std::clamp(load.position, 0.0, member.length);
        std::size_t k = 0;
        while (k + 1 < last && x[k + 1] <= s) {
            ++k;
        }
        placed.push_back(Placed{s, k, load.force, load.couple});
    }
    std::stable_sort(placed.begin(), placed.end(), [](const Placed& a, const Placed& b) { return a.s < b.s; });
    const bool pinned = member.nodes.value().root == SupportCondition::pinned;
    std::size_t first = 0;
    while (first < placed.size()) {
        std::size_t final = first;
        while (final + 1 < placed.size() && placed[final + 1].k - placed[final].k <= 2) {
            ++final;
        }
        const Placed& head = placed[first];
        const Placed& tail = placed[final];
        std::optional<Line> left;
        if (head.k >= 1) {
            left = through(x, m, head.k - 1);
        } else if (pinned) {
            left = Line{0.0, m[0], loads.start.force[1]};
        }
        Line right{member.length, m[last], -loads.end.force[1]};
        if (tail.k + 3 <= last) {
            right = through(x, m, tail.k + 2);
        }
        if (left.has_value()) {
            out.reads.push_back(CarriedMoment{at(left.value(), head.s), head.s});
        }
        out.reads.push_back(CarriedMoment{at(right, tail.s), tail.s});
        const std::size_t size = final - first + 1;
        bool coupled = false;
        double forces = 0.0;
        double couples = 0.0;
        double stub = 0.0;
        for (std::size_t i = first; i <= final; ++i) {
            const Placed& load = placed[i];
            if (std::abs(load.couple) > couple_floor * std::abs(load.force) * member.length) {
                coupled = true;
            }
            forces += std::abs(load.force);
            couples += std::abs(load.couple);
            if (load.k == 0) {
                stub += std::abs(load.force) * member.segment_length / 8.0 + std::abs(load.couple);
            }
        }
        if (!left.has_value() || size > 2 || (size == 2 && coupled)) {
            const double a = x[head.k];
            const double b = x[std::min(tail.k + 2, last)];
            double value = forces * (b - a) / 4.0 + couples;
            if (!left.has_value()) {
                value += stub;
            }
            out.bound = std::max(out.bound, value);
        }
        first = final + 1;
    }
    return out;
}

double peak_normal_stress(const kernel::MemberEndLoads& loads, std::span<const kernel::BodyState> bodies,
                          const MemberLimits& member, std::span<const double> offsets,
                          std::span<const CarriedAt> carried) {
    double peak = std::max(end_stress(loads.start, member), end_stress(loads.end, member));
    if (!member.nodes.has_value()) {
        return peak;
    }
    const std::vector<double> moments = node_moments(bodies, member, offsets);
    const double axial = std::max(std::abs(loads.start.force[0]), std::abs(loads.end.force[0])) / member.area;
    const auto interior = [&](double share, double moment_z) {
        const double moment_y = loads.start.moment[1] * (1.0 - share) - loads.end.moment[1] * share;
        return axial + std::abs(moment_y) * member.c_z / member.i_y + std::abs(moment_z) * member.c_y / member.i_z;
    };
    for (std::size_t j = 0; j < moments.size(); ++j) {
        const double share = (static_cast<double>(j) + 0.5) * member.segment_length / member.length;
        peak = std::max(peak, interior(share, moments[j]));
    }
    if (!member.nodes.value().carries) {
        return peak;
    }
    const CarriedReads reads = carried_reads(loads, moments, member, carried);
    for (const CarriedMoment& read : reads.reads) {
        peak = std::max(peak, interior(read.position / member.length, read.value));
    }
    return peak + (reads.bound * member.c_y / member.i_z);
}

double start_node_moment(const kernel::Wrench& start, double segment_length) {
    return -start.moment[2] + 0.5 * segment_length * start.force[1];
}

double end_node_moment(const kernel::Wrench& end, double segment_length) {
    return end.moment[2] + 0.5 * segment_length * end.force[1];
}

}  // namespace mechsim::integrity
