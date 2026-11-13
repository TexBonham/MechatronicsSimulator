#include "mechsim/integrity/beam.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include "mechsim/integrity/limits.hpp"
#include "mechsim/kernel/physics_types.hpp"

namespace mechsim::integrity {

namespace {

constexpr double present_tolerance = 1e-9;

// One bending plane in I1's form (beam.py `_plane`): E I v'' = M(x) = moment0 + shear0 x + load max(x - a, 0), the
// interior load -(F_start + F_end) resolved to its statically equivalent point a = balance / load, clipped to [0, L];
// absent (zero load at a = L) when it is below 1e-9 of the larger end shear.
struct Plane {
    double moment0 = 0.0;
    double shear0 = 0.0;
    double load = 0.0;
    double where = 0.0;
    bool present = false;

    [[nodiscard]] double moment(double x) const {
        return moment0 + shear0 * x + load * std::max(x - where, 0.0);
    }
    [[nodiscard]] double value(double x) const {
        const double beyond = std::max(x - where, 0.0);
        return moment0 * x * x / 2.0 + shear0 * x * x * x / 6.0 + load * beyond * beyond * beyond / 6.0;
    }
    [[nodiscard]] double slope(double x) const {
        const double beyond = std::max(x - where, 0.0);
        return moment0 * x + shear0 * x * x / 2.0 + load * beyond * beyond / 2.0;
    }
};

Plane plane(double moment0, double shear0, double shear1, double balance, double length) {
    Plane p;
    p.moment0 = moment0;
    p.shear0 = shear0;
    const double load = -(shear0 + shear1);
    const double scale = std::max(std::abs(shear0), std::abs(shear1));
    p.present = std::abs(load) > present_tolerance * scale;
    p.where = length;
    if (p.present) {
        p.load = load;
        p.where = std::clamp(balance / load, 0.0, length);
    }
    return p;
}

// The bending plane about z (deflection v along y, I_z, fibre c_y) and about y (w along z, I_y, fibre c_z).
Plane plane_y(const kernel::MemberEndLoads& loads, double length) {
    const kernel::Wrench& s = loads.start;
    const kernel::Wrench& e = loads.end;
    return plane(-s.moment[2], s.force[1], e.force[1], -(s.moment[2] + e.moment[2] + length * e.force[1]), length);
}

Plane plane_z(const kernel::MemberEndLoads& loads, double length) {
    const kernel::Wrench& s = loads.start;
    const kernel::Wrench& e = loads.end;
    return plane(s.moment[1], s.force[2], e.force[2], s.moment[1] + e.moment[1] - length * e.force[2], length);
}

}  // namespace

double end_stress(const kernel::Wrench& wrench, const MemberLimits& member) {
    return std::abs(wrench.force[0]) / member.area + std::abs(wrench.moment[1]) * member.c_z / member.i_y +
           std::abs(wrench.moment[2]) * member.c_y / member.i_z;
}

double peak_normal_stress(const kernel::MemberEndLoads& loads, const MemberLimits& member) {
    double peak = std::max(end_stress(loads.start, member), end_stress(loads.end, member));
    const Plane y = plane_y(loads, member.length);
    const Plane z = plane_z(loads, member.length);
    const double axial = std::max(std::abs(loads.start.force[0]), std::abs(loads.end.force[0])) / member.area;
    for (const Plane* resolved : {&y, &z}) {
        if (!resolved->present) {
            continue;
        }
        const double x = resolved->where;
        const double stress = axial + std::abs(z.moment(x)) * member.c_z / member.i_y +
                              std::abs(y.moment(x)) * member.c_y / member.i_z;
        peak = std::max(peak, stress);
    }
    return peak;
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

double start_node_moment(const kernel::Wrench& start, double segment_length) {
    return -start.moment[2] + 0.5 * segment_length * start.force[1];
}

double end_node_moment(const kernel::Wrench& end, double segment_length) {
    return end.moment[2] + 0.5 * segment_length * end.force[1];
}

}  // namespace mechsim::integrity
