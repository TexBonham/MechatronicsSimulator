#include "mechsim/integrity/beam.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

#include "mechsim/integrity/limits.hpp"
#include "mechsim/kernel/physics_types.hpp"

namespace mechsim::integrity {

namespace {

constexpr std::size_t grid_points = 201;
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

double static_deflection(const kernel::MemberEndLoads& loads, const MemberLimits& member) {
    const double length = member.length;
    std::vector<double> grid;
    grid.reserve(grid_points + member.supports.size());
    for (std::size_t k = 0; k < grid_points; ++k) {
        grid.push_back(length * static_cast<double>(k) / static_cast<double>(grid_points - 1));
    }
    for (const Support& support : member.supports) {
        grid.push_back(support.at);
    }
    std::sort(grid.begin(), grid.end());
    grid.erase(std::unique(grid.begin(), grid.end()), grid.end());
    const auto nearest = [&grid](double at) {
        std::size_t best = 0;
        for (std::size_t k = 1; k < grid.size(); ++k) {
            if (std::abs(grid[k] - at) < std::abs(grid[best] - at)) {
                best = k;
            }
        }
        return grid[best];
    };
    // Least squares for C0 + C1 x over the support conditions (v = 0 at each, v' = 0 at a clamp): the normal
    // equations of the n x 2 condition matrix, which is beam.py's pinv when it has rank 2.
    double a00 = 0.0;
    double a01 = 0.0;
    double a11 = 0.0;
    for (const Support& support : member.supports) {
        a00 += 1.0;
        a01 += support.at;
        a11 += support.at * support.at;
        if (support.condition == SupportCondition::clamped) {
            a11 += 1.0;
        }
    }
    const double det = a00 * a11 - a01 * a01;
    if (!(std::abs(det) > 1e-12 * std::max(1.0, a11 * a00))) {
        throw LimitsError("member '" + member.name + "': its supports leave it free to move; no deflection");
    }
    std::array<Plane, 2> planes{plane_y(loads, length), plane_z(loads, length)};
    std::array<std::array<double, 2>, 2> fit{};
    for (std::size_t p = 0; p < 2; ++p) {
        double b0 = 0.0;
        double b1 = 0.0;
        for (const Support& support : member.supports) {
            const double at = nearest(support.at);
            const double r = -planes[p].value(at);
            b0 += r;
            b1 += support.at * r;
            if (support.condition == SupportCondition::clamped) {
                b1 += -planes[p].slope(at);
            }
        }
        fit[p][0] = (a11 * b0 - a01 * b1) / det;
        fit[p][1] = (a00 * b1 - a01 * b0) / det;
    }
    const double ei_z = member.elastic_modulus * member.i_z;
    const double ei_y = member.elastic_modulus * member.i_y;
    double peak = 0.0;
    for (const double x : grid) {
        const double v = (planes[0].value(x) + fit[0][0] + fit[0][1] * x) / ei_z;
        const double w = (planes[1].value(x) + fit[1][0] + fit[1][1] * x) / ei_y;
        peak = std::max(peak, std::hypot(v, w));
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
