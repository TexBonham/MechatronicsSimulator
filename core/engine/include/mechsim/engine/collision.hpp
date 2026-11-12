#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace mechsim::engine {

// In-house collision detection (ADR 0002, ENG-016 to ENG-019). No collision library is linked (ENG-018).
//
// Narrow phase, a to b (normal points from a to b; depth > 0 is penetration):
//   sphere-sphere, sphere-capsule, capsule-capsule, sphere-box: closed form, one contact.
//   any shape-plane: closed form per support feature (sphere 1, capsule 2 cap centers, box 8 corners, hull every
//     vertex, cylinder 4 rim points per end cap, the first at the exact deepest rim point), one contact per
//     penetrating feature.
//   every other pair (box-box, box-capsule, hull-*, cylinder-*): GJK for overlap, EPA for depth, normal and witness
//     points, one contact. Cylinder is exact (its own support function), not a capsule or box approximation.
// The contact point is the midpoint of the two witness points (deepest point of a inside b, of b inside a).

enum class ShapeKind : std::uint8_t { Sphere, Capsule, Box, Cylinder, Plane, ConvexHull };

// The neutral model's shape names: sphere, capsule, box, cylinder, plane, convex_hull. Any other name throws
// InvalidModelError naming it (ENG-017).
[[nodiscard]] ShapeKind shape_kind(const std::string& name);
[[nodiscard]] const char* shape_name(ShapeKind kind);

struct Shape {
    ShapeKind kind = ShapeKind::Sphere;
    double radius = 0.0;                                   // sphere, capsule, cylinder, m
    double half_length = 0.0;                              // capsule cap centers and cylinder caps at +-z, m
    Eigen::Vector3d half_extents = Eigen::Vector3d::Zero();  // box, m
    std::vector<Eigen::Vector3d> vertices;                 // convex hull, shape frame, m
};

// Shape frame in world: x_world = position + rotation * x_shape. Plane: half-space z <= 0 of the shape frame.
struct Pose {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

struct ContactPoint {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();   // world, m
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();  // unit, world, from a to b
    double depth = 0.0;                                // > 0, m
};

struct Aabb {
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
};

// Validates sizes (finite, > 0; hull at least 4 finite vertices); throws InvalidModelError with `label` prefixed.
void validate_shape(const Shape& shape, const std::string& label);

[[nodiscard]] Aabb bounds(const Shape& shape, const Pose& pose);  // plane: unbounded
[[nodiscard]] bool overlaps(const Aabb& a, const Aabb& b);
// Broad phase: index pairs (i < j) whose boxes overlap, in ascending (i, j) order.
[[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>> broad_phase(const std::vector<Aabb>& boxes);

// Narrow phase; empty when the shapes do not penetrate. Plane-plane is always empty.
[[nodiscard]] std::vector<ContactPoint> collide(const Shape& a, const Pose& pa, const Shape& b, const Pose& pb);

// Farthest point of the shape along a world direction (GJK support); the plane has none and throws.
[[nodiscard]] Eigen::Vector3d support(const Shape& shape, const Pose& pose, const Eigen::Vector3d& direction);

}  // namespace mechsim::engine
