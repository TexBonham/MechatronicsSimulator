#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <vector>

#include "mechsim/engine/collision.hpp"
#include "mechsim/engine/contact.hpp"
#include "mechsim/engine/multibody_model.hpp"

namespace mechsim::engine::test {

inline double angle_between(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return std::atan2(a.cross(b).norm(), a.dot(b));
}

inline Shape sphere_shape(double radius) {
    Shape s;
    s.kind = ShapeKind::Sphere;
    s.radius = radius;
    return s;
}

inline Shape capsule_shape(double radius, double half_length) {
    Shape s;
    s.kind = ShapeKind::Capsule;
    s.radius = radius;
    s.half_length = half_length;
    return s;
}

inline Shape box_shape(const Eigen::Vector3d& half_extents) {
    Shape s;
    s.kind = ShapeKind::Box;
    s.half_extents = half_extents;
    return s;
}

inline Shape cylinder_shape(double radius, double half_length) {
    Shape s;
    s.kind = ShapeKind::Cylinder;
    s.radius = radius;
    s.half_length = half_length;
    return s;
}

inline Shape plane_shape() {
    Shape s;
    s.kind = ShapeKind::Plane;
    return s;
}

inline Shape hull_shape(std::vector<Eigen::Vector3d> vertices) {
    Shape s;
    s.kind = ShapeKind::ConvexHull;
    s.vertices = std::move(vertices);
    return s;
}

// Octahedron with vertices at +-a on each axis.
inline Shape octahedron(double a) {
    return hull_shape({{a, 0, 0}, {-a, 0, 0}, {0, a, 0}, {0, -a, 0}, {0, 0, a}, {0, 0, -a}});
}

inline Pose pose_at(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity()) {
    Pose p;
    p.position = position;
    p.rotation = rotation;
    return p;
}

inline Eigen::Matrix3d rotation_about(const Eigen::Vector3d& axis, double angle) {
    return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

// Free-floating body with a uniform-density inertia supplied by the caller.
inline Body free_body(std::string name, double mass, const Eigen::Matrix3d& inertia) {
    Body body;
    body.name = std::move(name);
    body.parent = -1;
    body.joint.type = JointType::Free;
    body.mass = mass;
    body.inertia = inertia;
    return body;
}

inline Eigen::Matrix3d solid_sphere_inertia(double mass, double radius) {
    return Eigen::Matrix3d::Identity() * (0.4 * mass * radius * radius);
}

inline Eigen::Matrix3d solid_box_inertia(double mass, const Eigen::Vector3d& half) {
    const Eigen::Vector3d full = 2.0 * half;
    return Eigen::Vector3d(mass * (full.y() * full.y() + full.z() * full.z()) / 12.0,
                           mass * (full.x() * full.x() + full.z() * full.z()) / 12.0,
                           mass * (full.x() * full.x() + full.y() * full.y()) / 12.0)
        .asDiagonal();
}

inline Geometry geometry(std::string kind, Eigen::Index body) {
    Geometry g;
    g.kind = std::move(kind);
    g.body = body;
    g.material = "steel";
    return g;
}

inline Geometry floor_plane(const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity(),
                            const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) {
    Geometry g = geometry("plane", -1);
    g.orientation = Eigen::Quaterniond(rotation);
    g.position = position;
    return g;
}

}  // namespace mechsim::engine::test
