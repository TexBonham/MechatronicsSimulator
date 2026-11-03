#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

// Spatial algebra after Featherstone, Rigid Body Dynamics Algorithms (2008): 6-vectors are angular part first,
// motion vectors [omega; v] and force vectors [n; f], both in Pluecker coordinates.
namespace mechsim::engine::spatial {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix6Xd = Eigen::Matrix<double, 6, Eigen::Dynamic>;

inline Eigen::Matrix3d skew(const Eigen::Vector3d& a) {
    Eigen::Matrix3d m;
    m << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
    return m;
}

// Motion cross product v x m.
inline Vector6d cross_motion(const Vector6d& v, const Vector6d& m) {
    Vector6d out;
    out.head<3>() = v.head<3>().cross(m.head<3>());
    out.tail<3>() = v.head<3>().cross(m.tail<3>()) + v.tail<3>().cross(m.head<3>());
    return out;
}

// Force cross product v x* f.
inline Vector6d cross_force(const Vector6d& v, const Vector6d& f) {
    Vector6d out;
    out.head<3>() = v.head<3>().cross(f.head<3>()) + v.tail<3>().cross(f.tail<3>());
    out.tail<3>() = v.head<3>().cross(f.tail<3>());
    return out;
}

// Spatial inertia about the body origin from mass (kg), center of mass c (m) and inertia about c (kg m^2).
inline Matrix6d inertia(double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia_com) {
    const Eigen::Matrix3d c = skew(com);
    Matrix6d out;
    out.topLeftCorner<3, 3>() = inertia_com + mass * c * c.transpose();
    out.topRightCorner<3, 3>() = mass * c;
    out.bottomLeftCorner<3, 3>() = mass * c.transpose();
    out.bottomRightCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();
    return out;
}

// Pluecker transform B_X_A: rotation E takes A coordinates to B coordinates, r is B's origin in A coordinates (m).
struct Transform {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    [[nodiscard]] Vector6d motion(const Vector6d& m) const {
        Vector6d out;
        out.head<3>() = rotation * m.head<3>();
        out.tail<3>() = rotation * (m.tail<3>() - translation.cross(m.head<3>()));
        return out;
    }

    [[nodiscard]] Vector6d force(const Vector6d& f) const {
        Vector6d out;
        out.head<3>() = rotation * (f.head<3>() - translation.cross(f.tail<3>()));
        out.tail<3>() = rotation * f.tail<3>();
        return out;
    }

    // A_X_B applied to a motion vector in B coordinates.
    [[nodiscard]] Vector6d inverse_motion(const Vector6d& m) const {
        Vector6d out;
        out.head<3>() = rotation.transpose() * m.head<3>();
        out.tail<3>() = rotation.transpose() * m.tail<3>() + translation.cross(out.head<3>());
        return out;
    }

    // A_X*_B applied to a force vector in B coordinates (the transpose of motion()).
    [[nodiscard]] Vector6d inverse_force(const Vector6d& f) const {
        Vector6d out;
        out.tail<3>() = rotation.transpose() * f.tail<3>();
        out.head<3>() = rotation.transpose() * f.head<3>() + translation.cross(out.tail<3>());
        return out;
    }

    [[nodiscard]] Matrix6d motion_matrix() const {
        Matrix6d out = Matrix6d::Zero();
        out.topLeftCorner<3, 3>() = rotation;
        out.bottomRightCorner<3, 3>() = rotation;
        out.bottomLeftCorner<3, 3>() = -rotation * skew(translation);
        return out;
    }

    // (B_X_A) * (A_X_C) = B_X_C.
    [[nodiscard]] Transform then_from(const Transform& a_from_c) const {
        Transform out;
        out.rotation = rotation * a_from_c.rotation;
        out.translation = a_from_c.translation + a_from_c.rotation.transpose() * translation;
        return out;
    }
};

}  // namespace mechsim::engine::spatial
