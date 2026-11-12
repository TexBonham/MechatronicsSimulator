#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <utility>
#include <vector>

#include "mechsim/engine/collision.hpp"

namespace mechsim::engine {

// Soft contact, integrated implicitly, with regularized Coulomb friction (ADR 0002, ENG-020). Per contact point, with
// n from a to b, depth d, relative velocity u = v_b(p) - v_a(p):
//   lambda = K_n d - C_n (n . u), clamped at zero (never adhesive; a clamped contact contributes nothing);
//   u_t = u - n (n . u); friction F_t = -mu lambda u_t / max(|u_t|, v_s) (viscous below the slip velocity v_s);
//   force on b: lambda n + F_t; on a: the opposite, both at p.
// Linearized for the step's (H + h D + h^2 K) with G = J_b(p) - J_a(p) (3 x nv point Jacobians):
//   K += G^T (K_n n n^T) G;
//   D += G^T (C_n n n^T + mu lambda / v_s P) G            when |u_t| < v_s, P = I - n n^T;
//   D += G^T (C_n n n^T + mu lambda / |u_t| (P - t t^T)) G otherwise, t = u_t / |u_t|.
// K_n is per contact point: a box flat on a plane carries 4 K_n.

struct ContactParameters {
    double stiffness = 1e5;  // K_n, N/m, finite > 0
    double damping = 1e3;    // C_n, N s/m, finite >= 0
    double friction = 0.5;   // mu, finite >= 0
};

// Overrides the default for one unordered material pair.
struct MaterialPair {
    std::string material_a;
    std::string material_b;
    ContactParameters parameters;
};

// One contact geometry in the neutral model's terms (sizes are full lengths, as in mechsim/physics/neutral.py).
struct Geometry {
    std::string kind = "sphere";                                     // shape name; see shape_kind()
    Eigen::Index body = -1;                                          // -1 is the world
    Eigen::Vector3d position = Eigen::Vector3d::Zero();               // shape frame in the body frame, m
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // shape frame relative to the body, unit
    double radius = 0.0;                                             // sphere, capsule, cylinder, m
    double length = 0.0;                                             // capsule cap-center distance, cylinder, m
    Eigen::Vector3d size = Eigen::Vector3d::Zero();                   // box edge lengths, m
    std::vector<Eigen::Vector3d> vertices;                           // convex hull, shape frame, m
    std::string material;
};

struct ContactModel {
    static constexpr double default_slip_velocity = 1e-4;  // m/s, v_s

    std::vector<Geometry> geometries;
    ContactParameters default_parameters;
    std::vector<MaterialPair> pairs;
    std::vector<std::pair<Eigen::Index, Eigen::Index>> exclusions;  // body pairs never in contact; -1 is the world
    double slip_velocity = default_slip_velocity;                   // finite > 0
};

// Shape of geometry `index`; throws InvalidModelError naming an unknown kind (ENG-017) or a bad size.
[[nodiscard]] Shape to_shape(const Geometry& geometry, Eigen::Index index);
// Validates every geometry, parameter set, exclusion and the slip velocity against a model of `bodies` bodies; planes
// only on the world (the neutral model's rule).
void validate_contact_model(const ContactModel& model, Eigen::Index bodies);
// Parameters for a material pair (either order), else the default.
[[nodiscard]] const ContactParameters& contact_parameters(const ContactModel& model, const std::string& material_a,
                                                          const std::string& material_b);
[[nodiscard]] bool excluded(const ContactModel& model, Eigen::Index body_a, Eigen::Index body_b);

struct ContactForce {
    bool active = false;
    double normal_force = 0.0;                              // lambda, N
    Eigen::Vector3d force_on_b = Eigen::Vector3d::Zero();   // world, N
    Eigen::Matrix3d stiffness = Eigen::Matrix3d::Zero();    // point space, N/m
    Eigen::Matrix3d damping = Eigen::Matrix3d::Zero();      // point space, N s/m
};

[[nodiscard]] ContactForce contact_force(const ContactParameters& parameters, double slip_velocity,
                                         const ContactPoint& contact, const Eigen::Vector3d& relative_velocity);

// Payload attachment (ENG-035): a weld of `body` to `carrier` (-1 the world) at the relative pose captured when it is
// enabled. Implicit 6-DOF spring-damper at the payload's center of mass, critically damped at `frequency`:
// k = m w^2, c = 2 m w, rotational K = w^2 I_c, C = 2 w I_c (I_c about the com, world), w = 2 pi frequency. Static
// sag g / w^2 (2.5e-7 m at 1 kHz); the weld force is exact at rest whatever w.
struct Attachment {
    static constexpr double default_frequency = 1000.0;  // Hz

    Eigen::Index body = 0;
    Eigen::Index carrier = -1;
    bool attached = false;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();         // payload com in the carrier frame, m
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();     // payload body frame to carrier frame
    double frequency = default_frequency;                       // Hz, finite > 0
};

[[nodiscard]] double angular_frequency(const Attachment& attachment);  // w = 2 pi frequency, rad/s

struct PayloadOutput {
    Eigen::Index body = 0;
    Eigen::Index carrier = -1;  // -1 once released
    bool attached = false;
    Eigen::Vector3d force = Eigen::Vector3d::Zero();  // attachment on the payload, world, N; zero when released
};

}  // namespace mechsim::engine
