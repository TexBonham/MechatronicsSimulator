#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

#include "mechsim/engine/contact.hpp"
#include "mechsim/engine/dynamics_provider.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_model.hpp"
#include "mechsim/engine/spatial.hpp"
#include "mechsim/engine/state.hpp"
#include "mechsim/engine/step_health.hpp"
#include "mechsim/engine/transmission.hpp"

namespace mechsim::engine {

struct BodyOutput {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();                // body origin, world, m
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // body to world, unit
    spatial::Vector6d spatial_velocity = spatial::Vector6d::Zero();    // Pluecker, world coordinates and origin
    Eigen::Vector3d origin_velocity = Eigen::Vector3d::Zero();         // body origin, world, m/s
};

struct JointOutput {
    Eigen::VectorXd generalized_force;                                 // S^T f per joint DOF, N or N m
    spatial::Vector6d reaction_wrench = spatial::Vector6d::Zero();     // parent on child: [moment about child
                                                                       // origin; force], world-aligned, N m; N
};

struct ConstraintOutput {
    Eigen::VectorXd impulse;  // on body_a (body_b gets the opposite): [point (N s); hinge rows (N m s)]
    Eigen::VectorXd force;    // impulse / h, N and N m
    double translational_error = 0.0;  // after projection, m
    double rotational_error = 0.0;     // after projection, rad
};

// One contact point at the step's start state (ENG-020, ENG-027); body -1 is the world.
struct ContactOutput {
    Eigen::Index body_a = -1;
    Eigen::Index body_b = -1;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();  // world, m
    Eigen::Vector3d force = Eigen::Vector3d::Zero();  // on body_a, world, N
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();  // unit, world, from a to b
    double depth = 0.0;                                // penetration, m
};

// Mid-run model changes, as the adapter reads them; shaped as E2's ModelChangeEvent so the merge unifies payload
// release with release_load (ENG-034, ENG-035).
enum class ModelChange : std::uint8_t {
    PayloadAttached,  // target payload body, index carrier (-1 the world)
    PayloadReleased,  // target payload body, index the carrier it left
    PayloadMass       // target payload body, value mass (kg)
};

struct ModelChangeEvent {
    ModelChange kind = ModelChange::PayloadAttached;
    Eigen::Index target = -1;
    Eigen::Index index = -1;
    double value = 0.0;
    double limit = 0.0;
    double time = 0.0;  // s: a requested change takes effect at this time (the next step's start)
};

struct MultibodyOutputs {
    std::vector<BodyOutput> bodies;
    std::vector<JointOutput> joints;  // one per body, the joint to its parent
    std::vector<ConstraintOutput> constraints;
    std::vector<ContactOutput> contacts;
    std::vector<TransmissionOutput> transmissions;  // at the end state, slip offset advanced by the step
    std::vector<PayloadOutput> payloads;            // one per attachment, at the step's start state (ENG-035)
    std::vector<ModelChangeEvent> events;           // changes applied in this step
    double time = 0.0;  // end of the step, s
};

struct ConstraintError {
    Eigen::VectorXd residual;    // [x_a - x_b (m); hinge u_k . b_axis (rad, small-angle)]
    double translational = 0.0;  // |x_a - x_b|, m
    double rotational = 0.0;     // angle between the hinge axes, rad; 0 for Point
};

// Reduced-coordinate rigid multibody (ADR 0013): H by CRBA, c by RNEA, joint springs in K and D (ENG-051), loops by
// velocity-level block Gauss-Seidel plus Gauss-Newton position projection. Step it through step() so inputs clear and outputs refresh.
class MultibodyProvider final : public DynamicsProvider {
public:
    explicit MultibodyProvider(MultibodyModel model, MultibodyConfig config = {});

    [[nodiscard]] Eigen::Index position_size() const override;
    [[nodiscard]] Eigen::Index velocity_size() const override;
    void evaluate(double t, const Eigen::VectorXd& q, const Eigen::VectorXd& v, DynamicsTerms& terms) const override;
    [[nodiscard]] Eigen::VectorXd integrate_positions(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                      double h) const override;
    int solve_constraints(double t, double h, const Eigen::VectorXd& q, const Eigen::MatrixXd& system,
                          Eigen::VectorXd& v) const override;
    double project_positions(double t, Eigen::VectorXd& q, Eigen::VectorXd& v) const override;

    [[nodiscard]] const MultibodyModel& model() const noexcept;
    [[nodiscard]] const MultibodyConfig& config() const noexcept;
    [[nodiscard]] Eigen::Index position_offset(Eigen::Index body) const;
    [[nodiscard]] Eigen::Index velocity_offset(Eigen::Index body) const;
    [[nodiscard]] Eigen::VectorXd neutral_positions() const;  // zero coordinates, identity quaternions
    [[nodiscard]] std::vector<ConstraintError> constraint_errors(const Eigen::VectorXd& q) const;
    [[nodiscard]] std::vector<BodyOutput> body_states(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const;

    // Inputs (ENG-028), applied to the next step only.
    void set_generalized_forces(const Eigen::VectorXd& tau);  // size nv, N or N m
    void apply_body_wrench(Eigen::Index body, const Eigen::Vector3d& torque, const Eigen::Vector3d& force);
    void clear_inputs();
    // Motor shaft torque on a transmission's input shaft for the next step, N m (ENG-048); adds to the shaft DOF.
    void set_shaft_torque(Eigen::Index transmission, double torque);
    // Persistent until changed; takes effect at the next step (ENG-046). Range [0, 1].
    void set_slip(Eigen::Index transmission, double slip);
    [[nodiscard]] double slip(Eigen::Index transmission) const;
    // Transmission state at (q, v) with the current slip, slip offsets and pending inputs.
    [[nodiscard]] std::vector<TransmissionOutput> transmission_states(const Eigen::VectorXd& q,
                                                                      const Eigen::VectorXd& v) const;

    // Contact (ENG-016 to ENG-021): validated against this model; replaces any earlier contact model.
    void set_contact_model(ContactModel contacts);
    [[nodiscard]] const ContactModel& contact_model() const noexcept;
    // Contacts and their forces at (q, v) with the current contact model.
    [[nodiscard]] std::vector<ContactOutput> contacts(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const;
    // Payload change (ENG-035), effective from the next step. Mass rescales the body's inertia by mass / old mass.
    void set_body_mass(Eigen::Index body, double mass);
    // Welds body to carrier (-1 the world) at their relative pose in q, or releases it (attached false).
    void set_attachment(Eigen::Index body, Eigen::Index carrier, bool attached, const Eigen::VectorXd& q,
                        double frequency = WeldAttachment::default_frequency);
    [[nodiscard]] const std::vector<WeldAttachment>& attachments() const noexcept;

    StepHealth step(Integrator& integrator, State& state);
    [[nodiscard]] const MultibodyOutputs& outputs() const noexcept;

    struct Kinematics;
    struct ConstraintRows;

private:
    [[nodiscard]] Kinematics kinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const;
    [[nodiscard]] Eigen::MatrixXd mass_matrix(const Kinematics& kin) const;
    Eigen::VectorXd inverse_dynamics(const Kinematics& kin, const Eigen::VectorXd& qdd,
                                     const spatial::Vector6d& base_acceleration,
                                     const std::vector<spatial::Vector6d>& external, bool velocity_terms,
                                     std::vector<spatial::Vector6d>& body_forces) const;
    [[nodiscard]] std::vector<spatial::Vector6d> applied_body_wrenches(const Kinematics& kin) const;
    [[nodiscard]] Eigen::VectorXd joint_spring_forces(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const;
    [[nodiscard]] spatial::Matrix6Xd world_jacobian(const Kinematics& kin, Eigen::Index body) const;
    [[nodiscard]] ConstraintRows constraint_rows(const Kinematics& kin, Eigen::Index constraint) const;
    [[nodiscard]] std::vector<ConstraintRows> all_rows(const Kinematics& kin) const;
    void record_outputs(const State& before, const State& after, double h);
    [[nodiscard]] TransmissionCoupling coupling(Eigen::Index transmission, const Eigen::VectorXd& q,
                                                const Eigen::VectorXd& v) const;
    void add_transmission_terms(const Eigen::VectorXd& q, const Eigen::VectorXd& v, DynamicsTerms& terms) const;
    [[nodiscard]] Eigen::MatrixXd point_jacobian(const Kinematics& kin, Eigen::Index body,
                                                 const Eigen::Vector3d& point) const;
    std::vector<ContactOutput> add_contact_terms(const Kinematics& kin, const Eigen::VectorXd& v,
                                                 DynamicsTerms& terms) const;
    void add_attachment_terms(const Kinematics& kin, const Eigen::VectorXd& v, DynamicsTerms& terms) const;

    MultibodyModel model_;
    MultibodyConfig config_;
    std::vector<Eigen::Index> q_offset_;
    std::vector<Eigen::Index> v_offset_;
    std::vector<spatial::Transform> placement_;
    std::vector<spatial::Matrix6d> inertia_;
    Eigen::VectorXd stiffness_;  // joint springs per velocity DOF (ENG-051)
    Eigen::VectorXd damping_;
    Eigen::VectorXd rest_;
    Eigen::Index nq_ = 0;
    Eigen::Index nv_ = 0;
    Eigen::VectorXd tau_;
    std::vector<spatial::Vector6d> wrench_;  // world-aligned [torque; force] at the center of mass
    std::vector<double> shaft_torque_;       // per transmission, cleared each step
    std::vector<double> slip_;               // per transmission, persistent
    std::vector<double> slip_offset_;        // s per transmission, advanced by step()
    MultibodyOutputs outputs_;
    // Written by the const step hooks for step(); single-threaded by ENG-023.
    mutable std::vector<Eigen::VectorXd> impulse_;
    mutable std::vector<ConstraintError> error_;
    ContactModel contact_model_;
    std::vector<Shape> contact_shapes_;
    std::vector<WeldAttachment> attachments_;
    std::vector<ModelChangeEvent> pending_events_;  // listed in the next step's outputs
    mutable std::vector<ContactOutput> contact_cache_;   // from the last evaluate
    mutable std::vector<PayloadOutput> payload_cache_;   // from the last evaluate
    mutable std::vector<spatial::Vector6d> attachment_wrench_;  // on the payload, world-aligned [torque; force]
    mutable std::vector<Eigen::Vector3d> attachment_point_;     // payload com, world
};

}  // namespace mechsim::engine
