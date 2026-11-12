#include "mechsim/physics/native/native_adapter.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "json.hpp"
#include "mechsim/engine/contact.hpp"
#include "mechsim/engine/dynamics_provider.hpp"
#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/member_loads.hpp"
#include "mechsim/engine/multibody_model.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/rope.hpp"
#include "mechsim/engine/state.hpp"
#include "mechsim/engine/step_config.hpp"
#include "mechsim/engine/transmission.hpp"

namespace mechsim::physics::native {

namespace eng = mechsim::engine;
using kernel::AdapterError;

// ===== Supported elements ===============================================
// Mirrored by mechsim/physics/native/translate.py SUPPORTED_ELEMENTS (tests/unit/test_native_translation.py checks).
namespace {

constexpr std::array<std::string_view, 21> supported_kinds{
    "body",
    "joint.revolute",
    "joint.prismatic",
    "joint.fixed",
    "joint.planar",
    "joint.spherical",
    "joint.free",
    "shape.sphere",
    "shape.capsule",
    "shape.box",
    "shape.cylinder",
    "shape.convex_hull",
    "shape.plane",
    "constraint.point",
    "constraint.hinge",
    "contacts",
    "lumped_member",
    "transmission.belt_pulley",
    "transmission.lead_screw",
    "transmission.gear_pair",
    "rope",
};
// ===== End supported elements ===========================================

constexpr std::string_view model_format = "mechsim-native-1";

// ===== Helpers ===========================================================

template <typename T>
const T& item(const std::vector<T>& items, Eigen::Index i) {
    return items[static_cast<std::size_t>(i)];
}

template <typename T>
T& item(std::vector<T>& items, Eigen::Index i) {
    return items[static_cast<std::size_t>(i)];
}

Eigen::Index index_of(std::size_t n) { return static_cast<Eigen::Index>(n); }

Eigen::Vector3d vec3(const json::Value& value, std::string_view what) {
    const std::vector<json::Value>& a = value.array(what);
    if (a.size() != 3) {
        throw json::ParseError(std::string(what) + ": expected 3 numbers");
    }
    return {a[0].number(what), a[1].number(what), a[2].number(what)};
}

Eigen::Quaterniond quat(const json::Value& value, std::string_view what) {
    const std::vector<json::Value>& a = value.array(what);
    if (a.size() != 4) {
        throw json::ParseError(std::string(what) + ": expected 4 numbers, w first");
    }
    return {a[0].number(what), a[1].number(what), a[2].number(what), a[3].number(what)};
}

Eigen::VectorXd vecx(const json::Value& value, std::string_view what) {
    const std::vector<json::Value>& a = value.array(what);
    Eigen::VectorXd out(index_of(a.size()));
    for (std::size_t i = 0; i < a.size(); ++i) {
        out[index_of(i)] = a[i].number(what);
    }
    return out;
}

eng::JointType joint_type(const std::string& name) {
    if (name == "revolute") {
        return eng::JointType::Revolute;
    }
    if (name == "prismatic") {
        return eng::JointType::Prismatic;
    }
    if (name == "fixed") {
        return eng::JointType::Fixed;
    }
    if (name == "planar") {
        return eng::JointType::Planar;
    }
    if (name == "spherical") {
        return eng::JointType::Spherical;
    }
    if (name == "free") {
        return eng::JointType::Free;
    }
    throw json::ParseError("joint type '" + name + "': not a neutral joint type");
}

eng::TransmissionType transmission_type(const std::string& name) {
    if (name == "belt_pulley") {
        return eng::TransmissionType::BeltPulley;
    }
    if (name == "lead_screw") {
        return eng::TransmissionType::LeadScrew;
    }
    if (name == "gear_pair") {
        return eng::TransmissionType::GearPair;
    }
    throw json::ParseError("transmission type '" + name + "': unknown");
}

eng::BeamEnds beam_ends(const std::string& name) {
    if (name == "cantilever") {
        return eng::BeamEnds::Cantilever;
    }
    if (name == "simply_supported") {
        return eng::BeamEnds::SimplySupported;
    }
    if (name == "clamped") {
        return eng::BeamEnds::Clamped;
    }
    throw json::ParseError("member ends '" + name + "': unknown");
}

eng::ContactParameters contact_parameters(const json::Value& value) {
    eng::ContactParameters p;
    p.stiffness = value.at("stiffness").number("contact stiffness");
    p.damping = value.at("damping").number("contact damping");
    p.friction = value.at("friction").number("contact friction");
    return p;
}

kernel::PowerFlow power_flow(eng::PowerFlow flow) {
    switch (flow) {
        case eng::PowerFlow::Static:
            return kernel::PowerFlow::Static;
        case eng::PowerFlow::Forward:
            return kernel::PowerFlow::Forward;
        case eng::PowerFlow::Backward:
            return kernel::PowerFlow::Backward;
        case eng::PowerFlow::Locked:
            return kernel::PowerFlow::Locked;
    }
    return kernel::PowerFlow::Static;
}

std::string change_kind(eng::ModelChange kind) {
    switch (kind) {
        case eng::ModelChange::SpringOffset:
            return "spring_offset";
        case eng::ModelChange::SpringStiffness:
            return "spring_stiffness";
        case eng::ModelChange::ConstraintDisabled:
            return "constraint_disabled";
        case eng::ModelChange::LoadReleased:
            return "load_released";
        case eng::ModelChange::RopeSnapped:
            return "rope_snapped";
        case eng::ModelChange::PayloadAttached:
            return "payload_attached";
        case eng::ModelChange::PayloadMass:
            return "payload_mass";
    }
    return "unknown";
}

std::string load_source(eng::LoadSource source) {
    switch (source) {
        case eng::LoadSource::Constraint:
            return "constraint";
        case eng::LoadSource::Rope:
            return "rope";
        case eng::LoadSource::Payload:
            return "payload";
    }
    return "unknown";
}

kernel::Vec3 to_vec3(const Eigen::Vector3d& v) { return {v.x(), v.y(), v.z()}; }

// A body index read from the model document (invariant 7): in [0, count), or -1 (the world) where allowed; anything
// else is a ParseError naming the field.
Eigen::Index body_index(const json::Value& value, const std::string& what, std::size_t count, bool world) {
    const int index = value.integer(what);
    if (index == -1 && world) {
        return -1;
    }
    if (index < 0 || static_cast<std::size_t>(index) >= count) {
        std::string range = "[0, " + std::to_string(count) + ")";
        if (world) {
            range = "-1 (world) or " + range;
        }
        throw json::ParseError(what + ": index " + std::to_string(index) + " outside " + range);
    }
    return index;
}

kernel::BodyState body_state(const eng::BodyOutput& pose) {
    kernel::BodyState s;
    s.position = to_vec3(pose.position);
    s.orientation = {pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z()};
    s.angular_velocity = to_vec3(pose.spatial_velocity.head<3>());
    s.linear_velocity = to_vec3(pose.origin_velocity);
    return s;
}

// ===== Joint extras ======================================================
// Joint spring changes (ENG-031, ENG-032 on joint springs), implicit through K so the integrator solves them at the
// step's end state (backward Euler, ADR 0013 A): the engine applies -K0 (q - rest0); the change to K1 and
// rest0 + offset adds -(K1 - K0)(q - rest0) + K1 offset with stiffness K1 - K0 (the total stays K1 >= 0). Joint
// limits, joint friction and joint drives are the engine's (ENG-013, ENG-014, ENG-051; ruling 37).
struct DofExtras {
    Eigen::Index q_index = -1;
    double base_stiffness = 0.0;
    double base_rest = 0.0;
    double stiffness = 0.0;
    double offset = 0.0;
};

struct Extras {
    std::vector<DofExtras> dofs;  // per engine velocity DOF
};

class ExtendedProvider final : public eng::DynamicsProvider {
public:
    ExtendedProvider(const eng::DynamicsProvider& inner, const Extras& extras) : inner_(inner), extras_(extras) {}

    [[nodiscard]] Eigen::Index position_size() const override { return inner_.position_size(); }
    [[nodiscard]] Eigen::Index velocity_size() const override { return inner_.velocity_size(); }

    void evaluate(double t, const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                  eng::DynamicsTerms& terms) const override {
        inner_.evaluate(t, q, v, terms);
        for (std::size_t i = 0; i < extras_.dofs.size(); ++i) {
            const DofExtras& d = extras_.dofs[i];
            const auto k = index_of(i);
            if (d.q_index < 0) {
                continue;
            }
            if (d.stiffness != d.base_stiffness || d.offset != 0.0) {
                terms.force[k] += -(d.stiffness - d.base_stiffness) * (q[d.q_index] - d.base_rest) +
                                  d.stiffness * d.offset;
                terms.stiffness(k, k) += d.stiffness - d.base_stiffness;
            }
        }
    }

    [[nodiscard]] Eigen::VectorXd integrate_positions(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                      double h) const override {
        return inner_.integrate_positions(q, v, h);
    }

    int solve_constraints(double t, double h, const Eigen::VectorXd& q, const Eigen::MatrixXd& system,
                          Eigen::VectorXd& v) const override {
        return inner_.solve_constraints(t, h, q, system, v);
    }

    double project(Eigen::VectorXd& q, Eigen::VectorXd& v) const override { return inner_.project(q, v); }

    double project_positions(double t, Eigen::VectorXd& q, Eigen::VectorXd& v) const override {
        return inner_.project_positions(t, q, v);
    }

private:
    const eng::DynamicsProvider& inner_;
    const Extras& extras_;
};

// MultibodyProvider::step hands itself to the integrator; this integrator wraps it with the adapter's extras and
// steps with the engine's SemiImplicitEuler, so the engine's step order and outputs are untouched.
class ExtendedIntegrator final : public eng::Integrator {
public:
    ExtendedIntegrator(eng::StepConfig config, const Extras& extras) : inner_(config), extras_(extras) {}

    [[nodiscard]] const eng::StepConfig& config() const noexcept override { return inner_.config(); }

    eng::StepHealth step(const eng::DynamicsProvider& provider, eng::State& state) override {
        const ExtendedProvider extended(provider, extras_);
        return inner_.step(extended, state);
    }

private:
    eng::SemiImplicitEuler inner_;
    const Extras& extras_;
};

struct Placement {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

}  // namespace

// ===== Engine ============================================================

struct NativeAdapter::Engine {
    eng::MultibodyModel model;
    std::unique_ptr<eng::MultibodyProvider> provider;
    Extras extras;
    std::unique_ptr<ExtendedIntegrator> integrator;
    eng::State state;
    kernel::Rate rate{1};
    double h = 1e-3;
    std::int64_t steps = 0;
    eng::StepHealth last_health;

    std::size_t neutral_bodies = 0;
    std::size_t neutral_constraints = 0;
    std::vector<eng::JointType> joint_types;     // per neutral body
    std::vector<Eigen::Index> shaft_of;          // per neutral body: its transmission when it is a shaft, else -1
    std::vector<std::pair<Eigen::Index, Eigen::Index>> springs;  // (v DOF, -1) or (member, node), spring order
    std::vector<Eigen::Index> payload_bodies;
    std::vector<Eigen::Index> neutral_dof_to_engine;  // neutral velocity DOF -> engine velocity DOF
    std::vector<kernel::JointDrive> drives;           // this step's drives, input order
    std::vector<Eigen::Index> driven;                 // joints carrying an engine drive (set_joint_drive)
    kernel::PhysicsOutputs outputs;
    std::vector<ModelChangeRecord> events;
    int substeps = 1;                                        // engine steps per adapter step
    kernel::PhysicsInputs inputs;                            // re-applied before each engine step of a tick
    std::vector<kernel::ModelChangeEvent> pending_changes;  // mid-run spring changes, listed in the next tick
    std::vector<Eigen::Index> node_bodies;  // member chain bodies, published after the neutral bodies, in order
};

namespace {

void parse_bodies(const json::Value& doc, eng::MultibodyModel& model) {
    for (const json::Value& b : doc.at("bodies").array("bodies")) {
        eng::Body body;
        body.name = b.at("name").string("body name");
        body.parent = body_index(b.at("parent"), "body parent", model.bodies.size(), true);
        body.mass = b.at("mass").number("body mass");
        body.com = vec3(b.at("com"), "body com");
        const Eigen::VectorXd inertia = vecx(b.at("inertia"), "body inertia");
        if (inertia.size() != 9) {
            throw json::ParseError("body '" + body.name + "': inertia takes 9 numbers, row major");
        }
        for (Eigen::Index r = 0; r < 3; ++r) {
            for (Eigen::Index c = 0; c < 3; ++c) {
                body.inertia(r, c) = inertia[(r * 3) + c];
            }
        }
        const json::Value& j = b.at("joint");
        body.joint.type = joint_type(j.at("type").string("joint type"));
        body.joint.position = vec3(j.at("position"), "joint position");
        body.joint.orientation = quat(j.at("orientation"), "joint orientation");
        body.joint.axis = vec3(j.at("axis"), "joint axis");
        body.joint.stiffness = vecx(j.at("stiffness"), "joint stiffness");
        body.joint.damping = vecx(j.at("damping"), "joint damping");
        body.joint.rest = vecx(j.at("rest"), "joint rest");
        const std::vector<json::Value>& friction = j.at("friction").array("joint friction");
        const std::vector<json::Value>& lower = j.at("lower").array("joint lower");
        const std::vector<json::Value>& upper = j.at("upper").array("joint upper");
        if (lower.size() != friction.size() || upper.size() != friction.size()) {
            throw json::ParseError("body '" + body.name + "': friction, lower and upper differ in length");
        }
        // Engine form (ENG-013, ENG-014): friction per DOF, empty when all zero so a frictionless run is bitwise the
        // engine's; a bound per side, empty when open (the document's null); the engine refuses what it cannot carry.
        Eigen::VectorXd rubbing(index_of(friction.size()));
        bool rubs = false;
        for (std::size_t k = 0; k < friction.size(); ++k) {
            rubbing[index_of(k)] = friction[k].number("joint friction");
            rubs = rubs || rubbing[index_of(k)] != 0.0;
        }
        if (rubs) {
            body.joint.friction = rubbing;
        }
        for (std::size_t k = 0; k < friction.size(); ++k) {
            if ((!lower[k].is_null() || !upper[k].is_null()) && friction.size() != 1) {
                throw json::ParseError("body '" + body.name + "': limits act on a single-DOF joint only (ENG-013)");
            }
            if (!lower[k].is_null()) {
                body.joint.lower = Eigen::VectorXd::Constant(1, lower[k].number("joint lower"));
            }
            if (!upper[k].is_null()) {
                body.joint.upper = Eigen::VectorXd::Constant(1, upper[k].number("joint upper"));
            }
        }
        model.bodies.push_back(std::move(body));
    }
}

}  // namespace

// ===== Adapter ===========================================================

NativeAdapter::NativeAdapter() {
    for (const std::string_view kind : supported_kinds) {
        supported_.emplace_back(kind);
    }
}

NativeAdapter::~NativeAdapter() = default;

std::span<const std::string> NativeAdapter::supported_elements() const { return supported_; }

NativeAdapter::Engine& NativeAdapter::engine(const char* operation) const {
    if (!engine_) {
        throw AdapterError(0, std::string(operation) + " before load");
    }
    return *engine_;
}

std::string NativeAdapter::integrator() const {
    static_cast<void>(engine("integrator"));
    return "backward_euler";
}

double NativeAdapter::timestep() const { return engine("timestep").h; }

const std::vector<ModelChangeRecord>& NativeAdapter::events() const { return engine("events").events; }

void NativeAdapter::load(const kernel::PreparedModel& prepared, const kernel::Rate& physics_rate) {
    if (engine_) {
        throw AdapterError(0, "load: already loaded; reset() first");
    }
    if (!prepared.engine_artifact.empty()) {
        throw AdapterError(0, "load: the native adapter takes no engine artifact (the engine model is the JSON)");
    }
    auto e = std::make_unique<Engine>();
    e->rate = physics_rate;
    try {
        const json::Value doc = json::Value::parse(prepared.neutral_model_json);
        if (doc.at("format").string("format") != model_format) {
            throw json::ParseError("format: expected '" + std::string(model_format) + "'");
        }
        if (doc.has("substeps")) {
            e->substeps = doc.at("substeps").integer("substeps");
            if (e->substeps < 1) {
                throw json::ParseError("substeps: at least 1 engine step per adapter step");
            }
        }
        e->h = static_cast<double>(physics_rate.denominator()) /
               (static_cast<double>(physics_rate.numerator()) * static_cast<double>(e->substeps));
        for (const json::Value& kind : doc.at("elements").array("elements")) {
            const std::string& name = kind.string("element kind");
            if (std::ranges::find(supported_kinds, std::string_view(name)) == supported_kinds.end()) {
                throw AdapterError(0, "load: element kind '" + name + "' is not mapped by the native adapter (PHY-019)");
            }
        }
        const json::Value& run = doc.at("config");
        eng::MultibodyConfig config;
        config.iteration_cap = run.at("iteration_cap").integer("iteration_cap");
        config.velocity_tolerance = run.at("velocity_tolerance").number("velocity_tolerance");
        config.projection_iterations = run.at("projection_iterations").integer("projection_iterations");
        config.drift_threshold_m = run.at("drift_threshold_m").number("drift_threshold_m");
        config.drift_threshold_rad = run.at("drift_threshold_rad").number("drift_threshold_rad");
        config.unit_tolerance = run.at("unit_tolerance").number("unit_tolerance");
        eng::StepConfig step_config;
        step_config.h = e->h;
        step_config.symmetry_tolerance = run.at("symmetry_tolerance").number("symmetry_tolerance");

        eng::MultibodyModel& model = e->model;
        model.gravity = vec3(doc.at("gravity"), "gravity");
        parse_bodies(doc, model);
        const std::size_t doc_bodies = model.bodies.size();
        const int neutral = doc.at("neutral_bodies").integer("neutral_bodies");
        if (neutral < 0 || static_cast<std::size_t>(neutral) > model.bodies.size()) {
            throw json::ParseError("neutral_bodies: " + std::to_string(neutral) + " outside [0, " +
                                   std::to_string(model.bodies.size()) + "]");
        }
        e->neutral_bodies = static_cast<std::size_t>(neutral);
        for (std::size_t i = 0; i < e->neutral_bodies; ++i) {
            e->joint_types.push_back(model.bodies[i].joint.type);
        }
        e->shaft_of.assign(e->neutral_bodies, -1);
        for (const json::Value& c : doc.at("constraints").array("constraints")) {
            eng::LoopConstraint lc;
            const std::string& type = c.at("type").string("constraint type");
            if (type == "point") {
                lc.type = eng::ConstraintType::Point;
            } else if (type == "hinge") {
                lc.type = eng::ConstraintType::Hinge;
                lc.axis_a = vec3(c.at("axis_a"), "constraint axis_a");
                lc.axis_b = vec3(c.at("axis_b"), "constraint axis_b");
            } else {
                throw json::ParseError("constraint type '" + type + "': unknown");
            }
            lc.body_a = body_index(c.at("body_a"), "constraint body_a", doc_bodies, true);
            lc.body_b = body_index(c.at("body_b"), "constraint body_b", doc_bodies, true);
            lc.point_a = vec3(c.at("point_a"), "constraint point_a");
            lc.point_b = vec3(c.at("point_b"), "constraint point_b");
            model.constraints.push_back(lc);
        }
        e->neutral_constraints = model.constraints.size();
        std::vector<std::pair<Eigen::Index, Eigen::Index>> member_springs;
        Eigen::Index member_index = 0;
        for (const json::Value& m : doc.at("members").array("members")) {
            eng::BeamSpec spec;
            spec.youngs_modulus = m.at("youngs_modulus").number("member youngs_modulus");
            spec.area_moment = m.at("area_moment").number("member area_moment");
            spec.density = m.at("density").number("member density");
            spec.area = m.at("area").number("member area");
            spec.length = m.at("length").number("member length");
            spec.segments = m.at("segments").integer("member segments");
            spec.node_damping = m.at("node_damping").number("member node_damping");
            spec.ends = beam_ends(m.at("ends").string("member ends"));
            spec.roller_mass_fraction = m.at("roller_mass_fraction").number("member roller_mass_fraction");
            const eng::LumpedBeam beam =
                eng::add_lumped_beam(model, spec, body_index(m.at("parent"), "member parent", doc_bodies, true),
                                     vec3(m.at("position"), "member position"),
                                     quat(m.at("orientation"), "member orientation"));
            const std::vector<Eigen::Index> nodes = eng::member_nodes(beam);
            for (std::size_t node = 0; node < nodes.size(); ++node) {
                member_springs.emplace_back(member_index, index_of(node));
            }
            ++member_index;
        }
        for (const json::Value& t : doc.at("transmissions").array("transmissions")) {
            eng::Transmission tr;
            tr.type = transmission_type(t.at("type").string("transmission type"));
            tr.input_body = body_index(t.at("input_body"), "transmission input_body", e->neutral_bodies, false);
            tr.output_body = body_index(t.at("output_body"), "transmission output_body", e->neutral_bodies, false);
            tr.ratio = t.at("ratio").number("transmission ratio");
            tr.reverses = t.at("reverses").boolean("transmission reverses");
            tr.efficiency = t.at("efficiency").number("transmission efficiency");
            tr.back_efficiency = t.at("back_efficiency").number("transmission back_efficiency");
            tr.self_locking = t.at("self_locking").boolean("transmission self_locking");
            tr.stiffness = t.at("stiffness").number("transmission stiffness");
            tr.damping = t.at("damping").number("transmission damping");
            tr.backlash = t.at("backlash").number("transmission backlash");
            tr.slip = t.at("slip").number("transmission slip");
            if (tr.input_body >= 0 && static_cast<std::size_t>(tr.input_body) < e->neutral_bodies) {
                if (item(e->shaft_of, tr.input_body) >= 0) {
                    throw AdapterError(0, "load: body " + std::to_string(tr.input_body) +
                                              " is the shaft of two transmissions (two actuators on one shaft)");
                }
                item(e->shaft_of, tr.input_body) = index_of(model.transmissions.size());
            }
            model.transmissions.push_back(tr);
        }

        std::vector<std::pair<Eigen::Index, double>> starts;
        for (const json::Value& s : doc.at("starts").array("starts")) {
            starts.emplace_back(body_index(s.at("body"), "start body", e->neutral_bodies, false), s.at("value").number("start value"));
        }
        const auto start_q = [&starts](const eng::MultibodyProvider& p) {
            Eigen::VectorXd q = p.neutral_positions();
            for (const auto& [b, value] : starts) {
                const eng::JointType type = item(p.model().bodies, b).joint.type;
                if (type != eng::JointType::Revolute && type != eng::JointType::Prismatic) {
                    throw AdapterError(0, "load: body " + std::to_string(b) + ": only a revolute or prismatic joint "
                                                                             "takes a start coordinate");
                }
                q[p.position_offset(b)] = value;
            }
            return q;
        };

        const std::vector<json::Value>& ropes = doc.at("ropes").array("ropes");
        if (!ropes.empty()) {
            const eng::MultibodyProvider before(model, config);
            const Eigen::VectorXd q = start_q(before);
            const std::vector<eng::BodyOutput> poses =
                before.body_states(q, Eigen::VectorXd::Zero(before.velocity_size()));
            const auto world = [&poses](Eigen::Index b, const Eigen::Vector3d& local) -> Eigen::Vector3d {
                if (b < 0) {
                    return local;
                }
                const eng::BodyOutput& pose = item(poses, b);
                return pose.position + (pose.orientation * local);
            };
            for (const json::Value& r : ropes) {
                eng::Rope rope;
                rope.from_body = body_index(r.at("from_body"), "rope from_body", doc_bodies, true);
                rope.from_point = vec3(r.at("from_point"), "rope from_point");
                rope.to_body = body_index(r.at("to_body"), "rope to_body", doc_bodies, true);
                rope.to_point = vec3(r.at("to_point"), "rope to_point");
                rope.segments = r.at("segments").integer("rope segments");
                rope.axial_rigidity = r.at("axial_rigidity").number("rope axial_rigidity");
                rope.damping = r.at("damping").number("rope damping");
                rope.mass_per_length = r.at("mass_per_length").number("rope mass_per_length");
                rope.rest_length = r.at("rest_length").number("rope rest_length");
                if (!r.at("breaking_tension").is_null()) {
                    rope.breaking_tension = r.at("breaking_tension").number("rope breaking_tension");
                }
                rope.attachment_damping = r.at("attachment_damping").number("rope attachment_damping");
                rope.attachment_friction = r.at("attachment_friction").number("rope attachment_friction");
                const Eigen::Vector3d from_world = world(rope.from_body, rope.from_point);
                const Eigen::Vector3d to_world = world(rope.to_body, rope.to_point);
                eng::add_rope(model, rope, from_world, to_world);
            }
        }

        e->provider = std::make_unique<eng::MultibodyProvider>(model, config);
        eng::MultibodyProvider& provider = *e->provider;

        if (!doc.at("contact").is_null()) {
            const json::Value& c = doc.at("contact");
            eng::ContactModel contacts;
            contacts.default_parameters = contact_parameters(c.at("default"));
            contacts.slip_velocity = c.at("slip_velocity").number("contact slip_velocity");
            for (const json::Value& p : c.at("pairs").array("contact pairs")) {
                eng::MaterialPair pair;
                pair.material_a = p.at("material_a").string("pair material_a");
                pair.material_b = p.at("material_b").string("pair material_b");
                pair.parameters = contact_parameters(p);
                contacts.pairs.push_back(pair);
            }
            for (const json::Value& x : c.at("exclusions").array("contact exclusions")) {
                const std::vector<json::Value>& two = x.array("exclusion");
                if (two.size() != 2) {
                    throw json::ParseError("exclusion: expected two body indices");
                }
                contacts.exclusions.emplace_back(body_index(two[0], "contact exclusion", doc_bodies, true),
                                                 body_index(two[1], "contact exclusion", doc_bodies, true));
            }
            for (const json::Value& g : c.at("geometries").array("geometries")) {
                eng::Geometry geometry;
                geometry.kind = g.at("kind").string("geometry kind");
                geometry.body = body_index(g.at("body"), "geometry body", doc_bodies, true);
                geometry.position = vec3(g.at("position"), "geometry position");
                geometry.orientation = quat(g.at("orientation"), "geometry orientation");
                geometry.radius = g.at("radius").number("geometry radius");
                geometry.length = g.at("length").number("geometry length");
                geometry.size = vec3(g.at("size"), "geometry size");
                for (const json::Value& v : g.at("vertices").array("geometry vertices")) {
                    geometry.vertices.push_back(vec3(v, "geometry vertex"));
                }
                geometry.material = g.at("material").string("geometry material");
                contacts.geometries.push_back(std::move(geometry));
            }
            provider.set_contact_model(std::move(contacts));
        }

        Eigen::VectorXd q = start_q(provider);
        for (std::size_t b = 0; b < doc_bodies; ++b) {
            const eng::Joint& joint = model.bodies[b].joint;
            double tolerance = config.drift_threshold_m;
            if (joint.type == eng::JointType::Revolute) {
                tolerance = config.drift_threshold_rad;
            }
            const double coordinate = q[provider.position_offset(index_of(b))];
            const bool below = joint.lower.size() == 1 && coordinate < joint.lower[0] - tolerance;
            const bool above = joint.upper.size() == 1 && coordinate > joint.upper[0] + tolerance;
            if (below || above) {
                throw AdapterError(0, "load: body '" + model.bodies[b].name + "': start coordinate " +
                                          std::to_string(coordinate) + " outside its joint limits (ruling 43)");
            }
        }
        const std::vector<json::Value>& payloads = doc.at("payloads").array("payloads");
        if (!starts.empty() && !payloads.empty()) {
            const Eigen::VectorXd zero = Eigen::VectorXd::Zero(provider.velocity_size());
            const std::vector<eng::BodyOutput> old_poses = provider.body_states(provider.neutral_positions(), zero);
            const std::vector<eng::BodyOutput> new_poses = provider.body_states(q, zero);
            for (const json::Value& p : payloads) {
                const Eigen::Index body = body_index(p.at("body"), "payload body", e->neutral_bodies, false);
                const Eigen::Index carrier = body_index(p.at("carrier"), "payload carrier", e->neutral_bodies, false);
                const eng::Body& payload_body = item(provider.model().bodies, body);
                if (payload_body.joint.type != eng::JointType::Free || payload_body.parent != -1) {
                    throw AdapterError(0, "load: payload '" + payload_body.name + "' is not a free body under world");
                }
                const Eigen::Quaterniond turn =
                    item(new_poses, carrier).orientation * item(old_poses, carrier).orientation.conjugate();
                const Eigen::Vector3d position =
                    item(new_poses, carrier).position +
                    (turn * (item(old_poses, body).position - item(old_poses, carrier).position));
                const Eigen::Quaterniond orientation = turn * item(old_poses, body).orientation;
                const Eigen::Quaterniond placement = payload_body.joint.orientation;
                const Eigen::Vector3d local = placement.conjugate() * (position - payload_body.joint.position);
                const Eigen::Quaterniond turned = (placement.conjugate() * orientation).normalized();
                const Eigen::Index offset = provider.position_offset(body);
                q.segment<3>(offset) = local;
                q[offset + 3] = turned.w();
                q[offset + 4] = turned.x();
                q[offset + 5] = turned.y();
                q[offset + 6] = turned.z();
            }
        }
        for (const json::Value& p : payloads) {
            const Eigen::Index body = body_index(p.at("body"), "payload body", e->neutral_bodies, false);
            const Eigen::Index carrier = body_index(p.at("carrier"), "payload carrier", e->neutral_bodies, false);
            provider.set_attachment(body, carrier, true, q);
            e->payload_bodies.push_back(body);
        }

        for (const eng::LumpedBeam& beam : model.members) {
            e->node_bodies.insert(e->node_bodies.end(), beam.bodies.begin(), beam.bodies.end());
        }
        e->extras.dofs.assign(static_cast<std::size_t>(provider.velocity_size()), DofExtras{});
        for (std::size_t b = 0; b < doc_bodies; ++b) {
            const eng::Body& body = model.bodies[b];
            const Eigen::Index v0 = provider.velocity_offset(index_of(b));
            const Eigen::Index q0 = provider.position_offset(index_of(b));
            const bool coordinate = body.joint.type == eng::JointType::Revolute ||
                                    body.joint.type == eng::JointType::Prismatic ||
                                    body.joint.type == eng::JointType::Planar;
            if (!coordinate) {
                continue;
            }
            const auto count = static_cast<std::size_t>(body.joint.type == eng::JointType::Planar) * 2 + 1;
            for (std::size_t k = 0; k < count; ++k) {
                DofExtras d;
                d.q_index = q0 + index_of(k);
                if (k < static_cast<std::size_t>(body.joint.stiffness.size())) {
                    d.base_stiffness = body.joint.stiffness[index_of(k)];
                }
                if (k < static_cast<std::size_t>(body.joint.rest.size())) {
                    d.base_rest = body.joint.rest[index_of(k)];
                }
                d.stiffness = d.base_stiffness;
                item(e->extras.dofs, v0 + index_of(k)) = d;
                if (b < e->neutral_bodies) {
                    e->springs.emplace_back(v0 + index_of(k), -1);
                }
            }
        }
        for (const auto& spring : member_springs) {
            e->springs.push_back(spring);
        }
        for (std::size_t b = 0; b < e->neutral_bodies; ++b) {
            const eng::Body& body = model.bodies[b];
            const Eigen::Index v0 = provider.velocity_offset(index_of(b));
            Eigen::Index next = provider.velocity_size();
            if (b + 1 < model.bodies.size()) {
                next = provider.velocity_offset(index_of(b) + 1);
            }
            const Eigen::Index dofs = next - v0;
            if (body.joint.type == eng::JointType::Free) {
                for (Eigen::Index k = 0; k < 3; ++k) {
                    e->neutral_dof_to_engine.push_back(v0 + 3 + k);
                }
                for (Eigen::Index k = 0; k < 3; ++k) {
                    e->neutral_dof_to_engine.push_back(v0 + k);
                }
                continue;
            }
            for (Eigen::Index k = 0; k < dofs; ++k) {
                e->neutral_dof_to_engine.push_back(v0 + k);
            }
        }
        e->state.q = q;
        e->state.v = Eigen::VectorXd::Zero(provider.velocity_size());
        e->state.t = 0.0;
        e->integrator = std::make_unique<ExtendedIntegrator>(step_config, e->extras);
    } catch (const AdapterError&) {
        throw;
    } catch (const eng::EngineError& error) {
        throw AdapterError(0, "load: engine: " + error.cause());
    } catch (const json::ParseError& error) {
        throw AdapterError(0, std::string("load: ") + error.what());
    } catch (const std::exception& error) {
        throw AdapterError(0, std::string("load: ") + error.what());
    }
    engine_ = std::move(e);
    Engine& en = *engine_;
    const std::vector<eng::BodyOutput> poses = en.provider->body_states(en.state.q, en.state.v);
    for (std::size_t b = 0; b < en.neutral_bodies; ++b) {
        const eng::BodyOutput& pose = poses[b];
        en.outputs.bodies.push_back(body_state(pose));
        double coordinate = 0.0;
        const eng::JointType type = item(en.joint_types, index_of(b));
        if (type == eng::JointType::Revolute || type == eng::JointType::Prismatic) {
            coordinate = en.state.q[en.provider->position_offset(index_of(b))];
        }
        en.outputs.joint_positions.push_back(coordinate);
    }
    for (const Eigen::Index b : en.node_bodies) {
        en.outputs.bodies.push_back(body_state(poses[static_cast<std::size_t>(b)]));
    }
    for (const Eigen::Index body : en.payload_bodies) {
        kernel::PayloadState p;
        p.body = static_cast<std::size_t>(body);
        for (const eng::WeldAttachment& a : en.provider->attachments()) {
            if (a.body == body) {
                p.attached = a.attached;
                p.carrier = static_cast<std::size_t>(a.carrier);
            }
        }
        en.outputs.payloads.push_back(p);
    }
}

void NativeAdapter::reset() { engine_.reset(); }

const kernel::PhysicsOutputs& NativeAdapter::outputs() const {
    if (!engine_) {
        return empty_;
    }
    return engine_->outputs;
}

kernel::PhysicsHealth NativeAdapter::health() const {
    kernel::PhysicsHealth health;
    if (!engine_) {
        return health;
    }
    health.max_constraint_drift = engine_->last_health.max_constraint_drift;
    health.converged = engine_->last_health.converged;
    health.iterations = engine_->last_health.iterations;
    health.steps = engine_->steps;
    health.physics_rate = engine_->rate;
    return health;
}

// ===== Inputs ============================================================

void NativeAdapter::set_inputs(const kernel::PhysicsInputs& inputs) {
    Engine& e = engine("set_inputs");
    try {
        apply_inputs(e, inputs);
    } catch (const eng::EngineError& error) {
        throw AdapterError(e.steps, "set_inputs: engine: " + error.cause());
    }
    e.inputs = inputs;
}

void NativeAdapter::apply_inputs(Engine& e, const kernel::PhysicsInputs& inputs) {
    eng::MultibodyProvider& provider = *e.provider;
    if (!inputs.generalized_forces.empty()) {
        if (inputs.generalized_forces.size() != e.neutral_dof_to_engine.size()) {
            throw AdapterError(e.steps, "set_inputs: " + std::to_string(inputs.generalized_forces.size()) +
                                            " generalized forces for " +
                                            std::to_string(e.neutral_dof_to_engine.size()) + " velocity DOFs");
        }
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(provider.velocity_size());
        for (std::size_t i = 0; i < inputs.generalized_forces.size(); ++i) {
            tau[e.neutral_dof_to_engine[i]] = inputs.generalized_forces[i];
        }
        for (std::size_t b = 0; b < e.neutral_bodies; ++b) {
            const Eigen::Index t = e.shaft_of[b];
            if (t < 0) {
                continue;
            }
            const Eigen::Index v = provider.velocity_offset(index_of(b));
            provider.set_shaft_torque(t, tau[v]);
            tau[v] = 0.0;
        }
        provider.set_generalized_forces(tau);
    }
    for (const kernel::BodyWrench& w : inputs.body_wrenches) {
        const std::size_t published = e.neutral_bodies + e.node_bodies.size();
        if (w.body >= published) {
            throw AdapterError(e.steps, "set_inputs: body wrench on body " + std::to_string(w.body) +
                                            ", outside the model's " + std::to_string(e.neutral_bodies) +
                                            " bodies and " + std::to_string(e.node_bodies.size()) + " member nodes");
        }
        Eigen::Index target = index_of(w.body);
        if (w.body >= e.neutral_bodies) {
            target = e.node_bodies[w.body - e.neutral_bodies];
        }
        provider.apply_body_wrench(target,
                                   Eigen::Vector3d(w.wrench.moment[0], w.wrench.moment[1], w.wrench.moment[2]),
                                   Eigen::Vector3d(w.wrench.force[0], w.wrench.force[1], w.wrench.force[2]));
    }
    std::vector<Eigen::Index> driven;
    for (const kernel::JointDrive& drive : inputs.joint_drives) {
        if (drive.joint >= e.neutral_bodies) {
            throw AdapterError(e.steps, "set_inputs: drive on joint " + std::to_string(drive.joint) +
                                            ", outside the model's " + std::to_string(e.neutral_bodies) + " joints");
        }
        const eng::JointType type = item(e.joint_types, index_of(drive.joint));
        if (type != eng::JointType::Revolute && type != eng::JointType::Prismatic) {
            throw AdapterError(e.steps, "set_inputs: drive on joint " + std::to_string(drive.joint) +
                                            ": only revolute and prismatic joints take a drive (PHY-034)");
        }
        if (!std::isfinite(drive.rest) || !std::isfinite(drive.stiffness) || !std::isfinite(drive.damping) ||
            drive.stiffness < 0.0 || drive.damping < 0.0) {
            throw AdapterError(e.steps, "set_inputs: drive on joint " + std::to_string(drive.joint) +
                                            ": rest finite, stiffness and damping finite and >= 0");
        }
        if (!drive.enabled) {
            continue;
        }
        if (std::ranges::find(driven, index_of(drive.joint)) != driven.end()) {
            throw AdapterError(e.steps, "set_inputs: two drives on joint " + std::to_string(drive.joint));
        }
        driven.push_back(index_of(drive.joint));
    }
    // The engine's drive persists (ENG-051) and adds to the model spring (ruling 42); a kernel drive lasts one tick,
    // so every drive of the previous inputs is cleared and this tick's set. The rest is the end-of-step rest (PHY-034).
    clear_drives(e);
    for (const kernel::JointDrive& drive : inputs.joint_drives) {
        if (drive.enabled) {
            provider.set_joint_drive(index_of(drive.joint), 0, drive.rest, drive.stiffness, drive.damping);
        }
    }
    e.driven = std::move(driven);
    e.drives = inputs.joint_drives;
}

void NativeAdapter::clear_drives(Engine& e) {
    for (const Eigen::Index joint : e.driven) {
        e.provider->clear_joint_drive(joint, 0);
    }
    e.driven.clear();
}

// ===== Step and outputs ==================================================

// The engine overwrites its events every engine step: each step's are appended here, so a tick of several engine
// steps lists every event once (ruling 40). Spring changes are listed by the mid-run entry with their spring index,
// so the engine's per-node spring events are kept on events() only.
namespace {

void collect_events(const std::vector<eng::ModelChangeEvent>& events, std::vector<ModelChangeRecord>& records,
                    std::vector<kernel::ModelChangeEvent>& changes) {
    for (const eng::ModelChangeEvent& event : events) {
        ModelChangeRecord r;
        r.kind = change_kind(event.kind);
        if (event.kind == eng::ModelChange::LoadReleased) {
            r.source = load_source(event.source);
        }
        r.target = event.target;
        r.index = event.index;
        r.value = event.value;
        r.limit = event.limit;
        r.time = event.time;
        records.push_back(r);
        kernel::ModelChangeEvent change;
        change.index = static_cast<std::int64_t>(event.target);
        switch (event.kind) {
            case eng::ModelChange::SpringOffset:
            case eng::ModelChange::SpringStiffness:
                continue;
            case eng::ModelChange::ConstraintDisabled:
                change.kind = kernel::ModelChangeKind::ConstraintDisabled;
                change.source = kernel::ModelChangeSource::Constraint;
                break;
            case eng::ModelChange::LoadReleased:
                change.kind = kernel::ModelChangeKind::LoadReleased;
                change.source = kernel::ModelChangeSource::Constraint;
                if (event.source == eng::LoadSource::Rope) {
                    change.source = kernel::ModelChangeSource::Rope;
                }
                if (event.source == eng::LoadSource::Payload) {
                    change.source = kernel::ModelChangeSource::Payload;
                    change.value = static_cast<double>(event.index);
                }
                break;
            case eng::ModelChange::RopeSnapped:
                change.kind = kernel::ModelChangeKind::RopeSnapped;
                change.source = kernel::ModelChangeSource::Rope;
                change.value = event.value;
                break;
            case eng::ModelChange::PayloadAttached:
                change.kind = kernel::ModelChangeKind::PayloadAttached;
                change.source = kernel::ModelChangeSource::Payload;
                change.value = static_cast<double>(event.index);
                break;
            case eng::ModelChange::PayloadMass:
                change.kind = kernel::ModelChangeKind::PayloadMass;
                change.source = kernel::ModelChangeSource::Payload;
                change.value = event.value;
                break;
        }
        changes.push_back(change);
    }
}

}  // namespace

void NativeAdapter::step() {
    Engine& e = engine("step");
    eng::MultibodyProvider& provider = *e.provider;
    e.events.clear();
    std::vector<kernel::ModelChangeEvent> changes = std::move(e.pending_changes);
    e.pending_changes.clear();
    double drift = 0.0;
    int iterations = 0;
    bool converged = true;
    for (int sub = 0; sub < e.substeps; ++sub) {
        try {
            if (sub > 0) {
                apply_inputs(e, e.inputs);
            }
            e.last_health = provider.step(*e.integrator, e.state);
        } catch (const eng::EngineError& error) {
            throw AdapterError(e.steps, "engine: " + error.cause());
        } catch (const std::exception& error) {
            throw AdapterError(e.steps, std::string("engine: ") + error.what());
        }
        drift = std::max(drift, e.last_health.max_constraint_drift);
        iterations = std::max(iterations, e.last_health.iterations);
        converged = converged && e.last_health.converged;
        collect_events(provider.outputs().events, e.events, changes);
    }
    e.last_health.max_constraint_drift = drift;
    e.last_health.iterations = iterations;
    e.last_health.converged = converged;
    ++e.steps;
    const eng::MultibodyOutputs& out = provider.outputs();
    kernel::PhysicsOutputs& o = e.outputs;
    o.steps = e.steps;
    o.model_changes = std::move(changes);
    o.bodies.clear();
    o.joints.clear();
    o.joint_positions.clear();
    for (std::size_t b = 0; b < e.neutral_bodies; ++b) {
        const eng::BodyOutput& pose = out.bodies[b];
        o.bodies.push_back(body_state(pose));
        const eng::JointOutput& joint = out.joints[b];
        kernel::JointForce f;
        const eng::JointType type = item(e.joint_types, index_of(b));
        if (type == eng::JointType::Free) {
            for (Eigen::Index k = 0; k < 3; ++k) {
                f.generalized_force.push_back(joint.generalized_force[3 + k]);
            }
            for (Eigen::Index k = 0; k < 3; ++k) {
                f.generalized_force.push_back(joint.generalized_force[k]);
            }
        } else {
            for (const double value : joint.generalized_force) {
                f.generalized_force.push_back(value);
            }
        }
        const Eigen::Index shaft = e.shaft_of[b];
        if (shaft >= 0) {
            f.generalized_force.assign(1, item(out.transmissions, shaft).shaft_load_torque);
        }
        f.reaction.moment = to_vec3(joint.reaction_wrench.head<3>());
        f.reaction.force = to_vec3(joint.reaction_wrench.tail<3>());
        o.joints.push_back(std::move(f));
        double coordinate = 0.0;
        if (type == eng::JointType::Revolute || type == eng::JointType::Prismatic) {
            coordinate = e.state.q[provider.position_offset(index_of(b))];
        }
        o.joint_positions.push_back(coordinate);
    }
    // Member node poses after the neutral bodies (BLD-017, X1): each lumped member in neutral order, its chain bodies
    // root to tip; named <member>.node<k> by mechsim.physics.native.member_node_names.
    for (const Eigen::Index b : e.node_bodies) {
        const eng::BodyOutput& pose = out.bodies[static_cast<std::size_t>(b)];
        o.bodies.push_back(body_state(pose));
    }
    o.constraints.clear();
    for (std::size_t c = 0; c < e.neutral_constraints; ++c) {
        const eng::ConstraintOutput& constraint = out.constraints[c];
        kernel::ConstraintForce f;
        for (const double value : constraint.force) {
            f.force.push_back(value);
        }
        f.translational_error = constraint.translational_error;
        f.rotational_error = constraint.rotational_error;
        o.constraints.push_back(std::move(f));
    }
    o.contacts.clear();
    const auto neutral_body = [&e](Eigen::Index b) {
        if (b < 0 || static_cast<std::size_t>(b) >= e.neutral_bodies) {
            return kernel::kWorldBody;
        }
        return static_cast<std::size_t>(b);
    };
    for (const eng::ContactOutput& contact : out.contacts) {
        kernel::ContactForce f;
        f.body_a = neutral_body(contact.body_a);
        f.body_b = neutral_body(contact.body_b);
        f.point = to_vec3(contact.point);
        f.force = to_vec3(contact.force);
        o.contacts.push_back(f);
    }
    o.members.clear();
    for (const eng::MemberOutput& m : out.members) {
        kernel::MemberEndLoads loads;
        loads.start.moment = to_vec3(m.start_moment);
        loads.start.force = to_vec3(m.start_force);
        loads.end.moment = to_vec3(m.end_moment);
        loads.end.force = to_vec3(m.end_force);
        o.members.push_back(loads);
    }
    o.ropes.clear();
    for (const eng::RopeOutput& r : out.ropes) {
        kernel::RopeTension t;
        t.tension = r.load_end_tension;
        t.max_tension = r.max_tension;
        t.slack = r.load_end_tension <= 0.0;
        o.ropes.push_back(t);
    }
    o.transmissions.clear();
    for (std::size_t t = 0; t < out.transmissions.size(); ++t) {
        const eng::TransmissionOutput& tr = out.transmissions[t];
        const double n = eng::output_per_input(provider.model().transmissions[t]);
        kernel::TransmissionState s;
        s.slip = tr.slip_offset / n;
        s.slipping = tr.slip_active;
        s.shaft_load_torque = tr.shaft_load_torque;
        s.force = tr.force;
        s.stretch = tr.stretch;
        s.power_flow = power_flow(tr.power_flow);
        o.transmissions.push_back(s);
    }
    o.joint_drive_torques.clear();
    for (const kernel::JointDrive& drive : e.drives) {
        kernel::JointDriveTorque torque;
        torque.joint = drive.joint;
        if (drive.enabled) {
            const Eigen::Index q = provider.position_offset(index_of(drive.joint));
            const Eigen::Index v = provider.velocity_offset(index_of(drive.joint));
            torque.torque = drive.stiffness * (drive.rest - e.state.q[q]) - drive.damping * e.state.v[v];
        }
        o.joint_drive_torques.push_back(torque);
    }
    e.drives.clear();
    e.inputs = kernel::PhysicsInputs{};
    clear_drives(e);
    o.payloads.clear();
    for (const Eigen::Index body : e.payload_bodies) {
        kernel::PayloadState p;
        p.body = static_cast<std::size_t>(body);
        for (const eng::PayloadOutput& payload : out.payloads) {
            if (payload.body == body) {
                p.attached = payload.attached;
                p.carrier = neutral_body(payload.carrier);
                p.force = to_vec3(payload.force);
            }
        }
        o.payloads.push_back(p);
    }
}

std::vector<std::vector<double>> NativeAdapter::rope_segment_tensions() const {
    const Engine& e = engine("rope_segment_tensions");
    std::vector<std::vector<double>> out;
    for (const eng::RopeOutput& r : e.provider->outputs().ropes) {
        out.push_back(r.tension);
    }
    return out;
}

std::vector<double> NativeAdapter::rope_load_end_tensions() const {
    const Engine& e = engine("rope_load_end_tensions");
    std::vector<double> out;
    for (const eng::RopeOutput& r : e.provider->outputs().ropes) {
        out.push_back(r.load_end_tension);
    }
    return out;
}

// ===== Mid-run changes ===================================================

void NativeAdapter::set_spring_offset(std::size_t spring, double offset) {
    Engine& e = engine("set_spring_offset");
    if (spring >= e.springs.size() || !std::isfinite(offset)) {
        throw AdapterError(e.steps, "set_spring_offset: spring " + std::to_string(spring) + " of " +
                                        std::to_string(e.springs.size()) + ", offset finite");
    }
    const auto [first, node] = e.springs[spring];
    if (node < 0) {
        item(e.extras.dofs, first).offset = offset;
    } else {
        try {
            e.provider->set_spring_offset(first, node, offset);
        } catch (const eng::EngineError& error) {
            throw AdapterError(e.steps, "set_spring_offset: engine: " + error.cause());
        }
    }
    e.pending_changes.push_back(kernel::ModelChangeEvent{kernel::ModelChangeKind::SpringOffset,
                                                         kernel::ModelChangeSource::Spring,
                                                         static_cast<std::int64_t>(spring), offset});
}

void NativeAdapter::set_spring_stiffness(std::size_t spring, double stiffness) {
    Engine& e = engine("set_spring_stiffness");
    if (spring >= e.springs.size() || !std::isfinite(stiffness) || stiffness < 0.0) {
        throw AdapterError(e.steps, "set_spring_stiffness: spring " + std::to_string(spring) + " of " +
                                        std::to_string(e.springs.size()) + ", stiffness finite and >= 0");
    }
    const auto [first, node] = e.springs[spring];
    if (node < 0) {
        item(e.extras.dofs, first).stiffness = stiffness;
    } else {
        try {
            e.provider->set_spring_stiffness(first, node, stiffness);
        } catch (const eng::EngineError& error) {
            throw AdapterError(e.steps, "set_spring_stiffness: engine: " + error.cause());
        }
    }
    e.pending_changes.push_back(kernel::ModelChangeEvent{kernel::ModelChangeKind::SpringStiffness,
                                                         kernel::ModelChangeSource::Spring,
                                                         static_cast<std::int64_t>(spring), stiffness});
}

void NativeAdapter::disable_constraint(std::size_t constraint) {
    Engine& e = engine("disable_constraint");
    if (constraint >= e.neutral_constraints) {
        throw AdapterError(e.steps, "disable_constraint: constraint " + std::to_string(constraint) + " of " +
                                        std::to_string(e.neutral_constraints));
    }
    try {
        e.provider->disable_constraint(index_of(constraint));
    } catch (const eng::EngineError& error) {
        throw AdapterError(e.steps, "disable_constraint: engine: " + error.cause());
    }
}

// Load index space: a payload body's neutral index releases that payload's weld; neutral body count + r releases rope
// r's load end.
void NativeAdapter::release_load(std::size_t load) {
    Engine& e = engine("release_load");
    try {
        if (load >= e.neutral_bodies) {
            const std::size_t rope = load - e.neutral_bodies;
            if (rope >= e.provider->model().ropes.size()) {
                throw AdapterError(e.steps, "release_load: load " + std::to_string(load) +
                                                " is neither a payload body nor a rope");
            }
            e.provider->release_rope(index_of(rope));
            return;
        }
        set_payload_attachment(load, kernel::kWorldBody);
    } catch (const eng::EngineError& error) {
        throw AdapterError(e.steps, "release_load: engine: " + error.cause());
    }
}

void NativeAdapter::set_payload_mass(std::size_t payload, double mass) {
    Engine& e = engine("set_payload_mass");
    if (std::ranges::find(e.payload_bodies, index_of(payload)) == e.payload_bodies.end()) {
        throw AdapterError(e.steps, "set_payload_mass: body " + std::to_string(payload) + " is not a payload");
    }
    try {
        e.provider->set_body_mass(index_of(payload), mass);
    } catch (const eng::EngineError& error) {
        throw AdapterError(e.steps, "set_payload_mass: engine: " + error.cause());
    }
}

void NativeAdapter::set_payload_attachment(std::size_t payload, std::size_t body) {
    Engine& e = engine("set_payload_attachment");
    if (std::ranges::find(e.payload_bodies, index_of(payload)) == e.payload_bodies.end()) {
        throw AdapterError(e.steps, "set_payload_attachment: body " + std::to_string(payload) + " is not a payload");
    }
    Eigen::Index carrier = -1;
    bool attached = false;
    for (const eng::WeldAttachment& a : e.provider->attachments()) {
        if (a.body == index_of(payload)) {
            carrier = a.carrier;
        }
    }
    if (body != kernel::kWorldBody) {
        if (body >= e.neutral_bodies || body == payload) {
            throw AdapterError(e.steps, "set_payload_attachment: carrier " + std::to_string(body) + " for payload " +
                                            std::to_string(payload));
        }
        carrier = index_of(body);
        attached = true;
    }
    try {
        e.provider->set_attachment(index_of(payload), carrier, attached, e.state.q);
    } catch (const eng::EngineError& error) {
        throw AdapterError(e.steps, "set_payload_attachment: engine: " + error.cause());
    }
}

void NativeAdapter::set_transmission_slip(std::size_t transmission, double fraction) {
    Engine& e = engine("set_transmission_slip");
    if (transmission >= e.provider->model().transmissions.size() || !std::isfinite(fraction) || fraction < 0.0 ||
        fraction > 1.0) {
        throw AdapterError(e.steps, "set_transmission_slip: transmission " + std::to_string(transmission) +
                                        ", fraction in [0, 1]");
    }
    try {
        e.provider->set_slip(index_of(transmission), fraction);
    } catch (const eng::EngineError& error) {
        throw AdapterError(e.steps, "set_transmission_slip: engine: " + error.cause());
    }
}

}  // namespace mechsim::physics::native
