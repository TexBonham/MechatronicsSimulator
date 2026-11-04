#include "mechsim/physics/mujoco/mujoco_adapter.hpp"

// The prebuilt library is not built with ASan. Under -fsanitize=address mjsan.h would declare the ASan-only stack
// hooks (mj__markStack, absent from the release binary) with attributes GCC rejects; skipping it keeps the
// declarations of the release ABI.
#define MUJOCO_INCLUDE_MJSAN_H_  // NOLINT(bugprone-reserved-identifier,cppcoreguidelines-macro-usage)
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::physics::mujoco {

namespace {

using kernel::AdapterError;
using kernel::Vec3;
using kernel::Wrench;

// ===== Supported elements ===============================================
// Mirrored by mechsim/physics/mujoco/translate.py SUPPORTED_ELEMENTS (tests/unit/test_mjcf_translation.py checks).
constexpr std::array<std::string_view, 20> supported_kinds{
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
};
// ===== End supported elements ===========================================

// ===== Helpers ===========================================================

struct ModelDeleter {
    void operator()(mjModel* model) const noexcept { mj_deleteModel(model); }
};
struct DataDeleter {
    void operator()(mjData* data) const noexcept { mj_deleteData(data); }
};
struct SpecDeleter {
    void operator()(mjSpec* spec) const noexcept { mj_deleteSpec(spec); }
};
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;
using SpecPtr = std::unique_ptr<mjSpec, SpecDeleter>;

// body is -1 for a reflected shaft joint, whose one neutral DOF lives in shafts[shaft].
struct JointMap {
    int body = 0;
    int dofadr = 0;
    int dofnum = 0;
    int shaft = -1;
};

// A transmission shaft folded into its driven joint (armature J / c^2); driven = c * shaft.
struct ShaftMap {
    std::size_t joint = 0;
    int parent = 0;
    int driven_dof = 0;
    int driven_qpos = 0;
    double coefficient = 1.0;
    double efficiency = 1.0;
    double inertia = 0.0;
    Vec3 position{};
    std::array<double, 4> orientation{1.0, 0.0, 0.0, 0.0};
    Vec3 axis{0.0, 0.0, 1.0};
};

struct ConstraintMap {
    std::vector<int> equalities;
    bool hinge = false;
};

struct MemberMap {
    int parent = 0;
    int first = 0;
    int roller = -1;
    Vec3 position{};
};

constexpr std::array<int, 3> fatal_warnings{mjWARN_BADQPOS, mjWARN_BADQVEL, mjWARN_BADQACC};

std::vector<std::string> split(std::string_view text, char separator) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (start <= text.size()) {
        std::size_t end = text.find(separator, start);
        if (end == std::string_view::npos) {
            end = text.size();
        }
        if (end > start) {
            parts.emplace_back(text.substr(start, end - start));
        }
        start = end + 1;
    }
    return parts;
}

std::size_t to_size(std::int64_t value) {
    return static_cast<std::size_t>(value);
}

std::size_t offset(std::int64_t index, std::size_t stride) {
    return static_cast<std::size_t>(index) * stride;
}

std::string custom_text(const mjModel& model, const char* name) {
    const int id = mj_name2id(&model, mjOBJ_TEXT, name);
    if (id < 0) {
        return {};
    }
    return {model.text_data + model.text_adr[id]};
}

int require_id(const mjModel& model, mjtObj type, const std::string& name) {
    const int id = mj_name2id(&model, type, name.c_str());
    if (id < 0) {
        throw AdapterError(0, "MJCF artifact has no " + std::string(mju_type2Str(type)) + " '" + name + "'");
    }
    return id;
}

Vec3 at3(const mjtNum* values, int index) {
    const mjtNum* base = values + offset(index, 3);
    return {base[0], base[1], base[2]};
}

Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

Vec3 rotate(const mjtNum* matrix, const Vec3& v) {
    return {matrix[0] * v[0] + matrix[1] * v[1] + matrix[2] * v[2],
            matrix[3] * v[0] + matrix[4] * v[1] + matrix[5] * v[2],
            matrix[6] * v[0] + matrix[7] * v[1] + matrix[8] * v[2]};
}

Vec3 body_point(const mjData& data, int body, const Vec3& local) {
    return add(at3(data.xpos, body), rotate(data.xmat + offset(body, 9), local));
}

// MuJoCo spatial forces are [moment; force] about the subtree centre of mass; this moves the moment to point.
Wrench wrench_about(const mjModel& model, const mjData& data, int body, const Vec3& point) {
    const mjtNum* spatial = data.cfrc_int + offset(body, 6);
    const Vec3 force{spatial[3], spatial[4], spatial[5]};
    const Vec3 moment{spatial[0], spatial[1], spatial[2]};
    const Vec3 centre = at3(data.subtree_com, model.body_rootid[body]);
    Wrench wrench;
    wrench.force = force;
    wrench.moment = add(moment, cross(sub(centre, point), force));
    return wrench;
}

std::pair<Vec3, Vec3> connect_points(const mjModel& model, const mjData& data, int equality) {
    const int first = model.eq_obj1id[equality];
    const int second = model.eq_obj2id[equality];
    if (model.eq_objtype[equality] == mjOBJ_SITE) {
        return {at3(data.site_xpos, first), at3(data.site_xpos, second)};
    }
    const mjtNum* anchors = model.eq_data + offset(equality, mjNEQDATA);
    return {body_point(data, first, {anchors[0], anchors[1], anchors[2]}),
            body_point(data, second, {anchors[3], anchors[4], anchors[5]})};
}

int parse_count(const std::string& text, const std::string& entry) {
    int value = 0;
    const char* end = text.data() + text.size();
    const auto [last, status] = std::from_chars(text.data(), end, value);
    if (status != std::errc{} || last != end || value < 1) {
        throw AdapterError(0, "malformed count in mechsim entry '" + entry + "'");
    }
    return value;
}

std::vector<double> equality_rows(const mjData& data, int equality) {
    std::vector<double> rows;
    for (int row = 0; row < data.nefc; ++row) {
        if (data.efc_type[row] == mjCNSTR_EQUALITY && data.efc_id[row] == equality) {
            rows.push_back(data.efc_force[row]);
        }
    }
    return rows;
}

bool finite_state(const mjModel& model, const mjData& data) {
    for (int i = 0; i < model.nq; ++i) {
        if (!std::isfinite(data.qpos[i])) {
            return false;
        }
    }
    for (int i = 0; i < model.nv; ++i) {
        if (!std::isfinite(data.qvel[i]) || !std::isfinite(data.qacc[i])) {
            return false;
        }
    }
    return true;
}

}  // namespace

// ===== Engine ============================================================

struct MujocoAdapter::Engine {
    ModelPtr model;
    DataPtr data;
    kernel::Rate rate;
    std::vector<int> bodies;
    std::vector<std::size_t> neutral_of_body;
    std::vector<JointMap> joints;
    std::vector<int> springs;
    std::vector<ConstraintMap> constraints;
    std::vector<MemberMap> members;
    std::vector<ShaftMap> shafts;
    std::vector<int> shaft_of_body;
    std::vector<double> shaft_torque;
    std::vector<double> driven_velocity;
    std::vector<double> shaft_gain;  // eta driving, 1 / eta back-driven, for the step taken
    std::vector<kernel::PowerFlow> shaft_flow;
    std::size_t velocity_dofs = 0;
    kernel::PhysicsInputs pending;
    kernel::PhysicsOutputs outputs;
    kernel::PhysicsHealth health;

    [[nodiscard]] std::int64_t steps() const { return outputs.steps; }

    void map_elements();
    void collect_forces();
    void collect_kinematics();
    void collect_solver();
    void map_shafts(const std::vector<std::string>& body_names);
    void apply_inputs();
    void shaft_state(std::size_t body, kernel::BodyState& out) const;
    void clear_inputs();
};

// ===== Element maps =====================================================

void MujocoAdapter::Engine::map_elements() {
    const mjModel& m = *model;
    const std::vector<std::string> kinds = split(custom_text(m, "mechsim:elements"), ' ');
    if (kinds.empty()) {
        throw AdapterError(0, "MJCF artifact carries no mechsim:elements list; build it with mechsim.physics.mujoco");
    }
    for (const std::string& kind : kinds) {
        if (std::find(supported_kinds.begin(), supported_kinds.end(), kind) == supported_kinds.end()) {
            throw AdapterError(0, "unsupported neutral element kind '" + kind + "'");
        }
    }

    neutral_of_body.assign(to_size(m.nbody), kernel::kWorldBody);
    const std::vector<std::string> body_names = split(custom_text(m, "mechsim:bodies"), ' ');
    map_shafts(body_names);
    for (std::size_t i = 0; i < body_names.size(); ++i) {
        if (shaft_of_body[i] >= 0) {
            bodies.push_back(-1);
            continue;
        }
        const int id = require_id(m, mjOBJ_BODY, body_names[i]);
        neutral_of_body[to_size(id)] = bodies.size();
        bodies.push_back(id);
    }

    const std::vector<std::string> joint_tokens = split(custom_text(m, "mechsim:joints"), ' ');
    if (joint_tokens.size() != bodies.size()) {
        throw AdapterError(0, "MJCF artifact lists " + std::to_string(joint_tokens.size()) + " joints for " +
                                  std::to_string(bodies.size()) + " bodies");
    }
    for (std::size_t i = 0; i < joint_tokens.size(); ++i) {
        const std::vector<std::string> parts = split(joint_tokens[i], ':');
        if (parts.size() != 2) {
            throw AdapterError(0, "malformed mechsim:joints entry '" + joint_tokens[i] + "'");
        }
        if (shaft_of_body[i] >= 0) {
            shafts[to_size(shaft_of_body[i])].joint = i;
            joints.push_back({-1, -1, 1, shaft_of_body[i]});
            velocity_dofs += 1;
            springs.push_back(-1);
            continue;
        }
        const int body = bodies[i];
        joints.push_back({body, m.body_dofadr[body], m.body_dofnum[body], -1});
        velocity_dofs += to_size(m.body_dofnum[body]);
        if (parts[1] == "revolute" || parts[1] == "prismatic") {
            springs.push_back(require_id(m, mjOBJ_JOINT, parts[0]));
        } else if (parts[1] == "planar") {
            for (const char* suffix : {".x", ".y", ".theta"}) {
                springs.push_back(require_id(m, mjOBJ_JOINT, parts[0] + suffix));
            }
        }
    }

    for (const std::string& token : split(custom_text(m, "mechsim:constraints"), ' ')) {
        const std::vector<std::string> parts = split(token, ':');
        ConstraintMap map;
        if (parts.size() == 2 && parts[1] == "point") {
            map.equalities.push_back(require_id(m, mjOBJ_EQUALITY, parts[0]));
        } else if (parts.size() == 2 && parts[1] == "hinge") {
            map.hinge = true;
            map.equalities.push_back(require_id(m, mjOBJ_EQUALITY, parts[0] + ".point"));
            map.equalities.push_back(require_id(m, mjOBJ_EQUALITY, parts[0] + ".axis"));
        } else {
            throw AdapterError(0, "malformed mechsim:constraints entry '" + token + "'");
        }
        constraints.push_back(std::move(map));
    }

    for (const std::string& token : split(custom_text(m, "mechsim:members"), ' ')) {
        const std::vector<std::string> parts = split(token, ':');
        if (parts.size() != 5) {
            throw AdapterError(0, "malformed mechsim:members entry '" + token + "'");
        }
        const std::string& name = parts[0];
        const int segments = parse_count(parts[4], token);
        MemberMap map;
        map.parent = 0;
        if (parts[1] != "world") {
            map.parent = require_id(m, mjOBJ_BODY, parts[1]);
        }
        map.first = require_id(m, mjOBJ_BODY, name + ".segment0");
        if (parts[2] == "pinned") {
            map.first = require_id(m, mjOBJ_BODY, name + ".pin");
        }
        if (parts[3] == "roller") {
            map.roller = require_id(m, mjOBJ_EQUALITY, name + ".roller");
        }
        for (int j = 0; j < segments; ++j) {
            springs.push_back(require_id(m, mjOBJ_JOINT, name + ".node" + std::to_string(j)));
        }
        const int frame = require_id(m, mjOBJ_NUMERIC, "mechsim:member:" + name);
        if (m.numeric_size[frame] != 8) {
            throw AdapterError(0, "mechsim:member:" + name + " must hold 8 numbers");
        }
        const mjtNum* values = m.numeric_data + to_size(m.numeric_adr[frame]);
        map.position = {values[1], values[2], values[3]};
        members.push_back(map);
    }

}

// ===== Shafts ============================================================

// Transmissions reflect their shaft body onto the driven joint; the shaft body is absent from the MJCF.
void MujocoAdapter::Engine::map_shafts(const std::vector<std::string>& body_names) {
    const mjModel& m = *model;
    shaft_of_body.assign(body_names.size(), -1);
    for (const std::string& token : split(custom_text(m, "mechsim:transmissions"), ' ')) {
        const std::vector<std::string> parts = split(token, ':');
        if (parts.size() != 4) {
            throw AdapterError(0, "malformed mechsim:transmissions entry '" + token + "'");
        }
        const auto body = std::find(body_names.begin(), body_names.end(), parts[1]);
        if (body == body_names.end()) {
            throw AdapterError(0, "transmission '" + parts[0] + "': shaft body '" + parts[1] + "' is not listed");
        }
        const int frame = require_id(m, mjOBJ_NUMERIC, "mechsim:transmission:" + parts[0]);
        if (m.numeric_size[frame] != 13) {
            throw AdapterError(0, "mechsim:transmission:" + parts[0] + " must hold 13 numbers");
        }
        const mjtNum* v = m.numeric_data + to_size(m.numeric_adr[frame]);
        ShaftMap shaft;
        if (parts[2] != "world") {
            shaft.parent = require_id(m, mjOBJ_BODY, parts[2]);
        }
        const int driven = require_id(m, mjOBJ_JOINT, parts[3]);
        shaft.driven_dof = m.jnt_dofadr[driven];
        shaft.driven_qpos = m.jnt_qposadr[driven];
        shaft.coefficient = v[0];
        shaft.efficiency = v[1];
        shaft.inertia = v[2];
        shaft.position = {v[3], v[4], v[5]};
        shaft.orientation = {v[6], v[7], v[8], v[9]};
        shaft.axis = {v[10], v[11], v[12]};
        if (!std::isfinite(shaft.coefficient) || shaft.coefficient == 0.0 || !std::isfinite(shaft.efficiency) ||
            shaft.efficiency <= 0.0 || shaft.efficiency > 1.0) {
            throw AdapterError(0, "transmission '" + parts[0] + "': ratio must be nonzero, efficiency in (0, 1]");
        }
        shaft_of_body[to_size(std::distance(body_names.begin(), body))] = static_cast<int>(shafts.size());
        shafts.push_back(shaft);
    }
    shaft_torque.assign(shafts.size(), 0.0);
    driven_velocity.assign(shafts.size(), 0.0);
    shaft_gain.assign(shafts.size(), 1.0);
    shaft_flow.assign(shafts.size(), kernel::PowerFlow::Static);
}

// ===== Outputs ==========================================================

// Forces of the step just taken: MuJoCo's constraint forces and accelerations belong to the state at its start.
void MujocoAdapter::Engine::collect_forces() {
    const mjModel& m = *model;
    mjData& d = *data;
    mj_rnePostConstraint(&m, &d);

    outputs.joints.assign(joints.size(), {});
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const JointMap& joint = joints[i];
        kernel::JointForce& out = outputs.joints[i];
        if (joint.shaft >= 0) {
            // ENG-048 load torque: what the shaft supplies beyond accelerating its own rotor, J qacc_driven / c.
            const auto index = to_size(joint.shaft);
            const ShaftMap& shaft = shafts[index];
            const double acceleration = (d.qvel[shaft.driven_dof] - driven_velocity[index]) / m.opt.timestep;
            out.generalized_force = {shaft_torque[index] - shaft.inertia * acceleration / shaft.coefficient};
            continue;
        }
        const mjtNum* spatial = d.cfrc_int + offset(joint.body, 6);
        for (int k = 0; k < joint.dofnum; ++k) {
            const mjtNum* axis = d.cdof + offset(joint.dofadr + k, 6);
            out.generalized_force.push_back(mju_dot(axis, spatial, 6));
        }
        out.reaction = wrench_about(m, d, joint.body, at3(d.xpos, joint.body));
    }

    outputs.constraints.assign(constraints.size(), {});
    for (std::size_t i = 0; i < constraints.size(); ++i) {
        for (const int equality : constraints[i].equalities) {
            const std::vector<double> rows = equality_rows(d, equality);
            outputs.constraints[i].force.insert(outputs.constraints[i].force.end(), rows.begin(), rows.end());
        }
    }

    outputs.contacts.clear();
    for (int i = 0; i < d.ncon; ++i) {
        const mjContact& contact = d.contact[i];
        if (contact.efc_address < 0) {
            continue;
        }
        std::array<mjtNum, 6> local{};
        mj_contactForce(&m, &d, i, local.data());
        Vec3 on_second{};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            for (std::size_t k = 0; k < 3; ++k) {
                on_second[k] += local[axis] * contact.frame[3 * axis + k];
            }
        }
        kernel::ContactForce out;
        out.body_a = neutral_of_body[to_size(m.geom_bodyid[contact.geom[0]])];
        out.body_b = neutral_of_body[to_size(m.geom_bodyid[contact.geom[1]])];
        out.point = {contact.pos[0], contact.pos[1], contact.pos[2]};
        out.force = {-on_second[0], -on_second[1], -on_second[2]};
        outputs.contacts.push_back(out);
    }

    outputs.members.assign(members.size(), {});
    for (std::size_t i = 0; i < members.size(); ++i) {
        const MemberMap& member = members[i];
        const Vec3 root = body_point(d, member.parent, member.position);
        outputs.members[i].start = wrench_about(m, d, member.first, root);
        if (member.roller >= 0) {
            const std::vector<double> rows = equality_rows(d, member.roller);
            if (rows.size() == 3) {
                outputs.members[i].end.force = {rows[0], rows[1], rows[2]};
            }
        }
    }

    outputs.ropes.clear();
    outputs.transmissions.assign(shafts.size(), {});
    for (std::size_t i = 0; i < shafts.size(); ++i) {
        const ShaftMap& shaft = shafts[i];
        const double acceleration = (d.qvel[shaft.driven_dof] - driven_velocity[i]) / m.opt.timestep;
        kernel::TransmissionState& out = outputs.transmissions[i];
        out.shaft_load_torque = shaft_torque[i] - shaft.inertia * acceleration / shaft.coefficient;
        out.force = shaft_gain[i] * out.shaft_load_torque / shaft.coefficient;
        out.power_flow = shaft_flow[i];
    }
}

// Poses, velocities and constraint errors of the state the step produced.
void MujocoAdapter::Engine::collect_kinematics() {
    const mjModel& m = *model;
    const mjData& d = *data;
    outputs.bodies.assign(bodies.size(), {});
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const int body = bodies[i];
        kernel::BodyState& out = outputs.bodies[i];
        if (body < 0) {
            shaft_state(i, out);
            continue;
        }
        out.position = at3(d.xpos, body);
        const mjtNum* quat = d.xquat + offset(body, 4);
        out.orientation = {quat[0], quat[1], quat[2], quat[3]};
        std::array<mjtNum, 6> velocity{};
        mj_objectVelocity(&m, &d, mjOBJ_BODY, body, velocity.data(), 0);
        out.angular_velocity = {velocity[0], velocity[1], velocity[2]};
        out.linear_velocity = {velocity[3], velocity[4], velocity[5]};
    }

    double drift = 0.0;
    for (std::size_t i = 0; i < constraints.size(); ++i) {
        const ConstraintMap& map = constraints[i];
        const auto [a, b] = connect_points(m, d, map.equalities[0]);
        kernel::ConstraintForce& out = outputs.constraints[i];
        out.translational_error = norm(sub(a, b));
        out.rotational_error = 0.0;
        if (map.hinge) {
            const auto [a_axis, b_axis] = connect_points(m, d, map.equalities[1]);
            const Vec3 axis_a = sub(a_axis, a);
            const Vec3 axis_b = sub(b_axis, b);
            out.rotational_error = std::atan2(norm(cross(axis_a, axis_b)), dot(axis_a, axis_b));
        }
        if (d.eq_active[map.equalities[0]]) {
            drift = std::max({drift, out.translational_error, out.rotational_error});
        }
    }
    for (const MemberMap& member : members) {
        if (member.roller >= 0 && d.eq_active[member.roller]) {
            const auto [a, b] = connect_points(m, d, member.roller);
            drift = std::max(drift, norm(sub(a, b)));
        }
    }
    health.max_constraint_drift = drift;
}

// The shaft turns q_driven / c from its joint frame on the parent: pose and velocity follow the parent's.
void MujocoAdapter::Engine::shaft_state(std::size_t body, kernel::BodyState& out) const {
    const mjModel& m = *model;
    const mjData& d = *data;
    const ShaftMap& shaft = shafts[to_size(shaft_of_body[body])];
    const double angle = (d.qpos[shaft.driven_qpos] - m.qpos0[shaft.driven_qpos]) / shaft.coefficient;
    const double spin = d.qvel[shaft.driven_dof] / shaft.coefficient;
    const mjtNum* parent_quat = d.xquat + offset(shaft.parent, 4);
    std::array<mjtNum, 4> frame{};
    mju_mulQuat(frame.data(), parent_quat, shaft.orientation.data());
    std::array<mjtNum, 4> turn{};
    mju_axisAngle2Quat(turn.data(), shaft.axis.data(), angle);
    std::array<mjtNum, 4> orientation{};
    mju_mulQuat(orientation.data(), frame.data(), turn.data());
    out.orientation = orientation;
    const Vec3 parent_origin = at3(d.xpos, shaft.parent);
    out.position = body_point(d, shaft.parent, shaft.position);
    std::array<mjtNum, 6> parent_velocity{};
    mj_objectVelocity(&m, &d, mjOBJ_BODY, shaft.parent, parent_velocity.data(), 0);
    const Vec3 parent_omega{parent_velocity[0], parent_velocity[1], parent_velocity[2]};
    Vec3 axis_world{};
    mju_rotVecQuat(axis_world.data(), shaft.axis.data(), frame.data());
    out.angular_velocity = add(parent_omega, {spin * axis_world[0], spin * axis_world[1], spin * axis_world[2]});
    out.linear_velocity = add({parent_velocity[3], parent_velocity[4], parent_velocity[5]},
                              cross(parent_omega, sub(out.position, parent_origin)));
}

void MujocoAdapter::Engine::collect_solver() {
    const mjData& d = *data;
    const int islands = std::clamp(d.nisland, 1, mjNISLAND);
    int iterations = 0;
    for (int i = 0; i < islands; ++i) {
        iterations = std::max(iterations, d.solver_niter[i]);
    }
    health.iterations = iterations;
    health.converged = d.nefc == 0 || iterations < model->opt.iterations;
    health.steps = outputs.steps;
    health.physics_rate = rate;
}

// ===== Inputs ===========================================================

// Shaft torque tau reaches the driven joint as F = eta tau / c when it drives (tau qd_driven c >= 0, so standing
// still counts as driving) and F = tau / (eta c) when the load back-drives it (PHY-018, ENG-048).
void MujocoAdapter::Engine::apply_inputs() {
    const mjData& d = *data;
    for (std::size_t i = 0; i < shafts.size(); ++i) {
        shaft_torque[i] = 0.0;
        driven_velocity[i] = d.qvel[shafts[i].driven_dof];
        shaft_gain[i] = shafts[i].efficiency;
        shaft_flow[i] = kernel::PowerFlow::Static;
    }
    if (!pending.generalized_forces.empty()) {
        std::size_t offset = 0;
        for (const JointMap& joint : joints) {
            if (joint.shaft >= 0) {
                const auto index = to_size(joint.shaft);
                const ShaftMap& shaft = shafts[index];
                const double tau = pending.generalized_forces[offset];
                shaft_torque[index] = tau;
                const double power = tau * driven_velocity[index] * shaft.coefficient;
                shaft_gain[index] = 1.0 / shaft.efficiency;
                shaft_flow[index] = kernel::PowerFlow::Backward;
                if (power >= 0.0) {
                    shaft_gain[index] = shaft.efficiency;
                    shaft_flow[index] = kernel::PowerFlow::Forward;
                }
                if (power == 0.0) {
                    shaft_flow[index] = kernel::PowerFlow::Static;
                }
                const double force = shaft_gain[index] * tau / shaft.coefficient;
                d.qfrc_applied[shaft.driven_dof] += force;
                ++offset;
                continue;
            }
            for (int k = 0; k < joint.dofnum; ++k) {
                d.qfrc_applied[joint.dofadr + k] += pending.generalized_forces[offset];
                ++offset;
            }
        }
    }
    for (const kernel::BodyWrench& wrench : pending.body_wrenches) {
        mjtNum* target = d.xfrc_applied + offset(bodies[wrench.body], 6);
        for (std::size_t k = 0; k < 3; ++k) {
            target[k] += wrench.wrench.force[k];
            target[3 + k] += wrench.wrench.moment[k];
        }
    }
}

void MujocoAdapter::Engine::clear_inputs() {
    mju_zero(data->qfrc_applied, static_cast<int>(model->nv));
    mju_zero(data->xfrc_applied, static_cast<int>(6 * model->nbody));
    pending = {};
}

// ===== Adapter ==========================================================

MujocoAdapter::MujocoAdapter() : supported_(supported_kinds.begin(), supported_kinds.end()) {}

MujocoAdapter::~MujocoAdapter() = default;

std::span<const std::string> MujocoAdapter::supported_elements() const {
    return supported_;
}

MujocoAdapter::Engine& MujocoAdapter::engine(const char* operation) const {
    if (!engine_) {
        throw AdapterError(0, std::string(operation) + " before load()");
    }
    return *engine_;
}

void MujocoAdapter::load(const kernel::PreparedModel& model, const kernel::Rate& physics_rate) {
    if (engine_) {
        throw AdapterError(engine_->steps(), "load() called twice without reset()");
    }
    if (model.engine_artifact.empty()) {
        throw AdapterError(0, "the prepared model carries no MJCF artifact");
    }
    std::array<char, 1024> error{};
    const SpecPtr spec(mj_parseXMLString(model.engine_artifact.c_str(), nullptr, error.data(),
                                         static_cast<int>(error.size())));
    if (!spec) {
        throw AdapterError(0, std::string("MJCF does not parse: ") + error.data());
    }
    auto engine = std::make_unique<Engine>();
    engine->model = ModelPtr(mj_compile(spec.get(), nullptr));
    if (!engine->model) {
        throw AdapterError(0, std::string("MJCF does not compile: ") + mjs_getError(spec.get()));
    }
    mjModel& m = *engine->model;
    m.opt.timestep = static_cast<double>(physics_rate.denominator()) / static_cast<double>(physics_rate.numerator());
    m.opt.integrator = mjINT_IMPLICITFAST;
    // A non-finite state ends the run (invariant 7); MuJoCo would otherwise reset the data and carry on.
    m.opt.disableflags |= mjDSBL_AUTORESET;
    engine->rate = physics_rate;
    engine->map_elements();
    engine->data = DataPtr(mj_makeData(&m));
    if (!engine->data) {
        throw AdapterError(0, "MuJoCo could not allocate its data");
    }
    mj_forward(&m, engine->data.get());
    engine->collect_forces();
    engine->collect_kinematics();
    engine->collect_solver();
    engine_ = std::move(engine);
}

void MujocoAdapter::reset() {
    engine_.reset();
}

void MujocoAdapter::step() {
    Engine& e = engine("step()");
    const mjModel& m = *e.model;
    mjData& d = *e.data;
    std::array<int, fatal_warnings.size()> before{};
    for (std::size_t i = 0; i < fatal_warnings.size(); ++i) {
        before[i] = d.warning[fatal_warnings[i]].number;
    }
    e.apply_inputs();
    mj_step(&m, &d);
    for (std::size_t i = 0; i < fatal_warnings.size(); ++i) {
        const mjWarningStat& warning = d.warning[fatal_warnings[i]];
        if (warning.number != before[i]) {
            e.clear_inputs();
            throw AdapterError(e.steps(), std::string("MuJoCo: ") +
                                              mju_warningText(fatal_warnings[i], to_size(warning.lastinfo)));
        }
    }
    if (!finite_state(m, d)) {
        e.clear_inputs();
        throw AdapterError(e.steps(), "non-finite state after the step");
    }
    ++e.outputs.steps;
    e.collect_forces();
    e.collect_solver();
    e.clear_inputs();
    mj_kinematics(&m, &d);
    mj_comPos(&m, &d);
    mj_comVel(&m, &d);
    e.collect_kinematics();
}

const kernel::PhysicsOutputs& MujocoAdapter::outputs() const {
    return engine("outputs()").outputs;
}

kernel::PhysicsHealth MujocoAdapter::health() const {
    return engine("health()").health;
}

void MujocoAdapter::set_inputs(const kernel::PhysicsInputs& inputs) {
    Engine& e = engine("set_inputs()");
    if (!inputs.generalized_forces.empty() && inputs.generalized_forces.size() != e.velocity_dofs) {
        throw AdapterError(e.steps(), "set_inputs(): " + std::to_string(inputs.generalized_forces.size()) +
                                          " generalized forces for " + std::to_string(e.velocity_dofs) +
                                          " velocity DOFs");
    }
    for (const double value : inputs.generalized_forces) {
        if (!std::isfinite(value)) {
            throw AdapterError(e.steps(), "set_inputs(): non-finite generalized force");
        }
    }
    for (const kernel::BodyWrench& wrench : inputs.body_wrenches) {
        if (wrench.body >= e.bodies.size()) {
            throw AdapterError(e.steps(), "body " + std::to_string(wrench.body) + " is not in the model");
        }
        if (e.bodies[wrench.body] < 0) {
            throw AdapterError(e.steps(), "body " + std::to_string(wrench.body) +
                                              " is a reflected transmission shaft; drive it by its joint torque");
        }
        for (std::size_t k = 0; k < 3; ++k) {
            if (!std::isfinite(wrench.wrench.force[k]) || !std::isfinite(wrench.wrench.moment[k])) {
                throw AdapterError(e.steps(), "set_inputs(): non-finite body wrench");
            }
        }
    }
    e.pending = inputs;
}

// ===== Mid-run changes ===================================================

void MujocoAdapter::set_spring_offset(std::size_t spring, double offset) {
    const Engine& e = engine("set_spring_offset()");
    if (spring >= e.springs.size()) {
        throw AdapterError(e.steps(), "spring " + std::to_string(spring) + " is not in the model");
    }
    if (e.springs[spring] < 0) {
        throw AdapterError(e.steps(), "spring " + std::to_string(spring) + " is on a reflected transmission shaft");
    }
    if (!std::isfinite(offset)) {
        throw AdapterError(e.steps(), "set_spring_offset(): non-finite offset");
    }
    const mjModel& m = *e.model;
    m.qpos_spring[m.jnt_qposadr[e.springs[spring]]] = offset;
}

void MujocoAdapter::set_spring_stiffness(std::size_t spring, double stiffness) {
    const Engine& e = engine("set_spring_stiffness()");
    if (spring >= e.springs.size()) {
        throw AdapterError(e.steps(), "spring " + std::to_string(spring) + " is not in the model");
    }
    if (e.springs[spring] < 0) {
        throw AdapterError(e.steps(), "spring " + std::to_string(spring) + " is on a reflected transmission shaft");
    }
    if (!std::isfinite(stiffness) || stiffness < 0.0) {
        throw AdapterError(e.steps(), "set_spring_stiffness(): stiffness must be finite and not negative");
    }
    e.model->jnt_stiffness[e.springs[spring]] = stiffness;
}

void MujocoAdapter::disable_constraint(std::size_t constraint) {
    const Engine& e = engine("disable_constraint()");
    if (constraint >= e.constraints.size()) {
        throw AdapterError(e.steps(), "constraint " + std::to_string(constraint) + " is not in the model");
    }
    for (const int equality : e.constraints[constraint].equalities) {
        e.data->eq_active[equality] = false;
    }
}

void MujocoAdapter::release_load(std::size_t load) {
    const Engine& e = engine("release_load()");
    throw AdapterError(e.steps(), "load " + std::to_string(load) +
                                      " is not in the model: the MuJoCo adapter maps no ropes in milestone 1");
}

void MujocoAdapter::set_payload_mass(std::size_t payload, double mass) {
    const Engine& e = engine("set_payload_mass()");
    if (payload >= e.bodies.size()) {
        throw AdapterError(e.steps(), "payload " + std::to_string(payload) + " is not in the model");
    }
    if (e.bodies[payload] < 0) {
        throw AdapterError(e.steps(), "payload " + std::to_string(payload) + " is a reflected transmission shaft");
    }
    if (!std::isfinite(mass) || mass <= 0.0) {
        throw AdapterError(e.steps(), "set_payload_mass(): mass must be finite and positive");
    }
    mjModel& m = *e.model;
    const auto body = to_size(e.bodies[payload]);
    const double scale = mass / m.body_mass[body];
    m.body_mass[body] = mass;
    for (std::size_t k = 0; k < 3; ++k) {
        m.body_inertia[3 * body + k] *= scale;
    }
    // Derived constants (subtree masses, DOF inverse weights) follow the new mass; a scratch data keeps the state.
    const DataPtr scratch(mj_makeData(&m));
    if (!scratch) {
        throw AdapterError(e.steps(), "MuJoCo could not allocate its data");
    }
    mj_setConst(&m, scratch.get());
}

void MujocoAdapter::set_payload_attachment(std::size_t payload, std::size_t body) {
    const Engine& e = engine("set_payload_attachment()");
    if (payload >= e.bodies.size()) {
        throw AdapterError(e.steps(), "payload " + std::to_string(payload) + " is not in the model");
    }
    if (body >= e.bodies.size()) {
        throw AdapterError(e.steps(), "body " + std::to_string(body) + " is not in the model");
    }
    throw AdapterError(e.steps(), "set_payload_attachment(): MuJoCo cannot re-parent a body mid-run (milestone 1)");
}

std::string MujocoAdapter::integrator() const {
    const Engine& e = engine("integrator()");
    if (e.model->opt.integrator == mjINT_IMPLICITFAST) {
        return "implicitfast";
    }
    return "other";
}

double MujocoAdapter::timestep() const {
    return engine("timestep()").model->opt.timestep;
}

}  // namespace mechsim::physics::mujoco
