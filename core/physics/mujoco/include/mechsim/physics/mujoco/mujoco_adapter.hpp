#pragma once

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::physics::mujoco {

// PhysicsAdapter over MuJoCo's C API with implicitfast at 1 / physics_rate (PHY-014, ADR 0014). load() compiles the
// MJCF in PreparedModel::engine_artifact, written by mechsim/physics/mujoco/translate.py; its mechsim custom block
// gives the neutral element order. No MuJoCo type appears in this header (invariant 5, PHY-011).
//
// Index spaces (neutral file order): bodies; joints (one per body); velocity DOFs (joint order, linear then
// angular); springs (each revolute, prismatic and planar joint DOF in joint order, then each lumped-member node,
// member order, root to tip); constraints; members; transmissions; payloads and loads are bodies, and only the
// translation's payloads (each welded to one carrier) can be released or re-attached (PHY-006, ENG-035).
class MujocoAdapter final : public kernel::PhysicsAdapter {
public:
    MujocoAdapter();
    ~MujocoAdapter() override;
    MujocoAdapter(const MujocoAdapter&) = delete;
    MujocoAdapter& operator=(const MujocoAdapter&) = delete;
    MujocoAdapter(MujocoAdapter&&) = delete;
    MujocoAdapter& operator=(MujocoAdapter&&) = delete;

    [[nodiscard]] std::span<const std::string> supported_elements() const override;
    void load(const kernel::PreparedModel& model, const kernel::Rate& physics_rate) override;
    void reset() override;
    void step() override;

    [[nodiscard]] const kernel::PhysicsOutputs& outputs() const override;
    [[nodiscard]] kernel::PhysicsHealth health() const override;

    void set_inputs(const kernel::PhysicsInputs& inputs) override;

    void set_spring_offset(std::size_t spring, double offset) override;
    void set_spring_stiffness(std::size_t spring, double stiffness) override;
    void disable_constraint(std::size_t constraint) override;
    void release_load(std::size_t load) override;
    void set_payload_mass(std::size_t payload, double mass) override;
    void set_payload_attachment(std::size_t payload, std::size_t body) override;
    void set_transmission_slip(std::size_t transmission, double fraction) override;

    // The integrator and step MuJoCo runs with after load(), for PHY-014 checks: "implicitfast" and seconds.
    [[nodiscard]] std::string integrator() const;
    [[nodiscard]] double timestep() const;

private:
    struct Engine;

    [[nodiscard]] Engine& engine(const char* operation) const;

    std::vector<std::string> supported_;
    std::unique_ptr<Engine> engine_;
};

}  // namespace mechsim::physics::mujoco
