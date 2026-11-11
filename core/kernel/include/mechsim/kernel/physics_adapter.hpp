#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

// Any adapter failure: drift past threshold, solver failure, unmappable element, call out of order (PHY-009,
// PHY-019). It ends the run; it is never a returned number. step is the completed step count when raised.
class AdapterError : public std::runtime_error {
public:
    AdapterError(std::int64_t step, std::string cause);

    [[nodiscard]] std::int64_t step() const noexcept;
    [[nodiscard]] const std::string& cause() const noexcept;

private:
    std::int64_t step_;
    std::string cause_;
};

// What the builder hands an adapter once per run.
struct PreparedModel {
    std::string neutral_model_json;  // the neutral physics model document; each adapter parses it (invariant 3)
    std::string engine_artifact;     // MJCF text for the MuJoCo stepping adapter (ADR 0014); empty for native
    std::string design_version;      // the design's semver string, kept for reports
};

// One stepping interface for every engine (PHY-001, ADR 0014): core/physics/native/ over core/engine/, and
// core/physics/mujoco/ over MuJoCo's C API. No engine type appears here (invariant 5, PHY-011, PHY-012).
//
// Contract:
// - supported_elements() lists the neutral-model element kinds the adapter maps, so the builder refuses an
//   unmappable model before load (PHY-019); load() also refuses one, with an AdapterError.
// - load() takes the prepared model and the physics rate once per run; the step length 1 / physics_rate is fixed
//   for the run. A second load() without reset() throws.
// - reset() returns the adapter to its unloaded state at step 0.
// - step() advances exactly one physics step (PHY-002); simulated time moves only here and never reads the wall
//   clock (PHY-032). Same model, inputs and seed give identical outputs at every step (PHY-010).
// - Inputs set before a step apply to that step only (PHY-005, ENG-028).
// - outputs() and health() describe the last completed step; before the first step they describe step 0.
// - Mid-run changes (PHY-008, ENG-031 to ENG-035) take effect from the next step and stay for the rest of the run.
// - Any call but load(), reset() and supported_elements() before load(), and any element index outside the model,
//   throws AdapterError.
class PhysicsAdapter {
public:
    PhysicsAdapter() = default;
    virtual ~PhysicsAdapter() = default;
    PhysicsAdapter(const PhysicsAdapter&) = delete;
    PhysicsAdapter& operator=(const PhysicsAdapter&) = delete;
    PhysicsAdapter(PhysicsAdapter&&) = delete;
    PhysicsAdapter& operator=(PhysicsAdapter&&) = delete;

    [[nodiscard]] virtual std::span<const std::string> supported_elements() const = 0;
    virtual void load(const PreparedModel& model, const Rate& physics_rate) = 0;
    virtual void reset() = 0;
    virtual void step() = 0;

    [[nodiscard]] virtual const PhysicsOutputs& outputs() const = 0;
    [[nodiscard]] virtual PhysicsHealth health() const = 0;

    virtual void set_inputs(const PhysicsInputs& inputs) = 0;

    virtual void set_spring_offset(std::size_t spring, double offset) = 0;        // m or rad; yield (ENG-031)
    virtual void set_spring_stiffness(std::size_t spring, double stiffness) = 0;  // N/m or N m/rad (ENG-032)
    virtual void disable_constraint(std::size_t constraint) = 0;                  // fastener break (ENG-033)
    virtual void release_load(std::size_t load) = 0;                              // rope snap (ENG-034)
    virtual void set_payload_mass(std::size_t payload, double mass) = 0;          // kg (ENG-035)
    virtual void set_payload_attachment(std::size_t payload, std::size_t body) = 0;  // re-attach (ENG-035)

    // Dynamic slip fault on a transmission (neutral order) for the fault window (PHY-006, TST-015): fraction in
    // [0, 1] of the commanded shaft travel lost; 0 clears it. Adapters without a slip model refuse it.
    virtual void set_transmission_slip(std::size_t transmission, double /*fraction*/) {
        throw AdapterError(0, "transmission " + std::to_string(transmission) + ": this adapter has no slip model");
    }
};

// Maps no element kind and publishes zero-sized outputs; the step count advances.
class NullPhysicsAdapter final : public PhysicsAdapter {
public:
    [[nodiscard]] std::span<const std::string> supported_elements() const override;
    void load(const PreparedModel& model, const Rate& physics_rate) override;
    void reset() override;
    void step() override;

    [[nodiscard]] const PhysicsOutputs& outputs() const override;
    [[nodiscard]] PhysicsHealth health() const override;

    void set_inputs(const PhysicsInputs& inputs) override;

    void set_spring_offset(std::size_t spring, double offset) override;
    void set_spring_stiffness(std::size_t spring, double stiffness) override;
    void disable_constraint(std::size_t constraint) override;
    void release_load(std::size_t load) override;
    void set_payload_mass(std::size_t payload, double mass) override;
    void set_payload_attachment(std::size_t payload, std::size_t body) override;

    [[nodiscard]] bool loaded() const noexcept;
    [[nodiscard]] const std::string& design_version() const noexcept;

private:
    void require_loaded(const char* operation) const;

    bool loaded_ = false;
    std::string design_version_;
    std::vector<std::string> supported_;
    PhysicsOutputs outputs_;
    PhysicsHealth health_;
};

}  // namespace mechsim::kernel
