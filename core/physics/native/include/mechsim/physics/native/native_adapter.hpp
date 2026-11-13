#pragma once

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::physics::native {

// A model change the engine applied in a step (ENG-031 to ENG-035, ENG-047), in neutral terms: kind is one of
// spring_offset, spring_stiffness, constraint_disabled, load_released, rope_snapped, payload_attached, payload_mass;
// source (load_released) is constraint, rope or payload. Indices are neutral (bodies, constraints, members, ropes).
struct ModelChangeRecord {
    std::string kind;
    std::string source;
    long long target = -1;
    long long index = -1;
    double value = 0.0;
    double limit = 0.0;
    double time = 0.0;  // s
};

// The largest scheme decay pi omega h the engine published for one driven joint over the run (rulings 44, 45): the
// driven mode is known only at run time, since the drive stiffness is the controller's. joint is the neutral index.
struct DrivenSchemeDecay {
    std::size_t joint = 0;
    double decay = 0.0;
};

// The run's DrivenSchemeDecay values, shared: a caller that takes it before handing the adapter to a party reads the
// numbers after the run whatever has become of the adapter (no handle to the adapter itself).
struct DrivenSchemeDecayRecord {
    std::vector<DrivenSchemeDecay> values;
};

// Own-engine adapter (PHY-020, ADR 0013, ADR 0014): MultibodyProvider stepped by SemiImplicitEuler. load() parses
// the engine model document mechsim/physics/native/translate.py writes into PreparedModel::neutral_model_json;
// engine_artifact must be empty. Every engine error ends the run as AdapterError (PHY-009, invariant 7).
class NativeAdapter final : public kernel::PhysicsAdapter {
public:
    NativeAdapter();
    ~NativeAdapter() override;
    NativeAdapter(const NativeAdapter&) = delete;
    NativeAdapter& operator=(const NativeAdapter&) = delete;
    NativeAdapter(NativeAdapter&&) = delete;
    NativeAdapter& operator=(NativeAdapter&&) = delete;

    [[nodiscard]] std::span<const std::string> supported_elements() const override;
    void load(const kernel::PreparedModel& prepared, const kernel::Rate& physics_rate) override;
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

    // "backward_euler" (SemiImplicitEuler with implicit springs and dampers, ADR 0013 A) and the step, s (PHY-031).
    [[nodiscard]] std::string integrator() const;
    [[nodiscard]] double timestep() const;
    // Model changes of the last step(), each delivered once (outputs().events of the engine, collected per step).
    [[nodiscard]] const std::vector<ModelChangeRecord>& events() const;
    // Per rope after the last step: tension of every segment from the from end, N (PHY-004).
    [[nodiscard]] std::vector<std::vector<double>> rope_segment_tensions() const;
    // Per rope after the last step: the load-end segment's tension, N (PHY-004).
    [[nodiscard]] std::vector<double> rope_load_end_tensions() const;
    // Per joint that carried an enabled drive since load, in joint order: the largest scheme decay of any engine step.
    [[nodiscard]] std::vector<DrivenSchemeDecay> driven_scheme_decay() const;
    // The same values as a shared record that outlives the adapter; reset by each load.
    [[nodiscard]] std::shared_ptr<const DrivenSchemeDecayRecord> scheme_decay_record() const;

private:
    struct Engine;

    [[nodiscard]] Engine& engine(const char* operation) const;
    static void apply_inputs(Engine& e, const kernel::PhysicsInputs& inputs);
    static void clear_drives(Engine& e);
    static void set_substep_rests(Engine& e, int sub);
    static void keep_scheme_decay(Engine& e);

    std::vector<std::string> supported_;
    std::unique_ptr<Engine> engine_;
    std::shared_ptr<DrivenSchemeDecayRecord> decay_record_ = std::make_shared<DrivenSchemeDecayRecord>();
    kernel::PhysicsOutputs empty_;
};

}  // namespace mechsim::physics::native
