#pragma once

#include "mechsim/engine/dynamics_provider.hpp"
#include "mechsim/engine/state.hpp"
#include "mechsim/engine/step_config.hpp"
#include "mechsim/engine/step_health.hpp"

namespace mechsim::engine {

class Integrator {
public:
    virtual ~Integrator() = default;

    [[nodiscard]] virtual const StepConfig& config() const noexcept = 0;

    // Advances state by one step; state is untouched when it throws EngineError.
    virtual StepHealth step(const DynamicsProvider& provider, State& state) = 0;

protected:
    Integrator() = default;
    Integrator(const Integrator&) = default;
    Integrator(Integrator&&) = default;
    Integrator& operator=(const Integrator&) = default;
    Integrator& operator=(Integrator&&) = default;
};

// ADR 0013 option A: (H + h D + h^2 K) dv = h (f - c) - h^2 K v; v += dv; q = integrate_positions(q, v, h).
class SemiImplicitEuler final : public Integrator {
public:
    explicit SemiImplicitEuler(StepConfig config);

    [[nodiscard]] const StepConfig& config() const noexcept override;
    StepHealth step(const DynamicsProvider& provider, State& state) override;

private:
    StepConfig config_;
    DynamicsTerms terms_;
};

}  // namespace mechsim::engine
