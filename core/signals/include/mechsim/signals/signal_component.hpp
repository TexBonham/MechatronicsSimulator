#pragma once

#include <cstdint>
#include <map>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::signals {

using PhysicalState = kernel::PhysicsOutputs;

enum class FailureMode : std::uint8_t {
    stuck,         // hold the last value (SIG-006)
    noisy,         // apply the fault's noise (SIG-007)
    dropped,       // deliver no new samples (SIG-008)
    forced_level,  // present forced_level whatever the physical input (SIG-036)
};

// Active for start <= t < end. Noise draws come from the component's child generator (SIG-005); the noise
// distribution family is an open SIG parameter, so only the magnitude is carried here.
struct FailureInjection {
    FailureMode mode = FailureMode::stuck;
    kernel::SimTime start;
    kernel::SimTime end;
    double noise_magnitude = 0.0;  // in the signal's unit
    double forced_level = 0.0;
};

// Throws std::invalid_argument unless end > start, noise_magnitude >= 0 and both values are finite.
void validate_injection(const FailureInjection& injection);

// One interface for every signal-layer component, built-in C++ and Python slow path alike (ADR 0005, SIG-030).
// Per tick: sample() reads the physics state just published, set_inputs() receives pin outputs routed to this
// component, latch_outputs() fixes this tick's output values, output_values() reads them. Values are parallel to
// inputs() and outputs(), in SI or in the signal's stated unit (counts, levels as 0 or 1).
class SignalComponent {
public:
    SignalComponent() = default;
    virtual ~SignalComponent() = default;
    SignalComponent(const SignalComponent&) = delete;
    SignalComponent& operator=(const SignalComponent&) = delete;
    SignalComponent(SignalComponent&&) = delete;
    SignalComponent& operator=(SignalComponent&&) = delete;

    [[nodiscard]] virtual std::string_view instance_name() const = 0;
    // Dotted "instance.signal" names, the instance part equal to instance_name() (ADR 0012 D2).
    [[nodiscard]] virtual std::span<const std::string> inputs() const = 0;
    [[nodiscard]] virtual std::span<const std::string> outputs() const = 0;

    virtual void sample(const PhysicalState& state) = 0;
    virtual void set_inputs(std::span<const double> values) = 0;
    virtual void latch_outputs() = 0;
    [[nodiscard]] virtual std::span<const double> output_values() const = 0;

    virtual void inject(const FailureInjection& injection) = 0;
};

// The Python tier's surface: the binding derives its trampoline from this, so a slow-path component is a
// SignalComponent with no method of its own (ADR 0005; KER-019 for the party analogue).
class PythonSlowPathComponent : public SignalComponent {};

// Collects the signal names of every component in a design and rejects any that is not dotted, whose instance part
// differs from its component's instance name, or that repeats within the table (SIG-023: unique per design).
class SignalTable {
public:
    // Throws SignalNameError naming the offending signal; on a throw the table is unchanged.
    void add(const SignalComponent& component);

    [[nodiscard]] bool contains(std::string_view signal) const;
    [[nodiscard]] std::size_t size() const noexcept;

private:
    std::map<std::string, std::string, std::less<>> owner_by_signal_;
};

// No behaviour: zero outputs, records what it is given.
class NullSignalComponent final : public SignalComponent {
public:
    // Throws SignalNameError on a bad instance or signal name.
    NullSignalComponent(std::string instance, std::vector<std::string> inputs, std::vector<std::string> outputs);

    [[nodiscard]] std::string_view instance_name() const override;
    [[nodiscard]] std::span<const std::string> inputs() const override;
    [[nodiscard]] std::span<const std::string> outputs() const override;

    void sample(const PhysicalState& state) override;
    // Throws std::invalid_argument unless values.size() == inputs().size().
    void set_inputs(std::span<const double> values) override;
    void latch_outputs() override;
    [[nodiscard]] std::span<const double> output_values() const override;

    void inject(const FailureInjection& injection) override;

    [[nodiscard]] std::size_t samples() const noexcept;
    [[nodiscard]] std::size_t latches() const noexcept;
    [[nodiscard]] std::int64_t last_sample_step() const noexcept;
    [[nodiscard]] std::span<const FailureInjection> injections() const noexcept;

private:
    std::string instance_;
    std::vector<std::string> inputs_;
    std::vector<std::string> outputs_;
    std::vector<double> input_values_;
    std::vector<double> output_values_;
    std::vector<FailureInjection> injections_;
    std::size_t samples_ = 0;
    std::size_t latches_ = 0;
    std::int64_t last_sample_step_ = 0;
};

}  // namespace mechsim::signals
