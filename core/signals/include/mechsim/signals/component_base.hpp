#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/signal_component.hpp"

namespace mechsim::signals {

// Bad component parameters or layer wiring, found before t = 0.
class SignalConfigError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

// A failure during a run (index out of the published state, UART overflow, malformed line): ends the run through
// the kernel's PartyFault (invariant 7).
class SignalRunError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Child generator seed for one instance (SIG-005): FNV-1a of the instance name mixed with the run seed.
[[nodiscard]] std::uint64_t derive_child_seed(std::uint64_t run_seed, std::string_view instance) noexcept;

// splitmix64 with a Box-Muller normal; written out so draws are identical on every standard library.
class SignalRng {
public:
    explicit SignalRng(std::uint64_t seed = 0) noexcept;
    [[nodiscard]] std::uint64_t next() noexcept;
    [[nodiscard]] double uniform() noexcept;   // [0, 1)
    [[nodiscard]] double gaussian() noexcept;  // standard normal

private:
    std::uint64_t state_;
};

// Whole layer periods in a parameter time. The 1e-9 slack absorbs quotients such as 0.02 / 0.001 that must count as
// exactly 20 periods.
[[nodiscard]] std::int64_t periods_floor(double seconds, double period);
[[nodiscard]] std::int64_t periods_ceil(double seconds, double period);

enum class CoordinateKind : std::uint8_t {
    linear,   // m along axis
    angular,  // rad about axis, unwrapped across ticks
};

// A body coordinate read from the published state: the body origin relative to a reference body (world when
// absent), resolved in the reference frame along or about a unit axis given in that frame.
struct JointCoordinate {
    CoordinateKind kind = CoordinateKind::linear;
    std::size_t body = 0;
    std::optional<std::size_t> reference;
    kernel::Vec3 axis{1.0, 0.0, 0.0};
};

// Reads a JointCoordinate each tick. Angular readings unwrap by taking the wrapped difference to the last reading,
// so the joint must turn less than half a revolution per layer period.
class CoordinateReader {
public:
    // Throws SignalConfigError on a zero or non-finite axis.
    explicit CoordinateReader(JointCoordinate coordinate);
    // Throws SignalRunError when a body index is outside the published state.
    [[nodiscard]] double read(const PhysicalState& state);

private:
    JointCoordinate coordinate_;
    bool primed_ = false;
    double last_wrapped_ = 0.0;
    double unwrapped_ = 0.0;
};

// Shared surface of the C++ built-ins: names prefixed with the instance, parallel value vectors, the failure
// window schedule, the clock and the child generator.
class BuiltinComponent : public SignalComponent {
public:
    [[nodiscard]] std::string_view instance_name() const override;
    [[nodiscard]] std::span<const std::string> inputs() const override;
    [[nodiscard]] std::span<const std::string> outputs() const override;
    // Throws std::invalid_argument unless values.size() == inputs().size().
    void set_inputs(std::span<const double> values) override;
    [[nodiscard]] std::span<const double> output_values() const override;
    // Throws std::invalid_argument on an invalid window or a mode this component does not support.
    void inject(const FailureInjection& injection) override;
    void start(const ComponentClock& clock) override;
    void begin_tick(kernel::SimTime t) override;

protected:
    // Throws SignalNameError on a bad instance name; signal names are given without the instance part.
    BuiltinComponent(std::string instance, const std::vector<std::string>& inputs,
                     const std::vector<std::string>& outputs, std::vector<FailureMode> supported);

    // The latest injected fault whose window holds the current tick.
    [[nodiscard]] std::optional<FailureInjection> active_fault() const;
    [[nodiscard]] bool fault_active(FailureMode mode) const;
    [[nodiscard]] double period_seconds() const noexcept;
    [[nodiscard]] std::int64_t sample_index() const noexcept;  // 0 at the first begin_tick
    [[nodiscard]] double input(std::size_t index) const;
    void set_output(std::size_t index, double value);
    [[nodiscard]] double output(std::size_t index) const;
    [[nodiscard]] SignalRng& rng() noexcept;
    [[nodiscard]] const ComponentClock& clock() const noexcept;
    virtual void on_start() {}

private:
    std::string instance_;
    std::vector<std::string> inputs_;
    std::vector<std::string> outputs_;
    std::vector<double> input_values_;
    std::vector<double> output_values_;
    std::vector<FailureMode> supported_;
    std::vector<FailureInjection> faults_;
    ComponentClock clock_;
    kernel::SimTime now_;
    std::int64_t samples_ = -1;
    SignalRng rng_;
};

}  // namespace mechsim::signals
