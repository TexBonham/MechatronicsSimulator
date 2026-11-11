#include "mechsim/signals/component_base.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/signals/signal_name.hpp"

namespace mechsim::signals {

namespace {

constexpr std::uint64_t fnv_offset = 14695981039346656037ULL;
constexpr std::uint64_t fnv_prime = 1099511628211ULL;
constexpr double period_slack = 1e-9;

std::uint64_t mix(std::uint64_t z) noexcept {
    z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31U);
}

kernel::Quat conjugate(const kernel::Quat& q) noexcept {
    return {q[0], -q[1], -q[2], -q[3]};
}

kernel::Quat multiply(const kernel::Quat& a, const kernel::Quat& b) noexcept {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

kernel::Vec3 rotate(const kernel::Quat& q, const kernel::Vec3& v) noexcept {
    const kernel::Quat p{0.0, v[0], v[1], v[2]};
    const kernel::Quat r = multiply(multiply(q, p), conjugate(q));
    return {r[1], r[2], r[3]};
}

double wrap_pi(double angle) noexcept {
    const double two_pi = 2.0 * std::numbers::pi;
    double wrapped = std::fmod(angle + std::numbers::pi, two_pi);
    if (wrapped < 0.0) {
        wrapped += two_pi;
    }
    return wrapped - std::numbers::pi;
}

const kernel::BodyState& body_at(const PhysicalState& state, std::size_t index) {
    if (index >= state.bodies.size()) {
        throw SignalRunError("body index " + std::to_string(index) + " is outside the published state");
    }
    return state.bodies[index];
}

}  // namespace

std::uint64_t derive_child_seed(std::uint64_t run_seed, std::string_view instance) noexcept {
    std::uint64_t hash = fnv_offset;
    for (const char c : instance) {
        hash ^= static_cast<std::uint8_t>(c);
        hash *= fnv_prime;
    }
    return mix(run_seed ^ mix(hash));
}

SignalRng::SignalRng(std::uint64_t seed) noexcept : state_(seed) {}

std::uint64_t SignalRng::next() noexcept {
    state_ += 0x9E3779B97F4A7C15ULL;
    return mix(state_);
}

std::uint64_t SignalRng::state() const noexcept {
    return state_;
}

double SignalRng::uniform() noexcept {
    return static_cast<double>(next() >> 11U) * 0x1.0p-53;
}

double SignalRng::gaussian() noexcept {
    const double u1 = 1.0 - uniform();
    const double u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
}

std::int64_t periods_floor(double seconds, double period) {
    if (!std::isfinite(seconds) || seconds < 0.0 || !(period > 0.0)) {
        throw SignalConfigError("a time parameter must be finite and non-negative");
    }
    return static_cast<std::int64_t>(std::floor((seconds / period) + period_slack));
}

std::int64_t periods_ceil(double seconds, double period) {
    if (!std::isfinite(seconds) || seconds < 0.0 || !(period > 0.0)) {
        throw SignalConfigError("a time parameter must be finite and non-negative");
    }
    return std::max<std::int64_t>(0, static_cast<std::int64_t>(std::ceil((seconds / period) - period_slack)));
}

CoordinateReader::CoordinateReader(JointCoordinate coordinate) : coordinate_(coordinate) {
    const kernel::Vec3& a = coordinate_.axis;
    const double norm = std::sqrt((a[0] * a[0]) + (a[1] * a[1]) + (a[2] * a[2]));
    if (!std::isfinite(norm) || norm == 0.0) {
        throw SignalConfigError("coordinate axis must be finite and non-zero");
    }
    for (double& component : coordinate_.axis) {
        component /= norm;
    }
}

double CoordinateReader::read(const PhysicalState& state) {
    const kernel::BodyState& body = body_at(state, coordinate_.body);
    kernel::BodyState reference;
    if (coordinate_.reference.has_value()) {
        reference = body_at(state, *coordinate_.reference);
    }
    const kernel::Vec3& a = coordinate_.axis;
    if (coordinate_.kind == CoordinateKind::linear) {
        const kernel::Vec3 offset{body.position[0] - reference.position[0], body.position[1] - reference.position[1],
                                  body.position[2] - reference.position[2]};
        const kernel::Vec3 local = rotate(conjugate(reference.orientation), offset);
        return (local[0] * a[0]) + (local[1] * a[1]) + (local[2] * a[2]);
    }
    const kernel::Quat relative = multiply(conjugate(reference.orientation), body.orientation);
    const double along = (relative[1] * a[0]) + (relative[2] * a[1]) + (relative[3] * a[2]);
    const double wrapped = wrap_pi(2.0 * std::atan2(along, relative[0]));
    if (!primed_) {
        primed_ = true;
        unwrapped_ = wrapped;
    } else {
        unwrapped_ += wrap_pi(wrapped - last_wrapped_);
    }
    last_wrapped_ = wrapped;
    return unwrapped_;
}

BuiltinComponent::BuiltinComponent(std::string instance, const std::vector<std::string>& inputs,
                                   const std::vector<std::string>& outputs, std::vector<FailureMode> supported)
    : instance_(std::move(instance)), supported_(std::move(supported)) {
    if (!is_instance_name(instance_)) {
        throw SignalNameError("instance name '" + instance_ + "' is not a single identifier");
    }
    for (const std::string& name : inputs) {
        inputs_.push_back(instance_ + "." + name);
    }
    for (const std::string& name : outputs) {
        outputs_.push_back(instance_ + "." + name);
    }
    for (const auto names : {std::span<const std::string>(inputs_), std::span<const std::string>(outputs_)}) {
        for (const std::string& name : names) {
            if (!is_dotted_signal_name(name)) {
                throw SignalNameError("signal '" + name + "' is not a dotted instance.signal name");
            }
        }
    }
    input_values_.assign(inputs_.size(), 0.0);
    output_values_.assign(outputs_.size(), 0.0);
}

std::string_view BuiltinComponent::instance_name() const {
    return instance_;
}

std::span<const std::string> BuiltinComponent::inputs() const {
    return inputs_;
}

std::span<const std::string> BuiltinComponent::outputs() const {
    return outputs_;
}

void BuiltinComponent::set_inputs(std::span<const double> values) {
    if (values.size() != inputs_.size()) {
        throw std::invalid_argument("instance '" + instance_ + "' takes " + std::to_string(inputs_.size()) +
                                    " inputs, got " + std::to_string(values.size()));
    }
    input_values_.assign(values.begin(), values.end());
}

std::span<const double> BuiltinComponent::output_values() const {
    return output_values_;
}

void BuiltinComponent::inject(const FailureInjection& injection) {
    validate_injection(injection);
    if (std::find(supported_.begin(), supported_.end(), injection.mode) == supported_.end()) {
        throw std::invalid_argument("instance '" + instance_ + "' does not support this failure mode");
    }
    faults_.push_back(injection);
}

void BuiltinComponent::start(const ComponentClock& clock) {
    clock_ = clock;
    rng_ = SignalRng(clock.seed);
    samples_ = -1;
    on_start();
}

void BuiltinComponent::begin_tick(kernel::SimTime t) {
    now_ = t;
    ++samples_;
}

std::optional<FailureInjection> BuiltinComponent::active_fault() const {
    for (const FailureInjection& fault : std::views::reverse(faults_)) {
        if (fault.start <= now_ && now_ < fault.end) {
            return fault;
        }
    }
    return std::nullopt;
}

bool BuiltinComponent::fault_active(FailureMode mode) const {
    const std::optional<FailureInjection> fault = active_fault();
    return fault.has_value() && fault->mode == mode;
}

double BuiltinComponent::period_seconds() const noexcept {
    return kernel::to_seconds(clock_.period, clock_.base_rate);
}

std::int64_t BuiltinComponent::sample_index() const noexcept {
    return samples_;
}

double BuiltinComponent::input(std::size_t index) const {
    return input_values_.at(index);
}

void BuiltinComponent::set_output(std::size_t index, double value) {
    output_values_.at(index) = value;
}

double BuiltinComponent::output(std::size_t index) const {
    return output_values_.at(index);
}

const SignalRng& BuiltinComponent::generator() const noexcept {
    return rng_;
}

SignalRng& BuiltinComponent::rng() noexcept {
    return rng_;
}

const ComponentClock& BuiltinComponent::clock() const noexcept {
    return clock_;
}

std::span<const FailureInjection> BuiltinComponent::injections() const noexcept {
    return faults_;
}

}  // namespace mechsim::signals
