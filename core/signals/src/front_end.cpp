#include "mechsim/signals/front_end.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>

namespace mechsim::signals {

namespace {

constexpr std::int64_t max_adc_bits = 16;

void require_positive(double value, const std::string& what) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw SignalConfigError(what + " must be finite and positive");
    }
}

}  // namespace

LoadCellAmplifier::LoadCellAmplifier(std::string instance, AmplifierParams params)
    : BuiltinComponent(std::move(instance), {"signal_in"}, {"out"}, {}), params_(params) {
    const std::string name = "amplifier '" + std::string(instance_name()) + "' ";
    require_positive(params_.gain, name + "gain");
    require_positive(params_.output_full_scale, name + "output full scale");
    require_positive(params_.bandwidth, name + "bandwidth");
    require_positive(params_.adc.reference, name + "ADC reference voltage");
    if (!std::isfinite(params_.offset)) {
        throw SignalConfigError(name + "offset must be finite");
    }
    if (params_.adc.bits < 1 || params_.adc.bits > max_adc_bits) {
        throw SignalConfigError(name + "ADC resolution must be 1 to 16 bits, got " + std::to_string(params_.adc.bits));
    }
}

void LoadCellAmplifier::on_start() {
    alpha_ = 1.0 - std::exp(-2.0 * std::numbers::pi * params_.bandwidth * period_seconds());
    primed_ = false;
    y_ = 0.0;
}

void LoadCellAmplifier::sample(const PhysicalState& /*state*/) {}

void LoadCellAmplifier::latch_outputs() {
    const double x = std::clamp((params_.gain * input(0)) + params_.offset, 0.0, params_.output_full_scale);
    if (!primed_) {
        primed_ = true;
        y_ = x;
    } else {
        y_ += alpha_ * (x - y_);
    }
    const double top = std::ldexp(1.0, static_cast<int>(params_.adc.bits)) - 1.0;
    const double counts = std::clamp(std::round(y_ / params_.adc.reference * top), 0.0, top);
    set_output(0, counts);
}

double LoadCellAmplifier::voltage() const noexcept {
    return y_;
}

}  // namespace mechsim::signals
