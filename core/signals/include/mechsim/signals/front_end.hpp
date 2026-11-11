#pragma once

#include <cstdint>
#include <string>

#include "mechsim/signals/component_base.hpp"

namespace mechsim::signals {

// The chip's ADC as the wiring reads it from the chip data (adc.resolution_bits, adc.reference_voltage).
struct AdcParams {
    std::int64_t bits = 12;     // 1 to 16
    double reference = 3.3;     // V, > 0
};

// Catalog type load_cell_amplifier.
struct AmplifierParams {
    double gain = 0.0;               // > 0
    double offset = 0.0;             // V, added after the gain
    double output_full_scale = 0.0;  // V, > 0
    double bandwidth = 0.0;          // Hz, > 0, first-order low-pass corner
    AdcParams adc;
};

// Input `signal_in` (V, the bridge output), output `out` (ADC counts), a front_end stage component (SIG-014).
// Per layer step h: x = clamp(gain signal_in + offset, 0, output_full_scale); y += (1 - exp(-2 pi bandwidth h))
// (x - y), y starting at the first x (no start-up transient), the exact discretization of the first-order lag for an
// input held over the step, so y_n = x (1 - exp(-2 pi bandwidth n h)) after a step from 0; counts =
// round(y / reference (2^bits - 1)), clamped to [0, 2^bits - 1].
class LoadCellAmplifier final : public BuiltinComponent {
public:
    // Throws SignalConfigError on a parameter out of range.
    LoadCellAmplifier(std::string instance, AmplifierParams params);

    void sample(const PhysicalState& state) override;
    void latch_outputs() override;
    [[nodiscard]] double voltage() const noexcept;  // the filtered output voltage y of the last step

private:
    void on_start() override;

    AmplifierParams params_;
    double alpha_ = 1.0;
    bool primed_ = false;
    double y_ = 0.0;
};

}  // namespace mechsim::signals
