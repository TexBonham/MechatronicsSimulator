#pragma once

#include <cstdint>
#include <string>

#include "mechsim/signals/component_base.hpp"

namespace mechsim::signals {

// Catalog type limit_switch. position, hysteresis and resolution are in the coordinate's unit (m or rad).
struct LimitSwitchParams {
    JointCoordinate coordinate;
    double position = 0.0;       // switch position
    bool actuate_above = true;   // actuated at coordinate >= position; false: at coordinate <= position
    double hysteresis = 0.0;     // released once the coordinate is this far back past the position
    double resolution = 0.0;     // quantization step, floor; 0 = none (SIG-001)
    double debounce_time = 0.0;  // s (SIG-010)
    bool normally_open = true;   // level 1 = contact closed: actuated when normally open, released when closed
};

// Output `out` (level). Faults: stuck holds the level, dropped takes no new samples (level and debounce frozen),
// noisy adds N(0, noise_magnitude) in the coordinate's unit before the threshold, forced_level presents 0 or 1.
// Debounce: a change appears at the first sample at which the new contact state has held for debounce_time,
// ceil(debounce_time / period) periods after it was first seen; shorter changes never appear.
class LimitSwitch final : public BuiltinComponent {
public:
    // Throws SignalConfigError on a negative or non-finite parameter.
    LimitSwitch(std::string instance, LimitSwitchParams params);

    void sample(const PhysicalState& state) override;
    void latch_outputs() override;
    void inject(const FailureInjection& injection) override;

private:
    void on_start() override;

    LimitSwitchParams params_;
    CoordinateReader reader_;
    std::int64_t debounce_periods_ = 0;
    bool primed_ = false;
    bool actuated_ = false;
    bool debounced_ = false;
    bool pending_ = false;
    std::int64_t pending_since_ = 0;
};

// Catalog type incremental_encoder. resolution is the angle per quadrature count, rad (0.15 deg = 2400 counts per
// revolution in quadrature).
struct EncoderParams {
    JointCoordinate coordinate;  // angular
    double resolution = 0.0;
    bool reversed = false;  // count direction
};

// Outputs `channel_a`, `channel_b`: signed quadrature edges on each phase during the tick (SIG-011). The count
// position is n = floor(angle / resolution); an A edge is a step between n = 2k and 2k + 1, a B edge one between
// 2k + 1 and 2k + 2, so a_edges + b_edges = n_now - n_last exactly. The counter channel sums both.
// Faults: stuck and dropped deliver no edges (counts are lost, as on hardware), noisy adds round(N(0, sigma))
// miscounts on channel_a, forced_level presents round(forced_level) counts per tick on channel_a.
class IncrementalEncoder final : public BuiltinComponent {
public:
    // Throws SignalConfigError on a non-positive resolution or a linear coordinate.
    IncrementalEncoder(std::string instance, EncoderParams params);

    void sample(const PhysicalState& state) override;
    void latch_outputs() override;

private:
    EncoderParams params_;
    CoordinateReader reader_;
    bool primed_ = false;
    std::int64_t last_count_ = 0;
    std::int64_t count_ = 0;
};

}  // namespace mechsim::signals
