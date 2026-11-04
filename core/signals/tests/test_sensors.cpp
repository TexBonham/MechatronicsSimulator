#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>
#include <stdexcept>
#include <vector>

#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/sensors.hpp"
#include "signal_test_support.hpp"

using mechsim::kernel::SimTime;
using mechsim::signals::CoordinateKind;
using mechsim::signals::EncoderParams;
using mechsim::signals::FailureInjection;
using mechsim::signals::FailureMode;
using mechsim::signals::IncrementalEncoder;
using mechsim::signals::JointCoordinate;
using mechsim::signals::LimitSwitch;
using mechsim::signals::LimitSwitchParams;
using mechsim::signals::SignalConfigError;
using signal_test::state_at;
using signal_test::Stepper;

namespace {

// 0.15 deg per count: 2400 quadrature counts per revolution (catalog part encoder_600ppr_generic).
constexpr double counts_per_rev = 2400.0;
constexpr double resolution = 2.0 * std::numbers::pi / counts_per_rev;

EncoderParams encoder_params() {
    EncoderParams params;
    params.coordinate = JointCoordinate{CoordinateKind::angular, 0, {}, {0.0, 0.0, 1.0}};
    params.resolution = resolution;
    return params;
}

// Exact binary steps keep every threshold comparison free of rounding: positions are multiples of 1/1024 m.
constexpr double tick_step = 1.0 / 1024.0;
constexpr double switch_position = 0.5;

LimitSwitchParams switch_params() {
    LimitSwitchParams params;
    params.coordinate = JointCoordinate{CoordinateKind::linear, 0, {}, {1.0, 0.0, 0.0}};
    params.position = switch_position;
    return params;
}

}  // namespace

TEST_CASE("Encoder quantizes to its resolution and rejects bad parameters", "[SIG-001][SIG-011]") {
    IncrementalEncoder encoder("x_encoder", encoder_params());
    Stepper run(encoder);
    run.step(state_at(0.5 * resolution, 0.0));
    // 10.7 counts from n = 0: floor(11.2) - floor(0.5) = 11; the fractional 0.2 count is not reported.
    const std::span<const double> out = run.step(state_at(11.2 * resolution, 0.0));
    CHECK(out[0] + out[1] == 11.0);

    CHECK_THROWS_AS(IncrementalEncoder("e", EncoderParams{}), SignalConfigError);
    EncoderParams linear = encoder_params();
    linear.coordinate.kind = CoordinateKind::linear;
    CHECK_THROWS_AS(IncrementalEncoder("e", linear), SignalConfigError);
}

TEST_CASE("Limit switch level changes at the switch position and releases past the hysteresis", "[SIG-009]") {
    LimitSwitchParams params = switch_params();
    params.hysteresis = 4.0 * tick_step;
    LimitSwitch limit("x_max_switch", params);
    Stepper run(limit);
    // Approach from 10 steps below: x_k = 0.5 - (10 - k) / 1024, so x_k >= 0.5 first at k = 10.
    for (int k = 0; k < 10; ++k) {
        CHECK(run.step(state_at(0.0, switch_position - ((10 - k) * tick_step)))[0] == 0.0);
    }
    CHECK(run.step(state_at(0.0, switch_position))[0] == 1.0);
    // Back off: still actuated down to 0.5 - 4/1024 (inside the hysteresis band), released at 0.5 - 5/1024.
    for (int k = 1; k <= 4; ++k) {
        CHECK(run.step(state_at(0.0, switch_position - (k * tick_step)))[0] == 1.0);
    }
    CHECK(run.step(state_at(0.0, switch_position - (5.0 * tick_step)))[0] == 0.0);

    LimitSwitchParams below = switch_params();
    below.actuate_above = false;
    below.normally_open = false;
    LimitSwitch home("x_min_switch", below);
    Stepper home_run(home);
    // Actuated at x <= 0.5; normally closed, so actuated reads 0.
    CHECK(home_run.step(state_at(0.0, 0.75))[0] == 1.0);
    CHECK(home_run.step(state_at(0.0, 0.5))[0] == 0.0);
}

TEST_CASE("Limit switch quantizes its position to its resolution before the threshold", "[SIG-001][SIG-009]") {
    LimitSwitchParams params = switch_params();
    params.resolution = 0.25;
    LimitSwitch limit("x_max_switch", params);
    Stepper run(limit);
    // floor(0.49 / 0.25) * 0.25 = 0.25 < 0.5: not actuated; 0.5 quantizes to itself: actuated.
    CHECK(run.step(state_at(0.0, 0.49))[0] == 0.0);
    CHECK(run.step(state_at(0.0, 0.5))[0] == 1.0);
}

TEST_CASE("Debounce delays a change by ceil(debounce / period) samples and drops shorter pulses", "[SIG-010]") {
    // debounce 5 ms at 1 kHz: K = 5. A change first seen at sample i appears at sample i + 5; a raw pulse seen on
    // m samples has held (m - 1) periods, so it appears only for m - 1 >= 5, i.e. m >= 6.
    LimitSwitchParams params = switch_params();
    params.debounce_time = 0.005;
    LimitSwitch limit("x_max_switch", params);
    Stepper run(limit);
    const double out = 0.25;
    const double in = 0.75;
    CHECK(run.step(state_at(0.0, out))[0] == 0.0);
    for (int k = 0; k < 5; ++k) {
        CHECK(run.step(state_at(0.0, in))[0] == 0.0);
    }
    CHECK(run.step(state_at(0.0, in))[0] == 1.0);

    LimitSwitch bouncing("x_max_switch", params);
    Stepper bounce(bouncing);
    bounce.step(state_at(0.0, out));
    for (int k = 0; k < 5; ++k) {
        CHECK(bounce.step(state_at(0.0, in))[0] == 0.0);
    }
    for (int k = 0; k < 10; ++k) {
        CHECK(bounce.step(state_at(0.0, out))[0] == 0.0);
    }
}

TEST_CASE("Limit switch stuck, dropped and forced-level faults hold for the window only",
          "[SIG-006][SIG-008][SIG-036]") {
    LimitSwitch stuck("s", switch_params());
    stuck.inject(FailureInjection{FailureMode::stuck, SimTime(1), SimTime(3), 0.0, 0.0});
    Stepper stuck_run(stuck);
    CHECK(stuck_run.step(state_at(0.0, 0.0))[0] == 0.0);
    CHECK(stuck_run.step(state_at(0.0, 1.0))[0] == 0.0);
    CHECK(stuck_run.step(state_at(0.0, 1.0))[0] == 0.0);
    CHECK(stuck_run.step(state_at(0.0, 1.0))[0] == 1.0);

    LimitSwitch dropped("s", switch_params());
    dropped.inject(FailureInjection{FailureMode::dropped, SimTime(1), SimTime(2), 0.0, 0.0});
    Stepper dropped_run(dropped);
    CHECK(dropped_run.step(state_at(0.0, 1.0))[0] == 1.0);
    CHECK(dropped_run.step(state_at(0.0, 0.0))[0] == 1.0);
    CHECK(dropped_run.step(state_at(0.0, 0.0))[0] == 0.0);

    LimitSwitch forced("s", switch_params());
    forced.inject(FailureInjection{FailureMode::forced_level, SimTime(1), SimTime(2), 0.0, 1.0});
    Stepper forced_run(forced);
    CHECK(forced_run.step(state_at(0.0, 0.0))[0] == 0.0);
    CHECK(forced_run.step(state_at(0.0, 0.0))[0] == 1.0);
    CHECK(forced_run.step(state_at(0.0, 0.0))[0] == 0.0);

    CHECK_THROWS_AS(forced.inject(FailureInjection{FailureMode::forced_level, SimTime(1), SimTime(2), 0.0, 0.5}),
                    std::invalid_argument);
    CHECK_THROWS_AS(forced.inject(FailureInjection{FailureMode::stall, SimTime(1), SimTime(2), 0.0, 0.0}),
                    std::invalid_argument);
}

TEST_CASE("Limit switch noisy fault perturbs the sensed position only inside the window", "[SIG-007]") {
    // Position 0 sits 0.5 m below the switch. Noise sigma 10 m: each sample is actuated with probability
    // P(N(0, 10) >= 0.5) = 0.48, so 100 noisy samples without a single actuation has probability 0.52^100 < 1e-28.
    // Outside the window the level is the noiseless 0.
    LimitSwitch noisy("s", switch_params());
    noisy.inject(FailureInjection{FailureMode::noisy, SimTime(1), SimTime(101), 10.0, 0.0});
    Stepper run(noisy);
    CHECK(run.step(state_at(0.0, 0.0))[0] == 0.0);
    int actuated = 0;
    for (int k = 0; k < 100; ++k) {
        actuated += static_cast<int>(run.step(state_at(0.0, 0.0))[0]);
    }
    CHECK(actuated > 0);
    CHECK(actuated < 100);
    CHECK(run.step(state_at(0.0, 0.0))[0] == 0.0);
}

TEST_CASE("Encoder stuck and dropped lose counts, forced presents its count, noisy miscounts",
          "[SIG-006][SIG-007][SIG-008][SIG-036]") {
    // One turn in 1000 ticks; a window over ticks 1 .. 100 delivers no edges while the count position moves on, so
    // the counts of those ticks are lost: n(100) - n(0) = floor(0.5 + 240) - floor(0.5) = 240.
    for (const FailureMode mode : {FailureMode::stuck, FailureMode::dropped}) {
        IncrementalEncoder encoder("e", encoder_params());
        encoder.inject(FailureInjection{mode, SimTime(1), SimTime(101), 0.0, 0.0});
        Stepper run(encoder);
        const double start = 0.5 * resolution;
        run.step(state_at(start, 0.0));
        std::int64_t total = 0;
        for (int k = 1; k <= 1000; ++k) {
            const std::span<const double> out = run.step(state_at(start + (2.0 * std::numbers::pi * k / 1000), 0.0));
            if (k <= 100) {
                CHECK(out[0] == 0.0);
                CHECK(out[1] == 0.0);
            }
            total += std::llround(out[0] + out[1]);
        }
        CHECK(total == 2400 - 240);
    }

    IncrementalEncoder forced("e", encoder_params());
    forced.inject(FailureInjection{FailureMode::forced_level, SimTime(0), SimTime(3), 0.0, 3.0});
    Stepper forced_run(forced);
    CHECK(forced_run.step(state_at(0.0, 0.0))[0] == 3.0);
    CHECK(forced_run.step(state_at(0.0, 0.0))[0] == 3.0);

    IncrementalEncoder noisy("e", encoder_params());
    // sigma 5 counts on a still shaft: |round(N(0, 5))| >= 1 with probability 0.92 per tick, so 50 ticks all zero
    // has probability below 1e-50.
    noisy.inject(FailureInjection{FailureMode::noisy, SimTime(1), SimTime(51), 5.0, 0.0});
    Stepper noisy_run(noisy);
    noisy_run.step(state_at(0.5 * resolution, 0.0));
    int miscounted = 0;
    for (int k = 0; k < 50; ++k) {
        miscounted += static_cast<int>(noisy_run.step(state_at(0.5 * resolution, 0.0))[0] != 0.0);
    }
    CHECK(miscounted > 0);
    CHECK(noisy_run.step(state_at(0.5 * resolution, 0.0))[0] == 0.0);
}
