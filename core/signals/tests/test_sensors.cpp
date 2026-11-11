#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
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
using mechsim::signals::PhysicalState;
using mechsim::signals::SampledSensor;
using mechsim::signals::SensorFidelity;
using mechsim::signals::SensorOutputKind;
using mechsim::signals::SignalConfigError;
using mechsim::signals::SignalRng;
using mechsim::signals::derive_child_seed;
using signal_test::khz_clock;
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

TEST_CASE("Debounce delays a change by ceil(debounce / period) samples", "[SIG-010]") {
    // debounce 5 ms at 1 kHz: N = 5. A change first sampled at i has held N periods at sample i + N and appears there.
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
    // sigma 5 counts of angle (in rad) on a still shaft half a count into its cell: the count stays put only while
    // N(0, 5) lies in [-0.5, 0.5), probability below 0.08 per tick, so 50 ticks all zero has probability below 1e-50.
    noisy.inject(FailureInjection{FailureMode::noisy, SimTime(1), SimTime(51), 5.0 * resolution, 0.0});
    Stepper noisy_run(noisy);
    noisy_run.step(state_at(0.5 * resolution, 0.0));
    int miscounted = 0;
    std::int64_t net = 0;
    for (int k = 0; k < 50; ++k) {
        const std::span<const double> out = noisy_run.step(state_at(0.5 * resolution, 0.0));
        miscounted += static_cast<int>(out[0] + out[1] != 0.0);
        net += std::llround(out[0] + out[1]);
    }
    CHECK(miscounted > 0);
    // The first sample after the window is noiseless again: its edges return the count to the true cell, so the
    // window's miscounts cancel, and from then on a still shaft gives no edges.
    const std::span<const double> back = noisy_run.step(state_at(0.5 * resolution, 0.0));
    CHECK(net + std::llround(back[0] + back[1]) == 0);
    const std::span<const double> still = noisy_run.step(state_at(0.5 * resolution, 0.0));
    CHECK(still[0] == 0.0);
    CHECK(still[1] == 0.0);
}

namespace {

// Ruling 32: a linear encoder on the carriage, 60000 quadrature counts per metre, so resolution = 1 / 60000 m.
constexpr double linear_resolution = 1.0 / 60000.0;

EncoderParams linear_params() {
    EncoderParams params;
    params.coordinate = JointCoordinate{CoordinateKind::linear, 0, {}, {1.0, 0.0, 0.0}};
    params.resolution = linear_resolution;
    params.linear = true;
    return params;
}

struct EdgeTotals {
    std::int64_t a = 0;
    std::int64_t b = 0;
};

// The carriage starts half a count into cell 0 and travels `travel` m in 1000 equal ticks.
EdgeTotals travel_counts(IncrementalEncoder& encoder, double travel) {
    Stepper run(encoder);
    const double start = 0.5 * linear_resolution;
    run.step(state_at(0.0, start));
    EdgeTotals totals;
    for (int k = 1; k <= 1000; ++k) {
        const std::span<const double> out = run.step(state_at(0.0, start + (travel * k / 1000.0)));
        totals.a += std::llround(out[0]);
        totals.b += std::llround(out[1]);
    }
    return totals;
}

std::string config_error(const EncoderParams& params) {
    try {
        const IncrementalEncoder encoder("y_encoder", params);
    } catch (const SignalConfigError& error) {
        return error.what();
    }
    return {};
}

}  // namespace

TEST_CASE("A linear encoder counts quadrature edges of a prismatic coordinate at its counts per metre",
          "[SIG-011][LIB-025]") {
    // 0.1 m at 60000 counts per metre is 6000 counts: n = floor((0.5 r + 0.1) / r) - floor(0.5) = 6000 (the half
    // count keeps every sample 0.5 count away from a cell boundary, far beyond the ~1e-12 count rounding of x / r).
    // Each tick moves exactly 6 counts from an even n = 6(k - 1), so it crosses 3 A edges (2j -> 2j + 1) and 3 B
    // edges (2j + 1 -> 2j + 2): 3000 each over 1000 ticks, the split the angular case gives.
    IncrementalEncoder forward("y_encoder", linear_params());
    const EdgeTotals ahead = travel_counts(forward, 0.1);
    CHECK(ahead.a == 3000);
    CHECK(ahead.b == 3000);

    // Reversed counts the same travel down: every edge negated.
    EncoderParams flipped = linear_params();
    flipped.reversed = true;
    IncrementalEncoder reversed("y_encoder", flipped);
    const EdgeTotals back = travel_counts(reversed, 0.1);
    CHECK(back.a == -3000);
    CHECK(back.b == -3000);

    // Travel the other way counts down without reversed: n(0.5 r - 0.1) = floor(0.5 - 6000) = -6000.
    IncrementalEncoder retreat("y_encoder", linear_params());
    const EdgeTotals down = travel_counts(retreat, -0.1);
    CHECK(down.a + down.b == -6000);
}

TEST_CASE("A linear encoder's stuck and dropped faults lose counts as the angular one does", "[SIG-006][SIG-036]") {
    // A window over ticks 1 .. 100 delivers no edges while n moves on: the first 100 ticks' 6 x 100 = 600 counts are
    // lost, so 6000 - 600 = 5400 arrive, split 2700 A and 2700 B (each later tick still moves 6 counts from an even n).
    for (const FailureMode mode : {FailureMode::stuck, FailureMode::dropped}) {
        IncrementalEncoder encoder("y_encoder", linear_params());
        encoder.inject(FailureInjection{mode, SimTime(1), SimTime(101), 0.0, 0.0});
        const EdgeTotals totals = travel_counts(encoder, 0.1);
        CHECK(totals.a == 2700);
        CHECK(totals.b == 2700);
    }
}

TEST_CASE("An encoder whose resolution kind disagrees with its coordinate is a configuration error naming it",
          "[SIG-011][LIB-025]") {
    // Angular resolution (linear false) on a prismatic coordinate, and a linear one on an angular coordinate.
    EncoderParams angular_on_prismatic = linear_params();
    angular_on_prismatic.linear = false;
    const std::string mixed = config_error(angular_on_prismatic);
    CHECK(mixed.find("y_encoder") != std::string::npos);
    CHECK(mixed.find("prismatic") != std::string::npos);

    EncoderParams linear_on_angular = encoder_params();
    linear_on_angular.linear = true;
    CHECK(config_error(linear_on_angular).find("y_encoder") != std::string::npos);

    CHECK(config_error(linear_params()).empty());
    EncoderParams zero = linear_params();
    zero.resolution = 0.0;
    CHECK(config_error(zero).find("y_encoder") != std::string::npos);
}

namespace {

// An analog SampledSensor reading the body's x as is: the base pipeline (noise, sample rate, delay, faults) seen
// without any sensor-specific conversion.
class ProbeSensor final : public SampledSensor {
public:
    ProbeSensor(std::string instance, SensorFidelity fidelity)
        : SampledSensor(std::move(instance), {"value"}, SensorOutputKind::analog, 1, 1, fidelity) {}

private:
    void measure(const PhysicalState& state, std::span<double> quantities) override {
        quantities[0] = state.bodies[0].position[0];
    }
    void convert(std::span<const double> quantities, std::int64_t /*step*/, std::span<double> frame) override {
        frame[0] = quantities[0];
    }
};

SensorFidelity fidelity(double noise, double delay = 0.0, double sample_rate = 0.0) {
    return SensorFidelity{noise, delay, sample_rate};
}

struct Moments {
    double mean = 0.0;
    double variance = 0.0;
};

// Mean and unbiased variance of the output less the true x over n samples of a still body at x.
Moments output_moments(SampledSensor& sensor, int n, double x) {
    Stepper run(sensor);
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) {
        values.push_back(run.step(state_at(0.0, x))[0] - x);
    }
    Moments m;
    for (const double v : values) {
        m.mean += v;
    }
    m.mean /= static_cast<double>(n);
    for (const double v : values) {
        m.variance += (v - m.mean) * (v - m.mean);
    }
    m.variance /= static_cast<double>(n - 1);
    return m;
}

// The probe's output for x_k = k over n steps.
std::vector<double> ramp_outputs(SampledSensor& sensor, int n) {
    Stepper run(sensor);
    std::vector<double> out;
    out.reserve(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) {
        out.push_back(run.step(state_at(0.0, static_cast<double>(k)))[0]);
    }
    return out;
}

}  // namespace

TEST_CASE("Sensor noise is N(0, noise) in the measured unit, plus the noisy fault's on top", "[SIG-002][SIG-007]") {
    // N = 20000 draws of N(0, sigma). The sample mean is N(0, sigma^2 / N), so |mean| < 4 sigma / sqrt(N) = 0.028 sigma
    // fails with probability 6e-5. (N - 1) s^2 / sigma^2 is chi^2(N - 1), mean N - 1 and SD sqrt(2 (N - 1)), close to
    // normal at this N, so |s^2 / sigma^2 - 1| < 4 sqrt(2 / (N - 1)) = 0.040 fails with probability about 6e-5. The
    // part noise and the fault noise are independent normals, so together they are N(0, noise^2 + magnitude^2).
    constexpr int n = 20000;
    const double mean_bound = 4.0 / std::sqrt(static_cast<double>(n));
    const double variance_bound = 4.0 * std::sqrt(2.0 / static_cast<double>(n - 1));
    constexpr double sigma = 0.01;
    ProbeSensor part("probe", fidelity(sigma));
    const Moments m = output_moments(part, n, 1.0);
    INFO("mean " << m.mean << " (bound " << mean_bound * sigma << "), variance " << m.variance << " (target "
                 << sigma * sigma << ", relative bound " << variance_bound << ")");
    CHECK(std::fabs(m.mean) < mean_bound * sigma);
    CHECK(std::fabs((m.variance / (sigma * sigma)) - 1.0) < variance_bound);

    constexpr double magnitude = 0.02;
    ProbeSensor both("probe", fidelity(sigma));
    both.inject(FailureInjection{FailureMode::noisy, SimTime(0), SimTime(n), magnitude, 0.0});
    const double total = (sigma * sigma) + (magnitude * magnitude);
    const Moments b = output_moments(both, n, 1.0);
    INFO("combined variance " << b.variance << " (target " << total << ")");
    CHECK(std::fabs(b.mean) < mean_bound * std::sqrt(total));
    CHECK(std::fabs((b.variance / total) - 1.0) < variance_bound);

    ProbeSensor clean("probe", fidelity(0.0));
    const Moments c = output_moments(clean, 100, 0.375);
    CHECK(c.mean == 0.0);
    CHECK(c.variance == 0.0);
    CHECK_THROWS_AS(ProbeSensor("probe", fidelity(-1.0)), SignalConfigError);
    CHECK_THROWS_AS(ProbeSensor("probe", fidelity(std::nan(""))), SignalConfigError);
}

TEST_CASE("Sensor delay delivers the sample taken ceil(delay / period) periods earlier", "[SIG-003]") {
    // x_k = k. Delay 3.5 ms at 1 kHz rounds up to D = 4: the output at k is x_(k - 4), the first sample before then.
    ProbeSensor late("probe", fidelity(0.0, 0.0035));
    const std::vector<double> out = ramp_outputs(late, 12);
    CHECK(late.delay_periods() == 4);
    for (int k = 0; k < 12; ++k) {
        CHECK(out[static_cast<std::size_t>(k)] == static_cast<double>(std::max(k - 4, 0)));
    }
    ProbeSensor exact("probe", fidelity(0.0, 0.003));
    const std::vector<double> three = ramp_outputs(exact, 6);
    CHECK(exact.delay_periods() == 3);
    CHECK(three[5] == 2.0);
    ProbeSensor none("probe", fidelity(0.0, 0.0));
    const std::vector<double> now = ramp_outputs(none, 3);
    CHECK(now[2] == 2.0);

    // 1e6 periods is the deepest ring; 1000.001 s rounds up to 1 000 001 periods.
    ProbeSensor deepest("probe", fidelity(0.0, 1000.0));
    CHECK_NOTHROW(deepest.start(khz_clock()));
    CHECK(deepest.delay_periods() == 1000000);
    ProbeSensor too_deep("probe", fidelity(0.0, 1000.001));
    CHECK_THROWS_AS(too_deep.start(khz_clock()), SignalConfigError);
    CHECK_THROWS_AS(ProbeSensor("probe", fidelity(0.0, -0.001)), SignalConfigError);
}

TEST_CASE("Sensor sample rate holds the output between samples on the layer clock", "[SIG-004]") {
    // 250 Hz on a 1 kHz layer: every 4th step samples, so the output at k is x at 4 floor(k / 4).
    ProbeSensor slow("probe", fidelity(0.0, 0.0, 250.0));
    const std::vector<double> out = ramp_outputs(slow, 12);
    CHECK(slow.sample_period() == 4);
    for (int k = 0; k < 12; ++k) {
        const int held = 4 * (k / 4);
        CHECK(out[static_cast<std::size_t>(k)] == static_cast<double>(held));
    }
    // 300 Hz: floor(3.33) = 3 periods. 1000 Hz equals the layer rate: every step.
    ProbeSensor odd("probe", fidelity(0.0, 0.0, 300.0));
    odd.start(khz_clock());
    CHECK(odd.sample_period() == 3);
    ProbeSensor equal("probe", fidelity(0.0, 0.0, 1000.0));
    const std::vector<double> every = ramp_outputs(equal, 3);
    CHECK(every[2] == 2.0);
    CHECK(equal.sample_period() == 1);

    // Rate and delay together: delay 2 ms delivers the frame held at k - 2, sampled at 4 floor((k - 2) / 4).
    ProbeSensor both("probe", fidelity(0.0, 0.002, 250.0));
    const std::vector<double> held = ramp_outputs(both, 12);
    for (int k = 0; k < 12; ++k) {
        double expected = 0.0;
        if (k >= 2) {
            const int quantized = 4 * ((k - 2) / 4);
            expected = static_cast<double>(quantized);
        }
        CHECK(held[static_cast<std::size_t>(k)] == expected);
    }

    // One draw per sample: 8 steps at period 4 take 2 samples, so the generator has made exactly 2 normal draws.
    ProbeSensor noisy("probe", fidelity(0.01, 0.0, 250.0));
    Stepper run(noisy);
    for (int k = 0; k < 8; ++k) {
        run.step(state_at(0.0, 0.0));
    }
    SignalRng reference(khz_clock().seed);
    static_cast<void>(reference.gaussian());
    static_cast<void>(reference.gaussian());
    CHECK(noisy.generator().state() == reference.state());

    ProbeSensor fast("probe", fidelity(0.0, 0.0, 1500.0));
    try {
        fast.start(khz_clock());
        FAIL("a sample rate above the layer rate was accepted");
    } catch (const SignalConfigError& error) {
        const std::string text = error.what();
        CHECK(text.find("1500") != std::string::npos);
        CHECK(text.find("1000") != std::string::npos);
    }
    CHECK_THROWS_AS(ProbeSensor("probe", fidelity(0.0, 0.0, -1.0)), SignalConfigError);
}

TEST_CASE("Sensor faults act for start <= step < end: stuck and dropped hold, dropped draws nothing",
          "[SIG-006][SIG-007][SIG-008][SIG-036]") {
    ProbeSensor stuck("probe", fidelity(0.0));
    stuck.inject(FailureInjection{FailureMode::stuck, SimTime(3), SimTime(6), 0.0, 0.0});
    const std::vector<double> held = ramp_outputs(stuck, 8);
    CHECK(held == std::vector<double>{0.0, 1.0, 2.0, 2.0, 2.0, 2.0, 6.0, 7.0});

    // Stuck holds what was delivered, the delayed value: with D = 2 the output at 4 is x_2, held through 6.
    ProbeSensor late("probe", fidelity(0.0, 0.002));
    late.inject(FailureInjection{FailureMode::stuck, SimTime(5), SimTime(7), 0.0, 0.0});
    CHECK(ramp_outputs(late, 8) == std::vector<double>{0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 5.0});

    // Stuck from the first step holds the output's initial 0.
    ProbeSensor first("probe", fidelity(0.0));
    first.inject(FailureInjection{FailureMode::stuck, SimTime(0), SimTime(2), 0.0, 0.0});
    CHECK(ramp_outputs(first, 3) == std::vector<double>{0.0, 0.0, 2.0});

    // Dropped with noise: no sample in the window, so no draw; the generator stands still from step 3 to step 5.
    ProbeSensor dropped("probe", fidelity(0.01));
    dropped.inject(FailureInjection{FailureMode::dropped, SimTime(3), SimTime(6), 0.0, 0.0});
    Stepper run(dropped);
    std::vector<double> out;
    std::vector<std::uint64_t> position;
    for (int k = 0; k < 8; ++k) {
        out.push_back(run.step(state_at(0.0, static_cast<double>(k)))[0]);
        position.push_back(dropped.generator().state());
    }
    CHECK(position[3] == position[2]);
    CHECK(position[5] == position[2]);
    CHECK(position[6] != position[5]);
    CHECK(out[3] == out[2]);
    CHECK(out[5] == out[2]);
    CHECK(std::fabs(out[6] - 6.0) < 0.1);

    // Dropped from the first step: nothing was ever sampled, the output stays at 0 until the window ends.
    ProbeSensor blind("probe", fidelity(0.0));
    blind.inject(FailureInjection{FailureMode::dropped, SimTime(0), SimTime(2), 0.0, 0.0});
    CHECK(ramp_outputs(blind, 3) == std::vector<double>{0.0, 0.0, 2.0});

    // Noisy only inside [3, 6): no draw before step 3, exact values outside the window.
    ProbeSensor noisy("probe", fidelity(0.0));
    noisy.inject(FailureInjection{FailureMode::noisy, SimTime(3), SimTime(6), 0.5, 0.0});
    Stepper noisy_run(noisy);
    const std::uint64_t untouched = SignalRng(khz_clock().seed).state();
    for (int k = 0; k < 8; ++k) {
        const double value = noisy_run.step(state_at(0.0, static_cast<double>(k)))[0];
        if (k < 3) {
            CHECK(noisy.generator().state() == untouched);
        }
        if (k >= 3 && k < 6) {
            CHECK(value != static_cast<double>(k));
        } else {
            CHECK(value == static_cast<double>(k));
        }
    }

    ProbeSensor analog("probe", fidelity(0.0));
    try {
        analog.inject(FailureInjection{FailureMode::forced_level, SimTime(0), SimTime(1), 0.0, 1.0});
        FAIL("a forced level on an analog sensor was accepted");
    } catch (const SignalConfigError& error) {
        CHECK(std::string(error.what()).find("forced_level") != std::string::npos);
    }
}

TEST_CASE("Two fault windows overlapping on one sensor are refused at start; adjacent ones are not", "[SIG-006]") {
    ProbeSensor overlapping("probe", fidelity(0.0));
    overlapping.inject(FailureInjection{FailureMode::stuck, SimTime(1), SimTime(4), 0.0, 0.0});
    overlapping.inject(FailureInjection{FailureMode::dropped, SimTime(3), SimTime(5), 0.0, 0.0});
    CHECK_THROWS_AS(overlapping.start(khz_clock()), SignalConfigError);

    ProbeSensor adjacent("probe", fidelity(0.0));
    adjacent.inject(FailureInjection{FailureMode::stuck, SimTime(1), SimTime(3), 0.0, 0.0});
    adjacent.inject(FailureInjection{FailureMode::dropped, SimTime(3), SimTime(5), 0.0, 0.0});
    CHECK(ramp_outputs(adjacent, 6) == std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 5.0});
}

TEST_CASE("Debounce: N - 1 periods suppressed, N passes, a short gap merges, 0 passes, a long one holds",
          "[SIG-010]") {
    // debounce 5 ms at 1 kHz: N = 5. levels[k] is the contact state sampled at step k (1 = past the switch).
    const auto run_levels = [](double debounce, const std::vector<int>& levels) {
        LimitSwitchParams params = switch_params();
        params.debounce_time = debounce;
        LimitSwitch limit("x_max_switch", params);
        Stepper run(limit);
        std::vector<int> out;
        for (const int level : levels) {
            double x = 0.25;
            if (level == 1) {
                x = 0.75;
            }
            out.push_back(static_cast<int>(run.step(state_at(0.0, x))[0]));
        }
        return out;
    };
    // A 4-period pulse (sampled on 4 steps) never appears.
    CHECK(run_levels(0.005, {0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0}) == std::vector<int>(12, 0));
    // A 5-period pulse appears at step 6, 5 periods after step 1, and its end appears 5 periods after step 6.
    CHECK(run_levels(0.005, {0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0}) ==
          std::vector<int>{0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0});
    // Two 5-period pulses 4 periods apart: the gap is a change shorter than N, so it is suppressed and the output
    // stays 1 from step 6 to the end of the second pulse (step 15) plus N.
    std::vector<int> merged_in{0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1};
    merged_in.resize(22, 0);
    std::vector<int> merged_out(6, 0);
    merged_out.resize(20, 1);
    merged_out.resize(22, 0);
    CHECK(run_levels(0.005, merged_in) == merged_out);
    // Bounces shorter than N separated by gaps shorter than N never accumulate: each gap restarts the count.
    CHECK(run_levels(0.005, {0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0}) == std::vector<int>(15, 0));
    // Debounce 0 passes a single-sample pulse through.
    CHECK(run_levels(0.0, {0, 1, 0, 1, 1, 0}) == std::vector<int>{0, 1, 0, 1, 1, 0});
    // Debounce 10 s, longer than the 100-step run: the initial level holds.
    std::vector<int> long_in(1, 0);
    long_in.resize(100, 1);
    CHECK(run_levels(10.0, long_in) == std::vector<int>(100, 0));
}

TEST_CASE("A noisy limit switch chatters at its position; noise 0 draws nothing", "[SIG-002][SIG-005]") {
    // The body sits exactly on the switch position, no hysteresis: each sample is actuated when the draw is >= 0,
    // probability 1/2, so 100 samples all one level has probability 2^-99.
    LimitSwitchParams params = switch_params();
    params.fidelity.noise = 0.001;
    LimitSwitch noisy("s", params);
    Stepper run(noisy);
    int actuated = 0;
    for (int k = 0; k < 100; ++k) {
        actuated += static_cast<int>(run.step(state_at(0.0, switch_position))[0]);
    }
    CHECK(actuated > 0);
    CHECK(actuated < 100);

    LimitSwitch quiet("s", switch_params());
    Stepper quiet_run(quiet);
    for (int k = 0; k < 100; ++k) {
        CHECK(quiet_run.step(state_at(0.0, switch_position))[0] == 1.0);
    }
    CHECK(quiet.generator().state() == SignalRng(khz_clock().seed).state());
}

TEST_CASE("Encoder delay shifts its edges, its sample rate gathers them at sample steps", "[SIG-003][SIG-004]") {
    // One turn in 100 ticks: 24 counts per tick, from half a count in so no sample sits on a cell edge.
    const auto angle = [](int k) { return (0.5 * resolution) + (2.0 * std::numbers::pi * k / 100.0); };
    const auto edges = [&angle](EncoderParams params, int n) {
        IncrementalEncoder encoder("e", params);
        Stepper run(encoder);
        std::vector<std::int64_t> out;
        for (int k = 0; k < n; ++k) {
            const std::span<const double> v = run.step(state_at(angle(k), 0.0));
            out.push_back(std::llround(v[0] + v[1]));
        }
        return out;
    };
    const std::vector<std::int64_t> prompt = edges(encoder_params(), 20);
    EncoderParams late_params = encoder_params();
    late_params.fidelity.delay = 0.002;
    const std::vector<std::int64_t> late = edges(late_params, 20);
    CHECK(late[0] == 0);
    CHECK(late[1] == 0);
    for (std::size_t k = 2; k < 20; ++k) {
        CHECK(late[k] == prompt[k - 2]);
    }
    EncoderParams slow_params = encoder_params();
    slow_params.fidelity.sample_rate = 250.0;
    const std::vector<std::int64_t> slow = edges(slow_params, 20);
    for (std::size_t k = 0; k < 20; ++k) {
        if (k % 4 == 0 && k > 0) {
            CHECK(slow[k] == prompt[k] + prompt[k - 1] + prompt[k - 2] + prompt[k - 3]);
        } else {
            CHECK(slow[k] == 0);
        }
    }

    // Noise 0 on the encoder makes no draw.
    IncrementalEncoder quiet("e", encoder_params());
    Stepper quiet_run(quiet);
    for (int k = 0; k < 20; ++k) {
        quiet_run.step(state_at(angle(k), 0.0));
    }
    CHECK(quiet.generator().state() == SignalRng(khz_clock().seed).state());
}

TEST_CASE("Child generators: names give different streams, one name and seed reproduces, noise 0 draws nothing",
          "[SIG-005]") {
    // Each instance draws from SignalRng(derive_child_seed(run seed, instance)), as the layer hands it at start.
    const auto stream = [](const std::string& name, std::uint64_t run_seed, double noise) {
        ProbeSensor probe(name, fidelity(noise));
        const signal_test::ComponentClock clock{mechsim::kernel::Rate(1000), mechsim::kernel::Duration(1),
                                                derive_child_seed(run_seed, name)};
        Stepper run(probe, clock);
        std::vector<double> out;
        out.reserve(51);
        for (int k = 0; k < 50; ++k) {
            out.push_back(run.step(state_at(0.0, 0.0))[0]);
        }
        out.push_back(static_cast<double>(probe.generator().state() == SignalRng(clock.seed).state()));
        return out;
    };
    const std::vector<double> a = stream("x_probe", 11, 0.01);
    CHECK(a == stream("x_probe", 11, 0.01));
    CHECK(a != stream("y_probe", 11, 0.01));
    CHECK(a != stream("x_probe", 12, 0.01));
    CHECK(a.back() == 0.0);
    const std::vector<double> silent = stream("x_probe", 11, 0.0);
    CHECK(silent.back() == 1.0);
}
