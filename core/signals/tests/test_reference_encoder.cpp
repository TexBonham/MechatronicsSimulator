#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numbers>
#include <span>

#include "mechsim/signals/sensors.hpp"
#include "signal_test_support.hpp"

using mechsim::signals::CoordinateKind;
using mechsim::signals::EncoderParams;
using mechsim::signals::IncrementalEncoder;
using mechsim::signals::JointCoordinate;
using signal_test::state_at;
using signal_test::Stepper;

namespace {

// 0.15 deg per count: 2400 quadrature counts per revolution (catalog part encoder_600ppr_generic).
constexpr double resolution = 2.0 * std::numbers::pi / 2400.0;

struct Totals {
    std::int64_t a = 0;
    std::int64_t b = 0;
};

Totals rotate(bool reversed, double start, double turn, int ticks) {
    EncoderParams params;
    params.coordinate = JointCoordinate{CoordinateKind::angular, 0, {}, {0.0, 0.0, 1.0}};
    params.resolution = resolution;
    params.reversed = reversed;
    IncrementalEncoder encoder("x_encoder", params);
    Stepper run(encoder);
    run.step(state_at(start, 0.0));
    Totals totals;
    for (int k = 1; k <= ticks; ++k) {
        const std::span<const double> out = run.step(state_at(start + (turn * k / ticks), 0.0));
        totals.a += std::llround(out[0]);
        totals.b += std::llround(out[1]);
    }
    return totals;
}

}  // namespace

TEST_CASE("Encoder counts for one turn equal the floor(angle / resolution) difference exactly", "[SIG-011][SIG-001]") {
    // Closed form: n = floor(angle / r). Starting half a count past zero keeps every sample off a count boundary:
    // n0 = floor(0.5) = 0, n1 = floor(2400.5) = 2400, so one turn gains exactly 2400 counts (SIG-011 proposed
    // tolerance: exact). Transitions k -> k + 1 for k = 0 .. 2399: the 1200 even k are A edges, the 1200 odd k B
    // edges. The step per tick, 2 pi / 1000 rad, stays below the half turn the unwrap allows.
    const Totals forward = rotate(false, 0.5 * resolution, 2.0 * std::numbers::pi, 1000);
    std::cout << "SIG-011 one turn: closed form 2400 counts (A 1200, B 1200); measured " << forward.a + forward.b
              << " (A " << forward.a << ", B " << forward.b << ")\n";
    CHECK(forward.a + forward.b == 2400);
    CHECK(forward.a == 1200);
    CHECK(forward.b == 1200);

    // Backwards: floor(0.5 - 2400) - 0 = -2400. Reversed counting: n runs floor(-0.5) = -1 to floor(-2400.5) = -2401.
    const Totals reverse = rotate(false, 0.5 * resolution, -2.0 * std::numbers::pi, 1000);
    const Totals mirrored = rotate(true, 0.5 * resolution, 2.0 * std::numbers::pi, 1000);
    std::cout << "SIG-011 reverse: closed form -2400; measured " << reverse.a + reverse.b << "; reversed encoder "
              << mirrored.a + mirrored.b << "\n";
    CHECK(reverse.a + reverse.b == -2400);
    CHECK(mirrored.a + mirrored.b == -2400);

    // Three turns in 3000 ticks: floor(7200.5) - 0 = 7200; angles past pi go through the unwrap.
    const Totals turns = rotate(false, 0.5 * resolution, 6.0 * std::numbers::pi, 3000);
    std::cout << "SIG-011 three turns: closed form 7200; measured " << turns.a + turns.b << "\n";
    CHECK(turns.a + turns.b == 7200);
}
