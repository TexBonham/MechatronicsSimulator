#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <span>

#include "mechsim/signals/safety.hpp"
#include "signal_test_support.hpp"

using mechsim::signals::EStopRelay;
using mechsim::signals::EStopRelayParams;
using mechsim::signals::PowerSupply;
using mechsim::signals::SignalConfigError;
using mechsim::signals::Watchdog;
using mechsim::signals::WatchdogParams;
using signal_test::state_at;
using signal_test::Stepper;

namespace {

// Contacts-closed level after one relay sample with loop level `loop` and reset level `reset`.
double relay_step(Stepper& run, double loop, double reset = 0.0) {
    return run.step(state_at(0.0, 0.0), {loop, reset})[1];
}

double watchdog_step(Stepper& run, double heartbeat) {
    return run.step(state_at(0.0, 0.0), {heartbeat})[1];
}

}  // namespace

TEST_CASE("A power supply carries its output voltage", "[SIG-015]") {
    PowerSupply supply("motor_supply", 24.0);
    Stepper run(supply);
    CHECK(run.step(state_at(0.0, 0.0))[0] == 24.0);
    CHECK_THROWS_AS(PowerSupply("s", -1.0), SignalConfigError);
}

TEST_CASE("The E-stop relay opens its contacts within the cutoff delay of a press", "[SIG-017]") {
    // cutoff 20 ms at 1 kHz: D = floor(0.02 / 0.001) = 20 periods. A press first seen at sample 100 opens the
    // contacts at sample 120: closed through 119, which is 19 ms after the press, open at 20 ms, within the delay.
    EStopRelay relay("estop", EStopRelayParams{0.020, true});
    Stepper run(relay);
    for (int k = 0; k < 100; ++k) {
        CHECK(relay_step(run, 1.0) == 1.0);
    }
    for (int k = 100; k < 120; ++k) {
        CHECK(relay_step(run, 0.0) == 1.0);
    }
    CHECK(relay_step(run, 0.0) == 0.0);
    // Manual reset: closing the loop is not enough; a reset level with the loop closed recloses the contacts.
    CHECK(relay_step(run, 1.0) == 0.0);
    CHECK(relay_step(run, 1.0, 1.0) == 1.0);

    // A cutoff of 2.5 ms rounds down to D = 2 periods (2 ms), so the opening is still within the delay.
    EStopRelay quick("estop", EStopRelayParams{0.0025, false});
    Stepper quick_run(quick);
    CHECK(relay_step(quick_run, 0.0) == 1.0);
    CHECK(relay_step(quick_run, 0.0) == 1.0);
    CHECK(relay_step(quick_run, 0.0) == 0.0);
    // No manual reset: the contacts close again with the loop.
    CHECK(relay_step(quick_run, 1.0) == 1.0);

    // A press shorter than the delay still completes: the relay coil has already dropped out.
    EStopRelay brief("estop", EStopRelayParams{0.003, false});
    Stepper brief_run(brief);
    CHECK(relay_step(brief_run, 0.0) == 1.0);
    CHECK(relay_step(brief_run, 1.0) == 1.0);
    CHECK(relay_step(brief_run, 1.0) == 1.0);
    CHECK(relay_step(brief_run, 1.0) == 0.0);
    CHECK(relay_step(brief_run, 1.0) == 1.0);

    CHECK_THROWS_AS(EStopRelay("estop", EStopRelayParams{-0.1, true}), SignalConfigError);
}

TEST_CASE("The watchdog trips when the heartbeat stops for longer than its timeout", "[SIG-018]") {
    // timeout 50 ms, cutoff 10 ms at 1 kHz: T = 50, D = 10. With the last edge at sample e, it trips at the first
    // sample j with j - e > 50, j = e + 51, and opens the contacts D samples later, at e + 61.
    const WatchdogParams params{0.050, 0.010, false};
    Watchdog watchdog("watchdog", params);
    Stepper run(watchdog);
    double level = 0.0;
    for (int k = 0; k < 200; ++k) {
        level = 1.0 - level;
        CHECK(watchdog_step(run, level) == 1.0);
    }
    // Sample 199 carried the last edge; the heartbeat now holds.
    const std::int64_t e = 199;
    for (std::int64_t j = e + 1; j < e + 61; ++j) {
        CHECK(watchdog_step(run, level) == 1.0);
    }
    const std::span<const double> open = run.step(state_at(0.0, 0.0), {level});
    CHECK(open[0] == 1.0);
    CHECK(open[1] == 0.0);
    // No manual reset: the next heartbeat edge re-arms it.
    CHECK(watchdog_step(run, 1.0 - level) == 1.0);

    // The first sample counts as an edge, so a controller that never beats trips at sample 51.
    Watchdog silent("watchdog", WatchdogParams{0.050, 0.0, true});
    Stepper silent_run(silent);
    for (int j = 0; j <= 50; ++j) {
        CHECK(silent_run.step(state_at(0.0, 0.0), {0.0})[0] == 0.0);
    }
    CHECK(silent_run.step(state_at(0.0, 0.0), {0.0})[0] == 1.0);
    // Manual reset: a late edge does not re-arm it.
    CHECK(watchdog_step(silent_run, 1.0) == 0.0);

    CHECK_THROWS_AS(Watchdog("w", WatchdogParams{-1.0, 0.0, false}), SignalConfigError);
}
