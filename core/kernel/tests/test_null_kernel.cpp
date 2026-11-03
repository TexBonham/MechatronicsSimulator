#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mechsim/kernel/kernel.hpp"
#include "mechsim/kernel/recorder_sink.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/time.hpp"
#include "null_parties.hpp"

using mechsim::kernel::Duration;
using mechsim::kernel::EndCondition;
using mechsim::kernel::KernelConfigError;
using mechsim::kernel::NullKernel;
using mechsim::kernel::NullRecorderSink;
using mechsim::kernel::PartyFault;
using mechsim::kernel::Rate;
using mechsim::kernel::RunConfig;
using mechsim::kernel::RunOutcome;
using mechsim::kernel::Seconds;
using mechsim::kernel::SimTime;
using mechsim::kernel::test::LoggingParty;
using mechsim::kernel::test::StepEvent;

namespace {

struct Logs {
    std::vector<StepEvent> steps;
    std::vector<std::string> faults;
};

std::unique_ptr<LoggingParty> party(const std::string& name, const Rate& rate, Logs& logs) {
    return std::make_unique<LoggingParty>(name, rate, logs.steps, logs.faults);
}

std::unique_ptr<LoggingParty> party(const std::string& name, std::int64_t hz, Logs& logs) {
    return party(name, Rate(hz), logs);
}

std::size_t count_steps(const Logs& logs, const std::string& name) {
    std::size_t n = 0;
    for (const StepEvent& event : logs.steps) {
        if (event.party == name) {
            ++n;
        }
    }
    return n;
}

EndCondition until(std::int64_t numerator, std::int64_t denominator) {
    return EndCondition{Seconds(numerator, denominator), {}};
}

}  // namespace

TEST_CASE("Run configuration defaults take the physics and control rates from data", "[KER-004][KER-005]") {
    const RunConfig config;
    CHECK(config.physics_rate == Rate(1'000));
    CHECK(config.control_rate == Rate(1'000));
    CHECK(config.party_rates.empty());
    CHECK(config.seed == 0);
    CHECK(config.solver.iteration_cap == 50);
    CHECK(config.solver.projection_iterations == 3);
    CHECK(config.solver.velocity_tolerance == 1e-9);
    CHECK(config.drift.translational_m == 1e-6);
    CHECK(config.drift.rotational_rad == 1e-6);
}

TEST_CASE("Five parties are stepped every tick in registration order", "[KER-013][KER-018][KER-009]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    const std::vector<std::string> names{"physics", "integrity", "sensors", "controller", "drivers"};
    for (const std::string& name : names) {
        kernel.register_party(party(name, 1'000, logs));
    }
    NullRecorderSink sink;
    const auto report = kernel.run(until(3, 1'000), sink);
    REQUIRE(report.outcome == RunOutcome::end_condition_met);
    REQUIRE(logs.steps.size() == 15);
    for (std::size_t tick = 0; tick < 3; ++tick) {
        for (std::size_t i = 0; i < names.size(); ++i) {
            const StepEvent& event = logs.steps[(tick * names.size()) + i];
            CHECK(event.party == names[i]);
            CHECK(event.t == SimTime(static_cast<std::int64_t>(tick)));
            CHECK(event.dt == Duration(1));
        }
    }
}

TEST_CASE("A run starts at t = 0 and advances only through run()", "[KER-001][KER-002]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    kernel.register_party(party("physics", 1'000, logs));
    CHECK(kernel.now() == SimTime(0));
    NullRecorderSink sink;
    const auto report = kernel.run(until(2, 1'000), sink);
    REQUIRE_FALSE(logs.steps.empty());
    CHECK(logs.steps.front().t == SimTime(0));
    CHECK(report.end_time == SimTime(2));
    CHECK(kernel.now() == report.end_time);
}

// 200 Hz and 1 kHz: base rate lcm(1000, 200) = 1000 Hz, so the slow party's period is 5 ticks. Over 1 s:
// 1000 fast steps, 1000 / 5 = 200 slow steps at t = 5k.
TEST_CASE("A 1 kHz and a 200 Hz party step 1000 and 200 times in 1 s", "[KER-003][KER-006]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    kernel.register_party(party("physics", 1'000, logs));
    kernel.register_party(party("controller", 200, logs));
    NullRecorderSink sink;
    const auto report = kernel.run(until(1, 1), sink);
    REQUIRE(report.outcome == RunOutcome::end_condition_met);
    CHECK(report.base_rate == Rate(1'000));
    CHECK(count_steps(logs, "physics") == 1'000);
    CHECK(count_steps(logs, "controller") == 200);
    CHECK(report.ticks == 1'000);
    CHECK(sink.rows_written() == 1'000);
    for (const StepEvent& event : logs.steps) {
        if (event.party == "controller") {
            CHECK(event.dt == Duration(5));
            CHECK(event.t.count() % 5 == 0);
        }
    }
}

// 3 kHz and 1 kHz: base rate lcm(3000, 1000) = 3000 Hz; physics period 3 ticks. Over 1 s: 3000 base ticks,
// 3000 encoder steps, 1000 physics steps and 1000 rows (one per physics step). 3 kHz has no whole-nanosecond
// period, which the tick base makes irrelevant.
TEST_CASE("A 3 kHz party runs beside 1 kHz physics on a 3 kHz base", "[KER-003]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    kernel.register_party(party("physics", 1'000, logs));
    kernel.register_party(party("encoder", 3'000, logs));
    NullRecorderSink sink;
    const auto report = kernel.run(until(1, 1), sink);
    CHECK(report.base_rate == Rate(3'000));
    CHECK(kernel.config().base_rate == Rate(3'000));
    CHECK(count_steps(logs, "physics") == 1'000);
    CHECK(count_steps(logs, "encoder") == 3'000);
    CHECK(report.ticks == 3'000);
    CHECK(sink.rows_written() == 1'000);
    CHECK(logs.steps.front().dt == Duration(3));
}

// 0.5 Hz beside 1 kHz: base lcm(1000/1, 1/2) = lcm(1000, 1) / gcd(1, 2) = 1000 Hz; the slow period is
// 1000 / 0.5 = 2000 ticks, so over 4 s it steps at t = 0 and t = 2000.
TEST_CASE("A sub-hertz party steps at its exact period", "[KER-003]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    kernel.register_party(party("physics", 1'000, logs));
    auto slow = party("thermal", Rate(1, 2), logs);
    const LoggingParty& slow_view = *slow;
    kernel.register_party(std::move(slow));
    NullRecorderSink sink;
    const auto report = kernel.run(until(4, 1), sink);
    CHECK(report.base_rate == Rate(1'000));
    CHECK(slow_view.base_rate() == Rate(1'000));
    CHECK(count_steps(logs, "thermal") == 2);
    std::vector<SimTime> thermal_times;
    for (const StepEvent& event : logs.steps) {
        if (event.party == "thermal") {
            thermal_times.push_back(event.t);
            CHECK(event.dt == Duration(2'000));
        }
    }
    const std::vector<SimTime> expected{SimTime(0), SimTime(2'000)};
    CHECK(thermal_times == expected);
}

TEST_CASE("A rate override in the run configuration replaces the nominal rate", "[KER-005][KER-006]") {
    Logs logs;
    RunConfig config;
    config.party_rates.emplace("controller", Rate(500));
    NullKernel kernel(config);
    kernel.register_party(party("physics", 1'000, logs));
    kernel.register_party(party("controller", 1'000, logs));
    NullRecorderSink sink;
    static_cast<void>(kernel.run(until(10, 1'000), sink));
    CHECK(count_steps(logs, "controller") == 5);
}

TEST_CASE("A party that throws ends the run with a report naming it", "[KER-015]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    kernel.register_party(party("physics", 1'000, logs));
    auto failing = party("integrity", 1'000, logs);
    const SimTime fail_time(4);
    failing->fail_at(fail_time);
    kernel.register_party(std::move(failing));
    kernel.register_party(party("controller", 1'000, logs));
    NullRecorderSink sink;
    const auto report = kernel.run(until(1, 1), sink);
    REQUIRE(report.outcome == RunOutcome::run_error);
    REQUIRE(report.fault.has_value());
    const PartyFault fault = report.fault.value_or(PartyFault("none", SimTime(), "none"));
    CHECK(fault.party() == "integrity");
    CHECK(fault.time() == fail_time);
    CHECK(fault.cause() == "injected failure");
    CHECK(report.end_time == fail_time);
    CHECK(report.ticks == 4);
    CHECK(sink.rows_written() == 4);
    CHECK(logs.steps.back().party == "physics");
    CHECK(logs.steps.back().t == fail_time);
    const std::vector<std::string> expected{"physics:integrity", "integrity:integrity", "controller:integrity"};
    CHECK(logs.faults == expected);
}

TEST_CASE("A fault at the very first tick is reported at t = 0", "[KER-015]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    auto failing = party("physics", 1'000, logs);
    failing->fail_at(SimTime(0));
    kernel.register_party(std::move(failing));
    NullRecorderSink sink;
    const auto report = kernel.run(until(1, 1), sink);
    REQUIRE(report.fault.has_value());
    CHECK(report.fault.value_or(PartyFault("none", SimTime(-1), "none")).time() == SimTime(0));
    CHECK(report.ticks == 0);
    CHECK(sink.rows_written() == 0);
}

TEST_CASE("A stop condition ends the run early and the report says so", "[KER-017]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    kernel.register_party(party("physics", 1'000, logs));
    NullRecorderSink sink;
    const SimTime stop_at(7);
    const auto report = kernel.run(EndCondition{Seconds(1), [stop_at](SimTime t) { return t >= stop_at; }}, sink);
    CHECK(report.outcome == RunOutcome::end_condition_met);
    CHECK(report.end_time == stop_at);
    CHECK(report.ticks == 7);
    CHECK_FALSE(report.fault.has_value());
}

TEST_CASE("An end time of zero runs no tick", "[KER-017]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    kernel.register_party(party("physics", 1'000, logs));
    NullRecorderSink sink;
    const auto report = kernel.run(until(0, 1), sink);
    CHECK(report.ticks == 0);
    CHECK(logs.steps.empty());
    CHECK(sink.rows_written() == 0);
}

// Base 1000 Hz: 1/3 s is 333.33 ticks, off the base, so the run is refused before t = 0 (invariant 7).
TEST_CASE("An end time that is not a whole number of base ticks is rejected at run start", "[KER-007][KER-017]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    kernel.register_party(party("physics", 1'000, logs));
    NullRecorderSink sink;
    CHECK_THROWS_AS(kernel.run(until(1, 3), sink), KernelConfigError);
    CHECK(logs.steps.empty());
    CHECK(sink.rows_written() == 0);
    CHECK(kernel.now() == SimTime(0));
}

TEST_CASE("The kernel writes one row per physics step to the recorder sink, stamped with its end time",
          "[KER-022][KER-009]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    kernel.register_party(party("physics", 1'000, logs));
    NullRecorderSink sink;
    std::vector<SimTime> times;
    const auto id = sink.subscribe([&times](const mechsim::kernel::TickRecord& row) { times.push_back(row.time); });
    static_cast<void>(id);
    static_cast<void>(kernel.run(until(3, 1'000), sink));
    const std::vector<SimTime> expected{SimTime(1), SimTime(2), SimTime(3)};
    CHECK(times == expected);
}

TEST_CASE("Every party registers through one interface carrying rate, step and coupling variables", "[KER-018]") {
    Logs logs;
    auto physics = party("physics", 1'000, logs);
    physics->set_coupling({"physics.body_poses", "physics.member_loads"}, {"physics.joint_efforts"});
    const mechsim::kernel::Party& as_party = *physics;
    CHECK(as_party.name() == "physics");
    CHECK(as_party.nominal_rate() == Rate(1'000));
    REQUIRE(as_party.published().size() == 2);
    CHECK(as_party.published()[1] == "physics.member_loads");
    REQUIRE(as_party.consumed().size() == 1);
    CHECK(as_party.consumed()[0] == "physics.joint_efforts");
    NullKernel kernel(RunConfig{});
    kernel.register_party(std::move(physics));
    NullRecorderSink sink;
    static_cast<void>(kernel.run(until(1, 1'000), sink));
    REQUIRE(logs.steps.size() == 1);
    CHECK(logs.steps[0].party == "physics");
}

TEST_CASE("Registration rejects bad parties before t = 0", "[KER-018]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    kernel.register_party(party("physics", 1'000, logs));
    CHECK_THROWS_AS(kernel.register_party(party("physics", 1'000, logs)), KernelConfigError);
    CHECK_THROWS_AS(kernel.register_party(party("", 1'000, logs)), KernelConfigError);
    CHECK_THROWS_AS(kernel.register_party(nullptr), KernelConfigError);
    NullRecorderSink sink;
    static_cast<void>(kernel.run(until(1, 1'000), sink));
    CHECK_THROWS_AS(kernel.register_party(party("late", 1'000, logs)), KernelConfigError);
    CHECK_THROWS_AS(kernel.run(until(1, 1'000), sink), KernelConfigError);
}

// lcm(1000, 1e9 + 7, 1e9 + 9) is about 1e21 > int64 max 9.2e18: refused naming the party that overflows it, before
// t = 0 (KER-007, invariant 7).
TEST_CASE("A rate combination whose base rate overflows is rejected naming the party", "[KER-007]") {
    Logs logs;
    NullKernel kernel(RunConfig{});
    kernel.register_party(party("a", 1'000'000'007, logs));
    kernel.register_party(party("b", 1'000'000'009, logs));
    NullRecorderSink sink;
    try {
        static_cast<void>(kernel.run(until(1, 1), sink));
        FAIL("the base rate overflows and must be rejected");
    } catch (const KernelConfigError& error) {
        CHECK(std::string(error.what()).find("'b'") != std::string::npos);
    }
    CHECK(logs.steps.empty());
}
