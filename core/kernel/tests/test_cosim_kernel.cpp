#include <bit>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "mechsim/kernel/cosim_kernel.hpp"
#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/kernel.hpp"
#include "mechsim/kernel/memory_recorder_sink.hpp"
#include "mechsim/kernel/null_party.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"
#include "scripted_party.hpp"

using Catch::Matchers::ContainsSubstring;
using mechsim::kernel::CoSimKernel;
using mechsim::kernel::CouplingValue;
using mechsim::kernel::CouplingWriter;
using mechsim::kernel::derive_child_seeds;
using mechsim::kernel::Duration;
using mechsim::kernel::EndCondition;
using mechsim::kernel::EndReason;
using mechsim::kernel::EndTriggers;
using mechsim::kernel::InMemoryRecorderSink;
using mechsim::kernel::KernelConfigError;
using mechsim::kernel::NullParty;
using mechsim::kernel::PartyFault;
using mechsim::kernel::PartyRole;
using mechsim::kernel::Rate;
using mechsim::kernel::RateSource;
using mechsim::kernel::RunConfig;
using mechsim::kernel::RunOutcome;
using mechsim::kernel::RunReport;
using mechsim::kernel::Seconds;
using mechsim::kernel::SeriesSample;
using mechsim::kernel::SimTime;
using mechsim::kernel::TickRecord;
using mechsim::kernel::test::FailIn;
using mechsim::kernel::test::ScriptedParty;

namespace {

using Log = std::vector<std::string>;

// Registers a scripted party and returns a non-owning handle the test uses to read it back; the kernel owns it.
ScriptedParty& add(CoSimKernel& kernel, std::string_view name, Rate rate, Log& log) {
    auto party = std::make_unique<ScriptedParty>(std::string(name), rate, log);
    ScriptedParty& handle = *party;
    kernel.register_party(std::move(party));
    return handle;
}

EndCondition until(std::int64_t numerator, std::int64_t denominator) {
    EndCondition end;
    end.end_time = Seconds(numerator, denominator);
    return end;
}

void log_rows(InMemoryRecorderSink& sink, Log& log) {
    static_cast<void>(
        sink.subscribe([&log](const TickRecord& row) { log.push_back("row@" + std::to_string(row.time.count())); }));
}

std::vector<std::string> steps_only(const Log& log) {
    std::vector<std::string> out;
    for (const std::string& entry : log) {
        if (entry.find('@') != std::string::npos) {
            out.push_back(entry);
        }
    }
    return out;
}

const PartyFault& fault_of(const RunReport& report) {
    if (!report.fault.has_value()) {
        throw std::logic_error("report has no fault");
    }
    return *report.fault;
}

RunReport run_with(CoSimKernel& kernel, const EndCondition& end, const EndTriggers& triggers,
                   mechsim::kernel::RecorderSink& sink) {
    kernel.set_end_triggers(triggers);
    return kernel.run(end, sink);
}

std::int64_t steps_of(const RunReport& report, const std::string& name) {
    for (const auto& count : report.step_counts) {
        if (count.party == name) {
            return count.steps;
        }
    }
    return -1;
}

}  // namespace

// ===== Ordering and rates ============================================================================================

TEST_CASE("Parties step in KER-009 registration order across 1 kHz, 200 Hz and 0.5 Hz, row last",
          "[KER-003][KER-005][KER-006][KER-009][KER-013][KER-025]") {
    // base = lcm(1000/1, 200/1, 1/2) = lcm(1000, 200, 1) / gcd(1, 1, 2) = 1000 Hz, so periods are 1000/1000 = 1,
    // 1000/200 = 5 and 1000/0.5 = 2000 ticks. Over end = 4 s = 4000 ticks: 1 kHz parties step 4000 times, the 200 Hz
    // controller 4000 / 5 = 800, the 0.5 Hz party 4000 / 2000 = 2, and physics at 1 kHz writes 4000 rows.
    // The controller's nominal 1 kHz is replaced by RunConfig::control_rate through its role (KER-005).
    RunConfig config;
    config.control_rate = Rate(200);
    CoSimKernel kernel(config);
    Log log;
    add(kernel, "physics", Rate(1000), log).party_role = PartyRole::physics;
    kernel.register_party(std::make_unique<NullParty>("integrity", Rate(1000)));
    add(kernel, "sensors", Rate(1000), log);
    ScriptedParty& controller = add(kernel, "controller", Rate(1000), log);
    controller.party_role = PartyRole::controller;
    add(kernel, "pins", Rate(1000), log);
    add(kernel, "drivers", Rate(1000), log);
    const ScriptedParty& thermal = add(kernel, "thermal", Rate(1, 2), log);
    std::vector<std::int64_t> controller_dt;
    controller.on_step = [&controller_dt](SimTime, Duration dt, const ScriptedParty::Inputs&) {
        controller_dt.push_back(dt.count());
    };
    InMemoryRecorderSink sink;
    log_rows(sink, log);

    const RunReport report = kernel.run(until(4, 1), sink);

    Log expected{"physics:seed",    "physics:start",    "sensors:seed", "sensors:start",
                 "controller:seed", "controller:start", "pins:seed",    "pins:start",
                 "drivers:seed",    "drivers:start",    "thermal:seed", "thermal:start"};
    for (std::int64_t t = 0; t < 4000; ++t) {
        const std::string at = "@" + std::to_string(t);
        expected.push_back("physics" + at);
        expected.push_back("sensors" + at);
        if (t % 5 == 0) {
            expected.push_back("controller" + at);
        }
        expected.push_back("pins" + at);
        expected.push_back("drivers" + at);
        if (t % 2000 == 0) {
            expected.push_back("thermal" + at);
        }
        expected.push_back("row@" + std::to_string(t + 1));
    }
    CHECK(log == expected);
    CHECK(report.base_rate == Rate(1000));
    CHECK(kernel.config().base_rate == Rate(1000));
    CHECK(thermal.base_rate() == Rate(1000));
    CHECK(report.ticks == 4000);
    CHECK(report.end_reason == EndReason::max_duration);
    CHECK(report.outcome == RunOutcome::end_condition_met);
    CHECK(steps_of(report, "physics") == 4000);
    CHECK(steps_of(report, "integrity") == 4000);
    CHECK(steps_of(report, "controller") == 800);
    CHECK(steps_of(report, "thermal") == 2);
    CHECK(sink.rows().size() == 4000);
    CHECK(controller_dt == std::vector<std::int64_t>(800, 5));
    REQUIRE(report.party_rates.size() == 7);
    CHECK(report.party_rates[3].party == "controller");
    CHECK(report.party_rates[3].rate == Rate(200));
    CHECK(report.party_rates[3].source == RateSource::control_rate);
    CHECK(report.party_rates[0].source == RateSource::nominal);
}

TEST_CASE("An explicit party_rates entry wins over control_rate and both are reported", "[KER-005][KER-006]") {
    // control_rate 200 Hz, party_rates["controller"] = 100 Hz: the controller runs at 100 Hz, period 1000/100 = 10
    // ticks at base lcm(1000, 100) = 1000 Hz, so 10 steps in 0.1 s = 100 ticks.
    RunConfig config;
    config.control_rate = Rate(200);
    config.party_rates.emplace("controller", Rate(100));
    CoSimKernel kernel(config);
    Log log;
    add(kernel, "controller", Rate(1000), log).party_role = PartyRole::controller;
    InMemoryRecorderSink sink;
    const RunReport report = kernel.run(until(1, 10), sink);
    REQUIRE(report.party_rates.size() == 1);
    CHECK(report.party_rates[0].rate == Rate(100));
    CHECK(report.party_rates[0].source == RateSource::override);
    CHECK(steps_of(report, "controller") == 10);
}

TEST_CASE("A physics-role party off the physics rate is a config error", "[KER-004][KER-007]") {
    RunConfig config;
    Log log;
    InMemoryRecorderSink sink;
    SECTION("nominal rate differs") {
        CoSimKernel kernel(config);
        add(kernel, "physics", Rate(500), log).party_role = PartyRole::physics;
        CHECK_THROWS_WITH(kernel.run(until(1, 1), sink), ContainsSubstring("'physics'"));
    }
    SECTION("override differs") {
        config.party_rates.emplace("physics", Rate(2000));
        CoSimKernel kernel(config);
        add(kernel, "physics", Rate(1000), log).party_role = PartyRole::physics;
        CHECK_THROWS_WITH(kernel.run(until(1, 1), sink), ContainsSubstring("'physics'"));
    }
    SECTION("matching rate runs") {
        CoSimKernel kernel(config);
        add(kernel, "physics", Rate(1000), log).party_role = PartyRole::physics;
        CHECK(kernel.run(until(1, 1000), sink).outcome == RunOutcome::end_condition_met);
        log.clear();
    }
    CHECK(log.empty());
}

TEST_CASE("End conditions are checked only at physics-step boundaries", "[KER-017][KER-009]") {
    // physics 1 kHz, signal 3 kHz: base 3000 Hz, physics period 3 ticks. stop is true from t = 1, but the first
    // boundary is t = 3, so the run ends there with 3 base ticks and 1 row.
    CoSimKernel kernel{RunConfig{}};
    Log log;
    add(kernel, "signal", Rate(3000), log);
    InMemoryRecorderSink sink;
    EndCondition end = until(1, 1);
    end.stop = [](SimTime t) { return t >= SimTime(1); };
    SECTION("stop") {
        const RunReport report = kernel.run(end, sink);
        CHECK(report.end_reason == EndReason::stop_requested);
        CHECK(report.ticks == 3);
        CHECK(report.end_time == SimTime(3));
        CHECK(sink.rows().size() == 1);
    }
    SECTION("at_time off a physics step: 1/3000 s is one base tick, a third of a physics step") {
        EndTriggers triggers;
        triggers.at_time = Seconds(1, 3000);
        CHECK_THROWS_WITH(run_with(kernel, until(1, 1), triggers, sink), ContainsSubstring("physics step"));
    }
}

TEST_CASE("Errors of dropped subscribers reach the run report", "[REC-020][KER-022]") {
    CoSimKernel kernel{RunConfig{}};
    Log log;
    add(kernel, "a", Rate(1000), log);
    InMemoryRecorderSink sink;
    static_cast<void>(sink.subscribe([](const TickRecord&) { throw std::runtime_error("viewer gone"); }));
    const RunReport report = kernel.run(until(5, 1000), sink);
    CHECK(report.outcome == RunOutcome::end_condition_met);
    CHECK(report.sink_errors == std::vector<std::string>{"viewer gone"});
    CHECK(sink.rows().size() == 5);
}

TEST_CASE("Step order follows registration, not names or addresses", "[KER-013]") {
    // Two parties at 1 kHz for 3 ms = 3 ticks; registered b then a, so b steps first at every tick.
    CoSimKernel kernel{RunConfig{}};
    Log log;
    add(kernel, "b", Rate(1000), log);
    add(kernel, "a", Rate(1000), log);
    InMemoryRecorderSink sink;
    static_cast<void>(kernel.run(until(3, 1000), sink));
    CHECK(steps_only(log) == Log{"b@0", "a@0", "b@1", "a@1", "b@2", "a@2"});
}

TEST_CASE("A party finer than physics and a chunked party; rows stay one per physics step",
          "[KER-003][KER-004][KER-009][KER-022]") {
    // physics 1 kHz, signal 3 kHz, emulator 100 Hz: base = lcm(1000, 3000, 100) = 3000 Hz; periods 3, 1 and 30 ticks.
    // end = 1/100 s = 30 ticks: physics 10 steps, signal 30, emulator 1 step of dt = 30 ticks (10 ms of virtual time).
    // Rows: one per physics step, 10, at the end of each physics period: t = 3, 6, ..., 30.
    CoSimKernel kernel{RunConfig{}};
    Log log;
    add(kernel, "physics", Rate(1000), log);
    add(kernel, "signal", Rate(3000), log);
    ScriptedParty& emulator = add(kernel, "emulator", Rate(100), log);
    std::vector<std::int64_t> chunks;
    emulator.on_step = [&chunks](SimTime, Duration dt, const ScriptedParty::Inputs&) { chunks.push_back(dt.count()); };
    InMemoryRecorderSink sink;
    log_rows(sink, log);
    const RunReport report = kernel.run(until(1, 100), sink);

    Log expected;
    for (std::int64_t t = 0; t < 30; ++t) {
        const std::string at = "@" + std::to_string(t);
        if (t % 3 == 0) {
            expected.push_back("physics" + at);
        }
        expected.push_back("signal" + at);
        if (t % 30 == 0) {
            expected.push_back("emulator" + at);
        }
        if ((t + 1) % 3 == 0) {
            expected.push_back("row@" + std::to_string(t + 1));
        }
    }
    CHECK(steps_only(log) == expected);
    CHECK(report.base_rate == Rate(3000));
    CHECK(chunks == std::vector<std::int64_t>{30});
    REQUIRE(sink.rows().size() == 10);
    for (std::size_t k = 0; k < 10; ++k) {
        CHECK(sink.rows()[k].time == SimTime(3 * (static_cast<std::int64_t>(k) + 1)));
    }
}

TEST_CASE("The physics rate comes from the run configuration", "[KER-004]") {
    // physics_rate 500 Hz, one party at 500 Hz: base 500 Hz, 1 tick per physics step; 1 s gives 500 rows.
    RunConfig config;
    config.physics_rate = Rate(500);
    CoSimKernel kernel(config);
    Log log;
    add(kernel, "physics", Rate(500), log);
    InMemoryRecorderSink sink;
    const RunReport report = kernel.run(until(1, 1), sink);
    CHECK(report.base_rate == Rate(500));
    CHECK(sink.rows().size() == 500);
}

TEST_CASE("Time starts at zero and advances only inside run", "[KER-001][KER-002][KER-024]") {
    CoSimKernel kernel{RunConfig{}};
    Log log;
    add(kernel, "physics", Rate(1000), log);
    CHECK(kernel.now() == SimTime());
    InMemoryRecorderSink sink;
    const RunReport report = kernel.run(until(5, 1000), sink);
    CHECK(steps_only(log).front() == "physics@0");
    CHECK(kernel.now() == SimTime(5));
    CHECK(report.end_time == SimTime(5));
}

// ===== Coupling exchange =============================================================================================

TEST_CASE("Zero-order hold: a 0.5 Hz publisher is held for a 1 kHz consumer", "[KER-008][KER-011]") {
    // base 1000 Hz; the slow party steps at t = 0, 2000, 4000 within 5 s = 5000 ticks and publishes its step count.
    // Consumer registered after it reads at its own step, after the slow party's write in the same tick:
    //   value(t) = floor(t / 2000) + 1.
    // Consumer registered before it reads before the slow party's write in that tick: t = 0 sees nothing,
    //   t >= 1 sees ceil(t / 2000) = (t + 1999) / 2000.
    CoSimKernel kernel{RunConfig{}};
    Log log;
    ScriptedParty& early = add(kernel, "early", Rate(1000), log);
    ScriptedParty& slow = add(kernel, "slow", Rate(1, 2), log);
    ScriptedParty& late = add(kernel, "late", Rate(1000), log);
    slow.published_names = {"slow.count"};
    early.consumed_names = {"slow.count"};
    late.consumed_names = {"slow.count"};
    std::int64_t count = 0;
    slow.on_write = [&count](SimTime, CouplingWriter& out) {
        ++count;
        out.set("slow.count", CouplingValue{count});
    };
    std::vector<CouplingValue> early_seen;
    std::vector<CouplingValue> late_seen;
    early.on_step = [&early_seen](SimTime, Duration, const ScriptedParty::Inputs& in) {
        early_seen.push_back(in.at("slow.count"));
    };
    late.on_step = [&late_seen](SimTime, Duration, const ScriptedParty::Inputs& in) {
        late_seen.push_back(in.at("slow.count"));
    };
    InMemoryRecorderSink sink;
    static_cast<void>(kernel.run(until(5, 1), sink));

    REQUIRE(early_seen.size() == 5000);
    REQUIRE(late_seen.size() == 5000);
    CHECK(std::holds_alternative<std::monostate>(early_seen[0]));
    for (std::int64_t t = 0; t < 5000; ++t) {
        const auto i = static_cast<std::size_t>(t);
        CHECK(late_seen[i] == CouplingValue{(t / 2000) + 1});
        if (t >= 1) {
            CHECK(early_seen[i] == CouplingValue{(t + 1999) / 2000});
        }
    }
}

TEST_CASE("Actuator force from tick n reaches physics in step n + 1", "[KER-010][KER-008]") {
    // transmission (registered last) publishes force = t at tick t; physics (first) reads at tick n the value
    // written at tick n - 1, so it sees n - 1, and nothing at n = 0. 10 ticks at 1 kHz.
    CoSimKernel kernel{RunConfig{}};
    Log log;
    ScriptedParty& physics = add(kernel, "physics", Rate(1000), log);
    ScriptedParty& transmission = add(kernel, "transmission", Rate(1000), log);
    transmission.published_names = {"joint.force"};
    physics.consumed_names = {"joint.force"};
    transmission.on_write = [](SimTime t, CouplingWriter& out) {
        out.set("joint.force", CouplingValue{static_cast<double>(t.count())});
    };
    std::vector<CouplingValue> seen;
    physics.on_step = [&seen](SimTime, Duration, const ScriptedParty::Inputs& in) {
        seen.push_back(in.at("joint.force"));
    };
    InMemoryRecorderSink sink;
    static_cast<void>(kernel.run(until(10, 1000), sink));
    REQUIRE(seen.size() == 10);
    CHECK(std::holds_alternative<std::monostate>(seen[0]));
    for (std::size_t n = 1; n < 10; ++n) {
        CHECK(seen[n] == CouplingValue{static_cast<double>(n - 1)});
    }
    CHECK(sink.rows().back().coupling.size() == 1);
    CHECK(sink.rows().back().coupling[0].name == "joint.force");
    CHECK(sink.rows().back().coupling[0].value == CouplingValue{9.0});
}

TEST_CASE("Coupling configuration errors are found before t = 0", "[KER-008][KER-007]") {
    CoSimKernel kernel{RunConfig{}};
    Log log;
    ScriptedParty& a = add(kernel, "a", Rate(1000), log);
    ScriptedParty& b = add(kernel, "b", Rate(1000), log);
    InMemoryRecorderSink sink;

    SECTION("consumed but unpublished") {
        b.consumed_names = {"ghost.value"};
        CHECK_THROWS_WITH(kernel.run(until(1, 1), sink), ContainsSubstring("'b'") && ContainsSubstring("ghost.value"));
    }
    SECTION("two publishers") {
        a.published_names = {"x.y"};
        b.published_names = {"x.y"};
        CHECK_THROWS_WITH(kernel.run(until(1, 1), sink), ContainsSubstring("'a'") && ContainsSubstring("'b'"));
    }
    SECTION("empty published name") {
        a.published_names = {""};
        CHECK_THROWS_AS(kernel.run(until(1, 1), sink), KernelConfigError);
    }
    CHECK(log.empty());
    CHECK(sink.rows().empty());
}

TEST_CASE("Coupling misuse inside a step is a party fault", "[KER-008][KER-015]") {
    CoSimKernel kernel{RunConfig{}};
    Log log;
    ScriptedParty& a = add(kernel, "a", Rate(1000), log);
    a.published_names = {"a.out"};
    InMemoryRecorderSink sink;

    SECTION("writing an unpublished name") {
        a.on_write = [](SimTime, CouplingWriter& out) { out.set("a.other", CouplingValue{1.0}); };
        const RunReport report = kernel.run(until(1, 1), sink);
        REQUIRE(report.fault.has_value());
        CHECK_THAT(fault_of(report).cause(), ContainsSubstring("a.other"));
    }
    SECTION("changing a variable's type") {
        a.on_write = [](SimTime t, CouplingWriter& out) {
            if (t == SimTime()) {
                out.set("a.out", CouplingValue{1.0});
            } else {
                out.set("a.out", CouplingValue{true});
            }
        };
        const RunReport report = kernel.run(until(1, 1), sink);
        REQUIRE(report.fault.has_value());
        CHECK(fault_of(report).time() == SimTime(1));
        CHECK_THAT(fault_of(report).cause(), ContainsSubstring("changed type"));
    }
}

// ===== End conditions ================================================================================================

TEST_CASE("Each end condition ends the run and is reported", "[KER-017]") {
    // All at 1 kHz, base 1000 Hz, end_time 1 s = 1000 ticks as the maximum duration.
    CoSimKernel kernel{RunConfig{}};
    Log log;
    ScriptedParty& a = add(kernel, "a", Rate(1000), log);
    a.published_names = {"a.done"};
    InMemoryRecorderSink sink;
    EndCondition end = until(1, 1);
    EndTriggers triggers;

    SECTION("maximum duration") {
        const RunReport report = run_with(kernel, end, triggers, sink);
        CHECK(report.end_reason == EndReason::max_duration);
        CHECK(report.ticks == 1000);
    }
    SECTION("at time: 1/2 s = 500 ticks") {
        triggers.at_time = Seconds(1, 2);
        const RunReport report = run_with(kernel, end, triggers, sink);
        CHECK(report.end_reason == EndReason::at_time);
        CHECK(report.ticks == 500);
        CHECK(report.end_time == SimTime(500));
    }
    SECTION("at time beyond the maximum duration") {
        triggers.at_time = Seconds(2, 1);
        const RunReport report = run_with(kernel, end, triggers, sink);
        CHECK(report.end_reason == EndReason::max_duration);
        CHECK(report.ticks == 1000);
    }
    SECTION("published boolean: true written at t = 299, seen at the sync point t = 300") {
        a.on_write = [](SimTime t, CouplingWriter& out) { out.set("a.done", CouplingValue{t.count() >= 299}); };
        triggers.on_signal = "a.done";
        const RunReport report = run_with(kernel, end, triggers, sink);
        CHECK(report.end_reason == EndReason::signal);
        CHECK(report.ticks == 300);
        CHECK(sink.rows().size() == 300);
    }
    SECTION("party-declared state: finished after 42 steps, at t = 42") {
        a.finish_after_steps = 42;
        triggers.party_state = "a";
        const RunReport report = run_with(kernel, end, triggers, sink);
        CHECK(report.end_reason == EndReason::party_state);
        CHECK(report.ticks == 42);
    }
    SECTION("signal outranks party state at the same sync point") {
        a.on_write = [](SimTime, CouplingWriter& out) { out.set("a.done", CouplingValue{true}); };
        a.finish_after_steps = 1;
        triggers.on_signal = "a.done";
        triggers.party_state = "a";
        const RunReport report = run_with(kernel, end, triggers, sink);
        CHECK(report.end_reason == EndReason::signal);
        CHECK(report.ticks == 1);
    }
    SECTION("stop callback at t = 7") {
        end.stop = [](SimTime t) { return t == SimTime(7); };
        const RunReport report = run_with(kernel, end, triggers, sink);
        CHECK(report.end_reason == EndReason::stop_requested);
        CHECK(report.ticks == 7);
    }
    SECTION("a non-boolean end signal is a run error naming its publisher") {
        a.on_write = [](SimTime, CouplingWriter& out) { out.set("a.done", CouplingValue{1.0}); };
        triggers.on_signal = "a.done";
        const RunReport report = run_with(kernel, end, triggers, sink);
        CHECK(report.outcome == RunOutcome::run_error);
        REQUIRE(report.fault.has_value());
        CHECK(fault_of(report).party() == "a");
    }
}

TEST_CASE("End condition configuration errors are found before t = 0", "[KER-017][KER-007]") {
    CoSimKernel kernel{RunConfig{}};
    Log log;
    add(kernel, "a", Rate(1000), log);
    InMemoryRecorderSink sink;
    EndCondition end = until(1, 1);
    EndTriggers triggers;

    SECTION("end time off the base tick: 1/3000 s at base 1000 Hz") {
        end.end_time = Seconds(1, 3000);
        CHECK_THROWS_WITH(run_with(kernel, end, triggers, sink), ContainsSubstring("end_time"));
    }
    SECTION("at time off the base tick") {
        triggers.at_time = Seconds(1, 3000);
        CHECK_THROWS_WITH(run_with(kernel, end, triggers, sink), ContainsSubstring("at_time"));
    }
    SECTION("at time zero") {
        triggers.at_time = Seconds(0, 1);
        CHECK_THROWS_AS(run_with(kernel, end, triggers, sink), KernelConfigError);
    }
    SECTION("unpublished end signal") {
        triggers.on_signal = "nobody.flag";
        CHECK_THROWS_WITH(run_with(kernel, end, triggers, sink), ContainsSubstring("nobody.flag"));
    }
    SECTION("unregistered end party") {
        triggers.party_state = "ghost";
        CHECK_THROWS_WITH(run_with(kernel, end, triggers, sink), ContainsSubstring("ghost"));
    }
    CHECK(log.empty());
}

// ===== Rate and registration configuration errors ====================================================================

TEST_CASE("Rate and registration errors are found before t = 0 and name the party", "[KER-007]") {
    RunConfig config;
    Log log;
    InMemoryRecorderSink sink;

    SECTION("lcm overflow names the party") {
        CoSimKernel kernel(config);
        add(kernel, "a", Rate(1000), log);
        // 9223372036854775783 is prime, so lcm(1000, it) = 1000 * it > INT64_MAX.
        add(kernel, "huge", Rate(9'223'372'036'854'775'783), log);
        CHECK_THROWS_WITH(kernel.run(until(1, 1), sink), ContainsSubstring("'huge'"));
    }
    SECTION("rate override for an unregistered party") {
        config.party_rates.emplace("ghost", Rate(10));
        CoSimKernel kernel(config);
        add(kernel, "a", Rate(1000), log);
        CHECK_THROWS_WITH(kernel.run(until(1, 1), sink), ContainsSubstring("ghost"));
    }
    SECTION("non-positive rate and negative time") {
        CHECK_THROWS_AS(Rate(0), KernelConfigError);
        CHECK_THROWS_AS(Rate(1, -1), KernelConfigError);
        CHECK_THROWS_AS(Seconds(-1, 1), KernelConfigError);
    }
    SECTION("duplicate, empty and null parties; registration after run; run twice") {
        CoSimKernel kernel(config);
        add(kernel, "a", Rate(1000), log);
        CHECK_THROWS_AS(add(kernel, "a", Rate(1000), log), KernelConfigError);
        CHECK_THROWS_AS(add(kernel, "", Rate(1000), log), KernelConfigError);
        CHECK_THROWS_AS(kernel.register_party(nullptr), KernelConfigError);
        static_cast<void>(kernel.run(until(1, 1000), sink));
        CHECK_THROWS_AS(add(kernel, "b", Rate(1000), log), KernelConfigError);
        CHECK_THROWS_AS(kernel.run(until(1, 1000), sink), KernelConfigError);
        log.clear();
    }
    CHECK(log.empty());
}

// ===== Faults ========================================================================================================

TEST_CASE("A fault in any hook is wrapped, fanned out and reported", "[KER-015][KER-017]") {
    // Three 1 kHz parties; b fails at t = 10. a stepped at t = 0..10 (11 steps), b and c at t = 0..9 (10 steps).
    // Ticks completed: 10; rows: 10 (none for the failed tick).
    CoSimKernel kernel{RunConfig{}};
    Log log;
    add(kernel, "a", Rate(1000), log);
    ScriptedParty& b = add(kernel, "b", Rate(1000), log);
    add(kernel, "c", Rate(1000), log);
    b.fail_at = SimTime(10);
    InMemoryRecorderSink sink;

    SECTION("in step") {
        b.fail_in = FailIn::step;
        const RunReport report = kernel.run(until(1, 1), sink);
        CHECK(report.outcome == RunOutcome::run_error);
        CHECK(report.end_reason == EndReason::run_error);
        REQUIRE(report.fault.has_value());
        CHECK(fault_of(report).party() == "b");
        CHECK(fault_of(report).time() == SimTime(10));
        CHECK(fault_of(report).cause() == "injected failure");
        CHECK(report.end_time == SimTime(10));
        CHECK(report.ticks == 10);
        CHECK(steps_of(report, "a") == 11);
        CHECK(steps_of(report, "b") == 10);
        CHECK(steps_of(report, "c") == 10);
        CHECK(sink.rows().size() == 10);
        const Log tail(log.end() - 3, log.end());
        CHECK(tail == Log{"a:fault:b", "b:fault:b", "c:fault:b"});
    }
    SECTION("in read_inputs") {
        b.fail_in = FailIn::read;
        const RunReport report = kernel.run(until(1, 1), sink);
        REQUIRE(report.fault.has_value());
        CHECK(fault_of(report).time() == SimTime(10));
        CHECK(steps_of(report, "b") == 10);
    }
    SECTION("in write_outputs: the step ran but does not count") {
        b.fail_in = FailIn::write;
        const RunReport report = kernel.run(until(1, 1), sink);
        REQUIRE(report.fault.has_value());
        CHECK(fault_of(report).time() == SimTime(10));
        CHECK(steps_of(report, "b") == 10);
    }
    SECTION("a non-standard exception") {
        b.fail_in = FailIn::step;
        b.throw_non_std = true;
        const RunReport report = kernel.run(until(1, 1), sink);
        REQUIRE(report.fault.has_value());
        CHECK(fault_of(report).cause() == "unknown exception");
    }
    SECTION("in record: row at t = 10 not written, 9 rows") {
        b.fail_in = FailIn::record;
        const RunReport report = kernel.run(until(1, 1), sink);
        REQUIRE(report.fault.has_value());
        CHECK(fault_of(report).party() == "b");
        CHECK(fault_of(report).time() == SimTime(10));
        CHECK(sink.rows().size() == 9);
    }
    SECTION("in start: fault at t = 0, nothing stepped") {
        b.fail_in = FailIn::start;
        b.fail_at = SimTime();
        const RunReport report = kernel.run(until(1, 1), sink);
        REQUIRE(report.fault.has_value());
        CHECK(fault_of(report).party() == "b");
        CHECK(fault_of(report).time() == SimTime());
        CHECK(report.ticks == 0);
        CHECK(steps_only(log).empty());
    }
}

TEST_CASE("A party that throws PartyFault keeps its cause under its registered name", "[KER-015]") {
    class Rogue final : public mechsim::kernel::Party {
    public:
        [[nodiscard]] std::string_view name() const override {
            return "rogue";
        }
        [[nodiscard]] Rate nominal_rate() const override {
            return Rate(1000);
        }
        [[nodiscard]] std::span<const std::string> published() const override {
            return {};
        }
        [[nodiscard]] std::span<const std::string> consumed() const override {
            return {};
        }
        void start(const Rate& /*base_rate*/) override {}
        void step(SimTime t, Duration /*dt*/) override {
            throw PartyFault("someone-else", t, "own cause");
        }
        void on_fault(const PartyFault& /*fault*/) noexcept override {}
    };
    CoSimKernel kernel{RunConfig{}};
    kernel.register_party(std::make_unique<Rogue>());
    InMemoryRecorderSink sink;
    const RunReport report = kernel.run(until(1, 1), sink);
    REQUIRE(report.fault.has_value());
    CHECK(fault_of(report).party() == "rogue");
    CHECK(fault_of(report).cause() == "own cause");
}

TEST_CASE("A throwing sink is a recorder fault", "[KER-015][KER-022]") {
    class BrokenSink final : public mechsim::kernel::RecorderSink {
    public:
        void write_row(const TickRecord& /*row*/) override {
            throw std::runtime_error("disk full");
        }
        [[nodiscard]] mechsim::kernel::SubscriptionId subscribe(mechsim::kernel::TickCallback /*callback*/) override {
            return {};
        }
        void unsubscribe(mechsim::kernel::SubscriptionId /*id*/) override {}
    };
    CoSimKernel kernel{RunConfig{}};
    Log log;
    add(kernel, "a", Rate(1000), log);
    BrokenSink sink;
    const RunReport report = kernel.run(until(1, 1), sink);
    REQUIRE(report.fault.has_value());
    CHECK(fault_of(report).party() == "recorder");
    CHECK(fault_of(report).time() == SimTime(1));
    CHECK(fault_of(report).cause() == "disk full");
    CHECK(log.back() == "a:fault:recorder");
}

// ===== Seeds and determinism =========================================================================================

TEST_CASE("Child seeds are SplitMix64 outputs of the run seed in registration order", "[KER-012]") {
    // Reference outputs of SplitMix64 (Vigna) from state 0: e220a8397b1dcdaf, 6e789e6aa1b965f4, 06c45d188009454f.
    const std::vector<std::uint64_t> reference{0xE220A8397B1DCDAFULL, 0x6E789E6AA1B965F4ULL, 0x06C45D188009454FULL};
    CHECK(derive_child_seeds(0, 3) == reference);
    CoSimKernel kernel{RunConfig{}};
    Log log;
    const ScriptedParty& first = add(kernel, "z", Rate(1000), log);
    const ScriptedParty& second = add(kernel, "a", Rate(1000), log);
    const ScriptedParty& third = add(kernel, "m", Rate(1000), log);
    InMemoryRecorderSink sink;
    static_cast<void>(kernel.run(until(1, 1000), sink));
    CHECK(first.seed() == reference[0]);
    CHECK(second.seed() == reference[1]);
    CHECK(third.seed() == reference[2]);
    CHECK(derive_child_seeds(1, 3) != reference);
}

namespace {

// A seeded noise source (xorshift64 on the child seed, mantissa bits to [0, 1)) at 3 kHz, an integrator at 1 kHz
// consuming it, and a recorder slice; returns the recorded rows of 0.2 s.
std::vector<TickRecord> seeded_run(std::uint64_t seed) {
    RunConfig config;
    config.seed = seed;
    CoSimKernel kernel(config);
    Log log;
    ScriptedParty& noise = add(kernel, "noise", Rate(3000), log);
    ScriptedParty& integrator = add(kernel, "integrator", Rate(1000), log);
    noise.published_names = {"noise.value"};
    integrator.consumed_names = {"noise.value"};
    integrator.published_names = {"integrator.sum"};
    auto state = std::make_shared<std::uint64_t>(0);
    auto sum = std::make_shared<double>(0.0);
    noise.on_write = [state, &noise](SimTime, CouplingWriter& out) {
        if (*state == 0) {
            *state = noise.seed() | 1U;
        }
        *state ^= *state << 13U;
        *state ^= *state >> 7U;
        *state ^= *state << 17U;
        const double unit = std::bit_cast<double>((*state >> 12U) | 0x3FF0000000000000ULL) - 1.0;
        out.set("noise.value", CouplingValue{unit});
    };
    integrator.on_step = [sum](SimTime, Duration, const ScriptedParty::Inputs& in) {
        *sum += std::get<double>(in.at("noise.value")) * 1e-3;
    };
    integrator.on_write = [sum](SimTime, CouplingWriter& out) { out.set("integrator.sum", CouplingValue{*sum}); };
    integrator.on_record = [sum](TickRecord& row) {
        row.actuator_efforts.push_back(SeriesSample{"integrator.sum", "N", *sum});
    };
    InMemoryRecorderSink sink;
    static_cast<void>(kernel.run(until(1, 5), sink));
    return {sink.rows().begin(), sink.rows().end()};
}

bool bitwise_equal(const CouplingValue& a, const CouplingValue& b) {
    if (a.index() != b.index()) {
        return false;
    }
    if (std::holds_alternative<double>(a)) {
        return std::bit_cast<std::uint64_t>(std::get<double>(a)) == std::bit_cast<std::uint64_t>(std::get<double>(b));
    }
    return a == b;
}

bool rows_bitwise_equal(const std::vector<TickRecord>& x, const std::vector<TickRecord>& y) {
    if (x.size() != y.size()) {
        return false;
    }
    for (std::size_t i = 0; i < x.size(); ++i) {
        if (x[i].time != y[i].time || x[i].coupling.size() != y[i].coupling.size() ||
            x[i].actuator_efforts.size() != y[i].actuator_efforts.size()) {
            return false;
        }
        for (std::size_t k = 0; k < x[i].coupling.size(); ++k) {
            if (x[i].coupling[k].name != y[i].coupling[k].name ||
                !bitwise_equal(x[i].coupling[k].value, y[i].coupling[k].value)) {
                return false;
            }
        }
        for (std::size_t k = 0; k < x[i].actuator_efforts.size(); ++k) {
            const SeriesSample& p = x[i].actuator_efforts[k];
            const SeriesSample& q = y[i].actuator_efforts[k];
            if (p.name != q.name || p.unit != q.unit ||
                std::bit_cast<std::uint64_t>(p.value) != std::bit_cast<std::uint64_t>(q.value)) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

TEST_CASE("Two runs with the same seed record bitwise-identical rows", "[KER-014][KER-012][KER-022][REC-011]") {
    // base lcm(1000, 3000) = 3000 Hz; 0.2 s = 600 ticks; physics 1 kHz gives 200 rows, stamped 3, 6, ..., 600.
    const std::vector<TickRecord> first = seeded_run(42);
    const std::vector<TickRecord> second = seeded_run(42);
    REQUIRE(first.size() == 200);
    CHECK(first.front().time == SimTime(3));
    CHECK(first.back().time == SimTime(600));
    CHECK(first.back().coupling.size() == 2);
    CHECK(rows_bitwise_equal(first, second));
    CHECK_FALSE(rows_bitwise_equal(first, seeded_run(43)));
}

// ===== Recorder streaming and trigger support =======================================================================

TEST_CASE("Subscribers receive every row in subscription order", "[KER-022][REC-019]") {
    // 1 kHz physics with a 3 kHz party for 1 s: base 3000 Hz, rows = 1000 physics steps.
    CoSimKernel kernel{RunConfig{}};
    Log log;
    add(kernel, "physics", Rate(1000), log);
    add(kernel, "signal", Rate(3000), log);
    InMemoryRecorderSink sink;
    Log streamed;
    static_cast<void>(sink.subscribe(
        [&streamed](const TickRecord& row) { streamed.push_back("A" + std::to_string(row.time.count())); }));
    static_cast<void>(sink.subscribe(
        [&streamed](const TickRecord& row) { streamed.push_back("B" + std::to_string(row.time.count())); }));
    static_cast<void>(kernel.run(until(1, 1), sink));
    Log expected;
    for (std::int64_t k = 1; k <= 1000; ++k) {
        expected.push_back("A" + std::to_string(3 * k));
        expected.push_back("B" + std::to_string(3 * k));
    }
    CHECK(sink.rows().size() == 1000);
    CHECK(streamed == expected);
}

TEST_CASE("A kernel without trigger support rejects end triggers; CoSimKernel rejects them after run", "[KER-017]") {
    mechsim::kernel::NullKernel null_kernel{RunConfig{}};
    CHECK_NOTHROW(null_kernel.set_end_triggers(EndTriggers{}));
    EndTriggers triggers;
    triggers.party_state = "a";
    CHECK_THROWS_AS(null_kernel.set_end_triggers(triggers), KernelConfigError);
    CoSimKernel kernel{RunConfig{}};
    InMemoryRecorderSink sink;
    static_cast<void>(kernel.run(until(1, 1000), sink));
    CHECK_THROWS_AS(kernel.set_end_triggers(EndTriggers{}), KernelConfigError);
}
