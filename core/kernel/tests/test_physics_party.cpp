#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/cosim_kernel.hpp"
#include "mechsim/kernel/kernel.hpp"
#include "mechsim/kernel/memory_recorder_sink.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_party.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"
#include "scripted_party.hpp"

using namespace mechsim::kernel;  // NOLINT(google-build-using-namespace)
using mechsim::kernel::test::ScriptedParty;

namespace {

// One body whose x position counts the steps; records the inputs each step received.
class CountingAdapter final : public PhysicsAdapter {
public:
    struct Shared {
        std::vector<PhysicsInputs> received;
        PhysicsHealth health;
        // Mid-run operations as "operation index value @completed steps".
        std::vector<std::string> calls;
    };

    explicit CountingAdapter(std::shared_ptr<Shared> shared) : shared_(std::move(shared)) {
        outputs_.bodies.resize(1);
    }

    [[nodiscard]] std::span<const std::string> supported_elements() const override { return {}; }
    void load(const PreparedModel& /*model*/, const Rate& /*rate*/) override {}
    void reset() override {}
    void step() override {
        shared_->received.push_back(pending_);
        pending_ = PhysicsInputs{};
        ++outputs_.steps;
        outputs_.bodies[0].position[0] = static_cast<double>(outputs_.steps);
    }
    [[nodiscard]] const PhysicsOutputs& outputs() const override { return outputs_; }
    [[nodiscard]] PhysicsHealth health() const override {
        PhysicsHealth health = shared_->health;
        health.steps = outputs_.steps;
        return health;
    }
    void set_inputs(const PhysicsInputs& inputs) override { pending_ = inputs; }
    void set_spring_offset(std::size_t /*spring*/, double /*offset*/) override {}
    void set_spring_stiffness(std::size_t /*spring*/, double /*stiffness*/) override {}
    void disable_constraint(std::size_t /*constraint*/) override {}
    void release_load(std::size_t /*load*/) override {}
    void set_payload_mass(std::size_t payload, double mass) override {
        log("mass " + std::to_string(payload) + " " + std::to_string(mass));
    }
    void set_payload_attachment(std::size_t payload, std::size_t body) override {
        std::string carrier = std::to_string(body);
        if (body == kWorldBody) {
            carrier = "world";
        }
        log("attach " + std::to_string(payload) + " " + carrier);
    }
    void set_transmission_slip(std::size_t transmission, double fraction) override {
        log("slip " + std::to_string(transmission) + " " + std::to_string(fraction));
    }

private:
    void log(const std::string& call) {
        shared_->calls.push_back(call + " @" + std::to_string(outputs_.steps));
    }

    std::shared_ptr<Shared> shared_;
    PhysicsOutputs outputs_;
    PhysicsInputs pending_;
};

PhysicsInputs force(std::vector<double> generalized, std::size_t wrench_body) {
    PhysicsInputs inputs;
    inputs.generalized_forces = std::move(generalized);
    inputs.body_wrenches.push_back(BodyWrench{wrench_body, {}});
    return inputs;
}

}  // namespace

TEST_CASE("Physics party exchanges outputs and summed inputs through the coupling table only",
          "[KER-008][KER-009][KER-016]") {
    auto shared = std::make_shared<CountingAdapter::Shared>();
    std::vector<std::string> log;
    CoSimKernel kernel{RunConfig{}};
    PhysicsPartyConfig config;
    config.inputs = {"a.inputs", "b.inputs"};
    kernel.register_party(std::make_unique<PhysicsParty>(std::make_unique<CountingAdapter>(shared), config));
    auto sensor = std::make_unique<ScriptedParty>("sensor", Rate(1000), log);
    std::vector<double> seen;
    sensor->consumed_names = {"physics.outputs"};
    sensor->on_step = [&seen](SimTime, Duration, const ScriptedParty::Inputs& inputs) {
        seen.push_back(payload_as<PhysicsOutputs>(inputs.at("physics.outputs"), "physics.outputs").bodies[0].position[0]);
    };
    kernel.register_party(std::move(sensor));
    auto a = std::make_unique<ScriptedParty>("a", Rate(1000), log);
    a->published_names = {"a.inputs"};
    a->on_write = [](SimTime, CouplingWriter& out) { out.set("a.inputs", make_payload(force({1.0, 2.0}, 0), "PhysicsInputs")); };
    kernel.register_party(std::move(a));
    auto b = std::make_unique<ScriptedParty>("b", Rate(1000), log);
    b->published_names = {"b.inputs"};
    b->on_write = [](SimTime, CouplingWriter& out) { out.set("b.inputs", make_payload(force({0.5, 0.25}, 3), "PhysicsInputs")); };
    kernel.register_party(std::move(b));
    InMemoryRecorderSink sink;
    const RunReport report = kernel.run(EndCondition{Seconds(3, 1000), {}}, sink);
    REQUIRE(report.outcome == RunOutcome::end_condition_met);
    // The sensor runs after physics in each tick, so it sees this tick's step.
    CHECK(seen == std::vector<double>{1.0, 2.0, 3.0});
    REQUIRE(shared->received.size() == 3);
    CHECK(shared->received[0].generalized_forces.empty());
    CHECK(shared->received[1].generalized_forces == std::vector<double>{1.5, 2.25});
    REQUIRE(shared->received[1].body_wrenches.size() == 2);
    CHECK(shared->received[1].body_wrenches[0].body == 0);
    CHECK(shared->received[1].body_wrenches[1].body == 3);
    REQUIRE(sink.rows().size() == 3);
    CHECK(sink.rows()[2].physics.bodies[0].position[0] == 3.0);
    CHECK(kernel.coupling().publisher("physics.outputs") == "physics");
}

TEST_CASE("Physics party ends the run on solver failure and drift", "[KER-016]") {
    for (const bool converged : {false, true}) {
        auto shared = std::make_shared<CountingAdapter::Shared>();
        shared->health.converged = converged;
        if (converged) {
            shared->health.max_constraint_drift = 2e-6;
        }
        CoSimKernel kernel{RunConfig{}};
        PhysicsPartyConfig config;
        config.inputs = {};
        kernel.register_party(std::make_unique<PhysicsParty>(std::make_unique<CountingAdapter>(shared), config));
        InMemoryRecorderSink sink;
        const RunReport report = kernel.run(EndCondition{Seconds(1, 100), {}}, sink);
        REQUIRE(report.outcome == RunOutcome::run_error);
        const PartyFault fault = report.fault.value_or(PartyFault("none", SimTime(), ""));
        CHECK(fault.party() == "physics");
        CHECK(fault.cause().find("KER-016") != std::string::npos);
        CHECK(sink.rows().empty());
    }
    CHECK_THROWS_AS(PhysicsParty(nullptr, PhysicsPartyConfig{}), std::invalid_argument);
}

TEST_CASE("Physics party refuses inputs of unequal sizes", "[KER-016]") {
    auto shared = std::make_shared<CountingAdapter::Shared>();
    std::vector<std::string> log;
    CoSimKernel kernel{RunConfig{}};
    PhysicsPartyConfig config;
    config.inputs = {"a.inputs", "b.inputs"};
    kernel.register_party(std::make_unique<PhysicsParty>(std::make_unique<CountingAdapter>(shared), config));
    auto a = std::make_unique<ScriptedParty>("a", Rate(1000), log);
    a->published_names = {"a.inputs", "b.inputs"};
    a->on_write = [](SimTime, CouplingWriter& out) {
        out.set("a.inputs", make_payload(force({1.0}, 0), "PhysicsInputs"));
        out.set("b.inputs", make_payload(force({1.0, 2.0}, 0), "PhysicsInputs"));
    };
    kernel.register_party(std::move(a));
    InMemoryRecorderSink sink;
    const RunReport report = kernel.run(EndCondition{Seconds(1, 100), {}}, sink);
    REQUIRE(report.outcome == RunOutcome::run_error);
    CHECK(report.fault.value_or(PartyFault("none", SimTime(), "")).party() == "physics");
    CHECK(report.end_time == SimTime(1));
}

TEST_CASE("Physics party passes joint drives through and refuses two drives on one joint in a tick", "[KER-027]") {
    for (const bool clash : {false, true}) {
        auto shared = std::make_shared<CountingAdapter::Shared>();
        std::vector<std::string> log;
        CoSimKernel kernel{RunConfig{}};
        PhysicsPartyConfig config;
        config.inputs = {"a.inputs", "b.inputs"};
        kernel.register_party(std::make_unique<PhysicsParty>(std::make_unique<CountingAdapter>(shared), config));
        auto a = std::make_unique<ScriptedParty>("a", Rate(1000), log);
        a->published_names = {"a.inputs", "b.inputs"};
        a->on_write = [clash](SimTime, CouplingWriter& out) {
            PhysicsInputs first;
            first.joint_drives.push_back(JointDrive{2, 0.5, 20.0, 1e-4, true});
            PhysicsInputs second;
            std::size_t joint = 4;
            if (clash) {
                joint = 2;
            }
            second.joint_drives.push_back(JointDrive{joint, -1.0, 3.0, 0.0, false});
            out.set("a.inputs", make_payload(first, "PhysicsInputs"));
            out.set("b.inputs", make_payload(second, "PhysicsInputs"));
        };
        kernel.register_party(std::move(a));
        InMemoryRecorderSink sink;
        const RunReport report = kernel.run(EndCondition{Seconds(2, 1000), {}}, sink);
        if (clash) {
            REQUIRE(report.outcome == RunOutcome::run_error);
            const PartyFault fault = report.fault.value_or(PartyFault("none", SimTime(), ""));
            CHECK(fault.party() == "physics");
            CHECK(fault.cause().find("second drive on joint 2") != std::string::npos);
            continue;
        }
        REQUIRE(report.outcome == RunOutcome::end_condition_met);
        REQUIRE(shared->received.size() == 2);
        CHECK(shared->received[0].joint_drives.empty());
        const std::vector<JointDrive>& drives = shared->received[1].joint_drives;
        REQUIRE(drives.size() == 2);
        CHECK(drives[0].joint == 2);
        CHECK(drives[0].rest == 0.5);
        CHECK(drives[0].stiffness == 20.0);
        CHECK(drives[0].damping == 1e-4);
        CHECK(drives[0].enabled);
        CHECK(drives[1].joint == 4);
        CHECK(drives[1].rest == -1.0);
        CHECK_FALSE(drives[1].enabled);
        CHECK(shared->received[1].generalized_forces.empty());
    }
}

TEST_CASE("A controller party consuming a physics party's output is a config error", "[KER-020]") {
    auto shared = std::make_shared<CountingAdapter::Shared>();
    std::vector<std::string> log;
    CoSimKernel kernel{RunConfig{}};
    PhysicsPartyConfig config;
    config.inputs = {};
    kernel.register_party(std::make_unique<PhysicsParty>(std::make_unique<CountingAdapter>(shared), config));
    auto controller = std::make_unique<ScriptedParty>("controller", Rate(1000), log);
    controller->party_role = PartyRole::controller;
    controller->consumed_names = {"physics.outputs"};
    kernel.register_party(std::move(controller));
    InMemoryRecorderSink sink;
    CHECK_THROWS_AS(kernel.run(EndCondition{Seconds(1, 100), {}}, sink), KernelConfigError);
}

TEST_CASE("Payload coupling values keep their type and read back typed", "[KER-008]") {
    CouplingTable table;
    table.declare("p.out", "p");
    CHECK_THROWS_AS(table.set("p.out", CouplingValue{PayloadPtr{}}), CouplingError);
    table.set("p.out", make_payload(PhysicsInputs{}, "PhysicsInputs"));
    CHECK_THROWS_AS(table.set("p.out", make_payload(PhysicsOutputs{}, "PhysicsOutputs")), CouplingError);
    CHECK_THROWS_AS(table.set("p.out", CouplingValue{1.0}), CouplingError);
    table.set("p.out", make_payload(force({4.0}, 1), "PhysicsInputs"));
    const std::vector<std::string> consumed{"p.out"};
    const CouplingReader reader(table, consumed);
    CHECK(reader.payload<PhysicsInputs>("p.out").generalized_forces == std::vector<double>{4.0});
    CHECK_THROWS_AS(reader.payload<PhysicsOutputs>("p.out"), CouplingError);
    CHECK_THROWS_AS(reader.real("p.out"), CouplingError);
    table.declare("p.scalar", "p");
    CHECK_THROWS_AS(payload_as<PhysicsInputs>(table.value("p.scalar"), "p.scalar"), CouplingError);
}

// KER-023 scheduling rule: each party steps on its own period boundaries; a slower party's outputs hold between its
// steps; end checks happen at physics boundaries. A 500 Hz controller under 1 kHz physics steps at even ticks only
// and physics sees each controller value for two physics steps.
TEST_CASE("Parties at different rates step on their own boundaries with outputs held between", "[KER-023]") {
    std::vector<std::string> log;
    RunConfig config;
    config.control_rate = Rate(500);
    CoSimKernel kernel{config};
    auto physics = std::make_unique<ScriptedParty>("physics", Rate(1000), log);
    physics->party_role = PartyRole::physics;
    physics->consumed_names = {"controller.command"};
    std::vector<std::int64_t> commands;
    physics->on_step = [&commands](SimTime, Duration, const ScriptedParty::Inputs& inputs) {
        const CouplingValue& value = inputs.at("controller.command");
        if (std::holds_alternative<std::int64_t>(value)) {
            commands.push_back(std::get<std::int64_t>(value));
        } else {
            commands.push_back(-1);
        }
    };
    kernel.register_party(std::move(physics));
    auto controller = std::make_unique<ScriptedParty>("controller", Rate(1000), log);
    controller->party_role = PartyRole::controller;
    controller->published_names = {"controller.command"};
    controller->period_hint = 2;
    controller->on_write = [](SimTime t, CouplingWriter& out) { out.set("controller.command", CouplingValue{t.count()}); };
    kernel.register_party(std::move(controller));
    InMemoryRecorderSink sink;
    std::vector<std::int64_t> stops;
    const RunReport report = kernel.run(EndCondition{Seconds(6, 1000), [&stops](SimTime t) {
                                             stops.push_back(t.count());
                                             return false;
                                         }},
                                        sink);
    REQUIRE(report.outcome == RunOutcome::end_condition_met);
    CHECK(report.base_rate == Rate(1000));
    CHECK(commands == std::vector<std::int64_t>{-1, 0, 0, 2, 2, 4});
    std::vector<std::string> controller_steps;
    for (const std::string& entry : log) {
        if (entry.starts_with("controller@")) {
            controller_steps.push_back(entry);
        }
    }
    CHECK(controller_steps == std::vector<std::string>{"controller@0", "controller@2", "controller@4"});
    CHECK(sink.rows().size() == 6);
    REQUIRE(stops.size() >= 5);
    for (std::size_t i = 1; i < stops.size(); ++i) {
        CHECK(stops[i] - stops[i - 1] == 1);
    }
}

namespace {

struct FaultRun {
    RunReport report;
    std::vector<std::string> calls;
    std::vector<RecordedEvent> events;
    std::vector<std::int64_t> event_rows;
};

FaultRun run_faults(const std::function<void(PhysicsParty&)>& schedule, std::int64_t ticks) {
    auto shared = std::make_shared<CountingAdapter::Shared>();
    PhysicsPartyConfig config;
    config.inputs = {};
    auto party = std::make_unique<PhysicsParty>(std::make_unique<CountingAdapter>(shared), config);
    schedule(*party);
    CoSimKernel kernel{RunConfig{}};
    kernel.register_party(std::move(party));
    InMemoryRecorderSink sink;
    FaultRun result;
    result.report = kernel.run(EndCondition{Seconds(ticks, 1000), {}}, sink);
    result.calls = shared->calls;
    for (const TickRecord& row : sink.rows()) {
        for (const RecordedEvent& event : row.events) {
            result.events.push_back(event);
            result.event_rows.push_back(row.time.count());
        }
    }
    return result;
}

std::string event_text(const RecordedEvent& event) {
    return std::to_string(event.time.count()) + " " + event.source + " " + event.kind + " " + event.detail;
}

std::vector<std::string> event_texts(const FaultRun& run) {
    std::vector<std::string> texts;
    texts.reserve(run.events.size());
    for (const RecordedEvent& event : run.events) {
        texts.push_back(event_text(event));
    }
    return texts;
}

std::string cause(const RunReport& report) {
    return report.fault.value_or(PartyFault("none", SimTime(), "")).cause();
}

}  // namespace

// ===== Scheduled faults ===============================================================================================

TEST_CASE("Scheduled slip and payload changes reach the adapter before the step of their tick", "[KER-023][KER-009]") {
    const FaultRun run = run_faults(
        [](PhysicsParty& party) {
            party.schedule_slip(1, SimTime(2), SimTime(5), 0.5);
            party.schedule_payload_change(0, SimTime(3), PayloadMass{12.5});
            party.schedule_payload_change(0, SimTime(4), PayloadCarrier{});
            party.schedule_payload_change(1, SimTime(4), PayloadCarrier{2});
        },
        8);
    REQUIRE(run.report.outcome == RunOutcome::end_condition_met);
    // At 1 kHz the step at tick t follows t completed steps, so "@t" means "before the step at t".
    CHECK(run.calls == std::vector<std::string>{"slip 1 0.500000 @2", "mass 0 12.500000 @3", "attach 0 world @4",
                                                "attach 1 2 @4", "slip 1 0.000000 @5"});
    CHECK(event_texts(run) == std::vector<std::string>{"2 kernel fault slip start transmission 1 fraction 0.500000",
                                                       "3 kernel fault payload 0 mass 12.500000",
                                                       "4 kernel fault payload 0 released",
                                                       "4 kernel fault payload 1 carrier 2",
                                                       "5 kernel fault slip end transmission 1"});
    // Each event rides on the row stamped at the end of its step, once.
    CHECK(run.event_rows == std::vector<std::int64_t>{3, 4, 5, 5, 6});
}

TEST_CASE("Adjacent slip windows on one transmission make one adapter call at the boundary", "[KER-023]") {
    const FaultRun run = run_faults(
        [](PhysicsParty& party) {
            party.schedule_slip(0, SimTime(3), SimTime(5), 0.25);
            party.schedule_slip(0, SimTime(1), SimTime(3), 0.5);
            party.schedule_slip(1, SimTime(1), SimTime(4), 1.0);
        },
        8);
    REQUIRE(run.report.outcome == RunOutcome::end_condition_met);
    CHECK(run.calls == std::vector<std::string>{"slip 0 0.500000 @1", "slip 1 1.000000 @1", "slip 0 0.250000 @3",
                                                "slip 1 0.000000 @4", "slip 0 0.000000 @5"});
    CHECK(event_texts(run) == std::vector<std::string>{
                                  "1 kernel fault slip start transmission 0 fraction 0.500000",
                                  "1 kernel fault slip start transmission 1 fraction 1.000000",
                                  "3 kernel fault slip end transmission 0",
                                  "3 kernel fault slip start transmission 0 fraction 0.250000",
                                  "4 kernel fault slip end transmission 1",
                                  "5 kernel fault slip end transmission 0"});
}

TEST_CASE("A slip window still open at the end of the run is never cleared", "[KER-023]") {
    const FaultRun run = run_faults([](PhysicsParty& party) { party.schedule_slip(0, SimTime(0), SimTime(100), 0.1); },
                                    3);
    REQUIRE(run.report.outcome == RunOutcome::end_condition_met);
    CHECK(run.calls == std::vector<std::string>{"slip 0 0.100000 @0"});
    CHECK(run.event_rows == std::vector<std::int64_t>{1});
}

TEST_CASE("Malformed fault schedules are config errors at the call", "[KER-023]") {
    auto shared = std::make_shared<CountingAdapter::Shared>();
    PhysicsParty party(std::make_unique<CountingAdapter>(shared), PhysicsPartyConfig{});
    CHECK_THROWS_AS(party.schedule_slip(0, SimTime(5), SimTime(5), 0.5), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_slip(0, SimTime(5), SimTime(4), 0.5), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_slip(0, SimTime(-1), SimTime(4), 0.5), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_slip(0, SimTime(0), SimTime(4), 0.0), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_slip(0, SimTime(0), SimTime(4), -0.5), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_slip(0, SimTime(0), SimTime(4), 1.5), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_slip(0, SimTime(0), SimTime(4), std::numeric_limits<double>::quiet_NaN()),
                    KernelConfigError);
    CHECK_NOTHROW(party.schedule_slip(0, SimTime(0), SimTime(4), 1.0));
    CHECK_THROWS_AS(party.schedule_payload_change(0, SimTime(-1), PayloadMass{1.0}), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_payload_change(0, SimTime(-1), PayloadCarrier{}), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_payload_change(0, SimTime(1), PayloadMass{0.0}), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_payload_change(0, SimTime(1), PayloadMass{-2.0}), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_payload_change(0, SimTime(1), PayloadMass{std::numeric_limits<double>::infinity()}),
                    KernelConfigError);
    party.start(Rate(1000));
    CHECK_THROWS_AS(party.schedule_slip(0, SimTime(10), SimTime(14), 0.5), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_payload_change(0, SimTime(10), PayloadMass{1.0}), KernelConfigError);
    CHECK_THROWS_AS(party.schedule_payload_change(0, SimTime(10), PayloadCarrier{1}), KernelConfigError);
}

TEST_CASE("Conflicting fault schedules end the run at start", "[KER-023]") {
    const FaultRun overlap = run_faults(
        [](PhysicsParty& party) {
            party.schedule_slip(2, SimTime(4), SimTime(9), 0.5);
            party.schedule_slip(2, SimTime(1), SimTime(5), 0.5);
        },
        3);
    CHECK(overlap.report.outcome == RunOutcome::run_error);
    CHECK(cause(overlap.report).find("slip windows [1, 5) and [4, 9) on transmission 2 overlap") != std::string::npos);
    CHECK(overlap.calls.empty());

    const FaultRun twice = run_faults(
        [](PhysicsParty& party) {
            party.schedule_payload_change(3, SimTime(2), PayloadMass{1.0});
            party.schedule_payload_change(3, SimTime(2), PayloadCarrier{});
        },
        3);
    CHECK(twice.report.outcome == RunOutcome::run_error);
    CHECK(cause(twice.report).find("two changes to payload 3 at tick 2") != std::string::npos);

    const FaultRun separate = run_faults(
        [](PhysicsParty& party) {
            party.schedule_slip(0, SimTime(1), SimTime(5), 0.5);
            party.schedule_slip(1, SimTime(1), SimTime(5), 0.5);
            party.schedule_payload_change(0, SimTime(2), PayloadMass{1.0});
            party.schedule_payload_change(1, SimTime(2), PayloadMass{1.0});
            party.schedule_payload_change(0, SimTime(3), PayloadMass{2.0});
        },
        6);
    CHECK(separate.report.outcome == RunOutcome::end_condition_met);
    CHECK(separate.calls.size() == 7);
}

TEST_CASE("A slip window between two physics steps is a run error", "[KER-023]") {
    auto shared = std::make_shared<CountingAdapter::Shared>();
    RunConfig config;
    config.physics_rate = Rate(500);
    PhysicsPartyConfig party_config;
    party_config.rate = Rate(500);
    party_config.inputs = {};
    auto party = std::make_unique<PhysicsParty>(std::make_unique<CountingAdapter>(shared), party_config);
    party->schedule_slip(0, SimTime(3), SimTime(4), 0.5);
    CoSimKernel kernel{config};
    kernel.register_party(std::move(party));
    std::vector<std::string> log;
    kernel.register_party(std::make_unique<ScriptedParty>("fast", Rate(1000), log));
    InMemoryRecorderSink sink;
    const RunReport report = kernel.run(EndCondition{Seconds(10, 1000), {}}, sink);
    CHECK(report.outcome == RunOutcome::run_error);
    CHECK(cause(report).find("slip window [3, 4) holds none of its steps") != std::string::npos);
    // Refused at start(), before any step (KER-023).
    CHECK(report.fault.value_or(PartyFault("none", SimTime(-1), "")).time() == SimTime(0));
    CHECK(sink.rows().empty());
    CHECK(shared->calls.empty());
}

TEST_CASE("A slip window ending exactly on the next physics step still holds one step", "[KER-023]") {
    auto shared = std::make_shared<CountingAdapter::Shared>();
    RunConfig config;
    config.physics_rate = Rate(500);
    PhysicsPartyConfig party_config;
    party_config.rate = Rate(500);
    party_config.inputs = {};
    auto party = std::make_unique<PhysicsParty>(std::make_unique<CountingAdapter>(shared), party_config);
    party->schedule_slip(0, SimTime(1), SimTime(3), 0.5);
    CoSimKernel kernel{config};
    kernel.register_party(std::move(party));
    std::vector<std::string> log;
    kernel.register_party(std::make_unique<ScriptedParty>("fast", Rate(1000), log));
    InMemoryRecorderSink sink;
    const RunReport report = kernel.run(EndCondition{Seconds(10, 1000), {}}, sink);
    REQUIRE(report.outcome == RunOutcome::end_condition_met);
    // Steps at even ticks: the window [1, 3) holds the step at 2 and ends before the step at 4.
    CHECK(shared->calls == std::vector<std::string>{"slip 0 0.500000 @1", "slip 0 0.000000 @2"});
}

TEST_CASE("Slip windows checked at a nominal rate the kernel overrides fault the run at the first step",
          "[KER-023]") {
    auto shared = std::make_shared<CountingAdapter::Shared>();
    RunConfig config;
    config.physics_rate = Rate(500);
    config.party_rates.emplace("physics", Rate(500));
    PhysicsPartyConfig party_config;
    party_config.inputs = {};
    auto party = std::make_unique<PhysicsParty>(std::make_unique<CountingAdapter>(shared), party_config);
    party->schedule_slip(0, SimTime(2), SimTime(4), 0.5);
    CoSimKernel kernel{config};
    kernel.register_party(std::move(party));
    std::vector<std::string> log;
    kernel.register_party(std::make_unique<ScriptedParty>("fast", Rate(1000), log));
    InMemoryRecorderSink sink;
    const RunReport report = kernel.run(EndCondition{Seconds(10, 1000), {}}, sink);
    CHECK(report.outcome == RunOutcome::run_error);
    CHECK(cause(report).find("steps every 2 ticks") != std::string::npos);
    CHECK(shared->calls.empty());
}
