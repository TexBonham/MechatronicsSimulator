#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
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
    void set_payload_mass(std::size_t /*payload*/, double /*mass*/) override {}
    void set_payload_attachment(std::size_t /*payload*/, std::size_t /*body*/) override {}

private:
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
