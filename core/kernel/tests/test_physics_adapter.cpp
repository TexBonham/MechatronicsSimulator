#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <string>
#include <type_traits>

#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/time.hpp"

using mechsim::kernel::AdapterError;
using mechsim::kernel::BodyWrench;
using mechsim::kernel::NullPhysicsAdapter;
using mechsim::kernel::PhysicsAdapter;
using mechsim::kernel::PhysicsInputs;
using mechsim::kernel::PreparedModel;
using mechsim::kernel::Rate;

static_assert(std::is_abstract_v<PhysicsAdapter>);
static_assert(!std::is_copy_constructible_v<PhysicsAdapter>);

namespace {

PreparedModel model(const std::string& version) {
    return PreparedModel{R"({"bodies": [], "joints": []})", "", version};
}

std::unique_ptr<PhysicsAdapter> loaded_adapter() {
    auto adapter = std::make_unique<NullPhysicsAdapter>();
    adapter->load(model("1.0.0"), Rate(1'000));
    return adapter;
}

}  // namespace

TEST_CASE("The null adapter loads a prepared model once and keeps its version and rate", "[PHY-001]") {
    NullPhysicsAdapter adapter;
    CHECK_FALSE(adapter.loaded());
    adapter.load(PreparedModel{"{}", "<mujoco/>", "1.2.3"}, Rate(500));
    CHECK(adapter.loaded());
    CHECK(adapter.design_version() == "1.2.3");
    CHECK(adapter.health().physics_rate == Rate(500));
}

TEST_CASE("The null adapter declares no supported element kinds", "[PHY-019]") {
    const NullPhysicsAdapter adapter;
    CHECK(adapter.supported_elements().empty());
}

TEST_CASE("A second load without reset is rejected; reset allows a new load", "[PHY-001]") {
    NullPhysicsAdapter adapter;
    adapter.load(model("1.0.0"), Rate(1'000));
    CHECK_THROWS_AS(adapter.load(model("1.0.0"), Rate(1'000)), AdapterError);
    adapter.step();
    adapter.reset();
    CHECK_FALSE(adapter.loaded());
    CHECK(adapter.outputs().steps == 0);
    adapter.load(model("2.0.0"), Rate(1'000));
    CHECK(adapter.design_version() == "2.0.0");
}

TEST_CASE("Each step advances one physics step and publishes zero-sized outputs", "[PHY-002]") {
    const auto adapter = loaded_adapter();
    CHECK(adapter->outputs().steps == 0);
    adapter->step();
    adapter->step();
    const auto& out = adapter->outputs();
    CHECK(out.steps == 2);
    CHECK(out.bodies.empty());
    CHECK(out.joints.empty());
    CHECK(out.constraints.empty());
    CHECK(out.contacts.empty());
    CHECK(out.members.empty());
    CHECK(out.ropes.empty());
    CHECK(out.transmissions.empty());
    const auto health = adapter->health();
    CHECK(health.steps == 2);
    CHECK(health.physics_rate == Rate(1'000));
    CHECK(health.converged);
    CHECK(health.iterations == 0);
    CHECK(health.max_constraint_drift == 0.0);
}

TEST_CASE("Stepping before load is an error", "[PHY-002]") {
    NullPhysicsAdapter adapter;
    CHECK_THROWS_AS(adapter.step(), AdapterError);
}

TEST_CASE("Inputs are accepted before a step; inputs naming absent elements are rejected", "[PHY-005]") {
    const auto adapter = loaded_adapter();
    adapter->set_inputs(PhysicsInputs{});
    PhysicsInputs forces;
    forces.generalized_forces = {1.0};
    CHECK_THROWS_AS(adapter->set_inputs(forces), AdapterError);
    PhysicsInputs wrenches;
    wrenches.body_wrenches.push_back(BodyWrench{});
    CHECK_THROWS_AS(adapter->set_inputs(wrenches), AdapterError);
    NullPhysicsAdapter unloaded;
    CHECK_THROWS_AS(unloaded.set_inputs(PhysicsInputs{}), AdapterError);
}

TEST_CASE("Mid-run changes exist on the interface and reject elements not in the model", "[PHY-008]") {
    const auto adapter = loaded_adapter();
    CHECK_THROWS_AS(adapter->set_spring_offset(0, 0.01), AdapterError);
    CHECK_THROWS_AS(adapter->set_spring_stiffness(0, 1.0), AdapterError);
    CHECK_THROWS_AS(adapter->disable_constraint(0), AdapterError);
    CHECK_THROWS_AS(adapter->release_load(0), AdapterError);
    CHECK_THROWS_AS(adapter->set_payload_mass(0, 1.0), AdapterError);
    CHECK_THROWS_AS(adapter->set_payload_attachment(0, 1), AdapterError);
    NullPhysicsAdapter unloaded;
    CHECK_THROWS_AS(unloaded.disable_constraint(0), AdapterError);
}

TEST_CASE("An adapter error carries the step it was raised at", "[PHY-009]") {
    const auto adapter = loaded_adapter();
    adapter->step();
    adapter->step();
    adapter->step();
    try {
        adapter->release_load(4);
        FAIL("release_load on an empty model must throw");
    } catch (const AdapterError& error) {
        CHECK(error.step() == 3);
        CHECK(error.cause().find("load 4") != std::string::npos);
    }
}
