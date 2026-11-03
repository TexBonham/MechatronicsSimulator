#include <array>
#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/signal_component.hpp"
#include "mechsim/signals/signal_name.hpp"

using mechsim::kernel::SimTime;
using mechsim::signals::FailureInjection;
using mechsim::signals::FailureMode;
using mechsim::signals::is_dotted_signal_name;
using mechsim::signals::NullSignalComponent;
using mechsim::signals::PhysicalState;
using mechsim::signals::PythonSlowPathComponent;
using mechsim::signals::signal_instance;
using mechsim::signals::SignalComponent;
using mechsim::signals::SignalNameError;
using mechsim::signals::SignalTable;

static_assert(std::is_abstract_v<SignalComponent>);
static_assert(std::is_abstract_v<PythonSlowPathComponent>);
static_assert(std::is_base_of_v<SignalComponent, PythonSlowPathComponent>);

TEST_CASE("Dotted signal names are instance.signal with two identifier segments", "[SIG-023]") {
    CHECK(is_dotted_signal_name("x_limit.level"));
    CHECK(is_dotted_signal_name("enc1.count"));
    CHECK(is_dotted_signal_name("_a._b2"));
    CHECK_FALSE(is_dotted_signal_name(""));
    CHECK_FALSE(is_dotted_signal_name("level"));
    CHECK_FALSE(is_dotted_signal_name(".level"));
    CHECK_FALSE(is_dotted_signal_name("x_limit."));
    CHECK_FALSE(is_dotted_signal_name("a.b.c"));
    CHECK_FALSE(is_dotted_signal_name("1enc.count"));
    CHECK_FALSE(is_dotted_signal_name("enc.2count"));
    CHECK_FALSE(is_dotted_signal_name("enc-1.count"));
    CHECK_FALSE(is_dotted_signal_name("enc .count"));
    CHECK(signal_instance("enc1.count") == "enc1");
    CHECK(signal_instance("count").empty());
}

TEST_CASE("A component rejects names that are not dotted or belong to another instance", "[SIG-023]") {
    CHECK_THROWS_AS(NullSignalComponent("enc1", {"count"}, {}), SignalNameError);
    CHECK_THROWS_AS(NullSignalComponent("enc1", {}, {"enc2.count"}), SignalNameError);
    CHECK_THROWS_AS(NullSignalComponent("enc.1", {}, {}), SignalNameError);
    CHECK_THROWS_AS(NullSignalComponent("", {}, {}), SignalNameError);
    const NullSignalComponent ok("enc1", {"enc1.reset"}, {"enc1.count"});
    CHECK(ok.instance_name() == "enc1");
    REQUIRE(ok.outputs().size() == 1);
    CHECK(ok.outputs()[0] == "enc1.count");
}

TEST_CASE("Signal names are unique within a table", "[SIG-023]") {
    SignalTable table;
    const NullSignalComponent first("enc1", {"enc1.reset"}, {"enc1.count"});
    const NullSignalComponent again("enc1", {}, {"enc1.count"});
    const NullSignalComponent other("enc2", {}, {"enc2.count"});
    const NullSignalComponent self_duplicate("lim", {"lim.level"}, {"lim.level"});
    table.add(first);
    CHECK(table.size() == 2);
    CHECK_THROWS_AS(table.add(again), SignalNameError);
    CHECK_THROWS_AS(table.add(self_duplicate), SignalNameError);
    CHECK(table.size() == 2);
    CHECK_FALSE(table.contains("lim.level"));
    table.add(other);
    CHECK(table.size() == 3);
    CHECK(table.contains("enc2.count"));
}

TEST_CASE("The null component samples, latches and outputs zeros", "[SIG-030]") {
    NullSignalComponent component("drv", {"drv.step", "drv.dir"}, {"drv.torque"});
    PhysicalState state;
    state.steps = 4;
    component.sample(state);
    const std::array<double, 2> pins{3.0, 1.0};
    component.set_inputs(pins);
    component.latch_outputs();
    CHECK(component.samples() == 1);
    CHECK(component.latches() == 1);
    CHECK(component.last_sample_step() == 4);
    REQUIRE(component.output_values().size() == 1);
    CHECK(component.output_values()[0] == 0.0);
    const std::array<double, 1> wrong{1.0};
    CHECK_THROWS_AS(component.set_inputs(wrong), std::invalid_argument);
}

TEST_CASE("Failure injections carry a mode and a time window and are validated",
          "[SIG-006][SIG-007][SIG-008][SIG-036]") {
    NullSignalComponent component("lim", {}, {"lim.level"});
    const SimTime start(10);
    const SimTime end(20);
    for (const FailureMode mode :
         {FailureMode::stuck, FailureMode::noisy, FailureMode::dropped, FailureMode::forced_level}) {
        component.inject(FailureInjection{mode, start, end, 0.5, 1.0});
    }
    CHECK(component.injections().size() == 4);
    CHECK(component.injections()[3].mode == FailureMode::forced_level);
    CHECK_THROWS_AS(component.inject(FailureInjection{FailureMode::stuck, end, start, 0.0, 0.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(component.inject(FailureInjection{FailureMode::stuck, start, start, 0.0, 0.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(component.inject(FailureInjection{FailureMode::noisy, start, end, -1.0, 0.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(component.inject(FailureInjection{FailureMode::forced_level, start, end, 0.0, std::numeric_limits<double>::infinity()}),
                    std::invalid_argument);
    CHECK(component.injections().size() == 4);
}
