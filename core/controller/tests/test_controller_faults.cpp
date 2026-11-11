// Scheduled controller faults through the kernel: freeze (TST-018, CTL-011, KER-021), slow-down (CTL-021), output
// latency (CTL-020) and brownout from the rail (TST-017, SIG-022, CTL-013). The firmware is the mirror test
// application (fixtures/mirror_app.cpp): state_code counts ticks since init, fault_code counts inits seen by this
// copy of the library, digital_out[0] = digital_in[31], uart_tx_count = uart_rx_count, 16 events every tick. All
// parties run at 1 kHz, so the base tick equals the control tick and step t is the t-th tick (0-based).
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/controller/controller_party.hpp"
#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/controller/native_controller_runtime.hpp"
#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/cosim_kernel.hpp"
#include "mechsim/kernel/kernel.hpp"
#include "mechsim/kernel/memory_recorder_sink.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/pins/pins.hpp"
#include "native_test_support.hpp"

using mechsim::controller::ControllerParty;
using mechsim::controller::ControllerPartyConfig;
using mechsim::controller::FirmwareImage;
using mechsim::controller::NativeControllerRuntime;
using mechsim::controller::NullControllerRuntime;
using mechsim::controller::PinInputs;
using mechsim::controller::PinOutputs;
using mechsim::kernel::CoSimKernel;
using mechsim::kernel::CouplingReader;
using mechsim::kernel::CouplingWriter;
using mechsim::kernel::Duration;
using mechsim::kernel::EndCondition;
using mechsim::kernel::InMemoryRecorderSink;
using mechsim::kernel::KernelConfigError;
using mechsim::kernel::Party;
using mechsim::kernel::PartyFault;
using mechsim::kernel::PartyRole;
using mechsim::kernel::Rate;
using mechsim::kernel::RecordedEvent;
using mechsim::kernel::RunConfig;
using mechsim::kernel::RunOutcome;
using mechsim::kernel::RunReport;
using mechsim::kernel::Seconds;
using mechsim::kernel::SimTime;
using mechsim::pins::PowerRail;

namespace {

// A party at 1 kHz with a role; publishes what `write` sets and logs what it reads.
class TestParty final : public Party {
public:
    TestParty(std::string name, PartyRole role) : name_(std::move(name)), role_(role) {}

    std::vector<std::string> published_names;
    std::vector<std::string> consumed_names;
    std::function<void(SimTime, CouplingWriter&)> write;
    std::function<void(const CouplingReader&)> read;
    std::int64_t steps = 0;

    [[nodiscard]] std::string_view name() const override { return name_; }
    [[nodiscard]] Rate nominal_rate() const override { return Rate(1'000); }
    [[nodiscard]] PartyRole role() const override { return role_; }
    [[nodiscard]] std::span<const std::string> published() const override { return published_names; }
    [[nodiscard]] std::span<const std::string> consumed() const override { return consumed_names; }
    void start(const Rate& /*base_rate*/) override {}
    void read_inputs(const CouplingReader& inputs) override {
        if (read) {
            read(inputs);
        }
    }
    void step(SimTime t, Duration /*dt*/) override {
        now_ = t;
        ++steps;
    }
    void write_outputs(CouplingWriter& outputs) override {
        if (write) {
            write(now_, outputs);
        }
    }
    void on_fault(const PartyFault& /*fault*/) noexcept override {}

private:
    std::string name_;
    PartyRole role_;
    SimTime now_;
};

// digital_in[31] high (digital_out[0] follows), three UART bytes and ADC values, so held levels and empty buffers
// differ from a fresh tick.
PinInputs busy_inputs() {
    PinInputs inputs;
    inputs.digital_in.at(31) = 1;
    inputs.uart_rx.at(0) = 7;
    inputs.uart_rx.at(1) = 8;
    inputs.uart_rx.at(2) = 9;
    inputs.uart_rx_count = 3;
    inputs.adc_sample.at(0) = 100;
    return inputs;
}

struct Harness {
    std::shared_ptr<std::vector<PinOutputs>> seen = std::make_shared<std::vector<PinOutputs>>();
    std::int64_t physics_steps = 0;
    std::int64_t signal_steps = 0;
    std::vector<std::string> controller_states;
    std::vector<RecordedEvent> events;
    RunReport report;
    // Runtime counters read before the kernel, which owns the runtime, is destroyed.
    std::size_t ticks_run = 0;
    std::size_t inits = 0;
    bool frozen = false;
};

using InputsAt = std::function<PinInputs(SimTime)>;
using VoltageAt = std::function<double(SimTime)>;

// Physics counter, signal source (pin inputs and, with a voltage function, the "logic" rail), the controller party,
// and a probe logging the published pin outputs each step; runs `ticks` steps.
Harness run(std::unique_ptr<ControllerParty> controller, std::int64_t ticks, const InputsAt& inputs_at,
            const VoltageAt& voltage_at = {}, const NativeControllerRuntime* runtime = nullptr) {
    Harness harness;
    CoSimKernel kernel{RunConfig{}};
    auto physics = std::make_unique<TestParty>("physics", PartyRole::physics);
    TestParty* physics_view = physics.get();
    kernel.register_party(std::move(physics));
    auto source = std::make_unique<TestParty>("signals", PartyRole::sensors);
    TestParty* source_view = source.get();
    source->published_names = {"signals.pin_inputs"};
    if (voltage_at) {
        source->published_names.emplace_back("signals.rails");
    }
    source->write = [inputs_at, voltage_at](SimTime t, CouplingWriter& out) {
        out.set("signals.pin_inputs", mechsim::kernel::make_payload(inputs_at(t), mechsim::pins::pin_inputs_kind));
        if (voltage_at) {
            const std::vector<PowerRail> rails{PowerRail{"logic", voltage_at(t), true}};
            out.set("signals.rails", mechsim::kernel::make_payload(rails, mechsim::pins::power_rails_kind));
        }
    };
    kernel.register_party(std::move(source));
    kernel.register_party(std::move(controller));
    auto probe = std::make_unique<TestParty>("probe", PartyRole::other);
    probe->consumed_names = {"controller.pin_outputs"};
    probe->read = [seen = harness.seen](const CouplingReader& in) {
        seen->push_back(in.payload<PinOutputs>("controller.pin_outputs"));
    };
    kernel.register_party(std::move(probe));
    InMemoryRecorderSink sink;
    harness.report = kernel.run(EndCondition{Seconds(ticks, 1'000), {}}, sink);
    harness.physics_steps = physics_view->steps;
    harness.signal_steps = source_view->steps;
    if (runtime != nullptr) {
        harness.ticks_run = runtime->ticks_run();
        harness.inits = runtime->inits();
        harness.frozen = runtime->frozen();
    }
    for (const mechsim::kernel::TickRecord& row : sink.rows()) {
        harness.controller_states.push_back(row.controller_state);
        harness.events.insert(harness.events.end(), row.events.begin(), row.events.end());
    }
    return harness;
}

PinInputs constant_inputs(SimTime /*t*/) {
    return busy_inputs();
}

std::unique_ptr<NativeControllerRuntime> mirror_runtime(const std::string& work, double threshold_v) {
    mechsim::controller::NativeRuntimeConfig config = native_test::config(work);
    config.brownout_threshold_v = threshold_v;
    auto runtime = std::make_unique<NativeControllerRuntime>(config);
    runtime->load(FirmwareImage{std::string(native_test::mirror_app), 1'000});
    return runtime;
}

// Without a rail the caller resets before the run.
std::unique_ptr<ControllerParty> mirror_party(const std::string& work, ControllerPartyConfig config = {},
                                              NativeControllerRuntime** view = nullptr) {
    auto runtime = mirror_runtime(work, native_test::brownout_threshold_v);
    runtime->reset(native_test::logic_rail);
    if (view != nullptr) {
        *view = runtime.get();
    }
    return std::make_unique<ControllerParty>(std::move(runtime), std::move(config));
}

bool fault_event(const RecordedEvent& event, std::int64_t time, const std::string& detail) {
    return event.time.count() == time && event.source == "kernel" && event.kind == "fault" && event.detail == detail;
}

std::string fault_cause(const RunReport& report) {
    return report.fault.value_or(PartyFault("none", SimTime(), "")).cause();
}

}  // namespace

TEST_CASE("A scheduled freeze stops the firmware tick while physics and signals keep stepping",
          "[KER-021][CTL-011][TST-018]") {
    NativeControllerRuntime* runtime = nullptr;
    auto party = mirror_party("faults-freeze", {}, &runtime);
    party->schedule_freeze(SimTime(10), SimTime(60));
    const Harness h = run(std::move(party), 100, constant_inputs, {}, runtime);
    REQUIRE(h.report.outcome == RunOutcome::end_condition_met);

    // KER-021: the 1 kHz physics and signal parties step through the 50-tick freeze; only the firmware stops.
    CHECK(h.physics_steps == 100);
    CHECK(h.signal_steps == 100);
    CHECK(h.ticks_run == 50);
    REQUIRE(h.seen->size() == 100);
    // Step 9 is the tenth tick since init; steps 10 to 59 hold its levels with empty buffers; step 60 runs tick 11.
    CHECK(h.seen->at(9).state_code == 10);
    CHECK(h.seen->at(9).uart_tx_count == 3);
    CHECK(h.seen->at(9).event_count == 16);
    for (std::size_t t = 10; t < 60; ++t) {
        const PinOutputs& out = h.seen->at(t);
        CHECK(out.state_code == 10);
        CHECK(out.digital_out.at(0) == 1);
        CHECK(out.uart_tx_count == 0);
        CHECK(out.event_count == 0);
    }
    CHECK(h.seen->at(60).state_code == 11);
    CHECK(h.seen->at(60).uart_tx_count == 3);
    CHECK(h.seen->at(99).state_code == 50);
    CHECK_FALSE(h.frozen);

    // Start and end recorded on their ticks (rows are stamped at the step's end, the event at the step).
    REQUIRE(h.events.size() == 2);
    CHECK(fault_event(h.events[0], 10, "freeze start controller"));
    CHECK(fault_event(h.events[1], 60, "freeze end controller"));
}

TEST_CASE("Adjacent freeze windows keep the controller frozen across the boundary", "[KER-021][CTL-011]") {
    NativeControllerRuntime* runtime = nullptr;
    auto party = mirror_party("faults-freeze-adjacent", {}, &runtime);
    party->schedule_freeze(SimTime(20), SimTime(30));
    party->schedule_freeze(SimTime(5), SimTime(20));
    const Harness h = run(std::move(party), 40, constant_inputs, {}, runtime);
    REQUIRE(h.report.outcome == RunOutcome::end_condition_met);
    CHECK(h.ticks_run == 15);
    CHECK(h.seen->at(29).state_code == 5);
    CHECK(h.seen->at(30).state_code == 6);
    REQUIRE(h.events.size() == 4);
    CHECK(fault_event(h.events[0], 5, "freeze start controller"));
    CHECK(fault_event(h.events[1], 20, "freeze end controller"));
    CHECK(fault_event(h.events[2], 20, "freeze start controller"));
    CHECK(fault_event(h.events[3], 30, "freeze end controller"));
}

TEST_CASE("A freeze window running past the end of the run leaves the controller frozen", "[KER-021][CTL-011]") {
    NativeControllerRuntime* runtime = nullptr;
    auto party = mirror_party("faults-freeze-open", {}, &runtime);
    party->schedule_freeze(SimTime(3), SimTime(1'000));
    const Harness h = run(std::move(party), 10, constant_inputs, {}, runtime);
    REQUIRE(h.report.outcome == RunOutcome::end_condition_met);
    CHECK(h.ticks_run == 3);
    CHECK(h.frozen);
    REQUIRE(h.events.size() == 1);
    CHECK(fault_event(h.events[0], 3, "freeze start controller"));
}

TEST_CASE("A slow-down window ticks the firmware once per factor control periods", "[CTL-021]") {
    NativeControllerRuntime* runtime = nullptr;
    auto party = mirror_party("faults-slowdown", {}, &runtime);
    party->schedule_slowdown(SimTime(10), SimTime(22), 3);
    const Harness h = run(std::move(party), 30, constant_inputs, {}, runtime);
    REQUIRE(h.report.outcome == RunOutcome::end_condition_met);
    // 10 ticks before, ticks at 10, 13, 16, 19 inside the 12-step window, 8 ticks from 22 to 29.
    CHECK(h.ticks_run == 22);
    CHECK(h.physics_steps == 30);
    const std::vector<std::uint8_t> codes{10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15};
    for (std::size_t i = 0; i < codes.size(); ++i) {
        const std::size_t t = 9 + i;
        CHECK(h.seen->at(t).state_code == codes[i]);
        CHECK(h.seen->at(t).digital_out.at(0) == 1);
        bool ticked = t < 10 || t >= 22 || (t - 10) % 3 == 0;
        if (ticked) {
            CHECK(h.seen->at(t).uart_tx_count == 3);
            CHECK(h.seen->at(t).event_count == 16);
        } else {
            CHECK(h.seen->at(t).uart_tx_count == 0);
            CHECK(h.seen->at(t).event_count == 0);
        }
    }
    REQUIRE(h.events.size() == 2);
    CHECK(fault_event(h.events[0], 10, "slowdown start controller factor 3"));
    CHECK(fault_event(h.events[1], 22, "slowdown end controller"));
}

TEST_CASE("A slow-down window shorter than its factor ticks once", "[CTL-021]") {
    NativeControllerRuntime* runtime = nullptr;
    auto party = mirror_party("faults-slowdown-short", {}, &runtime);
    party->schedule_slowdown(SimTime(4), SimTime(6), 5);
    const Harness h = run(std::move(party), 8, constant_inputs, {}, runtime);
    REQUIRE(h.report.outcome == RunOutcome::end_condition_met);
    CHECK(h.ticks_run == 7);
    CHECK(h.seen->at(4).state_code == 5);
    CHECK(h.seen->at(5).state_code == 5);
    CHECK(h.seen->at(6).state_code == 6);
}

TEST_CASE("Fault schedules are refused when malformed, overlapping or late", "[CTL-021][CTL-011][KER-021]") {
    auto party = std::make_unique<ControllerParty>(std::make_unique<NullControllerRuntime>(Rate(1'000)),
                                                   ControllerPartyConfig{});
    CHECK_THROWS_AS(party->schedule_slowdown(SimTime(0), SimTime(10), 1), KernelConfigError);
    CHECK_THROWS_AS(party->schedule_slowdown(SimTime(0), SimTime(10), 0), KernelConfigError);
    CHECK_THROWS_AS(party->schedule_slowdown(SimTime(0), SimTime(10), -2), KernelConfigError);
    CHECK_THROWS_AS(party->schedule_freeze(SimTime(5), SimTime(5)), KernelConfigError);
    CHECK_THROWS_AS(party->schedule_freeze(SimTime(5), SimTime(4)), KernelConfigError);
    CHECK_THROWS_AS(party->schedule_freeze(SimTime(-1), SimTime(4)), KernelConfigError);
    CHECK_THROWS_AS(party->schedule_slowdown(SimTime(-1), SimTime(4), 2), KernelConfigError);
    CHECK_NOTHROW(party->schedule_slowdown(SimTime(0), SimTime(10), 2));
    party->start(Rate(1'000));
    CHECK_THROWS_AS(party->schedule_freeze(SimTime(20), SimTime(30)), KernelConfigError);
    CHECK_THROWS_AS(party->schedule_slowdown(SimTime(20), SimTime(30), 2), KernelConfigError);

    const auto overlapping = [](const std::function<void(ControllerParty&)>& schedule) {
        auto runtime = std::make_unique<NullControllerRuntime>(Rate(1'000));
        runtime->load(FirmwareImage{"null", 1'000});
        runtime->reset(native_test::logic_rail);
        auto overlapped = std::make_unique<ControllerParty>(std::move(runtime), ControllerPartyConfig{});
        schedule(*overlapped);
        return run(std::move(overlapped), 5, constant_inputs);
    };
    const Harness freezes = overlapping([](ControllerParty& p) {
        p.schedule_freeze(SimTime(0), SimTime(10));
        p.schedule_freeze(SimTime(9), SimTime(12));
    });
    CHECK(freezes.report.outcome == RunOutcome::run_error);
    CHECK(fault_cause(freezes.report).find("freeze windows [0, 10) and [9, 12) overlap") != std::string::npos);
    const Harness slowdowns = overlapping([](ControllerParty& p) {
        p.schedule_slowdown(SimTime(3), SimTime(4), 2);
        p.schedule_slowdown(SimTime(0), SimTime(10), 2);
    });
    CHECK(slowdowns.report.outcome == RunOutcome::run_error);
    CHECK(fault_cause(slowdowns.report).find("slow-down windows [0, 10) and [3, 4) overlap") != std::string::npos);
    // A freeze inside a slow-down is two kinds, not an overlap.
    const Harness mixed = overlapping([](ControllerParty& p) {
        p.schedule_slowdown(SimTime(0), SimTime(10), 2);
        p.schedule_freeze(SimTime(1), SimTime(3));
    });
    CHECK(mixed.report.outcome == RunOutcome::end_condition_met);
}

TEST_CASE("A window between two of the controller's steps is a run error", "[CTL-021]") {
    auto runtime = std::make_unique<NullControllerRuntime>(Rate(500));
    runtime->load(FirmwareImage{"null", 500});
    runtime->reset(native_test::logic_rail);
    ControllerPartyConfig config;
    config.rate = Rate(500);
    auto party = std::make_unique<ControllerParty>(std::move(runtime), config);
    party->schedule_freeze(SimTime(3), SimTime(4));
    RunConfig run_config;
    run_config.control_rate = Rate(500);
    CoSimKernel kernel{run_config};
    kernel.register_party(std::make_unique<TestParty>("physics", PartyRole::physics));
    auto source = std::make_unique<TestParty>("signals", PartyRole::sensors);
    source->published_names = {"signals.pin_inputs"};
    source->write = [](SimTime t, CouplingWriter& out) {
        out.set("signals.pin_inputs", mechsim::kernel::make_payload(constant_inputs(t), mechsim::pins::pin_inputs_kind));
    };
    kernel.register_party(std::move(source));
    kernel.register_party(std::move(party));
    InMemoryRecorderSink sink;
    const RunReport report = kernel.run(EndCondition{Seconds(10, 1'000), {}}, sink);
    CHECK(report.outcome == RunOutcome::run_error);
    CHECK(fault_cause(report).find("holds none of its steps") != std::string::npos);
}

TEST_CASE("Output latency delays the published pin outputs, events and state codes by whole control ticks",
          "[CTL-020]") {
    // digital_in[31] steps high at tick 5, so the firmware's digital_out[0] rises at its tick 5.
    const InputsAt step_input = [](SimTime t) {
        PinInputs inputs = busy_inputs();
        inputs.digital_in.at(31) = 0;
        if (t.count() >= 5) {
            inputs.digital_in.at(31) = 1;
        }
        return inputs;
    };
    for (const std::size_t latency : {std::size_t{1}, std::size_t{3}}) {
        ControllerPartyConfig config;
        config.latency_ticks = latency;
        const Harness h = run(mirror_party("faults-latency-" + std::to_string(latency), config), 12, step_input);
        REQUIRE(h.report.outcome == RunOutcome::end_condition_met);
        REQUIRE(h.seen->size() == 12);
        for (std::size_t t = 0; t < 12; ++t) {
            const PinOutputs& out = h.seen->at(t);
            if (t < latency) {
                CHECK(native_test::same_outputs(out, PinOutputs{}));
                CHECK(h.controller_states.at(t) == "0");
                continue;
            }
            const std::size_t produced_at = t - latency;
            CHECK(static_cast<std::size_t>(out.state_code) == produced_at + 1);
            CHECK(h.controller_states.at(t) == std::to_string(produced_at + 1));
            CHECK(out.event_count == 16);
            std::uint8_t level = 0;
            if (produced_at >= 5) {
                level = 1;
            }
            CHECK(out.digital_out.at(0) == level);
        }
        CHECK(h.seen->at(4 + latency).digital_out.at(0) == 0);
        CHECK(h.seen->at(5 + latency).digital_out.at(0) == 1);
    }
}

TEST_CASE("Latency 0 publishes exactly what the runtime produced, as before latency existed", "[CTL-020]") {
    ControllerPartyConfig explicit_zero;
    explicit_zero.latency_ticks = 0;
    const Harness defaulted = run(mirror_party("faults-latency-default"), 20, constant_inputs);
    const Harness zero = run(mirror_party("faults-latency-zero", explicit_zero), 20, constant_inputs);
    auto reference = mirror_runtime("faults-latency-reference", native_test::brownout_threshold_v);
    reference->reset(native_test::logic_rail);
    REQUIRE(defaulted.seen->size() == 20);
    REQUIRE(zero.seen->size() == 20);
    for (std::size_t t = 0; t < 20; ++t) {
        PinOutputs direct;
        reference->tick(busy_inputs(), direct);
        CHECK(native_test::same_outputs(defaulted.seen->at(t), direct));
        CHECK(native_test::same_outputs(zero.seen->at(t), direct));
        CHECK(defaulted.controller_states.at(t) == zero.controller_states.at(t));
    }
    CHECK(defaulted.events.empty());
    CHECK(zero.events.empty());
}

TEST_CASE("A rail drop below the chip's brownout threshold holds the controller in reset and restarts it",
          "[CTL-013][TST-017][SIG-022][CTL-012]") {
    // The threshold comes from NativeRuntimeConfig (the chip's brownout_threshold); 2.0 V is the placeholder chip's
    // value, below the runtime's 2.7 V default, so a 2.5 V drop shows the configured value is the one applied.
    constexpr double threshold_v = 2.0;
    const auto rail_run = [](const std::string& work, double dropped_v, NativeControllerRuntime** view) {
        auto runtime = mirror_runtime(work, threshold_v);
        *view = runtime.get();
        ControllerPartyConfig config;
        config.rail = "logic";
        auto party = std::make_unique<ControllerParty>(std::move(runtime), config);
        const VoltageAt voltage = [dropped_v](SimTime t) {
            if (t.count() >= 20 && t.count() < 50) {
                return dropped_v;
            }
            return 3.3;
        };
        return run(std::move(party), 70, constant_inputs, voltage, *view);
    };

    NativeControllerRuntime* runtime = nullptr;
    const Harness below = rail_run("faults-brownout-below", 1.5, &runtime);
    REQUIRE(below.report.outcome == RunOutcome::end_condition_met);
    CHECK(below.seen->at(19).state_code == 20);
    CHECK(below.seen->at(19).fault_code == 1);
    for (std::size_t t = 20; t < 50; ++t) {
        CHECK(native_test::same_outputs(below.seen->at(t), PinOutputs{}));
    }
    // Recovery opens a fresh copy: the tick counter restarts and the copy has seen one init.
    CHECK(below.seen->at(50).state_code == 1);
    CHECK(below.seen->at(50).fault_code == 1);
    CHECK(below.seen->at(50).digital_out.at(0) == 1);
    CHECK(below.seen->at(69).state_code == 20);
    CHECK(below.inits == 2);
    CHECK(below.ticks_run == 40);
    CHECK(below.physics_steps == 70);

    for (const double dropped_v : {2.5, threshold_v}) {
        NativeControllerRuntime* steady = nullptr;
        const Harness above = rail_run("faults-brownout-above", dropped_v, &steady);
        REQUIRE(above.report.outcome == RunOutcome::end_condition_met);
        CHECK(above.inits == 1);
        CHECK(above.ticks_run == 70);
        for (std::size_t t = 0; t < 70; ++t) {
            CHECK(static_cast<std::size_t>(above.seen->at(t).state_code) == t + 1);
            CHECK(above.seen->at(t).fault_code == 1);
        }
    }
}
