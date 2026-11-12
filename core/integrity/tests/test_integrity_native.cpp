#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/integrity/integrity_party.hpp"
#include "mechsim/integrity/limits.hpp"
#include "mechsim/kernel/cosim_kernel.hpp"
#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/kernel.hpp"
#include "mechsim/kernel/memory_recorder_sink.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_party.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/physics/native/native_adapter.hpp"

using Catch::Matchers::WithinRel;
namespace integrity = mechsim::integrity;
namespace kernel = mechsim::kernel;

// ===== Cantilever on the native adapter ==================================

namespace {

// Steel cantilever, L = 1 m, N = 8 (l = 0.125 m), E I = 2000 N m^2, rho A = 0.78 kg/m, clamped on a fixed base body,
// no gravity, no node damping. The fibre c_y = 0.1 m is data only (the engine bends with E I), so the yield moment
// M_y = sigma_y I / c = 2.5e8 * 1e-8 / 0.1 = 25 N m is reached at a small elastic deflection.
constexpr double length = 1.0;
constexpr double segment = length / 8.0;
constexpr double flexural = 2e11 * 1e-8;
constexpr double stiffness = flexural / segment;  // k = E I / l = 16000 N m / rad
constexpr double yield_moment = 25.0;
constexpr double peak_load = 40.0;
constexpr double bump = 4.0;  // s: P = P_max sin^2(pi t / T), then zero
constexpr int ticks = 5000;

std::string doc() {
    const std::string base =
        R"({"name": "base", "parent": -1, "mass": 1.0, "com": [0, 0, 0], "inertia": [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01], )"
        R"("joint": {"type": "fixed", "position": [0, 0, 0], "orientation": [1, 0, 0, 0], "axis": [0, 0, 1], )"
        R"("stiffness": [], "damping": [], "rest": [], "friction": [], "lower": [], "upper": []}})";
    const std::string member =
        R"({"parent": 0, "position": [0, 0, 0], "orientation": [1, 0, 0, 0], "youngs_modulus": 2e11, )"
        R"("area_moment": 1e-8, "density": 7800, "area": 1e-4, "length": 1.0, "segments": 8, "node_damping": 0.0, )"
        R"("ends": "cantilever", "roller_mass_fraction": 0.001})";
    return R"({"format": "mechsim-native-1", "elements": ["body", "joint.fixed", "lumped_member"], )"
           R"("gravity": [0, 0, 0], "config": {"iteration_cap": 50, "velocity_tolerance": 1e-9, )"
           R"("projection_iterations": 3, "drift_threshold_m": 1e-6, "drift_threshold_rad": 1e-6, )"
           R"("unit_tolerance": 1e-12, "symmetry_tolerance": 1e-12}, "bodies": [)" +
           base + R"(], "neutral_bodies": 1, "constraints": [], "members": [)" + member +
           R"(], "transmissions": [], "ropes": [], "contact": null, "payloads": [], "starts": []})";
}

// The tip load (0, -P, 0) at x = L, applied at the tip segment's centre of mass (x = L - l / 4, it spans
// [L - l / 2, L]) with the moment (0, 0, -P l / 4) that moves it to the tip.
class TipLoad final : public kernel::Party {
public:
    [[nodiscard]] std::string_view name() const override { return "load"; }
    [[nodiscard]] kernel::Rate nominal_rate() const override { return kernel::Rate(1000); }
    [[nodiscard]] std::span<const std::string> published() const override { return published_; }
    [[nodiscard]] std::span<const std::string> consumed() const override { return {}; }
    void start(const kernel::Rate& /*base_rate*/) override {}
    void step(kernel::SimTime t, kernel::Duration /*dt*/) override {
        const double time = static_cast<double>(t.count()) * 1e-3;
        double p = 0.0;
        if (time < bump) {
            const double s = std::sin(std::numbers::pi * time / bump);
            p = peak_load * s * s;
        }
        inputs_.body_wrenches = {kernel::BodyWrench{8, kernel::Wrench{{0.0, 0.0, -p * segment / 4.0}, {0.0, -p, 0.0}}}};
    }
    void write_outputs(kernel::CouplingWriter& outputs) override {
        outputs.set(published_[0], kernel::make_payload(inputs_, std::string(kernel::physics_inputs_kind)));
    }
    void on_fault(const kernel::PartyFault& /*fault*/) noexcept override {}

private:
    std::vector<std::string> published_{"load.inputs"};
    kernel::PhysicsInputs inputs_;
};

integrity::Limits limits(bool stress, bool yield) {
    integrity::Limits out;
    out.stress = stress;
    out.yield_offset = yield;
    integrity::MemberLimits m;
    m.name = "arm";
    m.area = 1e-4;
    m.i_y = 1e-8;
    m.i_z = 1e-8;
    m.c_y = 0.1;
    m.c_z = 0.1;
    m.length = length;
    m.elastic_modulus = 2e11;
    m.effective_length_factor = 2.0;
    m.allowable_stress = 2.5e8;
    m.yield_strength = 2.5e8;
    m.supports = {integrity::Support{0.0, integrity::SupportCondition::clamped}};
    m.first_spring = 0;
    m.segments = 8;
    m.segment_length = segment;
    m.node_stiffness = stiffness;
    out.members.push_back(m);
    return out;
}

struct Result {
    std::vector<std::string> events;  // "<tick> <kind> <detail>"
    std::int64_t first_change = -1;
    double tip_y = 0.0;
    std::shared_ptr<integrity::IntegrityTrace> trace;
    kernel::RunReport report;
};

Result run(bool stress, bool yield) {
    auto adapter = std::make_unique<mechsim::physics::native::NativeAdapter>();
    adapter->load(kernel::PreparedModel{doc(), "", "1.0.0"}, kernel::Rate(1000));
    kernel::PhysicsPartyConfig physics;
    physics.rate = kernel::Rate(1000);
    physics.inputs = {"load.inputs"};
    physics.model_requests = "integrity.model_requests";
    if (!yield) {
        physics.model_requests.clear();
    }
    kernel::CoSimKernel k{kernel::RunConfig{}};
    k.register_party(std::make_unique<kernel::PhysicsParty>(std::move(adapter), physics));
    Result out;
    out.trace = std::make_shared<integrity::IntegrityTrace>();
    k.register_party(
        std::make_unique<integrity::IntegrityParty>(limits(stress, yield), integrity::IntegrityPartyConfig{}, out.trace));
    k.register_party(std::make_unique<TipLoad>());
    kernel::InMemoryRecorderSink sink;
    out.report = k.run(kernel::EndCondition{kernel::Seconds(ticks, 1000)}, sink);
    const std::vector<kernel::TickRecord> rows = sink.drain();
    for (const kernel::TickRecord& row : rows) {
        for (const kernel::RecordedEvent& event : row.events) {
            out.events.push_back(std::to_string(row.time.count()) + " " + event.kind + " " + event.detail);
            if (event.kind == "model_change" && out.first_change < 0) {
                out.first_change = row.time.count();
            }
        }
    }
    const kernel::BodyState& tip = rows.back().physics.bodies.at(8);
    const auto& q = tip.orientation;
    out.tip_y = tip.position[1] + (segment / 2.0) * 2.0 * (q[1] * q[2] + q[0] * q[3]);
    return out;
}

}  // namespace

TEST_CASE("a yielded cantilever keeps the tip offset of its peak root-spring moment", "[INT-028][KER-028][PHY-008]") {
    // Ruling 35: the root-side spring at l / 2 carries M_n = -P (L - l / 2) by statics over the stub; at the peak
    // P_max = 40 N, |M_n| = 37.5 N m > M_y = 25 N m, so phi = -(37.5 - 25) / k = -7.8125e-4 rad. Unloaded, every other
    // spring returns to its rest and the chain beyond l / 2 turns rigidly by phi: tip y = (L - l / 2) sin(phi).
    // Bound: (1) quasi-static error of the sin^2 bump at the peak |P''| / (omega_1^2 P) = 2 (pi / T)^2 / omega_1^2 with
    // omega_1 = 1.875^2 sqrt(E I / (rho A)) / L^2 = 178 rad/s: 3.9e-5 of M_n, 1.2e-4 of phi (M_n / (M_n - M_y) = 3);
    // (2) the world-aligned load's arm shortens by about theta_tip^2 L / 3 = 3.3e-5 m (theta_tip = P L^2 / 2 E I =
    // 0.01): 1.1e-4 of phi; (3) after 1 s unloaded the first mode keeps (1 + x^2)^(-28 pi / atan x) < 1e-6 of its
    // amplitude under backward Euler (x = omega_1 h = 0.178, ruling 21). Sum 2.3e-4; asserted at 1e-3.
    const Result r = run(false, true);
    const double phi = -(peak_load * (length - segment / 2.0) - yield_moment) / stiffness;
    const double expected = (length - segment / 2.0) * std::sin(phi);
    std::cout << "yield oracle: tip y " << r.tip_y << " m, closed form " << expected << " m, residual "
              << std::abs(r.tip_y - expected) / std::abs(expected) << " relative\n";
    CHECK_THAT(r.tip_y, WithinRel(expected, 1e-3));
    REQUIRE_FALSE(r.trace->requests.empty());
    CHECK_THAT(r.trace->requests.back().value, WithinRel(phi, 1e-3));
    for (const kernel::ModelChangeRequest& request : r.trace->requests) {
        CHECK(request.index == 0);
    }
    // Each request reaches the adapter before the next step and comes back once as a model_change event (ruling 26).
    std::size_t changes = 0;
    for (const std::string& event : r.events) {
        if (event.find(" model_change spring_offset spring 0 value ") != std::string::npos) {
            ++changes;
        }
        CHECK(event.find(" integrity ") == std::string::npos);
    }
    CHECK(changes == r.trace->requests.size());
    CHECK(r.report.outcome == kernel::RunOutcome::end_condition_met);
}

TEST_CASE("tier 2 leaves the tier 1 events unchanged up to its first model change", "[INT-003][INT-027][INT-028]") {
    // The root stress P L c / I passes sigma_y (P > 25 N) before the root spring's moment P (L - l / 2) passes M_y
    // (P > 26.7 N), so the stress event comes first in both runs.
    const Result tier1 = run(true, false);
    const Result tier2 = run(true, true);
    REQUIRE(tier2.first_change > 0);
    std::vector<std::string> before;
    for (const std::string& event : tier2.events) {
        if (std::stoll(event) < tier2.first_change) {
            before.push_back(event);
        }
    }
    std::vector<std::string> reference;
    for (const std::string& event : tier1.events) {
        if (std::stoll(event) < tier2.first_change) {
            reference.push_back(event);
        }
    }
    REQUIRE(before.size() == 1);
    CHECK(before[0].find(" integrity ") != std::string::npos);
    CHECK(before == reference);
    CHECK(tier1.first_change == -1);
}

TEST_CASE("a native run with integrity runs twice bitwise", "[INT-027]") {
    const Result first = run(true, true);
    const Result second = run(true, true);
    CHECK(first.events == second.events);
    CHECK(first.tip_y == second.tip_y);
}
