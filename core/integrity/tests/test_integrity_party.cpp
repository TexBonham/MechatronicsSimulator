#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/integrity/beam.hpp"
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

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
namespace integrity = mechsim::integrity;
namespace kernel = mechsim::kernel;

// ===== Fixtures ==========================================================

namespace {

constexpr double h = 1e-3;

// Publishes physics.outputs as the physics party would, from a closed-form load at the state's time t = (n + 1) h
// after the n-th step (KER-009: the outputs of the step from t_n hold the state at its end).
class LoadSource final : public kernel::Party {
public:
    explicit LoadSource(std::function<kernel::PhysicsOutputs(double)> load) : load_(std::move(load)) {}

    [[nodiscard]] std::string_view name() const override { return "physics"; }
    [[nodiscard]] kernel::Rate nominal_rate() const override { return kernel::Rate(1000); }
    [[nodiscard]] kernel::PartyRole role() const override { return kernel::PartyRole::physics; }
    [[nodiscard]] std::span<const std::string> published() const override { return published_; }
    [[nodiscard]] std::span<const std::string> consumed() const override { return {}; }
    void start(const kernel::Rate& /*base_rate*/) override {}
    void step(kernel::SimTime t, kernel::Duration dt) override {
        outputs_ = load_(static_cast<double>((t + dt).count()) * h);
    }
    void write_outputs(kernel::CouplingWriter& outputs) override {
        outputs.set(published_[0], kernel::make_payload(outputs_, std::string(kernel::physics_outputs_kind)));
    }
    void on_fault(const kernel::PartyFault& /*fault*/) noexcept override {}

private:
    std::function<kernel::PhysicsOutputs(double)> load_;
    std::vector<std::string> published_{"physics.outputs"};
    kernel::PhysicsOutputs outputs_;
};

integrity::MemberLimits cantilever(double length = 1.0) {
    integrity::MemberLimits m;
    m.name = "arm";
    m.area = 1e-4;
    m.i_y = 2e-8;
    m.i_z = 1e-8;
    m.c_y = 0.01;
    m.c_z = 0.02;
    m.length = length;
    m.elastic_modulus = 2e11;
    m.effective_length_factor = 2.0;
    m.allowable_stress = 1e8;
    m.yield_strength = 2.5e8;
    m.supports = {integrity::Support{0.0, integrity::SupportCondition::clamped}};
    m.segments = 8;
    m.segment_length = length / 8.0;
    m.node_stiffness = 2e11 * 1e-8 / (length / 8.0);
    m.first_spring = 3;
    return m;
}

// A cantilever clamped at x = 0 under a tip force (0, -P, 0) at x = L: the support on the member at the start carries
// F = (0, P, 0) and, about x = 0, M = (0, 0, P L); the free end carries nothing (statics).
kernel::PhysicsOutputs tip_loaded(double load, double length) {
    kernel::PhysicsOutputs out;
    kernel::MemberEndLoads loads;
    loads.start.force = {0.0, load, 0.0};
    loads.start.moment = {0.0, 0.0, load * length};
    out.members.push_back(loads);
    return out;
}

struct Run {
    std::vector<integrity::Crossing> crossings;
    std::vector<kernel::TickRecord> rows;
    kernel::RunReport report;
    std::shared_ptr<integrity::IntegrityTrace> trace;
};

Run run(integrity::Limits limits, std::function<kernel::PhysicsOutputs(double)> load, int ticks, bool halt = false) {
    kernel::CoSimKernel k{kernel::RunConfig{}};
    k.register_party(std::make_unique<LoadSource>(std::move(load)));
    integrity::IntegrityPartyConfig config;
    config.halt_on_crossing = halt;
    Run out;
    out.trace = std::make_shared<integrity::IntegrityTrace>();
    k.register_party(std::make_unique<integrity::IntegrityParty>(std::move(limits), config, out.trace));
    if (halt) {
        k.set_end_triggers(kernel::EndTriggers{std::nullopt, "", "integrity"});
    }
    kernel::InMemoryRecorderSink sink;
    out.report = k.run(kernel::EndCondition{kernel::Seconds(ticks, 1000), {}}, sink);
    out.rows = sink.drain();
    out.crossings = out.trace->crossings;
    return out;
}

std::vector<std::int64_t> event_ticks(const Run& r) {
    std::vector<std::int64_t> ticks;
    for (const kernel::TickRecord& row : r.rows) {
        for (const kernel::RecordedEvent& event : row.events) {
            CHECK(event.time == row.time);
            CHECK(event.source == "integrity");
            CHECK(event.kind == "integrity");
            ticks.push_back(row.time.count());
        }
    }
    return ticks;
}

}  // namespace

// ===== Crossing oracles (INT-023, INT-024) ===============================

TEST_CASE("a cantilever under a ramping tip load crosses the allowable stress at t*", "[INT-023][INT-024]") {
    // Root stress sigma = P L c_y / I_z with P = k t, so sigma > sigma_allow first at t > t* = sigma_allow I / (k L c).
    // k = 952.380952... N/s puts t* = 10.5 ms between ticks: the first tick strictly past it is t = 11 ms.
    integrity::Limits limits;
    limits.stress = true;
    limits.members.push_back(cantilever());
    const integrity::MemberLimits& m = limits.members[0];
    const double t_star = 0.0105;
    const double rate = m.allowable_stress * m.i_z / (t_star * m.length * m.c_y);
    const Run r = run(limits, [rate](double t) { return tip_loaded(rate * t, 1.0); }, 30);
    REQUIRE(r.crossings.size() == 1);
    CHECK(r.crossings[0].time.count() == 11);
    CHECK(r.crossings[0].quantity == "stress");
    CHECK(r.crossings[0].element == "arm");
    CHECK_THAT(r.crossings[0].value, WithinRel(rate * 0.011 * m.length * m.c_y / m.i_z, 1e-12));
    CHECK(r.crossings[0].limit == m.allowable_stress);
    const std::vector<std::int64_t> ticks = event_ticks(r);
    CHECK(ticks == std::vector<std::int64_t>{11});
    // Ruling 34: one event per (member, quantity) per run, though the load keeps rising past the limit.
    CHECK(r.report.end_reason == kernel::EndReason::max_duration);
    CHECK_THAT(r.trace->peak_stress[0], WithinRel(rate * 0.030 * m.length * m.c_y / m.i_z, 1e-12));
}

TEST_CASE("a pinned-pinned column under a ramping axial load crosses margin 1 at t*", "[INT-023][INT-024]") {
    // P_cr = pi^2 E I_min / (K L)^2 with K = 1; P = k t, so P_cr / P < 1 first at t > t* = pi^2 E I / (k L^2).
    integrity::Limits limits;
    limits.buckling = true;
    integrity::MemberLimits m = cantilever(2.0);
    m.effective_length_factor = 1.0;
    m.supports = {integrity::Support{0.0, integrity::SupportCondition::pinned},
                  integrity::Support{2.0, integrity::SupportCondition::pinned}};
    limits.members.push_back(m);
    const double critical = std::numbers::pi * std::numbers::pi * m.elastic_modulus * m.i_z / (m.length * m.length);
    const double t_star = 0.0205;
    const double rate = critical / t_star;
    const auto axial = [rate](double t) {
        kernel::PhysicsOutputs out;
        kernel::MemberEndLoads loads;
        loads.start.force = {rate * t, 0.0, 0.0};
        loads.end.force = {-rate * t, 0.0, 0.0};
        out.members.push_back(loads);
        return out;
    };
    const Run r = run(limits, axial, 40);
    REQUIRE(r.crossings.size() == 1);
    CHECK(r.crossings[0].time.count() == 21);
    CHECK(r.crossings[0].quantity == "buckling_margin");
    CHECK_THAT(r.crossings[0].value, WithinRel(t_star / 0.021, 1e-12));
    CHECK(r.crossings[0].limit == 1.0);
    CHECK_THAT(integrity::critical_load(m), WithinRel(critical, 1e-15));
}

TEST_CASE("a fastening under a ramping load crosses its rating at t*", "[INT-023][INT-024]") {
    // |F| = k t along (3, 4, 0) / 5; |F| > rating first at t > t* = rating / k.
    integrity::Limits limits;
    limits.fastener = true;
    limits.fastenings.push_back(integrity::FasteningLimits{"bolts", 1, 250.0});
    const double t_star = 0.0075;
    const double rate = 250.0 / t_star;
    const auto load = [rate](double t) {
        kernel::PhysicsOutputs out;
        out.joints.resize(2);
        out.joints[1].reaction.force = {0.6 * rate * t, -0.8 * rate * t, 0.0};
        return out;
    };
    const Run r = run(limits, load, 20);
    REQUIRE(r.crossings.size() == 1);
    CHECK(r.crossings[0].time.count() == 8);
    CHECK(r.crossings[0].quantity == "fastener_load");
    CHECK(r.crossings[0].element == "bolts");
    CHECK_THAT(r.crossings[0].value, WithinRel(rate * 0.008, 1e-12));
}

TEST_CASE("a cantilever's static tip deflection crosses its limit at t*", "[INT-023][INT-024]") {
    // Tip deflection P L^3 / (3 E I_z) (the grid holds x = L); > delta first at t > t* = 3 E I delta / (k L^3).
    integrity::Limits limits;
    limits.deflection = true;
    integrity::MemberLimits m = cantilever();
    m.deflection_limit = 0.01;
    limits.members.push_back(m);
    const double t_star = 0.0045;
    const double rate = 3.0 * m.elastic_modulus * m.i_z * 0.01 / (t_star * std::pow(m.length, 3));
    const Run r = run(limits, [rate](double t) { return tip_loaded(rate * t, 1.0); }, 10);
    REQUIRE(r.crossings.size() == 1);
    CHECK(r.crossings[0].time.count() == 5);
    CHECK(r.crossings[0].quantity == "deflection");
    CHECK_THAT(r.crossings[0].value, WithinRel(0.01 * 0.005 / t_star, 1e-9));
}

TEST_CASE("tier 1 quantities follow beam.py on mixed end loads", "[INT-023]") {
    integrity::MemberLimits m = cantilever();
    kernel::MemberEndLoads loads;
    loads.start = kernel::Wrench{{0.3, 1.5, -2.0}, {40.0, 3.0, 1.0}};
    loads.end = kernel::Wrench{{0.0, -0.7, 4.0}, {-40.0, 2.0, 0.0}};
    // Ends: |N| / A + |M_y| c_z / I_y + |M_z| c_y / I_z (ruling 25).
    const double start = 40.0 / 1e-4 + 1.5 * 0.02 / 2e-8 + 2.0 * 0.01 / 1e-8;
    const double end = 40.0 / 1e-4 + 0.7 * 0.02 / 2e-8 + 4.0 * 0.01 / 1e-8;
    CHECK_THAT(integrity::end_stress(loads.start, m), WithinRel(start, 1e-15));
    CHECK_THAT(integrity::end_stress(loads.end, m), WithinRel(end, 1e-15));
    // Interior load per plane (beam.py `_plane`): about z, moment0 = -M_z0 = 2, shear F_y0 = 3, Q = -(3 + 2) = -5 at
    // a = -(M_z0 + M_z1 + L F_y1) / Q = -4 / -5 = 0.8; about y, moment0 = M_y0 = 1.5, shear F_z0 = 1, Q = -1 at
    // a = (M_y0 + M_y1 - L F_z1) / Q = 0.8 / -1, clipped to 0 (the start). At x = 0.8: M_z = 2 + 3 (0.8) = 4.4,
    // M_y = 1.5 + 0.8 - 0.8 = 1.5, abs(N) the larger end value 40: 6.3e6 Pa against the ends' 3.9e6 and 5.1e6.
    const double interior = 40.0 / 1e-4 + 1.5 * 0.02 / 2e-8 + 4.4 * 0.01 / 1e-8;
    CHECK_THAT(integrity::peak_normal_stress(loads, m), WithinRel(interior, 1e-12));
    CHECK(integrity::compression(loads) == 40.0);
    loads.start.force[0] = -1.0;
    loads.end.force[0] = 2.0;
    CHECK(integrity::compression(loads) == 0.0);
    // Node moments by statics over the half segment l / 2 (ruling 35).
    CHECK_THAT(integrity::start_node_moment(loads.start, 0.125), WithinRel(2.0 + 0.0625 * 3.0, 1e-15));
    CHECK_THAT(integrity::end_node_moment(loads.end, 0.125), WithinRel(4.0 + 0.0625 * 2.0, 1e-15));
}

TEST_CASE("simply supported and clamped-clamped deflections are the closed forms", "[INT-023]") {
    // Midspan point load W on a simply supported span: W L^3 / (48 E I) at the midspan grid point (I1's rule, exact).
    integrity::MemberLimits m = cantilever(2.0);
    m.supports = {integrity::Support{0.0, integrity::SupportCondition::pinned},
                  integrity::Support{2.0, integrity::SupportCondition::pinned}};
    kernel::MemberEndLoads loads;
    loads.start.force = {0.0, 50.0, 0.0};
    loads.end.force = {0.0, 50.0, 0.0};
    const double ei = m.elastic_modulus * m.i_z;
    CHECK_THAT(integrity::static_deflection(loads, m), WithinRel(100.0 * 8.0 / (48.0 * ei), 1e-12));
    // Clamped-clamped: end moments W L / 8 hold the slopes; midspan W L^3 / (192 E I).
    m.supports = {integrity::Support{0.0, integrity::SupportCondition::clamped},
                  integrity::Support{2.0, integrity::SupportCondition::clamped}};
    loads.start.moment = {0.0, 0.0, 100.0 * 2.0 / 8.0};
    loads.end.moment = {0.0, 0.0, -100.0 * 2.0 / 8.0};
    CHECK_THAT(integrity::static_deflection(loads, m), WithinRel(100.0 * 8.0 / (192.0 * ei), 1e-9));
    m.supports = {integrity::Support{1.0, integrity::SupportCondition::pinned}};
    CHECK_THROWS_AS(integrity::static_deflection(loads, m), integrity::LimitsError);
}

// ===== Events, halt and determinism (INT-024, INT-026, INT-027) ==========

TEST_CASE("each element and quantity raises one event at its own first crossing", "[INT-024]") {
    integrity::Limits limits;
    limits.stress = true;
    limits.deflection = true;
    integrity::MemberLimits m = cantilever();
    m.deflection_limit = 1.05e-3;
    m.allowable_stress = 1.2e8;
    limits.members.push_back(m);
    m.name = "second";
    m.allowable_stress = 5.05e7;
    limits.members.push_back(m);
    const Run r = run(limits, [](double t) {
        kernel::PhysicsOutputs out = tip_loaded(1000.0 * t, 1.0);
        out.members.push_back(out.members[0]);
        return out;
    }, 100);
    // Stress P L c_y / I_z = 1e9 t Pa: the second member (5.05e7) at t > 50.5 ms, the first (1.2e8) at t > 120 ms
    // (never in 100 ticks). Deflection P L^3 / (3 E I) = t / 6 m: > 1.05e-3 at t > 6.3 ms, both members.
    std::vector<std::string> seen;
    for (const integrity::Crossing& c : r.crossings) {
        seen.push_back(c.element + " " + c.quantity + " " + std::to_string(c.time.count()));
    }
    CHECK(seen == std::vector<std::string>{"arm deflection 7", "second deflection 7", "second stress 51"});
    CHECK(integrity::crossing_detail(r.crossings[0]).starts_with(
        R"({"limit": 0.00105, "member": "arm", "quantity": "deflection", "tier": 1, "value": )"));
}

TEST_CASE("halting ends the run at the crossing tick through party_state", "[INT-026]") {
    integrity::Limits limits;
    limits.stress = true;
    limits.members.push_back(cantilever());
    const double rate = 1e8 * 1e-8 / (0.0105 * 0.01);
    const auto load = [rate](double t) { return tip_loaded(rate * t, 1.0); };
    const Run halted = run(limits, load, 50, true);
    CHECK(halted.report.end_reason == kernel::EndReason::party_state);
    CHECK(halted.report.end_time.count() == 11);
    REQUIRE(halted.rows.size() == 11);
    CHECK(halted.rows.back().events.size() == 1);
    const Run continued = run(limits, load, 50, false);
    CHECK(continued.report.end_reason == kernel::EndReason::max_duration);
    CHECK(continued.rows.size() == 50);
}

TEST_CASE("the same load gives the same events at the same times", "[INT-027]") {
    integrity::Limits limits;
    limits.stress = true;
    limits.deflection = true;
    limits.members.push_back(cantilever());
    limits.members[0].deflection_limit = 2e-3;
    const auto load = [](double t) { return tip_loaded(4000.0 * std::sin(40.0 * t) * t, 1.0); };
    const Run first = run(limits, load, 200);
    const Run second = run(limits, load, 200);
    const auto events = [](const Run& r) {
        std::vector<std::string> out;
        for (const kernel::TickRecord& row : r.rows) {
            for (const kernel::RecordedEvent& event : row.events) {
                out.push_back(std::to_string(event.time.count()) + " " + event.detail);
            }
        }
        return out;
    };
    const std::vector<std::string> a = events(first);
    const std::vector<std::string> b = events(second);
    CHECK(a.size() == 2);
    CHECK(a == b);
}

// ===== Yield requests (INT-028) ==========================================

TEST_CASE("the yield rule offsets the root spring by the peak excess over M_y", "[INT-028]") {
    // Root-side node spring at l / 2 carries M_n = -M_z0 + (l / 2) F_y0 = -P (L - l / 2) (statics over the stub).
    // M_y = sigma_y I_z / c_y = 250 N m; k = E I / l = 16000 N m / rad. P rises as 400 t to 0.8 s, then falls.
    integrity::Limits limits;
    limits.yield_offset = true;
    limits.members.push_back(cantilever());
    const integrity::MemberLimits& m = limits.members[0];
    const auto load = [](double t) {
        double p = 400.0 * t;
        if (t > 0.8) {
            p = 400.0 * (1.6 - t);
        }
        return tip_loaded(p, 1.0);
    };
    const Run r = run(limits, load, 1200);
    const double arm = m.length - m.segment_length / 2.0;
    const double yield_moment = m.yield_strength * m.i_z / m.c_y;
    REQUIRE_THAT(yield_moment, WithinRel(250.0, 1e-15));
    // First request at the first t with 400 t arm > 250: t > 0.6667 s, tick 667; the last at the peak, tick 800.
    REQUIRE(r.trace->requests.size() == 800 - 667 + 1);
    const kernel::ModelChangeRequest& last = r.trace->requests.back();
    CHECK(last.kind == kernel::ModelChangeKind::SpringOffset);
    CHECK(last.index == m.first_spring);
    CHECK_THAT(last.value, WithinRel(-(400.0 * 0.8 * arm - yield_moment) / m.node_stiffness, 1e-12));
    for (std::size_t k = 1; k < r.trace->requests.size(); ++k) {
        CHECK(r.trace->requests[k].value < r.trace->requests[k - 1].value);
    }
    CHECK(r.crossings.empty());
}

TEST_CASE("the yield rule takes the end with the larger moment", "[INT-028]") {
    // End moment larger: the last spring (first + N - 1) takes M_n = M_z1 + (l / 2) F_y1.
    integrity::Limits limits;
    limits.yield_offset = true;
    limits.members.push_back(cantilever());
    const integrity::MemberLimits& m = limits.members[0];
    const auto load = [](double /*t*/) {
        kernel::PhysicsOutputs out;
        kernel::MemberEndLoads loads;
        loads.start.moment = {0.0, 0.0, 10.0};
        loads.end.moment = {0.0, 0.0, 400.0};
        loads.end.force = {0.0, 80.0, 0.0};
        out.members.push_back(loads);
        return out;
    };
    const Run r = run(limits, load, 5);
    REQUIRE(r.trace->requests.size() == 1);
    CHECK(r.trace->requests[0].index == m.first_spring + 7);
    CHECK_THAT(r.trace->requests[0].value, WithinRel((400.0 + 0.0625 * 80.0 - 250.0) / m.node_stiffness, 1e-12));
}

// ===== Limits table ======================================================

TEST_CASE("the limits table parses and refuses malformed entries", "[INT-023]") {
    const std::string member =
        R"({"name": "arm", "index": 0, "area": 1e-4, "i_y": 2e-8, "i_z": 1e-8, "c_y": 0.01, "c_z": 0.02, )"
        R"("length": 1.0, "elastic_modulus": 2e11, "effective_length_factor": 2.0, "allowable_stress": 1e8, )"
        R"("yield_strength": 2.5e8, "deflection_limit": null, "supports": [{"at": 0.0, "condition": "clamped"}], )"
        R"("first_spring": 2, "segments": 8, "segment_length": 0.125, "node_stiffness": 16000.0})";
    const auto doc = [](const std::string& members, const std::string& checks = "true") {
        return R"({"format": "mechsim-integrity-limits-1", "minimum_safety_factor": 1.5, "checks": {"stress": )" +
               checks + R"(, "buckling": false, "deflection": false, "fastener": true, "yield_offset": false}, )"
               R"("members": [)" + members + R"(], "fastenings": [{"name": "bolts", "joint": 3, "load_limit": 10}]})";
    };
    const integrity::Limits limits = integrity::parse_limits(doc(member));
    REQUIRE(limits.members.size() == 1);
    CHECK(limits.members[0].first_spring == 2);
    CHECK_FALSE(limits.members[0].deflection_limit.has_value());
    CHECK(limits.minimum_safety_factor == 1.5);
    CHECK(limits.stress);
    CHECK(limits.fastenings[0].joint == 3);
    CHECK_THROWS_AS(integrity::parse_limits(doc(member, "1")), integrity::LimitsError);
    CHECK_THROWS_AS(integrity::parse_limits(doc(member) + "x"), integrity::LimitsError);
    CHECK_THROWS_AS(integrity::parse_limits("{"), integrity::LimitsError);
    CHECK_THROWS_AS(integrity::parse_limits(R"({"format": "other"})"), integrity::LimitsError);
    std::string negative = member;
    negative.replace(negative.find("1e-4"), 4, "-1.0");
    CHECK_THROWS_AS(integrity::parse_limits(doc(negative)), integrity::LimitsError);
    std::string interior = member;
    interior.replace(interior.find("clamped"), 7, "welded");
    CHECK_THROWS_AS(integrity::parse_limits(doc(interior)), integrity::LimitsError);
    std::string twice = member;
    twice.replace(twice.find(R"("index": 0)"), 10, R"("index": 0, "index": 1)");
    CHECK_THROWS_AS(integrity::parse_limits(doc(twice)), integrity::LimitsError);
    CHECK_THROWS_AS(integrity::parse_limits(doc(member + ", " + R"({"name": "x"})")), integrity::LimitsError);
}

TEST_CASE("a member published out of range faults the party", "[INT-023]") {
    integrity::Limits limits;
    limits.stress = true;
    limits.members.push_back(cantilever());
    limits.members[0].index = 4;
    const Run r = run(limits, [](double t) { return tip_loaded(t, 1.0); }, 5);
    CHECK(r.report.outcome == kernel::RunOutcome::run_error);
    REQUIRE(r.report.fault.has_value());
    CHECK(r.report.fault->party() == "integrity");
}
