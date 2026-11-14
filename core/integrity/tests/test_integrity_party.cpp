#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/integrity/beam.hpp"
#include "mechsim/integrity/integrity_party.hpp"
#include "mechsim/integrity/limits.hpp"
#include "mechsim/integrity/validity.hpp"
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
    integrity::NodeChain chain;
    chain.count = 8;
    m.nodes = chain;
    return m;
}

// Node poses of a cantilever along world x, clamped at the origin, with tip deflection v(L) = P L^3 / (3 E I_z) of the
// fixture section: interior nodes on the continuum shape v(x) = P x^2 (3 L - x) / (6 E I_z) at x_j = (j + 1/2) l, the
// last node level at (L - l/2, v(L)), so its far end (l / 2 along its x) is (L, v(L)) and the node-pose deflection
// (INT-006, the distance from the root tangent, world x) is abs(v(L)) to rounding.
std::vector<kernel::BodyState> cantilever_nodes(double load, double length) {
    const double ei = 2e11 * 1e-8;
    const double l = length / 8.0;
    std::vector<kernel::BodyState> bodies(8);
    for (std::size_t j = 0; j + 1 < bodies.size(); ++j) {
        const double x = (static_cast<double>(j) + 0.5) * l;
        bodies[j].position = {x, -load * x * x * (3.0 * length - x) / (6.0 * ei), 0.0};
    }
    bodies.back().position = {length - 0.5 * l, -load * length * length * length / (3.0 * ei), 0.0};
    return bodies;
}

// A cantilever clamped at x = 0 under a tip force (0, -P, 0) at x = L: the support on the member at the start carries
// F = (0, P, 0) and, about x = 0, M = (0, 0, P L); the free end carries nothing (statics).
kernel::PhysicsOutputs tip_loaded(double load, double length) {
    kernel::PhysicsOutputs out;
    kernel::MemberEndLoads loads;
    loads.start.force = {0.0, load, 0.0};
    loads.start.moment = {0.0, 0.0, load * length};
    out.members.push_back(loads);
    out.bodies = cantilever_nodes(load, length);
    return out;
}

struct Run {
    std::vector<integrity::Crossing> crossings;
    std::vector<kernel::TickRecord> rows;
    kernel::RunReport report;
    std::shared_ptr<integrity::IntegrityTrace> trace;
};

Run run(integrity::Limits limits, std::function<kernel::PhysicsOutputs(double)> load, int ticks, bool halt = false,
        kernel::SimTime evaluate_from = kernel::SimTime{0},
        std::shared_ptr<const integrity::ValiditySource> validity = nullptr) {
    kernel::CoSimKernel k{kernel::RunConfig{}};
    k.register_party(std::make_unique<LoadSource>(std::move(load)));
    integrity::IntegrityPartyConfig config;
    config.halt_on_crossing = halt;
    config.evaluate_from = evaluate_from;
    Run out;
    out.trace = std::make_shared<integrity::IntegrityTrace>();
    auto party = std::make_unique<integrity::IntegrityParty>(std::move(limits), config, out.trace);
    if (validity) {
        party->watch_validity(std::move(validity));
    }
    k.register_party(std::move(party));
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
    limits.fastenings.push_back(integrity::FasteningLimits{"bolts", 1, 250.0, std::nullopt});
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

TEST_CASE("a cantilever's tip deflection from its node poses crosses its limit at t*", "[INT-023][INT-024][INT-006]") {
    // Node-pose tip deflection P L^3 / (3 E I_z) (fixture `cantilever_nodes`); > delta first at t > t* =
    // 3 E I delta / (k L^3).
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

namespace {

kernel::Quat about_z(double angle) {
    return {std::cos(0.5 * angle), 0.0, 0.0, std::sin(0.5 * angle)};
}

}  // namespace

TEST_CASE("tier 1 stress takes the ends and every node spring's moment", "[INT-023][INT-005]") {
    const integrity::MemberLimits m = cantilever();
    kernel::MemberEndLoads loads;
    loads.start = kernel::Wrench{{0.3, 1.5, 2.0}, {40.0, 3.0, 1.0}};
    loads.end = kernel::Wrench{{0.0, 2.0, 0.0}, {-40.0, 2.0, 0.0}};
    // Ends: |N| / A + |M_y| c_z / I_y + |M_z| c_y / I_z (ruling 25): 3.9e6 and 2.4e6 Pa.
    const double start = 40.0 / 1e-4 + 1.5 * 0.02 / 2e-8 + 2.0 * 0.01 / 1e-8;
    const double end = 40.0 / 1e-4 + 2.0 * 0.02 / 2e-8;
    CHECK_THAT(integrity::end_stress(loads.start, m), WithinRel(start, 1e-15));
    CHECK_THAT(integrity::end_stress(loads.end, m), WithinRel(end, 1e-15));
    // Straight nodes: every spring moment is zero and the ends govern; the out-of-plane moment interpolated between
    // the ends (1.5 (1 - s) - 2 s) stays below the larger end's.
    std::vector<kernel::BodyState> bodies(8);
    CHECK(integrity::node_moments(bodies, m) == std::vector<double>(8, 0.0));
    CHECK_THAT(integrity::peak_normal_stress(loads, bodies, m), WithinRel(start, 1e-15));
    // Nodes 3 to 7 turned by delta about z: spring 3 (between nodes 2 and 3, x = 3.5 l = 0.4375 m) carries
    // M = k delta = (E I / l) delta = 16000 * 5e-4 = 8 N m, the others none. There M_y = 1.5 (0.5625) - 2 (0.4375) =
    // -0.03125 N m: 4e5 + 0.03125 (0.02) / 2e-8 + 8 (0.01) / 1e-8 = 8.43125e6 Pa, above both ends.
    const double delta = 5e-4;
    for (std::size_t j = 3; j < bodies.size(); ++j) {
        bodies[j].orientation = about_z(delta);
    }
    const std::vector<double> moments = integrity::node_moments(bodies, m);
    REQUIRE(moments.size() == 8);
    CHECK_THAT(moments[3], WithinRel(8.0, 1e-12));
    CHECK(moments[2] == 0.0);
    CHECK(moments[4] == 0.0);
    CHECK_THAT(integrity::peak_normal_stress(loads, bodies, m), WithinRel(4e5 + 31250.0 + 8e6, 1e-12));
    // A quaternion and its negative are one rotation: the moment does not change sign or size.
    bodies[3].orientation = {-bodies[3].orientation[0], 0.0, 0.0, -bodies[3].orientation[3]};
    CHECK_THAT(integrity::node_moments(bodies, m)[3], WithinRel(8.0, 1e-12));
    // A member without node poses: its ends only.
    integrity::MemberLimits rigid = m;
    rigid.nodes.reset();
    CHECK_THAT(integrity::peak_normal_stress(loads, bodies, rigid), WithinRel(start, 1e-15));
    CHECK_THROWS_AS(integrity::node_moments(bodies, rigid), integrity::LimitsError);
    CHECK(integrity::compression(loads) == 40.0);
    loads.start.force[0] = -1.0;
    loads.end.force[0] = 2.0;
    CHECK(integrity::compression(loads) == 0.0);
    // Node moments by statics over the half segment l / 2 (ruling 35).
    CHECK_THAT(integrity::start_node_moment(loads.start, 0.125), WithinRel(-2.0 + 0.0625 * 3.0, 1e-15));
    CHECK_THAT(integrity::end_node_moment(loads.end, 0.125), WithinRel(0.0625 * 2.0, 1e-15));
}

TEST_CASE("node spring moments start from the root frame or the pin segment", "[INT-023][INT-005]") {
    integrity::MemberLimits m = cantilever(2.0);
    m.segments = 2;
    m.segment_length = 1.0;
    m.node_stiffness = 100.0;
    integrity::NodeChain chain;
    chain.first_body = 1;
    chain.count = 2;
    chain.parent_body = 0;
    chain.orientation = about_z(0.5);
    m.nodes = chain;
    // Parent turned 0.25 rad, placement 0.5 rad: the root frame is at 0.75 rad. Node 0 at 0.75 + 0.01, node 1 at
    // 0.75 + 0.01 - 0.03: spring 0 carries k (0.01) = 1 N m, spring 1 k (-0.03) = -3 N m.
    std::vector<kernel::BodyState> bodies(3);
    bodies[0].orientation = about_z(0.25);
    bodies[1].orientation = about_z(0.76);
    bodies[2].orientation = about_z(0.73);
    std::vector<double> moments = integrity::node_moments(bodies, m);
    REQUIRE(moments.size() == 2);
    CHECK_THAT(moments[0], WithinRel(1.0, 1e-9));
    CHECK_THAT(moments[1], WithinRel(-3.0, 1e-9));
    // No parent body: the world holds the placement (0.5 rad), so spring 0 reads 0.76 - 0.5.
    m.nodes->parent_body.reset();
    CHECK_THAT(integrity::node_moments(bodies, m)[0], WithinRel(26.0, 1e-9));
    // A pinned root: node 0 is the spring-free pin segment, so its N + 1 nodes give N springs from node 1.
    m.nodes->root = integrity::SupportCondition::pinned;
    moments = integrity::node_moments(bodies, m);
    REQUIRE(moments.size() == 1);
    CHECK_THAT(moments[0], WithinRel(-3.0, 1e-9));
    m.nodes->count = 3;
    CHECK_THROWS_AS(integrity::node_moments(bodies, m), integrity::LimitsError);
    m.nodes->count = 2;
    m.nodes->root = integrity::SupportCondition::clamped;
    m.nodes->parent_body = 5;
    CHECK_THROWS_AS(integrity::node_moments(bodies, m), integrity::LimitsError);
}

TEST_CASE("node-pose deflection measures from the root tangent or the pinned chord", "[INT-023][INT-006]") {
    integrity::MemberLimits m = cantilever(2.0);
    integrity::NodeChain chain;
    chain.first_body = 1;
    chain.count = 3;
    chain.parent_body = 0;
    chain.position = {0.5, 0.0, 0.0};
    m.nodes = chain;
    m.segment_length = 0.2;
    // Parent at (1, 1, 0) turned +90 degrees about z: the member's root point is (1, 1.5, 0) and its root tangent
    // world +y. Nodes 0.3 and 0.1 off that line; the last node (1.2, 2.2, 0) turned by the parent's rotation reaches
    // l / 2 = 0.1 further along world +y, so the far end is (1.2, 2.3, 0), 0.2 off: the largest, the deflection.
    const double half = std::sqrt(0.5);
    std::vector<kernel::BodyState> bodies(4);
    bodies[0].position = {1.0, 1.0, 0.0};
    bodies[0].orientation = {half, 0.0, 0.0, half};
    bodies[1].position = {1.0, 1.6, 0.0};
    bodies[2].position = {0.7, 1.8, 0.0};
    bodies[3].position = {1.2, 2.2, 0.0};
    bodies[3].orientation = {half, 0.0, 0.0, half};
    CHECK_THAT(integrity::node_deflection(bodies, m), WithinRel(0.3, 1e-12));
    bodies[2].position = {1.1, 1.8, 0.0};
    CHECK_THAT(integrity::node_deflection(bodies, m), WithinRel(0.2, 1e-12));
    // A pinned root: the chord from node 0 (the pin, (1, 1.6, 0)) to the far end (1.2, 2.3, 0); node 1 at (1.1, 1.8,
    // 0) sits off it by |cross(r, d)| with r = (0.1, 0.2), d = (0.2, 0.7) / |(0.2, 0.7)|: |0.1 * 0.7 - 0.2 * 0.2| /
    // sqrt(0.53).
    m.nodes->root = integrity::SupportCondition::pinned;
    CHECK_THAT(integrity::node_deflection(bodies, m), WithinRel(0.03 / std::sqrt(0.53), 1e-12));
    // No parent body: the world frame holds the placement.
    m.nodes->root = integrity::SupportCondition::clamped;
    m.nodes->parent_body.reset();
    m.nodes->position = {0.0, 0.0, 0.0};
    bodies[2].position = {1.1, -0.4, 0.0};
    CHECK_THAT(integrity::node_deflection(bodies, m), WithinRel(2.3, 1e-12));
    m.nodes->count = 4;
    CHECK_THROWS_AS(integrity::node_deflection(bodies, m), integrity::LimitsError);
    m.nodes.reset();
    CHECK_THROWS_AS(integrity::node_deflection(bodies, m), integrity::LimitsError);
}

TEST_CASE("a member without node poses is not evaluated for deflection", "[INT-023][INT-006]") {
    integrity::Limits limits;
    limits.deflection = true;
    integrity::MemberLimits m = cantilever();
    m.deflection_limit = 1e-6;
    m.nodes.reset();
    limits.members.push_back(m);
    const Run r = run(limits, [](double t) { return tip_loaded(1000.0 * t, 1.0); }, 5);
    CHECK(r.crossings.empty());
    REQUIRE(r.trace->peak_deflection.size() == 1);
    CHECK(std::isnan(r.trace->peak_deflection[0]));
}

// ===== Moment under a carried load (ruling 71; INT-005, INT-023) ========
//
// Samples of a member, L = 1 m, N = 8, l = 0.125 m: s = 0, (j + 1/2) l, L. A pinned-pinned member under P at a (world
// -y) has, exactly by statics, the reactions R0 = P b / L and R1 = P a / L (b = L - a) and M(s) = R0 s for s <= a,
// R1 (L - s) for s >= a, spring sense; the start wrench is F_y0 = R0, M_z0 = 0 (M(0) = -M_z0 + s F_y0) and the end
// wrench F_y1 = R1, M_z1 = 0 (M(s) = M_z1 + (L - s) F_y1). The spring moments are M at the springs (the lever split
// is statically equivalent outside the loaded segment, mechsim/integrity/beam.py header). The intersection of the
// neighbouring segments' lines is then the tent's apex: P a b / L at s = a, to rounding (1e-12 relative: a few
// operations on numbers of one magnitude).

namespace {

constexpr double span_length = 1.0;
constexpr std::size_t span_segments = 8;
constexpr double span_l = span_length / static_cast<double>(span_segments);

integrity::MemberLimits span_member(bool carries, integrity::SupportCondition root) {
    integrity::MemberLimits m = cantilever(span_length);
    integrity::NodeChain chain;
    chain.root = root;
    chain.count = span_segments;
    if (root == integrity::SupportCondition::pinned) {
        chain.count = span_segments + 1;
    }
    chain.carries = carries;
    m.nodes = chain;
    return m;
}

std::vector<double> sampled(const std::function<double(double)>& moment) {
    std::vector<double> out;
    for (std::size_t j = 0; j < span_segments; ++j) {
        out.push_back(moment((static_cast<double>(j) + 0.5) * span_l));
    }
    return out;
}

kernel::MemberEndLoads pinned_point(double load, double a) {
    kernel::MemberEndLoads loads;
    loads.start.force = {0.0, load * (span_length - a) / span_length, 0.0};
    loads.end.force = {0.0, load * a / span_length, 0.0};
    return loads;
}

double pinned_moment(double load, double a, double s) {
    if (s <= a) {
        return load * (span_length - a) / span_length * s;
    }
    return load * a / span_length * (span_length - s);
}

// Pinned-pinned node bodies whose springs carry the given moments: node 0 the pin segment, node j + 1 turned by the sum
// of the spring rotations up to spring j.
std::vector<kernel::BodyState> pinned_nodes(const std::vector<double>& moments, double stiffness) {
    std::vector<kernel::BodyState> bodies(moments.size() + 1);
    double angle = 0.0;
    for (std::size_t j = 0; j < moments.size(); ++j) {
        angle += moments[j] / stiffness;
        bodies[j + 1].orientation = about_z(angle);
    }
    return bodies;
}


// The chain's samples under a force P (down, sagging positive) and a couple C at a on the pinned span (beam.py
// header): the true moment M(x) = R0 x - P (x - a)+ - C H(x - a), R0 = P b / L + C / L, so M(a+) - M(a-) = -C in the
// spring sense; the spring at x_{k+1} (the lever node above a) has only w_k C on its left where the point couple puts
// all of C, so it reads w_{k+1} C more. Every other sample is the true moment.
std::vector<double> chain_samples(double load, double couple, double a) {
    const auto moment = [&](double s) {
        double value = (load * (span_length - a) / span_length + couple / span_length) * s;
        if (s > a) {
            value += -load * (s - a) - couple;
        }
        return value;
    };
    std::vector<double> nodes{0.0};
    for (std::size_t j = 0; j < span_segments; ++j) {
        nodes.push_back((static_cast<double>(j) + 0.5) * span_l);
    }
    nodes.push_back(span_length);
    std::size_t k = 0;
    while (k + 2 < nodes.size() && nodes[k + 1] <= a) {
        ++k;
    }
    const double upper = (a - nodes[k]) / (nodes[k + 1] - nodes[k]);
    std::vector<double> out = sampled(moment);
    if (k + 1 >= 1 && k + 1 <= span_segments) {
        out[k] += upper * couple;
    }
    return out;
}

kernel::MemberEndLoads pinned_with_couple(double load, double couple, double a) {
    kernel::MemberEndLoads loads = pinned_point(load, a);
    loads.start.force[1] += couple / span_length;
    loads.end.force[1] -= couple / span_length;
    return loads;
}

}  // namespace

TEST_CASE("the moments just left and right of a carried force and couple are the clean samples' lines at s",
          "[INT-005][INT-023]") {
    const integrity::MemberLimits m = span_member(true, integrity::SupportCondition::pinned);
    const double load = 120.0;
    const double couple = 9.0;
    // a in the start's half segment (the pin's start line), interior segments, on a spring (0.3125 = 2.5 l) and in
    // the end's half segment (the end's line): M(a-) = P a b / L + C a / L, M(a+) = P a b / L - C b / L exactly, the
    // step -C of the header.
    for (const double a : {0.03, 0.1, 0.3125, 0.5, 0.77, 0.98}) {
        CAPTURE(a);
        const double b = span_length - a;
        const std::vector<integrity::CarriedAt> carried{{a, -load, couple}};
        const integrity::CarriedReads reads = integrity::carried_reads(
            pinned_with_couple(load, couple, a), chain_samples(load, couple, a), m, carried);
        REQUIRE(reads.reads.size() == 2);
        CHECK_THAT(reads.reads[0].value, WithinAbs(load * a * b / span_length + couple * a / span_length, 1e-12));
        CHECK_THAT(reads.reads[1].value, WithinAbs(load * a * b / span_length - couple * b / span_length, 1e-12));
        CHECK(reads.reads[0].position == a);
        CHECK(reads.bound == 0.0);
    }
}

TEST_CASE("two forces within two segments read exactly by the chord, whatever their signs", "[INT-005][INT-023]") {
    // P1 = 60 N at 0.46, P2 = -40 N at 0.54 (one segment) and at 0.62 (the next): between them the moment is linear,
    // so the peak is the larger of M(s1) and M(s2), which lines A and B give.
    const integrity::MemberLimits m = span_member(true, integrity::SupportCondition::pinned);
    for (const double second : {0.54, 0.62}) {
        CAPTURE(second);
        const auto moment = [&](double s) { return pinned_moment(60.0, 0.46, s) + pinned_moment(-40.0, second, s); };
        kernel::MemberEndLoads loads = pinned_point(60.0, 0.46);
        const kernel::MemberEndLoads other = pinned_point(-40.0, second);
        loads.start.force[1] += other.start.force[1];
        loads.end.force[1] += other.end.force[1];
        const std::vector<integrity::CarriedAt> carried{{second, 40.0, 0.0}, {0.46, -60.0, 0.0}};
        const integrity::CarriedReads reads = integrity::carried_reads(loads, sampled(moment), m, carried);
        REQUIRE(reads.reads.size() == 2);
        CHECK_THAT(reads.reads[0].value, WithinRel(moment(0.46), 1e-12));
        CHECK_THAT(reads.reads[1].value, WithinRel(moment(second), 1e-12));
        CHECK(reads.bound == 0.0);
    }
}

TEST_CASE("a couple in a cluster, three loads and a clamped root's stub are bounded", "[INT-005][INT-023]") {
    // U = sum abs(F) (b - a) / 4 + sum abs(C), [a, b] = [x_{k1}, x_{km + 2}] the clean samples bracketing the cluster;
    // in a clamped root's stub plus abs(F) l / 8 + abs(C) for the load there (beam.py header).
    const integrity::MemberLimits pinned = span_member(true, integrity::SupportCondition::pinned);
    const kernel::MemberEndLoads none;
    const std::vector<double> flat(span_segments, 0.0);
    // 0.46 and 0.54 in [0.4375, 0.5625] (k = 4): a = 0.4375, b = x_6 = 0.6875.
    const std::vector<integrity::CarriedAt> coupled{{0.46, -10.0, 2.0}, {0.54, -20.0, 0.0}};
    CHECK_THAT(integrity::carried_reads(none, flat, pinned, coupled).bound, WithinRel(30.0 * 0.25 / 4.0 + 2.0, 1e-12));
    const std::vector<integrity::CarriedAt> crowded{{0.46, -10.0, 0.0}, {0.54, -20.0, 0.0}, {0.70, 5.0, 0.0}};
    // k = 4, 4, 6 (0.70 >= x_6 = 0.6875), one cluster: b = x_8 = 0.9375.
    CHECK_THAT(integrity::carried_reads(none, flat, pinned, crowded).bound,
               WithinRel(35.0 * (0.9375 - 0.4375) / 4.0, 1e-12));
    const integrity::MemberLimits clamped = span_member(true, integrity::SupportCondition::clamped);
    const std::vector<integrity::CarriedAt> stub{{0.03, -50.0, 1.0}};
    // k = 0: a = 0, b = x_2 = 0.1875; one read, the right line.
    const integrity::CarriedReads reads = integrity::carried_reads(none, flat, clamped, stub);
    CHECK(reads.reads.size() == 1);
    CHECK_THAT(reads.bound, WithinRel(50.0 * 0.1875 / 4.0 + 1.0 + 50.0 * span_l / 8.0 + 1.0, 1e-12));
    // Clusters more than two segments apart are separate and exact.
    const std::vector<integrity::CarriedAt> apart{{0.1, -10.0, 1.0}, {0.6, -20.0, 1.0}};
    CHECK(integrity::carried_reads(none, flat, pinned, apart).bound == 0.0);
    CHECK(integrity::carried_reads(none, flat, pinned, apart).reads.size() == 4);
}

TEST_CASE("only a carrying member reads its carried loads", "[INT-005][INT-023]") {
    // Pinned-pinned, P = 120 N at 0.5 (mid-segment): the larger spring moment is R0 (0.4375) = 26.25 N m, below
    // P a b / L = 30 N m under the load; stress M c_y / I_z (no axial force, no M_y). A bound adds U c_y / I_z.
    const double load = 120.0;
    const double a = 0.5;
    const std::vector<double> moments = sampled([&](double s) { return pinned_moment(load, a, s); });
    const integrity::MemberLimits plain = span_member(false, integrity::SupportCondition::pinned);
    const std::vector<kernel::BodyState> bodies = pinned_nodes(moments, plain.node_stiffness);
    const std::vector<integrity::CarriedAt> carried{{a, -load, 0.0}};
    const double spring = pinned_moment(load, a, 0.4375) * plain.c_y / plain.i_z;
    CHECK_THAT(integrity::peak_normal_stress(pinned_point(load, a), bodies, plain, {}, carried), WithinRel(spring, 1e-9));
    const integrity::MemberLimits carrying = span_member(true, integrity::SupportCondition::pinned);
    const double apex = load * a * (span_length - a) / span_length * plain.c_y / plain.i_z;
    CHECK_THAT(integrity::peak_normal_stress(pinned_point(load, a), bodies, carrying, {}, carried), WithinRel(apex, 1e-9));
    CHECK_THAT(integrity::peak_normal_stress(pinned_point(load, a), bodies, carrying), WithinRel(spring, 1e-9));
    const std::vector<integrity::CarriedAt> pair{{0.46, -load, 1.0}, {0.54, 0.0, 0.0}};
    const double bound = load * 0.25 / 4.0 + 1.0;
    CHECK(integrity::peak_normal_stress(pinned_point(load, a), bodies, carrying, {}, pair) >=
          spring + bound * plain.c_y / plain.i_z);
}

TEST_CASE("a carried joint's foot point, force and couple come from its pose and reaction", "[INT-005][INT-023]") {
    // Root frame: parent body 0 at (1, 2, 0) turned 90 degrees about z, member placed at (0.5, 0, 0) in it: origin
    // (1, 2.5, 0), x = world y, y = world -x. Child body 1 at (0.8, 2.9, 0): s = 0.4, foot (1, 2.9, 0), arm (-0.2, 0, 0).
    // Reaction on the child R = (3, 0, 0) (world), M = (0, 0, 0.5): load F = (-3, 0, 0), F . y = 3; C = (-M + arm x F).z
    // = -0.5 + (-0.2)(0) - 0 (-3) = -0.5.
    integrity::MemberLimits m = span_member(true, integrity::SupportCondition::clamped);
    integrity::NodeChain chain;
    chain.root = integrity::SupportCondition::clamped;
    chain.count = span_segments;
    chain.carries = true;
    chain.parent_body = 0;
    chain.position = {0.5, 0.0, 0.0};
    chain.carried = {integrity::CarriedJoint{1, 2}};
    m.nodes = chain;
    std::vector<kernel::BodyState> bodies(2);
    bodies[0].position = {1.0, 2.0, 0.0};
    bodies[0].orientation = about_z(std::numbers::pi / 2.0);
    bodies[1].position = {0.8, 2.9, 0.0};
    std::vector<kernel::JointForce> joints(3);
    joints[2].reaction.force = {3.0, 0.0, 0.0};
    joints[2].reaction.moment = {0.0, 0.0, 0.5};
    const std::vector<integrity::CarriedAt> out = integrity::carried_at(bodies, joints, m);
    REQUIRE(out.size() == 1);
    CHECK_THAT(out[0].position, WithinAbs(0.4, 1e-12));
    CHECK_THAT(out[0].force, WithinAbs(3.0, 1e-12));
    CHECK_THAT(out[0].couple, WithinAbs(-0.5, 1e-12));
    joints.pop_back();
    CHECK_THROWS_AS(integrity::carried_at(bodies, joints, m), integrity::LimitsError);
}

TEST_CASE("a yielded spring's moment is k times its elastic rotation", "[INT-005][INT-028]") {
    const integrity::MemberLimits m = cantilever();
    std::vector<kernel::BodyState> bodies(8);
    for (kernel::BodyState& body : bodies) {
        body.orientation = about_z(1e-3);
    }
    const std::vector<double> offsets{4e-4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const std::vector<double> moments = integrity::node_moments(bodies, m, offsets);
    CHECK_THAT(moments[0], WithinRel(m.node_stiffness * 6e-4, 1e-12));
    CHECK(moments[1] == 0.0);
    const std::vector<double> short_offsets{1e-4};
    CHECK_THROWS_AS(integrity::node_moments(bodies, m, short_offsets), integrity::LimitsError);
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
    seen.reserve(r.crossings.size());
    for (const integrity::Crossing& c : r.crossings) {
        seen.push_back(c.element + " " + c.quantity + " " + std::to_string(c.time.count()));
    }
    CHECK(seen == std::vector<std::string>{"arm deflection 7", "second deflection 7", "second stress 51"});
    CHECK(integrity::crossing_detail(r.crossings[0]).starts_with(
        R"({"limit": 0.00105, "member": "arm", "quantity": "deflection", "tier": 1, "value": )"));
}

TEST_CASE("a crossing before the evaluation start is recorded and does not count", "[TST-044][INT-024][INT-026]") {
    // The ramp of the first oracle crosses at tick 11 (t* = 10.5 ms). With the start at tick 20 that crossing is one
    // before_evaluation event, and the first counted crossing is the first row at or after the start still past the
    // limit: tick 20, which halts the run there. The peak counts rows from tick 20 on.
    integrity::Limits limits;
    limits.stress = true;
    limits.members.push_back(cantilever());
    const integrity::MemberLimits& m = limits.members[0];
    const double rate = m.allowable_stress * m.i_z / (0.0105 * m.length * m.c_y);
    const auto ramp = [rate](double t) { return tip_loaded(rate * t, 1.0); };
    const Run r = run(limits, ramp, 30, true, kernel::SimTime{20});
    REQUIRE(r.trace->before_evaluation.size() == 1);
    CHECK(r.trace->before_evaluation[0].time.count() == 11);
    REQUIRE(r.crossings.size() == 1);
    CHECK(r.crossings[0].time.count() == 20);
    CHECK(r.report.end_reason == kernel::EndReason::party_state);
    std::vector<std::pair<std::int64_t, std::string>> events;
    for (const kernel::TickRecord& row : r.rows) {
        for (const kernel::RecordedEvent& event : row.events) {
            CHECK(event.source == "integrity");
            events.emplace_back(row.time.count(), event.kind);
        }
    }
    CHECK(events == std::vector<std::pair<std::int64_t, std::string>>{{11, "integrity_before_evaluation"},
                                                                       {20, "integrity"}});
    CHECK_THAT(r.trace->peak_stress[0], WithinRel(rate * 0.020 * m.length * m.c_y / m.i_z, 1e-12));
    // A load that crosses and falls back before the start: one early event, no counted crossing, no halt, and the
    // peak only of the rows from the start (zero load there).
    const auto pulse = [rate](double t) {
        double p = rate * t;
        if (t > 0.015) {
            p = 0.0;
        }
        return tip_loaded(p, 1.0);
    };
    const Run quiet = run(limits, pulse, 30, true, kernel::SimTime{20});
    CHECK(quiet.trace->before_evaluation.size() == 1);
    CHECK(quiet.crossings.empty());
    CHECK(quiet.report.end_reason == kernel::EndReason::max_duration);
    CHECK(quiet.trace->peak_stress[0] == 0.0);
    // The start at 0 (the default): every row counts, as the first oracle.
    const Run all = run(limits, ramp, 30, false, kernel::SimTime{0});
    CHECK(all.trace->before_evaluation.empty());
    REQUIRE(all.crossings.size() == 1);
    CHECK(all.crossings[0].time.count() == 11);
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

TEST_CASE("no yield request before the evaluation start; the same load after it yields", "[INT-028][TST-044]") {
    // A release transient: P = 1000 N for the first 10 ticks, |M_n| = 1000 arm = 937.5 N m > M_y = 250 N m, then 0.
    // With the start at tick 20 the transient is an integrity_before_evaluation stress event only: no request, so the
    // physics party would apply no offset and list no model_change. The same pulse at ticks 30 to 39, after the start,
    // yields: one request at its first row (tick 30), offset -(1000 arm - M_y) / k (sign of M_n, ruling 35).
    integrity::Limits limits;
    limits.yield_offset = true;
    limits.stress = true;
    limits.members.push_back(cantilever());
    const integrity::MemberLimits& m = limits.members[0];
    const auto early = [](double t) {
        double p = 0.0;
        if (t < 0.0105) {
            p = 1000.0;
        }
        return tip_loaded(p, 1.0);
    };
    const Run quiet = run(limits, early, 40, false, kernel::SimTime{20});
    CHECK(quiet.trace->requests.empty());
    CHECK(quiet.crossings.empty());
    CHECK(quiet.trace->before_evaluation.size() == 1);
    const auto late = [](double t) {
        double p = 0.0;
        if (t > 0.0295 && t < 0.0395) {
            p = 1000.0;
        }
        return tip_loaded(p, 1.0);
    };
    const Run yielded = run(limits, late, 50, false, kernel::SimTime{20});
    REQUIRE(yielded.trace->requests.size() == 1);
    const double arm = m.length - m.segment_length / 2.0;
    const double yield_moment = m.yield_strength * m.i_z / m.c_y;
    CHECK_THAT(yielded.trace->requests[0].value, WithinRel(-(1000.0 * arm - yield_moment) / m.node_stiffness, 1e-12));
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

TEST_CASE("the yield offset makes no request while the load stays below its running peak", "[INT-028]") {
    // P rises as 400 t to 0.8 s (peak |M_n| = 320 arm), falls to 160 N at 1.2 s, rises again to 300 N (below the
    // peak) and holds: the second rise never passes the running peak, so the requests stop at tick 800 (ruling 35).
    integrity::Limits limits;
    limits.yield_offset = true;
    limits.members.push_back(cantilever());
    const auto load = [](double t) {
        double p = 400.0 * t;
        if (t > 0.8 && t <= 1.2) {
            p = 400.0 * (1.6 - t);
        }
        if (t > 1.2) {
            p = 400.0 * std::min(t - 0.8, 0.75);
        }
        return tip_loaded(p, 1.0);
    };
    const Run r = run(limits, load, 1800);
    REQUIRE(r.trace->requests.size() == 800 - 667 + 1);
    const integrity::MemberLimits& m = limits.members[0];
    const double arm = m.length - m.segment_length / 2.0;
    CHECK_THAT(r.trace->requests.back().value, WithinRel(-(400.0 * 0.8 * arm - 250.0) / m.node_stiffness, 1e-12));
}

TEST_CASE("a larger reversed peak replaces the yield offset with the new peak's sign", "[INT-028]") {
    // Ruling 35 placeholder (spec row Tier 2 yield): the offset is sign(M_n) (peak - M_y) / k with the running peak
    // of abs(M_n), so a reversed load whose magnitude passes the old peak sets the offset to the reversed sign. P
    // rises to 320 N at tick 800, then runs linearly to -400 N at tick 1000 and holds.
    integrity::Limits limits;
    limits.yield_offset = true;
    limits.members.push_back(cantilever());
    const integrity::MemberLimits& m = limits.members[0];
    const auto load = [](double t) {
        double p = 400.0 * t;
        if (t > 0.8) {
            p = 320.0 - 3600.0 * (std::min(t, 1.0) - 0.8);
        }
        return tip_loaded(p, 1.0);
    };
    const Run r = run(limits, load, 1100);
    const double arm = m.length - m.segment_length / 2.0;
    const std::size_t first_rise = 800 - 667 + 1;
    REQUIRE(r.trace->requests.size() > first_rise);
    CHECK(r.trace->requests[first_rise - 1].value < 0.0);
    const kernel::ModelChangeRequest& last = r.trace->requests.back();
    CHECK(last.index == m.first_spring);
    CHECK_THAT(last.value, WithinRel((400.0 * arm - 250.0) / m.node_stiffness, 1e-12));
    for (std::size_t k = first_rise; k < r.trace->requests.size(); ++k) {
        CHECK(r.trace->requests[k].value > 0.0);
        CHECK(std::abs(r.trace->requests[k].value) > std::abs(r.trace->requests[k - 1].value));
    }
}

TEST_CASE("a value exactly at the limit is not a crossing and the next representable value above is", "[INT-024]") {
    // |F| = sqrt(fl(F^2)) = F exactly for a single nonzero component (correctly rounded square and root), so the load
    // equals the limit 250 N bit for bit on ticks 1 to 5 and is nextafter(250, +inf) from tick 6.
    integrity::Limits limits;
    limits.fastener = true;
    limits.fastenings.push_back(integrity::FasteningLimits{"bolts", 0, 250.0, std::nullopt});
    const double above = std::nextafter(250.0, 1e9);
    const auto load = [above](double t) {
        kernel::PhysicsOutputs out;
        out.joints.resize(1);
        double f = 250.0;
        if (t > 0.0055) {
            f = above;
        }
        out.joints[0].reaction.force = {0.0, f, 0.0};
        return out;
    };
    const Run r = run(limits, load, 10);
    REQUIRE(r.crossings.size() == 1);
    CHECK(r.crossings[0].time.count() == 6);
    CHECK(r.crossings[0].value == above);
    CHECK(r.crossings[0].limit == 250.0);
}

TEST_CASE("the party reads a yielded spring by its elastic rotation from the tick its offset applies",
          "[INT-028][INT-023]") {
    // Constant outputs: the start wrench gives the root spring M_n = -M_z0 = 500 N m = 2 M_y (M_y = 250 N m), so the
    // first tick requests o = (500 - 250) / k (k = 16000 N m / rad); the node poses turn every node by theta with
    // k theta = 750 N m (the poses need not match the wrench here: the test separates the two readings). From the
    // second tick (physics applies the request before its next step) the root spring reads k (theta - o) = 500 N m,
    // its stress 500 c_y / I_z = 5e8 Pa, equal to the start end's; the total rotation would read 7.5e8 Pa. The poses
    // turn only from the third tick, after the offset applies (no request precedes the evaluation start, TST-044).
    integrity::Limits limits;
    limits.stress = true;
    limits.yield_offset = true;
    limits.members.push_back(cantilever());
    const integrity::MemberLimits& m = limits.members[0];
    const double theta = 750.0 / m.node_stiffness;
    const auto load = [theta](double t) {
        kernel::PhysicsOutputs out;
        kernel::MemberEndLoads loads;
        loads.start.moment = {0.0, 0.0, -500.0};
        out.members.push_back(loads);
        out.bodies.resize(8);
        if (t > 0.0015) {
            for (kernel::BodyState& body : out.bodies) {
                body.orientation = about_z(theta);
            }
        }
        return out;
    };
    const Run r = run(limits, load, 5);
    REQUIRE(r.trace->requests.size() == 1);
    CHECK_THAT(r.trace->requests[0].value, WithinRel(250.0 / m.node_stiffness, 1e-12));
    CHECK_THAT(r.trace->peak_stress[0], WithinRel(500.0 * m.c_y / m.i_z, 1e-9));
}

// ===== Limits table ======================================================

TEST_CASE("the limits table parses and refuses malformed entries", "[INT-023][INT-006]") {
    const std::string member =
        R"({"name": "arm", "index": 0, "area": 1e-4, "i_y": 2e-8, "i_z": 1e-8, "c_y": 0.01, "c_z": 0.02, )"
        R"("length": 1.0, "elastic_modulus": 2e11, "effective_length_factor": 2.0, "allowable_stress": 1e8, )"
        R"("yield_strength": 2.5e8, "deflection_limit": null, "supports": [{"at": 0.0, "condition": "clamped"}], )"
        R"("first_spring": 2, "segments": 8, "segment_length": 0.125, "node_stiffness": 16000.0, "nodes": {"first_body": )"
        R"(5, "count": 9, "root": "pinned", "parent_body": null, "position": [0, 1, 0], "orientation": [1, 0, 0, 0], "carries": false, "carried": []}})";
    const auto doc = [](const std::string& members, const std::string& checks = "true",
                        const std::string& bodies = R"(, "bodies": 14)") {
        return R"({"format": "mechsim-integrity-limits-1", "minimum_safety_factor": 1.5, "checks": {"stress": )" +
               checks + R"(, "buckling": false, "deflection": false, "fastener": true, "yield_offset": false}, )"
               R"("members": [)" + members + R"(], "fastenings": [{"name": "bolts", "joint": 3, "load_limit": 10, "member": null}])" +
               bodies + "}";
    };
    const integrity::Limits limits = integrity::parse_limits(doc(member));
    REQUIRE(limits.members.size() == 1);
    CHECK(limits.members[0].first_spring == 2);
    CHECK_FALSE(limits.members[0].deflection_limit.has_value());
    CHECK(limits.minimum_safety_factor == 1.5);
    CHECK(limits.stress);
    CHECK(limits.fastenings[0].joint == 3);
    const auto& nodes = limits.members[0].nodes;
    REQUIRE(nodes.has_value());
    if (nodes.has_value()) {
        CHECK(nodes->first_body == 5);
        CHECK(nodes->count == 9);
        CHECK(nodes->root == integrity::SupportCondition::pinned);
        CHECK_FALSE(nodes->parent_body.has_value());
        CHECK(nodes->position[1] == 1.0);
        CHECK(nodes->carried.empty());
    }
    CHECK_FALSE(limits.fastenings[0].member.has_value());
    // Carried joints (ruling 75): body and joint indices; listed with carries false, or a mount's member outside the
    // members, is refused.
    std::string carrying = member;
    carrying.replace(carrying.find(R"("carries": false, "carried": [])"), std::string_view(R"("carries": false, "carried": [])").size(),
                     R"("carries": true, "carried": [{"body": 4, "joint": 1}])");
    const integrity::Limits carried = integrity::parse_limits(doc(carrying));
    const auto& carried_nodes = carried.members[0].nodes;
    REQUIRE(carried_nodes.has_value());
    if (carried_nodes.has_value()) {
        REQUIRE(carried_nodes->carried.size() == 1);
        CHECK(carried_nodes->carried[0].body == 4);
        CHECK(carried_nodes->carried[0].joint == 1);
    }
    std::string listed = member;
    listed.replace(listed.find(R"("carried": [])"), std::string_view(R"("carried": [])").size(), R"("carried": [{"body": 4, "joint": 1}])");
    CHECK_THROWS_AS(integrity::parse_limits(doc(listed)), integrity::LimitsError);
    std::string mounted = doc(member);
    mounted.replace(mounted.find(R"("member": null)"), 14, R"("member": 0)");
    CHECK(integrity::parse_limits(mounted).fastenings[0].member == std::optional<std::size_t>(0));
    mounted.replace(mounted.find(R"("member": 0)"), 11, R"("member": 1)");
    CHECK_THROWS_AS(integrity::parse_limits(mounted), integrity::LimitsError);
    std::string rigid = member;
    rigid.replace(rigid.find(R"("nodes": {)"), rigid.size() - rigid.find(R"("nodes": {)"), R"("nodes": null})");
    CHECK_FALSE(integrity::parse_limits(doc(rigid)).members[0].nodes.has_value());
    CHECK_FALSE(integrity::parse_limits(doc(rigid, "true", "")).bodies.has_value());
    CHECK(integrity::parse_limits(doc(member)).bodies == std::optional<std::size_t>(14));
    CHECK_THROWS_AS(integrity::parse_limits(doc(member, "true", "")), integrity::LimitsError);
    CHECK_THROWS_AS(integrity::parse_limits(doc(member, "true", R"(, "bodies": null)")), integrity::LimitsError);
    for (const auto& [from, to] : std::vector<std::pair<std::string, std::string>>{
             {R"("count": 9)", R"("count": 0)"},
             {R"("root": "pinned")", R"("root": "free")"},
             {"[1, 0, 0, 0]", "[2, 0, 0, 0]"},
             {"[0, 1, 0]", "[0, 1]"},
             {R"("parent_body": null)", R"("parent_body": -1)"}}) {
        std::string bad = member;
        bad.replace(bad.find(from), from.size(), to);
        CAPTURE(to);
        CHECK_THROWS_AS(integrity::parse_limits(doc(bad)), integrity::LimitsError);
    }
    std::string missing = member;
    missing.replace(missing.find(R"(, "nodes")"), missing.size() - missing.find(R"(, "nodes")"), "}");
    CHECK_THROWS_AS(integrity::parse_limits(doc(missing)), integrity::LimitsError);
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
    if (r.report.fault.has_value()) {
        CHECK(r.report.fault.value().party() == "integrity");
    }
}

TEST_CASE("the limits table's body count is exact, never an upper bound", "[INT-006]") {
    const std::string doc = R"({"format": "mechsim-integrity-limits-1", "minimum_safety_factor": 1.5, "checks": )"
                            R"({"stress": false, "buckling": false, "deflection": false, "fastener": false, )"
                            R"("yield_offset": false}, "members": [], "fastenings": [])";
    CHECK_FALSE(integrity::parse_limits(doc + "}").bodies.has_value());
    CHECK_FALSE(integrity::parse_limits(doc + R"(, "bodies": null})").bodies.has_value());
    CHECK(integrity::parse_limits(doc + R"(, "bodies": 36})").bodies == std::optional<std::size_t>(36));
    CHECK_THROWS_AS(integrity::parse_limits(doc + R"(, "bodies": -1})"), integrity::LimitsError);
    CHECK_THROWS_AS(integrity::parse_limits(doc + R"(, "bodies": 2.5})"), integrity::LimitsError);
    // tip_loaded publishes the 8 node bodies; 8 runs, 7 (fewer) and 10 (more: a bound would accept it) fault.
    for (const std::size_t expected : {std::size_t{7}, std::size_t{8}, std::size_t{10}}) {
        integrity::Limits limits;
        limits.deflection = true;
        limits.members.push_back(cantilever());
        limits.bodies = expected;
        const Run r = run(limits, [](double t) { return tip_loaded(t, 1.0); }, 5);
        CAPTURE(expected);
        if (expected == 8) {
            CHECK(r.report.outcome != kernel::RunOutcome::run_error);
            continue;
        }
        CHECK(r.report.outcome == kernel::RunOutcome::run_error);
        REQUIRE(r.report.fault.has_value());
        if (r.report.fault.has_value()) {
            const std::string cause = r.report.fault.value().cause();
            CHECK(cause.find("publishes 8 bodies, the limits table expects " + std::to_string(expected)) !=
                  std::string::npos);
        }
    }
}

// ===== Validity from a neutral source (rulings 75, 79; INT-026) ==========

namespace {

// A fake of the engine's flag: member 0 is flagged from the party's `from`-th evaluated step on (one call per member
// per step, the step stamped `from` or `from` + 1 ms; every threshold below sits at least one tick clear of the
// crossings it separates); any other index is not held.
class FlagFrom final : public integrity::ValiditySource {
public:
    explicit FlagFrom(std::int64_t from) : from_(from) {}

    [[nodiscard]] bool not_valid(std::size_t member) const override {
        if (member != 0) {
            throw integrity::ValidityError("member " + std::to_string(member) + " of 1");
        }
        ++calls_;
        return calls_ >= from_;
    }

private:
    std::int64_t from_;
    mutable std::int64_t calls_ = 0;
};

// The cantilever's root stress crosses at 11 ms (the first oracle above) and a mount holding it, rated 250 N under
// abs(F) = k t with t* = 7.5 ms, at 8 ms.
integrity::Limits held_member() {
    integrity::Limits limits;
    limits.stress = true;
    limits.fastener = true;
    limits.members.push_back(cantilever());
    limits.fastenings.push_back(integrity::FasteningLimits{"mount", 1, 250.0, std::size_t{0}});
    return limits;
}

kernel::PhysicsOutputs held_load(double t) {
    const integrity::MemberLimits m = cantilever();
    const double stress_rate = m.allowable_stress * m.i_z / (0.0105 * m.length * m.c_y);
    kernel::PhysicsOutputs out = tip_loaded(stress_rate * t, 1.0);
    out.joints.resize(2);
    out.joints[1].reaction.force = {0.0, 250.0 / 0.0075 * t, 0.0};
    return out;
}

std::vector<std::string> event_kinds(const Run& r) {
    std::vector<std::string> kinds;
    for (const kernel::TickRecord& row : r.rows) {
        for (const kernel::RecordedEvent& event : row.events) {
            kinds.push_back(event.kind + "@" + std::to_string(row.time.count()));
        }
    }
    return kinds;
}

}  // namespace

TEST_CASE("a crossing before the source flags the member counts; after it, it and its mount's are not valid",
          "[INT-026][INT-024]") {
    // Flag from 12 ms: the mount's crossing at 8 ms and the member's at 11 ms both precede it and count.
    const Run before = run(held_member(), held_load, 20, false, kernel::SimTime{0}, std::make_shared<FlagFrom>(12));
    CHECK(event_kinds(before) == std::vector<std::string>{"integrity@8", "integrity@11"});
    CHECK(before.trace->not_valid.empty());
    // Flag from 9 ms: the mount's crossing at 8 ms counts; the member's at 11 ms is integrity_not_valid.
    const Run between = run(held_member(), held_load, 20, false, kernel::SimTime{0}, std::make_shared<FlagFrom>(9));
    CHECK(event_kinds(between) == std::vector<std::string>{"integrity@8", "integrity_not_valid@11"});
    // Flag from 5 ms: the mount follows its member in the party (not only at parsing): both not valid, nothing counted,
    // and under halting nothing halts.
    const Run after = run(held_member(), held_load, 20, true, kernel::SimTime{0}, std::make_shared<FlagFrom>(5));
    CHECK(event_kinds(after) == std::vector<std::string>{"integrity_not_valid@8", "integrity_not_valid@11"});
    CHECK(after.crossings.empty());
    REQUIRE(after.trace->not_valid.size() == 2);
    CHECK(after.trace->not_valid[0].element == "mount");
    CHECK(after.trace->not_valid[1].element == "arm");
    CHECK(after.report.end_reason == kernel::EndReason::max_duration);
}

TEST_CASE("a member index the validity source does not hold is an error, never valid", "[INT-026]") {
    integrity::Limits limits = held_member();
    limits.members[0].index = 1;
    const auto two = [](double t) {
        kernel::PhysicsOutputs out = held_load(t);
        out.members.push_back(out.members[0]);
        return out;
    };
    const Run r = run(limits, two, 5, false, kernel::SimTime{0}, std::make_shared<FlagFrom>(100));
    CHECK(r.report.outcome == kernel::RunOutcome::run_error);
    REQUIRE(r.report.fault.has_value());
    if (r.report.fault.has_value()) {
        CHECK(r.report.fault.value().party() == "integrity");
    }
    CHECK_THROWS_AS(FlagFrom(1).not_valid(1), integrity::ValidityError);
}

// ===== Own weight under a carried couple (ruling 79; INT-005) ============

TEST_CASE("a carried couple applied inside a hogging stretch marks the read as possibly low", "[INT-005][INT-023]") {
    integrity::MemberLimits m = span_member(true, integrity::SupportCondition::clamped);
    m.nodes->hogging_bound = 0.5;
    m.nodes->hogging = {{0.0, 0.2}, {0.8, 1.0}};
    const std::vector<integrity::CarriedAt> inside{{0.1, -10.0, 2.0}};
    const std::vector<integrity::CarriedAt> at_end{{0.8, -10.0, 2.0}};
    const std::vector<integrity::CarriedAt> outside{{0.5, -10.0, 2.0}};
    const std::vector<integrity::CarriedAt> no_couple{{0.1, -10.0, 0.0}};
    // At the couple floor, 1e-9 abs(F) L = 1e-8 N m: not a couple; just above it, one.
    const std::vector<integrity::CarriedAt> at_floor{{0.1, -10.0, 1e-8}};
    const std::vector<integrity::CarriedAt> pure{{0.1, 0.0, 1e-300}};
    CHECK(integrity::hogs(m, inside));
    CHECK(integrity::hogs(m, at_end));
    CHECK_FALSE(integrity::hogs(m, outside));
    CHECK_FALSE(integrity::hogs(m, no_couple));
    CHECK_FALSE(integrity::hogs(m, at_floor));
    CHECK(integrity::hogs(m, pure));
    CHECK_FALSE(integrity::hogs(m, {}));
    m.nodes->hogging_bound = 0.0;
    CHECK_FALSE(integrity::hogs(m, inside));
    integrity::MemberLimits rigid = cantilever();
    rigid.nodes.reset();
    CHECK_FALSE(integrity::hogs(rigid, inside));
}

TEST_CASE("the limits table's own-weight record is optional and checked", "[INT-005][INT-023]") {
    const auto doc = [](const std::string& hogging) {
        return std::string(R"({"format": "mechsim-integrity-limits-1", "minimum_safety_factor": 1.0, "bodies": 9, )") +
               R"("checks": {"stress": true, "buckling": false, "deflection": false, "fastener": false, )" +
               R"("yield_offset": false}, "members": [{"name": "arm", "index": 0, "area": 1e-4, "i_y": 2e-8, )" +
               R"("i_z": 1e-8, "c_y": 0.01, "c_z": 0.02, "length": 1.0, "elastic_modulus": 2e11, )" +
               R"("effective_length_factor": 0.5, "allowable_stress": 1e8, "yield_strength": 2.5e8, )" +
               R"("deflection_limit": null, "supports": [{"at": 0.0, "condition": "clamped"}], "first_spring": 0, )" +
               R"("segments": 8, "segment_length": 0.125, "node_stiffness": 16000.0, "nodes": {"first_body": 1, )" +
               R"("count": 8, "root": "clamped", "parent_body": null, "position": [0, 0, 0], )" +
               R"("orientation": [1, 0, 0, 0], "carries": true, "carried": [])" + hogging + "}}], \"fastenings\": []}";
    };
    const integrity::Limits plain = integrity::parse_limits(doc(""));
    CHECK(plain.members[0].nodes->hogging_bound == 0.0);
    CHECK(plain.members[0].nodes->hogging.empty());
    const integrity::Limits set =
        integrity::parse_limits(doc(R"(, "hogging": {"bound": 0.25, "intervals": [[0.0, 0.2113], [0.7887, 1.0]]})"));
    CHECK(set.members[0].nodes->hogging_bound == 0.25);
    REQUIRE(set.members[0].nodes->hogging.size() == 2);
    CHECK(set.members[0].nodes->hogging[1][0] == 0.7887);
    CHECK_THROWS_AS(integrity::parse_limits(doc(R"(, "hogging": {"bound": -1, "intervals": []})")), integrity::LimitsError);
    CHECK_THROWS_AS(integrity::parse_limits(doc(R"(, "hogging": {"bound": 1, "intervals": [[0.5, 0.2]]})")),
                    integrity::LimitsError);
    CHECK_THROWS_AS(integrity::parse_limits(doc(R"(, "hogging": {"bound": 1, "intervals": [[0.5]]})")),
                    integrity::LimitsError);
    CHECK_THROWS_AS(integrity::parse_limits(doc(R"(, "hogging": {"intervals": []})")), integrity::LimitsError);
}
