#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/pins/pins.hpp"
#include "mechsim/signals/front_end.hpp"
#include "mechsim/signals/safety.hpp"
#include "mechsim/signals/sensors.hpp"
#include "mechsim/signals/signal_layer.hpp"
#include "signal_test_support.hpp"

using mechsim::kernel::Duration;
using mechsim::kernel::Rate;
using mechsim::kernel::SimTime;
using mechsim::kernel::TickRecord;
using namespace mechsim::signals;  // NOLINT(google-build-using-namespace)
using signal_test::Stepper;

namespace {

// Catalog part load_cell_5kg_generic.
constexpr double capacity = 49.05;
constexpr double sensitivity = 2e-3;
constexpr double excitation = 5.0;
constexpr double full_scale_output = sensitivity * excitation;  // V at capacity
constexpr double overload = 73.575;

LoadCellParams cell_on_joint(double resolution = 0.0) {
    LoadCellParams p;
    p.joint = 0;
    p.capacity = capacity;
    p.sensitivity = sensitivity;
    p.excitation_voltage = excitation;
    p.resolution = resolution;
    p.overload_limit = overload;
    return p;
}

PhysicalState joint_force(double force) {
    PhysicalState state;
    state.bodies.emplace_back();
    mechsim::kernel::JointForce joint;
    joint.generalized_force = {force};
    state.joints.push_back(joint);
    state.joint_positions.push_back(0.0);
    return state;
}

PhysicalState payload_force(const mechsim::kernel::Vec3& force) {
    PhysicalState state;
    mechsim::kernel::PayloadState payload;
    payload.attached = true;
    payload.force = force;
    state.payloads.push_back(payload);
    return state;
}

AmplifierParams amplifier() {
    AmplifierParams p;
    p.gain = 128.0;
    p.output_full_scale = 3.3;
    p.bandwidth = 10.0;
    p.adc = AdcParams{12, 3.3};
    return p;
}

ImuParams imu_params() {
    ImuParams p;
    p.body = 0;
    p.accel_range = 39.24;
    p.gyro_range = 500.0 * std::numbers::pi / 180.0;
    return p;
}

PhysicalState body_state(const mechsim::kernel::Quat& orientation, const mechsim::kernel::Vec3& velocity,
                         const mechsim::kernel::Vec3& angular_velocity) {
    PhysicalState state;
    mechsim::kernel::BodyState body;
    body.orientation = orientation;
    body.linear_velocity = velocity;
    body.angular_velocity = angular_velocity;
    state.bodies.push_back(body);
    return state;
}

BrakeParams brake_params() {
    BrakeParams p;
    p.joint = 0;
    p.brake_torque = 0.5;
    p.engage_delay = 0.020;
    p.release_delay = 0.030;
    return p;
}

PhysicalState joint_at(double theta, std::optional<double> drive_torque = std::nullopt) {
    PhysicalState state;
    state.joint_positions.push_back(theta);
    if (drive_torque.has_value()) {
        state.joint_drive_torques.push_back(mechsim::kernel::JointDriveTorque{0, *drive_torque});
    }
    return state;
}

FailureInjection window(FailureMode mode, std::int64_t start, std::int64_t end) {
    FailureInjection fault;
    fault.mode = mode;
    fault.start = SimTime(start);
    fault.end = SimTime(end);
    return fault;
}

}  // namespace

// ===== Load cell ====================================================================================================

TEST_CASE("A load cell senses exactly one of a payload and a joint, with parameters in range", "[SIG-014][LIB-019]") {
    LoadCellParams both = cell_on_joint();
    both.payload = 0;
    CHECK_THROWS_AS(LoadCell("cell", both), SignalConfigError);
    LoadCellParams none = cell_on_joint();
    none.joint.reset();
    CHECK_THROWS_AS(LoadCell("cell", none), SignalConfigError);
    LoadCellParams zero_capacity = cell_on_joint();
    zero_capacity.capacity = 0.0;
    CHECK_THROWS_AS(LoadCell("cell", zero_capacity), SignalConfigError);
    LoadCellParams zero_sensitivity = cell_on_joint();
    zero_sensitivity.sensitivity = 0.0;
    CHECK_THROWS_AS(LoadCell("cell", zero_sensitivity), SignalConfigError);
    LoadCellParams zero_excitation = cell_on_joint();
    zero_excitation.excitation_voltage = 0.0;
    CHECK_THROWS_AS(LoadCell("cell", zero_excitation), SignalConfigError);
    LoadCellParams zero_overload = cell_on_joint();
    zero_overload.overload_limit = 0.0;
    CHECK_THROWS_AS(LoadCell("cell", zero_overload), SignalConfigError);
    const LoadCellParams negative_resolution = cell_on_joint(-0.01);
    CHECK_THROWS_AS(LoadCell("cell", negative_resolution), SignalConfigError);
    LoadCellParams nonlinear = cell_on_joint();
    nonlinear.nonlinearity = 1.5;
    CHECK_THROWS_AS(LoadCell("cell", nonlinear), SignalConfigError);
    nonlinear.nonlinearity = -0.1;
    CHECK_THROWS_AS(LoadCell("cell", nonlinear), SignalConfigError);
    LoadCellParams noisy = cell_on_joint();
    noisy.fidelity.noise = -1.0;
    CHECK_THROWS_AS(LoadCell("cell", noisy), SignalConfigError);
    LoadCellParams joint_and_reaction = cell_on_joint();
    joint_and_reaction.joint_reaction = 0;
    CHECK_THROWS_AS(LoadCell("cell", joint_and_reaction), SignalConfigError);
    LoadCellParams payload_and_reaction = cell_on_joint();
    payload_and_reaction.joint.reset();
    payload_and_reaction.payload = 0;
    payload_and_reaction.joint_reaction = 0;
    CHECK_THROWS_AS(LoadCell("cell", payload_and_reaction), SignalConfigError);
    LoadCellParams reaction_only = cell_on_joint();
    reaction_only.joint.reset();
    reaction_only.joint_reaction = 0;
    CHECK_NOTHROW(LoadCell("cell", reaction_only));
}

namespace {

PhysicalState joint_reaction(const mechsim::kernel::Vec3& force) {
    PhysicalState state;
    state.bodies.emplace_back();
    mechsim::kernel::JointForce joint;
    joint.reaction.force = force;
    joint.reaction.moment = {5.0, -7.0, 11.0};
    state.joints.push_back(joint);
    return state;
}

}  // namespace

TEST_CASE("A load cell on a fixed joint reads the reaction force magnitude", "[SIG-014][LIB-019]") {
    // Ruling 24: a payload attached at t = 0 is a fixed child of the hook, so the cell reads the fixed joint's reaction,
    // parent on child. A 2 kg child at rest under g = 9.81 m/s^2 is held by m g = 19.62 N straight up; the moment the
    // bridge also publishes is ignored. A tilted reaction reads its magnitude: |(3, -4, 12)| = 13 N exactly.
    LoadCellParams p = cell_on_joint();
    p.joint.reset();
    p.joint_reaction = 0;
    LoadCell cell("cell", p);
    Stepper run(cell);
    const std::span<const double> hanging = run.step(joint_reaction({0.0, 0.0, 2.0 * 9.81}));
    CHECK(std::fabs(hanging[0] - (full_scale_output * 19.62 / capacity)) < 1e-15);
    const std::span<const double> tilted = run.step(joint_reaction({3.0, -4.0, 12.0}));
    CHECK(std::fabs(tilted[0] - (full_scale_output * 13.0 / capacity)) < 1e-15);
    LoadCellParams q = p;
    q.joint_reaction = 1;
    LoadCell outside("cell", q);
    Stepper outside_run(outside);
    CHECK_THROWS_AS(outside_run.step(joint_reaction({0.0, 0.0, 1.0})), SignalRunError);
}

TEST_CASE("A load cell on a joint reads its signed generalized force", "[SIG-014][LIB-019]") {
    // V = sensitivity x excitation x F / capacity: F = -10 N gives -1e-2 x 10 / 49.05 V (a compression reading).
    LoadCell cell("cell", cell_on_joint());
    Stepper run(cell);
    const std::span<const double> out = run.step(joint_force(-10.0));
    CHECK(std::fabs(out[0] - (full_scale_output * -10.0 / capacity)) < 1e-15);
}

TEST_CASE("A load cell on a payload reads the weld force magnitude", "[SIG-014][LIB-019]") {
    // |(3, 4, 0)| = 5 N exactly.
    LoadCellParams p = cell_on_joint();
    p.joint.reset();
    p.payload = 0;
    LoadCell cell("cell", p);
    Stepper run(cell);
    const std::span<const double> out = run.step(payload_force({3.0, -4.0, 0.0}));
    CHECK(std::fabs(out[0] - (full_scale_output * 5.0 / capacity)) < 1e-15);
}

TEST_CASE("A load cell outside the published state ends the run", "[SIG-014]") {
    LoadCellParams p = cell_on_joint();
    p.joint.reset();
    p.payload = 1;
    LoadCell payload_cell("cell", p);
    Stepper payload_run(payload_cell);
    CHECK_THROWS_AS(payload_run.step(payload_force({0.0, 0.0, 1.0})), SignalRunError);
    LoadCellParams q = cell_on_joint();
    q.joint = 3;
    LoadCell joint_cell("cell", q);
    Stepper joint_run(joint_cell);
    CHECK_THROWS_AS(joint_run.step(joint_force(1.0)), SignalRunError);
}

TEST_CASE("A load cell quantizes force to its resolution, rounding to nearest", "[SIG-014][LIB-025]") {
    // 1.234 N at 0.01 N resolution rounds to 1.23 N; 1.236 N to 1.24 N.
    LoadCell cell("cell", cell_on_joint(0.01));
    Stepper run(cell);
    CHECK(std::fabs(run.step(joint_force(1.234))[0] - (full_scale_output * 1.23 / capacity)) < 1e-12);
    CHECK(std::fabs(run.step(joint_force(1.236))[0] - (full_scale_output * 1.24 / capacity)) < 1e-12);
}

TEST_CASE("A load cell saturates at its overload limit without an error and notes it once", "[SIG-014][LIB-019]") {
    // Over the limit the force is held at +-73.575 N = 1.5 capacity, so V = 1.5 x full-scale output.
    LoadCell cell("cell", cell_on_joint());
    Stepper run(cell);
    CHECK(run.step(joint_force(10.0))[0] < full_scale_output);
    CHECK(cell.take_notes().empty());
    CHECK(std::fabs(run.step(joint_force(100.0))[0] - (1.5 * full_scale_output)) < 1e-15);
    const std::vector<std::string> notes = cell.take_notes();
    REQUIRE(notes.size() == 1);
    CHECK(notes[0].find("overload") != std::string::npos);
    CHECK(std::fabs(run.step(joint_force(-200.0))[0] + (1.5 * full_scale_output)) < 1e-15);
    CHECK(cell.take_notes().empty());
}

TEST_CASE("The nonlinear tier adds nonlinearity x (F / capacity)^2 x full-scale output", "[SIG-014][LIB-019]") {
    // F = capacity / 2, nonlinearity 0.01: V = FSO (0.5 + 0.01 x 0.25).
    LoadCellParams p = cell_on_joint();
    p.nonlinearity = 0.01;
    LoadCell cell("cell", p);
    Stepper run(cell);
    CHECK(std::fabs(run.step(joint_force(capacity / 2.0))[0] - (full_scale_output * 0.5025)) < 1e-15);
}

TEST_CASE("A load cell is analog: forced_level is refused, stuck holds", "[LIB-025][SIG-036]") {
    LoadCell cell("cell", cell_on_joint());
    CHECK_THROWS_AS(cell.inject(window(FailureMode::forced_level, 0, 5)), SignalConfigError);
    CHECK_THROWS_AS(cell.inject(window(FailureMode::stall, 0, 5)), std::invalid_argument);
    cell.inject(window(FailureMode::stuck, 1, 3));
    Stepper run(cell);
    const double first = run.step(joint_force(10.0))[0];
    CHECK(run.step(joint_force(20.0))[0] == first);
    CHECK(run.step(joint_force(20.0))[0] == first);
    CHECK(run.step(joint_force(20.0))[0] == full_scale_output * 20.0 / capacity);
}

TEST_CASE("A load cell takes its delay and sample rate from the shared fidelity", "[SIG-003][SIG-004][LIB-025]") {
    // 80 Hz on a 1 kHz layer: a sample every floor(12.5) = 12 periods, held between. 3 ms delay: 3 periods late.
    LoadCellParams p = cell_on_joint();
    p.fidelity.sample_rate = 80.0;
    p.fidelity.delay = 0.003;
    LoadCell cell("cell", p);
    Stepper run(cell);
    std::vector<double> out;
    out.reserve(30);
    for (int k = 0; k < 30; ++k) {
        out.push_back(run.step(joint_force(static_cast<double>(k)))[0]);
    }
    CHECK(cell.sample_period() == 12);
    CHECK(cell.delay_periods() == 3);
    for (int k = 0; k < 30; ++k) {
        const int seen = k - 3;
        double expected_force = 0.0;
        if (seen >= 0) {
            const int held = (seen / 12) * 12;
            expected_force = static_cast<double>(held);
        }
        CHECK(out[static_cast<std::size_t>(k)] == full_scale_output * (expected_force / capacity));
    }
}

// ===== Amplifier ====================================================================================================

TEST_CASE("The amplifier rejects parameters out of range", "[SIG-014]") {
    AmplifierParams p = amplifier();
    p.gain = 0.0;
    CHECK_THROWS_AS(LoadCellAmplifier("amp", p), SignalConfigError);
    p = amplifier();
    p.output_full_scale = 0.0;
    CHECK_THROWS_AS(LoadCellAmplifier("amp", p), SignalConfigError);
    p = amplifier();
    p.bandwidth = 0.0;
    CHECK_THROWS_AS(LoadCellAmplifier("amp", p), SignalConfigError);
    p = amplifier();
    p.adc.reference = 0.0;
    CHECK_THROWS_AS(LoadCellAmplifier("amp", p), SignalConfigError);
    p = amplifier();
    p.adc.bits = 0;
    CHECK_THROWS_AS(LoadCellAmplifier("amp", p), SignalConfigError);
    p.adc.bits = 17;
    CHECK_THROWS_AS(LoadCellAmplifier("amp", p), SignalConfigError);
    p = amplifier();
    p.offset = std::nan("");
    CHECK_THROWS_AS(LoadCellAmplifier("amp", p), SignalConfigError);
    p = amplifier();
    p.adc.bits = 16;
    CHECK_NOTHROW(LoadCellAmplifier("amp", p));
}

TEST_CASE("The amplifier clamps to [0, output full scale] and the ADC to [0, 2^bits - 1]", "[SIG-014]") {
    // 0.1 V x 128 = 12.8 V clamps to 3.3 V = the reference: 4095 counts. -0.1 V clamps to 0 V: 0 counts.
    LoadCellAmplifier high("amp", amplifier());
    Stepper high_run(high);
    CHECK(high_run.step({}, {0.1})[0] == 4095.0);
    CHECK(high.voltage() == 3.3);
    LoadCellAmplifier low("amp", amplifier());
    Stepper low_run(low);
    CHECK(low_run.step({}, {-0.1})[0] == 0.0);
    // Offset 0.5 V at 0 V in: round(0.5 / 3.3 x 4095) = round(620.45) = 620.
    AmplifierParams p = amplifier();
    p.offset = 0.5;
    LoadCellAmplifier offset("amp", p);
    Stepper offset_run(offset);
    CHECK(offset_run.step({}, {0.0})[0] == 620.0);
    // A 2 V full scale on a 3.3 V reference never reaches the top count: round(2 / 3.3 x 4095) = 2482.
    p = amplifier();
    p.output_full_scale = 2.0;
    LoadCellAmplifier narrow("amp", p);
    Stepper narrow_run(narrow);
    CHECK(narrow_run.step({}, {1.0})[0] == 2482.0);
}

// ===== IMU ==========================================================================================================

TEST_CASE("The IMU rejects parameters out of range", "[LIB-019][LIB-025]") {
    ImuParams p = imu_params();
    p.accel_range = 0.0;
    CHECK_THROWS_AS(Imu("imu", p), SignalConfigError);
    p = imu_params();
    p.gyro_range = -1.0;
    CHECK_THROWS_AS(Imu("imu", p), SignalConfigError);
    p = imu_params();
    p.resolution = -1.0;
    CHECK_THROWS_AS(Imu("imu", p), SignalConfigError);
    p = imu_params();
    p.gyro_resolution = -1.0;
    CHECK_THROWS_AS(Imu("imu", p), SignalConfigError);
    p = imu_params();
    p.gyro_noise = -1.0;
    CHECK_THROWS_AS(Imu("imu", p), SignalConfigError);
    p = imu_params();
    p.gravity = {0.0, 0.0, std::nan("")};
    CHECK_THROWS_AS(Imu("imu", p), SignalConfigError);
    Imu imu("imu", imu_params());
    CHECK_THROWS_AS(imu.inject(window(FailureMode::forced_level, 0, 5)), SignalConfigError);
    Stepper run(imu);
    CHECK_THROWS_AS(run.step(PhysicalState{}), SignalRunError);
}

TEST_CASE("A tilted IMU reads gravity in its body frame; ranges clamp", "[LIB-019][LIB-025]") {
    // Body turned +90 deg about world x: R^T (0, 0, g) = (0, g, 0). A 5 m/s^2 range clamps it to 5.
    const double half = std::numbers::pi / 4.0;
    const mechsim::kernel::Quat tilted{std::cos(half), std::sin(half), 0.0, 0.0};
    Imu imu("imu", imu_params());
    Stepper run(imu);
    const std::span<const double> out = run.step(body_state(tilted, {}, {}));
    CHECK(std::fabs(out[0]) < 1e-12);
    CHECK(std::fabs(out[1] - 9.81) < 1e-12);
    CHECK(std::fabs(out[2]) < 1e-12);
    ImuParams narrow = imu_params();
    narrow.accel_range = 5.0;
    narrow.gyro_range = 1.0;
    Imu clamped("imu", narrow);
    Stepper clamped_run(clamped);
    const std::span<const double> c = clamped_run.step(body_state(tilted, {}, {0.0, 0.0, -3.0}));
    CHECK(c[1] == 5.0);
    // World omega (0, 0, -3) in the tilted body frame is R^T (0, 0, -3) = (0, -3, 0), clamped to -1 rad/s.
    CHECK(c[4] == -1.0);
}

TEST_CASE("IMU gyro noise is drawn separately from accelerometer noise", "[LIB-025][SIG-002]") {
    // gyro_noise > 0 with noise 0: the accelerometer reads exactly, the gyro does not.
    ImuParams p = imu_params();
    p.gyro_noise = 0.01;
    Imu imu("imu", p);
    Stepper run(imu);
    bool gyro_moved = false;
    for (int k = 0; k < 20; ++k) {
        const std::span<const double> out = run.step(body_state({1.0, 0.0, 0.0, 0.0}, {}, {}));
        CHECK(out[2] == 9.81);
        if (out[3] != 0.0 || out[4] != 0.0 || out[5] != 0.0) {
            gyro_moved = true;
        }
    }
    CHECK(gyro_moved);
}

// ===== Brake ========================================================================================================

TEST_CASE("The brake rejects parameters out of range", "[SIG-020][LIB-027]") {
    BrakeParams p = brake_params();
    p.brake_torque = 0.0;
    CHECK_THROWS_AS(Brake("brake", p), SignalConfigError);
    p = brake_params();
    p.engage_delay = -1.0;
    CHECK_THROWS_AS(Brake("brake", p), SignalConfigError);
    p = brake_params();
    p.release_delay = -1.0;
    CHECK_THROWS_AS(Brake("brake", p), SignalConfigError);
    p = brake_params();
    p.damping = -1.0;
    CHECK_THROWS_AS(Brake("brake", p), SignalConfigError);
    p = brake_params();
    p.inverse_inertia = 0.0;
    CHECK_THROWS_AS(Brake("brake", p), SignalConfigError);
    Brake brake("brake", brake_params());
    CHECK_THROWS_AS(brake.inject(window(FailureMode::stuck, 0, 5)), std::invalid_argument);
    Stepper run(brake);
    CHECK_THROWS_AS(run.step(PhysicalState{}, {0.0, 24.0}), SignalRunError);
}

TEST_CASE("Brake stiffness follows the envelope with A0, else brake torque over the default compliance", "[SIG-020]") {
    // With A0 = 1000 /(kg m^2) and h = 1 ms: K_b = 100 / (1000 x 1e-6) = 1e5, so K_b A0 h^2 = 100 < 400.
    // Without: K_b = 0.5 / 0.01 = 50 N m/rad.
    Brake fallback("brake", brake_params());
    const Stepper fallback_run(fallback);
    CHECK(fallback.stiffness() == 50.0);
    BrakeParams p = brake_params();
    p.inverse_inertia = 1000.0;
    Brake envelope("brake", p);
    const Stepper envelope_run(envelope);
    CHECK(std::fabs(envelope.stiffness() - 1e5) < 1e-6);
    CHECK(envelope.stiffness() * 1000.0 * 1e-6 < 400.0);
}

TEST_CASE("A power-off brake engages with release low or the rail off, after its delays", "[SIG-020][LIB-027]") {
    // Inputs (release, vcc). Starts released (release high, 24 V): no drive. Release low first seen at step 10:
    // engages at 10 + floor(20 ms / 1 ms) = 30. Release high at step 50: released at 50 + 30 = 80. Rail off at 100
    // with release high: engaged at 120.
    Brake brake("brake", brake_params());
    Stepper run(brake);
    for (std::int64_t k = 0; k < 140; ++k) {
        double release = 1.0;
        if (k >= 10 && k < 50) {
            release = 0.0;
        }
        double vcc = 24.0;
        if (k >= 100) {
            vcc = 0.0;
        }
        const std::span<const double> out = run.step(joint_at(0.25), {release, vcc});
        const bool engaged = (k >= 30 && k < 80) || k >= 120;
        CHECK(out[0] == static_cast<double>(engaged));
        CHECK(brake.joint_drives().size() == static_cast<std::size_t>(engaged));
        if (engaged) {
            const mechsim::kernel::JointDrive drive = brake.joint_drives()[0];
            CHECK(drive.joint == 0);
            CHECK(drive.enabled);
            CHECK(drive.rest == 0.25);
            CHECK(drive.stiffness == 50.0);
            CHECK(drive.damping == 0.0);
        }
    }
}

TEST_CASE("A brake demand that reverts within its delay is cancelled; the first sample takes the demand at once",
          "[SIG-020]") {
    Brake brake("brake", brake_params());
    Stepper run(brake);
    CHECK(run.step(joint_at(0.0), {0.0, 24.0})[0] == 1.0);
    for (std::int64_t k = 1; k < 20; ++k) {
        CHECK(run.step(joint_at(0.0), {1.0, 24.0})[0] == 1.0);
    }
    for (std::int64_t k = 20; k < 60; ++k) {
        CHECK(run.step(joint_at(0.0), {0.0, 24.0})[0] == 1.0);
    }
}

TEST_CASE("A power-on brake engages while powered with release low", "[SIG-020]") {
    BrakeParams p = brake_params();
    p.power_off_engaged = false;
    p.engage_delay = 0.0;
    p.release_delay = 0.0;
    Brake brake("brake", p);
    Stepper run(brake);
    CHECK(run.step(joint_at(0.0), {0.0, 24.0})[0] == 1.0);
    CHECK(run.step(joint_at(0.0), {1.0, 24.0})[0] == 0.0);
    CHECK(run.step(joint_at(0.0), {0.0, 0.0})[0] == 0.0);
    CHECK(run.step(joint_at(0.0), {0.0, 24.0})[0] == 1.0);
}

TEST_CASE("The brake reads back its drive torque and slips past its brake torque", "[SIG-020]") {
    // Engaged at theta 0, K_b = 50. A read-back of 0.3 N m holds: rest stays 0. A read-back of -0.6 N m slips with
    // s = -1 while the joint moves +: rest = theta_p - 0.5 / 50. theta_p = theta + omega h + alpha h^2 from the
    // readings 0, 0, 0.001: omega = 1 rad/s, alpha = 1000 rad/s^2, theta_p = 0.001 + 0.001 + 0.001 = 0.003.
    Brake brake("brake", brake_params());
    Stepper run(brake);
    run.step(joint_at(0.0), {0.0, 24.0});
    CHECK(run.step(joint_at(0.0, 0.3), {0.0, 24.0})[1] == 0.3);
    CHECK(brake.joint_drives()[0].rest == 0.0);
    run.step(joint_at(0.001, -0.6), {0.0, 24.0});
    CHECK(std::fabs(brake.joint_drives()[0].rest - (0.003 - 0.01)) < 1e-12);
    // The joint stops (velocity 0 then back): predicted velocity no longer against s, so it sticks at theta_p.
    run.step(joint_at(0.001, -0.5), {0.0, 24.0});
    CHECK(std::fabs(brake.joint_drives()[0].rest - 0.0) < 1e-12);
}

// ===== Power loss ===================================================================================================

TEST_CASE("A power-loss fault drops the supply to 0 V for its window and restores it after", "[SIG-019]") {
    PowerSupply supply("supply", 24.0);
    CHECK_THROWS_AS(supply.inject(window(FailureMode::stuck, 0, 5)), std::invalid_argument);
    supply.inject(window(FailureMode::power_loss, 10, 20));
    Stepper run(supply);
    for (std::int64_t k = 0; k < 30; ++k) {
        double expected = 24.0;
        if (k >= 10 && k < 20) {
            expected = 0.0;
        }
        CHECK(run.step(PhysicalState{})[0] == expected);
    }
}

TEST_CASE("Under power loss every rail the supply feeds is off and its consumers read 0 V", "[SIG-019][SIG-022]") {
    SignalLayerConfig config;
    config.rails = {RailSpec{"logic", "supply.voltage", {}, {"brake.vcc"}}};
    config.controller_rail = "logic";
    config.externals = {{"brake.release", 1.0}};
    SignalLayer layer(config);
    auto supply = std::make_unique<PowerSupply>("supply", 3.3);
    supply->inject(window(FailureMode::power_loss, 5, 8));
    layer.add_component(std::move(supply), Stage::power);
    BrakeParams p = brake_params();
    p.engage_delay = 0.0;
    p.release_delay = 0.0;
    layer.add_component(std::make_unique<Brake>("brake", p), Stage::actuator);
    layer.start(Rate(1000));
    for (std::int64_t k = 0; k < 12; ++k) {
        layer.set_physical_state(joint_at(0.0));
        layer.step(SimTime(k), Duration(1));
        const bool lost = k >= 5 && k < 8;
        CHECK(layer.rails()[0].on == !lost);
        CHECK(layer.controller_rail().value_or(PowerRail{}).voltage == static_cast<double>(!lost) * 3.3);
        CHECK(layer.signal_value("brake.engaged") == static_cast<double>(lost));
    }
}

// ===== Layer: front_end stage and approximation notes ==============================================================

TEST_CASE("The front_end stage runs after sensors in one step; a saturation note is recorded once",
          "[SIG-014][SIG-028]") {
    // 2 kg payload, F = 19.62 N: V = 2e-3 x 5 x 19.62 / 49.05 = 4e-3 V, x 128 = 0.512 V: round(0.512 / 3.3 x 4095)
    // = round(635.35) = 635 counts at the first step (the filter starts at its first input).
    SignalLayerConfig config;
    config.bindings.add(PinBinding{"amp.out", "PA0", PeripheralMode::adc_input});
    config.slots = {{"amp.out", PinField::adc_sample, 2}};
    config.connections = {{"cell.signal", "amp.signal_in"}};
    SignalLayer layer(config);
    layer.add_component(std::make_unique<LoadCellAmplifier>("amp", amplifier()), Stage::front_end);
    LoadCellParams p = cell_on_joint();
    p.joint.reset();
    p.payload = 0;
    layer.add_component(std::make_unique<LoadCell>("cell", p), Stage::sensor);
    layer.start(Rate(1000));
    layer.set_physical_state(payload_force({0.0, 0.0, 19.62}));
    layer.step(SimTime(0), Duration(1));
    CHECK(layer.pin_inputs().adc_sample[2] == 635);
    layer.set_physical_state(payload_force({0.0, 0.0, 100.0}));
    layer.step(SimTime(1), Duration(1));
    TickRecord saturated;
    layer.record(saturated);
    REQUIRE(saturated.events.size() == 1);
    CHECK(saturated.events[0].source == "cell");
    CHECK(saturated.events[0].kind == "approximation");
    layer.step(SimTime(2), Duration(1));
    TickRecord again;
    layer.record(again);
    CHECK(again.events.empty());
}
