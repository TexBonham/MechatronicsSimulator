#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numbers>
#include <span>
#include <vector>

#include "mechsim/signals/front_end.hpp"
#include "mechsim/signals/safety.hpp"
#include "mechsim/signals/sensors.hpp"
#include "signal_test_support.hpp"

using namespace mechsim::signals;  // NOLINT(google-build-using-namespace)
using signal_test::Stepper;

namespace {

constexpr double g = 9.81;
constexpr double h = 1e-3;  // layer period of signal_test::khz_clock

LoadCellParams catalog_cell() {
    LoadCellParams p;
    p.payload = 0;
    p.capacity = 49.05;
    p.sensitivity = 2e-3;
    p.excitation_voltage = 5.0;
    p.resolution = 0.01;
    p.overload_limit = 73.575;
    return p;
}

AmplifierParams catalog_amplifier() {
    AmplifierParams p;
    p.gain = 128.0;
    p.output_full_scale = 3.3;
    p.bandwidth = 10.0;
    p.adc = AdcParams{12, 3.3};
    return p;
}

PhysicalState payload_force(double force) {
    PhysicalState state;
    mechsim::kernel::PayloadState payload;
    payload.attached = true;
    payload.force = {0.0, 0.0, force};
    state.payloads.push_back(payload);
    return state;
}

PhysicalState body(const mechsim::kernel::Quat& q, const mechsim::kernel::Vec3& v, const mechsim::kernel::Vec3& w) {
    PhysicalState state;
    mechsim::kernel::BodyState b;
    b.orientation = q;
    b.linear_velocity = v;
    b.angular_velocity = w;
    state.bodies.push_back(b);
    return state;
}

ImuParams catalog_imu(double resolution, double gyro_resolution) {
    ImuParams p;
    p.accel_range = 39.24;
    p.gyro_range = 500.0 * std::numbers::pi / 180.0;
    p.resolution = resolution;
    p.gyro_resolution = gyro_resolution;
    return p;
}

// One joint of inertia J under a constant load torque L, stepped semi-implicitly as the adapters do (v' = v + h a,
// theta' = theta + h v') with the brake's drive solved implicitly: J (v' - v) / h = L + K (rest - theta') - D v'.
struct Plant {
    double inertia = 1e-3;
    double load = 0.0;
    double theta = 0.0;
    double velocity = 0.0;
    double torque = 0.0;  // the drive's torque in the last step

    void step(std::span<const mechsim::kernel::JointDrive> drives) {
        if (drives.empty()) {
            velocity += h * load / inertia;
            theta += h * velocity;
            torque = 0.0;
            return;
        }
        const mechsim::kernel::JointDrive& d = drives[0];
        velocity = ((inertia * velocity / h) + load + (d.stiffness * (d.rest - theta))) /
                   ((inertia / h) + (d.stiffness * h) + d.damping);
        theta += h * velocity;
        torque = (d.stiffness * (d.rest - theta)) - (d.damping * velocity);
    }

    [[nodiscard]] PhysicalState published() const {
        PhysicalState state;
        state.joint_positions.push_back(theta);
        state.joint_drive_torques.push_back(mechsim::kernel::JointDriveTorque{0, torque});
        return state;
    }
};

BrakeParams catalog_brake() {
    BrakeParams p;
    p.joint = 0;
    p.brake_torque = 0.5;
    p.engage_delay = 0.020;
    p.release_delay = 0.030;
    p.inverse_inertia = 1000.0;  // A0 = 1 / J
    return p;
}

}  // namespace

TEST_CASE("Load cell and amplifier: a 2 kg payload on the 5 kg cell reads the closed-form ADC count",
          "[SIG-014][LIB-021][LIB-019]") {
    // F = 2 x 9.81 = 19.62 N, a whole number of 0.01 N steps. V = sensitivity x excitation x F / capacity =
    // 2e-3 x 5 x 19.62 / 49.05 = 4.000e-3 V; x gain 128 = 0.5120 V (inside [0, 3.3]); the filter starts at its first
    // input; counts = round(0.512 / 3.3 x 4095) = round(635.35) = 635. Tolerance: the bridge voltage to 1e-15 V
    // (rounding of three products), the count exact.
    LoadCell cell("cell", catalog_cell());
    Stepper cell_run(cell);
    const double volts = cell_run.step(payload_force(19.62))[0];
    LoadCellAmplifier amp("amp", catalog_amplifier());
    Stepper amp_run(amp);
    const double counts = amp_run.step({}, {volts})[0];
    std::cout << "SIG-014 2 kg: closed form 4.000e-3 V, 0.5120 V, 635 counts; measured " << volts << " V, "
              << amp.voltage() << " V, " << counts << " counts\n";
    CHECK(std::fabs(volts - 4e-3) < 1e-15);
    CHECK(std::fabs(amp.voltage() - 0.512) < 1e-12);
    CHECK(counts == 635.0);
}

TEST_CASE("Load cell: over the overload limit the output saturates at 1.5 x full-scale output", "[SIG-014][LIB-021]") {
    // overload_limit 73.575 N = 1.5 capacity: V = 0.01 x 1.5 = 0.015 V for any larger force, no error.
    LoadCell cell("cell", catalog_cell());
    Stepper run(cell);
    const double volts = run.step(payload_force(500.0))[0];
    std::cout << "SIG-014 overload: closed form 0.015 V; measured " << volts << " V\n";
    CHECK(std::fabs(volts - 0.015) < 1e-15);
}

TEST_CASE("Amplifier: the 10 Hz low-pass step response reaches 63.2 percent at 1 / (2 pi 10) s", "[SIG-014][LIB-021]") {
    // A step from 0 to x = 1 V (input 1/128 V) at step 1. The update y += (1 - e^(-2 pi f h)) (x - y) is the exact
    // zero-order-hold discretization, so y_n = 1 - e^(-2 pi f n h) n steps after the step, to rounding (1e-12).
    // The time constant 1 / (2 pi 10) = 15.92 ms falls between samples 15 and 16; at the nearest sample, 16 ms, y
    // lies within one sample's increment y_16 - y_15 of 1 - 1/e = 0.632.
    LoadCellAmplifier amp("amp", catalog_amplifier());
    Stepper run(amp);
    run.step({}, {0.0});
    std::vector<double> y{0.0};
    for (int n = 1; n <= 40; ++n) {
        run.step({}, {1.0 / 128.0});
        y.push_back(amp.voltage());
        const double closed = 1.0 - std::exp(-2.0 * std::numbers::pi * 10.0 * n * h);
        CHECK(std::fabs(y.back() - closed) < 1e-12);
    }
    const double target = 1.0 - std::exp(-1.0);
    std::cout << "SIG-014 step response: closed form 0.6321 at 15.9 ms; measured " << y[16] << " at 16 ms (bound "
              << y[16] - y[15] << ")\n";
    CHECK(std::fabs(y[16] - target) <= y[16] - y[15]);
}

TEST_CASE("IMU: at rest it reads (0, 0, +g) and zero rate; in free fall zero specific force", "[LIB-019][LIB-021]") {
    // At rest a = 0: f = R^T (0 - g) = (0, 0, 9.81) with R = identity, exact at catalog resolution 1.2e-3 m/s^2 only
    // to the nearest step: round(9.81 / 1.2e-3) x 1.2e-3 = 8175 x 1.2e-3 = 9.810 (within half a step, 6e-4).
    Imu rest("imu", catalog_imu(1.2e-3, 0.015 * std::numbers::pi / 180.0));
    Stepper rest_run(rest);
    std::span<const double> out;
    for (int k = 0; k < 5; ++k) {
        out = rest_run.step(body({1.0, 0.0, 0.0, 0.0}, {}, {}));
    }
    std::cout << "LIB-019 IMU at rest: closed form (0, 0, 9.81), gyro 0; measured (" << out[0] << ", " << out[1]
              << ", " << out[2] << "), gyro (" << out[3] << ", " << out[4] << ", " << out[5] << ")\n";
    CHECK(out[0] == 0.0);
    CHECK(out[1] == 0.0);
    CHECK(std::fabs(out[2] - g) <= 0.5 * 1.2e-3);
    CHECK(out[3] == 0.0);
    CHECK(out[4] == 0.0);
    CHECK(out[5] == 0.0);
    // Free fall: v_n = -g n h along z, so (v_n - v_{n-1}) / h = -g exactly in reals, a - g = 0. In doubles the
    // difference quotient carries rounding of order eps |v| / h <= 2.2e-16 x 0.2 / 1e-3 = 4.4e-14; bound 1e-10.
    Imu fall("imu", catalog_imu(0.0, 0.0));
    Stepper fall_run(fall);
    double worst = 0.0;
    for (int n = 0; n < 20; ++n) {
        const std::span<const double> f = fall_run.step(body({1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, -g * n * h}, {}));
        if (n >= 1) {
            for (int i = 0; i < 3; ++i) {
                worst = std::fmax(worst, std::fabs(f[static_cast<std::size_t>(i)]));
            }
        }
    }
    std::cout << "LIB-019 IMU free fall: closed form 0; measured max |f| " << worst << "\n";
    CHECK(worst < 1e-10);
}

TEST_CASE("IMU: a body spinning at 2 rad/s about its z reads gyro_z = 2 within its resolution", "[LIB-019][LIB-021]") {
    // omega_world = (0, 0, 2) with the body turning about world z: R^T omega = (0, 0, 2) at any angle. Rounded to
    // the catalog gyro resolution 0.015 deg/s = 2.618e-4 rad/s: within half a step.
    const double resolution = 0.015 * std::numbers::pi / 180.0;
    Imu imu("imu", catalog_imu(1.2e-3, resolution));
    Stepper run(imu);
    for (int n = 0; n < 50; ++n) {
        const double angle = 2.0 * n * h;
        const std::span<const double> out =
            run.step(body({std::cos(angle / 2.0), 0.0, 0.0, std::sin(angle / 2.0)}, {}, {0.0, 0.0, 2.0}));
        CHECK(std::fabs(out[5] - 2.0) <= 0.5 * resolution);
        CHECK(std::fabs(out[3]) <= 0.5 * resolution);
        CHECK(std::fabs(out[4]) <= 0.5 * resolution);
    }
}

TEST_CASE("Brake: a 0.3 N m load under a 0.5 N m brake holds at its static sag", "[SIG-020][LIB-021][LIB-027]") {
    // K_b = 100 / (A0 h^2) = 1e5 N m/rad (J = 1e-3). Static closed form: K_b (0 - theta) + L = 0, so theta = L / K_b
    // = 3e-6 rad and the drive torque -L = -0.3 N m. The implicit step's free response decays by 1 / |1 + i omega h|
    // = 1 / sqrt(1 + 100) per step (omega h = 10), so after 200 steps the transient is far below 1e-9.
    Brake brake("brake", catalog_brake());
    Stepper run(brake);
    Plant plant;
    plant.load = 0.3;
    for (int k = 0; k < 200; ++k) {
        run.step(plant.published(), {0.0, 24.0});
        plant.step(brake.joint_drives());
    }
    std::cout << "SIG-020 hold: closed form torque -0.3 N m, sag 3e-6 rad; measured " << plant.torque << " N m, "
              << plant.theta << " rad\n";
    CHECK(std::fabs(plant.torque + 0.3) < 1e-9);
    CHECK(std::fabs(plant.theta - (0.3 / brake.stiffness())) < 1e-12);
    CHECK(brake.joint_drives()[0].rest == 0.0);
}

TEST_CASE("Brake: a 0.8 N m load slips a 0.5 N m brake with the drive torque pinned at the brake torque",
          "[SIG-020][LIB-021][LIB-027]") {
    // Slipping, the brake sets rest = theta_p - 0.5 / K_b, theta_p = theta + omega h + alpha h^2. Under the plant's
    // step this has the fixed point a = (L - 0.5) / J with drive torque exactly -0.5; a deviation in a decays by
    // K_b h^2 / (J + K_b h^2) = 100 / 101 per step, so after 2000 steps it is (100/101)^2000 = 2e-9 of its start
    // (below 1), and the torque error K_b h^2 times the acceleration change, below 1e-6. Floating rounding at
    // theta ~ 600 rad: K_b x eps x 600 = 1.3e-8 N m.
    Brake brake("brake", catalog_brake());
    Stepper run(brake);
    Plant plant;
    plant.load = 0.8;
    double theta_at_100 = 0.0;
    for (int k = 0; k < 2000; ++k) {
        run.step(plant.published(), {0.0, 24.0});
        plant.step(brake.joint_drives());
        if (k == 100) {
            theta_at_100 = plant.theta;
        }
    }
    std::cout << "SIG-020 slip: closed form torque -0.5 N m; measured " << plant.torque << " N m, theta "
              << plant.theta << " rad\n";
    CHECK(std::fabs(plant.torque + 0.5) < 1e-6);
    CHECK(plant.theta > theta_at_100);
    // Released, it publishes no drive and the load runs free.
    for (int k = 0; k < 31; ++k) {
        run.step(plant.published(), {1.0, 24.0});
        plant.step(brake.joint_drives());
    }
    CHECK(brake.joint_drives().empty());
}
