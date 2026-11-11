#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "mechsim/signals/component_base.hpp"

namespace mechsim::signals {

// What a sensor's output is, which decides what a forced_level fault means (SIG-036).
enum class SensorOutputKind : std::uint8_t {
    digital,  // levels 0 or 1: forced_level presents the fault's level, which must be 0 or 1
    analog,   // a value in the output unit: forced_level is a SignalConfigError naming the fault
    counter,  // edges per tick: forced_level presents round(level) counts per tick
};

// The catalog's noise, delay and sample_rate, common to every sensor (SIG-002, SIG-003, SIG-004).
struct SensorFidelity {
    double noise = 0.0;        // standard deviation of N(0, noise) in the measured quantity's unit (m or rad)
    double delay = 0.0;        // s
    double sample_rate = 0.0;  // Hz; 0 samples every layer step
};

inline constexpr std::int64_t max_delay_periods = 1000000;

// Shared sensor pipeline. Every layer step: measure() reads the physical quantities. On a sample step (step index a
// multiple of the sample period, periods_floor(1 / sample_rate), at least 1): each quantity gets N(0, noise) when noise
// > 0 and N(0, magnitude) more under a noisy fault (one draw each from the child generator, none when 0, SIG-005),
// then convert() makes the sample's frame. Between samples the frame holds. A ring of periods_ceil(delay) + 1 frames,
// filled with the first frame, delivers at step n the frame held at step n - periods_ceil(delay) (SIG-003).
// Faults, for start <= n < end: stuck holds the delivered output; dropped takes no sample (counters keep sampling so
// the window's counts are lost) and holds the output; forced_level per SensorOutputKind. Two injections whose
// windows overlap are a SignalConfigError at start().
class SampledSensor : public BuiltinComponent {
public:
    void sample(const PhysicalState& state) final;
    void latch_outputs() final;
    // Throws SignalConfigError for forced_level on an analog sensor, std::invalid_argument for a digital forced level
    // other than 0 or 1 and for what BuiltinComponent::inject refuses.
    void inject(const FailureInjection& injection) override;

    [[nodiscard]] std::int64_t sample_period() const noexcept;  // layer periods, after start()
    [[nodiscard]] std::int64_t delay_periods() const noexcept;   // layer periods, after start()

protected:
    // Throws SignalConfigError on a negative or non-finite fidelity parameter.
    SampledSensor(std::string instance, const std::vector<std::string>& outputs, SensorOutputKind kind,
                  std::size_t quantities, std::size_t frame, SensorFidelity fidelity);

    virtual void measure(const PhysicalState& state, std::span<double> quantities) = 0;
    virtual void convert(std::span<const double> quantities, std::int64_t step, std::span<double> frame) = 0;
    // Default for digital and analog sensors, one frame value per output: stuck and dropped hold, forced_level sets
    // every output to the level, else the outputs are the delivered frame.
    virtual void present(std::span<const double> delivered, const std::optional<FailureInjection>& fault);
    virtual void on_sensor_start() {}
    // Part noise standard deviation of one quantity; default the fidelity's noise for every quantity.
    [[nodiscard]] virtual double part_noise(std::size_t quantity) const;
    [[nodiscard]] const SensorFidelity& fidelity() const noexcept;
    [[nodiscard]] static bool holds(const std::optional<FailureInjection>& fault) noexcept;

private:
    void on_start() final;

    SensorOutputKind kind_;
    SensorFidelity fidelity_;
    std::vector<double> quantities_;
    std::vector<double> frame_;
    std::vector<double> ring_;
    std::size_t frame_size_;
    std::int64_t sample_period_ = 1;
    std::int64_t delay_periods_ = 0;
    std::int64_t pushed_ = 0;
};

// Catalog type limit_switch. position, hysteresis, resolution and noise are in the coordinate's unit (m or rad).
struct LimitSwitchParams {
    JointCoordinate coordinate;
    double position = 0.0;       // switch position
    bool actuate_above = true;   // actuated at coordinate >= position; false: at coordinate <= position
    double hysteresis = 0.0;     // released once the coordinate is this far back past the position
    double resolution = 0.0;     // quantization step, floor; 0 = none (SIG-001)
    double debounce_time = 0.0;  // s (SIG-010)
    bool normally_open = true;   // level 1 = contact closed: actuated when normally open, released when closed
    SensorFidelity fidelity;
};

// Output `out` (level), a digital SampledSensor; noise applies to the position compared against the threshold, so a
// noisy switch chatters near its position. Debounce (SIG-010): with N = periods_ceil(debounce_time), a new contact
// state appears at the first sample at least N periods after it was first sampled, provided every sample in between
// saw it too (that sample itself need not); a state held for fewer periods never appears. N = 0 passes every sample.
class LimitSwitch final : public SampledSensor {
public:
    // Throws SignalConfigError on a negative or non-finite parameter.
    LimitSwitch(std::string instance, LimitSwitchParams params);

private:
    void measure(const PhysicalState& state, std::span<double> quantities) override;
    void convert(std::span<const double> quantities, std::int64_t step, std::span<double> frame) override;
    void on_sensor_start() override;

    LimitSwitchParams params_;
    CoordinateReader reader_;
    std::int64_t debounce_periods_ = 0;
    bool primed_ = false;
    bool actuated_ = false;
    bool debounced_ = false;
    bool pending_ = false;
    std::int64_t pending_since_ = 0;
};

// Catalog type incremental_encoder. resolution is the angle per quadrature count, rad (0.15 deg = 2400 counts per
// revolution in quadrature); noise is in rad.
struct EncoderParams {
    JointCoordinate coordinate;  // angular
    double resolution = 0.0;
    bool reversed = false;  // count direction
    SensorFidelity fidelity;
};

// Outputs `channel_a`, `channel_b`: signed quadrature edges on each phase between the last two delivered samples
// (SIG-011), a counter SampledSensor whose frame is the count position n = floor(angle / resolution). An A edge is a
// step between n = 2k and 2k + 1, a B edge one between 2k + 1 and 2k + 2, so a_edges + b_edges = n_now - n_last
// exactly. Noise (part and noisy fault) is added to the angle in rad before the floor. Stuck and dropped deliver no
// edges (counts are lost, as on hardware); forced_level presents round(forced_level) counts per tick on channel_a.
class IncrementalEncoder final : public SampledSensor {
public:
    // Throws SignalConfigError on a non-positive resolution or a linear coordinate.
    IncrementalEncoder(std::string instance, EncoderParams params);

private:
    void measure(const PhysicalState& state, std::span<double> quantities) override;
    void convert(std::span<const double> quantities, std::int64_t step, std::span<double> frame) override;
    void present(std::span<const double> delivered, const std::optional<FailureInjection>& fault) override;
    void on_sensor_start() override;

    EncoderParams params_;
    CoordinateReader reader_;
    bool primed_ = false;
    std::int64_t last_count_ = 0;
};

// Catalog type load_cell. Senses exactly one of a payload (the magnitude of physics_outputs.payloads[payload].force,
// the weld force on it) or a joint (generalized_force[0] of physics_outputs.joints[joint], signed along its axis).
// Forces in N; fidelity.noise in N.
struct LoadCellParams {
    std::optional<std::size_t> payload;
    std::optional<std::size_t> joint;
    double capacity = 0.0;            // N, > 0
    double sensitivity = 0.0;         // V/V at capacity, > 0
    double excitation_voltage = 0.0;  // V, > 0
    double resolution = 0.0;          // N, quantization step (round to nearest); 0 = none
    double overload_limit = 0.0;      // N, > 0
    double nonlinearity = 0.0;        // [0, 1], the nonlinear tier's coefficient; 0 = ideal
    SensorFidelity fidelity;
};

// Output `signal`, V, an analog SampledSensor (SIG-014 source, LIB-019). F (plus part and fault noise) rounds to
// resolution, or past +-overload_limit is held at the limit itself; V = sensitivity excitation F / capacity + nonlinearity (F / capacity)^2
// sensitivity excitation. Saturation raises no error (overload is the firmware's business): the first saturated
// sample of a run is noted once as an approximation line (take_notes).
class LoadCell final : public SampledSensor {
public:
    // Throws SignalConfigError unless exactly one of payload and joint is set and the parameters are in range.
    LoadCell(std::string instance, LoadCellParams params);

private:
    void measure(const PhysicalState& state, std::span<double> quantities) override;
    void convert(std::span<const double> quantities, std::int64_t step, std::span<double> frame) override;
    void on_sensor_start() override;

    LoadCellParams params_;
    bool noted_ = false;
};

// Catalog type imu (ruling 2 placeholder: one analog output per axis carrying the SI value itself, no volts).
struct ImuParams {
    std::size_t body = 0;
    kernel::Vec3 gravity{0.0, 0.0, -9.81};  // m/s^2, world frame (the model's gravity)
    double accel_range = 0.0;               // m/s^2, > 0, clamps each axis to +-range
    double gyro_range = 0.0;                // rad/s, > 0
    double resolution = 0.0;                // m/s^2, round to nearest; 0 = none
    double gyro_resolution = 0.0;           // rad/s
    double gyro_noise = 0.0;                // rad/s; fidelity.noise is the accelerometer's, m/s^2
    SensorFidelity fidelity;
};

// Outputs accel_x, accel_y, accel_z (m/s^2), gyro_x, gyro_y, gyro_z (rad/s), an analog SampledSensor (LIB-019).
// Specific force f = R^T (a - g), a = (v_n - v_{n-1}) / layer period from the body's world linear velocity (a = 0 at
// the first step), R the body's orientation; gyro = R^T omega_world. Noise (fidelity.noise on the accelerometer,
// gyro_noise on the gyro; a noisy fault's magnitude on all six in their own units), then clamp to range, then round
// to resolution.
class Imu final : public SampledSensor {
public:
    // Throws SignalConfigError on a non-positive range, a negative resolution or noise, or a non-finite gravity.
    Imu(std::string instance, ImuParams params);

private:
    void measure(const PhysicalState& state, std::span<double> quantities) override;
    void convert(std::span<const double> quantities, std::int64_t step, std::span<double> frame) override;
    [[nodiscard]] double part_noise(std::size_t quantity) const override;
    void on_sensor_start() override;

    ImuParams params_;
    bool primed_ = false;
    kernel::Vec3 last_velocity_{};
};

}  // namespace mechsim::signals
