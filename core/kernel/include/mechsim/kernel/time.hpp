#pragma once

#include <compare>
#include <cstdint>
#include <stdexcept>

namespace mechsim::kernel {

// Rejected configuration found before t = 0: bad rate, end time off the base tick, duplicate party name.
class KernelConfigError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

// A rate as exact rational hertz, numerator / denominator, both positive, stored reduced (3 kHz, 60 Hz, 0.1 Hz).
// Throws KernelConfigError on a non-positive part.
class Rate {
public:
    Rate();  // 1 Hz
    explicit Rate(std::int64_t numerator, std::int64_t denominator = 1);

    [[nodiscard]] std::int64_t numerator() const noexcept { return num_; }
    [[nodiscard]] std::int64_t denominator() const noexcept { return den_; }

    friend bool operator==(const Rate&, const Rate&) noexcept = default;

private:
    std::int64_t num_ = 1;
    std::int64_t den_ = 1;
};

// Exact non-negative rational seconds, used where a time is given before the base tick is known (end time).
class Seconds {
public:
    Seconds() = default;
    explicit Seconds(std::int64_t numerator, std::int64_t denominator = 1);

    [[nodiscard]] std::int64_t numerator() const noexcept { return num_; }
    [[nodiscard]] std::int64_t denominator() const noexcept { return den_; }

    friend bool operator==(const Seconds&, const Seconds&) noexcept = default;

private:
    std::int64_t num_ = 0;
    std::int64_t den_ = 1;
};

// Simulated time and intervals are integer counts of the run's base tick, 1 / base rate, where the base rate is the
// lcm of the physics rate and every party rate (computed at run start). Integer ticks keep rate arithmetic exact and
// runs bit-identical (invariant 4); converting to seconds is always an explicit call.
class Duration {
public:
    constexpr Duration() noexcept = default;
    constexpr explicit Duration(std::int64_t ticks) noexcept : ticks_(ticks) {}

    [[nodiscard]] constexpr std::int64_t count() const noexcept { return ticks_; }

    friend constexpr auto operator<=>(Duration, Duration) noexcept = default;
    friend constexpr Duration operator+(Duration a, Duration b) noexcept { return Duration(a.ticks_ + b.ticks_); }
    friend constexpr Duration operator-(Duration a, Duration b) noexcept { return Duration(a.ticks_ - b.ticks_); }
    friend constexpr Duration operator*(Duration a, std::int64_t k) noexcept { return Duration(a.ticks_ * k); }

private:
    std::int64_t ticks_ = 0;
};

class SimTime {
public:
    constexpr SimTime() noexcept = default;
    constexpr explicit SimTime(std::int64_t ticks) noexcept : ticks_(ticks) {}

    [[nodiscard]] constexpr std::int64_t count() const noexcept { return ticks_; }

    friend constexpr auto operator<=>(SimTime, SimTime) noexcept = default;
    friend constexpr SimTime operator+(SimTime t, Duration d) noexcept { return SimTime(t.ticks_ + d.count()); }
    friend constexpr Duration operator-(SimTime a, SimTime b) noexcept { return Duration(a.ticks_ - b.ticks_); }

private:
    std::int64_t ticks_ = 0;
};

// lcm of rates as rationals: lcm(numerators) / gcd(denominators). Throws KernelConfigError on int64 overflow.
[[nodiscard]] Rate lcm(const Rate& a, const Rate& b);

// Ticks of base per period of rate; throws KernelConfigError unless rate divides base exactly.
[[nodiscard]] Duration period_in_ticks(const Rate& rate, const Rate& base);

// Ticks of base in an exact time; throws KernelConfigError unless the time is a whole number of ticks.
[[nodiscard]] SimTime ticks_at(const Seconds& time, const Rate& base);

[[nodiscard]] double to_seconds(Duration d, const Rate& base) noexcept;
[[nodiscard]] double to_seconds(SimTime t, const Rate& base) noexcept;

}  // namespace mechsim::kernel
