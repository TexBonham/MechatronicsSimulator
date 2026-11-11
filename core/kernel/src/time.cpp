#include "mechsim/kernel/time.hpp"

#if defined(_MSC_VER) && defined(_M_X64)
#include <intrin.h>
#endif

#include <cstdint>
#include <numeric>
#include <string>

namespace mechsim::kernel {

namespace {

std::int64_t checked_multiply(std::int64_t a, std::int64_t b) {
    std::int64_t result = 0;
#if defined(_MSC_VER) && defined(_M_X64)
    // MSVC has no __builtin_mul_overflow: the full 128-bit product fits int64 exactly when its high half is the sign
    // extension of its low half.
    std::int64_t high = 0;
    result = _mul128(a, b, &high);
    if (high != (result >> 63)) {
        throw KernelConfigError("rate arithmetic overflows int64");
    }
#else
    if (__builtin_mul_overflow(a, b, &result)) {
        throw KernelConfigError("rate arithmetic overflows int64");
    }
#endif
    return result;
}

}  // namespace

Rate::Rate() = default;

Rate::Rate(std::int64_t numerator, std::int64_t denominator) {
    if (numerator <= 0 || denominator <= 0) {
        throw KernelConfigError("rate must be positive, got " + std::to_string(numerator) + "/" +
                                std::to_string(denominator) + " Hz");
    }
    const std::int64_t g = std::gcd(numerator, denominator);
    num_ = numerator / g;
    den_ = denominator / g;
}

Seconds::Seconds(std::int64_t numerator, std::int64_t denominator) {
    if (numerator < 0 || denominator <= 0) {
        throw KernelConfigError("time must be non-negative with a positive denominator, got " +
                                std::to_string(numerator) + "/" + std::to_string(denominator) + " s");
    }
    const std::int64_t g = std::gcd(numerator, denominator);
    if (g > 0) {
        num_ = numerator / g;
        den_ = denominator / g;
    }
}

Rate lcm(const Rate& a, const Rate& b) {
    const std::int64_t g = std::gcd(a.numerator(), b.numerator());
    const std::int64_t num = checked_multiply(a.numerator() / g, b.numerator());
    return Rate(num, std::gcd(a.denominator(), b.denominator()));
}

Duration period_in_ticks(const Rate& rate, const Rate& base) {
    // base / rate = (bn * rd) / (bd * rn)
    const std::int64_t top = checked_multiply(base.numerator(), rate.denominator());
    const std::int64_t bottom = checked_multiply(base.denominator(), rate.numerator());
    if (top % bottom != 0) {
        throw KernelConfigError("rate " + std::to_string(rate.numerator()) + "/" + std::to_string(rate.denominator()) +
                                " Hz does not divide the base rate");
    }
    return Duration(top / bottom);
}

SimTime ticks_at(const Seconds& time, const Rate& base) {
    const std::int64_t top = checked_multiply(time.numerator(), base.numerator());
    const std::int64_t bottom = checked_multiply(time.denominator(), base.denominator());
    if (top % bottom != 0) {
        throw KernelConfigError("time " + std::to_string(time.numerator()) + "/" +
                                std::to_string(time.denominator()) + " s is not a whole number of base ticks");
    }
    return SimTime(top / bottom);
}

double to_seconds(Duration d, const Rate& base) noexcept {
    return static_cast<double>(d.count()) * static_cast<double>(base.denominator()) /
           static_cast<double>(base.numerator());
}

double to_seconds(SimTime t, const Rate& base) noexcept {
    return to_seconds(Duration(t.count()), base);
}

}  // namespace mechsim::kernel
