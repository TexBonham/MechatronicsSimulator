#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <type_traits>

#include "mechsim/kernel/time.hpp"

using mechsim::kernel::Duration;
using mechsim::kernel::KernelConfigError;
using mechsim::kernel::lcm;
using mechsim::kernel::period_in_ticks;
using mechsim::kernel::Rate;
using mechsim::kernel::Seconds;
using mechsim::kernel::SimTime;
using mechsim::kernel::ticks_at;
using mechsim::kernel::to_seconds;

static_assert(std::is_same_v<decltype(SimTime{}.count()), std::int64_t>);
static_assert(std::is_same_v<decltype(Duration{}.count()), std::int64_t>);
static_assert(!std::is_convertible_v<SimTime, double>);
static_assert(!std::is_convertible_v<Duration, double>);

TEST_CASE("Simulated time is an integer tick count with exact arithmetic", "[KER-001][KER-024]") {
    SimTime t;
    CHECK(t.count() == 0);
    for (int i = 0; i < 1'000; ++i) {
        t = t + Duration(3);
    }
    CHECK(t == SimTime(3'000));
    CHECK(t - SimTime(1'000) == Duration(2'000));
    CHECK(Duration(5) - Duration(2) == Duration(3));
    CHECK(Duration(1) * 5 == Duration(5));
    CHECK(Duration(1) < Duration(2));
}

TEST_CASE("Rates are positive reduced rationals", "[KER-007]") {
    CHECK(Rate(3'000).numerator() == 3'000);
    CHECK(Rate(3'000).denominator() == 1);
    const Rate tenth(2, 20);
    CHECK(tenth.numerator() == 1);
    CHECK(tenth.denominator() == 10);
    CHECK(Rate(2, 4) == Rate(1, 2));
    CHECK_THROWS_AS(Rate(0), KernelConfigError);
    CHECK_THROWS_AS(Rate(-1), KernelConfigError);
    CHECK_THROWS_AS(Rate(1, 0), KernelConfigError);
    CHECK_THROWS_AS(Rate(1, -2), KernelConfigError);
    CHECK_THROWS_AS(Seconds(-1), KernelConfigError);
    CHECK_THROWS_AS(Seconds(1, 0), KernelConfigError);
    CHECK(Seconds(0, 7) == Seconds(0));
}

// lcm of a/b and c/d is lcm(a, c) / gcd(b, d): 3000 and 1000 -> 3000; 60 and 1000 -> 3000; 0.1 and 1000 -> 1000;
// 1/2 and 1/3 -> 1.
TEST_CASE("The base rate is the rational lcm of the rates", "[KER-003]") {
    CHECK(lcm(Rate(3'000), Rate(1'000)) == Rate(3'000));
    CHECK(lcm(Rate(60), Rate(1'000)) == Rate(3'000));
    CHECK(lcm(Rate(1, 10), Rate(1'000)) == Rate(1'000));
    CHECK(lcm(Rate(1, 2), Rate(1, 3)) == Rate(1));
    CHECK(lcm(Rate(3, 2), Rate(1)) == Rate(3));
    CHECK_THROWS_AS(lcm(Rate(INT64_MAX), Rate(INT64_MAX - 1)), KernelConfigError);
}

TEST_CASE("Periods and times convert to whole base ticks or are rejected", "[KER-007]") {
    const Rate base(3'000);
    CHECK(period_in_ticks(Rate(1'000), base) == Duration(3));
    CHECK(period_in_ticks(Rate(60), base) == Duration(50));
    CHECK(period_in_ticks(Rate(1, 10), Rate(1'000)) == Duration(10'000));
    CHECK_THROWS_AS(period_in_ticks(Rate(7), base), KernelConfigError);
    CHECK(ticks_at(Seconds(1), base) == SimTime(3'000));
    CHECK(ticks_at(Seconds(1, 3), base) == SimTime(1'000));
    CHECK(ticks_at(Seconds(0), base) == SimTime(0));
    CHECK_THROWS_AS(ticks_at(Seconds(1, 7), base), KernelConfigError);
}

TEST_CASE("Conversion to seconds is an explicit function of the base rate", "[KER-024]") {
    CHECK(to_seconds(Duration(3'000), Rate(3'000)) == 1.0);
    CHECK(to_seconds(SimTime(1'500), Rate(1'000)) == 1.5);
    CHECK(to_seconds(SimTime(1), Rate(1, 10)) == 10.0);
}
