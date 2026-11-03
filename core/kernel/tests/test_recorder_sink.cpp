#include <catch2/catch_test_macros.hpp>
#include <stdexcept>
#include <vector>

#include "mechsim/kernel/recorder_sink.hpp"
#include "mechsim/kernel/time.hpp"

using mechsim::kernel::NullRecorderSink;
using mechsim::kernel::SimTime;
using mechsim::kernel::TickRecord;

namespace {

TickRecord row_at(SimTime t) {
    TickRecord row;
    row.time = t;
    return row;
}

}  // namespace

TEST_CASE("The null sink counts rows", "[REC-019]") {
    NullRecorderSink sink;
    CHECK(sink.rows_written() == 0);
    sink.write_row(row_at(SimTime(1)));
    sink.write_row(row_at(SimTime(2)));
    CHECK(sink.rows_written() == 2);
}

TEST_CASE("Subscribers receive every row written after they subscribe, in subscription order", "[REC-019]") {
    NullRecorderSink sink;
    sink.write_row(row_at(SimTime(1)));
    std::vector<int> order;
    std::vector<SimTime> first_seen;
    const auto first = sink.subscribe([&](const TickRecord& row) {
        order.push_back(1);
        first_seen.push_back(row.time);
    });
    const auto second = sink.subscribe([&](const TickRecord&) { order.push_back(2); });
    CHECK_FALSE(first == second);
    CHECK(sink.subscriber_count() == 2);
    sink.write_row(row_at(SimTime(2)));
    sink.write_row(row_at(SimTime(3)));
    const std::vector<int> expected_order{1, 2, 1, 2};
    CHECK(order == expected_order);
    const std::vector<SimTime> expected_times{SimTime(2), SimTime(3)};
    CHECK(first_seen == expected_times);
    CHECK(sink.rows_written() == 3);
}

TEST_CASE("Unsubscribing stops delivery; unknown ids and empty callbacks are rejected", "[REC-019]") {
    NullRecorderSink sink;
    int calls = 0;
    const auto id = sink.subscribe([&calls](const TickRecord&) { ++calls; });
    sink.write_row(row_at(SimTime(0)));
    sink.unsubscribe(id);
    sink.write_row(row_at(SimTime(0)));
    CHECK(calls == 1);
    CHECK(sink.subscriber_count() == 0);
    CHECK_THROWS_AS(sink.unsubscribe(id), std::invalid_argument);
    CHECK_THROWS_AS(sink.subscribe({}), std::invalid_argument);
}

TEST_CASE("A subscriber does not change the row count it observes", "[REC-020]") {
    NullRecorderSink observed;
    NullRecorderSink unobserved;
    int calls = 0;
    static_cast<void>(observed.subscribe([&calls](const TickRecord&) { ++calls; }));
    for (int i = 0; i < 5; ++i) {
        observed.write_row(row_at(SimTime(i)));
        unobserved.write_row(row_at(SimTime(i)));
    }
    CHECK(calls == 5);
    CHECK(observed.rows_written() == unobserved.rows_written());
}
