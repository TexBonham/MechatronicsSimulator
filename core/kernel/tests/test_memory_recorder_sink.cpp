#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "mechsim/kernel/memory_recorder_sink.hpp"
#include "mechsim/kernel/recorder_sink.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"

using mechsim::kernel::InMemoryRecorderSink;
using mechsim::kernel::SimTime;
using mechsim::kernel::SubscriptionId;
using mechsim::kernel::TickRecord;

namespace {

TickRecord row_at(std::int64_t t) {
    TickRecord row;
    row.time = SimTime(t);
    row.controller_state = "run";
    return row;
}

}  // namespace

TEST_CASE("The in-memory sink stores every row and drains each once", "[REC-019][KER-022]") {
    InMemoryRecorderSink sink;
    sink.write_row(row_at(1));
    sink.write_row(row_at(2));
    CHECK(sink.rows().size() == 2);
    const std::vector<TickRecord> first = sink.drain();
    REQUIRE(first.size() == 2);
    CHECK(first[0].time == SimTime(1));
    CHECK(first[1].time == SimTime(2));
    CHECK(sink.drained() == 2);
    CHECK(sink.drain().empty());
    sink.write_row(row_at(3));
    const std::vector<TickRecord> second = sink.drain();
    REQUIRE(second.size() == 1);
    CHECK(second[0].time == SimTime(3));
    CHECK(sink.rows().size() == 3);
}

TEST_CASE("Subscribers get rows written after subscribing, in subscription order", "[REC-019]") {
    InMemoryRecorderSink sink;
    std::vector<std::string> seen;
    sink.write_row(row_at(1));
    const SubscriptionId a =
        sink.subscribe([&seen](const TickRecord& r) { seen.push_back("a" + std::to_string(r.time.count())); });
    static_cast<void>(
        sink.subscribe([&seen](const TickRecord& r) { seen.push_back("b" + std::to_string(r.time.count())); }));
    CHECK(sink.subscriber_count() == 2);
    sink.write_row(row_at(2));
    sink.unsubscribe(a);
    CHECK(sink.subscriber_count() == 1);
    sink.write_row(row_at(3));
    CHECK(seen == std::vector<std::string>{"a2", "b2", "b3"});
    CHECK_THROWS_AS(sink.unsubscribe(a), std::invalid_argument);
    CHECK_THROWS_AS(sink.subscribe({}), std::invalid_argument);
}

TEST_CASE("Subscribing and unsubscribing inside a callback take effect from the next row", "[REC-019]") {
    InMemoryRecorderSink sink;
    std::vector<std::string> seen;
    SubscriptionId second;
    SubscriptionId late;
    bool joined = false;
    static_cast<void>(sink.subscribe([&](const TickRecord& r) {
        seen.push_back("first" + std::to_string(r.time.count()));
        if (r.time == SimTime(1)) {
            sink.unsubscribe(second);
        }
        if (!joined) {
            joined = true;
            late = sink.subscribe(
                [&seen](const TickRecord& row) { seen.push_back("late" + std::to_string(row.time.count())); });
        }
    }));
    second =
        sink.subscribe([&seen](const TickRecord& r) { seen.push_back("second" + std::to_string(r.time.count())); });
    sink.write_row(row_at(1));
    sink.write_row(row_at(2));
    CHECK(seen == std::vector<std::string>{"first1", "first2", "late2"});
    sink.unsubscribe(late);
    CHECK(sink.subscriber_count() == 1);
}

TEST_CASE("A streaming consumer cannot change the recording", "[REC-020][REC-019]") {
    // The row is stored before dispatch and handed out const; a throwing consumer is removed, its error kept, the
    // other consumers still receive the row, and nothing reaches the writer.
    InMemoryRecorderSink sink;
    std::vector<std::int64_t> seen;
    static_cast<void>(sink.subscribe([](const TickRecord&) { throw std::runtime_error("consumer broke"); }));
    static_cast<void>(sink.subscribe([&sink](const TickRecord& r) { sink.write_row(r); }));
    static_cast<void>(sink.subscribe([&seen](const TickRecord& r) { seen.push_back(r.time.count()); }));
    CHECK_NOTHROW(sink.write_row(row_at(1)));
    CHECK_NOTHROW(sink.write_row(row_at(2)));
    CHECK(seen == std::vector<std::int64_t>{1, 2});
    CHECK(sink.subscriber_count() == 1);
    REQUIRE(sink.subscriber_errors().size() == 2);
    CHECK(sink.subscriber_errors()[0] == "consumer broke");
    REQUIRE(sink.rows().size() == 2);
    CHECK(sink.rows()[0].time == SimTime(1));
    CHECK(sink.rows()[0].controller_state == "run");
}

TEST_CASE("A dropped subscriber can be unsubscribed once without error", "[REC-020]") {
    InMemoryRecorderSink sink;
    const SubscriptionId id = sink.subscribe([](const TickRecord&) { throw std::runtime_error("gone"); });
    sink.write_row(row_at(1));
    CHECK(sink.subscriber_count() == 0);
    CHECK_NOTHROW(sink.unsubscribe(id));
    CHECK_THROWS_AS(sink.unsubscribe(id), std::invalid_argument);
}
