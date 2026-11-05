#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "mechsim/kernel/recorder_sink.hpp"
#include "mechsim/kernel/tick_record.hpp"

namespace mechsim::kernel {

// Stores every row in write order and streams each one, after storing it, to the subscribers present when it was
// written, synchronously and in subscription order (REC-019). A subscriber cannot change the stored row or the run
// (REC-020): it gets a const view, and an exception it throws removes that subscriber and is kept in
// subscriber_errors() instead of reaching the kernel; unsubscribing a dropped subscriber once is not an error.
// Subscribing or unsubscribing inside a callback is allowed and takes effect from the next row. drain() hands the rows
// written since the last drain to the Python side.
class InMemoryRecorderSink final : public RecorderSink {
public:
    void write_row(const TickRecord& row) override;
    [[nodiscard]] SubscriptionId subscribe(TickCallback callback) override;
    void unsubscribe(SubscriptionId id) override;

    [[nodiscard]] std::span<const TickRecord> rows() const noexcept;
    [[nodiscard]] std::vector<TickRecord> drain();
    [[nodiscard]] std::size_t drained() const noexcept;
    [[nodiscard]] std::size_t subscriber_count() const noexcept;
    [[nodiscard]] std::vector<std::string> subscriber_errors() const override;

private:
    struct Subscriber {
        SubscriptionId id;
        TickCallback callback;
        bool active = true;
    };

    void compact();

    std::vector<TickRecord> rows_;
    std::size_t drained_ = 0;
    std::uint64_t next_id_ = 1;
    std::vector<Subscriber> subscribers_;
    std::vector<Subscriber> pending_;
    std::vector<std::string> subscriber_errors_;
    std::vector<SubscriptionId> dropped_;
    bool dispatching_ = false;
};

}  // namespace mechsim::kernel
