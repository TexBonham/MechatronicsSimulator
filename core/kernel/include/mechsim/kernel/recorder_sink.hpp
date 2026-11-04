#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "mechsim/kernel/tick_record.hpp"

namespace mechsim::kernel {

using TickCallback = std::function<void(const TickRecord&)>;

class SubscriptionId {
public:
    constexpr SubscriptionId() noexcept = default;
    constexpr explicit SubscriptionId(std::uint64_t value) noexcept : value_(value) {}

    [[nodiscard]] constexpr std::uint64_t value() const noexcept {
        return value_;
    }

    friend constexpr bool operator==(SubscriptionId, SubscriptionId) noexcept = default;

private:
    std::uint64_t value_ = 0;
};

// Where the kernel streams each tick (KER-022). Subscribers receive every row written after they subscribe, in
// subscription order, synchronously inside write_row (REC-019); they get a const view and cannot change the row
// or the run (REC-020). Back-pressure and the facade-side shape are the facade protocol's (REC open parameter).
class RecorderSink {
public:
    RecorderSink() = default;
    virtual ~RecorderSink() = default;
    RecorderSink(const RecorderSink&) = delete;
    RecorderSink& operator=(const RecorderSink&) = delete;
    RecorderSink(RecorderSink&&) = delete;
    RecorderSink& operator=(RecorderSink&&) = delete;

    virtual void write_row(const TickRecord& row) = 0;
    [[nodiscard]] virtual SubscriptionId subscribe(TickCallback callback) = 0;
    virtual void unsubscribe(SubscriptionId id) = 0;

    // Defaulted in milestone 1: errors of subscribers the sink dropped for throwing (REC-020); the kernel copies them
    // into RunReport::sink_errors.
    [[nodiscard]] virtual std::vector<std::string> subscriber_errors() const {
        return {};
    }
};

// Stores nothing; counts rows and forwards them to subscribers.
class NullRecorderSink final : public RecorderSink {
public:
    void write_row(const TickRecord& row) override;
    [[nodiscard]] SubscriptionId subscribe(TickCallback callback) override;
    void unsubscribe(SubscriptionId id) override;

    [[nodiscard]] std::size_t rows_written() const noexcept;
    [[nodiscard]] std::size_t subscriber_count() const noexcept;

private:
    std::size_t rows_ = 0;
    std::uint64_t next_id_ = 1;
    std::vector<std::pair<SubscriptionId, TickCallback>> subscribers_;
};

}  // namespace mechsim::kernel
