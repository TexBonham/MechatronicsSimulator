#include <cstddef>
#include <stdexcept>
#include <utility>

#include "mechsim/kernel/recorder_sink.hpp"

namespace mechsim::kernel {

void NullRecorderSink::write_row(const TickRecord& row) {
    ++rows_;
    for (const auto& subscriber : subscribers_) {
        subscriber.second(row);
    }
}

SubscriptionId NullRecorderSink::subscribe(TickCallback callback) {
    if (!callback) {
        throw std::invalid_argument("empty subscriber callback");
    }
    const SubscriptionId id(next_id_);
    ++next_id_;
    subscribers_.emplace_back(id, std::move(callback));
    return id;
}

void NullRecorderSink::unsubscribe(SubscriptionId id) {
    for (auto it = subscribers_.begin(); it != subscribers_.end(); ++it) {
        if (it->first == id) {
            subscribers_.erase(it);
            return;
        }
    }
    throw std::invalid_argument("unknown subscription");
}

std::size_t NullRecorderSink::rows_written() const noexcept {
    return rows_;
}

std::size_t NullRecorderSink::subscriber_count() const noexcept {
    return subscribers_.size();
}

}  // namespace mechsim::kernel
