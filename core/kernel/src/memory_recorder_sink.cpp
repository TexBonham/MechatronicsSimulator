#include "mechsim/kernel/memory_recorder_sink.hpp"

#include <algorithm>
#include <cstddef>
#include <exception>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mechsim::kernel {

void InMemoryRecorderSink::write_row(const TickRecord& row) {
    if (dispatching_) {
        throw std::logic_error("write_row called from a subscriber");
    }
    rows_.push_back(row);
    const TickRecord& stored = rows_.back();
    dispatching_ = true;
    for (Subscriber& subscriber : subscribers_) {
        if (!subscriber.active) {
            continue;
        }
        try {
            subscriber.callback(stored);
        } catch (const std::exception& error) {
            subscriber.active = false;
            dropped_.push_back(subscriber.id);
            subscriber_errors_.emplace_back(error.what());
        } catch (...) {
            subscriber.active = false;
            dropped_.push_back(subscriber.id);
            subscriber_errors_.emplace_back("unknown exception");
        }
    }
    dispatching_ = false;
    compact();
}

void InMemoryRecorderSink::compact() {
    for (Subscriber& joining : pending_) {
        subscribers_.push_back(std::move(joining));
    }
    pending_.clear();
    std::erase_if(subscribers_, [](const Subscriber& subscriber) { return !subscriber.active; });
}

SubscriptionId InMemoryRecorderSink::subscribe(TickCallback callback) {
    if (!callback) {
        throw std::invalid_argument("empty subscriber callback");
    }
    const SubscriptionId id(next_id_);
    ++next_id_;
    if (dispatching_) {
        pending_.push_back(Subscriber{id, std::move(callback), true});
    } else {
        subscribers_.push_back(Subscriber{id, std::move(callback), true});
    }
    return id;
}

void InMemoryRecorderSink::unsubscribe(SubscriptionId id) {
    for (std::vector<Subscriber>* list : {&subscribers_, &pending_}) {
        for (Subscriber& subscriber : *list) {
            if (subscriber.id == id && subscriber.active) {
                subscriber.active = false;
                if (!dispatching_) {
                    compact();
                }
                return;
            }
        }
    }
    const auto dropped = std::ranges::find(dropped_, id);
    if (dropped != dropped_.end()) {
        dropped_.erase(dropped);
        return;
    }
    throw std::invalid_argument("unknown subscription");
}

std::span<const TickRecord> InMemoryRecorderSink::rows() const noexcept {
    return rows_;
}

std::vector<TickRecord> InMemoryRecorderSink::drain() {
    std::vector<TickRecord> fresh(rows_.begin() + static_cast<std::ptrdiff_t>(drained_), rows_.end());
    drained_ = rows_.size();
    return fresh;
}

std::size_t InMemoryRecorderSink::drained() const noexcept {
    return drained_;
}

std::size_t InMemoryRecorderSink::subscriber_count() const noexcept {
    return static_cast<std::size_t>(
        std::ranges::count_if(subscribers_, [](const Subscriber& subscriber) { return subscriber.active; }) +
        std::ranges::count_if(pending_, [](const Subscriber& subscriber) { return subscriber.active; }));
}

std::vector<std::string> InMemoryRecorderSink::subscriber_errors() const {
    return subscriber_errors_;
}

}  // namespace mechsim::kernel
