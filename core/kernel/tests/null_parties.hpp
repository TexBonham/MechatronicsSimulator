#pragma once

#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel::test {

struct StepEvent {
    std::string party;
    SimTime t;
    Duration dt;
};

// Appends every step and fault notification to a log the test owns; throws from step() at fail_at if set.
class LoggingParty final : public Party {
public:
    LoggingParty(std::string name, Rate rate, std::vector<StepEvent>& steps, std::vector<std::string>& faults)
        : name_(std::move(name)), rate_(rate), steps_(steps), faults_(faults) {}

    void fail_at(SimTime t) {
        fail_ = true;
        fail_at_ = t;
    }

    [[nodiscard]] std::string_view name() const override { return name_; }
    [[nodiscard]] Rate nominal_rate() const override { return rate_; }
    [[nodiscard]] std::span<const std::string> published() const override { return published_; }
    [[nodiscard]] std::span<const std::string> consumed() const override { return consumed_; }

    void set_coupling(std::vector<std::string> published, std::vector<std::string> consumed) {
        published_ = std::move(published);
        consumed_ = std::move(consumed);
    }

    void start(const Rate& base_rate) override { base_rate_ = base_rate; }
    [[nodiscard]] const Rate& base_rate() const noexcept { return base_rate_; }

    void step(SimTime t, Duration dt) override {
        if (fail_ && t == fail_at_) {
            throw std::runtime_error("injected failure");
        }
        steps_.push_back(StepEvent{name_, t, dt});
    }

    void on_fault(const PartyFault& fault) noexcept override { faults_.push_back(name_ + ":" + fault.party()); }

private:
    std::string name_;
    Rate rate_;
    std::vector<StepEvent>& steps_;
    std::vector<std::string>& faults_;
    std::vector<std::string> published_;
    std::vector<std::string> consumed_;
    Rate base_rate_;
    bool fail_ = false;
    SimTime fail_at_;
};

}  // namespace mechsim::kernel::test
