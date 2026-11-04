#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel::test {

// Where a ScriptedParty throws an injected failure.
enum class FailIn : std::uint8_t { none, seed, start, read, step, write, record };

// A party driven by test lambdas. read_inputs copies every consumed value; step(t, dt) logs "name@t" to a log the
// test owns and calls on_step with those inputs; write_outputs calls on_write with the step's start time.
class ScriptedParty final : public Party {
public:
    using Inputs = std::map<std::string, CouplingValue, std::less<>>;
    using StepFn = std::function<void(SimTime, Duration, const Inputs&)>;
    using WriteFn = std::function<void(SimTime, CouplingWriter&)>;
    using RecordFn = std::function<void(TickRecord&)>;

    ScriptedParty(std::string name, Rate rate, std::vector<std::string>& log)
        : name_(std::move(name)), rate_(rate), log_(&log) {}

    std::vector<std::string> published_names;
    std::vector<std::string> consumed_names;
    StepFn on_step;
    WriteFn on_write;
    RecordFn on_record;
    std::int64_t finish_after_steps = -1;
    FailIn fail_in = FailIn::none;
    SimTime fail_at;
    bool throw_non_std = false;
    PartyRole party_role = PartyRole::other;
    // Base ticks per step, so a read failure can be placed at the coming step's start time.
    std::int64_t period_hint = 1;

    [[nodiscard]] std::string_view name() const override {
        return name_;
    }
    [[nodiscard]] Rate nominal_rate() const override {
        return rate_;
    }
    [[nodiscard]] PartyRole role() const override {
        return party_role;
    }
    [[nodiscard]] std::span<const std::string> published() const override {
        return published_names;
    }
    [[nodiscard]] std::span<const std::string> consumed() const override {
        return consumed_names;
    }

    void set_seed(std::uint64_t child_seed) override {
        maybe_fail(FailIn::seed, SimTime());
        log_->push_back(name_ + ":seed");
        seed_ = child_seed;
    }

    void start(const Rate& base_rate) override {
        maybe_fail(FailIn::start, SimTime());
        log_->push_back(name_ + ":start");
        base_rate_ = base_rate;
    }

    void read_inputs(const CouplingReader& inputs) override {
        maybe_fail(FailIn::read, SimTime(steps_ * period_hint));
        inputs_.clear();
        for (const std::string& name : consumed_names) {
            inputs_.emplace(name, inputs.value(name));
        }
    }

    void step(SimTime t, Duration dt) override {
        maybe_fail(FailIn::step, t);
        log_->push_back(name_ + "@" + std::to_string(t.count()));
        last_t_ = t;
        ++steps_;
        if (on_step) {
            on_step(t, dt, inputs_);
        }
    }

    void write_outputs(CouplingWriter& outputs) override {
        maybe_fail(FailIn::write, last_t_);
        if (on_write) {
            on_write(last_t_, outputs);
        }
    }

    [[nodiscard]] bool finished() const override {
        return finish_after_steps >= 0 && steps_ >= finish_after_steps;
    }

    void record(TickRecord& row) const override {
        if (fail_in == FailIn::record && row.time == fail_at) {
            throw std::runtime_error("injected failure");
        }
        if (on_record) {
            on_record(row);
        }
    }

    void on_fault(const PartyFault& fault) noexcept override {
        log_->push_back(name_ + ":fault:" + fault.party());
    }

    [[nodiscard]] std::uint64_t seed() const noexcept {
        return seed_;
    }
    [[nodiscard]] const Rate& base_rate() const noexcept {
        return base_rate_;
    }

private:
    void maybe_fail(FailIn where, SimTime t) const {
        if (fail_in != where || t != fail_at) {
            return;
        }
        if (throw_non_std) {
            throw 7;
        }
        throw std::runtime_error("injected failure");
    }

    std::string name_;
    Rate rate_;
    std::vector<std::string>* log_;
    Inputs inputs_;
    SimTime last_t_;
    std::int64_t steps_ = 0;
    std::uint64_t seed_ = 0;
    Rate base_rate_;
};

}  // namespace mechsim::kernel::test
