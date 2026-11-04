#include "mechsim/kernel/cosim_kernel.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace mechsim::kernel {

namespace {

const std::string recorder_party = "recorder";

std::string current_cause() {
    try {
        throw;
    } catch (const PartyFault& thrown) {
        return thrown.cause();
    } catch (const std::exception& error) {
        return error.what();
    } catch (...) {
        return "unknown exception";
    }
}

}  // namespace

std::uint64_t splitmix64_next(std::uint64_t& state) noexcept {
    state += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state;
    z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31U);
}

std::vector<std::uint64_t> derive_child_seeds(std::uint64_t seed, std::size_t count) {
    std::vector<std::uint64_t> seeds;
    seeds.reserve(count);
    std::uint64_t state = seed;
    for (std::size_t i = 0; i < count; ++i) {
        seeds.push_back(splitmix64_next(state));
    }
    return seeds;
}

CoSimKernel::CoSimKernel(RunConfig config) : config_(std::move(config)) {}

// ===== Registration ==================================================================================================

void CoSimKernel::register_party(std::unique_ptr<Party> party) {
    if (ran_) {
        throw KernelConfigError("party registered after the run started");
    }
    if (party == nullptr) {
        throw KernelConfigError("null party");
    }
    std::string name(party->name());
    if (name.empty()) {
        throw KernelConfigError("party name is empty");
    }
    for (const Entry& entry : parties_) {
        if (entry.name == name) {
            throw KernelConfigError("duplicate party name '" + name + "'");
        }
    }
    parties_.push_back(Entry{std::move(party), std::move(name), Rate(), RateSource::nominal, Duration(), 0});
}

// ===== Preparation before t = 0 ======================================================================================

void CoSimKernel::assign_rates() {
    for (Entry& entry : parties_) {
        const PartyRole role = entry.party->role();
        entry.rate = entry.party->nominal_rate();
        entry.source = RateSource::nominal;
        if (role == PartyRole::controller) {
            entry.rate = config_.control_rate;
            entry.source = RateSource::control_rate;
        }
        const auto override_rate = config_.party_rates.find(entry.name);
        if (override_rate != config_.party_rates.end()) {
            entry.rate = override_rate->second;
            entry.source = RateSource::override;
        }
        if (role == PartyRole::physics && !(entry.rate == config_.physics_rate)) {
            throw KernelConfigError("physics party '" + entry.name + "' runs at " +
                                    std::to_string(entry.rate.numerator()) + "/" +
                                    std::to_string(entry.rate.denominator()) + " Hz, not the physics rate " +
                                    std::to_string(config_.physics_rate.numerator()) + "/" +
                                    std::to_string(config_.physics_rate.denominator()) + " Hz");
        }
    }
}

CoSimKernel::Limits CoSimKernel::prepare(const EndCondition& end) {
    assign_rates();
    Rate base = config_.physics_rate;
    for (const Entry& entry : parties_) {
        try {
            base = lcm(base, entry.rate);
        } catch (const KernelConfigError& error) {
            throw KernelConfigError("party '" + entry.name + "': " + error.what());
        }
    }
    config_.base_rate = base;
    for (Entry& entry : parties_) {
        entry.period = period_in_ticks(entry.rate, base);
    }
    for (const auto& [name, rate] : config_.party_rates) {
        bool known = false;
        for (const Entry& entry : parties_) {
            if (entry.name == name) {
                known = true;
            }
        }
        if (!known) {
            throw KernelConfigError("party_rates names '" + name + "', which is not a registered party");
        }
    }

    Limits limits;
    limits.physics_period = period_in_ticks(config_.physics_rate, base);
    try {
        limits.end = ticks_at(end.end_time, base);
    } catch (const KernelConfigError& error) {
        throw KernelConfigError(std::string("end_time: ") + error.what());
    }
    if (triggers_.at_time.has_value()) {
        try {
            limits.at = ticks_at(triggers_.at_time.value(), base);
        } catch (const KernelConfigError& error) {
            throw KernelConfigError(std::string("at_time: ") + error.what());
        }
        if (limits.at.value() == SimTime()) {
            throw KernelConfigError("at_time must be after t = 0");
        }
        if (limits.at.value().count() % limits.physics_period.count() != 0) {
            throw KernelConfigError("at_time must be a whole number of physics steps");
        }
    }

    for (const Entry& entry : parties_) {
        for (const std::string& name : entry.party->published()) {
            coupling_.declare(name, entry.name);
        }
    }
    for (const Entry& entry : parties_) {
        for (const std::string& name : entry.party->consumed()) {
            if (!coupling_.contains(name)) {
                throw KernelConfigError("party '" + entry.name + "' consumes '" + name + "', which no party publishes");
            }
        }
    }
    if (!triggers_.on_signal.empty() && !coupling_.contains(triggers_.on_signal)) {
        throw KernelConfigError("end signal '" + triggers_.on_signal + "' is not published by any party");
    }
    if (!triggers_.party_state.empty()) {
        bool known = false;
        for (const Entry& entry : parties_) {
            if (entry.name == triggers_.party_state) {
                known = true;
            }
        }
        if (!known) {
            throw KernelConfigError("end party_state names '" + triggers_.party_state + "', which is not registered");
        }
    }
    return limits;
}

std::optional<PartyFault> CoSimKernel::start_parties() {
    const std::vector<std::uint64_t> seeds = derive_child_seeds(config_.seed, parties_.size());
    for (std::size_t i = 0; i < parties_.size(); ++i) {
        Entry& entry = parties_[i];
        try {
            entry.party->set_seed(seeds[i]);
            entry.party->start(config_.base_rate);
        } catch (...) {
            return PartyFault(entry.name, now_, current_cause());
        }
    }
    return std::nullopt;
}

// ===== Tick loop =====================================================================================================

std::optional<PartyFault> CoSimKernel::step_party(Entry& entry) {
    try {
        const CouplingReader reader(coupling_, entry.party->consumed());
        entry.party->read_inputs(reader);
        entry.party->step(now_, entry.period);
        CouplingWriter writer(coupling_, entry.party->published());
        entry.party->write_outputs(writer);
    } catch (...) {
        return PartyFault(entry.name, now_, current_cause());
    }
    ++entry.steps;
    return std::nullopt;
}

std::optional<PartyFault> CoSimKernel::write_row(RecorderSink& sink) {
    TickRecord row;
    row.time = now_;
    for (const Entry& entry : parties_) {
        try {
            entry.party->record(row);
        } catch (...) {
            return PartyFault(entry.name, now_, current_cause());
        }
    }
    row.coupling = coupling_.snapshot();
    try {
        sink.write_row(row);
    } catch (...) {
        return PartyFault(recorder_party, now_, current_cause());
    }
    return std::nullopt;
}

std::optional<EndReason> CoSimKernel::end_reached(const EndCondition& end, const Limits& limits,
                                                  std::optional<PartyFault>& fault) const {
    if (!triggers_.on_signal.empty()) {
        const CouplingValue& value = coupling_.value(triggers_.on_signal);
        if (std::holds_alternative<bool>(value)) {
            if (std::get<bool>(value)) {
                return EndReason::signal;
            }
        } else if (!std::holds_alternative<std::monostate>(value)) {
            fault.emplace(coupling_.publisher(triggers_.on_signal), now_,
                          "end signal '" + triggers_.on_signal + "' is not boolean");
            return EndReason::run_error;
        }
    }
    if (!triggers_.party_state.empty()) {
        for (const Entry& entry : parties_) {
            if (entry.name == triggers_.party_state && entry.party->finished()) {
                return EndReason::party_state;
            }
        }
    }
    if (end.stop && end.stop(now_)) {
        return EndReason::stop_requested;
    }
    if (limits.at.has_value() && now_ >= limits.at.value()) {
        return EndReason::at_time;
    }
    return std::nullopt;
}

RunReport CoSimKernel::finish(RunReport report, const RecorderSink& sink) const {
    report.end_time = now_;
    for (const Entry& entry : parties_) {
        report.step_counts.push_back(PartyStepCount{entry.name, entry.steps});
        report.party_rates.push_back(PartyRate{entry.name, entry.rate, entry.source});
    }
    report.sink_errors = sink.subscriber_errors();
    return report;
}

RunReport CoSimKernel::fail(RunReport report, const PartyFault& fault, const RecorderSink& sink) const {
    for (const Entry& entry : parties_) {
        entry.party->on_fault(fault);
    }
    report.outcome = RunOutcome::run_error;
    report.end_reason = EndReason::run_error;
    report.fault = fault;
    report = finish(std::move(report), sink);
    report.end_time = fault.time();
    return report;
}

RunReport CoSimKernel::run(const EndCondition& end, RecorderSink& sink) {
    if (ran_) {
        throw KernelConfigError("run() called twice");
    }
    ran_ = true;
    const Limits limits = prepare(end);

    RunReport report;
    report.base_rate = config_.base_rate;
    const std::optional<PartyFault> start_fault = start_parties();
    if (start_fault.has_value()) {
        return fail(std::move(report), start_fault.value(), sink);
    }
    while (now_ < limits.end) {
        for (Entry& entry : parties_) {
            if (now_.count() % entry.period.count() != 0) {
                continue;
            }
            const std::optional<PartyFault> fault = step_party(entry);
            if (fault.has_value()) {
                return fail(std::move(report), fault.value(), sink);
            }
        }
        now_ = now_ + Duration(1);
        if (now_.count() % limits.physics_period.count() != 0) {
            ++report.ticks;
            continue;
        }
        const std::optional<PartyFault> fault = write_row(sink);
        if (fault.has_value()) {
            return fail(std::move(report), fault.value(), sink);
        }
        ++report.ticks;
        std::optional<PartyFault> end_fault;
        const std::optional<EndReason> reason = end_reached(end, limits, end_fault);
        if (end_fault.has_value()) {
            return fail(std::move(report), end_fault.value(), sink);
        }
        if (reason.has_value()) {
            report.end_reason = reason.value();
            return finish(std::move(report), sink);
        }
    }
    report.end_reason = EndReason::max_duration;
    return finish(std::move(report), sink);
}

SimTime CoSimKernel::now() const {
    return now_;
}

const RunConfig& CoSimKernel::config() const {
    return config_;
}

void CoSimKernel::set_end_triggers(const EndTriggers& triggers) {
    if (ran_) {
        throw KernelConfigError("end triggers set after the run started");
    }
    triggers_ = triggers;
}

const CouplingTable& CoSimKernel::coupling() const noexcept {
    return coupling_;
}

}  // namespace mechsim::kernel
