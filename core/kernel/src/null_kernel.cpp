#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "mechsim/kernel/kernel.hpp"

namespace mechsim::kernel {

NullKernel::NullKernel(RunConfig config) : config_(std::move(config)) {}

void NullKernel::register_party(std::unique_ptr<Party> party) {
    if (ran_) {
        throw KernelConfigError("party registered after the run started");
    }
    if (party == nullptr) {
        throw KernelConfigError("null party");
    }
    const std::string name(party->name());
    if (name.empty()) {
        throw KernelConfigError("party name is empty");
    }
    for (const Entry& entry : parties_) {
        if (entry.party->name() == name) {
            throw KernelConfigError("duplicate party name '" + name + "'");
        }
    }
    Rate rate = party->nominal_rate();
    const auto override_rate = config_.party_rates.find(name);
    if (override_rate != config_.party_rates.end()) {
        rate = override_rate->second;
    }
    parties_.push_back(Entry{std::move(party), rate, Duration()});
}

void NullKernel::prepare_rates() {
    Rate base = config_.physics_rate;
    for (const Entry& entry : parties_) {
        try {
            base = lcm(base, entry.rate);
        } catch (const KernelConfigError& error) {
            throw KernelConfigError("party '" + std::string(entry.party->name()) + "': " + error.what());
        }
    }
    config_.base_rate = base;
    for (Entry& entry : parties_) {
        entry.period = period_in_ticks(entry.rate, base);
    }
}

void NullKernel::fan_out(const PartyFault& fault) noexcept {
    for (const Entry& entry : parties_) {
        entry.party->on_fault(fault);
    }
}

RunReport NullKernel::run(const EndCondition& end, RecorderSink& sink) {
    if (ran_) {
        throw KernelConfigError("run() called twice");
    }
    ran_ = true;
    prepare_rates();
    const Rate base = config_.base_rate;
    const SimTime end_time = ticks_at(end.end_time, base);
    const Duration physics_period = period_in_ticks(config_.physics_rate, base);
    for (const Entry& entry : parties_) {
        entry.party->start(base);
    }
    RunReport report;
    report.base_rate = base;
    while (now_ < end_time) {
        for (const Entry& entry : parties_) {
            if (now_.count() % entry.period.count() != 0) {
                continue;
            }
            std::optional<PartyFault> fault;
            try {
                entry.party->step(now_, entry.period);
            } catch (const PartyFault& thrown) {
                fault.emplace(std::string(entry.party->name()), now_, thrown.cause());
            } catch (const std::exception& error) {
                fault.emplace(std::string(entry.party->name()), now_, error.what());
            } catch (...) {
                fault.emplace(std::string(entry.party->name()), now_, "unknown exception");
            }
            if (fault.has_value()) {
                const PartyFault& raised = fault.value();
                fan_out(raised);
                report.fault = raised;
                report.outcome = RunOutcome::run_error;
                report.end_reason = EndReason::run_error;
                report.end_time = now_;
                return report;
            }
        }
        now_ = now_ + Duration(1);
        ++report.ticks;
        if (now_.count() % physics_period.count() == 0) {
            TickRecord row;
            row.time = now_;
            sink.write_row(row);
        }
        if (end.stop && end.stop(now_)) {
            report.end_reason = EndReason::stop_requested;
            break;
        }
    }
    report.outcome = RunOutcome::end_condition_met;
    report.end_time = now_;
    return report;
}

SimTime NullKernel::now() const {
    return now_;
}

const RunConfig& NullKernel::config() const {
    return config_;
}

}  // namespace mechsim::kernel
