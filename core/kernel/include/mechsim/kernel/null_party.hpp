#pragma once

#include <span>
#include <string>
#include <string_view>

#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

// A party that does nothing at its rate, publishing and consuming nothing: the tier 1 integrity slot in the KER-009
// order until milestone 3 replaces it (ADR 0010).
class NullParty final : public Party {
public:
    NullParty(std::string name, Rate rate);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] Rate nominal_rate() const override;
    [[nodiscard]] std::span<const std::string> published() const override;
    [[nodiscard]] std::span<const std::string> consumed() const override;
    void start(const Rate& base_rate) override;
    void step(SimTime t, Duration dt) override;
    void on_fault(const PartyFault& fault) noexcept override;

private:
    std::string name_;
    Rate rate_;
};

}  // namespace mechsim::kernel
