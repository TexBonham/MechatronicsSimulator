#include "mechsim/kernel/coupling.hpp"

#include <algorithm>
#include <cstdint>
#include <span>
#include <string>
#include <typeinfo>
#include <string_view>
#include <variant>
#include <vector>

#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

namespace {

bool listed(std::span<const std::string> names, std::string_view name) {
    return std::ranges::any_of(names, [name](const std::string& candidate) { return candidate == name; });
}

bool same_payload_type(const CouplingPayload& a, const CouplingPayload& b) {
    const std::type_info& left = typeid(a);
    const std::type_info& right = typeid(b);
    return left == right;
}

}  // namespace

void CouplingTable::declare(const std::string& name, const std::string& publisher) {
    if (name.empty()) {
        throw KernelConfigError("party '" + publisher + "' publishes an empty coupling variable name");
    }
    const auto existing = slots_.find(name);
    if (existing != slots_.end()) {
        throw KernelConfigError("coupling variable '" + name + "' is published by both '" +
                                existing->second.publisher + "' and '" + publisher + "'");
    }
    slots_.emplace(name, Slot{publisher, CouplingValue{}});
}

bool CouplingTable::contains(std::string_view name) const {
    return slots_.find(name) != slots_.end();
}

const CouplingTable::Slot& CouplingTable::slot(std::string_view name) const {
    const auto found = slots_.find(name);
    if (found == slots_.end()) {
        throw CouplingError("unknown coupling variable '" + std::string(name) + "'");
    }
    return found->second;
}

const std::string& CouplingTable::publisher(std::string_view name) const {
    return slot(name).publisher;
}

const CouplingValue& CouplingTable::value(std::string_view name) const {
    return slot(name).value;
}

void CouplingTable::set(std::string_view name, const CouplingValue& value) {
    const auto found = slots_.find(name);
    if (found == slots_.end()) {
        throw CouplingError("unknown coupling variable '" + std::string(name) + "'");
    }
    if (std::holds_alternative<std::monostate>(value)) {
        throw CouplingError("coupling variable '" + std::string(name) + "' written with no value");
    }
    const PayloadPtr* incoming = std::get_if<PayloadPtr>(&value);
    if (incoming != nullptr && *incoming == nullptr) {
        throw CouplingError("coupling variable '" + std::string(name) + "' written with a null payload");
    }
    CouplingValue& held = found->second.value;
    if (!std::holds_alternative<std::monostate>(held) && held.index() != value.index()) {
        throw CouplingError("coupling variable '" + std::string(name) + "' changed type");
    }
    const PayloadPtr* previous = std::get_if<PayloadPtr>(&held);
    if (previous != nullptr && incoming != nullptr && !same_payload_type(**previous, **incoming)) {
        throw CouplingError("coupling variable '" + std::string(name) + "' changed payload type from " +
                            std::string((*previous)->kind()) + " to " + std::string((*incoming)->kind()));
    }
    held = value;
}

std::vector<CouplingSample> CouplingTable::snapshot() const {
    std::vector<CouplingSample> samples;
    samples.reserve(slots_.size());
    for (const auto& [name, held] : slots_) {
        samples.push_back(CouplingSample{name, held.value});
    }
    return samples;
}

void throw_payload_mismatch(std::string_view name, const CouplingValue& value) {
    std::string held = "a scalar";
    if (std::holds_alternative<std::monostate>(value)) {
        held = "no value";
    }
    if (const PayloadPtr* payload = std::get_if<PayloadPtr>(&value)) {
        held = std::string((*payload)->kind());
    }
    throw CouplingError("coupling variable '" + std::string(name) + "' holds " + held +
                        ", not the payload type this party reads");
}

CouplingReader::CouplingReader(const CouplingTable& table, std::span<const std::string> consumed) noexcept
    : table_(&table), consumed_(consumed) {}

const CouplingValue& CouplingReader::value(std::string_view name) const {
    if (!listed(consumed_, name)) {
        throw CouplingError("coupling variable '" + std::string(name) + "' is not consumed by this party");
    }
    return table_->value(name);
}

bool CouplingReader::has_value(std::string_view name) const {
    return !std::holds_alternative<std::monostate>(value(name));
}

double CouplingReader::real(std::string_view name) const {
    const CouplingValue& held = value(name);
    if (!std::holds_alternative<double>(held)) {
        throw CouplingError("coupling variable '" + std::string(name) + "' is not a real value");
    }
    return std::get<double>(held);
}

std::int64_t CouplingReader::integer(std::string_view name) const {
    const CouplingValue& held = value(name);
    if (!std::holds_alternative<std::int64_t>(held)) {
        throw CouplingError("coupling variable '" + std::string(name) + "' is not an integer value");
    }
    return std::get<std::int64_t>(held);
}

bool CouplingReader::flag(std::string_view name) const {
    const CouplingValue& held = value(name);
    if (!std::holds_alternative<bool>(held)) {
        throw CouplingError("coupling variable '" + std::string(name) + "' is not a boolean value");
    }
    return std::get<bool>(held);
}

CouplingWriter::CouplingWriter(CouplingTable& table, std::span<const std::string> published) noexcept
    : table_(&table), published_(published) {}

void CouplingWriter::set(std::string_view name, const CouplingValue& value) {
    if (!listed(published_, name)) {
        throw CouplingError("coupling variable '" + std::string(name) + "' is not published by this party");
    }
    table_->set(name, value);
}

}  // namespace mechsim::kernel
