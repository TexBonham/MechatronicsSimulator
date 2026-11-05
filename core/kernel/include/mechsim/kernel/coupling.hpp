#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace mechsim::kernel {

// An immutable structured coupling value (PhysicsOutputs, PinInputs, ...), shared between the table, the readers and
// the recorder rows; the kernel never looks inside. kind names the type in errors and in the recorder (KER-008).
class CouplingPayload {
public:
    CouplingPayload() = default;
    virtual ~CouplingPayload() = default;
    CouplingPayload(const CouplingPayload&) = delete;
    CouplingPayload& operator=(const CouplingPayload&) = delete;
    CouplingPayload(CouplingPayload&&) = delete;
    CouplingPayload& operator=(CouplingPayload&&) = delete;

    [[nodiscard]] virtual std::string_view kind() const noexcept = 0;
};

template <typename T>
class TypedPayload final : public CouplingPayload {
public:
    TypedPayload(T value, std::string kind) : value_(std::move(value)), kind_(std::move(kind)) {}

    [[nodiscard]] std::string_view kind() const noexcept override {
        return kind_;
    }
    [[nodiscard]] const T& get() const noexcept {
        return value_;
    }

private:
    T value_;
    std::string kind_;
};

using PayloadPtr = std::shared_ptr<const CouplingPayload>;

// A coupling variable's value at a sync point (KER-008). monostate: not yet published this run. Once published, a
// variable keeps its alternative (and, for a payload, its payload type) for the whole run.
using CouplingValue = std::variant<std::monostate, double, std::int64_t, bool, PayloadPtr>;

template <typename T>
[[nodiscard]] CouplingValue make_payload(T value, std::string kind) {
    return CouplingValue{PayloadPtr{std::make_shared<const TypedPayload<T>>(std::move(value), std::move(kind))}};
}

// The typed content of a payload value; throws CouplingError naming the variable when it holds anything else.
template <typename T>
[[nodiscard]] const T& payload_as(const CouplingValue& value, std::string_view name);

// One coupling variable as streamed in a tick row (KER-022).
struct CouplingSample {
    std::string name;
    CouplingValue value;
};

// Misuse of the coupling table inside a step: reading a name the party does not consume, writing one it does not
// publish, changing a variable's type, or reading the wrong type. Thrown into the party; the kernel turns it into a
// PartyFault.
class CouplingError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// The kernel's value table, keyed by dotted signal name, in name order (no hashing, invariant 4). Each name has
// exactly one publisher.
class CouplingTable {
public:
    // Throws KernelConfigError if the name is empty or already has a publisher.
    void declare(const std::string& name, const std::string& publisher);

    [[nodiscard]] bool contains(std::string_view name) const;
    [[nodiscard]] const std::string& publisher(std::string_view name) const;
    [[nodiscard]] const CouplingValue& value(std::string_view name) const;
    // Throws CouplingError on an undeclared name, a monostate value or a change of alternative.
    void set(std::string_view name, const CouplingValue& value);

    [[nodiscard]] std::vector<CouplingSample> snapshot() const;

private:
    struct Slot {
        std::string publisher;
        CouplingValue value;
    };

    [[nodiscard]] const Slot& slot(std::string_view name) const;

    std::map<std::string, Slot, std::less<>> slots_;
};

// A party's read view of the table at the start of its step: only its consumed names (KER-008, KER-020).
class CouplingReader {
public:
    CouplingReader(const CouplingTable& table, std::span<const std::string> consumed) noexcept;

    [[nodiscard]] const CouplingValue& value(std::string_view name) const;
    [[nodiscard]] bool has_value(std::string_view name) const;
    [[nodiscard]] double real(std::string_view name) const;
    [[nodiscard]] std::int64_t integer(std::string_view name) const;
    [[nodiscard]] bool flag(std::string_view name) const;
    template <typename T>
    [[nodiscard]] const T& payload(std::string_view name) const {
        return payload_as<T>(value(name), name);
    }

private:
    const CouplingTable* table_;
    std::span<const std::string> consumed_;
};

// A party's write view of the table at the end of its step: only its published names. A name not written keeps its
// last value (zero-order hold, KER-011).
class CouplingWriter {
public:
    CouplingWriter(CouplingTable& table, std::span<const std::string> published) noexcept;

    void set(std::string_view name, const CouplingValue& value);

private:
    CouplingTable* table_;
    std::span<const std::string> published_;
};

[[noreturn]] void throw_payload_mismatch(std::string_view name, const CouplingValue& value);

template <typename T>
const T& payload_as(const CouplingValue& value, std::string_view name) {
    if (const PayloadPtr* held = std::get_if<PayloadPtr>(&value)) {
        if (const auto* typed = dynamic_cast<const TypedPayload<T>*>(held->get())) {
            return typed->get();
        }
    }
    throw_payload_mismatch(name, value);
}

}  // namespace mechsim::kernel
