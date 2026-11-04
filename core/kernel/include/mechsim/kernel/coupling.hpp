#pragma once

#include <cstdint>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace mechsim::kernel {

// A coupling variable's value at a sync point (KER-008). monostate: not yet published this run. Once published, a
// variable keeps its alternative for the whole run.
using CouplingValue = std::variant<std::monostate, double, std::int64_t, bool>;

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

}  // namespace mechsim::kernel
