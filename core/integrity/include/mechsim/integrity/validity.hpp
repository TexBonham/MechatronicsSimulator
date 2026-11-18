#pragma once

#include <cstddef>
#include <stdexcept>

namespace mechsim::integrity {

// A member index the validity source does not hold (ruling 79): an error, never read as valid.
class ValidityError : public std::out_of_range {
public:
    using std::out_of_range::out_of_range;
};

// The engine's live carried-load validity per member, as the tier 1 party reads it (rulings 69, 75, 79). Declared
// here so the integrity package depends on no adapter (invariant 5): the physics binding fills it from its adapter.
class ValiditySource {
public:
    ValiditySource() = default;
    ValiditySource(const ValiditySource&) = default;
    ValiditySource& operator=(const ValiditySource&) = default;
    ValiditySource(ValiditySource&&) = default;
    ValiditySource& operator=(ValiditySource&&) = default;
    virtual ~ValiditySource() = default;

    // True while the member's one-way carried-load result is not valid (flagged at or before the current tick);
    // `member` is MemberLimits::index, the physics outputs' member order. Throws ValidityError for an index the
    // source does not hold.
    [[nodiscard]] virtual bool not_valid(std::size_t member) const = 0;
};

}  // namespace mechsim::integrity
