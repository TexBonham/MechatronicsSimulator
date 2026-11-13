#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace mechsim::integrity {

// A malformed limits table (INT-023): the party is never built from it.
class LimitsError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

enum class SupportCondition : std::uint8_t { clamped, pinned };

struct Support {
    double at = 0.0;  // m along the member
    SupportCondition condition = SupportCondition::clamped;
};

// One structural member, SI, the member frame of ruling 24 (x start to end, y and z the section's principal axes).
struct MemberLimits {
    std::string name;
    std::size_t index = 0;  // in PhysicsOutputs::members
    double area = 0.0;
    double i_y = 0.0;
    double i_z = 0.0;
    double c_y = 0.0;
    double c_z = 0.0;
    double length = 0.0;
    double elastic_modulus = 0.0;
    double effective_length_factor = 1.0;
    double allowable_stress = 0.0;  // conservative strength over the minimum safety factor
    double yield_strength = 0.0;    // conservative yield strength, no safety factor (tier 2 yield)
    std::optional<double> deflection_limit;
    std::vector<Support> supports;
    // The lumped chain the physics adapter steps (tier 2 yield): its first node spring in the adapter's spring order,
    // N springs at (j + 1/2) l, each of stiffness k = E I / l.
    std::size_t first_spring = 0;
    std::size_t segments = 0;
    double segment_length = 0.0;
    double node_stiffness = 0.0;
};

// A rated fastening (fastener set or mount): the neutral joint whose reaction it carries.
struct FasteningLimits {
    std::string name;
    std::size_t joint = 0;    // in PhysicsOutputs::joints
    double load_limit = 0.0;  // N: the rating over the minimum safety factor
};

struct Limits {
    std::vector<MemberLimits> members;
    std::vector<FasteningLimits> fastenings;
    double minimum_safety_factor = 1.0;
    bool stress = false;
    bool buckling = false;
    bool deflection = false;
    bool fastener = false;
    bool yield_offset = false;
};

inline constexpr std::string_view limits_format = "mechsim-integrity-limits-1";

// Parses the table mechsim/integrity/tier1.py writes; throws LimitsError naming the field.
[[nodiscard]] Limits parse_limits(std::string_view json);

}  // namespace mechsim::integrity
