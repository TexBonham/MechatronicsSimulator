#include "mechsim/engine/engine_error.hpp"

#include <sstream>
#include <string>
#include <utility>

namespace mechsim::engine {

namespace {

std::string format_message(double time, const std::string& cause) {
    std::ostringstream out;
    out.precision(17);
    out << "engine error at t = " << time << " s: " << cause;
    return out.str();
}

}  // namespace

EngineError::EngineError(double time, std::string cause)
    : std::runtime_error(format_message(time, cause)), time_(time), cause_(std::move(cause)) {}

double EngineError::time() const noexcept { return time_; }

const std::string& EngineError::cause() const noexcept { return cause_; }

}  // namespace mechsim::engine
