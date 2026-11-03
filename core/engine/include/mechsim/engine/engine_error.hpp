#pragma once

#include <stdexcept>
#include <string>

namespace mechsim::engine {

class EngineError : public std::runtime_error {
public:
    EngineError(double time, std::string cause);

    [[nodiscard]] double time() const noexcept;
    [[nodiscard]] const std::string& cause() const noexcept;

private:
    double time_;
    std::string cause_;
};

class NonFiniteStateError : public EngineError {
public:
    using EngineError::EngineError;
};

class SolverNonConvergenceError : public EngineError {
public:
    using EngineError::EngineError;
};

class InvalidModelError : public EngineError {
public:
    using EngineError::EngineError;
};

// Raised before any step, so its time is 0.
class InvalidConfigError : public EngineError {
public:
    using EngineError::EngineError;
};

}  // namespace mechsim::engine
