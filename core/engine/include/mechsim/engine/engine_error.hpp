#pragma once

#include <cstddef>
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

// Constraint drift past the threshold (ENG-024); carries the offending constraint's index.
class DriftError : public EngineError {
public:
    DriftError(double time, std::string cause, std::ptrdiff_t constraint);

    [[nodiscard]] std::ptrdiff_t constraint_index() const noexcept;

private:
    std::ptrdiff_t constraint_;
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
