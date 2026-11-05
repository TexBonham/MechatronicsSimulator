#include "mechsim/engine/transmission.hpp"

#include <cmath>
#include <numbers>
#include <string>

#include "mechsim/engine/engine_error.hpp"

namespace mechsim::engine {

namespace {

bool in_unit_interval(double x, bool zero_allowed) {
    if (zero_allowed) {
        return x >= 0.0 && x <= 1.0;
    }
    return x > 0.0 && x <= 1.0;
}

}  // namespace

double output_per_input(const Transmission& transmission) {
    double n = transmission.ratio;
    if (transmission.type == TransmissionType::LeadScrew) {
        n = transmission.ratio / (2.0 * std::numbers::pi);
    }
    if (transmission.type == TransmissionType::GearPair) {
        n = 1.0 / transmission.ratio;
    }
    if (transmission.reverses) {
        n = -n;
    }
    return n;
}

void validate_transmission_parameters(const Transmission& transmission, Eigen::Index index) {
    const std::string label = "transmission " + std::to_string(index) + ": ";
    if (!std::isfinite(transmission.ratio) || transmission.ratio <= 0.0) {
        throw InvalidModelError(0.0, label + "ratio must be finite and positive");
    }
    if (!in_unit_interval(transmission.efficiency, false)) {
        throw InvalidModelError(0.0, label + "efficiency must be in (0, 1]");
    }
    if (!in_unit_interval(transmission.back_efficiency, false)) {
        throw InvalidModelError(0.0, label + "back efficiency must be in (0, 1]");
    }
    if (!std::isfinite(transmission.stiffness) || transmission.stiffness <= 0.0) {
        throw InvalidModelError(0.0, label + "stiffness must be finite and positive");
    }
    if (!std::isfinite(transmission.damping) || transmission.damping < 0.0) {
        throw InvalidModelError(0.0, label + "damping must be finite and non-negative");
    }
    if (!std::isfinite(transmission.backlash) || transmission.backlash < 0.0) {
        throw InvalidModelError(0.0, label + "backlash must be finite and non-negative");
    }
    if (!in_unit_interval(transmission.slip, true)) {
        throw InvalidModelError(0.0, label + "slip must be in [0, 1]");
    }
    if (transmission.self_locking && transmission.type != TransmissionType::LeadScrew) {
        throw InvalidModelError(0.0, label + "only a lead screw can be self-locking");
    }
}

TransmissionCoupling couple(const Transmission& transmission, double slip, double slip_offset, double theta,
                            double omega, double x, double xdot, double drive) {
    TransmissionCoupling c;
    c.n = output_per_input(transmission);
    c.n_eff = (1.0 - slip) * c.n;
    TransmissionOutput& out = c.output;
    out.slip = slip;
    out.slip_active = slip > 0.0;
    out.slip_offset = slip_offset;
    out.stretch = c.n * theta - x - slip_offset;
    const double half = 0.5 * transmission.backlash;
    double deflection = 0.0;
    out.backlash_position = out.stretch;
    if (out.stretch > half) {
        deflection = out.stretch - half;
        out.backlash_position = half;
        out.in_contact = true;
    }
    if (out.stretch < -half) {
        deflection = out.stretch + half;
        out.backlash_position = -half;
        out.in_contact = true;
    }
    if (out.in_contact) {
        const double rate = c.n_eff * omega - xdot;
        out.force = transmission.stiffness * deflection + transmission.damping * rate;
    }
    const double lossless = -c.n * out.force;
    const double power = lossless * omega;
    if (power < 0.0) {
        out.power_flow = PowerFlow::Forward;
        c.gain = c.n / transmission.efficiency;
    } else if (power > 0.0) {
        out.power_flow = PowerFlow::Backward;
        c.gain = c.n * transmission.back_efficiency;
        if (transmission.self_locking) {
            out.power_flow = PowerFlow::Locked;
            c.gain = 0.0;
        }
    } else {
        out.power_flow = PowerFlow::Static;
        c.gain = c.n;
        if (transmission.self_locking && drive * lossless >= 0.0) {
            out.power_flow = PowerFlow::Locked;
            c.gain = 0.0;
        }
    }
    out.shaft_load_torque = -c.gain * out.force;
    return c;
}

}  // namespace mechsim::engine
