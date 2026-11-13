#include "mechsim/engine/transmission.hpp"

#include <algorithm>
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
                            double omega, double x, double xdot, double drive, double slip_threshold) {
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
    if (out.stretch >= half) {
        deflection = out.stretch - half;
        out.backlash_position = half;
        out.in_contact = true;
    } else if (out.stretch <= -half) {
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
    const double speed = std::abs(omega);
    double loss = 0.0;
    double full_gain = c.n;
    double damping_loss = std::max(1.0 / transmission.efficiency - 1.0, 1.0 - transmission.back_efficiency);
    out.power_flow = PowerFlow::Static;
    if (power < 0.0) {
        out.power_flow = PowerFlow::Forward;
        loss = 1.0 / transmission.efficiency - 1.0;
        full_gain = c.n / transmission.efficiency;
        damping_loss = loss;
    } else if (power > 0.0) {
        out.power_flow = PowerFlow::Backward;
        loss = transmission.back_efficiency - 1.0;
        full_gain = c.n * transmission.back_efficiency;
        damping_loss = -loss;
    }
    if (speed >= slip_threshold) {
        c.gain = full_gain;
        c.stiffness_gain = full_gain;
    } else {
        c.gain = c.n * (1.0 + (speed / slip_threshold) * loss);
        c.stiffness_gain = c.n;
        c.shaft_damping = damping_loss * std::abs(lossless) / slip_threshold;
    }
    bool locked = false;
    if (transmission.self_locking && out.in_contact && lossless == 0.0 && drive == 0.0) {
        c.lock = true;
        c.lock_bilateral = true;
        c.lock_sign = 1.0;
        locked = true;
        out.power_flow = PowerFlow::Locked;
    }
    if (transmission.self_locking && out.in_contact && lossless != 0.0) {
        const double opposition = drive * lossless;
        if (opposition <= 0.0) {
            c.lock = true;
            c.lock_sign = 1.0;
            if (lossless > 0.0) {
                c.lock_sign = -1.0;
            }
        }
        if (power >= 0.0 && opposition >= 0.0) {
            locked = true;
            out.power_flow = PowerFlow::Locked;
        }
        if (power >= 0.0 && opposition > 0.0) {
            c.gain = 0.0;
            c.stiffness_gain = 0.0;
            c.shaft_damping = 0.0;
        }
    }
    c.shaft_torque = -c.gain * out.force;
    out.shaft_load_torque = c.shaft_torque;
    if (locked) {
        out.shaft_load_torque = 0.0;
    }
    return c;
}

}  // namespace mechsim::engine
