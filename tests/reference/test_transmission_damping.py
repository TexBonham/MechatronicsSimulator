"""Transmission damping through the native adapter (BLD-041, LIB-035, PHY-037, ruling 78).

A 2 kg carriage on a prismatic joint driven by a belt (k = 1e5 N/m at the output, pulley radius 0.01 m) from a shaft
of axial inertia 1e9 kg m^2: the coupling's force moves the shaft by m r^2 / I = 2e-13 of the carriage's travel, so the
shaft is held and the carriage is a mass on the belt's spring-damper, omega = sqrt(k / m) = 223.6 rad/s, no gravity.
D is the builder's conversion D = 2 zeta sqrt(k m) (`physics.transmission_damping`), which makes the mode's damping
ratio zeta exactly.

Release: the carriage starts 0.01 m out with the shaft at 0 (an explicit shaft start, so the belt starts stretched)
and rings about x = 0; the peaks of its positive lobes decay at Lambda = 2 pi zeta / sqrt(1 - zeta^2).

Bound: 2 percent of Lambda. Backward Euler adds about pi omega h per period; with omega h <= zeta / 100 that is at
most 0.5 percent; the scheme-corrected decrement is checked separately at the same tolerance. One degree of freedom,
so no higher modes; peaks refined by a parabola over 280 or more samples per period (below 1e-6 relative).
"""

from __future__ import annotations

import math
from typing import Any

import pytest

from mechsim import _core as core
from mechsim.builder.physics import transmission_damping
from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.neutral import PhysicsModel
from tests.reference.test_member_damping import (
    HZ,
    TOLERANCE,
    closed_decrement,
    lobe_peaks,
    measured_decrement,
    scheme_decrement,
    substeps_for,
)
from tests.reference.test_phy_stubs import ORIGIN, base

MASS = 2.0
STIFFNESS = 1e5
STRETCH = 0.01


def _body(name: str, mass: float, kind: str, axis: tuple[float, float, float], izz: str) -> dict[str, Any]:
    return {
        "name": name,
        "mass": f"{mass} kg",
        "com": ORIGIN,
        "inertia": {"ixx": "0.01 kg*m**2", "iyy": "0.01 kg*m**2", "izz": izz},
        "material": "steel",
        "parent": "world",
        "joint": {"name": name + "_joint", "type": kind, "position": ORIGIN, "axis": axis},
    }


def belt_model(damping: float) -> PhysicsModel:
    data = base(("0 m/s**2", "0 m/s**2", "0 m/s**2"))
    data["bodies"] = [
        _body("shaft", 0.1, "revolute", (0.0, 0.0, 1.0), "1e9 kg*m**2"),
        _body("carriage", MASS, "prismatic", (1.0, 0.0, 0.0), "0.01 kg*m**2"),
    ]
    data["transmissions"] = [
        {
            "type": "belt_pulley",
            "name": "belt",
            "shaft_joint": "shaft_joint",
            "driven_joint": "carriage_joint",
            "efficiency": 1.0,
            "pulley_radius": "0.01 m",
            "stiffness": f"{STIFFNESS} N/m",
            "damping": f"{damping} N*s/m",
        }
    ]
    return PhysicsModel.model_validate(data)


@pytest.mark.req("BLD-041", "PHY-037")
@pytest.mark.parametrize(("zeta", "periods"), [(0.01, 4), (0.05, 3)])
def test_belt_mode_decrement_is_the_closed_form(zeta: float, periods: int) -> None:
    omega = math.sqrt(STIFFNESS / MASS)
    model = belt_model(transmission_damping(zeta, STIFFNESS, MASS))
    substeps = substeps_for(omega, zeta)
    prepared = prepare_model(model, {"carriage_joint": STRETCH, "shaft_joint": 0.0}, 1.0 / HZ, substeps=substeps)
    adapter = core.make_physics_adapter("native")
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(HZ))
    samples: list[float] = []
    for _ in range(math.ceil((periods + 4) * 2.0 * math.pi / omega * HZ)):
        adapter.step()
        samples.append(float(adapter.outputs().bodies[1].position[0]))
    measured = measured_decrement(lobe_peaks(samples), periods)
    h = 1.0 / (HZ * substeps)
    target = closed_decrement(zeta)
    corrected = scheme_decrement(zeta, omega, h)
    print(
        f"belt zeta {zeta}: closed form {target:.6g}, measured {measured:.6g} ({measured / target - 1:+.3%}), "
        f"scheme-corrected {corrected:.6g} ({measured / corrected - 1:+.3%}), x = {omega * h:.3g}"
    )
    assert abs(measured - target) <= TOLERANCE * target
    assert abs(measured - corrected) <= TOLERANCE * corrected
