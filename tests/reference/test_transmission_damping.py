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
from mechsim.builder import BuildResult, build
from mechsim.builder.physics import transmission_damping
from mechsim.design.schema import Design, shipped_materials
from mechsim.library.registry import default_registry
from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.neutral import PhysicsModel
from tests.reference.test_member_damping import (
    CORRECTED_TOLERANCE,
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


# ===== A member riding the driven carriage (BLD-041, PHY-037) ===========

CARRIAGE = 2.0
ARM_BODY = 0.1
ARM_SIDE = 0.04
ARM_LENGTH = 0.25
ARM_SEGMENTS = 8
ARM_MATERIAL = "aluminium_6063_t5_extrusion"


def _declared(mass: float, izz: str) -> dict[str, Any]:
    return {
        "mass": f"{mass} kg",
        "com": ["0 m", "0 m", "0 m"],
        "inertia": {"ixx": "0.01 kg*m**2", "iyy": "0.01 kg*m**2", "izz": izz},
    }


def _box(position: str, size: list[str]) -> dict[str, Any]:
    return {"kind": "primitives", "shapes": [{"shape": "box", "position": [position, "0 m", "0 m"], "size": size}]}


def member_belt_data(member: bool) -> dict[str, Any]:
    """A carriage on a prismatic joint along world x, belt-driven (catalog GT2 belt, k = 1e5 N/m at the output, zeta
    0.01) from a shaft of axial inertia 1e9 kg m^2 (held, as `belt_model`), with an aluminium arm fixed to it; the arm
    is a clamped-free lumped member when `member`, else rigid. No gravity."""
    arm: dict[str, Any] = {
        "name": "arm",
        "material": ARM_MATERIAL,
        "process": {"kind": "extrusion"},
        "geometry": _box("0.125 m", ["0.25 m", "0.04 m", "0.04 m"]),
        "mass_properties": _declared(ARM_BODY, "0.01 kg*m**2"),
    }
    if member:
        arm["flexible"] = {
            "section": {
                "area": f"{ARM_SIDE**2!r} m**2",
                "second_moment_y": f"{ARM_SIDE**4 / 12.0!r} m**4",
                "second_moment_z": f"{ARM_SIDE**4 / 12.0!r} m**4",
                "fibre_y": f"{ARM_SIDE / 2.0!r} m",
                "fibre_z": f"{ARM_SIDE / 2.0!r} m",
                "length": f"{ARM_LENGTH!r} m",
            },
            "supports": ["clamped", "free"],
            "segments": ARM_SEGMENTS,
        }
    return {
        "name": "member_belt",
        "version": "1.0.0",
        "source_hash": "unset",
        "status": "release",
        "environment": {"gravity": ["0 m/s**2", "0 m/s**2", "0 m/s**2"], "ambient_temperature": "293.15 K"},
        "bodies": [
            {
                "name": "shaft",
                "material": "bearing_steel_52100",
                "process": {"kind": "machined"},
                "geometry": _box("0 m", ["0.01 m", "0.01 m", "0.01 m"]),
                "mass_properties": _declared(0.1, "1e9 kg*m**2"),
            },
            {
                "name": "carriage",
                "material": ARM_MATERIAL,
                "process": {"kind": "extrusion"},
                "geometry": _box("0 m", ["0.05 m", "0.05 m", "0.05 m"]),
                "mass_properties": _declared(CARRIAGE, "0.01 kg*m**2"),
            },
            arm,
        ],
        "joints": [
            {
                "name": "shaft_spin",
                "type": "revolute",
                "parent": "world",
                "child": "shaft",
                "placement": {"position": ["0 m", "-0.3 m", "0 m"]},
                "axis": [0.0, 0.0, 1.0],
                "angular": [{"friction": "0 N*m", "damping": "0 N*m*s/rad", "backlash": "0 rad"}],
            },
            {
                "name": "carriage_slide",
                "type": "prismatic",
                "parent": "world",
                "child": "carriage",
                "placement": {"position": ["0 m", "0 m", "0 m"]},
                "axis": [1.0, 0.0, 0.0],
                "linear": [{"friction": "0 N", "damping": "0 N*s/m", "backlash": "0 m"}],
            },
            {
                "name": "arm_mount",
                "type": "fixed",
                "parent": "carriage",
                "child": "arm",
                "placement": {"position": ["0.03 m", "0 m", "0 m"]},
            },
        ],
        "components": [
            {
                "name": "belt",
                "type": "belt_pulley",
                "part": "gt2_20t_6mm_generic",
                "overrides": {"damping_ratio": "0.01"},
                "mechanical": [
                    {"mount": "pulley_bore", "joint": "shaft_spin"},
                    {"mount": "belt_clamp", "body": "carriage"},
                ],
            }
        ],
    }


def member_belt_build(member: bool) -> BuildResult:
    return build(Design.model_validate(member_belt_data(member)), default_registry(), shipped_materials())


def chain_mass() -> float:
    """The engine's segment bodies of a clamped-free chain (`core/engine` lumped beam): N - 1 spans of l = L / N and a
    half span at the tip, the root's half span left on the parent, so rho A (L - l / 2)."""
    density = shipped_materials()[ARM_MATERIAL].density.typ
    return density * ARM_SIDE**2 * (ARM_LENGTH - ARM_LENGTH / ARM_SEGMENTS / 2.0)


@pytest.mark.req("BLD-041", "PHY-037")
def test_belt_mode_decrement_with_a_member_on_the_carriage_is_the_stated_ratio() -> None:
    """The driven mass is the carriage, the arm's body and the arm's chain: m = 2 + 0.1 + rho A (L - l / 2) = 3.11 kg
    (aluminium 2700 kg/m^3, 40 mm square, L = 0.25 m, N = 8), omega = sqrt(k / m) = 179 rad/s. The arm is stiff: its
    chain's first mode is about 3.5 sqrt(E I / (rho A L^4)) = 3.3e3 rad/s, 18 times the belt mode, so at the belt mode
    it rides as a rigid mass to (omega / omega_1)^2 times its modal mass, under 0.1 percent of m (a 0.03 percent shift
    in zeta, inside the 0.2 percent scheme-corrected tolerance). D from the build counts m whole, so the measured
    decrement is the stated 0.01's; an anchor without the chain (2.1 kg) would realise 0.01 sqrt(2.1 / 3.11) = 0.0082,
    18 percent low."""
    zeta = 0.01
    result = member_belt_build(True)
    assert result.physics is not None
    model = result.physics
    mass = CARRIAGE + ARM_BODY + chain_mass()
    omega = math.sqrt(STIFFNESS / mass)
    assert float(model.transmissions[0].damping) == pytest.approx(
        transmission_damping(zeta, STIFFNESS, mass), rel=1e-12
    )
    substeps = substeps_for(omega, zeta)
    prepared = prepare_model(model, {"carriage_slide": STRETCH, "shaft_spin": 0.0}, 1.0 / HZ, substeps=substeps)
    adapter = core.make_physics_adapter("native")
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(HZ))
    carriage = [body.name for body in model.bodies].index("carriage")
    samples: list[float] = []
    for _ in range(math.ceil(7 * 2.0 * math.pi / omega * HZ)):
        adapter.step()
        samples.append(float(adapter.outputs().bodies[carriage].position[0]))
    measured = measured_decrement(lobe_peaks(samples), 3)
    h = 1.0 / (HZ * substeps)
    target = closed_decrement(zeta)
    corrected = scheme_decrement(zeta, omega, h)
    print(
        f"belt with member, m = {mass:.6g} kg: closed form {target:.6g}, measured {measured:.6g} "
        f"({measured / target - 1:+.3%}), scheme-corrected {corrected:.6g} ({measured / corrected - 1:+.3%})"
    )
    assert abs(measured - target) <= TOLERANCE * target
    assert abs(measured - corrected) <= CORRECTED_TOLERANCE * corrected
