"""Mass properties of primitive solids against closed forms (BLD-010).

Tolerance. The spec leaves the BLD-010 tolerance not set (owed by the owner, proposed 1e-9 relative). The bound used
here is a property of the arithmetic, tighter than the proposal: each closed form and each builder value is a product
or sum of at most about 30 double operations on exact inputs, each rounding by at most 2^-53 ~ 1.1e-16 relative, so
the two agree to within 30 * 1.1e-16 ~ 3.3e-15; 1e-12 bounds that with margin and still catches any formula error,
which shows at the percent level. Absolute terms that are exactly zero in the closed form get 1e-12 times the
largest moment, the same rounding argument applied to cancelling sums.
"""

from __future__ import annotations

import math
from typing import Any

import pytest

from mechsim.builder import build
from mechsim.builder.physics import primitive_mass
from mechsim.design.schema import PrimitiveGeometry, shipped_materials
from mechsim.library.registry import default_registry
from tests.unit.test_builder_support import MATERIAL, filled_gantry, gantry_data

RHO = 2700.0
REL = 1e-12


def _geometry(*shapes: dict[str, Any]) -> PrimitiveGeometry:
    return PrimitiveGeometry.model_validate({"kind": "primitives", "shapes": list(shapes)})


def _check(name: str, actual: float, expected: float, scale: float) -> None:
    print(f"{name}: closed form {expected!r}, builder {actual!r}")
    assert abs(actual - expected) <= REL * scale


def _assert(geometry: PrimitiveGeometry, mass: float, com: tuple[float, ...], diagonal: tuple[float, ...]) -> None:
    result = primitive_mass(geometry, RHO)
    scale = max(diagonal)
    _check("mass", result.mass, mass, mass)
    for axis in range(3):
        _check(f"com[{axis}]", result.com[axis], com[axis], max(1.0, abs(com[axis])))
        _check(f"I[{axis}][{axis}]", result.inertia[axis][axis], diagonal[axis], scale)
        for other in range(3):
            if other != axis:
                _check(f"I[{axis}][{other}]", result.inertia[axis][other], 0.0, scale)


@pytest.mark.req("BLD-010")
def test_box() -> None:
    """m = rho a b c; Ixx = m (b^2 + c^2) / 12 and cyclic."""
    a, b, c = 0.3, 0.2, 0.1
    m = RHO * a * b * c
    geometry = _geometry({"shape": "box", "position": ["0 m"] * 3, "size": [f"{a} m", f"{b} m", f"{c} m"]})
    _assert(
        geometry, m, (0.0, 0.0, 0.0), (m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12)
    )


@pytest.mark.req("BLD-010")
def test_cylinder() -> None:
    """Along z: m = rho pi r^2 L; Izz = m r^2 / 2; Ixx = Iyy = m (3 r^2 + L^2) / 12."""
    r, length = 0.05, 0.4
    m = RHO * math.pi * r * r * length
    geometry = _geometry({"shape": "cylinder", "position": ["0 m"] * 3, "radius": f"{r} m", "length": f"{length} m"})
    side = m * (3 * r * r + length * length) / 12
    _assert(geometry, m, (0.0, 0.0, 0.0), (side, side, m * r * r / 2))


@pytest.mark.req("BLD-010")
def test_sphere() -> None:
    """m = rho 4/3 pi r^3; I = 2/5 m r^2."""
    r = 0.07
    m = RHO * 4 / 3 * math.pi * r**3
    geometry = _geometry({"shape": "sphere", "position": ["0 m"] * 3, "radius": f"{r} m"})
    _assert(geometry, m, (0.0, 0.0, 0.0), (0.4 * m * r * r,) * 3)


@pytest.mark.req("BLD-010")
def test_capsule() -> None:
    """Cylinder L between two hemispherical caps, each of mass m_h = rho 2/3 pi r^3. Axial: m_c r^2 / 2 + 2 (2/5 m_h
    r^2). Transverse: m_c (3 r^2 + L^2) / 12 plus, per cap, its own 83/320 m_h r^2 about its center of mass and
    m_h d^2 with d = L/2 + 3 r / 8, the cap's center of mass from the capsule center."""
    r, length = 0.04, 0.3
    m_c = RHO * math.pi * r * r * length
    m_h = RHO * 2 / 3 * math.pi * r**3
    d = length / 2 + 3 * r / 8
    side = m_c * (3 * r * r + length * length) / 12 + 2 * (83 / 320 * m_h * r * r + m_h * d * d)
    axial = m_c * r * r / 2 + 2 * (2 / 5 * m_h * r * r)
    geometry = _geometry({"shape": "capsule", "position": ["0 m"] * 3, "radius": f"{r} m", "length": f"{length} m"})
    _assert(geometry, m_c + 2 * m_h, (0.0, 0.0, 0.0), (side, side, axial))


@pytest.mark.req("BLD-010")
def test_capsule_tends_to_a_sphere() -> None:
    """With L -> 0 the capsule is a sphere: the hemisphere moments must sum to 2/5 m r^2 about the center, a check on
    83/320 and 3/8 independent of the capsule formula (83/320 + 9/64 = 2/5)."""
    r = 0.05
    capsule = primitive_mass(
        _geometry({"shape": "capsule", "position": ["0 m"] * 3, "radius": f"{r} m", "length": "1e-30 m"}), RHO
    )
    m = RHO * 4 / 3 * math.pi * r**3
    _check("mass", capsule.mass, m, m)
    _check("Ixx", capsule.inertia[0][0], 0.4 * m * r * r, 0.4 * m * r * r)
    _check("Izz", capsule.inertia[2][2], 0.4 * m * r * r, 0.4 * m * r * r)


@pytest.mark.req("BLD-010")
def test_placed_and_turned_cylinder() -> None:
    """A cylinder turned 90 degrees about y lies along x: Ixx = m r^2 / 2, Iyy = Izz = m (3 r^2 + L^2) / 12; placed at
    p its center of mass is p."""
    r, length = 0.05, 0.4
    half = math.sqrt(0.5)
    m = RHO * math.pi * r * r * length
    side = m * (3 * r * r + length * length) / 12
    geometry = _geometry(
        {
            "shape": "cylinder",
            "position": ["0.25 m", "-0.5 m", "1 m"],
            "orientation": [half, 0.0, half, 0.0],
            "radius": f"{r} m",
            "length": f"{length} m",
        }
    )
    _assert(geometry, m, (0.25, -0.5, 1.0), (m * r * r / 2, side, side))


@pytest.mark.req("BLD-010")
def test_two_spheres_shift_by_the_parallel_axis_rule() -> None:
    """Spheres at x = +-d: com 0; Ixx = 2 (2/5 m r^2); Iyy = Izz = 2 (2/5 m r^2 + m d^2)."""
    r, d = 0.03, 0.2
    m = RHO * 4 / 3 * math.pi * r**3
    geometry = _geometry(
        {"shape": "sphere", "position": [f"{d} m", "0 m", "0 m"], "radius": f"{r} m"},
        {"shape": "sphere", "position": [f"{-d} m", "0 m", "0 m"], "radius": f"{r} m"},
    )
    own = 0.4 * m * r * r
    _assert(geometry, 2 * m, (0.0, 0.0, 0.0), (2 * own, 2 * (own + m * d * d), 2 * (own + m * d * d)))


@pytest.mark.req("BLD-010")
def test_offset_pair_has_the_product_of_inertia() -> None:
    """Two spheres at (d, d, 0) and (-d, -d, 0): the tensor entry Ixy = -sum m x y = -2 m d^2."""
    r, d = 0.02, 0.1
    m = RHO * 4 / 3 * math.pi * r**3
    geometry = _geometry(
        {"shape": "sphere", "position": [f"{d} m", f"{d} m", "0 m"], "radius": f"{r} m"},
        {"shape": "sphere", "position": [f"{-d} m", f"{-d} m", "0 m"], "radius": f"{r} m"},
    )
    result = primitive_mass(geometry, RHO)
    expected = -2 * m * d * d
    _check("Ixy", result.inertia[0][1], expected, abs(expected))
    _check("Iyx", result.inertia[1][0], expected, abs(expected))


@pytest.mark.req("BLD-010")
def test_built_body_uses_the_material_density() -> None:
    """End to end: the filled gantry frame is a 0.1 m cube of the shipped aluminium, m = rho a^3, I = m a^2 / 6."""
    density = shipped_materials()[MATERIAL].density.typ
    data = gantry_data()
    model = build(filled_gantry(data), default_registry(), shipped_materials()).physics
    assert model is not None
    frame = model.bodies[0]
    m = density * 0.1**3
    _check("mass", frame.mass, m, m)
    _check("ixx", frame.inertia.ixx, m * 0.01 / 6, m * 0.01 / 6)
