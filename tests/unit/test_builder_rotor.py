"""Rotor inertia into the shaft body (BLD-034)."""

from __future__ import annotations

import math

import pytest

from mechsim.builder import BuildError, build
from mechsim.design.schema import MassProperties, shipped_materials
from mechsim.library.registry import default_registry
from tests.unit.test_builder_support import gantry


@pytest.mark.req("BLD-034")
def test_rotor_inertia_lands_on_the_shaft_body_axis() -> None:
    """Closed form: the placeholder shaft is a solid steel cylinder (bearing_steel_52100; body_materials covers the
    printed bodies only) of radius r = 2.5 mm and length L = 20 mm on the z axis, so m = rho pi r^2 L and
    I_xx = I_yy = m (3 r^2 + L^2) / 12. The shaft body's axial inertia is the rotor inertia (BLD-034): I_zz is set to
    the NEMA 23 placeholder's J = 280 g cm^2 = 2.8e-5 kg m^2, replacing the cylinder's m r^2 / 2, and only I_zz
    changes (the shaft axis is z, I + (J - a^T I a) a a^T). Tolerance: 1e-12 relative, the round-off of a sum of a few
    products of SI floats (BLD-034 row)."""
    registry = default_registry()
    materials = shipped_materials()
    result = build(gantry(), registry, materials, rigid_members=True)
    assert result.physics is not None
    density = materials["bearing_steel_52100"].density.typ
    r, length, rotor = 2.5e-3, 20e-3, 2.8e-5
    mass = density * math.pi * r * r * length
    transverse = mass * (3.0 * r * r + length * length) / 12.0
    shafts = {body.name: body for body in result.physics.bodies if body.name.endswith("_shaft")}
    assert sorted(shafts) == ["lift_motor_shaft", "x_motor_shaft", "y_motor_shaft"]
    for name, body in shafts.items():
        print(f"{name}: izz closed form {rotor:.9e}, built {body.inertia.izz:.9e}")
        assert body.inertia.izz == pytest.approx(rotor, rel=1e-12)
        assert body.inertia.ixy == 0.0
        assert body.inertia.ixx == pytest.approx(transverse, rel=1e-12)
        assert body.inertia.iyy == pytest.approx(transverse, rel=1e-12)
    frame = next(body for body in result.physics.bodies if body.name == "frame")
    assert frame.inertia.izz > 0.0
    assert result.physics.signals.load_torque_joints == ("x_motor_spin", "y_motor_spin", "lift_motor_spin")


@pytest.mark.req("BLD-034")
def test_a_shaft_body_declaring_its_own_inertia_is_a_build_error_naming_body_and_motor() -> None:
    design = gantry()
    declared = MassProperties.model_validate(
        {
            "mass": "0.001 kg",
            "com": ["0 m", "0 m", "0 m"],
            "inertia": {"ixx": "1e-7 kg*m**2", "iyy": "1e-7 kg*m**2", "izz": "5.4e-6 kg*m**2"},
        }
    )
    bodies = []
    for body in design.bodies:
        if body.name == "x_motor_shaft":
            body = body.model_copy(update={"mass_properties": declared})
        bodies.append(body)
    with pytest.raises(BuildError) as raised:
        build(design.model_copy(update={"bodies": bodies}), default_registry(), shipped_materials(), rigid_members=True)
    text = str(raised.value)
    assert "body 'x_motor_shaft'" in text
    assert "motor 'x_motor'" in text
    assert "BLD-034" in text
    assert "y_motor_shaft" not in text
