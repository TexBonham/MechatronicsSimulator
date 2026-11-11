"""Unit tier stubs for the component library spec (LIB), strict xfail until converted. Converted here: LIB-019 and
LIB-027, each shipped milestone 2 part building its C++ built-in from its own catalog values."""

from typing import Any

import pytest

from mechsim import _core as core
from mechsim.library.registry import default_registry

# Catalog parameter -> make_signal_component parameter, per built-in; parameters absent here (mass, supply and coil
# ratings) describe the part, not the signal model.
BUILTIN_PARAMETERS: dict[str, dict[str, str]] = {
    "load_cell": {
        name: name
        for name in (
            "capacity",
            "sensitivity",
            "excitation_voltage",
            "resolution",
            "overload_limit",
            "nonlinearity",
            "noise",
            "delay",
            "sample_rate",
        )
    },
    "load_cell_amplifier": {name: name for name in ("gain", "offset", "output_full_scale", "bandwidth")},
    "imu": {
        name: name
        for name in (
            "accel_range",
            "gyro_range",
            "resolution",
            "gyro_resolution",
            "noise",
            "gyro_noise",
            "delay",
            "sample_rate",
        )
    },
    "watchdog": {name: name for name in ("timeout", "cutoff_delay", "manual_reset")},
    "brake": {name: name for name in ("brake_torque", "engage_delay", "release_delay", "power_off_engaged")},
}
PLACEMENT: dict[str, dict[str, Any]] = {"load_cell": {"payload": 0}, "imu": {"body": 0}, "brake": {"joint": 0}}


def built_in(part_name: str) -> Any:
    registry = default_registry()
    part = registry.resolve_part(part_name)
    component_type = registry.component_type(part.type)
    assert component_type.plugin.cpp == part.type
    mapping = BUILTIN_PARAMETERS[part.type]
    params: dict[str, Any] = dict(PLACEMENT.get(part.type, {}))
    for name, value in part.values.items():
        if name in mapping:
            params[mapping[name]] = value
    component = core.make_signal_component(component_type.plugin.cpp, "part", params)
    component.start(core.ComponentClock(core.Rate(1000), core.Duration(1), 7))
    return component


@pytest.mark.req("LIB-019")
@pytest.mark.parametrize(
    ("part", "outputs"),
    [
        ("load_cell_5kg_generic", ["part.signal"]),
        ("load_cell_amplifier_generic", ["part.out"]),
        (
            "imu_6axis_generic",
            ["part.accel_x", "part.accel_y", "part.accel_z", "part.gyro_x", "part.gyro_y", "part.gyro_z"],
        ),
        ("watchdog_24v_generic", ["part.tripped", "part.contact_closed"]),
    ],
)
def test_lib_019_catalog_types_build_their_cpp_built_in(part: str, outputs: list[str]) -> None:
    """The shipped catalog shall provide the types load cell, load-cell amplifier, IMU and watchdog."""
    assert built_in(part).outputs() == outputs


@pytest.mark.req("LIB-027")
@pytest.mark.parametrize(
    ("part", "inputs"),
    [("brake_24v_generic", ["part.release", "part.vcc"]), ("watchdog_24v_generic", ["part.heartbeat"])],
)
def test_lib_027_safety_parts_carry_their_parameters_into_the_built_in(part: str, inputs: list[str]) -> None:
    """Power and safety hardware types shall carry cutoff delay, brake torque and heartbeat timeout parameters where
    they apply."""
    assert built_in(part).inputs() == inputs


@pytest.mark.req("LIB-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_007() -> None:
    """A user shall be able to add a new component type through the plugin interface without changing any file outside
    the plugin and its catalog entry.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_008() -> None:
    """A built-in component type and a user-added component type shall implement the same plugin interface and shall be
    checked by the same reference tests.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_009() -> None:
    """A user-added component type written in Python shall run through the per-tick slow path without being compiled
    into the core runtime.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_018() -> None:
    """The shipped catalog shall provide the types rigid body, revolute, prismatic, fixed, planar and spherical joints,
    stepper (behavioral), stepper driver, belt and pulley, lead screw, limit switch, encoder, power supply and
    E-stop relay.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_020() -> None:
    """The shipped catalog shall provide the types flexible beam, hook and rope."""
    raise NotImplementedError


@pytest.mark.req("LIB-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_023() -> None:
    """Joint types shall carry range, friction, damping and backlash parameters, and revolute, prismatic, fixed, planar
    and spherical joints shall support travel limits.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-028")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_028() -> None:
    """A catalog part shall be referenceable from an imported STEP body, so that the body takes its component type and
    parameters from the catalog part.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_029() -> None:
    """The rigid body type shall carry mass, center of mass, inertia, geometry and material parameters."""
    raise NotImplementedError


@pytest.mark.req("LIB-030")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_030() -> None:
    """The flexible beam type shall carry cross-section and material parameters in addition to the rigid body
    parameters.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_031() -> None:
    """The library shall model rigid payloads with mass and attach point, and end effectors with capacity and attach
    and release events.
    """
    raise NotImplementedError
