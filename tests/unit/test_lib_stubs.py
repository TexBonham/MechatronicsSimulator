"""Unit tier stubs for the component library spec (LIB), strict xfail until converted."""

import pytest


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


@pytest.mark.req("LIB-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_019() -> None:
    """The shipped catalog shall provide the types load cell, load-cell amplifier, IMU and watchdog."""
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


@pytest.mark.req("LIB-027")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_027() -> None:
    """Power and safety hardware types shall carry cutoff delay, brake torque and heartbeat timeout parameters where
    they apply.
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
