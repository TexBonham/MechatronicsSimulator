"""Unit tier stubs for the component library spec (LIB), strict xfail until converted."""

import pytest


@pytest.mark.req("LIB-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_001() -> None:
    """Every component type shall declare a mechanical facet holding its mass, mounts and shaft axis as parameters."""
    raise NotImplementedError


@pytest.mark.req("LIB-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_002() -> None:
    """Every component type shall declare an electrical facet listing its pins and the signals it emits or consumes."""
    raise NotImplementedError


@pytest.mark.req("LIB-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_003() -> None:
    """Every component type shall declare a ratings facet holding its absolute maximums, safe operating area and, where
    applicable, holding torque.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_004() -> None:
    """Every component type shall carry a thermal facet slot whose value is held constant in v1, so that no v1
    computation changes it during a run.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_005() -> None:
    """Every component type shall name its fidelity tiers, and a component instance shall select one of the tiers its
    type names.
    """
    raise NotImplementedError


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


@pytest.mark.req("LIB-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_011() -> None:
    """Every component type shall define its parameters as a pydantic v2 model from which its JSON Schema entry is
    generated.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_012() -> None:
    """The library shall parse every quantity in a catalog or instance file from a unit string with pint at load and
    hold it as a plain SI float afterward.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_013() -> None:
    """A unit string whose dimension does not match its parameter shall be rejected at load with an error naming the
    file, the parameter and the expected dimension.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_014() -> None:
    """A missing required parameter shall be rejected at load with an error naming the file, the component type and the
    parameter.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_015() -> None:
    """An impossible parameter value, such as a negative mass or a step angle outside its type's declared range, shall
    be rejected at load with an error naming the file, the parameter, the value and the violated bound.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_016() -> None:
    """Every parameter of every component type shall be addressable by a sweep or variation definition and overridable
    for one run without editing the catalog.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_017() -> None:
    """A sweep shall be able to substitute one catalog part for another of the same component type, such as one stepper
    size for another.
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


@pytest.mark.req("LIB-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_024() -> None:
    """Transmission types shall carry ratio, efficiency, stiffness, backlash, self-locking and slip parameters,
    with slip zero by default."""
    raise NotImplementedError


@pytest.mark.req("LIB-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_025() -> None:
    """Sensor types shall carry resolution, noise, delay, sample rate and failure-mode parameters."""
    raise NotImplementedError


@pytest.mark.req("LIB-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_026() -> None:
    """Drivers and sensor front-ends shall be component types separate from the motors and sensors they serve."""
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


@pytest.mark.req("LIB-033")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_033() -> None:
    """No pint quantity or unit string shall be reachable from a loaded component type's parameter values."""
    raise NotImplementedError
