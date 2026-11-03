"""Unit tier stubs for the materials spec (MAT), strict xfail until converted."""

import pytest


@pytest.mark.req("MAT-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_001() -> None:
    """Every material record shall hold density, Young's modulus, Poisson's ratio, yield strength, ultimate strength
    and shear strength.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_002() -> None:
    """Every material record shall hold thermal conductivity, specific heat, thermal expansion coefficient and maximum
    service temperature.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_003() -> None:
    """Every material record shall hold an S-N fatigue curve."""
    raise NotImplementedError


@pytest.mark.req("MAT-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_004() -> None:
    """Every material property shall carry a min, typical and max value."""
    raise NotImplementedError


@pytest.mark.req("MAT-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_005() -> None:
    """A property whose min, typical and max are not in non-decreasing order shall be rejected at load with an error
    naming the file, the material and the property.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_006() -> None:
    """Every material property shall accept a temperature dependence, and a lookup shall evaluate the property at the
    requested temperature.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_007() -> None:
    """A lookup at a temperature outside a property's stated temperature range shall end with an error rather than an
    extrapolated value.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_008() -> None:
    """A lookup shall return the min, typical or max value of a property as the caller requests."""
    raise NotImplementedError


@pytest.mark.req("MAT-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_009() -> None:
    """The database shall accept the process attributes printed (with print orientation, wall count and infill),
    machined and molded.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_010() -> None:
    """A material with a process shall hold strengths that depend on loading direction relative to the process
    orientation.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_011() -> None:
    """A strength lookup shall take a loading direction in the body frame and the body's process attributes and return
    the strength that applies to that direction.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_012() -> None:
    """An S-N curve lookup shall return the allowable stress amplitude for a cycle count inside the curve's data range
    and end with an error for a cycle count outside it.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_013() -> None:
    """The shipped database shall contain aluminum extrusion grades, steel fastener grades and common bearing steels."""
    raise NotImplementedError


@pytest.mark.req("MAT-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_014() -> None:
    """The shipped database shall contain PLA, PETG and ABS as printed materials with orientation and infill."""
    raise NotImplementedError


@pytest.mark.req("MAT-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_015() -> None:
    """Shipped process-aware properties for printed materials shall be conservative defaults with wide min to max
    spreads.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_016() -> None:
    """The team shall be able to override any property value and spread of a shipped material, for example from coupon
    tests, without editing the shipped database file.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_017() -> None:
    """An override shall be held in a data file that the run's design or project references, so that a run's material
    values are reproducible from its recorded inputs.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_019() -> None:
    """Every quantity in a materials file shall be a unit string parsed by pint at load and held as a plain SI float
    afterward.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_020() -> None:
    """The database shall supply density to the builder for mass and inertia computation."""
    raise NotImplementedError


@pytest.mark.req("MAT-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_021() -> None:
    """The database shall supply yield, ultimate and shear strength, the S-N curve and the direction-dependent
    strengths to integrity checks, with their spreads.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_022() -> None:
    """A lookup of a property absent from a material shall end with an error naming the material and the property,
    never a default value.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_023() -> None:
    """A reference to a material name the database does not hold shall be rejected at load with an error naming the
    reference and the file it appears in.
    """
    raise NotImplementedError


@pytest.mark.req("MAT-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_024() -> None:
    """A process attribute value outside its valid range, such as an infill above 100 percent or a wall count below
    one, shall be rejected at load.
    """
    raise NotImplementedError
