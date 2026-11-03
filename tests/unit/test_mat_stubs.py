"""Unit tier stubs for the materials spec (MAT), strict xfail until converted."""

import pytest


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
