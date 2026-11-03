"""Strict-xfail stubs for the INT spec (reference tier)."""

import pytest


@pytest.mark.req("INT-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_004() -> None:
    """Each check shall ship with an analytical reference test that its result matches within the tolerance
    this spec states for it.
    """
    raise NotImplementedError


@pytest.mark.req("INT-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_005() -> None:
    """The strength check shall compute each member's and mount's peak stress from the recorded member end-loads using
    beam and plate theory on its section, and its safety factor against the material's process- and direction-dependent
    limits.
    """
    raise NotImplementedError


@pytest.mark.req("INT-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_006() -> None:
    """The deflection check shall compute each member's peak deflection over the run."""
    raise NotImplementedError


@pytest.mark.req("INT-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_007() -> None:
    """The buckling check shall compute each compression member's buckling margin over the run."""
    raise NotImplementedError


@pytest.mark.req("INT-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_008() -> None:
    """The fatigue check shall count each member's stress cycles from the recorded loads of a run and
    compare them with the material's S-N curve.
    """
    raise NotImplementedError


@pytest.mark.req("INT-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_010() -> None:
    """The fastener check shall compute each fastener's peak load and its margin against the fastener
    grade's limit.
    """
    raise NotImplementedError


@pytest.mark.req("INT-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_011() -> None:
    """The impact check shall evaluate members and mounts against the peak loads of impact events in the
    run.
    """
    raise NotImplementedError


@pytest.mark.req("INT-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_012() -> None:
    """The resonance screening check shall flag each flexible member whose natural frequency falls within
    the screening band of an excitation present in the run.
    """
    raise NotImplementedError


@pytest.mark.req("INT-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_013() -> None:
    """The bearing life check shall compute each bearing's rated life from its recorded loads."""
    raise NotImplementedError
