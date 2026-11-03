"""Unit tier stubs for the design spec (DES), strict xfail until converted."""

import pytest


@pytest.mark.req("DES-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_017() -> None:
    """STEP files shall be read with Open CASCADE through the OCP Python bindings for body geometry."""
    raise NotImplementedError


@pytest.mark.req("DES-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_019() -> None:
    """STEP import shall export collision meshes by convex decomposition with a library (CoACD), producing convex
    hulls.
    """
    raise NotImplementedError


@pytest.mark.req("DES-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_020() -> None:
    """An imported body whose collision geometry needs true mesh contact shall be flagged at import as outside the v1
    contact scope.
    """
    raise NotImplementedError


@pytest.mark.req("DES-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_021() -> None:
    """URDF files shall be parsed directly into bodies and joints."""
    raise NotImplementedError


@pytest.mark.req("DES-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_022() -> None:
    """Declared mass properties from Fusion or URDF shall be used as the design's values, and the values recomputed
    from STEP shall be compared with them and a mismatch beyond tolerance reported at import.
    """
    raise NotImplementedError


@pytest.mark.req("DES-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_023() -> None:
    """An imported STEP body shall be declarable as a catalog part, and the design shall then take its component type
    and parameters from that part.
    """
    raise NotImplementedError


@pytest.mark.req("DES-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_024() -> None:
    """A URDF joint type with no matching library joint type shall be rejected at import with an error naming the
    joint.
    """
    raise NotImplementedError


@pytest.mark.req("DES-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_029() -> None:
    """Every pin-binding table entry shall name a pin and peripheral mode that the chip's pin map provides, and any
    other entry shall be rejected at load.
    """
    raise NotImplementedError


@pytest.mark.req("DES-041")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_041() -> None:
    """A design file shall declare the electrical wiring between component instances (driver to motor, supply to
    driver) and the consumers of each power rail, so the builder can gate motor power and wire the signal layer
    (BLD-006, G7, G8).
    """
    raise NotImplementedError


@pytest.mark.req("DES-042")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_042() -> None:
    """A design file's environment shall declare the contact model defaults, per-material-pair overrides, body-pair
    exclusions and the floor plane, from which the builder fills the neutral model's contact section."""
    raise NotImplementedError
