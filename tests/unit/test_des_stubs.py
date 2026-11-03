"""Unit tier stubs for the design spec (DES), strict xfail until converted."""

import pytest


@pytest.mark.req("DES-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_001() -> None:
    """A design file shall declare its bodies, each with a material and a process."""
    raise NotImplementedError


@pytest.mark.req("DES-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_002() -> None:
    """A design file shall declare its joints and mates, each with limits, friction and backlash."""
    raise NotImplementedError


@pytest.mark.req("DES-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_003() -> None:
    """A design file shall declare component instances, each naming a library type or catalog part and giving its
    parameters.
    """
    raise NotImplementedError


@pytest.mark.req("DES-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_004() -> None:
    """A design file shall declare cross-domain bindings, such as a motor shaft to a joint and a housing to a mount
    face.
    """
    raise NotImplementedError


@pytest.mark.req("DES-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_005() -> None:
    """A design file shall declare a pin-binding table that maps every controller-facing signal to an MCU pin and
    peripheral mode.
    """
    raise NotImplementedError


@pytest.mark.req("DES-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_006() -> None:
    """A design file shall declare its power hardware and rails."""
    raise NotImplementedError


@pytest.mark.req("DES-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_007() -> None:
    """A design file shall declare its environment, including gravity and ambient conditions."""
    raise NotImplementedError


@pytest.mark.req("DES-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_008() -> None:
    """A design file shall carry a semver design version, and the loader shall expose it together with the design
    file's git blob hash.
    """
    raise NotImplementedError


@pytest.mark.req("DES-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_009() -> None:
    """Every signal name in a design shall have the dotted form `instance.signal`, and a name of any other form shall
    be rejected at load.
    """
    raise NotImplementedError


@pytest.mark.req("DES-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_010() -> None:
    """Every signal name shall be unique within its design, and a duplicate shall be rejected at load with an error
    naming both declarations.
    """
    raise NotImplementedError


@pytest.mark.req("DES-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_013() -> None:
    """Every quantity in a design file shall be a unit string parsed by pint at load and held as a plain SI float
    afterward.
    """
    raise NotImplementedError


@pytest.mark.req("DES-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_014() -> None:
    """Loading a design file with ruamel.yaml in round-trip mode and writing it back unchanged shall reproduce the file
    byte for byte, comments and key order included.
    """
    raise NotImplementedError


@pytest.mark.req("DES-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_015() -> None:
    """Every design validation shall run at load or import, before any run starts, and each error shall name the file,
    the path within it and the reason.
    """
    raise NotImplementedError


@pytest.mark.req("DES-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_016() -> None:
    """A reference to a library type, catalog part, material, body, joint or signal that does not exist shall be
    rejected at load with an error naming the reference.
    """
    raise NotImplementedError


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


@pytest.mark.req("DES-027")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_027() -> None:
    """A CAD file referenced from a design shall be resolved relative to the design file, and a missing file shall be
    rejected at load.
    """
    raise NotImplementedError


@pytest.mark.req("DES-028")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_028() -> None:
    """A joint limit whose lower bound exceeds its upper bound shall be rejected at load."""
    raise NotImplementedError


@pytest.mark.req("DES-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_029() -> None:
    """Every pin-binding table entry shall name a pin and peripheral mode that the chip's pin map provides, and any
    other entry shall be rejected at load.
    """
    raise NotImplementedError


@pytest.mark.req("DES-030")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_030() -> None:
    """Every component instance's parameters shall be validated at load against its library type's parameter model."""
    raise NotImplementedError


@pytest.mark.req("DES-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_031() -> None:
    """A design file shall carry its design-load rule as data."""
    raise NotImplementedError


@pytest.mark.req("DES-032")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_032() -> None:
    """A design file shall carry its accuracy target used as the deflection limit for structural checks."""
    raise NotImplementedError


@pytest.mark.req("DES-033")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_033() -> None:
    """A design file shall declare which members are flexible and the cross-section of each."""
    raise NotImplementedError


@pytest.mark.req("DES-034")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_034() -> None:
    """A design file shall declare, for each flexible member, the known deflection and first natural frequency it is
    calibrated to, from hand calculation or FEA.
    """
    raise NotImplementedError


@pytest.mark.req("DES-035")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_035() -> None:
    """Imported FEA results (stiffness, deflection, natural frequencies) for a flexible member shall be accepted into
    the design together with the design version they belong to.
    """
    raise NotImplementedError


@pytest.mark.req("DES-036")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_036() -> None:
    """Loading and validating a design shall be callable without starting a run and shall return every error found, for
    the CLI validate command.
    """
    raise NotImplementedError


@pytest.mark.req("DES-037")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_037() -> None:
    """An imported body's material and process shall be declared in the design file when the CAD source does not supply
    them, and a body without them shall be rejected at load.
    """
    raise NotImplementedError
