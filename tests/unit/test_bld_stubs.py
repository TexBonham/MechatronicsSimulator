"""Unit tier stubs for the builder spec (BLD), strict xfail until converted."""

import pytest


@pytest.mark.req("BLD-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_001() -> None:
    """The builder shall turn a validated design into one engine-neutral physics model made of bodies, joints,
    constraints, contacts, lumped members and transmissions.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_004() -> None:
    """A kinematic loop in the design shall be emitted as a constraint in the neutral model."""
    raise NotImplementedError


@pytest.mark.req("BLD-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_005() -> None:
    """Belts, lead screws and gears shall be emitted as our transmission models in the neutral form, not as engine
    features.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_006() -> None:
    """The builder shall emit the signal-layer wiring for every component instance's signals and every cross-domain
    binding.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_007() -> None:
    """The builder shall emit the chip and pin configuration from the design's pin-binding table."""
    raise NotImplementedError


@pytest.mark.req("BLD-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_008() -> None:
    """The builder shall emit a structural member list naming every structural member and mount of the design."""
    raise NotImplementedError


@pytest.mark.req("BLD-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_011() -> None:
    """Where a design declares mass properties, the builder shall use the declared values; it shall compute them only
    where none are declared.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_013() -> None:
    """The contacts element shall express floor contact, a body catching an obstacle and a carriage hitting a hard stop
    using primitives and convex hulls only.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_014() -> None:
    """Every flexible member shall be emitted as a multi-segment lumped beam."""
    raise NotImplementedError


@pytest.mark.req("BLD-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_022() -> None:
    """Overlapping bodies shall end the build with a report naming both bodies."""
    raise NotImplementedError


@pytest.mark.req("BLD-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_023() -> None:
    """A controller-facing signal with no pin binding shall end the build with a report naming the signal."""
    raise NotImplementedError


@pytest.mark.req("BLD-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_024() -> None:
    """When the build targets an adapter, an element of the neutral model that adapter cannot map shall end the build
    with a report naming the element and the adapter.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_025() -> None:
    """A failed build shall emit no model, wiring or configuration, and its report shall list every failure found."""
    raise NotImplementedError


@pytest.mark.req("BLD-027")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_027() -> None:
    """The builder shall resolve every catalog part reference, combining the catalog part's parameters with the
    instance's overrides.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-028")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_028() -> None:
    """Every cross-domain binding shall resolve to an existing joint or mount face, and an unresolved binding shall end
    the build with a report.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_029() -> None:
    """The milestone 1 pendulum link shall be emitted as a rigid body on a revolute joint with no rope element."""
    raise NotImplementedError


@pytest.mark.req("BLD-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_031() -> None:
    """Every fastener and mount shall be emitted as an individually addressable constraint, so that a modeled failure
    can disable it.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-032")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_032() -> None:
    """A rope shall be emitted as a lumped chain in the neutral form."""
    raise NotImplementedError
