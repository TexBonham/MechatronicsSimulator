"""Strict-xfail stubs for the VIE spec (unit tier)."""

import pytest


@pytest.mark.req("VIE-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_001() -> None:
    """The viewer shall replay a run in 3D from its run archive."""
    raise NotImplementedError


@pytest.mark.req("VIE-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_002() -> None:
    """The replay shall show an event timeline marking every operator command, injected fault, safety event
    and criterion failure at its simulated time.
    """
    raise NotImplementedError


@pytest.mark.req("VIE-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_003() -> None:
    """The viewer shall offer a probe on any body that shows its recorded values over time."""
    raise NotImplementedError


@pytest.mark.req("VIE-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_004() -> None:
    """The viewer shall offer a probe on any joint that shows its recorded values over time."""
    raise NotImplementedError


@pytest.mark.req("VIE-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_005() -> None:
    """The viewer shall offer a probe on any pin that shows its recorded values over time."""
    raise NotImplementedError


@pytest.mark.req("VIE-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_006() -> None:
    """The viewer shall offer a probe on any signal that shows its recorded values over time."""
    raise NotImplementedError


@pytest.mark.req("VIE-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_007() -> None:
    """The replay shall scrub to any recorded simulated time."""
    raise NotImplementedError


@pytest.mark.req("VIE-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_008() -> None:
    """The replay shall pause and resume."""
    raise NotImplementedError


@pytest.mark.req("VIE-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_009() -> None:
    """The replay shall step forward and backward one recorded sample at a time."""
    raise NotImplementedError


@pytest.mark.req("VIE-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_010() -> None:
    """The replay shall overlay member stress on the structural members."""
    raise NotImplementedError


@pytest.mark.req("VIE-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_011() -> None:
    """The replay shall overlay the swing angle of suspended loads."""
    raise NotImplementedError


@pytest.mark.req("VIE-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_012() -> None:
    """The early front end shall be a browser client in `web/` over the local Python facade service."""
    raise NotImplementedError


@pytest.mark.req("VIE-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_013() -> None:
    """A standalone app replacing the browser client shall use the same facade protocol and pass the same
    conformance suite.
    """
    raise NotImplementedError


@pytest.mark.req("VIE-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_014() -> None:
    """Overlays, probe values and timeline events shall be computed in `mechsim/viewer/` and served through
    the facade, never in a client.
    """
    raise NotImplementedError


@pytest.mark.req("VIE-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_015() -> None:
    """The viewer shall take body geometry from the run's design file through the facade."""
    raise NotImplementedError


@pytest.mark.req("VIE-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_016() -> None:
    """Replay shall work from any run archive without the engine that produced it."""
    raise NotImplementedError


@pytest.mark.req("VIE-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_017() -> None:
    """The viewer shall produce on-screen output only and write no files."""
    raise NotImplementedError


@pytest.mark.req("VIE-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_018() -> None:
    """A GUI editor, when it exists, shall write the same design and test case files through the facade."""
    raise NotImplementedError


@pytest.mark.req("VIE-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_019() -> None:
    """GUI editor forms shall be generated from the YAML schemas."""
    raise NotImplementedError
