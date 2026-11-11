"""Strict-xfail stubs for the VIE spec (unit tier)."""

import pytest


@pytest.mark.req("VIE-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_vie_001() -> None:
    """The viewer shall replay a run in 3D from its run archive."""
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
