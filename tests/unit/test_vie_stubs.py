"""Strict-xfail stubs for the VIE spec (unit tier). VIE-001 and VIE-012 are tested in
`tests/unit/test_web_client.py`."""

import pytest


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
