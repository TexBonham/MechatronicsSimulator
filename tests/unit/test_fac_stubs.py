"""Strict-xfail stubs for the FAC spec (unit tier)."""

import pytest


@pytest.mark.req("FAC-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_013() -> None:
    """The facade shall expose integrity results and the load-case export."""
    raise NotImplementedError
