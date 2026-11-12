"""Strict-xfail stubs for the TST spec (unit tier)."""

import pytest


@pytest.mark.req("TST-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_029() -> None:
    """The test runner shall take every random draw it makes from its own seeded child generator and from
    no other source.
    """
    raise NotImplementedError
