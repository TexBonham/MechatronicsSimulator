"""Strict-xfail stubs for the TST spec (unit tier)."""

import pytest


@pytest.mark.req("TST-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_026() -> None:
    """The test runner shall run seeded randomized variations of a case, each variation recorded with its
    own integer seed so that it can be rerun alone with an identical result.
    """
    raise NotImplementedError


@pytest.mark.req("TST-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_029() -> None:
    """The test runner shall take every random draw it makes from its own seeded child generator and from
    no other source.
    """
    raise NotImplementedError


@pytest.mark.req("TST-034")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_034() -> None:
    """The CLI shall drive the simulator only through the facade and hold no domain logic of its own."""
    raise NotImplementedError
