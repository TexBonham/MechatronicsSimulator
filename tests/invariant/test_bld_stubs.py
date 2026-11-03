"""Invariant tier stubs for the builder spec (BLD), strict xfail until converted."""

import pytest


@pytest.mark.req("BLD-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_026() -> None:
    """Building the same design with the same inputs twice shall produce an identical neutral model, wiring, chip
    configuration and member list.
    """
    raise NotImplementedError
