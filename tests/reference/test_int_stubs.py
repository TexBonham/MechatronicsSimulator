"""Strict-xfail stubs for the INT spec (reference tier)."""

import pytest


@pytest.mark.req("INT-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_013() -> None:
    """The bearing life check shall compute each bearing's rated life from its recorded loads."""
    raise NotImplementedError
