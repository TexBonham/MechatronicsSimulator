"""KER stubs, unit tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("KER-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_016() -> None:
    """The kernel shall report constraint drift and solver failure from physics as run errors, never as recorded
    values.
    """
    raise NotImplementedError


@pytest.mark.req("KER-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_019() -> None:
    """A party implemented on the Python slow path shall register and step through the same party interface as a C++
    party.
    """
    raise NotImplementedError


@pytest.mark.req("KER-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_023() -> None:
    """The kernel shall deliver a scheduled operator command or injected fault to its owning party at the sync point
    given by the scheduling rule.
    """
    raise NotImplementedError
