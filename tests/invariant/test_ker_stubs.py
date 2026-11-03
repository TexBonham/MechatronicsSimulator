"""KER stubs, invariant tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("KER-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_014() -> None:
    """Two runs with the same design, test case, seed and controller mode shall produce identical recordings."""
    raise NotImplementedError


@pytest.mark.req("KER-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_020() -> None:
    """The kernel shall route physics outputs only to the signal layer and other non-controller parties; the
    controller runtime party shall consume only pin inputs and power rail state.
    """
    raise NotImplementedError


@pytest.mark.req("KER-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_026() -> None:
    """The kernel shall carry no Qt or display dependency."""
    raise NotImplementedError
