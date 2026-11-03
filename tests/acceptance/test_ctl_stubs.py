"""CTL stubs, acceptance tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("CTL-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_015() -> None:
    """The same firmware run in native and emulated mode shall agree on the G1 to G4 pass criteria and on recorded
    pin outputs within the cross-mode tolerance.
    """
    raise NotImplementedError
