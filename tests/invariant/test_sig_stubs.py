"""SIG stubs, invariant tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("SIG-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_031() -> None:
    """Adding a higher fidelity tier to a signal-layer component shall not change the results of its lower tier."""
    raise NotImplementedError
