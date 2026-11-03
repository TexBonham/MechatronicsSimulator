"""SIG stubs, invariant tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("SIG-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_005() -> None:
    """Every random draw in the signal layer shall come from the drawing instance's child generator derived from the
    run seed.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_021() -> None:
    """The controller shall see an E-stop press, rail loss or watchdog trip only through its own pin inputs."""
    raise NotImplementedError


@pytest.mark.req("SIG-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_024() -> None:
    """Every value crossing between the signal layer and the controller runtime shall cross through a pin-binding
    table entry.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_031() -> None:
    """Adding a higher fidelity tier to a signal-layer component shall not change the results of its lower tier."""
    raise NotImplementedError
