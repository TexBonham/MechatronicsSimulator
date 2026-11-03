"""CTL stubs, invariant tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("CTL-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_014() -> None:
    """The controller runtime shall receive only pin inputs and power rail state, never true physical state, in
    either mode.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_017() -> None:
    """No Renode type shall cross the `core/controller/` runtime boundary."""
    raise NotImplementedError


@pytest.mark.req("CTL-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_022() -> None:
    """An emulated run shall be deterministic, with Renode's virtual time driven only by the kernel."""
    raise NotImplementedError
