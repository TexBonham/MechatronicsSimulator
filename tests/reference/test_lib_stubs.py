"""Reference tier stubs for the component library spec (LIB), strict xfail until converted."""

import pytest


@pytest.mark.req("LIB-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_022() -> None:
    """The behavioral stepper type shall reproduce the pull-out torque of its catalog part against the published
    torque-speed curve.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-032")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_032() -> None:
    """The flexible beam type shall reproduce cantilever and simply supported beam deflection and first natural
    frequency against their closed-form results.
    """
    raise NotImplementedError
