"""Acceptance tier stubs for the design spec (DES), strict xfail until converted."""

import pytest


@pytest.mark.req("DES-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_025() -> None:
    """The gantry shall ship as `designs/gantry/` with its test cases under `test_cases/gantry/` and shall build, run
    and score with the library as shipped and no code outside those folders and `firmware/`.
    """
    raise NotImplementedError


@pytest.mark.req("DES-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_026() -> None:
    """The pendulum shall ship as `designs/pendulum/` with a rigid link and shall build, run and score with the library
    as shipped and no design-specific code.
    """
    raise NotImplementedError
