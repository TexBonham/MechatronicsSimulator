"""SIG stubs, unit tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("SIG-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_002() -> None:
    """A sensor model shall add noise with the distribution and magnitude given by its noise parameters."""
    raise NotImplementedError


@pytest.mark.req("SIG-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_003() -> None:
    """A sensor model shall delay its signal by its delay parameter."""
    raise NotImplementedError


@pytest.mark.req("SIG-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_004() -> None:
    """A sensor model shall update its signal at its sample-rate parameter and hold it between samples."""
    raise NotImplementedError


@pytest.mark.req("SIG-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_014() -> None:
    """An amplifier front end shall turn its sensor's output into an ADC sample with its gain and resolution
    parameters.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_019() -> None:
    """A power-loss fault shall drop the rail voltage for the fault window and restore it after."""
    raise NotImplementedError


@pytest.mark.req("SIG-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_020() -> None:
    """A brake component shall apply its brake-torque parameter to its joint while engaged."""
    raise NotImplementedError


@pytest.mark.req("SIG-035")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_035() -> None:
    """The stepper component shall reserve a phase-current fidelity tier slot, which v1 accepts in data and does not
    evaluate.
    """
    raise NotImplementedError
