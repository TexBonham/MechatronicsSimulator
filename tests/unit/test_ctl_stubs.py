"""CTL stubs, unit tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("CTL-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_008() -> None:
    """The emulated runtime shall run the firmware's compiled .elf on Renode in lockstep: for each kernel tick, apply
    pin inputs, run the emulator for the tick's virtual time, then read pin outputs.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_009() -> None:
    """The emulated runtime shall wire the signal layer to the emulated peripherals: GPIO, timer encoder and PWM
    channels, ADC and USART.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_010() -> None:
    """Interrupt semantics shall exist only in emulated mode."""
    raise NotImplementedError


@pytest.mark.req("CTL-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_016() -> None:
    """The emulated runtime shall reach Renode through the transport ruled by the milestone 4 ADR, from the core
    runtime directly.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_019() -> None:
    """An emulator fault shall end the run with a report carrying the simulated time and the emulator's error."""
    raise NotImplementedError


@pytest.mark.req("CTL-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_023() -> None:
    """The runtime interface shall admit a further implementation, such as hardware in the loop, without a change to
    its callers.
    """
    raise NotImplementedError
