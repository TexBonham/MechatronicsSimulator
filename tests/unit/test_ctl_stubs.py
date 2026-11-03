"""CTL stubs, unit tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("CTL-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_001() -> None:
    """The controller runtime shall expose one runtime interface with a native and an emulated implementation."""
    raise NotImplementedError


@pytest.mark.req("CTL-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_002() -> None:
    """The native runtime shall call the firmware's init entry point once after each reset and its tick entry point
    at the configured fixed control rate.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_003() -> None:
    """The native runtime shall pass inputs to the tick entry point and read outputs from it as a plain C struct."""
    raise NotImplementedError


@pytest.mark.req("CTL-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_004() -> None:
    """The native runtime shall present inputs with polled semantics: counters as counts accumulated over the tick,
    levels as latched values, and no interrupts.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_005() -> None:
    """The native shim shall call the firmware entry points directly from the core runtime, with no cffi or other
    foreign-function layer.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_006() -> None:
    """The native shim shall read the pin-binding table and present each bound signal as the board interface's value
    for its pin.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_007() -> None:
    """The native runtime shall build the firmware source for the host before the run starts."""
    raise NotImplementedError


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


@pytest.mark.req("CTL-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_011() -> None:
    """A freeze operation shall stop the controller in both modes: in native mode the tick entry point is not called,
    in emulated mode the core is halted.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_012() -> None:
    """A reset operation shall restart the firmware from its reset state in both modes, with the power rail up at run
    start.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_013() -> None:
    """When rail voltage falls below the chip's brownout threshold, the runtime shall hold the controller in reset
    until the voltage recovers, in both modes.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_016() -> None:
    """The emulated runtime shall reach Renode through the transport ruled by the milestone 4 ADR, from the core
    runtime directly.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_018() -> None:
    """The runtime shall publish the controller's state and fault log each tick: in native mode through the shim, in
    emulated mode through a debug channel.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_019() -> None:
    """An emulator fault shall end the run with a report carrying the simulated time and the emulator's error."""
    raise NotImplementedError


@pytest.mark.req("CTL-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_020() -> None:
    """The runtime shall delay the controller's pin outputs by a configurable input-to-output latency."""
    raise NotImplementedError


@pytest.mark.req("CTL-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_021() -> None:
    """The runtime shall slow the controller on demand for a scheduled window by the slow-down factor in the run
    data.
    """
    raise NotImplementedError


@pytest.mark.req("CTL-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ctl_023() -> None:
    """The runtime interface shall admit a further implementation, such as hardware in the loop, without a change to
    its callers.
    """
    raise NotImplementedError
