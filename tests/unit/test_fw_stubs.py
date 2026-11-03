"""FW stubs, unit tier (T3); real tests replace them with the firmware."""

import pytest


@pytest.mark.req("FW-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_001() -> None:
    """Firmware application code shall be written in C11."""
    raise NotImplementedError


@pytest.mark.req("FW-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_002() -> None:
    """C++ in a firmware tree shall appear only inside a module behind a C interface."""
    raise NotImplementedError


@pytest.mark.req("FW-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_003() -> None:
    """The board interface shall be a C header."""
    raise NotImplementedError


@pytest.mark.req("FW-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_004() -> None:
    """The board interface and firmware application code shall use fixed-width integer types."""
    raise NotImplementedError


@pytest.mark.req("FW-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_007() -> None:
    """The firmware build shall provide a host preset and an arm-none-eabi preset that build the same application
    sources.
    """
    raise NotImplementedError


@pytest.mark.req("FW-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_008() -> None:
    """The host and arm-none-eabi presets shall use identical warning flags, with warnings as errors."""
    raise NotImplementedError


@pytest.mark.req("FW-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_009() -> None:
    """The board interface header shall be owned by the simulator and live in `board_interface/`."""
    raise NotImplementedError


@pytest.mark.req("FW-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_010() -> None:
    """The board interface shall declare the firmware's init and tick entry points and the plain C input and output
    structs they exchange.
    """
    raise NotImplementedError


@pytest.mark.req("FW-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_011() -> None:
    """Firmware application code shall be correct under polled semantics in native mode, with no dependence on
    interrupt timing.
    """
    raise NotImplementedError
