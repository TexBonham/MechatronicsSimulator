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
