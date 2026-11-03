"""FW stubs, invariant tier (T3); real tests replace them with the firmware."""

import pytest


@pytest.mark.req("FW-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_005() -> None:
    """Firmware application code shall include the board interface header and nothing else from the simulator or any
    vendor HAL.
    """
    raise NotImplementedError


@pytest.mark.req("FW-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_006() -> None:
    """The host build preset shall fail on any include in application code outside the board interface."""
    raise NotImplementedError


@pytest.mark.req("FW-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_012() -> None:
    """Vendor HAL code shall appear only under `firmware/board/<chip>/`."""
    raise NotImplementedError
