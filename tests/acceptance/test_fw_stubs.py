"""FW stubs, acceptance tier (T3); real tests replace them with the firmware.

FW-017 to FW-021, FW-024, FW-026 to FW-028 are proven on the host plant (firmware/tests/test_gantry*.cpp); their
end-to-end halves are the ACC stubs. FW-025 stays: its watchdog half needs the SIG relay end to end.
"""

import pytest


@pytest.mark.req("FW-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_025() -> None:
    """Reference design (gantry): the firmware shall send the "heartbeat" to the E-stop relay while running, so that
    a freeze lets the watchdog cut motor power.
    """
    raise NotImplementedError