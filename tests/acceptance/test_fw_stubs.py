"""FW acceptance tests: end-to-end halves of the gantry firmware rows on the real stack.

FW-017 to FW-021, FW-024, FW-026 to FW-028 are proven on the host plant (firmware/tests/test_gantry*.cpp); their
end-to-end halves cite the ACC tests (tests/acceptance/test_acc_stubs.py). FW-025 is proven here on G8 alone.
"""

from typing import Any

import numpy as np
import pytest

from mechsim.testing.core_driver import CoreDriver, core_services
from mechsim.testing.runner import Services, run_case
from mechsim.testing.schema import ROOT


def _toggles(series: Any, start: float, end: float) -> list[float]:
    times = np.asarray(series.times)
    values = np.asarray(series.values)
    changes = np.nonzero(np.diff(values))[0] + 1
    return [float(times[i]) for i in changes if start <= times[i] < end]


@pytest.mark.req("FW-025")
def test_fw_025() -> None:
    """Reference design (gantry): the firmware shall send the "heartbeat" to the E-stop relay while running, so that
    a freeze lets the watchdog cut motor power.

    G8: the heartbeat output toggles every 10 ticks (placeholder period, ruling 6) while the firmware runs, so the
    second before the freeze at 9 s holds 1 s / 10 ms = 100 toggles (one either way for the phase); during the 1 s
    freeze it toggles never; the watchdog (50 ms timeout, 10 ms cutoff delay) then drops the motor rail within
    50 + 10 + 10 ms (one period of slack for the last edge) of the freeze and not before it.
    """
    driver = CoreDriver()
    result = run_case(
        ROOT / "test_cases" / "gantry" / "G8.yaml",
        Services(lambda: driver, core_services().builder),
        None,
        None,
    )
    assert result.report is not None
    assert not result.report.errored, result.report.error
    heartbeat = driver.recording().series("signal:watchdog.heartbeat")
    before = _toggles(heartbeat, 8.0, 9.0)
    during = _toggles(heartbeat, 9.001, 10.0)
    print(f"G8 heartbeat toggles: {len(before)} in the second before the freeze, {len(during)} during it")
    assert 99 <= len(before) <= 101
    assert during == []
    rail = driver.recording().series("signal:motor_power.on")
    off = [time for time, value in zip(rail.times, rail.values, strict=True) if value == 0.0 and time >= 9.0]
    print(f"G8 motor rail off at {off[0]} s (bound 9.0 s to 9.07 s)")
    assert 9.0 < off[0] <= 9.07
