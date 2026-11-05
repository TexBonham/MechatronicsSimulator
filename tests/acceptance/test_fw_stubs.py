"""FW stubs, acceptance tier (T3); real tests replace them with the firmware."""

import pytest


@pytest.mark.req("FW-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_017() -> None:
    """Reference design (gantry): after any power loss the firmware shall return to "not homed" and allow no motion
    until homed.
    """
    raise NotImplementedError


@pytest.mark.req("FW-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_018() -> None:
    """Reference design (gantry): with a load over the limit attached before a move, the firmware shall ignore lift
    commands and execute lower commands.
    """
    raise NotImplementedError


@pytest.mark.req("FW-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_019() -> None:
    """Reference design (gantry): when the load becomes overweight mid-lift, the firmware shall stop lifting and
    still allow lowering.
    """
    raise NotImplementedError


@pytest.mark.req("FW-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_020() -> None:
    """Reference design (gantry): when encoder position disagrees with the commanded position, the firmware shall
    stop, fault and require re-homing.
    """
    raise NotImplementedError


@pytest.mark.req("FW-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_021() -> None:
    """Reference design (gantry): when a limit switch trips where it should not, the firmware shall stop, fault and
    require re-homing.
    """
    raise NotImplementedError


@pytest.mark.req("FW-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_024() -> None:
    """Reference design (gantry): after an E-stop, the firmware shall stay stopped until a manual reset and a new
    command, with no automatic resume.
    """
    raise NotImplementedError


@pytest.mark.req("FW-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_025() -> None:
    """Reference design (gantry): the firmware shall send the "heartbeat" to the E-stop relay while running, so that
    a freeze lets the watchdog cut motor power.
    """
    raise NotImplementedError


@pytest.mark.req("FW-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_026() -> None:
    """Reference design (gantry): a new target during a move shall be handled by the one policy set as a firmware
    parameter (cancel and replace, or queue), identically every run.
    """
    raise NotImplementedError


@pytest.mark.req("FW-027")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_027() -> None:
    """Reference design (gantry): a lift command while the gantry is moving shall be handled by the one policy set as
    a firmware parameter (queue or reject), identically every run.
    """
    raise NotImplementedError


@pytest.mark.req("FW-028")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fw_028() -> None:
    """Reference design (gantry): rapid-fire and conflicting commands shall never produce overlapping actuator
    commands.
    """
    raise NotImplementedError
