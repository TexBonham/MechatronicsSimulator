"""SIG stubs, unit tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("SIG-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_001() -> None:
    """A sensor model shall turn the physical state published by physics into a signal quantized to the sensor's
    resolution parameter.
    """
    raise NotImplementedError


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


@pytest.mark.req("SIG-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_006() -> None:
    """A sensor in the stuck failure mode shall hold its last value for the fault window."""
    raise NotImplementedError


@pytest.mark.req("SIG-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_007() -> None:
    """A sensor in the noisy failure mode shall apply the fault's noise for the fault window."""
    raise NotImplementedError


@pytest.mark.req("SIG-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_008() -> None:
    """A sensor in the dropped failure mode shall deliver no new samples for the fault window."""
    raise NotImplementedError


@pytest.mark.req("SIG-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_009() -> None:
    """A limit switch model shall present a level that changes when its actuator crosses the switch position."""
    raise NotImplementedError


@pytest.mark.req("SIG-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_010() -> None:
    """A debounce front end shall suppress level changes shorter than its debounce time parameter."""
    raise NotImplementedError


@pytest.mark.req("SIG-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_012() -> None:
    """A step/dir driver model shall count the step pulses of each tick and latch the direction and enable levels."""
    raise NotImplementedError


@pytest.mark.req("SIG-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_013() -> None:
    """The driver model shall hand each tick's pulse count, direction and enable to the stepper model it drives, whose
    motor shaft torque the transmission (ENG) turns into joint force for the next physics step.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_014() -> None:
    """An amplifier front end shall turn its sensor's output into an ADC sample with its gain and resolution
    parameters.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_015() -> None:
    """A power rail model shall carry its supply voltage to every component bound to it."""
    raise NotImplementedError


@pytest.mark.req("SIG-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_016() -> None:
    """When a motor rail is off, no actuator torque or force from components on that rail shall reach physics."""
    raise NotImplementedError


@pytest.mark.req("SIG-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_017() -> None:
    """The E-stop relay shall open the motor rail within its cutoff delay parameter of an E-stop press, with no
    action by the controller.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_018() -> None:
    """The watchdog shall open the motor rail when its heartbeat input shows no heartbeat for longer than its
    heartbeat timeout parameter, with no action by the controller.
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


@pytest.mark.req("SIG-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_022() -> None:
    """The signal layer shall pass rail voltage to the controller runtime so that it can apply the chip's brownout
    threshold.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_023() -> None:
    """The pin-binding table shall map each dotted signal name (instance.signal), unique per design, to one MCU pin
    and peripheral mode.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_025() -> None:
    """The operator link codec and its transport shall be separate classes, with UART as the v1 transport."""
    raise NotImplementedError


@pytest.mark.req("SIG-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_026() -> None:
    """The operator link shall encode scripted operator messages into UART bytes on the controller's receive pin and
    parse the controller's transmitted bytes back into named status messages.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-027")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_027() -> None:
    """The operator link's message set shall come from design data, using the requirements doc labels verbatim, so
    that the reference design's "target (x, y)", "home" and "current position, ready / fault" messages need no
    code.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-028")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_028() -> None:
    """The signal layer shall publish every signal on every pin and every rail state to the recorder stream each
    tick.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_029() -> None:
    """The built-in component set shall include, by milestone 1, the limit switch, encoder, step/dir driver, UART
    operator link and power rail with E-stop relay.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-032")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_032() -> None:
    """The behavioral stepper model (tier 1) shall produce motor shaft torque each tick from the driver's pulse count,
    direction and enable, using its torque-speed curve and the state of its motor rail.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-033")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_033() -> None:
    """When the torque the stepper model demands exceeds the torque available at the current shaft speed, given the load
    torque physics publishes for the shaft joint, the stepper model shall stall or miss steps and report the stall or
    the missed steps.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-034")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_034() -> None:
    """A motor stall or missed-steps fault shall, for the fault window, reduce the stepper model's available torque or
    raise its demand above the available torque.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-035")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_035() -> None:
    """The stepper component shall reserve a phase-current fidelity tier slot, which v1 accepts in data and does not
    evaluate.
    """
    raise NotImplementedError


@pytest.mark.req("SIG-036")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_036() -> None:
    """A digital sensor in the forced-level failure mode shall present the level the fault names for the fault window,
    whatever its physical input, generalizing the stuck mode (SIG-006) to a chosen level.
    """
    raise NotImplementedError
