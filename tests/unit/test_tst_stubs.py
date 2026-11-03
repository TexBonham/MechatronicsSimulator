"""Strict-xfail stubs for the TST spec (unit tier)."""

import pytest


@pytest.mark.req("TST-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_001() -> None:
    """A test case file shall name the design version it runs against, as the design's semver string."""
    raise NotImplementedError


@pytest.mark.req("TST-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_002() -> None:
    """A test case file shall define the initial state the run starts from."""
    raise NotImplementedError


@pytest.mark.req("TST-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_003() -> None:
    """A test case file shall define a timeline of operator commands, each with its scheduled simulated
    time.
    """
    raise NotImplementedError


@pytest.mark.req("TST-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_004() -> None:
    """A test case file shall define a timeline of injected faults, each with its fault type, target, start
    time and window.
    """
    raise NotImplementedError


@pytest.mark.req("TST-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_005() -> None:
    """A test case file shall define its pass criteria, each naming a measured value and a limit."""
    raise NotImplementedError


@pytest.mark.req("TST-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_006() -> None:
    """A test case file shall carry one integer random seed."""
    raise NotImplementedError


@pytest.mark.req("TST-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_007() -> None:
    """A test case or suite file shall be able to define parameter sweeps and seeded variations."""
    raise NotImplementedError


@pytest.mark.req("TST-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_008() -> None:
    """A suite file shall name the regression baseline it is compared against."""
    raise NotImplementedError


@pytest.mark.req("TST-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_009() -> None:
    """The test runner shall validate every test case file against its schema in `schemas/` before a run
    starts and reject an invalid file with an error naming the failing field.
    """
    raise NotImplementedError


@pytest.mark.req("TST-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_010() -> None:
    """The test runner shall parse quantities in test case files from unit strings into SI values at load,
    and never use unit objects inside the run.
    """
    raise NotImplementedError


@pytest.mark.req("TST-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_011() -> None:
    """The test runner shall load the design the test case names, have the model builder build it, and end
    with an error when the design file's version differs from the version the test case names.
    """
    raise NotImplementedError


@pytest.mark.req("TST-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_012() -> None:
    """The test runner shall set the test case's initial state before the kernel starts at t = 0."""
    raise NotImplementedError


@pytest.mark.req("TST-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_013() -> None:
    """The test runner shall deliver each operator command through the operator link of the signal layer at
    its scheduled simulated time, and by no other path to the controller.
    """
    raise NotImplementedError


@pytest.mark.req("TST-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_014() -> None:
    """The test runner shall route a sensor stuck, noisy or dropped fault to the signal layer, which sets
    that sensor model's failure mode for the fault window.
    """
    raise NotImplementedError


@pytest.mark.req("TST-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_015() -> None:
    """The test runner shall route a belt or rope slip fault to physics, which makes that transmission
    model slip for the fault window.
    """
    raise NotImplementedError


@pytest.mark.req("TST-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_016() -> None:
    """The test runner shall route a motor stall or missed-steps fault to the signal layer, which reduces the behavioral
    stepper's available torque for the fault window.
    """
    raise NotImplementedError


@pytest.mark.req("TST-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_017() -> None:
    """The test runner shall route a power loss and restore fault to the signal layer, which drops and
    restores the rail voltage; the controller runtime resets the controller only when the rail falls
    below the chip's brownout threshold.
    """
    raise NotImplementedError


@pytest.mark.req("TST-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_018() -> None:
    """The test runner shall route a controller freeze fault to the controller runtime, which stops calling
    the tick function (native) or halts the core (emulated) for the fault window.
    """
    raise NotImplementedError


@pytest.mark.req("TST-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_019() -> None:
    """The test runner shall route an E-stop press to the signal layer, which opens the relay on the motor
    rail; the test runner sends the controller nothing for it.
    """
    raise NotImplementedError


@pytest.mark.req("TST-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_020() -> None:
    """The test runner shall route a payload change to physics, which changes the payload mass or
    attachment at the scheduled time.
    """
    raise NotImplementedError


@pytest.mark.req("TST-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_021() -> None:
    """The test runner shall run the kernel until the test case's end condition is met."""
    raise NotImplementedError


@pytest.mark.req("TST-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_022() -> None:
    """The test runner shall end the run on a solver failure, constraint drift beyond limit or emulator
    fault, and report the error with its simulated time; such a run is never reported as passed.
    """
    raise NotImplementedError


@pytest.mark.req("TST-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_023() -> None:
    """The test runner shall hand every run to scoring with the reason it ended, including runs ended by a solver error
    or a fault.
    """
    raise NotImplementedError


@pytest.mark.req("TST-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_024() -> None:
    """The test runner shall run every case of a suite in one batch invocation."""
    raise NotImplementedError


@pytest.mark.req("TST-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_025() -> None:
    """The test runner shall sweep any design or test case parameter over the values a sweep definition
    lists, one run per value, each run recorded with the value it used.
    """
    raise NotImplementedError


@pytest.mark.req("TST-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_026() -> None:
    """The test runner shall run seeded randomized variations of a case, each variation recorded with its
    own integer seed so that it can be rerun alone with an identical result.
    """
    raise NotImplementedError


@pytest.mark.req("TST-027")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_027() -> None:
    """The test runner shall rerun a suite and pass its scoring summary with the suite's regression
    baseline to scoring for the regression diff.
    """
    raise NotImplementedError


@pytest.mark.req("TST-028")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_028() -> None:
    """The test runner shall seed each run from the one integer seed, spawning one child generator per
    module instance from a SeedSequence in a fixed order.
    """
    raise NotImplementedError


@pytest.mark.req("TST-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_029() -> None:
    """The test runner shall take every random draw it makes from its own seeded child generator and from
    no other source.
    """
    raise NotImplementedError


@pytest.mark.req("TST-030")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_030() -> None:
    """The CLI command `validate` shall validate design, test case and suite files against their schemas
    without running anything.
    """
    raise NotImplementedError


@pytest.mark.req("TST-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_031() -> None:
    """The CLI command `run` shall run one test case and write its run archive."""
    raise NotImplementedError


@pytest.mark.req("TST-032")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_032() -> None:
    """The CLI command `suite` shall run a suite in batch and write the suite's pass/fail report."""
    raise NotImplementedError


@pytest.mark.req("TST-033")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_033() -> None:
    """The CLI command `replay` shall replay a run from its archive alone, without the engine that produced
    it.
    """
    raise NotImplementedError


@pytest.mark.req("TST-034")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_034() -> None:
    """The CLI shall drive the simulator only through the facade and hold no domain logic of its own."""
    raise NotImplementedError


@pytest.mark.req("TST-035")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_035() -> None:
    """The CLI shall run with no display, so that every run CI performs is a headless CLI run."""
    raise NotImplementedError


@pytest.mark.req("TST-036")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_036() -> None:
    """The test runner shall take the controller mode (native or emulated) as a run input and pass it to
    the kernel and controller runtime.
    """
    raise NotImplementedError


@pytest.mark.req("TST-037")
@pytest.mark.xfail(strict=True, reason="stub")
def test_tst_037() -> None:
    """The fault timeline shall be able to inject a forced-level fault on a digital sensor, which the test runner routes
    to the signal layer for the fault window.
    """
    raise NotImplementedError
