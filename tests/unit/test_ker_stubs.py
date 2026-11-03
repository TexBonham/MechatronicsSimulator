"""KER stubs, unit tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("KER-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_001() -> None:
    """The kernel shall own simulated time; no party shall advance simulated time except through a kernel step."""
    raise NotImplementedError


@pytest.mark.req("KER-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_002() -> None:
    """The kernel shall start every run at simulated time t = 0."""
    raise NotImplementedError


@pytest.mark.req("KER-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_003() -> None:
    """The kernel shall step each registered party at that party's own rate, including signal parties at rates finer
    than the physics step and an emulator advanced in chunks of virtual time.
    """
    raise NotImplementedError


@pytest.mark.req("KER-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_004() -> None:
    """The kernel shall take the physics step from the run configuration, defaulting to 1 ms, with no rate held as a
    code constant.
    """
    raise NotImplementedError


@pytest.mark.req("KER-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_005() -> None:
    """The kernel shall take the control tick rate from the run configuration, defaulting to 1 kHz, with no rate held
    as a code constant.
    """
    raise NotImplementedError


@pytest.mark.req("KER-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_006() -> None:
    """The kernel shall support a control tick rate unequal to the physics rate."""
    raise NotImplementedError


@pytest.mark.req("KER-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_007() -> None:
    """The kernel shall reject, before t = 0, a rate combination that the rate ratio rule does not allow, with an
    error naming the parties.
    """
    raise NotImplementedError


@pytest.mark.req("KER-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_008() -> None:
    """The kernel shall exchange coupling variables between parties only at sync points."""
    raise NotImplementedError


@pytest.mark.req("KER-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_009() -> None:
    """Within each physics tick the kernel shall run, in this order: physics advances one step and publishes poses,
    velocities, forces and member end-loads; the tier 1 integrity party evaluates the published end-loads; sensor models
    sample the new state; the controller runtime advances by the same interval; the signal layer collects pin outputs;
    driver and stepper models turn them into motor shaft torque gated by the power rail, which the transmissions turn
    into joint force; the tick's recorder row is written last.
    """
    raise NotImplementedError


@pytest.mark.req("KER-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_010() -> None:
    """Actuator forces computed in tick n shall be applied by physics in step n + 1."""
    raise NotImplementedError


@pytest.mark.req("KER-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_011() -> None:
    """Between sync points the kernel shall hold every coupling variable a party consumes at its value from the last
    sync point (zero-order hold).
    """
    raise NotImplementedError


@pytest.mark.req("KER-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_012() -> None:
    """The kernel shall accept one integer seed per run and derive one child generator per party instance from it, in
    a fixed instance order.
    """
    raise NotImplementedError


@pytest.mark.req("KER-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_013() -> None:
    """The kernel shall step parties within a sync point in a fixed order taken from the model data, independent of
    memory addresses, hashing and thread timing.
    """
    raise NotImplementedError


@pytest.mark.req("KER-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_015() -> None:
    """The kernel shall end the run with a report when any party raises a fault, naming the party, the simulated time
    and the cause.
    """
    raise NotImplementedError


@pytest.mark.req("KER-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_016() -> None:
    """The kernel shall report constraint drift and solver failure from physics as run errors, never as recorded
    values.
    """
    raise NotImplementedError


@pytest.mark.req("KER-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_017() -> None:
    """The kernel shall end the run when the test case's end condition is met or a run error occurs, and shall report
    which.
    """
    raise NotImplementedError


@pytest.mark.req("KER-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_019() -> None:
    """A party implemented on the Python slow path shall register and step through the same party interface as a C++
    party.
    """
    raise NotImplementedError


@pytest.mark.req("KER-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_021() -> None:
    """While the controller runtime is frozen, the kernel shall keep stepping physics and the signal layer."""
    raise NotImplementedError


@pytest.mark.req("KER-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_022() -> None:
    """The kernel shall stream every tick's published coupling variables and signals to a recorder sink interface,
    present from milestone 1.
    """
    raise NotImplementedError


@pytest.mark.req("KER-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_023() -> None:
    """The kernel shall deliver a scheduled operator command or injected fault to its owning party at the sync point
    given by the scheduling rule.
    """
    raise NotImplementedError


@pytest.mark.req("KER-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_024() -> None:
    """The kernel shall advance simulated time independently of wall-clock time."""
    raise NotImplementedError


@pytest.mark.req("KER-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_ker_025() -> None:
    """A party added later, such as a thermal party, shall register with the kernel without a change to kernel code."""
    raise NotImplementedError
