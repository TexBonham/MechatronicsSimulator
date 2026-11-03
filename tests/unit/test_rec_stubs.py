"""Strict-xfail stubs for the REC spec (unit tier)."""

import pytest


@pytest.mark.req("REC-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_001() -> None:
    """The recorder shall be the single sink of run data: integrity, scoring, the viewer and exports read the recording
    and never live run state, except the tier 1 integrity party, which reads live member loads through the kernel.
    """
    raise NotImplementedError


@pytest.mark.req("REC-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_002() -> None:
    """The recorder shall record every body's pose and velocity on every physics tick."""
    raise NotImplementedError


@pytest.mark.req("REC-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_003() -> None:
    """The recorder shall record every joint force and contact force on every physics tick."""
    raise NotImplementedError


@pytest.mark.req("REC-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_004() -> None:
    """The recorder shall record every structural member's end-loads on every physics tick."""
    raise NotImplementedError


@pytest.mark.req("REC-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_005() -> None:
    """The recorder shall record every signal on every pin, in both directions between the signal layer and
    the controller.
    """
    raise NotImplementedError


@pytest.mark.req("REC-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_006() -> None:
    """The recorder shall record the state of every power rail."""
    raise NotImplementedError


@pytest.mark.req("REC-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_007() -> None:
    """The recorder shall record the controller states and the controller fault log the controller runtime
    reports.
    """
    raise NotImplementedError


@pytest.mark.req("REC-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_008() -> None:
    """The recorder shall record every injected fault and every safety event with its simulated time."""
    raise NotImplementedError


@pytest.mark.req("REC-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_009() -> None:
    """The recorder shall record every operator command and status message with its simulated time, using
    the message labels verbatim.
    """
    raise NotImplementedError


@pytest.mark.req("REC-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_010() -> None:
    """The recorder shall record every in-loop integrity event with its simulated time."""
    raise NotImplementedError


@pytest.mark.req("REC-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_011() -> None:
    """The recorder shall stamp every recorded sample and event with the simulated time."""
    raise NotImplementedError


@pytest.mark.req("REC-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_012() -> None:
    """The recorder shall write one HDF5 archive per run through h5py."""
    raise NotImplementedError


@pytest.mark.req("REC-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_013() -> None:
    """The run archive shall carry the design file, the test case, the seed, the controller mode and the
    git hashes as attributes.
    """
    raise NotImplementedError


@pytest.mark.req("REC-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_014() -> None:
    """The run archive shall carry the design version as the semver string plus the design file's git blob
    hash.
    """
    raise NotImplementedError


@pytest.mark.req("REC-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_015() -> None:
    """The recorder shall export any recorded series as CSV."""
    raise NotImplementedError


@pytest.mark.req("REC-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_016() -> None:
    """The recorder shall export any recorded series as JSON."""
    raise NotImplementedError


@pytest.mark.req("REC-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_017() -> None:
    """The recorder shall export any recorded series as a MAT file."""
    raise NotImplementedError


@pytest.mark.req("REC-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_018() -> None:
    """Every archive series and export shall hold SI values and state the unit of each quantity explicitly."""
    raise NotImplementedError


@pytest.mark.req("REC-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_019() -> None:
    """The recorder shall expose a streaming interface through which a consumer receives recorded ticks
    during a run.
    """
    raise NotImplementedError


@pytest.mark.req("REC-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_020() -> None:
    """A streaming consumer shall not change the recording or the run it observes."""
    raise NotImplementedError


@pytest.mark.req("REC-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_023() -> None:
    """The recorder shall write no wall-clock time or other nondeterministic value into the recorded series
    or attributes.
    """
    raise NotImplementedError


@pytest.mark.req("REC-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_024() -> None:
    """A run archive shall hold everything replay needs, so that a run replays without the engine that
    produced it.
    """
    raise NotImplementedError


@pytest.mark.req("REC-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_025() -> None:
    """The run archive shall record how the run ended: end condition reached, or the run error report with
    its simulated time.
    """
    raise NotImplementedError


@pytest.mark.req("REC-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_026() -> None:
    """The recorder shall record every rope tension on every physics tick."""
    raise NotImplementedError


@pytest.mark.req("REC-027")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_027() -> None:
    """The recorder shall record the actuator torques and forces the signal layer applies to physics on
    every tick.
    """
    raise NotImplementedError
