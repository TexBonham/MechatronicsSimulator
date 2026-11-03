"""Strict-xfail stubs for the SCO spec (unit tier)."""

import pytest


@pytest.mark.req("SCO-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_001() -> None:
    """Scoring shall evaluate each pass criterion of a test case against the run's recording and nothing
    else.
    """
    raise NotImplementedError


@pytest.mark.req("SCO-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_002() -> None:
    """Scoring shall report each criterion's measured value beside its limit."""
    raise NotImplementedError


@pytest.mark.req("SCO-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_003() -> None:
    """Scoring shall report the simulated time of each criterion's worst case."""
    raise NotImplementedError


@pytest.mark.req("SCO-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_004() -> None:
    """Scoring shall report pass or fail for each criterion."""
    raise NotImplementedError


@pytest.mark.req("SCO-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_005() -> None:
    """Scoring shall report a case as passed only when every one of its criteria passes."""
    raise NotImplementedError


@pytest.mark.req("SCO-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_006() -> None:
    """Scoring shall report a suite as passed only when every one of its cases passes."""
    raise NotImplementedError


@pytest.mark.req("SCO-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_007() -> None:
    """Scoring shall report a case whose run ended in a run error as errored, never as passed."""
    raise NotImplementedError


@pytest.mark.req("SCO-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_008() -> None:
    """Scoring shall aggregate every case of a suite into one pass/fail report."""
    raise NotImplementedError


@pytest.mark.req("SCO-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_009() -> None:
    """Scoring shall diff a suite's results against its regression baseline."""
    raise NotImplementedError


@pytest.mark.req("SCO-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_010() -> None:
    """The regression diff shall highlight every criterion and case that newly passes or newly fails."""
    raise NotImplementedError


@pytest.mark.req("SCO-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_011() -> None:
    """Scoring shall write each suite's scoring summary as YAML in the regression baseline format, so that
    it can be committed as the next baseline.
    """
    raise NotImplementedError


@pytest.mark.req("SCO-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_012() -> None:
    """Scoring shall produce the pass/fail report as a shareable document."""
    raise NotImplementedError


@pytest.mark.req("SCO-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_013() -> None:
    """The report shall state, for each case, the seed, the design version and the controller mode of the
    run it scored.
    """
    raise NotImplementedError


@pytest.mark.req("SCO-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_014() -> None:
    """Scoring shall read structural criteria (safety factor, deflection) from the integrity results, not
    recompute them.
    """
    raise NotImplementedError


@pytest.mark.req("SCO-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_015() -> None:
    """Scoring shall fail a case for which integrity recorded a tier 1 limit crossing."""
    raise NotImplementedError


@pytest.mark.req("SCO-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_016() -> None:
    """Scoring the same archive twice shall give an identical report."""
    raise NotImplementedError


@pytest.mark.req("SCO-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_017() -> None:
    """Scoring shall take every criterion, measure and limit from the test case file, with no design-specific criterion
    in code.
    """
    raise NotImplementedError


@pytest.mark.req("SCO-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_018() -> None:
    """Scoring shall report a sweep's criterion values per sweep value side by side."""
    raise NotImplementedError


@pytest.mark.req("SCO-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_019() -> None:
    """Scoring shall report the results of seeded variations per variation seed."""
    raise NotImplementedError


@pytest.mark.req("SCO-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_020() -> None:
    """Scoring shall support a criterion comparing the peak, minimum or final value of a recorded series
    with a limit.
    """
    raise NotImplementedError


@pytest.mark.req("SCO-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_021() -> None:
    """Scoring shall support a criterion comparing the simulated time between two recorded events with a
    limit.
    """
    raise NotImplementedError


@pytest.mark.req("SCO-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_022() -> None:
    """Scoring shall support a criterion requiring the presence or absence of a recorded event or operator
    message within a time window.
    """
    raise NotImplementedError


@pytest.mark.req("SCO-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_023() -> None:
    """Scoring shall support a criterion requiring no actuator motion within a time window, measured
    against a motion threshold.
    """
    raise NotImplementedError


@pytest.mark.req("SCO-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_024() -> None:
    """Scoring shall support a criterion comparing the settle time of a recorded series into a band with a
    limit.
    """
    raise NotImplementedError


@pytest.mark.req("SCO-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_025() -> None:
    """Scoring shall support a criterion requiring that no two recorded actuator commands are active at
    once on the same actuator.
    """
    raise NotImplementedError
