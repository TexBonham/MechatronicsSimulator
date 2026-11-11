"""Strict-xfail stubs for the SCO spec (unit tier)."""

import pytest


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


@pytest.mark.req("SCO-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sco_019() -> None:
    """Scoring shall report the results of seeded variations per variation seed."""
    raise NotImplementedError
