"""Strict-xfail stubs for the INT spec (unit tier)."""

import pytest


@pytest.mark.req("INT-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_014() -> None:
    """The support spacing design rule shall flag a member whose supports are farther apart than the rule's
    limit.
    """
    raise NotImplementedError


@pytest.mark.req("INT-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_015() -> None:
    """The cantilever ratio design rule shall flag a cantilevered member whose length-to-section ratio
    exceeds the rule's limit.
    """
    raise NotImplementedError


@pytest.mark.req("INT-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_016() -> None:
    """The clearance design rule shall flag two bodies whose clearance anywhere in their joint range is
    below the rule's limit.
    """
    raise NotImplementedError


@pytest.mark.req("INT-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_017() -> None:
    """The misalignment binding design rule shall flag parallel guides or shafts whose misalignment exceeds
    the rule's limit.
    """
    raise NotImplementedError


@pytest.mark.req("INT-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_018() -> None:
    """The tip-over design rule shall flag a design whose combined center of mass can leave its support
    footprint under the design load.
    """
    raise NotImplementedError


@pytest.mark.req("INT-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_023() -> None:
    """A tier 1 kernel party shall evaluate member stress, buckling margin, fastener load and deflection
    every tick from the live loads physics publishes.
    """
    raise NotImplementedError


@pytest.mark.req("INT-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_024() -> None:
    """When a tier 1 quantity crosses its limit, integrity shall raise an event stamped with the simulated
    time of the crossing.
    """
    raise NotImplementedError


@pytest.mark.req("INT-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_025() -> None:
    """A tier 1 limit crossing shall fail the case."""
    raise NotImplementedError


@pytest.mark.req("INT-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_026() -> None:
    """A tier 1 limit crossing shall halt the run when halting is configured, and let the run continue
    otherwise.
    """
    raise NotImplementedError


@pytest.mark.req("INT-027")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_027() -> None:
    """Tier 1 events shall be deterministic: the same design, test case, seed and mode raise the same
    events at the same simulated times.
    """
    raise NotImplementedError


@pytest.mark.req("INT-028")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_028() -> None:
    """A tier 2 yield shall set a permanent spring offset in the member's model for the rest of the run."""
    raise NotImplementedError


@pytest.mark.req("INT-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_029() -> None:
    """A tier 2 buckling failure shall collapse the member's stiffness for the rest of the run."""
    raise NotImplementedError


@pytest.mark.req("INT-030")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_030() -> None:
    """A tier 2 fastener break shall disable that fastener's constraint for the rest of the run."""
    raise NotImplementedError


@pytest.mark.req("INT-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_031() -> None:
    """A tier 2 rope snap shall drop the load the rope carries."""
    raise NotImplementedError


@pytest.mark.req("INT-032")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_032() -> None:
    """Integrity shall export, per test case, the peak force and torque at each joint and mount as a load-case file
    giving location, direction and magnitude.
    """
    raise NotImplementedError


@pytest.mark.req("INT-033")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_033() -> None:
    """Every load-case export shall record the design version it belongs to."""
    raise NotImplementedError


@pytest.mark.req("INT-034")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_034() -> None:
    """Integrity shall import FEA results (stiffness, deflection, natural frequencies) and write the calibrated
    deflection and first natural frequency of each matching flexible member into the design file, with the design
    version recorded, for the builder to use."""
    raise NotImplementedError


@pytest.mark.req("INT-035")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_035() -> None:
    """An FEA import shall record the design version it belongs to and end with an error when that version
    differs from the design being calibrated.
    """
    raise NotImplementedError


@pytest.mark.req("INT-038")
@pytest.mark.xfail(strict=True, reason="stub")
def test_int_038() -> None:
    """Build-time design rules shall run on the design before a run and be able to fail a design without
    simulating it.
    """
    raise NotImplementedError
