"""ENG stubs, unit tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("ENG-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_013() -> None:
    """The engine shall enforce joint travel limits as one-sided constraints."""
    raise NotImplementedError


@pytest.mark.req("ENG-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_014() -> None:
    """The engine shall apply joint friction and joint damping from per-joint parameters."""
    raise NotImplementedError


@pytest.mark.req("ENG-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_015() -> None:
    """The engine shall apply joint backlash from a per-joint parameter."""
    raise NotImplementedError


@pytest.mark.req("ENG-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_016() -> None:
    """The engine shall detect collisions between primitives and between convex hulls with in-house code."""
    raise NotImplementedError


@pytest.mark.req("ENG-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_017() -> None:
    """The engine shall end the run with an error when a contact geometry is neither a primitive nor a convex hull."""
    raise NotImplementedError


@pytest.mark.req("ENG-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_020() -> None:
    """The engine shall model contact as soft contact integrated implicitly, with contact friction."""
    raise NotImplementedError


@pytest.mark.req("ENG-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_029() -> None:
    """The engine shall run faster than real time for batch runs."""
    raise NotImplementedError


@pytest.mark.req("ENG-030")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_030() -> None:
    """The engine shall publish the end-loads (three forces and three moments at each end) of every lumped member
    every step.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_031() -> None:
    """The engine shall accept a mid-run change that sets a permanent offset on a lumped spring."""
    raise NotImplementedError


@pytest.mark.req("ENG-032")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_032() -> None:
    """The engine shall accept a mid-run change of a lumped spring's stiffness."""
    raise NotImplementedError


@pytest.mark.req("ENG-033")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_033() -> None:
    """The engine shall accept a mid-run disable of a constraint."""
    raise NotImplementedError


@pytest.mark.req("ENG-034")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_034() -> None:
    """The engine shall accept a mid-run release of a load from its attachment."""
    raise NotImplementedError


@pytest.mark.req("ENG-035")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_035() -> None:
    """The engine shall accept a mid-run change of a payload's mass or attachment at a scheduled time."""
    raise NotImplementedError


@pytest.mark.req("ENG-047")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_047() -> None:
    """The engine shall simulate a rope as a lumped chain with unilateral tension, slack and length change."""
    raise NotImplementedError
