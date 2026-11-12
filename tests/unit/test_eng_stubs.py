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


@pytest.mark.req("ENG-035")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_035() -> None:
    """The engine shall accept a mid-run change of a payload's mass or attachment at a scheduled time."""
    raise NotImplementedError
