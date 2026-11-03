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


@pytest.mark.req("ENG-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_022() -> None:
    """The engine shall simulate a flexible member as a multi-segment lumped beam built from the member's section and
    material.
    """
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


@pytest.mark.req("ENG-043")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_043() -> None:
    """The engine shall simulate a belt and pulley transmission element of the neutral model with its ratio, efficiency,
    stiffness, backlash and slip parameters.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-044")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_044() -> None:
    """The engine shall simulate a lead screw transmission element with its lead, efficiency, stiffness, backlash and
    slip parameters, and a self-locking lead screw shall hold its joint against back-driving load with no motor torque.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-045")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_045() -> None:
    """The engine shall simulate a gear pair transmission element with its ratio, efficiency, stiffness, backlash and
    slip parameters.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-046")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_046() -> None:
    """Every transmission element's slip parameter shall be zero by default, and the engine shall accept a mid-run
    change of it so that a slip fault makes that transmission slip for the fault window.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-047")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_047() -> None:
    """The engine shall simulate a rope as a lumped chain with unilateral tension, slack and length change."""
    raise NotImplementedError


@pytest.mark.req("ENG-048")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_048() -> None:
    """Each transmission element shall map the motor shaft torque the signal layer applies into joint force for the
    step, and the engine shall publish the load torque on each motor shaft after every step for the stepper model.
    """
    raise NotImplementedError
