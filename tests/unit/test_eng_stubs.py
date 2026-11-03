"""ENG stubs, unit tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("ENG-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_001() -> None:
    """The engine shall do its linear algebra through Eigen and shall carry no other linear algebra dependency."""
    raise NotImplementedError


@pytest.mark.req("ENG-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_002() -> None:
    """The engine shall advance the model by a fixed step whose length is supplied by the run configuration, with 1
    ms as the default and no step length held as a code constant.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_003() -> None:
    """The engine shall integrate the equations of motion with the scheme ruled by the own-engine integrator ADR (ADR
    0013, proposed), behind an integrator interface internal to the engine.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_004() -> None:
    """The engine shall hold every state, parameter and output quantity in SI units."""
    raise NotImplementedError


@pytest.mark.req("ENG-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_005() -> None:
    """The engine shall simulate rigid bodies with mass, center of mass and inertia tensor under a gravity vector
    supplied by the model.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_006() -> None:
    """The engine shall provide a revolute joint."""
    raise NotImplementedError


@pytest.mark.req("ENG-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_007() -> None:
    """The engine shall provide a prismatic joint."""
    raise NotImplementedError


@pytest.mark.req("ENG-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_008() -> None:
    """The engine shall provide a fixed joint."""
    raise NotImplementedError


@pytest.mark.req("ENG-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_009() -> None:
    """The engine shall provide a planar joint."""
    raise NotImplementedError


@pytest.mark.req("ENG-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_010() -> None:
    """The engine shall provide a spherical joint."""
    raise NotImplementedError


@pytest.mark.req("ENG-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_011() -> None:
    """The engine shall provide a free joint (six degrees of freedom) so an unattached body such as a payload
    resting on the floor is a tree root with no constraint to the world."""
    raise NotImplementedError


@pytest.mark.req("ENG-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_012() -> None:
    """The engine shall close kinematic loops through bilateral constraints between bodies."""
    raise NotImplementedError


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


@pytest.mark.req("ENG-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_024() -> None:
    """The engine shall measure constraint drift every step and end the run with a drift error, carrying the time and
    the offending constraint, when drift exceeds the drift threshold.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_025() -> None:
    """The engine shall end the run with a solver-failure error, carrying the time and cause, when a state value
    becomes non-finite or the solver fails to converge within its iteration limit.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_026() -> None:
    """The engine shall publish a solver health record every step: maximum constraint drift and solver convergence
    status.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-027")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_027() -> None:
    """The engine shall publish body poses and velocities, joint forces and contact forces after every step."""
    raise NotImplementedError


@pytest.mark.req("ENG-028")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_028() -> None:
    """The engine shall accept actuator torques and forces before each step and apply them for that step only."""
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
