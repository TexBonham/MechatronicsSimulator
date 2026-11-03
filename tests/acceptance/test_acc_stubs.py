"""Strict-xfail stubs for the ACC spec (acceptance tier)."""

import pytest


@pytest.mark.req("ACC-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_001() -> None:
    """Reference design (gantry), G1: when target (40, 25) is sent at power on before homing, no actuator
    shall move.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_002() -> None:
    """Reference design (gantry), G1: the refusal shall be reported with status "not homed"."""
    raise NotImplementedError


@pytest.mark.req("ACC-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_003() -> None:
    """Reference design (gantry), G2: after homing, each axis's origin shall lie within the tolerance of
    its true origin.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_004() -> None:
    """Reference design (gantry), G2: after homing, the status shall be "ready"."""
    raise NotImplementedError


@pytest.mark.req("ACC-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_005() -> None:
    """Reference design (gantry), G3: after target (40, 25) is sent following homing, the hook's final
    position shall lie within the tolerance of the target.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_006() -> None:
    """Reference design (gantry), G3: no travel limit shall be exceeded during the move."""
    raise NotImplementedError


@pytest.mark.req("ACC-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_007() -> None:
    """Reference design (gantry), G4: for a target outside the workspace, no actuator shall move."""
    raise NotImplementedError


@pytest.mark.req("ACC-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_008() -> None:
    """Reference design (gantry), G4: the target shall be rejected with a reported fault, not clipped."""
    raise NotImplementedError


@pytest.mark.req("ACC-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_009() -> None:
    """Reference design (gantry), G5: with a load over the limit attached before a move, a lift command
    shall be ignored.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_010() -> None:
    """Reference design (gantry), G5: with a load over the limit attached, a lower command shall work."""
    raise NotImplementedError


@pytest.mark.req("ACC-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_011() -> None:
    """Reference design (gantry), G6: when the load becomes overweight mid-lift, the lift shall stop."""
    raise NotImplementedError


@pytest.mark.req("ACC-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_012() -> None:
    """Reference design (gantry), G6: after an overweight stop, the load shall be able to be set down."""
    raise NotImplementedError


@pytest.mark.req("ACC-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_013() -> None:
    """Reference design (gantry), G7: when the E-stop is pressed mid-move with a load, motor power shall be
    cut within the relay delay.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_014() -> None:
    """Reference design (gantry), G7: after the E-stop, the hook shall not drop."""
    raise NotImplementedError


@pytest.mark.req("ACC-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_015() -> None:
    """Reference design (gantry), G7: after the E-stop, motion shall resume only after a reset and a new
    command.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_016() -> None:
    """Reference design (gantry), G8: when the controller freezes mid-move, the motors shall stop within
    the heartbeat timeout.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_017() -> None:
    """Reference design (gantry), G9: after power is lost and restored, no motion shall occur until the
    gantry is homed.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_018() -> None:
    """Reference design (gantry), G9: during the power loss and restore, the load shall not drop."""
    raise NotImplementedError


@pytest.mark.req("ACC-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_019() -> None:
    """Reference design (gantry), G10: on a motor stall or belt slip during a move, motion shall stop."""
    raise NotImplementedError


@pytest.mark.req("ACC-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_020() -> None:
    """Reference design (gantry), G10: on a motor stall or belt slip during a move, an encoder-mismatch
    fault shall be reported.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_021() -> None:
    """Reference design (gantry), G10: after a stall or slip fault, re-homing shall be required before
    further motion.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_022() -> None:
    """Reference design (gantry), G11: when a limit switch trips mid-workspace, motion shall stop."""
    raise NotImplementedError


@pytest.mark.req("ACC-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_023() -> None:
    """Reference design (gantry), G11: when a limit switch trips mid-workspace, a fault shall be reported."""
    raise NotImplementedError


@pytest.mark.req("ACC-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_024() -> None:
    """Reference design (gantry), G11: after a mid-workspace limit trip, re-homing shall be required before
    further motion.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_025() -> None:
    """Reference design (gantry), G12: after a fast move and after an E-stop, the peak load swing angle
    shall be within its limit.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_026() -> None:
    """Reference design (gantry), G12: after a fast move and after an E-stop, the swing settle time shall
    be within its limit.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-027")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_027() -> None:
    """Reference design (gantry), G13: a lift command while the gantry is moving shall be handled by the
    defined policy (queue or reject) in every run.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-028")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_028() -> None:
    """Reference design (gantry), G14: under rapid-fire and conflicting commands, no two actuator commands
    shall overlap.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_029() -> None:
    """Reference design (gantry), G15: under maximum acceleration with rated load and an E-stop with a
    swinging load, every member shall stay above the minimum safety factor.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-030")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_030() -> None:
    """Reference design (gantry), G15: under the same worst cases, member deflection shall stay under the
    target.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_031() -> None:
    """Reference design (gantry), G16: a sweep of NEMA 17 against NEMA 23 with the rated load shall report
    the torque margin per motor.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-032")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_032() -> None:
    """Reference design (gantry), G16: the same sweep shall report any missed steps per motor."""
    raise NotImplementedError


@pytest.mark.req("ACC-033")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_033() -> None:
    """Pendulum design, rigid link: the small-amplitude swing period shall match the small-angle formula
    for the link, 2 pi sqrt(I / (m g d)) (2 pi sqrt(L / g) for a point mass), within the period
    tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-034")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_034() -> None:
    """Pendulum design, rigid link: the swing amplitude decay shall match the closed-form decay for the
    link's stated joint damping within the decay tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-035")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_035() -> None:
    """Acceptance criterion 1: the simulator shall build the reference design from data files or an
    imported URDF with no design-specific code.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-036")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_036() -> None:
    """Acceptance criterion 2: the simulator shall run G1 to G16 in one batch and produce the pass/fail
    report.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-037")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_037() -> None:
    """Acceptance criterion 3, rewritten: the rope pendulum's period and swing decay shall match their
    closed-form values within tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-038")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_038() -> None:
    """Acceptance criterion 3, rewritten: the stepper's pull-out torque shall match its published curve
    within tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-039")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_039() -> None:
    """Acceptance criterion 3, rewritten: the settle time of a step response shall match its closed-form
    value within tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-040")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_040() -> None:
    """Acceptance criterion 4, rewritten: a cantilever beam's static deflection shall match the closed-form
    value within tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-041")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_041() -> None:
    """Acceptance criterion 4, rewritten: a cantilever beam's first natural frequency shall match the
    closed-form value within tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-042")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_042() -> None:
    """Acceptance criterion 4, rewritten: a simply supported beam's static deflection shall match the
    closed-form value within tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-043")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_043() -> None:
    """Acceptance criterion 4, rewritten: a simply supported beam's first natural frequency shall match the
    closed-form value within tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-044")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_044() -> None:
    """Acceptance criterion 5: a second, different design (the two-joint arm) shall be built and tested
    using only the library and data files.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-045")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_045() -> None:
    """Acceptance criterion 6: rerunning any case with the same seed shall give identical results."""
    raise NotImplementedError
