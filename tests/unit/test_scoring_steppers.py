"""Stepper quantities from the record (SCO-031 missed steps, SCO-032 torque margin)."""

from __future__ import annotations

import math
from pathlib import Path
from dataclasses import dataclass
from typing import Any

import numpy as np
import pytest

from mechsim.scoring.score import LOAD_TORQUE, StepperDrive, available_torque, score_run
from mechsim.testing.core_driver import core_services
from mechsim.testing.driver import Series
from mechsim.testing.runner import prepare, stepper_drives
from mechsim.testing.schema import cross_check
from tests.unit.test_runner_fakes import FakeRecording, recording_from
from tests.unit.test_scoring import ENDED, case_with, value

STEP = 2.0 * math.pi / 200.0
HOLD = 0.4
CURVE = ((0.0, 0.40), (10.0, 0.30), (20.0, 0.10))
TIMES = [0.0, 1.0, 2.0, 3.0]
MOTOR = StepperDrive("m", "small_part", "spin", STEP, HOLD, CURVE, "drv", 4, "belt")


@dataclass
class _Stored:
    values: Any
    names: tuple[str, ...]


@dataclass
class _Raw:
    series: dict[str, _Stored]


class _WithColumns(FakeRecording):
    def __init__(self, base: FakeRecording, load: list[float], names: tuple[str, ...] = ("other", "belt")) -> None:
        super().__init__(base._times, base._rows, base._units)
        rows = [[0.0, item] for item in load]
        self.recording = _Raw({LOAD_TORQUE: _Stored(np.asarray(rows), names)})

    def series(self, reference: str) -> Series:
        return super().series(reference)


def _recording(steps: list[float], dirs: list[float], rotor: list[float], torque: list[float]) -> FakeRecording:
    return recording_from(
        TIMES,
        {
            "signal:drv.step": steps,
            "signal:drv.dir": dirs,
            "joint:spin.position": rotor,
            "joint:spin.velocity": [0.0, 5.0, -15.0, 25.0],
            "signal:m.torque": torque,
        },
    )


def _score(quantity: str, statistic: str, comparison: str, limit: str, recording: Any, **extra: Any) -> Any:
    case = case_with(value(statistic, comparison, limit, quantity=quantity, **extra))
    return score_run(case, recording, ENDED, stepper_drives=(MOTOR,)).criteria[0]


@pytest.mark.req("SCO-031")
def test_missed_steps_is_the_commanded_angle_less_the_rotor_travel_less_the_lead() -> None:
    # Microstep STEP / 4. Commanded: +4, +4 (dir high), then 2 back (dir low): 0, 1, 2, 1.5 full steps. Rotor from
    # its first sample 0.5 rad: travel 0, 1, 1, 1.5 steps. Drive torque 0, HOLD sin(pi / 4) (lead (2 / pi)(pi / 4)
    # = 0.5 step), 0, then 2 HOLD clipped to HOLD (lead 1 step). Missed: 0, 1 - 0 - 0.5, 2 - 1 - 0, 1.5 - 1.5 - 1.
    rotor = [0.5, 0.5 + STEP, 0.5 + STEP, 0.5 + 1.5 * STEP]
    torque = [0.0, HOLD * math.sin(math.pi / 4.0), 0.0, 2.0 * HOLD]
    recording = _recording([0.0, 4.0, 4.0, 2.0], [1.0, 1.0, 1.0, 0.0], rotor, torque)
    for statistic, measured, worst in (("final", -1.0, 3.0), ("maximum", 1.0, 2.0), ("minimum", -1.0, 3.0)):
        result = _score("missed_steps:m", statistic, "at_most", "5", recording)
        print(f"missed steps {statistic}: expected {measured}, measured {result.measured}")
        assert result.measured == pytest.approx(measured, abs=1e-12)
        assert result.worst_time == worst
        assert result.note == "part small_part"
    whole = _score("missed_steps:m", "final", "within", "0.25", recording, target="-1")
    assert whole.status == "pass"


@pytest.mark.req("SCO-031")
def test_missed_steps_without_driver_rotor_or_description_is_unevaluable() -> None:
    rotor = [0.0] * 4
    recording = _recording([0.0] * 4, [1.0] * 4, rotor, [0.0] * 4)
    unknown = _score("missed_steps:other", "final", "at_most", "1", recording)
    assert (unknown.status, unknown.note) == ("error", "missed_steps:other: 'other' is not a stepper motor of the run")
    undriven = StepperDrive("m", "small_part", "spin", STEP, HOLD, CURVE, None, 1, "belt")
    case = case_with(value("final", "at_most", "1", quantity="missed_steps:m"))
    result = score_run(case, recording, ENDED, stepper_drives=(undriven,)).criteria[0]
    assert (result.status, result.note) == ("error", "missed_steps:m: motor 'm' has no step and direction driver")
    no_rotor = recording_from(TIMES, {"signal:drv.step": [0.0] * 4, "signal:drv.dir": [1.0] * 4})
    missing = _score("missed_steps:m", "final", "at_most", "1", no_rotor)
    assert missing.status == "error"
    assert missing.note == "missed_steps:m: motor 'm' has no recorded rotor position (joint:spin.position)"
    no_pin = recording_from(TIMES, {"joint:spin.position": rotor})
    assert _score("missed_steps:m", "final", "at_most", "1", no_pin).note == (
        "missed_steps:m: motor 'm' has no recorded step pin (signal:drv.step)"
    )


@pytest.mark.req("SCO-032")
@pytest.mark.parametrize(
    ("speed", "expected"),
    [(0.0, 0.40), (5.0, 0.35), (-5.0, 0.35), (10.0, 0.30), (15.0, 0.20), (20.0, 0.10), (20.5, 0.0), (-25.0, 0.0)],
)
def test_available_torque_is_the_curve_linear_between_points_and_zero_above_it(speed: float, expected: float) -> None:
    assert available_torque(CURVE, speed) == pytest.approx(expected, abs=1e-12)


@pytest.mark.req("SCO-032")
def test_torque_margin_is_the_curve_at_the_shaft_speed_less_the_load_torque() -> None:
    # Speeds 0, 5, -15, 25 rad/s: available 0.40, 0.35, 0.20, 0 (above the curve). Loads 0.1, -0.3, 0.05, 0:
    # margins 0.30, 0.05, 0.15, 0.
    base = _recording([0.0] * 4, [1.0] * 4, [0.0] * 4, [0.0] * 4)
    recording = _WithColumns(base, [0.1, -0.3, 0.05, 0.0])
    result = _score("torque_margin:m", "minimum", "above", "0 N*m", recording)
    print(f"torque margin minimum: expected 0 at 3 s, measured {result.measured} at {result.worst_time}")
    assert (result.status, result.unit, result.note) == ("fail", "N m", "part small_part")
    assert result.measured == pytest.approx(0.0, abs=1e-12)
    assert result.worst_time == 3.0
    early = _score("torque_margin:m", "minimum", "above", "0 N*m", recording, window={"end": "2 s"})
    assert early.status == "pass"
    assert early.measured == pytest.approx(0.05, abs=1e-12)
    assert early.worst_time == 1.0


@pytest.mark.req("SCO-032")
def test_torque_margin_without_a_load_torque_column_is_unevaluable() -> None:
    base = _recording([0.0] * 4, [1.0] * 4, [0.0] * 4, [0.0] * 4)
    plain = _score("torque_margin:m", "minimum", "above", "0 N*m", base)
    assert (plain.status, plain.note) == ("error", "torque_margin:m: motor 'm' has no recorded shaft load torque")
    other = _WithColumns(base, [0.0] * 4, ("other", "chain"))
    result = _score("torque_margin:m", "minimum", "above", "0 N*m", other)
    assert result.note == "torque_margin:m: motor 'm' has no recorded shaft load torque (transmission 'belt')"
    case = case_with(value("minimum", "above", "0 N*m", quantity="torque_margin:m"))
    no_transmission = StepperDrive("m", "small_part", "spin", STEP, HOLD, CURVE, "drv", 4, None)
    alone = score_run(case, _WithColumns(base, [0.0] * 4), ENDED, stepper_drives=(no_transmission,)).criteria[0]
    assert alone.note == "torque_margin:m: motor 'm' has no recorded shaft load torque"


@pytest.mark.req("SCO-032")
def test_a_torque_margin_limit_has_the_torque_dimension() -> None:
    with pytest.raises(ValueError, match="expected"):
        case_with(value("minimum", "above", "0 m", quantity="torque_margin:m"))


GANTRY_G16 = Path(__file__).resolve().parents[2] / "test_cases" / "gantry" / "G16.yaml"


@pytest.mark.req("SCO-031")
@pytest.mark.req("SCO-032")
@pytest.mark.req("LIB-017")
def test_the_runner_describes_each_stepper_from_the_build_after_part_substitution() -> None:
    prepared = prepare(GANTRY_G16, {"instance:lift_motor.part": "nema17_class_generic"})
    drives = {item.motor: item for item in stepper_drives(core_services().builder(prepared.design))}
    assert set(drives) == {"lift_motor", "x_motor", "y_motor"}
    lift = drives["lift_motor"]
    assert (lift.part, lift.shaft_joint, lift.driver, lift.microsteps, lift.transmission) == (
        "nema17_class_generic",
        "lift_motor_spin",
        "lift_driver",
        16,
        "lift_belt",
    )
    assert lift.step_angle == pytest.approx(math.radians(1.8), rel=1e-12)
    assert lift.holding_torque == pytest.approx(0.40, rel=1e-12)
    assert lift.torque_speed[1] == pytest.approx((300.0 * 2.0 * math.pi / 60.0, 0.36), rel=1e-12)
    assert drives["x_motor"].part == "nema23_class_generic"
    assert drives["x_motor"].holding_torque == pytest.approx(1.26, rel=1e-12)


@pytest.mark.req("SCO-031")
def test_a_build_without_a_wiring_table_describes_no_stepper() -> None:
    class Bare:
        design = None
        module_instances: tuple[str, ...] = ()

    assert stepper_drives(Bare()) == ()  # type: ignore[arg-type]


@pytest.mark.req("SCO-031")
@pytest.mark.req("SCO-032")
def test_a_stepper_quantity_must_name_a_component_of_the_design() -> None:
    prepared = prepare(GANTRY_G16, {})
    known = case_with(
        value("final", "at_most", "1", quantity="missed_steps:lift_motor"),
        value("minimum", "above", "0 N*m", name="d", quantity="torque_margin:x_motor"),
        design={"name": "gantry", "version": "0.1.0"},
    )
    assert cross_check(known, prepared.design) == []
    unknown = case_with(
        value("final", "at_most", "1", quantity="missed_steps:nobody"),
        value("minimum", "above", "0 N*m", name="d", quantity="torque_margin:nothing"),
        design={"name": "gantry", "version": "0.1.0"},
    )
    problems = cross_check(unknown, prepared.design)
    assert len(problems) == 2
    assert "unknown component 'nobody'" in problems[0]
    assert "unknown component 'nothing'" in problems[1]
