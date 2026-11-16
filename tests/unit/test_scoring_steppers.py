"""Stepper quantities from the record (SCO-031 missed steps, SCO-032 torque margin)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mechsim.library.registry import default_registry
from mechsim.scoring.score import LOAD_TORQUE, StepperDrive, available_torque, score_run
from mechsim.testing.core_driver import core_services
from mechsim.testing.driver import RunReport, Series
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


def _recording(
    steps: list[float],
    dirs: list[float],
    rotor: list[float],
    torque: list[float],
    speeds: tuple[float, ...] = (0.0, 5.0, -15.0, 25.0),
) -> FakeRecording:
    return recording_from(
        TIMES,
        {
            "signal:drv.step": steps,
            "signal:drv.dir": dirs,
            "joint:spin.position": rotor,
            "joint:spin.velocity": list(speeds),
            "signal:m.torque": torque,
        },
    )


def _score(quantity: str, statistic: str, comparison: str, limit: str, recording: Any, **extra: Any) -> Any:
    case = case_with(value(statistic, comparison, limit, quantity=quantity, **extra))
    return score_run(case, recording, ENDED, stepper_drives=(MOTOR,)).criteria[0]


@pytest.mark.req("SCO-031")
def test_missed_steps_is_the_commanded_angle_less_the_rotor_travel_less_the_lead() -> None:
    # Microstep STEP / 4. Commanded: +4, +4 (dir high), then none: 0, 1, 2, 2 full steps. Rotor from its first sample
    # 0.5 rad: travel 0, 1, 1, 2 steps. Drive torque 0, HOLD sin(pi / 4) (lead (2 / pi)(pi / 4) = 0.5 step), 0, 0.
    # Missed: 0, 1 - 1 - 0.5, 2 - 1 - 0, 2 - 2 - 0. The first and last samples are at rest (no microstep, speed 0).
    rotor = [0.5, 0.5 + STEP, 0.5 + STEP, 0.5 + 2.0 * STEP]
    torque = [0.0, HOLD * math.sin(math.pi / 4.0), 0.0, 0.0]
    recording = _recording([0.0, 4.0, 4.0, 0.0], [1.0, 1.0, 1.0, 0.0], rotor, torque, (0.0, 5.0, -15.0, 0.0))
    for statistic, measured, worst in (("final", 0.0, 3.0), ("maximum", 1.0, 2.0), ("minimum", -0.5, 1.0)):
        result = _score("missed_steps:m", statistic, "at_most", "5", recording)
        print(f"missed steps {statistic}: expected {measured}, measured {result.measured}")
        assert result.measured == pytest.approx(measured, abs=1e-12)
        assert result.worst_time == worst
        assert result.note == "part small_part"
    whole = _score("missed_steps:m", "final", "within", "0.25", recording, target="0")
    assert whole.status == "pass"
    moving = _recording([0.0, 4.0, 4.0, 2.0], [1.0] * 4, rotor, torque)
    stopped = _score("missed_steps:m", "final", "at_most", "5", moving)
    assert (stopped.status, stopped.note) == (
        "error",
        "missed_steps:m: motor 'm' is not at rest at the final sample 3 s",
    )


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
    x_part = next(item.part for item in prepared.design.components if item.name == "x_motor")
    assert drives["x_motor"].part == x_part
    assert drives["x_motor"].holding_torque == pytest.approx(
        default_registry().part(x_part).values["holding_torque"], rel=1e-12
    )


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


def _windowed_recording(final_speed: float = 0.0, first_speed: float = 0.0) -> _WithColumns:
    rotor = [0.0, STEP, STEP, 0.0]
    torque = [0.0, HOLD, 0.0, 0.0]
    speeds = (first_speed, 5.0, 0.0, final_speed)
    base = _recording([0.0, 4.0, 0.0, 0.0], [1.0] * 4, rotor, torque, speeds)
    return _WithColumns(base, [0.1, -0.3, 0.05, 0.0])


def _windowed_case(start: str) -> Any:
    return case_with(
        value("final", "within", "0.25", quantity="missed_steps:m", target="0"),
        value("minimum", "above", "0 N*m", name="d", quantity="torque_margin:m"),
        integrity={"evaluate_from": start},
    )


# The window recording: 4 microsteps (1 step) commanded at 1 s, the rotor follows, then slips back a step at 3 s.
# Missed steps 0, 1 - 1 - 1 = -1 (lead 1 step at HOLD), 1 - 1 - 0 = 0, 1 - 0 - 0 = 1. Rest (no microstep and speed
# 0; threshold a tenth of a microstep per 1 s interval): 0 s, 2 s, 3 s; 1 s moves. Margins at speeds 0, 5, 0, 0
# (available 0.40, 0.35, 0.40, 0.40) less loads 0.1, 0.3, 0.05, 0: 0.30, 0.05, 0.35, 0.40.


@pytest.mark.req("SCO-031")
@pytest.mark.req("SCO-032")
def test_the_stepper_quantities_judge_the_span_from_the_rest_before_the_evaluation_start() -> None:
    # Start 2.5 s or 2 s: the last rest at or before it is 2 s. Missed steps from it: 0, 1 (final 1 at 3 s, the step
    # lost in the span); least margin over 2 s and 3 s 0.35 at 2 s. Before the rest: count 0 at it, first at or
    # beyond half a step at 1 s (-1); least margin over 0 s and 1 s 0.05 at 1 s.
    for start in ("2.5 s", "2 s"):
        score = score_run(_windowed_case(start), _windowed_recording(), ENDED, stepper_drives=(MOTOR,))
        missed, margin = score.criteria
        print(f"span from the rest before {start}: missed {missed.measured} (expected 1), margin {margin.measured}")
        assert missed.measured == pytest.approx(1.0, abs=1e-12)
        assert (missed.worst_time, missed.status) == (3.0, "fail")
        assert margin.measured == pytest.approx(0.35, abs=1e-12)
        assert margin.worst_time == 2.0
        start_text = start.split(" ")[0]
        assert score.notes == (
            f"motor 'm' (part small_part): before the rest at 2 s (evaluation start {start_text} s), missed steps 0 "
            "(first beyond half a step at 1 s), least torque margin 0.05 N m at 1 s; the sizing criteria judge from "
            "that rest (SCO-031, SCO-032, rulings 91, 93)",
        )


@pytest.mark.req("SCO-031")
@pytest.mark.req("SCO-032")
def test_a_start_inside_a_move_judges_the_move_whole_from_the_rest_before_it() -> None:
    # Start 1.5 s falls in the move at 1 s: the span starts at the rest at 0 s, the whole run; nothing precedes it, so
    # no before line. Missed final 1, least margin 0.05 at 1 s.
    score = score_run(_windowed_case("1.5 s"), _windowed_recording(), ENDED, stepper_drives=(MOTOR,))
    assert score.criteria[0].measured == pytest.approx(1.0, abs=1e-12)
    assert score.criteria[1].measured == pytest.approx(0.05, abs=1e-12)
    assert score.criteria[1].worst_time == 1.0
    assert score.notes == ()


@pytest.mark.req("SCO-031")
@pytest.mark.req("SCO-032")
def test_rest_needs_an_idle_step_pin_a_slow_shaft_and_a_final_rest() -> None:
    # Final speed 3 rad/s (above STEP / 4 / 10 per second): the count is unevaluable, the margin still judged from
    # the rest at 2 s. First sample moving at 5 rad/s and a start at 0.5 s: no rest at or before it, both unevaluable.
    moving_end = score_run(_windowed_case("2 s"), _windowed_recording(final_speed=3.0), ENDED, stepper_drives=(MOTOR,))
    assert moving_end.criteria[0].status == "error"
    assert moving_end.criteria[0].note == "missed_steps:m: motor 'm' is not at rest at the final sample 3 s"
    assert moving_end.criteria[1].measured == pytest.approx(0.35, abs=1e-12)
    slow = 0.09 * STEP / 4.0
    creeping = score_run(_windowed_case("2 s"), _windowed_recording(final_speed=slow), ENDED, stepper_drives=(MOTOR,))
    assert creeping.criteria[0].measured == pytest.approx(1.0, abs=1e-12)
    early = score_run(_windowed_case("0.5 s"), _windowed_recording(first_speed=5.0), ENDED, stepper_drives=(MOTOR,))
    for result in early.criteria:
        assert result.status == "error"
        assert result.note is not None
        assert "no rest sample at or before the evaluation start 0.5 s" in result.note
    assert early.notes == ()


@pytest.mark.req("SCO-031")
@pytest.mark.req("SCO-032")
def test_no_before_line_without_a_start_an_evaluable_motor_or_a_run() -> None:
    zero = score_run(_windowed_case("0 s"), _windowed_recording(), ENDED, stepper_drives=(MOTOR,))
    assert zero.notes == ()
    assert zero.criteria[0].measured == pytest.approx(1.0, abs=1e-12)
    undescribed = score_run(_windowed_case("2 s"), _windowed_recording(), ENDED)
    assert undescribed.notes == ()
    assert undescribed.criteria[0].status == "error"
    plain = _recording([0.0] * 4, [1.0] * 4, [0.0] * 4, [0.0] * 4, (0.0,) * 4)
    no_load = score_run(_windowed_case("2 s"), plain, ENDED, stepper_drives=(MOTOR,))
    assert no_load.notes == ()
    assert no_load.criteria[1].status == "error"
    failed = RunReport("run_error", 2, Fraction(1), "boom")
    errored = score_run(_windowed_case("2 s"), _windowed_recording(), failed, stepper_drives=(MOTOR,))
    assert errored.notes == ()


@pytest.mark.req("SCO-031")
def test_missed_steps_count_from_the_first_sample_of_a_motor_holding_a_load() -> None:
    # A static start: at rest at 0 s with a drive torque HOLD sin(pi / 4) (lead 0.5 step). Then 4 microsteps (1 step)
    # and the rotor follows; torque 0 at 2 s; held again at 3 s. Raw: 0 - 0 - 0.5, 1 - 1 - 0.5, 1 - 1 - 0, 1 - 1 - 0.5
    # = -0.5, -0.5, 0, -0.5; relative to the first sample 0, 0, 0.5, 0 (unreferenced, a held motor would read -0.5).
    held = HOLD * math.sin(math.pi / 4.0)
    rotor = [0.0, STEP, STEP, STEP]
    recording = _recording([0.0, 4.0, 0.0, 0.0], [1.0] * 4, rotor, [held, held, 0.0, held], (0.0, 5.0, 0.0, 0.0))
    for statistic, measured, worst in (("final", 0.0, 3.0), ("maximum", 0.5, 2.0)):
        result = _score("missed_steps:m", statistic, "at_most", "5", recording)
        print(f"held start missed steps {statistic}: expected {measured}, measured {result.measured}")
        assert result.measured == pytest.approx(measured, abs=1e-12)
        assert result.worst_time == worst
