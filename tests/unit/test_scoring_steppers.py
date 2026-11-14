"""Stepper quantities from the record (SCO-031 missed steps, SCO-032 torque margin)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np
import pytest

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


def _windowed_recording() -> _WithColumns:
    rotor = [0.5, 0.5 + STEP, 0.5 + STEP, 0.5 + 1.5 * STEP]
    torque = [0.0, HOLD * math.sin(math.pi / 4.0), 0.0, 2.0 * HOLD]
    base = _recording([0.0, 4.0, 4.0, 2.0], [1.0, 1.0, 1.0, 0.0], rotor, torque)
    return _WithColumns(base, [0.1, -0.3, 0.05, 0.0])


def _windowed_case(start: str) -> Any:
    return case_with(
        value("final", "within", "0.25", quantity="missed_steps:m", target="0"),
        value("minimum", "above", "0 N*m", name="d", quantity="torque_margin:m"),
        integrity={"evaluate_from": start},
    )


@pytest.mark.req("SCO-031")
@pytest.mark.req("SCO-032")
def test_the_stepper_quantities_judge_the_evaluation_window() -> None:
    # Whole-run missed steps 0, -0.5, 1, -1 (rotor travel 1 step at 1 s, half-step lead) and margins 0.30, 0.05,
    # 0.15, 0 (the two tests above). From the case's
    # evaluation start 1.5 s the window's first sample is 2 s: missed steps counted from its value 1 are 0, -2
    # (final -2 at 3 s), the least margin over 2 s and 3 s is 0 at 3 s. Before it: the missed steps at the start
    # sample are 1, first at or beyond half a step at 1 s (-0.5); the least margin over 0 s and 1 s is 0.05 at 1 s.
    # A start exactly on a sample (2 s) gives the same window.
    for start in ("1.5 s", "2 s"):
        score = score_run(_windowed_case(start), _windowed_recording(), ENDED, stepper_drives=(MOTOR,))
        missed, margin = score.criteria
        print(f"windowed from {start}: missed {missed.measured} (expected -2), margin {margin.measured} (expected 0)")
        assert missed.measured == pytest.approx(-2.0, abs=1e-12)
        assert missed.worst_time == 3.0
        assert missed.status == "fail"
        assert margin.measured == pytest.approx(0.0, abs=1e-12)
        assert margin.worst_time == 3.0
        start_text = start.split(" ")[0]
        assert score.notes == (
            f"motor 'm' (part small_part): before the evaluation start {start_text} s, missed steps 1 (first beyond "
            "half a step at 1 s), least torque margin 0.05 N m at 1 s; the sizing criteria judge from the start "
            "(SCO-031, SCO-032, ruling 91)",
        )


@pytest.mark.req("SCO-031")
@pytest.mark.req("SCO-032")
def test_a_window_starting_after_a_clean_start_names_no_lost_step_time() -> None:
    # Start 0.5 s: the window opens at 1 s; before it only the 0 s sample (missed 0, margin 0.30). The start sample's
    # missed steps -0.5 are the value before the window (final -1 less it: -0.5); no sample before it reaches half a
    # step.
    score = score_run(_windowed_case("0.5 s"), _windowed_recording(), ENDED, stepper_drives=(MOTOR,))
    assert score.criteria[0].measured == pytest.approx(-0.5, abs=1e-12)
    assert score.notes == (
        "motor 'm' (part small_part): before the evaluation start 0.5 s, missed steps -0.5, least torque margin "
        "0.3 N m at 0 s; the sizing criteria judge from the start (SCO-031, SCO-032, ruling 91)",
    )


@pytest.mark.req("SCO-031")
@pytest.mark.req("SCO-032")
def test_no_before_line_without_a_start_an_evaluable_motor_or_a_run() -> None:
    zero = score_run(_windowed_case("0 s"), _windowed_recording(), ENDED, stepper_drives=(MOTOR,))
    assert zero.notes == ()
    assert zero.criteria[0].measured == pytest.approx(-1.0, abs=1e-12)
    undescribed = score_run(_windowed_case("1.5 s"), _windowed_recording(), ENDED)
    assert undescribed.notes == ()
    assert undescribed.criteria[0].status == "error"
    plain = _recording([0.0] * 4, [1.0] * 4, [0.0] * 4, [0.0] * 4)
    no_load = score_run(_windowed_case("1.5 s"), plain, ENDED, stepper_drives=(MOTOR,))
    assert no_load.notes == ()
    assert no_load.criteria[1].status == "error"
    failed = RunReport("run_error", 2, Fraction(1), "boom")
    errored = score_run(_windowed_case("1.5 s"), _windowed_recording(), failed, stepper_drives=(MOTOR,))
    assert errored.notes == ()


@pytest.mark.req("SCO-031")
@pytest.mark.req("SCO-032")
def test_an_evaluation_start_past_the_last_sample_is_unevaluable() -> None:
    score = score_run(_windowed_case("3.5 s"), _windowed_recording(), ENDED, stepper_drives=(MOTOR,))
    for result in score.criteria:
        assert result.status == "error"
        assert result.note is not None
        assert "no sample at or after the evaluation start 3.5 s" in result.note
    assert score.notes == ()


@pytest.mark.req("SCO-031")
def test_missed_steps_count_from_the_first_sample_of_a_motor_holding_a_load() -> None:
    # A static start: the first sample already carries 4 microsteps (1 full step commanded) and a drive torque of
    # HOLD sin(pi / 4) (lead 0.5 step) with the rotor at rest. Raw values: 1 - 0 - 0.5 = 0.5, then (1 more step
    # commanded, rotor 1 step on, same torque) 2 - 1 - 0.5 = 0.5, (torque 0) 2 - 1 - 0 = 1, (rotor 2 steps on, torque
    # HOLD sin(pi / 4)) 2 - 2 - 0.5 = -0.5. Relative to the first sample: 0, 0, 0.5, -1.
    rotor = [0.0, STEP, STEP, 2.0 * STEP]
    held = HOLD * math.sin(math.pi / 4.0)
    recording = _recording([4.0, 4.0, 0.0, 0.0], [1.0] * 4, rotor, [held, held, 0.0, held])
    for statistic, measured, worst in (("final", -1.0, 3.0), ("maximum", 0.5, 2.0)):
        result = _score("missed_steps:m", statistic, "at_most", "5", recording)
        print(f"held start missed steps {statistic}: expected {measured}, measured {result.measured}")
        assert result.measured == pytest.approx(measured, abs=1e-12)
        assert result.worst_time == worst
    first = _score("missed_steps:m", "final", "at_most", "5", recording, window={"end": "0.5 s"})
    assert first.measured == 0.0
