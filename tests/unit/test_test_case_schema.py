"""Unit tier: test case file schema, design cross-check and loader (TST)."""

from __future__ import annotations

import copy
import functools
import io
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError
from ruamel.yaml import YAML

from mechsim.design.schema import load_design, yaml_loader
from mechsim.testing.schema import (
    Case,
    CaseError,
    ControllerFreeze,
    PayloadChange,
    Quantity,
    SensorForced,
    ValueCriterion,
    cross_check,
    load_test_case,
    parse_any_quantity,
    validate_test_case_file,
)

ROOT = Path(__file__).resolve().parents[2]
G3 = ROOT / "test_cases" / "gantry" / "G3.yaml"
P1 = ROOT / "test_cases" / "pendulum" / "P1_period.yaml"
GANTRY = ROOT / "designs" / "gantry" / "design.yaml"
PENDULUM = ROOT / "designs" / "pendulum" / "design.yaml"


@functools.cache
def _raw(path: Path) -> Any:
    return YAML(typ="safe").load(path.read_text(encoding="utf-8"))


def gantry_case() -> dict[str, Any]:
    return copy.deepcopy(_raw(G3))


def pendulum_case() -> dict[str, Any]:
    return copy.deepcopy(_raw(P1))


def rejected(data: Any, *fragments: str) -> None:
    with pytest.raises(ValidationError) as caught:
        Case.model_validate(data)
    text = str(caught.value)
    for fragment in fragments:
        assert fragment in text, text


def accepted(data: Any) -> Case:
    return Case.model_validate(data)


def check_errors(data: dict[str, Any]) -> list[str]:
    case = accepted(data)
    path = PENDULUM
    if case.design.name == "gantry":
        path = GANTRY
    return cross_check(case, load_design(path))


def cross_rejected(data: dict[str, Any], *fragments: str) -> None:
    text = "\n".join(check_errors(data))
    assert text != ""
    for fragment in fragments:
        assert fragment in text, text


def window(start: str = "1 s", **extra: Any) -> dict[str, Any]:
    return {"start": start, "duration": "2 s", **extra}


def fault(kind: str, **fields: Any) -> dict[str, Any]:
    return {"kind": kind, **window(), **fields}


VALID_FAULTS = {
    "sensor_stuck": fault("sensor_stuck", target="x_min_switch"),
    "sensor_noisy": fault("sensor_noisy", target="x_encoder", standard_deviation="0.1"),
    "sensor_dropped": fault("sensor_dropped", target="x_encoder"),
    "sensor_forced": fault("sensor_forced", target="x_min_switch", level="high"),
    "slip": fault("slip", target="x_belt", fraction="20 percent"),
    "motor_stall": fault("motor_stall", target="x_motor", available_torque="0.1"),
    "power_loss": fault("power_loss", rail="motor_power"),
    "controller_freeze": fault("controller_freeze"),
    "estop_press": fault("estop_press", target="estop"),
    "payload_change": {"kind": "payload_change", "start": "1 s", "target": "hook", "mass": "2 kg"},
}


def with_faults(*faults: dict[str, Any]) -> dict[str, Any]:
    data = gantry_case()
    data["end"] = {"at": "10 s", "max_duration": "10 s"}
    data["commands"][1]["at"] = "5 s"
    for criterion in data["criteria"]:
        if "window" in criterion:
            criterion["window"]["start"] = "5 s"
    data["open_parameters"] = [
        item for item in data["open_parameters"] if item["name"] not in ("homing_time", "run_time")
    ]
    data["faults"] = copy.deepcopy(list(faults))
    return data


# ===== Shipped examples ================================================


@pytest.mark.req("TST-009")
@pytest.mark.parametrize("path", [G3, P1], ids=["G3", "P1"])
def test_shipped_cases_validate_against_their_designs(path: Path) -> None:
    assert validate_test_case_file(path) == []
    assert load_test_case(path).id == path.stem.split("_")[0]


@pytest.mark.req("TST-009")
@pytest.mark.parametrize("path", [G3, P1], ids=["G3", "P1"])
def test_round_trip_preserves_bytes_comments_and_order(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    loader = yaml_loader()
    data = loader.load(text)
    stream = io.StringIO()
    loader.dump(data, stream)
    assert stream.getvalue() == text
    assert list(data.keys())[:3] == ["id", "title", "design"]


@pytest.mark.req("TST-001", "TST-002", "TST-003", "TST-005", "TST-006", "TST-010")
def test_g3_holds_homing_then_target_and_si_values() -> None:
    case = accepted(gantry_case())
    assert case.design.name == "gantry"
    assert case.design.version == "0.1.0"
    assert case.controller_mode == "native"
    assert case.seed == 3
    assert [command.message for command in case.commands] == ["home", "target"]
    assert case.commands[0].at == 0.0
    assert case.initial_state.controller == {"homed": False}
    assert case.initial_state.payload is not None
    assert case.initial_state.payload.attached is False
    final_x = case.criteria[0]
    assert isinstance(final_x, ValueCriterion)
    assert isinstance(final_x.limit, Quantity)
    assert final_x.limit.value == pytest.approx(0.01)
    assert isinstance(final_x.target, Quantity)
    assert final_x.target.value == pytest.approx(0.4)
    assert final_x.limit_text() == "final within 1 cm of 40 cm"
    assert case.baseline == "baseline.yaml"


@pytest.mark.req("TST-002", "TST-010", "SCO-020")
def test_p1_releases_the_pendulum_with_a_relative_period_limit() -> None:
    """Parsing only: the period criterion and its relative limit load; measuring the period is the scorer's."""
    case = accepted(pendulum_case())
    assert case.controller_mode is None
    state = case.initial_state.joints[0]
    assert state.motion() == "angular"
    assert isinstance(state.position, Quantity)
    assert state.position.value == pytest.approx(0.05)
    period = case.criteria[0]
    assert isinstance(period, ValueCriterion)
    assert period.relative_limit == pytest.approx(0.001)
    assert isinstance(period.target, Quantity)
    assert period.target.value == pytest.approx(1.1596536)
    assert period.limit_text() == "period within 0.1 percent of 1.1596536 s"
    assert case.end.at == pytest.approx(24.4)


# ===== Header ==========================================================


@pytest.mark.parametrize(
    "field",
    [
        pytest.param("id", marks=pytest.mark.req("TST-009")),
        pytest.param("title", marks=pytest.mark.req("TST-009")),
        pytest.param("design", marks=pytest.mark.req("TST-001")),
        pytest.param("seed", marks=pytest.mark.req("TST-006")),
        pytest.param("initial_state", marks=pytest.mark.req("TST-002")),
        pytest.param("end", marks=pytest.mark.req("TST-009")),
        pytest.param("criteria", marks=pytest.mark.req("TST-005")),
    ],
)
def test_required_field_missing_rejected(field: str) -> None:
    data = pendulum_case()
    del data[field]
    rejected(data, field, "Field required")


@pytest.mark.req("TST-001")
@pytest.mark.parametrize("field", ["name", "version"])
def test_design_reference_field_missing_rejected(field: str) -> None:
    data = pendulum_case()
    del data["design"][field]
    rejected(data, f"design.{field}")


@pytest.mark.req("TST-009")
@pytest.mark.parametrize("value", ["G3", "P1", "G16", "AB_12"])
def test_case_id_pattern_accepted(value: str) -> None:
    data = pendulum_case()
    data["id"] = value
    assert accepted(data).id == value


@pytest.mark.req("TST-009")
@pytest.mark.parametrize("value", ["g3", "G", "3G", "G3a", "G-3", ""])
def test_case_id_pattern_rejected(value: str) -> None:
    data = pendulum_case()
    data["id"] = value
    rejected(data, "id")


@pytest.mark.req("TST-001")
@pytest.mark.parametrize("value", ["0.1", "v0.1.0", "0.1.0-rc1", ""])
def test_design_version_must_be_semver(value: str) -> None:
    data = pendulum_case()
    data["design"]["version"] = value
    rejected(data, "design.version")


@pytest.mark.req("TST-006")
@pytest.mark.parametrize("value", [0, 1, 2**64 - 1])
def test_seed_bounds_accepted(value: int) -> None:
    data = pendulum_case()
    data["seed"] = value
    assert accepted(data).seed == value


@pytest.mark.req("TST-006")
@pytest.mark.parametrize("value", [-1, 2**64, 1.0, "3", True, None])
def test_seed_out_of_bounds_or_wrong_type_rejected(value: Any) -> None:
    data = pendulum_case()
    data["seed"] = value
    rejected(data, "seed")


@pytest.mark.req("TST-036")
@pytest.mark.parametrize("value", ["native", "emulated"])
def test_controller_mode_values(value: str) -> None:
    data = gantry_case()
    data["controller_mode"] = value
    assert accepted(data).controller_mode == value


@pytest.mark.req("TST-036")
def test_controller_mode_unknown_rejected() -> None:
    data = gantry_case()
    data["controller_mode"] = "hardware"
    rejected(data, "controller_mode")


@pytest.mark.req("TST-036")
def test_controller_mode_required_with_firmware_and_forbidden_without() -> None:
    data = gantry_case()
    del data["controller_mode"]
    cross_rejected(data, "controller_mode: the design has firmware")
    data = pendulum_case()
    data["controller_mode"] = "native"
    cross_rejected(data, "controller_mode: the design has no firmware")


@pytest.mark.req("TST-009")
@pytest.mark.parametrize("where", ["root", "design", "initial_state", "end", "criterion", "command", "fault"])
def test_unknown_key_rejected(where: str) -> None:
    data = with_faults(VALID_FAULTS["sensor_stuck"])
    targets = {
        "root": data,
        "design": data["design"],
        "initial_state": data["initial_state"],
        "end": data["end"],
        "criterion": data["criteria"][0],
        "command": data["commands"][0],
        "fault": data["faults"][0],
    }
    targets[where]["colour"] = "red"
    rejected(data, "colour", "Extra inputs are not permitted")


@pytest.mark.req("TST-010")
@pytest.mark.parametrize(("field", "value"), [("physics_step", "0.5 ms"), ("control_rate", "2 kHz")])
def test_rates_override_parsed_to_si(field: str, value: str) -> None:
    data = pendulum_case()
    data["rates"] = {field: value}
    rates = accepted(data).rates
    assert rates is not None
    assert getattr(rates, field) == pytest.approx({"physics_step": 0.0005, "control_rate": 2000.0}[field])


@pytest.mark.req("TST-010")
@pytest.mark.parametrize(
    ("rates", "fragment"),
    [
        ({}, "needs physics_step, control_rate or both"),
        ({"physics_step": "0 ms"}, "must be positive"),
        ({"physics_step": "-1 ms"}, "must be positive"),
        ({"control_rate": "0 Hz"}, "must be positive"),
        ({"physics_step": "1 Hz"}, "expected [time]"),
        ({"control_rate": "1 ms"}, "expected 1 / [time]"),
        ({"physics_step": "1"}, "rates.physics_step"),
        ({"physics_step": 0.001}, "must be a string with a unit"),
    ],
)
def test_rates_bounds_and_units_rejected(rates: dict[str, Any], fragment: str) -> None:
    data = pendulum_case()
    data["rates"] = rates
    rejected(data, fragment)


@pytest.mark.req("TST-001")
def test_design_name_and_version_must_match_the_design() -> None:
    data = pendulum_case()
    data["design"]["version"] = "0.2.0"
    cross_rejected(data, "written against 0.2.0, design is 0.1.0")
    case = accepted(pendulum_case())
    assert any("names design 'pendulum'" in error for error in cross_check(case, load_design(GANTRY)))


# ===== Open parameters =================================================


@pytest.mark.req("TST-009")
def test_release_design_case_may_not_contain_open_values() -> None:
    data = pendulum_case()
    data["criteria"][0]["target"] = "${period}"
    cross_rejected(data, "a case for a release design may not contain ${period}")


@pytest.mark.req("TST-009")
def test_release_design_case_may_not_declare_open_parameters() -> None:
    data = pendulum_case()
    data["open_parameters"] = [{"name": "x", "dimension": "[time]", "owed_by": "owner", "note": "n"}]
    cross_rejected(data, "may not have open parameters")


@pytest.mark.req("TST-009")
def test_release_design_case_may_not_sweep_an_open_parameter() -> None:
    data = pendulum_case()
    data["sweeps"] = [{"parameter": "${length}", "values": ["1 m"]}]
    cross_rejected(data, "may not contain ${length}")


@pytest.mark.req("TST-009")
def test_draft_case_open_value_must_be_declared() -> None:
    data = gantry_case()
    data["criteria"][0]["limit"] = "${tolerance}"
    cross_rejected(data, "undeclared open parameter ${tolerance}")


@pytest.mark.req("TST-009")
def test_case_open_parameter_must_be_used_and_not_shadow_the_design() -> None:
    data = gantry_case()
    data["open_parameters"].append({"name": "spare", "dimension": "[time]", "owed_by": "team", "note": "n"})
    cross_rejected(data, "'spare' is declared but never used")
    data = gantry_case()
    data["open_parameters"].append({"name": "workspace_x", "dimension": "[length]", "owed_by": "team", "note": "n"})
    cross_rejected(data, "'workspace_x' is already an open parameter of the design")


@pytest.mark.req("TST-009")
def test_duplicate_case_open_parameter_rejected() -> None:
    data = gantry_case()
    data["open_parameters"].append(copy.deepcopy(data["open_parameters"][0]))
    rejected(data, "duplicate open parameter 'start_x'")


@pytest.mark.req("TST-009", "TST-010")
def test_open_value_dimension_checked() -> None:
    data = gantry_case()
    data["commands"][1]["at"] = "${start_x}"
    cross_rejected(data, "'start_x' has dimension [length], expected [time]")
    data = gantry_case()
    data["criteria"][2]["limit"] = "${homing_time}"
    cross_rejected(data, "'homing_time' has dimension [time], expected [length]")
    data = gantry_case()
    data["commands"][1]["payload"]["x"] = "${homing_time}"
    cross_rejected(data, "'homing_time' has dimension [time], expected [length]")


# ===== Initial state ===================================================


@pytest.mark.req("TST-002")
def test_initial_state_may_be_empty() -> None:
    data = pendulum_case()
    data["initial_state"] = {}
    state = accepted(data).initial_state
    assert state.joints == []
    assert state.controller == {}
    assert state.payload is None


@pytest.mark.req("TST-002")
@pytest.mark.parametrize("field", ["joint", "position", "velocity"])
def test_joint_state_field_missing_rejected(field: str) -> None:
    data = pendulum_case()
    del data["initial_state"]["joints"][0][field]
    rejected(data, field, "Field required")


@pytest.mark.req("TST-002", "TST-010")
@pytest.mark.parametrize(
    ("position", "velocity", "motion"),
    [("0.05 rad", "0 rad/s", "angular"), ("3 deg", "1 deg/s", "angular"), ("2 cm", "0 m/s", "linear")],
)
def test_joint_state_units(position: str, velocity: str, motion: str) -> None:
    data = pendulum_case()
    data["initial_state"]["joints"][0].update(position=position, velocity=velocity)
    assert accepted(data).initial_state.joints[0].motion() == motion


@pytest.mark.req("TST-002", "TST-010")
@pytest.mark.parametrize(
    ("position", "velocity", "fragment"),
    [
        ("1 s", "0 rad/s", "must be a length or an angle"),
        ("0.05 rad", "0 m/s", "expected dimensionless / [time]"),
        ("1 m", "0 rad/s", "expected [length] / [time]"),
        ("rad", "0 rad/s", "must start with a number"),
        (0.05, "0 rad/s", "must be a string with a unit"),
        ("${start}", "1 kg", "must be a speed or an angular rate"),
    ],
)
def test_joint_state_bad_units_rejected(position: Any, velocity: Any, fragment: str) -> None:
    data = pendulum_case()
    data["initial_state"]["joints"][0].update(position=position, velocity=velocity)
    rejected(data, fragment)


@pytest.mark.req("TST-002")
def test_joint_state_duplicate_rejected() -> None:
    data = pendulum_case()
    data["initial_state"]["joints"].append(copy.deepcopy(data["initial_state"]["joints"][0]))
    rejected(data, "duplicate initial joint 'pivot'")


@pytest.mark.req("TST-002")
def test_joint_state_checked_against_design_joints() -> None:
    data = pendulum_case()
    data["initial_state"]["joints"][0]["joint"] = "hinge"
    cross_rejected(data, "unknown joint 'hinge'")
    data = pendulum_case()
    data["initial_state"]["joints"][0].update(position="1 cm", velocity="0 m/s")
    cross_rejected(data, "a revolute joint takes dimensionless values")
    data = gantry_case()
    data["initial_state"]["joints"][0].update(position="0.1 rad", velocity="0 rad/s")
    cross_rejected(data, "a prismatic joint takes [length] values")
    data = gantry_case()
    data["initial_state"]["joints"].append({"joint": "hook_swing", "position": "0 rad", "velocity": "0 rad/s"})
    cross_rejected(data, "initial state of a spherical joint is not supported")


@pytest.mark.req("TST-002")
@pytest.mark.parametrize("value", [True, False, "idle"])
def test_controller_assumption_values(value: Any) -> None:
    data = gantry_case()
    data["initial_state"]["controller"] = {"homed": value}
    assert accepted(data).initial_state.controller == {"homed": value}


@pytest.mark.req("TST-002")
@pytest.mark.parametrize("controller", [{"Homed": True}, {"homed": 1}, {"homed": "Idle"}, {"homed": None}])
def test_controller_assumption_bad_rejected(controller: dict[str, Any]) -> None:
    data = gantry_case()
    data["initial_state"]["controller"] = controller
    rejected(data, "initial_state.controller")


@pytest.mark.req("TST-002")
def test_controller_assumption_needs_firmware() -> None:
    data = pendulum_case()
    data["initial_state"]["controller"] = {"homed": True}
    cross_rejected(data, "the design has no controller")


@pytest.mark.req("TST-002", "TST-010")
def test_attached_payload() -> None:
    data = gantry_case()
    data["initial_state"]["payload"] = {"attached": True, "body": "hook", "mass": "3 kg"}
    payload = accepted(data).initial_state.payload
    assert payload is not None
    assert payload.mass == pytest.approx(3.0)
    assert check_errors(data) == []


@pytest.mark.req("TST-002")
@pytest.mark.parametrize(
    ("payload", "fragment"),
    [
        ({"attached": True, "body": "hook"}, "needs body and mass"),
        ({"attached": True, "mass": "3 kg"}, "needs body and mass"),
        ({"attached": True, "body": "hook", "mass": "0 kg"}, "must be positive"),
        ({"attached": True, "body": "hook", "mass": "-1 kg"}, "must be positive"),
        ({"attached": True, "body": "hook", "mass": "3 m"}, "expected [mass]"),
        ({"attached": True, "body": "hook", "mass": "3"}, "payload.mass"),
        ({"attached": False, "mass": "3 kg"}, "takes no body or mass"),
        ({"attached": False, "body": "hook"}, "takes no body or mass"),
        ({"attached": "no"}, "payload.attached"),
        ({}, "payload.attached"),
    ],
)
def test_bad_payload_rejected(payload: dict[str, Any], fragment: str) -> None:
    data = gantry_case()
    data["initial_state"]["payload"] = payload
    rejected(data, fragment)


@pytest.mark.req("TST-002")
def test_payload_body_must_exist() -> None:
    data = gantry_case()
    data["initial_state"]["payload"] = {"attached": True, "body": "crane", "mass": "3 kg"}
    cross_rejected(data, "unknown body 'crane'")


# ===== Commands ========================================================


@pytest.mark.req("TST-003")
def test_command_timeline_may_be_empty() -> None:
    data = pendulum_case()
    data["commands"] = []
    assert accepted(data).commands == []


@pytest.mark.req("TST-003")
@pytest.mark.parametrize("field", ["at", "message"])
def test_command_field_missing_rejected(field: str) -> None:
    data = gantry_case()
    del data["commands"][0][field]
    rejected(data, field, "Field required")


@pytest.mark.req("TST-003")
def test_equal_command_times_accepted_and_decreasing_rejected() -> None:
    data = with_faults()
    data["commands"][1]["at"] = "0 s"
    assert [command.at for command in accepted(data).commands] == [0.0, 0.0]
    data["commands"] = [data["commands"][1], {"at": "1 ms", "message": "home"}, {"at": "0.5 ms", "message": "home"}]
    data["commands"][0]["at"] = "0 s"
    rejected(data, "commands[2].at: 0.0005 s is before the previous command at 0.001 s")


@pytest.mark.req("TST-003")
@pytest.mark.parametrize(("value", "fragment"), [("-1 s", "must not be negative"), ("1 m", "expected [time]")])
def test_bad_command_time_rejected(value: str, fragment: str) -> None:
    data = gantry_case()
    data["commands"][0]["at"] = value
    rejected(data, fragment)


@pytest.mark.req("TST-003")
def test_command_after_max_duration_rejected() -> None:
    data = with_faults()
    data["commands"][1]["at"] = "11 s"
    rejected(data, "commands[1].at: 11.0 s is after end.max_duration 10.0 s")
    data["commands"][1]["at"] = "10 s"
    assert len(accepted(data).commands) == 2


@pytest.mark.req("TST-003")
def test_command_checked_against_design_messages() -> None:
    data = gantry_case()
    data["commands"][0]["message"] = "jump"
    cross_rejected(data, "unknown operator message 'jump'")
    data = gantry_case()
    data["commands"][0]["message"] = "ready_fault"
    cross_rejected(data, "'ready_fault' is sent by controller, not the operator")
    data = gantry_case()
    del data["commands"][1]["payload"]["y"]
    cross_rejected(data, "'target' takes fields ['x', 'y'], got ['x']")
    data = gantry_case()
    data["commands"][1]["payload"]["x"] = "40 s"
    cross_rejected(data, "payload.x '40 s' has dimension [time], expected [length]")


@pytest.mark.req("TST-003", "TST-010")
def test_command_payload_values_parsed_to_si() -> None:
    data = gantry_case()
    data["criteria"] = data["criteria"][2:]
    payload = accepted(data).commands[1].payload
    assert isinstance(payload["x"], Quantity)
    assert payload["x"].value == pytest.approx(0.4)
    assert check_errors(data) == []


# ===== Faults ==========================================================


@pytest.mark.req("TST-004")
def test_fault_timeline_may_be_empty() -> None:
    assert accepted(gantry_case()).faults == []


@pytest.mark.req("TST-004")
@pytest.mark.parametrize("kind", sorted(VALID_FAULTS))
def test_every_fault_kind_accepted_and_cross_checked(kind: str) -> None:
    data = with_faults(VALID_FAULTS[kind])
    assert accepted(data).faults[0].kind == kind
    assert check_errors(data) == []


@pytest.mark.req("TST-004", "TST-037")
def test_forced_level_fault_parsed() -> None:
    """Parsing only: the timeline can state a forced-level fault; routing it is the runner's (stub kept)."""
    forced = accepted(with_faults(VALID_FAULTS["sensor_forced"])).faults[0]
    assert isinstance(forced, SensorForced)
    assert forced.level == "high"
    assert forced.start == pytest.approx(1.0)
    assert forced.duration == pytest.approx(2.0)


@pytest.mark.req("TST-004")
@pytest.mark.parametrize(
    ("kind", "field"),
    [
        ("sensor_stuck", "target"),
        ("sensor_noisy", "standard_deviation"),
        ("sensor_forced", "level"),
        ("slip", "fraction"),
        ("motor_stall", "available_torque"),
        ("power_loss", "rail"),
        ("estop_press", "target"),
        ("payload_change", "mass"),
        ("payload_change", "target"),
        ("controller_freeze", "start"),
    ],
)
def test_fault_missing_kind_specific_field_rejected(kind: str, field: str) -> None:
    item = copy.deepcopy(VALID_FAULTS[kind])
    del item[field]
    rejected(with_faults(item), kind, field, "Field required")


@pytest.mark.req("TST-004")
def test_fault_kind_missing_or_unknown_rejected() -> None:
    item = copy.deepcopy(VALID_FAULTS["sensor_stuck"])
    del item["kind"]
    rejected(with_faults(item), "kind")
    item["kind"] = "meteor"
    rejected(with_faults(item), "meteor")


@pytest.mark.req("TST-004")
@pytest.mark.parametrize(
    ("window_fields", "fragment"),
    [
        ({"start": "1 s"}, "exactly one of end and duration"),
        ({"start": "1 s", "end": "2 s", "duration": "1 s"}, "exactly one of end and duration"),
        ({"start": "-1 s", "duration": "1 s"}, "must not be negative"),
        ({"start": "1 s", "duration": "0 s"}, "must be positive"),
        ({"start": "1 s", "end": "1 s"}, "must be after its start"),
        ({"start": "1 s", "end": "0.5 s"}, "must be after its start"),
        ({"start": "1 m", "duration": "1 s"}, "expected [time]"),
        ({"start": "1", "duration": "1 s"}, "start"),
    ],
)
def test_bad_fault_window_rejected(window_fields: dict[str, str], fragment: str) -> None:
    item = {"kind": "controller_freeze", **window_fields}
    rejected(with_faults(item), fragment)


@pytest.mark.req("TST-004")
def test_fault_window_with_end_accepted_at_zero() -> None:
    freeze = accepted(with_faults({"kind": "controller_freeze", "start": "0 s", "end": "1 ms"})).faults[0]
    assert isinstance(freeze, ControllerFreeze)
    assert freeze.end == pytest.approx(0.001)


@pytest.mark.req("TST-004")
def test_fault_starting_at_or_after_max_duration_rejected() -> None:
    rejected(with_faults(fault("controller_freeze", start="10 s")), "is not before end.max_duration")
    assert len(accepted(with_faults(fault("controller_freeze", start="9.999 s"))).faults) == 1


@pytest.mark.req("TST-004")
@pytest.mark.parametrize(
    ("kind", "field", "value", "fragment"),
    [
        ("slip", "fraction", "0", "above 0 and at most 1"),
        ("slip", "fraction", "1.01", "above 0 and at most 1"),
        ("slip", "fraction", "-10 percent", "above 0 and at most 1"),
        ("slip", "fraction", "1 m", "expected dimensionless"),
        ("motor_stall", "available_torque", "1", "at least 0 and below 1"),
        ("motor_stall", "available_torque", "-0.1", "at least 0 and below 1"),
        ("sensor_noisy", "standard_deviation", "0 m", "must be positive"),
        ("sensor_noisy", "standard_deviation", "-1 m", "must be positive"),
        ("sensor_noisy", "standard_deviation", 1.0, "must be a string with a unit"),
        ("sensor_forced", "level", "medium", "level"),
        ("sensor_forced", "level", True, "level"),
        ("payload_change", "mass", "-1 kg", "must not be negative"),
        ("payload_change", "mass", "1 m", "expected [mass]"),
    ],
)
def test_fault_parameter_bounds_rejected(kind: str, field: str, value: Any, fragment: str) -> None:
    item = copy.deepcopy(VALID_FAULTS[kind])
    item[field] = value
    rejected(with_faults(item), fragment)


@pytest.mark.req("TST-004")
@pytest.mark.parametrize(
    ("kind", "field", "value"),
    [
        ("slip", "fraction", "1"),
        ("slip", "fraction", "0.1 percent"),
        ("motor_stall", "available_torque", "0"),
        ("motor_stall", "available_torque", "99 percent"),
        ("payload_change", "mass", "0 kg"),
    ],
)
def test_fault_parameter_bounds_accepted(kind: str, field: str, value: str) -> None:
    item = copy.deepcopy(VALID_FAULTS[kind])
    item[field] = value
    assert len(accepted(with_faults(item)).faults) == 1


@pytest.mark.req("TST-004")
def test_payload_change_is_instantaneous() -> None:
    item = copy.deepcopy(VALID_FAULTS["payload_change"])
    change = accepted(with_faults(item)).faults[0]
    assert isinstance(change, PayloadChange)
    assert change.mass == pytest.approx(2.0)
    item["duration"] = "1 s"
    rejected(with_faults(item), "duration", "Extra inputs are not permitted")


@pytest.mark.req("TST-004")
@pytest.mark.parametrize(
    ("kind", "field", "value", "fragment"),
    [
        ("sensor_stuck", "target", "z_switch", "unknown component instance 'z_switch'"),
        ("slip", "target", "z_belt", "unknown component instance 'z_belt'"),
        ("power_loss", "rail", "mains", "unknown power rail 'mains'"),
        ("payload_change", "target", "crane", "unknown body 'crane'"),
    ],
)
def test_fault_targets_checked_against_design(kind: str, field: str, value: str, fragment: str) -> None:
    item = copy.deepcopy(VALID_FAULTS[kind])
    item[field] = value
    cross_rejected(with_faults(item), fragment)


@pytest.mark.req("TST-004")
def test_controller_freeze_needs_firmware() -> None:
    data = pendulum_case()
    data["faults"] = [fault("controller_freeze")]
    cross_rejected(data, "controller_freeze needs a design with firmware")


# ===== End condition ===================================================


@pytest.mark.req("TST-009")
@pytest.mark.parametrize(
    "trigger",
    [{"at": "5 s"}, {"message": "ready_fault"}, {"controller_state": "ready"}],
    ids=["at", "message", "state"],
)
def test_end_condition_forms(trigger: dict[str, str]) -> None:
    data = with_faults()
    data["end"] = {**trigger, "max_duration": "10 s"}
    assert accepted(data).end.max_duration == pytest.approx(10.0)
    assert check_errors(data) == []


@pytest.mark.req("TST-009")
@pytest.mark.parametrize(
    ("end", "fragment"),
    [
        ({"at": "5 s"}, "max_duration"),
        ({"max_duration": "10 s"}, "exactly one of at, message and controller_state"),
        ({"at": "5 s", "message": "ready_fault", "max_duration": "10 s"}, "exactly one of"),
        ({"at": "11 s", "max_duration": "10 s"}, "exceeds max_duration"),
        ({"at": "0 s", "max_duration": "10 s"}, "end time must be positive"),
        ({"at": "5 s", "max_duration": "0 s"}, "max_duration must be positive"),
        ({"at": "5 s", "max_duration": "10 m"}, "expected [time]"),
        ({"message": "Ready", "max_duration": "10 s"}, "end.message"),
    ],
)
def test_bad_end_condition_rejected(end: dict[str, str], fragment: str) -> None:
    data = with_faults()
    data["end"] = end
    rejected(data, fragment)


@pytest.mark.req("TST-009")
def test_end_condition_at_equal_to_max_duration_accepted() -> None:
    data = with_faults()
    data["end"] = {"at": "10 s", "max_duration": "10 s"}
    assert accepted(data).end.at == pytest.approx(10.0)


@pytest.mark.req("TST-009")
def test_end_message_checked_against_design() -> None:
    data = with_faults()
    data["end"] = {"message": "done", "max_duration": "10 s"}
    cross_rejected(data, "end.message: unknown operator message 'done'")


# ===== Criteria ========================================================


def value_criterion(**fields: Any) -> dict[str, Any]:
    base = {"name": "c", "check": "value", "quantity": "pose:hook.x", "statistic": "maximum", "comparison": "at_most"}
    base["limit"] = "1 m"
    base.update(fields)
    return {key: item for key, item in base.items() if item is not None}


def with_criterion(item: dict[str, Any]) -> dict[str, Any]:
    data = with_faults()
    data["criteria"].append(item)
    return data


@pytest.mark.req("TST-005")
def test_criteria_may_not_be_empty() -> None:
    data = pendulum_case()
    data["criteria"] = []
    rejected(data, "criteria", "at least 1")


@pytest.mark.req("TST-005")
def test_duplicate_criterion_names_rejected() -> None:
    data = gantry_case()
    data["criteria"][1]["name"] = "final_x"
    rejected(data, "duplicate criterion 'final_x'")


@pytest.mark.req("TST-005")
@pytest.mark.parametrize("field", ["name", "check", "quantity", "statistic", "comparison"])
def test_value_criterion_field_missing_rejected(field: str) -> None:
    item = value_criterion()
    del item[field]
    rejected(with_criterion(item), field)


@pytest.mark.req("TST-005")
@pytest.mark.parametrize(
    "fields",
    [
        {"comparison": "below", "limit": "1 m"},
        {"comparison": "above", "limit": "-1 m"},
        {"comparison": "at_least", "limit": "-1 m"},
        {"comparison": "within", "target": "1 m", "limit": "1 cm"},
        {"comparison": "within", "target": "1 m", "limit": None, "relative_limit": "1 percent"},
        {"quantity": "pose:hook.yaw", "limit": "5 deg"},
        {"quantity": "joint:x_axis.velocity", "limit": "1 m/s"},
        {"quantity": "signal:x_encoder.channel_a", "limit": "1"},
        {"quantity": "count:message.ready_fault", "statistic": "final", "comparison": "at_most", "limit": "1"},
        {"quantity": "state:ready", "statistic": "final", "limit": "1"},
        {"quantity": "joint:x_axis.position", "statistic": "period", "limit": "2 s"},
        {"statistic": "peak_absolute", "window": {"start": "1 s", "end": "2 s"}},
        {"statistic": "minimum", "window": {"end": "2 s"}},
    ],
)
def test_value_criterion_forms_accepted(fields: dict[str, Any]) -> None:
    data = with_criterion(value_criterion(**fields))
    assert accepted(data).criteria[-1].name == "c"
    assert check_errors(data) == []


@pytest.mark.req("TST-005")
@pytest.mark.parametrize(
    ("fields", "fragment"),
    [
        ({"comparison": "within", "limit": "1 cm"}, "needs a target"),
        ({"comparison": "within", "target": "1 m", "limit": None}, "exactly one of limit and relative_limit"),
        (
            {"comparison": "within", "target": "1 m", "relative_limit": "1 percent"},
            "exactly one of limit and relative_limit",
        ),
        ({"comparison": "within", "target": "1 m", "limit": "0 m"}, "must be positive"),
        ({"comparison": "within", "target": "1 m", "limit": None, "relative_limit": "0"}, "must be positive"),
        ({"limit": None}, "needs a limit"),
        ({"target": "1 m"}, "takes no target or relative_limit"),
        ({"relative_limit": "1 percent"}, "takes no target or relative_limit"),
        ({"limit": "1 s"}, "expected [length]"),
        ({"limit": "1"}, "expected [length]"),
        ({"comparison": "within", "target": "1 kg", "limit": "1 cm"}, "target '1 kg' has dimension [mass]"),
        ({"quantity": "pose:hook.roll", "limit": "1 m"}, "expected dimensionless"),
        ({"statistic": "period", "limit": "1 m"}, "expected [time]"),
        ({"quantity": "state:ready", "statistic": "period", "limit": "1 s"}, "statistic 'period' needs a series"),
        (
            {"quantity": "signal:x_encoder.channel_a", "comparison": "within", "target": "1 V", "limit": "1 A"},
            "limit '1 A' has dimension [current]",
        ),
        ({"comparison": "=="}, "comparison"),
        ({"comparison": "<="}, "comparison"),
        ({"statistic": "mean"}, "statistic"),
        ({"quantity": "pose:hook.w"}, "quantity"),
        ({"quantity": "hook.x"}, "quantity"),
        ({"quantity": "joint:x_axis.force"}, "quantity"),
        ({"limit": 1.0}, "must be a string with a unit"),
        ({"limit": "m"}, "must start with a number"),
        ({"name": "Final"}, "name"),
    ],
)
def test_value_criterion_bad_rejected(fields: dict[str, Any], fragment: str) -> None:
    rejected(with_criterion(value_criterion(**fields)), fragment)


@pytest.mark.req("TST-005")
@pytest.mark.parametrize(
    ("fields", "fragment"),
    [
        ({"quantity": "pose:crane.x"}, "unknown body 'crane'"),
        ({"quantity": "joint:z_axis.position"}, "unknown joint 'z_axis'"),
        ({"quantity": "signal:x_encoder.index"}, "unknown signal 'x_encoder.index'"),
        ({"quantity": "count:message.done", "limit": "1"}, "unknown operator message 'done'"),
        ({"quantity": "joint:x_axis.position", "limit": "1 rad"}, "expected [length]"),
        ({"quantity": "joint:x_axis.velocity", "limit": "1 m"}, "expected [length] / [time]"),
    ],
)
def test_value_criterion_checked_against_design(fields: dict[str, Any], fragment: str) -> None:
    cross_rejected(with_criterion(value_criterion(**fields)), fragment)


@pytest.mark.req("TST-005")
def test_pendulum_criterion_joint_dimension_checked() -> None:
    data = pendulum_case()
    data["criteria"].append(value_criterion(quantity="joint:pivot.position", limit="1 m"))
    data["criteria"][-1]["quantity"] = "joint:pivot.position"
    cross_rejected(data, "expected dimensionless")


@pytest.mark.req("TST-005")
@pytest.mark.parametrize(
    "item",
    [
        {
            "check": "interval",
            "start_event": "message:home",
            "end_event": "state:ready",
            "comparison": "below",
            "limit": "2 s",
        },
        {"check": "event", "event": "message:ready_fault", "expect": "present"},
        {"check": "event", "event": "state:ready", "expect": "absent", "window": {"start": "1 s"}},
        {"check": "no_motion", "actuators": ["x_motor", "y_motor"], "threshold": "1 mm"},
        {"check": "no_overlap", "actuators": ["x_motor"]},
        {
            "check": "settle_time",
            "quantity": "pose:hook.roll",
            "band": "1 deg",
            "comparison": "at_most",
            "limit": "3 s",
        },
        {
            "check": "settle_time",
            "quantity": "pose:hook.x",
            "band": "1 cm",
            "target": "20 cm",
            "comparison": "below",
            "limit": "3 s",
        },
    ],
    ids=["interval", "present", "absent", "no_motion", "no_overlap", "settle", "settle_target"],
)
def test_other_criterion_checks_accepted(item: dict[str, Any]) -> None:
    data = with_criterion({"name": "c", **item})
    assert accepted(data).criteria[-1].limit_text() != ""
    assert check_errors(data) == []


@pytest.mark.req("TST-005")
@pytest.mark.parametrize(
    ("item", "fragment"),
    [
        (
            {"check": "interval", "start_event": "message:home", "end_event": "state:ready", "comparison": "below"},
            "limit",
        ),
        (
            {
                "check": "interval",
                "start_event": "home",
                "end_event": "state:ready",
                "comparison": "below",
                "limit": "2 s",
            },
            "start_event",
        ),
        (
            {"check": "interval", "start_event": "message:home", "end_event": "state:ready", "comparison": "within"},
            "comparison",
        ),
        ({"check": "event", "event": "message:ready_fault"}, "expect"),
        ({"check": "event", "event": "message:ready_fault", "expect": "maybe"}, "expect"),
        ({"check": "event", "event": "message:ready_fault", "expect": "present", "window": {}}, "start, end or both"),
        ({"check": "no_motion", "actuators": [], "threshold": "1 mm"}, "at least 1"),
        ({"check": "no_motion", "actuators": ["x_motor", "x_motor"], "threshold": "1 mm"}, "duplicate actuator"),
        ({"check": "no_motion", "actuators": ["x_motor"], "threshold": "0 mm"}, "threshold must be positive"),
        ({"check": "no_motion", "actuators": ["x_motor"]}, "threshold"),
        ({"check": "no_overlap", "actuators": []}, "at least 1"),
        (
            {"check": "settle_time", "quantity": "pose:hook.x", "band": "0 cm", "comparison": "below", "limit": "3 s"},
            "band",
        ),
        (
            {"check": "settle_time", "quantity": "pose:hook.x", "band": "1 s", "comparison": "below", "limit": "3 s"},
            "band",
        ),
        (
            {"check": "settle_time", "quantity": "pose:hook.x", "band": "1 cm", "comparison": "below", "limit": "3 m"},
            "limit",
        ),
        ({"check": "teleport"}, "teleport"),
        ({}, "check"),
    ],
)
def test_other_criterion_checks_rejected(item: dict[str, Any], fragment: str) -> None:
    rejected(with_criterion({"name": "c", **item}), fragment)


@pytest.mark.req("TST-005")
@pytest.mark.parametrize(
    ("item", "fragment"),
    [
        ({"check": "event", "event": "message:done", "expect": "present"}, "unknown operator message 'done'"),
        ({"check": "no_motion", "actuators": ["z_motor"], "threshold": "1 mm"}, "unknown component instance 'z_motor'"),
        (
            {
                "check": "interval",
                "start_event": "message:x",
                "end_event": "state:ready",
                "comparison": "below",
                "limit": "1 s",
            },
            "unknown operator message 'x'",
        ),
    ],
)
def test_other_criterion_checks_against_design(item: dict[str, Any], fragment: str) -> None:
    cross_rejected(with_criterion({"name": "c", **item}), fragment)


@pytest.mark.req("TST-005")
def test_controller_state_reference_needs_firmware() -> None:
    data = pendulum_case()
    data["criteria"].append({"name": "c", "check": "event", "event": "state:ready", "expect": "present"})
    cross_rejected(data, "controller state 'ready' needs a design with firmware")


@pytest.mark.req("TST-005")
@pytest.mark.parametrize(
    ("window_fields", "fragment"),
    [
        ({"start": "-1 s"}, "must not be negative"),
        ({"end": "0 s"}, "must be positive"),
        ({"start": "2 s", "end": "2 s"}, "must be after its start"),
        ({"start": "2 s", "end": "1 s"}, "must be after its start"),
        ({"start": "2 m"}, "expected [time]"),
    ],
)
def test_bad_criterion_window_rejected(window_fields: dict[str, str], fragment: str) -> None:
    rejected(with_criterion(value_criterion(window=window_fields)), fragment)


# ===== Sweeps, variations, baseline ====================================


@pytest.mark.req("TST-007")
def test_sweeps_and_variations_stored() -> None:
    data = gantry_case()
    data["sweeps"] = [
        {"parameter": "components.x_motor.part", "values": ["nema17_class_generic", "nema23_class_generic"]},
        {"parameter": "${rated_load}", "values": ["1 kg", "3 kg"]},
    ]
    data["variations"] = {
        "count": 5,
        "parameters": [
            {"parameter": "joints.x_axis.linear", "distribution": "normal", "spread": "1 N"},
            {"parameter": "${axis_damping}", "distribution": "uniform", "spread": "0.1 N*s/m"},
        ],
    }
    case = accepted(data)
    assert len(case.sweeps) == 2
    assert case.variations is not None
    assert case.variations.count == 5
    assert check_errors(data) == []


@pytest.mark.req("TST-007")
@pytest.mark.parametrize(
    ("sweeps", "variations", "fragment"),
    [
        ([{"parameter": "components.x_motor.part", "values": []}], None, "at least 1"),
        ([{"parameter": "components.x_motor.part", "values": ["a", "a"]}], None, "duplicate sweep value"),
        ([{"parameter": "components.x_motor.part", "values": [""]}], None, "at least 1 character"),
        ([{"parameter": "x_motor.part", "values": ["a"]}], None, "parameter"),
        ([{"parameter": "components.x_motor", "values": ["a"]}], None, "parameter"),
        ([{"parameter": "${a}", "values": ["1"]}, {"parameter": "${a}", "values": ["2"]}], None, "duplicate sweep"),
        ([], {"count": 0, "parameters": [{"parameter": "${a}", "distribution": "normal", "spread": "1"}]}, "count"),
        ([], {"count": 1, "parameters": []}, "at least 1"),
        (
            [],
            {"count": 1, "parameters": [{"parameter": "${a}", "distribution": "beta", "spread": "1"}]},
            "distribution",
        ),
        ([], {"count": 1, "parameters": [{"parameter": "${a}", "distribution": "normal", "spread": "0"}]}, "positive"),
        ([], {"count": True, "parameters": [{"parameter": "${a}", "distribution": "normal", "spread": "1"}]}, "count"),
        (
            [],
            {
                "count": 2,
                "parameters": [
                    {"parameter": "${a}", "distribution": "normal", "spread": "1"},
                    {"parameter": "${a}", "distribution": "uniform", "spread": "1"},
                ],
            },
            "duplicate varied parameter",
        ),
    ],
)
def test_bad_sweeps_and_variations_rejected(sweeps: list[Any], variations: Any, fragment: str) -> None:
    data = gantry_case()
    data["sweeps"] = sweeps
    if variations is not None:
        data["variations"] = variations
    rejected(data, fragment)


@pytest.mark.req("TST-007")
@pytest.mark.parametrize(
    ("parameter", "fragment"),
    [
        ("components.z_motor.part", "components has no item 'z_motor'"),
        ("components.x_motor.colour", "does not name a design field"),
        ("${unknown}", "undeclared open parameter ${unknown}"),
    ],
)
def test_sweep_parameter_resolved_against_design(parameter: str, fragment: str) -> None:
    data = gantry_case()
    data["sweeps"] = [{"parameter": parameter, "values": ["a"]}]
    cross_rejected(data, fragment)


@pytest.mark.req("TST-009")
def test_missing_baseline_file_rejected(tmp_path: Path) -> None:
    data = pendulum_case()
    data["baseline"] = "baseline.yaml"
    path = tmp_path / "P1.yaml"
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    path.write_text(stream.getvalue(), encoding="utf-8")
    errors = validate_test_case_file(path)
    assert errors == [f"{path}: baseline: missing baseline file 'baseline.yaml'"]
    data["baseline"] = "../baseline.yaml"
    rejected(data, "may not leave its folder")


# ===== Files ===========================================================


def write_case(tmp_path: Path, name: str, data: dict[str, Any]) -> Path:
    path = tmp_path / name
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    path.write_text(stream.getvalue(), encoding="utf-8")
    return path


@pytest.mark.req("TST-009")
def test_file_errors_name_the_file_and_field(tmp_path: Path) -> None:
    data = pendulum_case()
    data["seed"] = -1
    path = write_case(tmp_path, "P1.yaml", data)
    errors = validate_test_case_file(path)
    assert len(errors) == 1
    assert errors[0].startswith(f"{path}: seed:")
    with pytest.raises(CaseError, match="seed"):
        load_test_case(path)


@pytest.mark.req("TST-009")
@pytest.mark.parametrize(
    ("name", "valid"), [("P1.yaml", True), ("P1_x.yaml", True), ("P12.yaml", False), ("x.yaml", False)]
)
def test_file_name_starts_with_case_id(tmp_path: Path, name: str, valid: bool) -> None:
    errors = validate_test_case_file(write_case(tmp_path, name, pendulum_case()))
    assert (errors == []) == valid


@pytest.mark.req("TST-009")
def test_unknown_design_and_bad_yaml_reported(tmp_path: Path) -> None:
    data = pendulum_case()
    data["design"]["name"] = "crane"
    errors = validate_test_case_file(write_case(tmp_path, "P1.yaml", data))
    assert "design.name: no design file" in errors[0]
    bad = tmp_path / "P2.yaml"
    bad.write_text("id: [", encoding="utf-8")
    assert "YAML parse error" in validate_test_case_file(bad)[0]


@pytest.mark.req("TST-010")
@pytest.mark.parametrize(("text", "value", "dimension"), [("3", 3.0, "dimensionless"), ("40 cm", 0.4, "[length]")])
def test_any_dimension_quantity(text: str, value: float, dimension: str) -> None:
    quantity = parse_any_quantity(text)
    assert quantity.value == pytest.approx(value)
    assert quantity.dimension == dimension
    assert str(quantity) == text


@pytest.mark.req("TST-010")
@pytest.mark.parametrize("text", ["", "cm", "1 furlongz", "inf m", None, True, 3])
def test_any_dimension_quantity_rejected(text: Any) -> None:
    with pytest.raises(ValueError, match="quantity"):
        parse_any_quantity(text)
