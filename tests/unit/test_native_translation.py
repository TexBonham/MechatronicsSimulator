"""Neutral model to the own engine's model document (PHY-018, PHY-019, PHY-020) and the adapter selection."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

import pytest

from mechsim.builder import structure
from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.native.translate import (
    SUPPORTED_ELEMENTS,
    StartStateError,
    UnmappableElementError,
    element_kinds,
    translate,
)
from mechsim.physics.neutral import PhysicsModel, from_yaml

ROOT = Path(__file__).resolve().parents[2]
ORIGIN = ("0 m", "0 m", "0 m")
PENDULUM = (ROOT / "schemas/examples/physics_model.pendulum.yaml").read_text()


def pendulum_data() -> dict[str, Any]:
    data = from_yaml(PENDULUM).model_dump(mode="json")
    data["bodies"] = list(data["bodies"])
    return data


def model_of(data: dict[str, Any]) -> PhysicsModel:
    return PhysicsModel.model_validate(data)


def slider(name: str, parent: str = "world", axis: tuple[float, float, float] = (1.0, 0.0, 0.0)) -> dict[str, Any]:
    return {
        "name": name,
        "mass": "2 kg",
        "com": ("0 m", "0 m", "0 m"),
        "inertia": {"ixx": "0.01 kg*m**2", "iyy": "0.01 kg*m**2", "izz": "0.01 kg*m**2"},
        "material": "steel",
        "parent": parent,
        "joint": {"name": name + "_joint", "type": "prismatic", "position": ORIGIN, "axis": axis},
    }


def free_body(name: str) -> dict[str, Any]:
    body = slider(name)
    body["joint"] = {
        "name": name + "_joint",
        "type": "free",
        "position": ("0 m", "0 m", "-0.3 m"),
        "linear": [{"damping": f"{d} N*s/m"} for d in (1, 2, 3)],
        "angular": [{"damping": f"{d} N*m*s/rad"} for d in (4, 5, 6)],
    }
    return body


@pytest.mark.req("PHY-019")
def test_supported_elements_mirror_the_cpp_list() -> None:
    source = (ROOT / "core/physics/native/src/native_adapter.cpp").read_text()
    block = source.split("// ===== Supported elements")[1].split("// ===== End supported elements")[0]
    assert tuple(re.findall(r'"([a-z_.]+)"', block)) == SUPPORTED_ELEMENTS


@pytest.mark.req("PHY-001", "PHY-018", "PHY-020")
def test_pendulum_translates_in_neutral_order_with_the_engine_config() -> None:
    model = from_yaml(PENDULUM)
    translation = translate(model)
    doc = json.loads(translation.text)
    assert doc["format"] == "mechsim-native-1"
    assert doc["elements"] == list(element_kinds(model))
    assert doc["neutral_bodies"] == 1
    link = doc["bodies"][0]
    assert link["name"] == "link"
    assert link["parent"] == -1
    assert link["joint"]["type"] == "revolute"
    assert link["joint"]["damping"] == [pytest.approx(0.04520785330006282)]
    assert link["joint"]["friction"] == [0.0]
    assert link["joint"]["lower"] == [None]
    assert link["inertia"][4] == pytest.approx(0.020833333333333332)
    assert doc["gravity"] == [0.0, -9.81, 0.0]
    assert doc["config"]["iteration_cap"] == 50
    assert doc["config"]["drift_threshold_m"] == 1e-6
    assert doc["contact"] is None
    assert translation.report[0].startswith("native adapter: own engine, backward Euler")
    assert translate(model).text == translation.text


@pytest.mark.req("PHY-018")
def test_free_joint_dofs_are_in_engine_order_and_world_constraints_get_an_anchor() -> None:
    data = pendulum_data()
    data["bodies"].append(free_body("ball"))
    data["constraints"] = [
        {
            "type": "point",
            "name": "tie",
            "body_a": "ball",
            "body_b": "world",
            "anchor_a": ORIGIN,
            "anchor_b": ("0 m", "0 m", "-0.3 m"),
        }
    ]
    doc = translate(model_of(data)).document
    assert doc["bodies"][1]["joint"]["damping"] == [4.0, 5.0, 6.0, 1.0, 2.0, 3.0]
    assert doc["bodies"][1]["joint"]["friction"] == []
    assert doc["neutral_bodies"] == 2
    assert doc["bodies"][2]["name"] == "world.anchor"
    assert doc["constraints"][0]["body_b"] == 2
    assert doc["constraints"][0]["point_b"] == [0.0, 0.0, -0.3]


@pytest.mark.req("PHY-019")
def test_friction_on_a_ball_joint_and_an_unsupported_member_are_refused() -> None:
    data = pendulum_data()
    ball = free_body("ball")
    ball["joint"]["linear"] = [{"friction": "1 N"}, {}, {}]
    data["bodies"].append(ball)
    with pytest.raises(UnmappableElementError, match="ball_joint"):
        translate(model_of(data))
    data = pendulum_data()
    section = {
        "elastic_modulus": "2e11 Pa",
        "second_moment_of_area": "1e-8 m**4",
        "density": "7800 kg/m**3",
        "area": "1e-4 m**2",
        "length": "1 m",
    }
    data["lumped_members"] = [
        {"name": "beam", "parent": "link", "position": ORIGIN, "section": section, "root": "clamped", "end": "roller"}
    ]
    with pytest.raises(UnmappableElementError, match="beam"):
        translate(model_of(data))
    data["lumped_members"][0]["end"] = "free"
    doc = translate(model_of(data)).document
    assert doc["members"][0]["ends"] == "cantilever"
    assert doc["members"][0]["segments"] == 8


@pytest.mark.req("PHY-006", "PHY-018")
def test_transmission_mapping_and_its_report() -> None:
    data = pendulum_data()
    shaft = slider("motor")
    shaft["joint"] = {"name": "motor_joint", "type": "revolute", "position": ORIGIN, "axis": (0, 0, 1)}
    data["bodies"] += [shaft, slider("carriage")]
    data["transmissions"] = [
        {
            "type": "gear_pair",
            "name": "gear",
            "shaft_joint": "motor_joint",
            "driven_joint": "carriage_joint",
            "efficiency": 0.9,
            "ratio": -3.0,
            "stiffness": "1e4 N*m/rad",
            "backlash": "0.01 rad",
        }
    ]
    with pytest.raises(ValueError, match="must be revolute"):
        model_of(data)
    carriage = data["bodies"][2]
    carriage["joint"] = {"name": "carriage_joint", "type": "revolute", "position": ORIGIN, "axis": (0, 0, 1)}
    doc = translate(model_of(data)).document
    gear = doc["transmissions"][0]
    assert (gear["input_body"], gear["output_body"], gear["ratio"], gear["reverses"]) == (1, 2, 3.0, True)
    assert (gear["efficiency"], gear["back_efficiency"], gear["backlash"]) == (0.9, 0.9, 0.01)
    report = translate(model_of(data)).report
    assert any(line.startswith("transmission 'gear': compliant") for line in report)
    assert any(line.startswith("joint drive on 'motor_joint'") for line in report)


@pytest.mark.req("PHY-006")
def test_starts_and_payloads() -> None:
    data = pendulum_data()
    data["bodies"] += [slider("carrier"), free_body("load")]
    model = model_of(data)
    doc = translate(model, payloads={"load": "carrier"}, starts={"carrier_joint": 0.2, "pivot": 0.1}).document
    assert doc["payloads"] == [{"body": 2, "carrier": 1}]
    assert doc["starts"] == [{"body": 1, "value": 0.2}, {"body": 0, "value": 0.1}]
    with pytest.raises(StartStateError, match="free joint"):
        translate(model, starts={"load_joint": 0.1})
    with pytest.raises(StartStateError, match="no such joint"):
        translate(model, starts={"missing": 0.1})
    with pytest.raises(UnmappableElementError, match="not a free body"):
        translate(model, payloads={"carrier": "link"})
    with pytest.raises(UnmappableElementError, match="not another body"):
        translate(model, payloads={"load": "load"})
    prepared = prepare_model(model, {"pivot": 0.1}, 5e-5)
    assert prepared.version == "0.1.0"
    assert "5e-05 s" in prepared.report[0]


@pytest.mark.req("BLD-018")
def test_segment_defaults_by_support_condition() -> None:
    assert structure.default_segments("clamped", "clamped") == 16
    assert structure.calibration_min_segments("clamped", "clamped") == 16
    for root, end in (("clamped", "free"), ("pinned", "roller"), ("clamped", "roller")):
        assert structure.default_segments(root, end) == 8
        assert structure.calibration_min_segments(root, end) == 8


@pytest.mark.req("PHY-019")
@pytest.mark.parametrize("kind", ["spherical", "free"])
@pytest.mark.parametrize(
    ("field", "value", "refusal"),
    [("stiffness", 3.0, "stiffness on a"), ("lower", -0.5, "limits on a"), ("upper", 0.5, "limits on a")],
)
def test_stiffness_or_limits_on_a_ball_or_free_joint_are_refused(
    kind: str, field: str, value: float, refusal: str
) -> None:
    """The own engine's joint springs and limits act on revolute, prismatic and planar coordinates. The neutral model
    already refuses them on a ball or free joint; a model built without validation reaches the translation, which
    refuses it with the joint named instead of dropping the value (PHY-019, invariant 7)."""
    data = pendulum_data()
    body = free_body("ball")
    joint: dict[str, Any] = {"name": "ball_joint", "type": kind, "position": ("0 m", "0 m", "-0.3 m")}
    joint["angular"] = [{}, {}, {}]
    if kind == "free":
        joint["linear"] = [{}, {}, {}]
    body["joint"] = joint
    data["bodies"] = [body]
    data["signals"] = {}
    model = model_of(data)
    translate(model)
    bad = dict(joint)
    bad["angular"] = [{field: f"{value} rad"}, {}, {}]
    if field == "stiffness":
        bad["angular"] = [{field: f"{value} N*m/rad"}, {}, {}]
    with pytest.raises(ValueError, match=f"joint 'ball_joint' \\({kind}\\) takes no"):
        model_of({**data, "bodies": [{**body, "joint": bad}]})
    built = model.bodies[0]
    assert built.joint is not None
    dof = built.joint.angular[0].model_copy(update={field: value})
    unvalidated_joint = built.joint.model_copy(update={"angular": (dof, *built.joint.angular[1:])})
    unvalidated = model.model_copy(update={"bodies": (built.model_copy(update={"joint": unvalidated_joint}),)})
    with pytest.raises(UnmappableElementError, match=f"joint 'ball_joint'.*{refusal} {kind} joint"):
        translate(unvalidated)


@pytest.mark.req("PHY-031")
def test_the_drive_line_names_the_loss_per_period() -> None:
    """Ruling 21: backward Euler loses pi omega h of a mode's amplitude per period (it keeps about e^(-pi omega h))."""
    data = pendulum_data()
    data["bodies"] = [slider("cart")]
    data["signals"] = {"drive_joints": ["cart_joint"]}
    lines = [line for line in translate(model_of(data), step=1e-3).report if "joint drive" in line]
    assert len(lines) == 1
    assert "loses about pi x of its amplitude per period" in lines[0]
    assert "keeps (1 + x^2)^(-pi / atan x), about e^(-pi x)" in lines[0]
