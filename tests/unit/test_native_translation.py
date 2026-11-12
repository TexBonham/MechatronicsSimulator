"""Neutral model to the own engine's model document (PHY-018, PHY-019, PHY-020) and the adapter selection."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

import pytest

from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.native.translate import (
    SUPPORTED_ELEMENTS,
    StartStateError,
    UnmappableElementError,
    element_kinds,
    translate,
)
from mechsim.physics.neutral import PhysicsModel, from_yaml
from mechsim.record.reader import DriverError
from mechsim.testing.core_driver import ADAPTER_VARIABLE, physics_adapter_name

ROOT = Path(__file__).resolve().parents[2]
PENDULUM = (ROOT / "schemas/examples/physics_model.pendulum.yaml").read_text()


def pendulum_data() -> dict[str, Any]:
    return from_yaml(PENDULUM).model_dump(mode="python")


def model_of(data: dict[str, Any]) -> PhysicsModel:
    return PhysicsModel.model_validate(data)


def slider(name: str, parent: str = "world", axis: tuple[float, float, float] = (1.0, 0.0, 0.0)) -> dict[str, Any]:
    return {
        "name": name,
        "mass": 2.0,
        "com": (0.0, 0.0, 0.0),
        "inertia": {"ixx": 0.01, "iyy": 0.01, "izz": 0.01},
        "material": "steel",
        "parent": parent,
        "joint": {"name": name + "_joint", "type": "prismatic", "position": (0.0, 0.0, 0.0), "axis": axis},
    }


def free_body(name: str) -> dict[str, Any]:
    body = slider(name)
    body["joint"] = {
        "name": name + "_joint",
        "type": "free",
        "position": (0.0, 0.0, -0.3),
        "linear": [{"damping": 1.0}, {"damping": 2.0}, {"damping": 3.0}],
        "angular": [{"damping": 4.0}, {"damping": 5.0}, {"damping": 6.0}],
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
        {"type": "point", "name": "tie", "body_a": "ball", "body_b": "world", "anchor_a": (0, 0, 0), "anchor_b": (0, 0, -0.3)}
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
    ball["joint"]["linear"] = [{"friction": 1.0}, {}, {}]
    data["bodies"].append(ball)
    with pytest.raises(UnmappableElementError, match="ball_joint"):
        translate(model_of(data))
    data = pendulum_data()
    section = {"elastic_modulus": 2e11, "second_moment_of_area": 1e-8, "density": 7800.0, "area": 1e-4, "length": 1.0}
    data["lumped_members"] = [
        {"name": "beam", "parent": "link", "position": (0, 0, 0), "section": section, "root": "clamped", "end": "roller"}
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
    shaft["joint"] = {"name": "motor_joint", "type": "revolute", "position": (0, 0, 0), "axis": (0, 0, 1)}
    data["bodies"] += [shaft, slider("carriage")]
    data["transmissions"] = [
        {
            "type": "gear_pair",
            "name": "gear",
            "shaft_joint": "motor_joint",
            "driven_joint": "carriage_joint",
            "efficiency": 0.9,
            "ratio": -3.0,
            "stiffness": 1e4,
            "backlash": 0.01,
        }
    ]
    with pytest.raises(ValueError):
        model_of(data)
    carriage = data["bodies"][2]
    carriage["joint"] = {"name": "carriage_joint", "type": "revolute", "position": (0, 0, 0), "axis": (0, 0, 1)}
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


@pytest.mark.req("PHY-031")
def test_adapter_selection(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(ADAPTER_VARIABLE, raising=False)
    assert physics_adapter_name() == "mujoco"
    assert physics_adapter_name("native") == "native"
    monkeypatch.setenv(ADAPTER_VARIABLE, "native")
    assert physics_adapter_name() == "native"
    assert physics_adapter_name("mujoco") == "mujoco"
    with pytest.raises(DriverError, match="physics.adapter"):
        physics_adapter_name("bullet")
