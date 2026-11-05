"""Designs for the builder tests: the shipped files, and the gantry with every physics open value filled in."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from ruamel.yaml import YAML

from mechsim.design.schema import Design, load_design

ROOT = Path(__file__).resolve().parents[2]
GANTRY = ROOT / "designs" / "gantry" / "design.yaml"
PENDULUM = ROOT / "designs" / "pendulum" / "design.yaml"
EXAMPLE = ROOT / "schemas" / "examples" / "physics_model.pendulum.yaml"
MATERIAL = "aluminium_6063_t5_extrusion"
CUBE = "0.1 m"

# Child placement in the parent frame. World positions at the reference configuration, 0.1 m cubes, at least 1 m
# apart: frame (0,0,0), rail_x (1,0,0), rail_y (1,1,0), trolley (1,2,0), lift_carriage (1,2,1), hook (1,2,2),
# x_motor_shaft (-1,0,0), y_motor_shaft (1,1,-1), lift_motor_shaft (2,2,0).
PLACEMENTS: dict[str, list[str]] = {
    "frame_mount": ["0 m", "0 m", "0 m"],
    "rail_x_mount": ["1 m", "0 m", "0 m"],
    "x_axis": ["0 m", "1 m", "0 m"],
    "y_axis": ["0 m", "1 m", "0 m"],
    "lift_axis": ["0 m", "0 m", "1 m"],
    "hook_swing": ["0 m", "0 m", "1 m"],
    "x_motor_spin": ["-1 m", "0 m", "0 m"],
    "y_motor_spin": ["0 m", "0 m", "-1 m"],
    "lift_motor_spin": ["1 m", "0 m", "0 m"],
}

SCALARS: dict[str, str] = {
    "workspace_x": "1 m",
    "workspace_y": "0.8 m",
    "lift_travel": "0.5 m",
    "rated_load": "3 kg",
    "rail_section_area": "1e-4 m**2",
    "rail_section_second_moment": "1e-8 m**4",
    "minimum_safety_factor": "2",
    "ambient_temperature": "298.15 K",
    "contact_stiffness": "100000 N/m",
    "contact_damping": "100 N*s/m",
    "contact_friction": "0.5",
    "axis_friction": "1 N",
    "axis_damping": "10 N*s/m",
    "axis_backlash": "0 m",
    "shaft_friction": "0.01 N*m",
    "shaft_damping": "0.001 N*m*s/rad",
    "shaft_backlash": "0 rad",
    "hook_friction": "0.001 N*m",
    "hook_damping": "0.001 N*m*s/rad",
    "hook_backlash": "0 rad",
    "deflection_target_mode": "static",
    "encoder_placement": "frame",
}


def _replace(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _replace(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_replace(item) for item in value]
    if isinstance(value, str) and value.startswith("${") and value[2:-1] in SCALARS:
        return SCALARS[value[2:-1]]
    return value


def gantry_data() -> dict[str, Any]:
    """The gantry file as plain data with the test values below in place of its placeholder geometry; rail_x is a
    cantilever (clamped, free), since the neutral lumped beam has no clamped-clamped form."""
    data: dict[str, Any] = YAML(typ="safe", pure=True).load(GANTRY.read_text(encoding="utf-8"))
    for body in data["bodies"]:
        body["material"] = MATERIAL
        body["process"] = {"kind": "extrusion"}
        body["geometry"] = {
            "kind": "primitives",
            "shapes": [{"shape": "box", "position": ["0 m", "0 m", "0 m"], "size": [CUBE, CUBE, CUBE]}],
        }
        body.pop("mass_properties", None)
        if "flexible" in body:
            body["flexible"]["section"] = {
                "area": "${rail_section_area}",
                "second_moment": "${rail_section_second_moment}",
                "length": "${workspace_x}",
            }
            body["flexible"]["calibration"] = {
                "source": "hand",
                "load": "10 N",
                "deflection": "1 mm",
                "first_natural_frequency": "50 Hz",
            }
    data["bodies"][1]["flexible"]["supports"] = ["clamped", "free"]
    data["bodies"][2]["flexible"]["section"]["length"] = "${workspace_y}"
    data["environment"]["contact"]["default"] = {
        "stiffness": "${contact_stiffness}",
        "damping": "${contact_damping}",
        "friction": "${contact_friction}",
    }
    limits = {"x_axis": "${workspace_x}", "y_axis": "${workspace_y}", "lift_axis": "${lift_travel}"}
    for joint in data["joints"]:
        joint["placement"] = {"position": PLACEMENTS[joint["name"]]}
        if joint["type"] == "revolute":
            joint["axis"] = [0.0, 0.0, 1.0]
        for dof in joint.get("linear", []):
            dof.update(
                {
                    "limits": {"lower": "0 m", "upper": limits[joint["name"]]},
                    "friction": "${axis_friction}",
                    "damping": "${axis_damping}",
                    "backlash": "${axis_backlash}",
                }
            )
        prefix = "hook"
        if joint["type"] == "revolute":
            prefix = "shaft"
        for dof in joint.get("angular", []):
            dof.update(
                {
                    "friction": "${" + prefix + "_friction}",
                    "damping": "${" + prefix + "_damping}",
                    "backlash": "${" + prefix + "_backlash}",
                }
            )
    return _replace(data)


def filled_gantry(data: dict[str, Any] | None = None) -> Design:
    if data is None:
        data = gantry_data()
    return Design.model_validate(data)


def gantry() -> Design:
    return load_design(GANTRY)


def pendulum() -> Design:
    return load_design(PENDULUM)
