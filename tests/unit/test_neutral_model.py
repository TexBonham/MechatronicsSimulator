"""Engine-neutral physics model as a data-file kind (ADR 0003): validation, units, references, serialization."""

from __future__ import annotations

import copy
import json
import math
import re
from pathlib import Path
from typing import Any

import jsonschema
import pytest
from pydantic import BaseModel, ValidationError
from ruamel.yaml import YAML

from mechsim.physics import neutral

ROOT = Path(__file__).resolve().parents[2]
EXAMPLE = ROOT / "schemas" / "examples" / "physics_model.pendulum.yaml"

_MISSING = object()


def _example() -> dict[str, Any]:
    data: dict[str, Any] = YAML(typ="safe", pure=True).load(EXAMPLE.read_text(encoding="utf-8"))
    return data


def _vec(x: str, y: str, z: str) -> list[str]:
    return [x, y, z]


def _full() -> dict[str, Any]:
    """The pendulum plus one element of every other kind, all valid."""
    data = _example()
    inertia = {"ixx": "1e-4 kg*m**2", "iyy": "1e-4 kg*m**2", "izz": "1e-4 kg*m**2"}
    data["bodies"].append(
        {
            "name": "motor",
            "mass": "0.5 kg",
            "com": _vec("0 m", "0 m", "0 m"),
            "inertia": dict(inertia),
            "material": "steel",
            "parent": "world",
            "joint": {"name": "shaft", "type": "revolute", "position": _vec("1 m", "0 m", "0 m")},
            "catalog_part": "nema17",
        }
    )
    data["bodies"].append(
        {
            "name": "carriage",
            "mass": "2 kg",
            "com": _vec("0 m", "0 m", "0 m"),
            "inertia": dict(inertia),
            "material": "aluminium",
            "parent": "world",
            "joint": {
                "name": "slide",
                "type": "prismatic",
                "position": _vec("0 m", "1 m", "0 m"),
                "axis": [1.0, 0.0, 0.0],
                "linear": [{"lower": "-0.5 m", "upper": "0.5 m", "friction": "2 N"}],
            },
            "shapes": [
                {"shape": "sphere", "position": _vec("0 m", "0 m", "1 cm"), "radius": "0.01 m", "material": "steel"},
                {"shape": "capsule", "radius": "0.01 m", "length": "0.1 m", "material": "steel"},
                {"shape": "box", "size": _vec("0.1 m", "0.2 m", "0.3 m"), "material": "steel"},
                {
                    "shape": "convex_hull",
                    "vertices": [
                        _vec("0 m", "0 m", "0 m"),
                        _vec("1 cm", "0 m", "0 m"),
                        _vec("0 m", "1 cm", "0 m"),
                        _vec("0 m", "0 m", "1 cm"),
                    ],
                    "material": "steel",
                },
            ],
        }
    )
    data["constraints"] = [
        {
            "type": "point",
            "name": "tie",
            "body_a": "link",
            "body_b": "world",
            "anchor_a": _vec("0.5 m", "0 m", "0 m"),
            "anchor_b": _vec("0 m", "-0.5 m", "0 m"),
        },
        {
            "type": "hinge",
            "name": "hinge",
            "body_a": "carriage",
            "body_b": "motor",
            "anchor_a": _vec("0 m", "0 m", "0 m"),
            "anchor_b": _vec("0 m", "0 m", "0 m"),
            "axis_a": [0.0, 0.0, 1.0],
            "axis_b": [0.0, 0.0, 1.0],
        },
    ]
    section = {
        "elastic_modulus": "70 GPa",
        "second_moment_of_area": "1e-8 m**4",
        "density": "2700 kg/m**3",
        "area": "1e-4 m**2",
        "length": "1 m",
    }
    data["lumped_members"] = [
        {
            "name": "beam",
            "parent": "world",
            "position": _vec("0 m", "2 m", "0 m"),
            "section": dict(section),
            "root": "clamped",
            "end": "free",
            "segments": 4,
            "roller_mass_fraction": 1e-3,
            "node_damping": "0.01 N*m*s/rad",
            "structural_member": "rail",
        }
    ]
    data["transmissions"] = [
        {
            "type": "belt_pulley",
            "name": "belt",
            "shaft_joint": "shaft",
            "driven_joint": "slide",
            "efficiency": 0.95,
            "pulley_radius": "10 mm",
            "stiffness": "1e5 N/m",
            "backlash": "0 m",
            "slip": 0.0,
        },
        {
            "type": "lead_screw",
            "name": "screw",
            "shaft_joint": "shaft",
            "driven_joint": "slide",
            "efficiency": 0.4,
            "lead": "8 mm",
            "stiffness": "1e7 N/m",
            "backlash": "0.05 mm",
            "self_locking": True,
        },
        {
            "type": "gear_pair",
            "name": "gears",
            "shaft_joint": "shaft",
            "driven_joint": "pivot",
            "efficiency": 0.98,
            "ratio": -3.0,
            "stiffness": "1e4 N*m/rad",
            "backlash": "0.1 deg",
        },
    ]
    data["signals"] = {"load_torque_joints": ["shaft"], "pose_bodies": ["link", "carriage"]}
    data["world_shapes"] = [{"shape": "plane", "material": "concrete"}]
    data["contacts"] = {
        "default": {"stiffness": "1e5 N/m", "damping": "100 N*s/m", "friction": 0.5},
        "pairs": [
            {"materials": ["steel", "concrete"], "stiffness": "1e6 N/m", "damping": "200 N*s/m", "friction": 0.6}
        ],
        "exclusions": [["link", "world"]],
    }
    data["structural_members"] = [
        {
            "name": "rail",
            "section": dict(section),
            "material": "aluminium",
            "supports": [{"at": "0 m", "condition": "clamped"}, {"at": "1 m", "condition": "roller"}],
            "fasteners": [{"kind": "M5 bolt", "count": 4}],
        }
    ]
    return data


def _get(data: Any, path: tuple[Any, ...]) -> Any:
    node = data
    for key in path:
        node = node[key]
    return node


def _set(data: Any, path: tuple[Any, ...], value: Any) -> dict[str, Any]:
    result = copy.deepcopy(data)
    parent = _get(result, path[:-1])
    if value is _MISSING:
        del parent[path[-1]]
    else:
        parent[path[-1]] = value
    return result


def _model(data: dict[str, Any]) -> neutral.PhysicsModel:
    return neutral.PhysicsModel.model_validate(data)


def _rejects(data: dict[str, Any], message: str = "") -> None:
    with pytest.raises(ValidationError) as caught:
        _model(data)
    assert message in str(caught.value)


BODY0 = ("bodies", 0)
JOINT0 = (*BODY0, "joint")
MOTOR = ("bodies", 1)
CARRIAGE = ("bodies", 2)
SHAPES = (*CARRIAGE, "shapes")
SLIDE_DOF = (*CARRIAGE, "joint", "linear", 0)
PIVOT_DOF = (*JOINT0, "angular", 0)
POINT = ("constraints", 0)
HINGE = ("constraints", 1)
BEAM = ("lumped_members", 0)
SECTION = (*BEAM, "section")
BELT = ("transmissions", 0)
SCREW = ("transmissions", 1)
GEARS = ("transmissions", 2)
RAIL = ("structural_members", 0)


# ===== Worked example =================================================


@pytest.mark.req("INV-012")
@pytest.mark.req("BLD-030")
def test_example_validates_round_trips_and_matches_json_schema() -> None:
    model = neutral.from_yaml(EXAMPLE.read_text(encoding="utf-8"))
    text = neutral.to_yaml(model)
    assert neutral.from_yaml(text) == model
    schema = json.loads(neutral.schema_text())
    jsonschema.validate(_example(), schema)
    jsonschema.validate(YAML(typ="safe", pure=True).load(text), schema)


@pytest.mark.req("BLD-002")
def test_example_is_the_engine_reference_pendulum_in_si() -> None:
    model = _model(_example())
    assert len(model.bodies) == 1
    assert model.lumped_members == ()
    link = model.bodies[0]
    assert link.joint is not None
    assert link.joint.type == "revolute"
    assert link.parent == "world"
    assert model.gravity == (0.0, -9.81, 0.0)
    assert link.mass == 1.0
    assert link.com == (0.25, 0.0, 0.0)
    assert math.isclose(link.inertia.iyy, 1.0 * 0.5**2 / 12.0, rel_tol=1e-15)
    assert math.isclose(link.inertia.ixx, 0.5 * 1.0 * 0.01**2, rel_tol=1e-15)
    pivot_inertia = 1.0 * 0.5**2 / 3.0
    omega = math.sqrt(1.0 * 9.81 * 0.25 / pivot_inertia)
    assert math.isclose(link.joint.angular[0].damping, 2.0 * 0.05 * pivot_inertia * omega, rel_tol=1e-15)
    assert model.run.step == 1e-3
    assert link.material == "aluminium_6063_t5_extrusion"


@pytest.mark.req("BLD-002")
@pytest.mark.req("BLD-003")
@pytest.mark.req("BLD-012")
def test_full_model_carries_every_element_kind() -> None:
    model = _model(_full())
    assert [body.name for body in model.bodies] == ["link", "motor", "carriage"]
    assert [type(c).__name__ for c in model.constraints] == ["PointConstraint", "HingeConstraint"]
    assert [type(t).__name__ for t in model.transmissions] == ["BeltPulley", "LeadScrew", "GearPair"]
    assert [type(s).__name__ for s in model.bodies[2].shapes] == ["Sphere", "Capsule", "Box", "ConvexHull"]
    assert model.bodies[1].catalog_part == "nema17"
    assert model.signals.load_torque_joints == ("shaft",)
    assert model.structural_members[0].fasteners == ({"kind": "M5 bolt", "count": 4},)


@pytest.mark.req("BLD-030")
def test_quantities_convert_to_si_and_serialize_with_si_units() -> None:
    data = _set(_full(), (*CARRIAGE, "mass"), "1500 g")
    model = _model(data)
    assert model.bodies[2].mass == 1.5
    assert model.transmissions[0].pulley_radius == 0.01
    assert math.isclose(model.transmissions[2].backlash, math.radians(0.1), rel_tol=1e-15)
    assert model.lumped_members[0].section.elastic_modulus == 70e9
    dumped = model.model_dump(mode="json")
    assert dumped["bodies"][2]["mass"] == "1.5 kg"
    assert dumped["transmissions"][0]["pulley_radius"] == "0.01 m"


# ===== Run configuration defaults =======================================


@pytest.mark.req("BLD-030")
def test_run_configuration_defaults_follow_adr_0013() -> None:
    model = _model(_set(_example(), ("run",), _MISSING))
    assert model.run.step == 1e-3
    assert model.run.iteration_cap == 50
    assert model.run.projection_iterations == 3
    assert model.run.drift_threshold_linear == 1e-6
    assert model.run.drift_threshold_angular == 1e-6
    assert model.run.symmetry_tolerance == 1e-12
    assert model.run.unit_quaternion_tolerance == 1e-12
    assert model.run.velocity_tolerance_linear == 1e-9
    assert model.run.velocity_tolerance_angular == 1e-9


@pytest.mark.req("BLD-018")
def test_segment_count_is_data_with_spec_defaults() -> None:
    data = _set(_set(_full(), (*BEAM, "segments"), _MISSING), (*BEAM, "roller_mass_fraction"), _MISSING)
    assert _model(data).lumped_members[0].segments == 8
    pinned = _set(_set(data, (*BEAM, "root"), "pinned"), (*BEAM, "end"), "roller")
    assert _model(pinned).lumped_members[0].segments == 8
    clamped_roller = _set(data, (*BEAM, "end"), "roller")
    assert _model(clamped_roller).lumped_members[0].segments == 8
    override = _set(pinned, (*BEAM, "segments"), 12)
    assert _model(override).lumped_members[0].segments == 12
    assert _model(data).lumped_members[0].roller_mass_fraction == 1e-3


# ===== Required fields ==================================================

REQUIRED = [
    ("schema_version",),
    ("design",),
    ("design", "name"),
    ("design", "version"),
    ("design", "blob"),
    ("gravity",),
    ("bodies",),
    (*BODY0, "name"),
    (*BODY0, "mass"),
    (*BODY0, "com"),
    (*BODY0, "inertia"),
    (*BODY0, "inertia", "ixx"),
    (*BODY0, "inertia", "iyy"),
    (*BODY0, "inertia", "izz"),
    (*BODY0, "parent"),
    (*JOINT0, "name"),
    (*JOINT0, "type"),
    (*JOINT0, "position"),
    (*SHAPES, 0, "shape"),
    (*SHAPES, 0, "radius"),
    (*SHAPES, 0, "material"),
    (*SHAPES, 1, "length"),
    (*SHAPES, 2, "size"),
    (*SHAPES, 3, "vertices"),
    (*POINT, "type"),
    (*POINT, "name"),
    (*POINT, "body_a"),
    (*POINT, "body_b"),
    (*POINT, "anchor_a"),
    (*POINT, "anchor_b"),
    (*HINGE, "axis_a"),
    (*HINGE, "axis_b"),
    (*BEAM, "name"),
    (*BEAM, "parent"),
    (*BEAM, "position"),
    (*BEAM, "section"),
    (*BEAM, "root"),
    (*BEAM, "end"),
    (*SECTION, "elastic_modulus"),
    (*SECTION, "second_moment_of_area"),
    (*SECTION, "density"),
    (*SECTION, "area"),
    (*SECTION, "length"),
    (*BELT, "type"),
    (*BELT, "name"),
    (*BELT, "shaft_joint"),
    (*BELT, "driven_joint"),
    (*BELT, "efficiency"),
    (*BELT, "pulley_radius"),
    (*BELT, "stiffness"),
    (*SCREW, "lead"),
    (*SCREW, "stiffness"),
    (*GEARS, "ratio"),
    (*GEARS, "stiffness"),
    (*RAIL, "name"),
    (*RAIL, "section"),
    (*RAIL, "material"),
    (*RAIL, "supports"),
    (*RAIL, "fasteners"),
    (*RAIL, "supports", 0, "at"),
    (*RAIL, "supports", 0, "condition"),
    ("contacts", "default"),
    ("contacts", "default", "stiffness"),
    ("contacts", "default", "damping"),
    ("contacts", "default", "friction"),
    ("contacts", "pairs", 0, "materials"),
    ("contacts", "pairs", 0, "friction"),
    ("world_shapes", 0, "material"),
]


@pytest.mark.req("BLD-002")
@pytest.mark.req("BLD-003")
@pytest.mark.req("BLD-009")
@pytest.mark.req("BLD-012")
@pytest.mark.parametrize("path", REQUIRED, ids=["/".join(str(p) for p in path) for path in REQUIRED])
def test_each_required_field_missing_is_rejected(path: tuple[Any, ...]) -> None:
    _rejects(_set(_full(), path, _MISSING))


@pytest.mark.req("BLD-020")
def test_body_without_material_is_rejected_naming_the_body() -> None:
    _rejects(_set(_full(), (*MOTOR, "material"), _MISSING), "body 'motor' has no material")


@pytest.mark.req("BLD-021")
def test_unconstrained_body_is_rejected_naming_the_body() -> None:
    _rejects(_set(_full(), (*MOTOR, "joint"), _MISSING), "unconstrained body 'motor'")


@pytest.mark.req("BLD-021")
def test_free_body_is_constrained_by_its_free_joint() -> None:
    data = _set(_full(), (*MOTOR, "joint"), {"name": "float", "type": "free", "position": _vec("0 m", "0 m", "0 m")})
    data["signals"]["load_torque_joints"] = []
    data["transmissions"] = []
    assert _model(data).bodies[1].joint is not None


@pytest.mark.req("BLD-012")
@pytest.mark.parametrize(
    "path",
    [
        (*JOINT0, "orientation"),
        (*JOINT0, "axis"),
        (*CARRIAGE, "joint", "linear"),
        (*JOINT0, "angular"),
        (*BODY0, "shapes"),
        (*MOTOR, "catalog_part"),
        (*SHAPES, 0, "position"),
        (*SLIDE_DOF, "lower"),
        (*BEAM, "node_damping"),
        ("run", "velocity_tolerance_linear"),
        ("run", "velocity_tolerance_angular"),
        ("run", "unit_quaternion_tolerance"),
        ("world_shapes",),
        ("contacts",),
        ("contacts", "pairs"),
        ("contacts", "exclusions"),
        (*BEAM, "segments"),
        (*BEAM, "roller_mass_fraction"),
        (*BEAM, "structural_member"),
        (*BELT, "slip"),
        (*BELT, "backlash"),
        (*SCREW, "self_locking"),
        (*PIVOT_DOF, "damping"),
        ("run",),
        ("run", "step"),
        ("constraints",),
        ("lumped_members",),
        ("transmissions",),
        ("signals",),
        ("structural_members",),
    ],
)
def test_optional_fields_take_defaults(path: tuple[Any, ...]) -> None:
    data = _full()
    if path == ("structural_members",):
        del data["lumped_members"][0]["structural_member"]
    _model(_set(data, path, _MISSING))


@pytest.mark.req("BLD-030")
def test_slip_defaults_to_zero_and_self_locking_to_false() -> None:
    model = _model(_set(_full(), (*SCREW, "self_locking"), _MISSING))
    assert [t.slip for t in model.transmissions] == [0.0, 0.0, 0.0]
    screw = model.transmissions[1]
    assert isinstance(screw, neutral.LeadScrew)
    assert screw.self_locking is False


# ===== Bounds ===========================================================

BOUNDS: list[tuple[tuple[Any, ...], Any, bool]] = [
    ((*BODY0, "mass"), "0 kg", False),
    ((*BODY0, "mass"), "1e-9 kg", True),
    ((*BODY0, "mass"), "-1 kg", False),
    ((*BODY0, "inertia", "ixx"), "0 kg*m**2", True),
    ((*BODY0, "inertia", "ixx"), "-1e-12 kg*m**2", False),
    ((*BODY0, "inertia", "ixy"), "1e-3 kg*m**2", True),
    ((*BODY0, "inertia", "ixy"), "1.1e-3 kg*m**2", False),
    ((*BODY0, "inertia", "iyz"), "0.020833333333333332 kg*m**2", True),
    ((*BODY0, "inertia", "iyz"), "0.021 kg*m**2", False),
    ((*SHAPES, 0, "radius"), "0 m", False),
    ((*SHAPES, 0, "radius"), "1e-6 m", True),
    ((*SHAPES, 0, "radius"), "-1 m", False),
    ((*SHAPES, 1, "length"), "0 m", False),
    ((*SHAPES, 1, "length"), "1 mm", True),
    ((*SHAPES, 2, "size", 1), "0 m", False),
    ((*SHAPES, 2, "size", 1), "1 mm", True),
    ((*BODY0, "shapes", 0, "radius"), "0 m", False),
    ((*BODY0, "shapes", 0, "length"), "0 m", False),
    ((*PIVOT_DOF, "damping"), "0 N*m*s/rad", True),
    ((*PIVOT_DOF, "damping"), "-1e-9 N*m*s/rad", False),
    ((*PIVOT_DOF, "friction"), "0 N*m", True),
    ((*PIVOT_DOF, "friction"), "-1 N*m", False),
    ((*PIVOT_DOF, "stiffness"), "0 N*m/rad", True),
    ((*PIVOT_DOF, "stiffness"), "-1 N*m/rad", False),
    ((*SLIDE_DOF, "friction"), "-1 N", False),
    ((*SLIDE_DOF, "damping"), "-1 N*s/m", False),
    ((*SLIDE_DOF, "stiffness"), "-1 N/m", False),
    ((*SLIDE_DOF, "upper"), "-0.5 m", False),
    ((*SLIDE_DOF, "upper"), "-0.49 m", True),
    ((*SLIDE_DOF, "upper"), "-0.6 m", False),
    ((*PIVOT_DOF, "lower"), "10 deg", True),
    ((*SECTION, "elastic_modulus"), "0 Pa", False),
    ((*SECTION, "elastic_modulus"), "1 Pa", True),
    ((*SECTION, "second_moment_of_area"), "0 m**4", False),
    ((*SECTION, "density"), "0 kg/m**3", False),
    ((*SECTION, "area"), "0 m**2", False),
    ((*SECTION, "length"), "0 m", False),
    ((*SECTION, "length"), "-1 m", False),
    ((*BEAM, "segments"), 0, False),
    ((*BEAM, "segments"), 1, True),
    ((*BEAM, "segments"), -1, False),
    ((*BEAM, "roller_mass_fraction"), 0.0, False),
    ((*BEAM, "roller_mass_fraction"), 1e-9, True),
    ((*BEAM, "roller_mass_fraction"), -1e-3, False),
    ((*BELT, "efficiency"), 0.0, False),
    ((*BELT, "efficiency"), 1e-6, True),
    ((*BELT, "efficiency"), 1.0, True),
    ((*BELT, "efficiency"), 1.000001, False),
    ((*BELT, "slip"), 0.0, True),
    ((*BELT, "slip"), 0.5, True),
    ((*BELT, "slip"), 1.0, True),
    ((*BELT, "slip"), -1e-9, False),
    ((*BELT, "slip"), 1.000001, False),
    ((*BELT, "stiffness"), "0 N/m", False),
    ((*BELT, "stiffness"), "1 N/m", True),
    ((*BELT, "pulley_radius"), "0 m", False),
    ((*BELT, "backlash"), "0 m", True),
    ((*BELT, "backlash"), "-1 um", False),
    ((*SCREW, "lead"), "0 m", False),
    ((*SCREW, "lead"), "1 um", True),
    ((*SCREW, "stiffness"), "0 N/m", False),
    ((*SCREW, "backlash"), "-1 um", False),
    ((*GEARS, "ratio"), 0.0, False),
    ((*GEARS, "ratio"), 1e-6, True),
    ((*GEARS, "ratio"), -1e-6, True),
    ((*GEARS, "stiffness"), "0 N*m/rad", False),
    ((*GEARS, "backlash"), "0 rad", True),
    ((*GEARS, "backlash"), "-1 deg", False),
    (("run", "step"), "0 s", False),
    (("run", "step"), "1 us", True),
    (("run", "step"), "-1 ms", False),
    (("run", "iteration_cap"), 0, False),
    (("run", "iteration_cap"), 1, True),
    (("run", "projection_iterations"), 0, True),
    (("run", "projection_iterations"), -1, False),
    (("run", "drift_threshold_linear"), "0 m", False),
    (("run", "drift_threshold_linear"), "1e-9 m", True),
    (("run", "drift_threshold_angular"), "0 rad", False),
    (("run", "drift_threshold_angular"), "1e-9 rad", True),
    (("run", "symmetry_tolerance"), 0.0, False),
    (("run", "symmetry_tolerance"), 1e-15, True),
    (("run", "velocity_tolerance_linear"), "0 m/s", False),
    (("run", "velocity_tolerance_linear"), "1e-12 m/s", True),
    (("run", "velocity_tolerance_linear"), "-1e-9 m/s", False),
    (("run", "velocity_tolerance_angular"), "0 rad/s", False),
    (("run", "velocity_tolerance_angular"), "1e-12 rad/s", True),
    (("run", "velocity_tolerance_angular"), "-1e-9 rad/s", False),
    (("run", "unit_quaternion_tolerance"), 0.0, False),
    (("run", "unit_quaternion_tolerance"), 1e-15, True),
    ((*BEAM, "node_damping"), "0 N*m*s/rad", True),
    ((*BEAM, "node_damping"), "0.1 N*m*s/rad", True),
    ((*BEAM, "node_damping"), "-1e-9 N*m*s/rad", False),
    (("contacts", "default", "stiffness"), "0 N/m", False),
    (("contacts", "default", "stiffness"), "1 N/m", True),
    (("contacts", "default", "damping"), "0 N*s/m", True),
    (("contacts", "default", "damping"), "-1 N*s/m", False),
    (("contacts", "default", "friction"), 0.0, True),
    (("contacts", "default", "friction"), 1.5, True),
    (("contacts", "default", "friction"), -1e-9, False),
    (("contacts", "pairs", 0, "friction"), -0.1, False),
    (("contacts", "pairs", 0, "stiffness"), "0 N/m", False),
    ((*RAIL, "supports", 1, "at"), "1 m", True),
    ((*RAIL, "supports", 1, "at"), "1.001 m", False),
    ((*RAIL, "supports", 0, "at"), "-1 mm", False),
    ((*JOINT0, "axis"), [0.0, 0.0, 0.0], False),
    ((*JOINT0, "axis"), [0.0, 1e-9, 0.0], True),
    ((*JOINT0, "orientation"), [0.0, 0.0, 0.0, 0.0], False),
    ((*HINGE, "axis_b"), [0.0, 0.0, 0.0], False),
    (("schema_version",), 2, False),
    (("schema_version",), 0, False),
    (("design", "version"), "1.2", False),
    (("design", "version"), "1.2.3-rc.1", True),
    (("design", "version"), "01.2.3", False),
    (("design", "blob"), "a" * 64, True),
    (("design", "blob"), "a" * 39, False),
    (("design", "blob"), "A" * 40, False),
    (("gravity",), ["0 m/s**2", "0 m/s**2", "0 m/s**2"], True),
]


@pytest.mark.req("BLD-002")
@pytest.mark.req("BLD-009")
@pytest.mark.req("BLD-018")
@pytest.mark.parametrize(("path", "value", "valid"), BOUNDS)
def test_bounds_at_inside_and_outside(path: tuple[Any, ...], value: Any, valid: bool) -> None:
    data = _set(_full(), path, value)
    if valid:
        _model(data)
    else:
        _rejects(data)


# ===== Types and units ==================================================

WRONG_TYPES = [
    ((*BODY0, "mass"), 1.0),
    ((*BODY0, "mass"), 1),
    ((*BODY0, "mass"), True),
    ((*BODY0, "mass"), ["1 kg"]),
    ((*BODY0, "name"), 5),
    ((*BODY0, "name"), ""),
    ((*BODY0, "name"), "has space"),
    ((*BODY0, "name"), "Link"),
    ((*BODY0, "name"), "link.rod"),
    ((*BODY0, "name"), "link-rod"),
    ((*BODY0, "name"), "1link"),
    ((*BODY0, "material"), "Aluminium"),
    ((*SHAPES, 0, "material"), "steel 304"),
    (("contacts", "default", "friction"), "0.5"),
    (("contacts", "pairs", 0, "materials"), ["steel"]),
    (("contacts", "exclusions", 0), ["link", "world", "motor"]),
    (("contacts", "default"), None),
    ((*BODY0, "com"), "0 m"),
    ((*BODY0, "com"), ["0 m", "0 m"]),
    ((*BODY0, "com"), ["0 m", "0 m", "0 m", "0 m"]),
    (("gravity",), ["0 m/s**2", "-9.81 m/s**2"]),
    ((*JOINT0, "type"), "ball"),
    ((*JOINT0, "axis"), ["0", "0", "1"]),
    ((*JOINT0, "axis"), [0.0, 0.0, True]),
    ((*JOINT0, "axis"), [0.0, 0.0, float("nan")]),
    ((*JOINT0, "orientation"), [1.0, 0.0, 0.0]),
    ((*BEAM, "segments"), "4"),
    ((*BEAM, "segments"), 4.0),
    ((*BEAM, "segments"), True),
    ((*BEAM, "root"), "fixed"),
    ((*BEAM, "end"), "pinned"),
    ((*BELT, "efficiency"), "0.9"),
    ((*BELT, "efficiency"), float("inf")),
    ((*SCREW, "self_locking"), "yes"),
    ((*SCREW, "self_locking"), 1),
    (("run", "iteration_cap"), True),
    (("run", "iteration_cap"), 50.0),
    (("schema_version",), "1"),
    (("bodies",), {"name": "link"}),
    (("signals", "pose_bodies"), "link"),
    ((*RAIL, "supports", 0, "condition"), "free"),
    ((*RAIL, "fasteners", 0), "M5 bolt"),
    ((*BODY0, "catalog_part"), 3),
]


@pytest.mark.req("BLD-030")
@pytest.mark.parametrize(("path", "value"), WRONG_TYPES)
def test_wrong_type_is_rejected(path: tuple[Any, ...], value: Any) -> None:
    _rejects(_set(_full(), path, value))


UNIT_ERRORS = [
    ((*BODY0, "mass"), "1"),
    ((*BODY0, "mass"), "1 m"),
    ((*BODY0, "mass"), "kg"),
    ((*BODY0, "com", 0), "0.25 kg"),
    ((*BODY0, "com", 0), "0.25"),
    ((*BODY0, "inertia", "ixx"), "5e-5 kg*m"),
    (("gravity", 1), "-9.81 m/s"),
    ((*PIVOT_DOF, "damping"), "0.045 N*s"),
    ((*PIVOT_DOF, "damping"), "0.045 N*m"),
    ((*PIVOT_DOF, "friction"), "1 N"),
    ((*SLIDE_DOF, "damping"), "1 N*m*s/rad"),
    ((*SLIDE_DOF, "lower"), "-0.5 rad"),
    ((*SLIDE_DOF, "friction"), "2 N*m"),
    ((*SECTION, "elastic_modulus"), "70 GN"),
    ((*SECTION, "second_moment_of_area"), "1e-8 m**3"),
    ((*SECTION, "density"), "2700 kg/m**2"),
    ((*BELT, "stiffness"), "1e5 N*m/rad"),
    ((*GEARS, "stiffness"), "1e4 N/m"),
    ((*GEARS, "backlash"), "0.1 mm"),
    ((*BELT, "backlash"), "0.1 deg"),
    (("run", "step"), "1"),
    (("run", "step"), "1 m"),
    (("run", "drift_threshold_linear"), "1e-6 rad"),
    (("run", "velocity_tolerance_linear"), "1e-9 rad/s"),
    (("run", "velocity_tolerance_linear"), "1e-9"),
    (("run", "velocity_tolerance_angular"), "1e-9 m/s"),
    (("run", "velocity_tolerance_angular"), "1e-9 rad"),
    ((*BEAM, "node_damping"), "0.1 N*m"),
    ((*BEAM, "node_damping"), "0.1 N*s/m"),
    (("contacts", "default", "stiffness"), "1e5 N*m/rad*m"),
    (("contacts", "default", "damping"), "100 N*s"),
]


@pytest.mark.req("BLD-030")
@pytest.mark.parametrize(("path", "value"), UNIT_ERRORS)
def test_missing_or_wrong_unit_is_rejected(path: tuple[Any, ...], value: Any) -> None:
    _rejects(_set(_full(), path, value))


# ===== Unknown keys and empty collections ===============================

LEVELS = [
    (),
    ("design",),
    ("run",),
    BODY0,
    (*BODY0, "inertia"),
    JOINT0,
    PIVOT_DOF,
    SLIDE_DOF,
    (*SHAPES, 0),
    (*SHAPES, 3),
    POINT,
    HINGE,
    BEAM,
    SECTION,
    BELT,
    SCREW,
    GEARS,
    ("signals",),
    RAIL,
    (*RAIL, "supports", 0),
    ("contacts",),
    ("contacts", "default"),
    ("contacts", "pairs", 0),
    ("world_shapes", 0),
]


@pytest.mark.req("BLD-030")
@pytest.mark.parametrize("path", LEVELS, ids=["/".join(str(p) for p in path) or "root" for path in LEVELS])
def test_unknown_key_is_rejected(path: tuple[Any, ...]) -> None:
    _rejects(_set(_full(), (*path, "colour"), "red"), "colour")


@pytest.mark.req("BLD-002")
@pytest.mark.req("BLD-009")
@pytest.mark.parametrize(
    ("path", "valid"),
    [
        (("bodies",), False),
        ((*SHAPES, 3, "vertices"), False),
        ((*RAIL, "supports"), False),
        ((*RAIL, "fasteners", 0), False),
        ((*RAIL, "fasteners"), True),
        (("constraints",), True),
        (("lumped_members",), True),
        (("structural_members",), True),
        (("contacts", "pairs", 0, "materials"), False),
        (("signals", "pose_bodies"), True),
        (("signals", "load_torque_joints"), True),
        (("world_shapes",), True),
        (("contacts", "pairs"), True),
        (("contacts", "exclusions"), True),
        (SHAPES, True),
        ((*JOINT0, "angular"), True),
        ((*CARRIAGE, "joint", "linear"), True),
    ],
)
def test_empty_collections(path: tuple[Any, ...], valid: bool) -> None:
    empty: Any = []
    if path == (*RAIL, "fasteners", 0):
        empty = {}
    data = _set(_full(), path, empty)
    if path == ("structural_members",):
        del data["lumped_members"][0]["structural_member"]
    if path == ("bodies",):
        data["constraints"] = []
    if valid:
        _model(data)
    else:
        _rejects(data)


@pytest.mark.req("BLD-002")
def test_transmissions_empty_is_valid_when_nothing_references_them() -> None:
    _model(_set(_full(), ("transmissions",), []))


# ===== Names and references =============================================


@pytest.mark.parametrize(
    ("path", "value", "message"),
    [
        ((*MOTOR, "name"), "link", "duplicate name 'link'"),
        ((*MOTOR, "joint", "name"), "pivot", "duplicate name 'pivot'"),
        ((*MOTOR, "joint", "name"), "link", "duplicate name 'link'"),
        ((*POINT, "name"), "carriage", "duplicate name 'carriage'"),
        ((*HINGE, "name"), "tie", "duplicate name 'tie'"),
        ((*BEAM, "name"), "belt", "duplicate name 'belt'"),
        ((*SCREW, "name"), "belt", "duplicate name 'belt'"),
        ((*BODY0, "name"), "world", "name 'world' is reserved"),
        ((*JOINT0, "name"), "world", "name 'world' is reserved"),
        (("signals", "pose_bodies"), ["link", "link"], "pose body 'link' listed twice"),
        (("signals", "load_torque_joints"), ["shaft", "shaft"], "load torque joint 'shaft' listed twice"),
    ],
)
@pytest.mark.req("BLD-030")
def test_duplicate_names_are_rejected(path: tuple[Any, ...], value: Any, message: str) -> None:
    data = _set(_full(), path, value)
    if path == (*BODY0, "name"):
        data["signals"]["pose_bodies"] = ["carriage"]
        data["constraints"] = []
    _rejects(data, message)


@pytest.mark.req("BLD-009")
def test_duplicate_structural_member_is_rejected() -> None:
    data = _full()
    data["structural_members"].append(copy.deepcopy(data["structural_members"][0]))
    _rejects(data, "duplicate structural member 'rail'")


DANGLING = [
    ((*MOTOR, "parent"), "nowhere", "parent 'nowhere' is not world or a body listed before it"),
    ((*MOTOR, "parent"), "carriage", "parent 'carriage' is not world or a body listed before it"),
    ((*MOTOR, "parent"), "motor", "parent 'motor' is not world or a body listed before it"),
    ((*POINT, "body_b"), "ghost", "constraint 'tie': unknown body 'ghost'"),
    ((*HINGE, "body_a"), "ghost", "constraint 'hinge': unknown body 'ghost'"),
    ((*HINGE, "body_a"), "motor", "constraint 'hinge': body_a and body_b are the same body"),
    ((*BEAM, "parent"), "ghost", "lumped member 'beam': unknown parent 'ghost'"),
    ((*BEAM, "structural_member"), "ghost", "lumped member 'beam': unknown structural member 'ghost'"),
    ((*BELT, "shaft_joint"), "ghost", "transmission 'belt': unknown shaft joint 'ghost'"),
    ((*BELT, "driven_joint"), "ghost", "transmission 'belt': unknown driven joint 'ghost'"),
    ((*BELT, "shaft_joint"), "link", "transmission 'belt': unknown shaft joint 'link'"),
    ((*BELT, "shaft_joint"), "slide", "shaft joint 'slide' must be revolute, is prismatic"),
    ((*BELT, "driven_joint"), "pivot", "driven joint 'pivot' must be prismatic, is revolute"),
    ((*SCREW, "driven_joint"), "shaft", "shaft and driven joint are the same joint"),
    ((*GEARS, "driven_joint"), "slide", "driven joint 'slide' must be revolute, is prismatic"),
    (("signals", "load_torque_joints"), ["ghost"], "signals: unknown load torque joint 'ghost'"),
    (("signals", "pose_bodies"), ["ghost"], "signals: unknown pose body 'ghost'"),
    (("signals", "pose_bodies"), ["world"], "signals: unknown pose body 'world'"),
]


@pytest.mark.req("BLD-021")
@pytest.mark.parametrize(("path", "value", "message"), DANGLING)
def test_dangling_references_are_rejected(path: tuple[Any, ...], value: Any, message: str) -> None:
    _rejects(_set(_full(), path, value), message)


@pytest.mark.req("BLD-021")
def test_every_reference_failure_is_reported_together() -> None:
    data = _set(_full(), (*BEAM, "parent"), "ghost")
    data = _set(data, ("signals", "pose_bodies"), ["phantom"])
    with pytest.raises(ValidationError) as caught:
        _model(data)
    assert "unknown parent 'ghost'" in str(caught.value)
    assert "unknown pose body 'phantom'" in str(caught.value)


@pytest.mark.req("BLD-021")
def test_body_without_material_or_joint_reports_both() -> None:
    data = _set(_set(_full(), (*MOTOR, "joint"), _MISSING), (*MOTOR, "material"), _MISSING)
    _rejects(data, "body 'motor' has no material; unconstrained body 'motor'")


# ===== Collision shapes =================================================


@pytest.mark.req("BLD-012")
@pytest.mark.parametrize("kind", ["mesh", "plane", "heightfield", "ellipsoid", "sdf", ""])
def test_disallowed_collision_shape_is_rejected(kind: str) -> None:
    _rejects(_set(_full(), (*SHAPES, 0, "shape"), kind))


@pytest.mark.req("BLD-012")
def test_convex_hull_must_span_a_volume() -> None:
    coplanar = [_vec("0 m", "0 m", "0 m"), _vec("1 m", "0 m", "0 m"), _vec("0 m", "1 m", "0 m")]
    coplanar.append(_vec("1 m", "1 m", "0 m"))
    _rejects(_set(_full(), (*SHAPES, 3, "vertices"), coplanar), "coplanar")
    three = coplanar[:3]
    _rejects(_set(_full(), (*SHAPES, 3, "vertices"), three))
    many = [*coplanar, _vec("0 m", "0 m", "1 m")]
    _model(_set(_full(), (*SHAPES, 3, "vertices"), many))


# ===== Joint types ======================================================

DOF_CASES = [
    ("revolute", 0, 1),
    ("prismatic", 1, 0),
    ("fixed", 0, 0),
    ("planar", 2, 1),
    ("spherical", 0, 3),
    ("free", 3, 3),
]


def _chain() -> dict[str, Any]:
    data = _example()
    template = copy.deepcopy(data["bodies"][0])
    template["shapes"] = []
    parent = "world"
    bodies = []
    for joint_type, linear, angular in DOF_CASES:
        body = copy.deepcopy(template)
        body["name"] = f"{joint_type}_body"
        body["parent"] = parent
        body["joint"] = {
            "name": f"{joint_type}_joint",
            "type": joint_type,
            "position": _vec("0.1 m", "0 m", "0 m"),
            "linear": [{"damping": "1 N*s/m"} for _ in range(linear)],
            "angular": [{"damping": "1 N*m*s/rad"} for _ in range(angular)],
        }
        bodies.append(body)
        parent = body["name"]
    data["bodies"] = bodies
    data["signals"] = {}
    return data


@pytest.mark.req("BLD-003")
def test_all_six_joint_types_with_full_dof_lists() -> None:
    model = _model(_chain())
    assert [body.joint.type for body in model.bodies if body.joint is not None] == [c[0] for c in DOF_CASES]
    assert set(neutral.DOF_COUNTS) == {c[0] for c in DOF_CASES}


@pytest.mark.req("BLD-003")
@pytest.mark.parametrize(("index", "case"), list(enumerate(DOF_CASES)), ids=[c[0] for c in DOF_CASES])
def test_dof_list_length_must_match_joint_type(index: int, case: tuple[str, int, int]) -> None:
    joint_type, linear, angular = case
    joint_path = ("bodies", index, "joint")
    for field, count, entry in (("linear", linear, {"rest": "0 m"}), ("angular", angular, {"rest": "0 rad"})):
        _model(_set(_chain(), (*joint_path, field), []))
        _rejects(_set(_chain(), (*joint_path, field), [entry] * (count + 1)), f"joint '{joint_type}_joint'")
        if count > 1:
            _rejects(_set(_chain(), (*joint_path, field), [entry] * (count - 1)))


# ===== Determinism and SI content =======================================


@pytest.mark.req("BLD-030")
def test_serialization_is_deterministic() -> None:
    first = _model(_full())
    second = _model(_full())
    assert neutral.to_yaml(first) == neutral.to_yaml(first)
    assert neutral.to_yaml(first) == neutral.to_yaml(second)
    assert neutral.from_yaml(neutral.to_yaml(first)) == first
    assert neutral.schema_text() == neutral.schema_text()


def _leaves(node: Any, path: str) -> list[tuple[str, Any]]:
    if isinstance(node, BaseModel):
        found: list[tuple[str, Any]] = []
        for name in type(node).model_fields:
            found += _leaves(getattr(node, name), f"{path}.{name}")
        return found
    if isinstance(node, tuple):
        found = []
        for index, item in enumerate(node):
            found += _leaves(item, f"{path}[{index}]")
        return found
    return [(path, node)]


@pytest.mark.req("BLD-030")
def test_model_holds_only_plain_si_values() -> None:
    model = _model(_full())
    for path, leaf in _leaves(model, "model"):
        if ".fasteners[" in path:
            assert isinstance(leaf, dict)
            continue
        assert leaf is None or type(leaf) in (float, int, str, bool), f"{path}: {type(leaf).__name__}"


@pytest.mark.req("BLD-030")
def test_model_is_immutable() -> None:
    model = _model(_example())
    with pytest.raises(ValidationError):
        model.bodies[0].mass = 2.0  # type: ignore[misc]


@pytest.mark.req("INV-013")
def test_write_schema_cli(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    target = tmp_path / "physics_model.json"
    monkeypatch.setattr(neutral, "SCHEMA_PATH", target)
    assert neutral.main(["--write-schema"]) == 0
    assert target.read_text(encoding="utf-8") == neutral.schema_text()
    assert neutral.main([]) == 2


# ===== Joint tiers, contacts, names ====================================

SPRINGLESS = [
    ("spherical", 4, "angular", {"stiffness": "1 N*m/rad"}, "takes no stiffness or rest position"),
    ("spherical", 4, "angular", {"rest": "0.1 rad"}, "takes no stiffness or rest position"),
    ("spherical", 4, "angular", {"lower": "-1 rad"}, "takes no limits"),
    ("spherical", 4, "angular", {"upper": "1 rad"}, "takes no limits"),
    ("free", 5, "angular", {"stiffness": "1 N*m/rad"}, "takes no stiffness or rest position"),
    ("free", 5, "angular", {"rest": "0.1 rad"}, "takes no stiffness or rest position"),
    ("free", 5, "angular", {"lower": "-1 rad"}, "takes no limits"),
    ("free", 5, "angular", {"upper": "1 rad"}, "takes no limits"),
    ("free", 5, "linear", {"stiffness": "1 N/m"}, "takes no stiffness or rest position"),
    ("free", 5, "linear", {"rest": "0.1 m"}, "takes no stiffness or rest position"),
    ("free", 5, "linear", {"lower": "-1 m"}, "takes no limits"),
    ("free", 5, "linear", {"upper": "1 m"}, "takes no limits"),
]


@pytest.mark.req("BLD-003")
@pytest.mark.parametrize(("joint_type", "index", "field", "entry", "message"), SPRINGLESS)
def test_spherical_and_free_joints_reject_springs_rest_and_limits(
    joint_type: str, index: int, field: str, entry: dict[str, str], message: str
) -> None:
    data = _chain()
    dofs = data["bodies"][index]["joint"][field]
    dofs[1] = {**dofs[1], **entry}
    _rejects(data, f"joint '{joint_type}_joint' ({joint_type}) {message}")


@pytest.mark.req("BLD-003")
@pytest.mark.parametrize(
    ("index", "field", "entry"),
    [
        (0, "angular", {"stiffness": "1 N*m/rad", "rest": "0.1 rad", "lower": "-1 rad", "upper": "1 rad"}),
        (1, "linear", {"stiffness": "1 N/m", "rest": "0.1 m", "lower": "-1 m", "upper": "1 m"}),
        (3, "linear", {"stiffness": "1 N/m", "rest": "0.1 m", "lower": "-1 m", "upper": "1 m"}),
        (3, "angular", {"stiffness": "1 N*m/rad", "rest": "0.1 rad", "lower": "-1 rad", "upper": "1 rad"}),
        (4, "angular", {"friction": "0.1 N*m", "damping": "0.1 N*m*s/rad"}),
        (5, "linear", {"friction": "0.1 N", "damping": "0.1 N*s/m"}),
    ],
)
def test_springs_rest_and_limits_kept_where_the_engine_has_them(index: int, field: str, entry: dict[str, str]) -> None:
    data = _chain()
    dofs = data["bodies"][index]["joint"][field]
    dofs[0] = {**dofs[0], **entry}
    _model(data)


@pytest.mark.req("BLD-012")
def test_plane_is_allowed_only_on_the_world() -> None:
    model = _model(_full())
    assert type(model.world_shapes[0]).__name__ == "Plane"
    plane = {"shape": "plane", "material": "concrete"}
    _rejects(_set(_full(), (*SHAPES, 0), plane), "body 'carriage': a plane shape is allowed only on the world")


@pytest.mark.req("BLD-012")
@pytest.mark.parametrize(
    ("path", "value", "message"),
    [
        (("contacts", "exclusions"), [["link", "ghost"]], "contacts: exclusion names unknown body 'ghost'"),
        (("contacts", "exclusions"), [["ghost", "world"]], "contacts: exclusion names unknown body 'ghost'"),
        (("contacts", "exclusions"), [["link", "link"]], "contacts: exclusion pairs body 'link' with itself"),
        (("contacts", "exclusions"), [["link", "world"], ["world", "link"]], "exclusion link/world listed twice"),
        (("contacts", "exclusions"), [["link", "motor"], ["carriage", "world"]], ""),
    ],
)
def test_contact_exclusions_name_known_distinct_bodies(path: tuple[Any, ...], value: Any, message: str) -> None:
    data = _set(_full(), path, value)
    if message == "":
        assert _model(data).contacts is not None
    else:
        _rejects(data, message)


@pytest.mark.req("BLD-012")
def test_material_pair_listed_once_in_either_order() -> None:
    data = _full()
    pair = copy.deepcopy(data["contacts"]["pairs"][0])
    pair["materials"] = ["concrete", "steel"]
    data["contacts"]["pairs"].append(pair)
    _rejects(data, "material pair concrete/steel listed twice")


@pytest.mark.req("BLD-030")
def test_contacts_convert_to_si() -> None:
    contacts = _model(_full()).contacts
    assert contacts is not None
    assert contacts.default.stiffness == 1e5
    assert contacts.default.damping == 100.0
    assert contacts.pairs[0].materials == ("steel", "concrete")
    assert contacts.exclusions == (("link", "world"),)


@pytest.mark.req("BLD-020")
def test_every_material_name_in_the_example_matches_the_catalog_pattern() -> None:
    pattern = re.compile(r"^[a-z][a-z0-9_]*$")
    model = _model(_example())
    names = [body.material for body in model.bodies]
    names += [shape.material for body in model.bodies for shape in body.shapes]
    names += [shape.material for shape in model.world_shapes]
    assert names
    for name in names:
        assert name is not None
        assert pattern.match(name), name


@pytest.mark.req("INV-013")
def test_schema_declares_draft_and_id() -> None:
    schema = json.loads(neutral.schema_text())
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert schema["$id"] == "https://mechsim.local/schemas/physics_model.json"
    run = schema["$defs"]["RunConfig"]["properties"]
    assert run["step"]["default"] == "0.001 s"
    assert run["velocity_tolerance_angular"]["default"] == "1e-09 rad/s"
