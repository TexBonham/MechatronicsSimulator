"""Neutral model to MJCF translation (ADR 0014): structure per element kind, refusals, determinism.

The `mujoco` Python package is not a dependency; the tests parse the emitted XML and check its structure against
the mapping table in `mechsim/physics/mujoco/CLAUDE.md`. The C++ adapter's tests compile the pendulum fixture.
"""

from __future__ import annotations

import copy
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import pytest
from ruamel.yaml import YAML

from mechsim.physics import neutral
from mechsim.physics.mujoco import translate as mj

ROOT = Path(__file__).resolve().parents[2]
EXAMPLE = ROOT / "schemas" / "examples" / "physics_model.pendulum.yaml"
FIXTURES = ROOT / "core" / "physics" / "mujoco" / "tests" / "fixtures"
ADAPTER_SOURCE = ROOT / "core" / "physics" / "mujoco" / "src" / "mujoco_adapter.cpp"

INERTIA = {"ixx": "1e-4 kg*m**2", "iyy": "1e-4 kg*m**2", "izz": "1e-4 kg*m**2"}


def _example() -> dict[str, Any]:
    data: dict[str, Any] = YAML(typ="safe", pure=True).load(EXAMPLE.read_text(encoding="utf-8"))
    return data


def _body(name: str, joint: dict[str, Any], parent: str = "world", **extra: Any) -> dict[str, Any]:
    body: dict[str, Any] = {
        "name": name,
        "mass": "2 kg",
        "com": ["0 m", "0 m", "0 m"],
        "inertia": dict(INERTIA),
        "material": "steel",
        "parent": parent,
        "joint": joint,
    }
    body.update(extra)
    return body


def _joint(name: str, kind: str, **extra: Any) -> dict[str, Any]:
    joint: dict[str, Any] = {"name": name, "type": kind, "position": ["0 m", "0 m", "0 m"]}
    joint.update(extra)
    return joint


def _section() -> dict[str, Any]:
    return {
        "elastic_modulus": "2e11 Pa",
        "second_moment_of_area": "1e-8 m**4",
        "density": "7850 kg/m**3",
        "area": "1e-4 m**2",
        "length": "1 m",
    }


def _full() -> dict[str, Any]:
    """The pendulum plus one element of every kind the translation maps."""
    data = _example()
    data["bodies"] += [
        _body(
            "motor",
            _joint(
                "shaft",
                "revolute",
                angular=[
                    {
                        "lower": "-1 rad",
                        "upper": "2 rad",
                        "friction": "0.01 N*m",
                        "damping": "0.002 N*m*s/rad",
                        "stiffness": "0.5 N*m/rad",
                        "rest": "0.1 rad",
                    }
                ],
            ),
            shapes=[{"shape": "sphere", "radius": "0.02 m", "material": "steel"}],
        ),
        _body(
            "carriage",
            _joint("slide", "prismatic", axis=[1.0, 0.0, 0.0], linear=[{"lower": "0 m", "upper": "0.5 m"}]),
            shapes=[{"shape": "box", "size": ["0.1 m", "0.2 m", "0.3 m"], "material": "aluminium"}],
        ),
        _body("bracket", _joint("weld", "fixed", position=["0 m", "1 m", "0 m"]), parent="carriage"),
        _body(
            "puck",
            _joint(
                "glide",
                "planar",
                axis=[0.0, 0.0, 1.0],
                linear=[{"damping": "1 N*s/m"}, {"damping": "2 N*s/m"}],
                angular=[{"damping": "0.5 N*m*s/rad"}],
            ),
            shapes=[{"shape": "capsule", "radius": "0.01 m", "length": "0.1 m", "material": "steel"}],
        ),
        _body(
            "ball",
            _joint("socket", "spherical", angular=[{"damping": "0.1 N*m*s/rad"}] * 3),
            parent="bracket",
            shapes=[{"shape": "cylinder", "radius": "0.01 m", "length": "0.1 m", "material": "steel"}],
        ),
        _body(
            "payload",
            _joint("float", "free", position=["0 m", "0 m", "1 m"]),
            shapes=[
                {
                    "shape": "convex_hull",
                    "vertices": [
                        ["0 m", "0 m", "0 m"],
                        ["0.1 m", "0 m", "0 m"],
                        ["0 m", "0.1 m", "0 m"],
                        ["0 m", "0 m", "0.1 m"],
                    ],
                    "material": "steel",
                }
            ],
        ),
    ]
    data["bodies"].append(
        _body("rotor", _joint("spindle", "revolute", angular=[{"damping": "0.004 N*m*s/rad"}]), parent="motor")
    )
    data["world_shapes"] = [{"shape": "plane", "material": "concrete"}]
    data["contacts"] = {
        "default": {"stiffness": "1e5 N/m", "damping": "100 N*s/m", "friction": 0.5},
        "pairs": [
            {"materials": ["steel", "concrete"], "stiffness": "4e5 N/m", "damping": "200 N*s/m", "friction": 0.8}
        ],
        "exclusions": [["motor", "carriage"]],
    }
    data["constraints"] = [
        {
            "type": "point",
            "name": "tie",
            "body_a": "link",
            "body_b": "motor",
            "anchor_a": ["0.5 m", "0 m", "0 m"],
            "anchor_b": ["0 m", "0.1 m", "0 m"],
        },
        {
            "type": "hinge",
            "name": "door",
            "body_a": "world",
            "body_b": "puck",
            "anchor_a": ["0 m", "0 m", "0 m"],
            "anchor_b": ["0 m", "0 m", "0 m"],
            "axis_a": [0.0, 0.0, 2.0],
            "axis_b": [0.0, 0.0, 1.0],
        },
    ]
    data["lumped_members"] = [
        {
            "name": "beam",
            "parent": "world",
            "position": ["0 m", "2 m", "0 m"],
            "section": _section(),
            "root": "clamped",
            "end": "free",
            "node_damping": "0.01 N*m*s/rad",
        },
        {
            "name": "span",
            "parent": "carriage",
            "position": ["0 m", "0 m", "1 m"],
            "orientation": [0.7071067811865476, 0.0, 0.0, 0.7071067811865476],
            "section": _section(),
            "root": "pinned",
            "end": "roller",
        },
    ]
    data["transmissions"] = [
        {
            "type": "belt_pulley",
            "name": "belt",
            "shaft_joint": "spindle",
            "driven_joint": "slide",
            "efficiency": 0.95,
            "pulley_radius": "0.02 m",
            "stiffness": "1e5 N/m",
        },
    ]
    return data


def _mechanism() -> dict[str, Any]:
    """C++ fixture: a horizontal arm held at its tip by a point constraint, and a soft clamped beam (PHY-008)."""
    data = _example()
    data["design"]["name"] = "mechanism"
    arm = data["bodies"][0]
    arm["name"] = "arm"
    arm["joint"] = _joint("hinge", "revolute")
    arm["shapes"] = []
    data["signals"] = {"pose_bodies": ["arm"]}
    data["constraints"] = [
        {
            "type": "point",
            "name": "tie",
            "body_a": "arm",
            "body_b": "world",
            "anchor_a": ["0.5 m", "0 m", "0 m"],
            "anchor_b": ["0.5 m", "0 m", "0 m"],
        }
    ]
    data["lumped_members"] = [
        {
            "name": "beam",
            "parent": "world",
            "position": ["0 m", "1 m", "0 m"],
            "section": {
                "elastic_modulus": "1e8 Pa",
                "second_moment_of_area": "1e-7 m**4",
                "density": "1000 kg/m**3",
                "area": "1e-4 m**2",
                "length": "1 m",
            },
            "root": "clamped",
            "end": "free",
            "node_damping": "1 N*m*s/rad",
        }
    ]
    return data


def _box_on_floor() -> dict[str, Any]:
    """C++ fixture: a 1 kg, 0.1 m cube on a free joint resting on the floor plane (contact outputs, PHY-003)."""
    data = _example()
    data["design"]["name"] = "box_on_floor"
    data["gravity"] = ["0 m/s**2", "0 m/s**2", "-9.81 m/s**2"]
    data["world_shapes"] = [{"shape": "plane", "material": "concrete"}]
    data["contacts"] = {"default": {"stiffness": "1e5 N/m", "damping": "632 N*s/m", "friction": 0.5}}
    side = "0.0016666666666666668 kg*m**2"
    data["bodies"] = [
        {
            "name": "box",
            "mass": "1 kg",
            "com": ["0 m", "0 m", "0 m"],
            "inertia": {"ixx": side, "iyy": side, "izz": side},
            "material": "steel",
            "parent": "world",
            "joint": _joint("float", "free", position=["0 m", "0 m", "0.05 m"]),
            "shapes": [{"shape": "box", "size": ["0.1 m", "0.1 m", "0.1 m"], "material": "steel"}],
        }
    ]
    data["signals"] = {"pose_bodies": ["box"]}
    return data


def _lead_screw(efficiency: float) -> dict[str, Any]:
    """C++ fixture: a light rotor (J = 5.4e-6 kg m^2) driving a 10 kg carriage through an 8 mm lead screw."""
    data = _example()
    data["design"]["name"] = "lead_screw"
    data["gravity"] = ["0 m/s**2", "0 m/s**2", "-9.81 m/s**2"]
    rotor = "5.4e-6 kg*m**2"
    data["bodies"] = [
        _body("rotor", _joint("screw", "revolute"), inertia={"ixx": rotor, "iyy": rotor, "izz": rotor}, mass="0.2 kg"),
        _body("carriage", _joint("rail", "prismatic", axis=[1.0, 0.0, 0.0]), mass="10 kg"),
    ]
    data["signals"] = {"load_torque_joints": ["screw"], "pose_bodies": ["carriage"]}
    data["transmissions"] = [
        {
            "type": "lead_screw",
            "name": "drive",
            "shaft_joint": "screw",
            "driven_joint": "rail",
            "efficiency": efficiency,
            "lead": "0.008 m",
            "stiffness": "1e8 N/m",
        }
    ]
    return data


def _lift_screw() -> dict[str, Any]:
    """C++ fixture: the lead screw turned vertical, so gravity loads the carriage along the rail (PHY-034)."""
    data = _lead_screw(1.0)
    data["design"]["name"] = "lift_screw"
    data["bodies"][1]["joint"] = _joint("rail", "prismatic", axis=[0.0, 0.0, 1.0])
    return data


def _slider() -> dict[str, Any]:
    """C++ fixture: a 1 kg body on a drivable prismatic joint along x, no gravity (PHY-034)."""
    data = _example()
    data["design"]["name"] = "slider"
    data["gravity"] = ["0 m/s**2", "0 m/s**2", "0 m/s**2"]
    data["bodies"] = [_body("block", _joint("slide", "prismatic", axis=[1.0, 0.0, 0.0]), mass="1 kg")]
    data["signals"] = {"pose_bodies": ["block"], "drive_joints": ["slide"]}
    return data


def _gear(ratio: float) -> dict[str, Any]:
    """C++ fixture: the same rotor driving a wheel (izz 0.01 kg m^2) about z through a rigid gear pair, no gravity."""
    data = _example()
    data["design"]["name"] = "gear_pair"
    data["gravity"] = ["0 m/s**2", "0 m/s**2", "0 m/s**2"]
    rotor = "5.4e-6 kg*m**2"
    wheel = "0.01 kg*m**2"
    data["bodies"] = [
        _body(
            "rotor", _joint("spindle", "revolute"), inertia={"ixx": rotor, "iyy": rotor, "izz": rotor}, mass="0.2 kg"
        ),
        _body("wheel", _joint("axle", "revolute"), inertia={"ixx": wheel, "iyy": wheel, "izz": wheel}, mass="1 kg"),
    ]
    data["signals"] = {"load_torque_joints": ["spindle"], "pose_bodies": ["wheel"]}
    data["transmissions"] = [
        {
            "type": "gear_pair",
            "name": "gears",
            "shaft_joint": "spindle",
            "driven_joint": "axle",
            "efficiency": 1.0,
            "ratio": ratio,
            "stiffness": "1e6 N*m/rad",
        }
    ]
    return data


def fixtures() -> dict[str, str]:
    """The MJCF fixtures committed under core/physics/mujoco/tests/fixtures/, by file name."""
    return {
        "pendulum.xml": mj.translate(neutral.from_yaml(EXAMPLE.read_text(encoding="utf-8"))),
        "mechanism.xml": mj.translate(_model(_mechanism())),
        "box_on_floor.xml": mj.translate(_model(_box_on_floor())),
        "lead_screw.xml": mj.translate(_model(_lead_screw(1.0))),
        "lead_screw_lossy.xml": mj.translate(_model(_lead_screw(0.8))),
        "lift_screw.xml": mj.translate(_model(_lift_screw())),
        "slider.xml": mj.translate(_model(_slider())),
        "gear_pair.xml": mj.translate(_model(_gear(4.0))),
        "gear_pair_reversed.xml": mj.translate(_model(_gear(-4.0))),
    }


def _model(data: dict[str, Any]) -> neutral.PhysicsModel:
    return neutral.PhysicsModel.model_validate(data)


def _xml(data: dict[str, Any]) -> ET.Element:
    return ET.fromstring(mj.translate(_model(data)))


def _floats(text: str | None) -> list[float]:
    assert text is not None
    return [float(token) for token in text.split()]


def _named(root: ET.Element, tag: str, name: str) -> ET.Element:
    found = root.find(f".//{tag}[@name='{name}']")
    assert found is not None, f"no <{tag} name='{name}'>"
    return found


def _custom(root: ET.Element, name: str) -> list[str]:
    text = root.find(f"./custom/text[@name='{name}']")
    if text is None:
        return []
    return text.get("data", "").split()


# ===== Model level ======================================================


@pytest.mark.req("PHY-013")
@pytest.mark.req("PHY-014")
def test_pendulum_translates_to_loadable_mjcf_structure() -> None:
    root = _xml(_example())
    assert root.tag == "mujoco"
    assert root.get("model") == "pendulum"
    compiler = root.find("compiler")
    assert compiler is not None
    assert compiler.get("inertiafromgeom") == "false"
    assert compiler.get("angle") == "radian"
    option = root.find("option")
    assert option is not None
    assert option.get("integrator") == "implicitfast"
    assert float(option.get("timestep", "")) == 1e-3
    assert _floats(option.get("gravity")) == [0.0, -9.81, 0.0]
    assert option.get("iterations") == "50"
    world = root.find("worldbody")
    assert world is not None
    link = _named(root, "body", "link")
    assert link in list(world)
    assert _floats(link.get("quat")) == [0.7071067811865476, 0.0, 0.0, -0.7071067811865476]
    inertial = link.find("inertial")
    assert inertial is not None
    assert _floats(inertial.get("pos")) == [0.25, 0.0, 0.0]
    assert float(inertial.get("mass", "")) == 1.0
    assert _floats(inertial.get("fullinertia")) == [5e-5, 0.020833333333333332, 0.020833333333333332, 0.0, 0.0, 0.0]
    pivot = _named(link, "joint", "pivot")
    assert pivot.get("type") == "hinge"
    assert _floats(pivot.get("axis")) == [0.0, 0.0, 1.0]
    assert float(pivot.get("damping", "")) == 0.04520785330006282
    geom = _named(link, "geom", "link.shape0")
    assert geom.get("type") == "cylinder"
    assert _floats(geom.get("size")) == [0.01, 0.25]
    assert _custom(root, "mechsim:bodies") == ["link"]
    assert _custom(root, "mechsim:joints") == ["pivot:revolute"]
    assert _custom(root, "mechsim:elements") == ["body", "joint.revolute", "shape.cylinder"]


@pytest.mark.req("PHY-014")
@pytest.mark.req("PHY-031")
def test_step_override_sets_timestep_and_equality_reference() -> None:
    data = _full()
    root = ET.fromstring(mj.translate(_model(data), step=5e-4))
    option = root.find("option")
    assert option is not None
    assert float(option.get("timestep", "")) == 5e-4
    assert _floats(_named(root, "connect", "tie").get("solref")) == [1e-3, 1.0]


@pytest.mark.req("PHY-013")
@pytest.mark.parametrize(
    "name",
    [
        "pendulum.xml",
        "mechanism.xml",
        "box_on_floor.xml",
        "lead_screw.xml",
        "lead_screw_lossy.xml",
        "lift_screw.xml",
        "slider.xml",
        "gear_pair.xml",
        "gear_pair_reversed.xml",
    ],
)
def test_committed_cpp_fixtures_are_current_translations(name: str) -> None:
    assert (FIXTURES / name).read_text(encoding="utf-8") == fixtures()[name]


@pytest.mark.req("PHY-013")
@pytest.mark.req("PHY-010")
def test_translation_is_deterministic() -> None:
    first = mj.translate(_model(_full()))
    second = mj.translate(_model(copy.deepcopy(_full())))
    assert first == second
    assert first.endswith("\n")


@pytest.mark.req("PHY-019")
def test_supported_elements_mirror_the_cpp_adapter() -> None:
    source = ADAPTER_SOURCE.read_text(encoding="utf-8")
    block = re.search(r"// ===== Supported elements =+\n(.*?)// ===== End supported elements", source, re.DOTALL)
    assert block is not None
    assert tuple(re.findall(r'"([a-z_.]+)"', block.group(1))) == mj.supported_elements()


@pytest.mark.req("PHY-017")
@pytest.mark.req("PHY-018")
def test_full_model_maps_every_supported_element_kind() -> None:
    root = _xml(_full())
    # The other two transmission kinds have their own tests below; one driven joint takes one transmission here.
    other_transmissions = {"transmission.gear_pair", "transmission.lead_screw"}
    assert _custom(root, "mechsim:elements") == sorted(set(mj.supported_elements()) - other_transmissions)
    assert _custom(root, "mechsim:bodies") == [
        "link",
        "motor",
        "carriage",
        "bracket",
        "puck",
        "ball",
        "payload",
        "rotor",
    ]
    assert _custom(root, "mechsim:joints") == [
        "pivot:revolute",
        "shaft:revolute",
        "slide:prismatic",
        "weld:fixed",
        "glide:planar",
        "socket:spherical",
        "float:free",
        "spindle:revolute",
    ]
    assert _custom(root, "mechsim:constraints") == ["tie:point", "door:hinge"]
    assert _custom(root, "mechsim:members") == ["beam:world:clamped:free:4", "span:carriage:pinned:roller:8"]
    assert _custom(root, "mechsim:transmissions") == ["belt:rotor:motor:slide"]


@pytest.mark.req("PHY-017")
def test_no_actuator_tendon_or_sensor_is_added() -> None:
    root = _xml(_full())
    for tag in ("actuator", "tendon", "sensor", "keyframe", "flex", "deformable"):
        assert root.find(tag) is None


# ===== Joints ===========================================================


@pytest.mark.req("PHY-027")
def test_revolute_dof_carries_limits_friction_damping_and_spring() -> None:
    joint = _named(_xml(_full()), "joint", "shaft")
    assert joint.get("type") == "hinge"
    assert joint.get("limited") == "true"
    assert _floats(joint.get("range")) == [-1.0, 2.0]
    assert float(joint.get("frictionloss", "")) == 0.01
    assert float(joint.get("damping", "")) == 0.002
    assert float(joint.get("stiffness", "")) == 0.5
    assert float(joint.get("springref", "")) == 0.1
    assert _floats(joint.get("solreflimit")) == [2e-3, 1.0]


@pytest.mark.req("PHY-027")
def test_prismatic_maps_to_slide_with_range() -> None:
    joint = _named(_xml(_full()), "joint", "slide")
    assert joint.get("type") == "slide"
    assert _floats(joint.get("axis")) == [1.0, 0.0, 0.0]
    assert _floats(joint.get("range")) == [0.0, 0.5]
    assert joint.get("frictionloss") is None


def test_fixed_joint_welds_the_body_with_no_joint() -> None:
    root = _xml(_full())
    bracket = _named(root, "body", "bracket")
    assert bracket.find("joint") is None
    assert bracket in list(_named(root, "body", "carriage"))
    assert _floats(bracket.get("pos")) == [0.0, 1.0, 0.0]


def test_planar_maps_to_two_slides_and_a_hinge_in_the_engine_basis() -> None:
    puck = _named(_xml(_full()), "body", "puck")
    joints = puck.findall("joint")
    assert [joint.get("name") for joint in joints] == ["glide.x", "glide.y", "glide.theta"]
    assert [joint.get("type") for joint in joints] == ["slide", "slide", "hinge"]
    # The engine's plane basis for n = z: smallest |n_i| is x, u = n x e_x = y, w = n x u = -x; u x w = n.
    assert _floats(joints[0].get("axis")) == [0.0, 1.0, 0.0]
    assert _floats(joints[1].get("axis")) == [-1.0, 0.0, 0.0]
    assert _floats(joints[2].get("axis")) == [0.0, 0.0, 1.0]
    assert [float(joint.get("damping", "")) for joint in joints] == [1.0, 2.0, 0.5]


def test_plane_basis_is_orthonormal_and_right_handed_for_any_normal() -> None:
    for normal in ((1.0, 0.0, 0.0), (0.0, -3.0, 0.0), (1.0, 2.0, 3.0), (-0.2, 0.1, -5.0)):
        u, w = mj._plane_basis(normal)
        norm = math.sqrt(sum(component * component for component in normal))
        n = [component / norm for component in normal]
        cross = mj._cross(u, w)
        for i in range(3):
            assert cross[i] == pytest.approx(n[i], abs=1e-15)
        assert sum(a * b for a, b in zip(u, n, strict=True)) == pytest.approx(0.0, abs=1e-15)
        assert sum(a * a for a in u) == pytest.approx(1.0, abs=1e-15)


def test_spherical_and_free_map_to_ball_and_free_with_uniform_damping() -> None:
    root = _xml(_full())
    socket = _named(root, "joint", "socket")
    assert socket.get("type") == "ball"
    assert float(socket.get("damping", "")) == 0.1
    assert socket.get("stiffness") is None
    free = _named(root, "joint", "float")
    assert free.get("type") == "free"
    assert free.get("damping") is None


def test_one_sided_limit_is_unmappable_naming_the_joint() -> None:
    data = _full()
    data["bodies"][2]["joint"]["linear"] = [{"lower": "0 m"}]
    with pytest.raises(mj.UnmappableElementError, match=r"joint 'slide'.*one-sided") as raised:
        mj.translate(_model(data))
    assert raised.value.element == "joint 'slide'"


def test_upper_only_limit_is_unmappable() -> None:
    data = _full()
    data["bodies"][2]["joint"]["linear"] = [{"upper": "1 m"}]
    with pytest.raises(mj.UnmappableElementError, match=r"joint 'slide'"):
        mj.translate(_model(data))


@pytest.mark.parametrize("index", [5, 6])
def test_non_uniform_damping_on_ball_or_free_is_unmappable(index: int) -> None:
    data = _full()
    joint = data["bodies"][index]["joint"]
    count = 3
    if joint["type"] == "free":
        joint["linear"] = [{"damping": "1 N*s/m"}] * 3
        count = 3
    joint["angular"] = [{"damping": "0.1 N*m*s/rad"}] * (count - 1) + [{"damping": "0.2 N*m*s/rad"}]
    with pytest.raises(mj.UnmappableElementError, match=rf"joint '{joint['name']}'.*one value per joint"):
        mj.translate(_model(data))


def test_non_uniform_friction_on_ball_is_unmappable() -> None:
    data = _full()
    data["bodies"][5]["joint"]["angular"] = [{"friction": "0 N*m"}, {"friction": "0 N*m"}, {"friction": "1 N*m"}]
    with pytest.raises(mj.UnmappableElementError, match=r"joint 'socket'"):
        mj.translate(_model(data))


def test_free_joint_with_uniform_damping_maps_once() -> None:
    data = _full()
    joint = data["bodies"][6]["joint"]
    joint["linear"] = [{"damping": "0.3 N*s/m", "friction": "0.1 N"}] * 3
    joint["angular"] = [{"damping": "0.3 N*m*s/rad", "friction": "0.1 N*m"}] * 3
    free = _named(_xml(data), "joint", "float")
    assert float(free.get("damping", "")) == 0.3
    assert float(free.get("frictionloss", "")) == 0.1


def test_free_joint_below_a_body_is_unmappable() -> None:
    data = _full()
    data["bodies"][6]["parent"] = "carriage"
    with pytest.raises(mj.UnmappableElementError, match=r"joint 'float'.*free joints only on world"):
        mj.translate(_model(data))


def test_unknown_joint_type_is_unmappable() -> None:
    model = _model(_example())
    joint = model.bodies[0].joint
    assert joint is not None
    odd = joint.model_construct(**{**dict(joint), "type": "screw"})
    body = model.bodies[0].model_construct(**{**dict(model.bodies[0]), "joint": odd})
    broken = model.model_construct(**{**dict(model), "bodies": (body,)})
    with pytest.raises(mj.UnmappableElementError, match=r"joint 'pivot'.*'screw'"):
        mj.translate(broken)


# ===== Shapes and contacts ==============================================


def test_every_shape_maps_to_its_geom() -> None:
    root = _xml(_full())
    expected = {
        "motor.shape0": ("sphere", [0.02]),
        "carriage.shape0": ("box", [0.05, 0.1, 0.15]),
        "puck.shape0": ("capsule", [0.01, 0.05]),
        "ball.shape0": ("cylinder", [0.01, 0.05]),
        "world.shape0": ("plane", [0.0, 0.0, 1.0]),
    }
    for name, (kind, size) in expected.items():
        geom = _named(root, "geom", name)
        assert geom.get("type") == kind
        assert _floats(geom.get("size")) == size
        assert geom.get("contype") == "0"
        assert geom.get("conaffinity") == "0"
    hull = _named(root, "geom", "payload.shape0")
    assert hull.get("type") == "mesh"
    mesh = _named(root, "mesh", "payload.shape0")
    assert len(_floats(mesh.get("vertex"))) == 12
    world = root.find("worldbody")
    assert world is not None
    assert _named(root, "geom", "world.shape0") in list(world)


def test_contact_pairs_follow_defaults_overrides_and_exclusions() -> None:
    root = _xml(_full())
    pairs = root.findall("./contact/pair")
    keyed = {(pair.get("geom1"), pair.get("geom2")): pair for pair in pairs}
    assert ("motor.shape0", "carriage.shape0") not in keyed
    assert ("carriage.shape0", "motor.shape0") not in keyed
    # Six geoms on six bodies plus the floor: 7 choose 2 = 21 pairs; the motor-carriage exclusion and the four bodies
    # jointed to the world (link, motor, carriage, puck) against the floor leave 16.
    assert len(pairs) == 16
    for body in ("link", "motor", "carriage", "puck"):
        assert ("world.shape0", f"{body}.shape0") not in keyed
    floor_payload = keyed[("world.shape0", "payload.shape0")]
    # Steel on concrete takes the override; world mass is infinite, so the reduced mass is the payload's 2 kg.
    assert _floats(floor_payload.get("solref")) == [-4e5 / 2.0, -200.0 / 2.0]
    assert _floats(floor_payload.get("friction")) == [0.8, 0.8, 0.0, 0.0, 0.0]
    report = mj.translate_report(_model(_full())).report
    for body in ("carriage", "link", "motor", "puck"):
        assert f"contacts: bodies '{body}' and 'world' are joined by a joint; their shapes do not collide" in report
    steel_aluminium = keyed[("carriage.shape0", "puck.shape0")]
    # Default parameters; reduced mass of two 2 kg bodies is 1 kg.
    assert _floats(steel_aluminium.get("solref")) == [-1e5, -100.0]
    assert _floats(steel_aluminium.get("solimp"))[:2] == [mj.CONTACT_IMPEDANCE, mj.CONTACT_IMPEDANCE]
    assert steel_aluminium.get("condim") == "3"


def test_no_contact_model_emits_no_pairs() -> None:
    data = _full()
    del data["contacts"]
    root = _xml(data)
    assert root.find("contact") is None
    assert "contacts" not in _custom(root, "mechsim:elements")


def test_unknown_shape_is_unmappable() -> None:
    with pytest.raises(mj.UnmappableElementError, match=r"unknown shape type"):
        mj._shape_kind(object())


# ===== Constraints, members, transmissions ==============================


def test_point_constraint_is_a_site_connect_with_both_anchors() -> None:
    root = _xml(_full())
    connect = _named(root, "connect", "tie")
    assert (connect.get("site1"), connect.get("site2")) == ("tie.a", "tie.b")
    assert _floats(_named(_named(root, "body", "link"), "site", "tie.a").get("pos")) == [0.5, 0.0, 0.0]
    assert _floats(_named(_named(root, "body", "motor"), "site", "tie.b").get("pos")) == [0.0, 0.1, 0.0]
    assert _floats(connect.get("solref")) == [2e-3, 1.0]


def test_hinge_constraint_is_two_connects_along_the_unit_axes() -> None:
    root = _xml(_full())
    point = _named(root, "connect", "door.point")
    axis = _named(root, "connect", "door.axis")
    assert point.get("site1") == "door.point.a"
    assert axis.get("site2") == "door.axis.b"
    world = root.find("worldbody")
    assert world is not None
    assert _floats(_named(world, "site", "door.axis.a").get("pos")) == [0.0, 0.0, mj.HINGE_AXIS_OFFSET]
    assert _floats(_named(root, "site", "door.axis.b").get("pos")) == [0.0, 0.0, mj.HINGE_AXIS_OFFSET]


def test_unknown_constraint_is_unmappable_naming_it() -> None:
    with pytest.raises(mj.UnmappableElementError, match=r"constraint 'odd'"):
        mj._constraint(ET.Element("equality"), {}, _Named("odd"), "0.002 1")


class _Named:
    def __init__(self, name: str) -> None:
        self.name = name


@pytest.mark.req("PHY-018")
def test_clamped_member_is_a_midpoint_spring_chain() -> None:
    root = _xml(_full())
    # l = L / N = 0.25 m; k = E I / l = 2e11 * 1e-8 / 0.25 = 8000 N m/rad at x_j = (j + 1/2) l (ENG-022).
    first = _named(root, "body", "beam.segment0")
    world = root.find("worldbody")
    assert world is not None
    assert first in list(world)
    assert _floats(first.get("pos")) == [0.125, 2.0, 0.0]
    spans = [0.25, 0.25, 0.25, 0.125]
    for j, span in enumerate(spans):
        segment = _named(root, "body", f"beam.segment{j}")
        joint = _named(segment, "joint", f"beam.node{j}")
        assert joint.get("type") == "hinge"
        assert float(joint.get("stiffness", "")) == pytest.approx(8000.0, rel=1e-15)
        assert float(joint.get("damping", "")) == 0.01
        inertial = segment.find("inertial")
        assert inertial is not None
        mass = 7850.0 * 1e-4 * span
        assert float(inertial.get("mass", "")) == pytest.approx(mass, rel=1e-15)
        assert _floats(inertial.get("diaginertia"))[1] == pytest.approx(mass * span * span / 12.0, rel=1e-15)
        if j > 0:
            assert segment in list(_named(root, "body", f"beam.segment{j - 1}"))
            assert _floats(segment.get("pos")) == [0.25, 0.0, 0.0]
    assert root.find(".//body[@name='beam.pin']") is None
    assert root.find(".//body[@name='beam.roller']") is None


@pytest.mark.req("PHY-018")
def test_pinned_roller_member_has_free_pin_roller_slide_and_end_connect() -> None:
    root = _xml(_full())
    carriage = _named(root, "body", "carriage")
    pin = _named(carriage, "body", "span.pin")
    pin_joint = _named(pin, "joint", "span.pin")
    assert pin_joint.get("stiffness") is None
    assert len(root.findall(".//body[@name='span.pin']//joint")) == 1 + 8
    roller = _named(carriage, "body", "span.roller")
    # Root frame turned +90 deg about z, so the far end x = L lies along parent +y.
    position = _floats(roller.get("pos"))
    assert position == pytest.approx([0.0, 1.0, 1.0], abs=1e-15)
    slide = _named(roller, "joint", "span.roller")
    assert slide.get("type") == "slide"
    assert _floats(slide.get("axis")) == [1.0, 0.0, 0.0]
    inertial = roller.find("inertial")
    assert inertial is not None
    assert float(inertial.get("mass", "")) == pytest.approx(1e-3 * 7850.0 * 1e-4 * 1.0, rel=1e-15)
    end = _named(root, "connect", "span.roller")
    assert (end.get("body1"), end.get("body2")) == ("span.segment7", "span.roller")
    assert _floats(end.get("anchor")) == [0.0625, 0.0, 0.0]
    frame = root.find("./custom/numeric[@name='mechsim:member:span']")
    assert frame is not None
    assert _floats(frame.get("data")) == [0.125, 0.0, 0.0, 1.0, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476]


def _drive(transmission: dict[str, Any]) -> dict[str, Any]:
    data = _full()
    data["transmissions"] = [
        {"name": "drive", "shaft_joint": "spindle", "driven_joint": "slide", "efficiency": 0.9, **transmission}
    ]
    return data


@pytest.mark.req("PHY-018")
@pytest.mark.parametrize(
    ("transmission", "coefficient"),
    [
        ({"type": "belt_pulley", "pulley_radius": "0.02 m", "stiffness": "1e5 N/m"}, 0.02),
        ({"type": "lead_screw", "lead": "0.008 m", "stiffness": "1e5 N/m"}, 0.008 / (2 * math.pi)),
    ],
)
def test_linear_transmission_reflects_the_shaft_onto_the_driven_joint(
    transmission: dict[str, Any], coefficient: float
) -> None:
    root = _xml(_drive(transmission))
    assert root.find(".//body[@name='rotor']") is None
    assert root.find(".//joint[@name='spindle']") is None
    slide = _named(root, "joint", "slide")
    # Rotor: izz = 1e-4 kg m^2 about z through its centre of mass at the joint origin, so J = 1e-4; damping 0.004.
    assert float(slide.get("armature", "")) == pytest.approx(1e-4 / coefficient**2, rel=1e-15)
    assert float(slide.get("damping", "")) == pytest.approx(0.004 / coefficient**2, rel=1e-15)
    assert f"transmission.{transmission['type']}" in _custom(root, "mechsim:elements")
    assert _custom(root, "mechsim:transmissions") == ["drive:rotor:motor:slide"]
    frame = root.find("./custom/numeric[@name='mechsim:transmission:drive']")
    assert frame is not None
    values = _floats(frame.get("data"))
    assert values[:3] == pytest.approx([coefficient, 0.9, 1e-4], rel=1e-15)
    assert values[3:] == [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


@pytest.mark.req("PHY-018")
def test_reflection_report_names_every_approximation() -> None:
    data = _drive({"type": "belt_pulley", "pulley_radius": "0.02 m", "stiffness": "1e5 N/m"})
    data["bodies"][-1]["shapes"] = [{"shape": "sphere", "radius": "0.01 m", "material": "steel"}]
    report = mj.translate_report(_model(data)).report
    reflected = [line for line in report if line.startswith("transmission 'drive'")]
    assert len(reflected) == 5
    assert "armature J / c^2 = 0.25" in reflected[0]
    assert "mass 2.0 kg" in reflected[0]
    assert "1 shape(s) of shaft body 'rotor' are not emitted" in reflected[1]
    assert reflected[2] == "transmission 'drive': stiffness 100000.0 treated as rigid (milestone 1)"
    assert "reaction torque on its mount body" in reflected[3]
    assert "spin-mount" in reflected[3]
    assert "lossless" in reflected[4]
    assert "eta tau / c" in reflected[4]


@pytest.mark.req("PHY-034")
def test_shaft_joint_is_drivable_through_an_inactive_equality_on_the_driven_joint() -> None:
    data = _drive({"type": "lead_screw", "lead": "0.008 m", "stiffness": "1e5 N/m"})
    translation = mj.translate_report(_model(data))
    root = ET.fromstring(translation.mjcf)
    drives = root.findall("equality/joint")
    assert len(drives) == 1
    drive = drives[0]
    assert drive.get("name") == "spindle.drive"
    assert drive.get("joint1") == "slide"
    assert drive.get("joint2") is None
    assert drive.get("active") == "false"
    assert _floats(drive.get("polycoef")) == [0.0, 0.0, 0.0, 0.0, 0.0]
    assert drive.get("solref") == mj.DRIVE_SOLREF
    assert drive.get("solimp") == mj.DRIVE_SOLIMP
    assert _custom(root, "mechsim:drives") == ["spindle"]
    lines = [line for line in translation.report if line.startswith("joint 'spindle': drive")]
    assert len(lines) == 1
    assert "'spindle.drive' on joint 'slide'" in lines[0]
    assert "inactive at load (PHY-034)" in lines[0]


@pytest.mark.req("PHY-034")
def test_listed_drive_joint_gets_its_own_equality_and_none_is_emitted_without_one() -> None:
    root = _xml(_slider())
    drive = _named(root, "joint", "slide.drive")
    assert drive.get("joint1") == "slide"
    assert _custom(root, "mechsim:drives") == ["slide"]
    plain = _slider()
    plain["signals"] = {"pose_bodies": ["block"]}
    bare = _xml(plain)
    assert bare.find("equality") is None
    assert _custom(bare, "mechsim:drives") == []


@pytest.mark.req("PHY-034")
@pytest.mark.parametrize(
    ("joint", "reason"),
    [
        ("spindle", "listed as a drive joint; shafts always are"),
        ("slide", "a drive on a transmission's driven joint"),
        ("weld", "a drive on a fixed joint"),
    ],
)
def test_drive_joints_the_bridge_cannot_drive_are_unmappable(joint: str, reason: str) -> None:
    data = _drive({"type": "lead_screw", "lead": "0.008 m", "stiffness": "1e5 N/m"})
    data["signals"] = {"drive_joints": [joint]}
    with pytest.raises(mj.UnmappableElementError, match=reason):
        mj.translate(_model(data))


def test_neutral_model_refuses_unknown_or_repeated_drive_joints() -> None:
    data = _slider()
    data["signals"] = {"drive_joints": ["slide", "slide", "nowhere"]}
    with pytest.raises(ValueError, match=r"drive joint 'slide' listed twice.*unknown drive joint 'nowhere'"):
        _model(data)


def test_axis_inertia_adds_the_offset_mass() -> None:
    body = _model(_full()).bodies[-1]
    shifted = body.model_copy(update={"com": (0.1, 0.0, 0.3)})
    # About z: I_zz + m (x^2 + y^2) = 1e-4 + 2 * 0.01.
    assert mj._axis_inertia(shifted, (0.0, 0.0, 2.0)) == pytest.approx(1e-4 + 0.02, rel=1e-15)


@pytest.mark.req("PHY-018")
def test_gear_pair_reflects_with_c_one_over_ratio() -> None:
    data = _full()
    data["bodies"].append(_body("wheel", _joint("axle", "revolute")))
    data["transmissions"] = [
        {
            "type": "gear_pair",
            "name": "gears",
            "shaft_joint": "spindle",
            "driven_joint": "axle",
            "efficiency": 1.0,
            "ratio": -4.0,
            "stiffness": "100 N*m/rad",
        }
    ]
    axle = _named(_xml(data), "joint", "axle")
    assert float(axle.get("armature", "")) == pytest.approx(1e-4 * 16.0, rel=1e-15)


@pytest.mark.req("PHY-018")
@pytest.mark.req("PHY-033")
def test_negative_gear_ratio_flips_the_coefficient_sign_only() -> None:
    # c = 1 / ratio carries the direction: q_driven = c q_shaft, F = eta tau / c, tau_load = tau - J qacc_driven / c.
    # Reversing the ratio flips c and so, together, the signs of the driven side's F, q and qd; the shaft's load
    # torque (qacc_driven / c) keeps its sign; the armature J / c^2 is unchanged.
    frames = {}
    for ratio in (4.0, -4.0):
        root = _xml(_gear(ratio))
        frame = root.find("./custom/numeric[@name='mechsim:transmission:gears']")
        assert frame is not None
        frames[ratio] = [float(v) for v in frame.get("data", "").split()]
        axle = _named(root, "joint", "axle")
        assert float(axle.get("armature", "")) == pytest.approx(5.4e-6 * 16.0, rel=1e-15)
    assert frames[4.0][0] == 0.25
    assert frames[-4.0][0] == -0.25
    assert frames[4.0][1:] == frames[-4.0][1:]


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    [("backlash", "0.001 m", "backlash"), ("slip", 0.1, "slip")],
)
def test_transmission_backlash_or_slip_is_unmappable(field: str, value: Any, reason: str) -> None:
    data = _full()
    data["transmissions"][0][field] = value
    with pytest.raises(mj.UnmappableElementError, match=rf"transmission 'belt'.*{reason}"):
        mj.translate(_model(data))


def test_self_locking_lead_screw_is_unmappable() -> None:
    data = _drive({"type": "lead_screw", "lead": "0.008 m", "stiffness": "1e5 N/m", "self_locking": True})
    with pytest.raises(mj.UnmappableElementError, match=r"transmission 'drive'.*self-locking"):
        mj.translate(_model(data))


def test_shaft_that_carries_a_body_is_unmappable() -> None:
    data = _full()
    data["bodies"].append(_body("fan", _joint("fan_hub", "fixed"), parent="rotor"))
    with pytest.raises(mj.UnmappableElementError, match=r"shaft body 'rotor' carries fan"):
        mj.translate(_model(data))


def test_shaft_with_a_spring_is_unmappable() -> None:
    data = _full()
    data["bodies"][-1]["joint"]["angular"] = [{"stiffness": "1 N*m/rad"}]
    with pytest.raises(mj.UnmappableElementError, match=r"joint 'spindle'.*spring or limit"):
        mj.translate(_model(data))


def test_driven_shaft_chain_is_unmappable() -> None:
    data = _full()
    data["bodies"].append(_body("wheel", _joint("axle", "revolute")))
    data["transmissions"].append(
        {
            "type": "gear_pair",
            "name": "gears",
            "shaft_joint": "axle",
            "driven_joint": "spindle",
            "efficiency": 1.0,
            "ratio": 2.0,
            "stiffness": "100 N*m/rad",
        }
    )
    with pytest.raises(mj.UnmappableElementError, match=r"shaft joint 'spindle' is itself driven"):
        mj.translate(_model(data))


def test_unknown_transmission_is_unmappable() -> None:
    with pytest.raises(mj.UnmappableElementError, match=r"transmission 'odd'"):
        mj._coefficient(_Named("odd"))


def test_contact_between_two_world_shapes_is_unmappable() -> None:
    with pytest.raises(mj.UnmappableElementError, match=r"two world shapes"):
        mj._reduced_mass(None, None)


def test_non_finite_value_is_unmappable() -> None:
    with pytest.raises(mj.UnmappableElementError, match=r"not finite"):
        mj._num(math.inf)


def test_structural_members_and_signals_add_no_mjcf() -> None:
    data = _example()
    plain = mj.translate(_model(data))
    data["structural_members"] = [
        {
            "name": "post",
            "section": _section(),
            "material": "steel",
            "supports": [{"at": "0 m", "condition": "clamped"}],
            "fasteners": [{"kind": "bolt"}],
        }
    ]
    assert mj.translate(_model(data)) == plain
