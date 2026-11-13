"""Builder physics output (BLD): neutral model, mass properties, lumped members, transmissions, loud failures."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import pytest

from mechsim.builder import BuildError, BuildResult, build
from mechsim.builder.mass import apply, rotation
from mechsim.builder.physics import element_kinds
from mechsim.design.cad import ConvexHull as HullResult
from mechsim.design.cad import MassPropertiesSI, StepImporter, StepImportResult
from mechsim.design.schema import Design, load_design, shipped_materials
from mechsim.library.registry import default_registry
from mechsim.physics.neutral import (
    BeltPulley,
    Body,
    Box,
    Capsule,
    ConvexHull,
    Cylinder,
    LeadScrew,
    PhysicsModel,
    Plane,
    Sphere,
    from_yaml,
)
from tests.unit.test_builder_support import EXAMPLE, GANTRY, MATERIAL, filled_gantry, gantry, gantry_data, pendulum


def _build(design: Design, **options: Any) -> BuildResult:
    return build(design, default_registry(), shipped_materials(), **options)


def _physics(design: Design, **options: Any) -> PhysicsModel:
    model = _build(design, **options).physics
    assert model is not None
    return model


def _failures(design: Design, **options: Any) -> list[str]:
    with pytest.raises(BuildError) as raised:
        _build(design, **options)
    return raised.value.failures


def _body(model: PhysicsModel, name: str) -> Body:
    return next(body for body in model.bodies if body.name == name)


def _close(actual: Any, expected: Any, path: str = "") -> None:
    if isinstance(expected, dict):
        assert set(actual) == set(expected), path
        for key in expected:
            _close(actual[key], expected[key], f"{path}.{key}")
    elif isinstance(expected, (list, tuple)):
        assert len(actual) == len(expected), path
        for index, (left, right) in enumerate(zip(actual, expected, strict=True)):
            _close(left, right, f"{path}[{index}]")
    elif isinstance(expected, float):
        assert actual == pytest.approx(expected, rel=1e-12, abs=1e-12), path
    else:
        assert actual == expected, path


# ===== Pendulum and the neutral model ==================================


@pytest.mark.req("BLD-001")
@pytest.mark.req("BLD-029")
def test_pendulum_builds_to_the_worked_example() -> None:
    """Field for field against schemas/examples/physics_model.pendulum.yaml; only the blob stamp may differ."""
    model = _physics(pendulum())
    example = from_yaml(EXAMPLE.read_text(encoding="utf-8"))
    built = model.model_dump(mode="python")
    expected = example.model_dump(mode="python")
    built["design"].pop("blob")
    expected["design"].pop("blob")
    _close(built, expected)
    assert len(model.bodies) == 1
    assert model.bodies[0].joint is not None
    assert model.bodies[0].joint.type == "revolute"
    assert model.lumped_members == ()


@pytest.mark.req("BLD-001")
def test_filled_gantry_builds_every_element_kind() -> None:
    model = _physics(filled_gantry())
    assert model.bodies[0].name == "frame"
    assert len(model.bodies) == 9
    assert model.contacts is not None
    assert len(model.lumped_members) == 2
    assert len(model.transmissions) == 3
    assert model.constraints == ()


def test_blob_stamp_defaults_to_zeros_and_takes_the_given_hash() -> None:
    assert _physics(pendulum()).design.blob == "0" * 40
    assert _physics(pendulum(), blob="ab" * 20).design.blob == "ab" * 20


@pytest.mark.req("BLD-002")
def test_every_body_carries_mass_com_and_inertia() -> None:
    for body in _physics(filled_gantry()).bodies:
        assert body.mass > 0.0
        assert len(body.com) == 3
        assert body.inertia.ixx > 0.0
        assert body.material == MATERIAL


@pytest.mark.req("BLD-003")
def test_joints_map_one_to_one_with_their_types() -> None:
    design = filled_gantry()
    model = _physics(design)
    built = {body.joint.name: body.joint.type for body in model.bodies if body.joint is not None}
    assert built == {joint.name: joint.type for joint in design.joints}
    x_axis = _body(model, "rail_y").joint
    assert x_axis is not None
    # rail_y's frame has x along world y, y along world z (M3-X1d, ruling 65), so the world-x travel is its z.
    assert x_axis.axis == (0.0, 0.0, 1.0)
    assert x_axis.linear[0].lower == 0.0
    assert x_axis.linear[0].upper == 1.0
    assert x_axis.linear[0].friction == 1.0
    assert x_axis.linear[0].damping == 10.0
    hook = _body(model, "hook").joint
    assert hook is not None
    assert len(hook.angular) == 3
    assert hook.axis == (0.0, 0.0, 1.0)
    frame = _body(model, "frame").joint
    assert frame is not None
    assert frame.linear == ()
    assert frame.angular == ()


def test_bodies_are_ordered_parent_before_child() -> None:
    model = _physics(filled_gantry())
    seen = {"world"}
    for body in model.bodies:
        assert body.parent in seen
        seen.add(body.name)


def test_run_step_is_not_taken_from_the_tick_rate() -> None:
    """The physics step is run configuration (C4): the model keeps the 1 ms default whatever the control tick."""
    data = gantry_data()
    data["firmware"]["tick_rate"] = "500 Hz"
    assert _physics(filled_gantry(data)).run.step == 1e-3
    assert _physics(pendulum()).run.step == 1e-3


def test_signal_slots_name_actuator_shafts_and_every_body() -> None:
    model = _physics(filled_gantry())
    assert model.signals.load_torque_joints == ("x_motor_spin", "y_motor_spin", "lift_motor_spin")
    assert model.signals.pose_bodies == tuple(body.name for body in model.bodies)


@pytest.mark.req("BLD-030")
def test_model_holds_plain_si_values_only() -> None:
    def walk(value: Any) -> None:
        if isinstance(value, dict):
            for item in value.values():
                walk(item)
        elif isinstance(value, (list, tuple)):
            for item in value:
                walk(item)
        else:
            assert value is None or type(value) in (float, int, str, bool), repr(value)

    walk(_physics(filled_gantry()).model_dump(mode="python"))


# ===== Mass properties ==================================================


@pytest.mark.req("BLD-011")
def test_declared_mass_properties_win_over_geometry() -> None:
    """The pendulum declares 1 kg; its cylinder at the aluminium density would weigh rho pi r^2 L ~ 0.42 kg."""
    body = _physics(pendulum()).bodies[0]
    assert body.mass == 1.0
    assert body.inertia.iyy == 0.020833333333333332


@pytest.mark.req("BLD-011")
@pytest.mark.req("BLD-010")
def test_undeclared_mass_properties_come_from_geometry() -> None:
    """0.1 m aluminium cube: m = rho a^3, I = m a^2 / 6 about each axis."""
    body = _physics(filled_gantry()).bodies[0]
    density = shipped_materials()[MATERIAL].density.typ
    mass = density * 0.1**3
    assert body.mass == pytest.approx(mass, rel=1e-12)
    assert body.inertia.ixx == pytest.approx(mass * 0.01 / 6.0, rel=1e-12)
    assert body.com == (0.0, 0.0, 0.0)


# ===== Shapes and contacts =============================================


@pytest.mark.req("BLD-012")
@pytest.mark.req("BLD-013")
def test_shapes_are_primitives_and_the_floor_is_a_world_plane() -> None:
    model = _physics(filled_gantry())
    for body in model.bodies:
        for shape in body.shapes:
            assert isinstance(shape, (Sphere, Capsule, Box, Cylinder, ConvexHull))
    assert len(model.world_shapes) == 1
    floor = model.world_shapes[0]
    assert isinstance(floor, Plane)
    assert floor.position == (0.0, 0.0, 0.0)
    assert floor.orientation == (1.0, 0.0, 0.0, 0.0)
    assert model.contacts is not None
    assert model.contacts.default.stiffness == 100000.0
    assert model.contacts.default.damping == 100.0
    assert model.contacts.default.friction == 0.5


@pytest.mark.req("BLD-013")
@pytest.mark.parametrize(
    "normal",
    [
        [0.0, 1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.6, 0.8],
        [0.6, 0.0, -0.8],
    ],
)
def test_floor_plane_turns_z_onto_the_normal_at_the_offset(normal: list[float]) -> None:
    data = gantry_data()
    data["environment"]["contact"]["floor"] = {"normal": normal, "offset": "-2 m"}
    floor = _physics(filled_gantry(data)).world_shapes[0]
    for axis in range(3):
        assert floor.position[axis] == pytest.approx(-2.0 * normal[axis], abs=1e-15)
    turned = apply(rotation(floor.orientation), (0.0, 0.0, 1.0))
    for axis in range(3):
        assert turned[axis] == pytest.approx(normal[axis], abs=1e-15)
    assert math.fsum(component * component for component in floor.orientation) == pytest.approx(1.0, abs=1e-15)


@pytest.mark.req("BLD-013")
def test_contact_pairs_and_exclusions_are_copied() -> None:
    data = gantry_data()
    data["environment"]["contact"]["pairs"] = [
        {"materials": [MATERIAL, "pla_printed"], "stiffness": "1 N/m", "damping": "2 N*s/m", "friction": "0.1"}
    ]
    data["environment"]["contact"]["exclusions"] = [["hook", "frame"]]
    contacts = _physics(filled_gantry(data)).contacts
    assert contacts is not None
    assert contacts.pairs[0].materials == (MATERIAL, "pla_printed")
    assert contacts.pairs[0].stiffness == 1.0
    assert contacts.exclusions == (("hook", "frame"),)


def test_no_contact_section_gives_no_contacts_or_floor() -> None:
    model = _physics(pendulum())
    assert model.contacts is None
    assert model.world_shapes == ()


def test_contact_without_floor_gives_no_world_shape() -> None:
    data = gantry_data()
    del data["environment"]["contact"]["floor"]
    model = _physics(filled_gantry(data))
    assert model.world_shapes == ()
    assert model.contacts is not None


# ===== Lumped and structural members ===================================


@pytest.mark.req("BLD-014")
@pytest.mark.req("BLD-018")
def test_flexible_bodies_become_lumped_members() -> None:
    result = _build(filled_gantry())
    model = result.physics
    assert model is not None
    scales = {record.member: record for record in result.lumped}
    members = {member.name: member for member in model.lumped_members}
    rail_x = members["rail_x_beam"]
    assert (rail_x.root, rail_x.end, rail_x.segments) == ("clamped", "free", 16)
    rail_y = members["rail_y_beam"]
    assert (rail_y.root, rail_y.end, rail_y.segments) == ("pinned", "roller", 8)
    assert rail_x.parent == "rail_x"
    assert rail_x.structural_member == "rail_x"
    material = shipped_materials()[MATERIAL]
    record = scales["rail_x_beam"]
    assert record.calibrated
    assert record.stiffness_scale is not None
    assert record.mass_scale is not None
    # BLD-019: the lumped section carries the calibrated E and rho; the member list keeps the material's (BLD-009).
    assert rail_x.section.elastic_modulus == pytest.approx(
        material.youngs_modulus.typ * record.stiffness_scale, rel=1e-15
    )
    assert rail_x.section.density == pytest.approx(material.density.typ * record.mass_scale, rel=1e-15)
    member = next(item for item in model.structural_members if item.name == "rail_x")
    assert member.section.elastic_modulus == material.youngs_modulus.typ
    assert member.section.density == material.density.typ
    assert rail_x.section.area == 1e-4
    assert rail_x.section.second_moment_of_area == 1e-8
    assert rail_x.section.length == 1.0


@pytest.mark.req("BLD-018")
def test_declared_segment_count_overrides_the_default() -> None:
    data = gantry_data()
    data["bodies"][2]["flexible"]["segments"] = 12
    data["bodies"][1]["flexible"]["segments"] = 10
    members = {member.name: member for member in _physics(filled_gantry(data)).lumped_members}
    assert members["rail_y_beam"].segments == 12
    assert members["rail_x_beam"].segments == 10


@pytest.mark.req("BLD-014")
def test_lumped_record_follows_the_engine_segment_rule() -> None:
    """core/engine lumped beam: l = L / N, k = E I / l per node; spans node to node, so a clamped root leaves l/2 on
    the parent (spans l x (N - 1) + l/2) and a pinned root adds a first span l/2; each span has m = rho A s."""
    result = _build(filled_gantry())
    material = shipped_materials()[MATERIAL]
    e, rho = material.youngs_modulus.typ, material.density.typ
    records = {record.member: record for record in result.lumped}
    rail_x = records["rail_x_beam"]
    assert rail_x.stiffness_scale is not None
    assert rail_x.mass_scale is not None
    e, rho = e * rail_x.stiffness_scale, rho * rail_x.mass_scale
    assert rail_x.segment_length == 0.0625
    assert rail_x.node_stiffness == pytest.approx(e * 1e-8 / 0.0625, rel=1e-15)
    assert rail_x.spans == (0.0625,) * 15 + (0.03125,)
    for span, mass in zip(rail_x.spans, rail_x.masses, strict=True):
        assert mass == pytest.approx(rho * 1e-4 * span, rel=1e-15)
    assert math.fsum(rail_x.masses) == pytest.approx(rho * 1e-4 * (1.0 - 0.03125), rel=1e-14)
    rail_y = records["rail_y_beam"]
    assert rail_y.stiffness_scale is not None
    assert rail_y.mass_scale is not None
    e = material.youngs_modulus.typ * rail_y.stiffness_scale
    rho = material.density.typ * rail_y.mass_scale
    assert rail_y.segments == 8
    assert rail_y.node_stiffness == pytest.approx(e * 1e-8 / 0.1, rel=1e-14)
    assert len(rail_y.spans) == 9
    assert math.fsum(rail_y.spans) == pytest.approx(0.8, rel=1e-14)
    assert math.fsum(rail_y.masses) == pytest.approx(rho * 1e-4 * 0.8, rel=1e-14)
    assert (rail_y.calibration_source, rail_y.calibration_load) == ("hand", 10.0)
    assert rail_y.calibration_deflection == 0.001
    assert rail_y.calibration_frequency == 50.0


@pytest.mark.req("BLD-014")
def test_single_segment_beam() -> None:
    data = gantry_data()
    data["bodies"][1]["flexible"]["segments"] = 1
    del data["bodies"][1]["flexible"]["calibration"]
    record = next(item for item in _build(filled_gantry(data)).lumped if item.member == "rail_x_beam")
    assert record.spans == (0.5,)


@pytest.mark.req("BLD-009")
def test_structural_members_record_section_material_and_supports() -> None:
    data = gantry_data()
    data["bodies"][1]["fasteners"] = [{"kind": "m5_bolt", "count": 4, "grade": "steel_8_8_fastener", "preload": "1 kN"}]
    result = _build(filled_gantry(data))
    assert result.physics is not None
    assert result.members == result.physics.structural_members
    members = {member.name: member for member in result.members}
    assert set(members) == {"rail_x", "rail_y"}
    assert [(s.at, s.condition) for s in members["rail_x"].supports] == [(0.0, "clamped")]
    assert [(s.at, s.condition) for s in members["rail_y"].supports] == [(0.0, "pinned"), (0.8, "roller")]
    assert members["rail_x"].material == MATERIAL
    assert members["rail_x"].fasteners[0]["count"] == 4
    assert members["rail_y"].fasteners == ()


@pytest.mark.parametrize(
    "supports", [["pinned", "clamped"], ["roller", "free"], ["free", "free"], ["pinned", "pinned"]]
)
def test_support_pair_without_a_lumped_form_fails(supports: list[str]) -> None:
    data = gantry_data()
    data["bodies"][1]["flexible"]["supports"] = supports
    failures = _failures(filled_gantry(data))
    assert any("flexible body 'rail_x'" in failure for failure in failures)


# ===== Transmissions ====================================================


@pytest.mark.req("BLD-005")
def test_belts_become_neutral_transmissions() -> None:
    """Catalog ratio is travel per shaft turn in SI m/rad: belt pulley radius = 40 mm / 2 pi on every axis, the lift
    included (gantry team 2026-11-11: no lead screw)."""
    model = _physics(filled_gantry())
    transmissions = {item.name: item for item in model.transmissions}
    belt = transmissions["x_belt"]
    assert isinstance(belt, BeltPulley)
    assert (belt.shaft_joint, belt.driven_joint) == ("x_motor_spin", "x_axis")
    assert belt.pulley_radius == pytest.approx(0.04 / (2.0 * math.pi), rel=1e-15)
    assert belt.efficiency == 0.95
    assert belt.stiffness == 100000.0
    assert belt.backlash == 0.0
    assert transmissions["y_belt"].driven_joint == "y_axis"
    lift = transmissions["lift_belt"]
    assert isinstance(lift, BeltPulley)
    assert (lift.shaft_joint, lift.driven_joint) == ("lift_motor_spin", "lift_axis")
    assert lift.pulley_radius == pytest.approx(0.04 / (2.0 * math.pi), rel=1e-15)
    assert lift.backlash == 0.0
    assert not any(isinstance(item, LeadScrew) for item in model.transmissions)


@pytest.mark.req("BLD-027")
def test_instance_overrides_reach_the_transmission() -> None:
    data = gantry_data()
    data["components"][2]["overrides"] = {"ratio": "60 mm/turn", "efficiency": "0.9"}
    data["components"][8]["overrides"] = {"ratio": "20 mm/turn"}
    transmissions = {item.name: item for item in _physics(filled_gantry(data)).transmissions}
    belt = transmissions["x_belt"]
    assert isinstance(belt, BeltPulley)
    assert belt.pulley_radius == pytest.approx(0.06 / (2.0 * math.pi), rel=1e-15)
    assert belt.efficiency == 0.9
    lift = transmissions["lift_belt"]
    assert isinstance(lift, BeltPulley)
    assert lift.pulley_radius == pytest.approx(0.02 / (2.0 * math.pi), rel=1e-15)


@pytest.mark.req("BLD-005")
def test_transmission_whose_clamp_body_has_no_joint_fails() -> None:
    design = filled_gantry()
    design.components[2].mechanical[1].body = "nowhere"
    failures = _failures(design)
    assert any("'x_belt'" in failure for failure in failures)


# ===== STEP bodies ======================================================


class _Importer(StepImporter):
    def __init__(self, mass: float, mesh: bool = False) -> None:
        self.mass = mass
        self.mesh = mesh

    def import_body(self, path: Path, body: str, density: float) -> StepImportResult:
        vertices = [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.1)]
        return StepImportResult(
            body=body,
            volume=0.001,
            mass_properties=MassPropertiesSI(
                mass=self.mass, center_of_mass=(0.0, 0.0, 0.0), inertia=(0.01, 0.01, 0.01, 0.0, 0.0, 0.0)
            ),
            hulls=[HullResult(vertices=vertices)],
            needs_mesh_contact=self.mesh,
        )

    def export_collision_mesh(self, result: StepImportResult, directory: Path) -> list[Path]:
        return []


def _step_gantry() -> Design:
    data = gantry_data()
    data["bodies"][5]["geometry"] = {"kind": "step", "file": "hook.step", "mass_check_tolerance": "1 percent"}
    data["bodies"][5]["mass_properties"] = {
        "mass": "2 kg",
        "com": ["0 m", "0 m", "0 m"],
        "inertia": {"ixx": "0.01 kg*m**2", "iyy": "0.01 kg*m**2", "izz": "0.01 kg*m**2"},
    }
    return filled_gantry(data)


@pytest.mark.req("BLD-011")
def test_step_body_without_importer_uses_declared_values_with_a_note() -> None:
    result = _build(_step_gantry())
    assert result.physics is not None
    hook = _body(result.physics, "hook")
    assert hook.mass == 2.0
    assert hook.shapes == ()
    assert any("'hook'" in note for note in result.notes)


@pytest.mark.req("BLD-011")
def test_step_body_cross_checks_and_takes_the_hulls() -> None:
    result = _build(_step_gantry(), step_importer=_Importer(2.0), base=Path("."))
    assert result.physics is not None
    hook = _body(result.physics, "hook")
    assert hook.mass == 2.0
    assert len(hook.shapes) == 1
    assert isinstance(hook.shapes[0], ConvexHull)
    rope_note = (
        "rope 'payload_rope': its to end is case payload 'payload'; not emitted until the case places the payload "
        "(BLD-032)"
    )
    plane = "gravity entirely outside its bending plane"
    others = [note for note in result.notes if "is an approximation" not in note and plane not in note]
    assert others == [rope_note]
    approximations = [note for note in result.notes if "is an approximation" in note]
    assert [note.split("'")[1] for note in approximations] == ["rail_x", "rail_y"]
    # The fixture's placements carry no rotation, so both rails' frames are the world's, z along gravity (BLD-038).
    assert [note.split("'")[1] for note in result.notes if plane in note] == ["rail_x", "rail_y"]


def test_step_body_mass_mismatch_fails() -> None:
    failures = _failures(_step_gantry(), step_importer=_Importer(2.5), base=Path("."))
    assert any("body 'hook'" in failure and "mass" in failure for failure in failures)


@pytest.mark.req("BLD-012")
def test_step_body_needing_mesh_contact_fails() -> None:
    failures = _failures(_step_gantry(), step_importer=_Importer(2.0, mesh=True), base=Path("."))
    assert any("mesh contact" in failure for failure in failures)


# ===== Shipped gantry, rigid members, module order ======================


def test_open_transmission_override_keeps_physics_pending() -> None:
    data = gantry_data()
    data["components"][2]["overrides"] = {"ratio": "${workspace_x}"}
    result = _build(filled_gantry(data))
    assert result.physics is None
    assert "components[2].overrides.ratio" in result.pending


@pytest.mark.req("BLD-033")
def test_shipped_gantry_builds_rigid_on_its_placeholder_geometry() -> None:
    result = _build(gantry(), rigid_members=True)
    assert result.physics is not None
    assert len(result.physics.bodies) == 9
    assert result.physics.lumped_members == ()
    assert [member.name for member in result.members] == ["rail_x", "rail_y"]
    assert "power.rails[1].supply" in result.pending
    assert not any(path.startswith("bodies") for path in result.pending)
    assert list(result.pending) == sorted(set(result.pending))


@pytest.mark.req("BLD-018")
def test_shipped_gantry_flexible_build_maps_clamped_clamped_rail_x(tmp_path: Path) -> None:
    """Ruling 16: the neutral lumped beam has a clamped far end; rail_x gets sixteen segments (ruling 31). The
    shipped calibration is the open `${rail_calibration}`; it is bound here to the design's own default."""
    text = GANTRY.read_text(encoding="utf-8").replace(
        "${rail_calibration}", "{source: hand, load: 100 N, deflection: 1 mm, first_natural_frequency: 50 Hz}"
    )
    bound = tmp_path / "design.yaml"
    bound.write_text(text, encoding="utf-8")
    result = _build(load_design(bound))
    assert result.physics is not None
    members = {member.structural_member: member for member in result.physics.lumped_members}
    assert (members["rail_x"].end, members["rail_x"].segments) == ("clamped", 16)


def test_open_physics_value_keeps_physics_pending() -> None:
    data = gantry_data()
    data["joints"][2]["placement"] = "${cad_model}"
    result = _build(filled_gantry(data))
    assert result.physics is None
    assert result.members == ()
    assert "joints[2].placement" in result.pending


@pytest.mark.req("BLD-033")
def test_rigid_members_option_builds_flexible_bodies_rigid() -> None:
    flexible = _build(filled_gantry())
    rigid = _build(filled_gantry(), rigid_members=True)
    assert flexible.physics is not None
    assert rigid.physics is not None
    assert len(flexible.physics.lumped_members) == 2
    assert len(flexible.lumped) == 2
    assert rigid.physics.lumped_members == ()
    assert rigid.lumped == ()
    assert rigid.members == flexible.members
    assert rigid.physics.structural_members == flexible.physics.structural_members
    assert rigid.physics.bodies == flexible.physics.bodies


@pytest.mark.req("BLD-033")
def test_rigid_build_needs_no_calibration_and_accepts_any_supports() -> None:
    data = gantry_data()
    data["bodies"][1]["flexible"]["calibration"] = "${rail_calibration}"
    data["bodies"][1]["flexible"]["supports"] = ["clamped", "clamped"]
    rigid = _build(filled_gantry(data), rigid_members=True)
    assert rigid.physics is not None
    assert [(s.at, s.condition) for s in rigid.members[0].supports] == [(0.0, "clamped"), (1.0, "clamped")]
    data["bodies"][1]["flexible"]["supports"] = ["pinned", "clamped"]
    assert _build(filled_gantry(data), rigid_members=True).physics is not None
    assert any("supports (pinned, clamped)" in failure for failure in _failures(filled_gantry(data)))


def test_module_instances_follow_the_tick_order() -> None:
    """KER-009: physics, integrity, sensors, controller, signal layer, rail (power, safety), drivers, steppers,
    transmissions, recorder; design order within a group."""
    result = _build(gantry(), rigid_members=True)
    assert result.module_instances == (
        "physics",
        "integrity",
        "x_min_switch",
        "x_max_switch",
        "y_min_switch",
        "y_max_switch",
        "lift_min_switch",
        "lift_max_switch",
        "x_encoder",
        "y_encoder",
        "lift_encoder",
        "load_cell",
        "load_amp",
        "controller",
        "signals",
        "lift_brake",
        "motor_supply",
        "logic_psu",
        "estop",
        "watchdog",
        "x_driver",
        "y_driver",
        "lift_driver",
        "operator_link",
        "x_motor",
        "y_motor",
        "lift_motor",
        "x_belt",
        "y_belt",
        "lift_belt",
        "hook_effector",
        "recorder",
    )
    assert result.design == gantry()
    assert _build(pendulum()).module_instances == ("physics", "integrity", "controller", "signals", "recorder")


# ===== Loud failures ====================================================


@pytest.mark.req("BLD-020")
def test_body_with_no_material_fails_naming_the_body() -> None:
    materials = {name: record for name, record in shipped_materials().items() if name != MATERIAL}
    with pytest.raises(BuildError) as raised:
        build(filled_gantry(), default_registry(), materials)
    for body in ("frame", "rail_x", "hook"):
        assert any(f"body '{body}' has no material" in failure for failure in raised.value.failures)


@pytest.mark.req("BLD-021")
def test_unconstrained_body_fails_naming_the_body() -> None:
    design = filled_gantry()
    design.joints = [joint for joint in design.joints if joint.name != "hook_swing"]
    failures = _failures(design)
    assert any("unconstrained body 'hook'" in failure for failure in failures)


@pytest.mark.req("BLD-021")
def test_body_in_a_joint_cycle_is_unconstrained() -> None:
    design = filled_gantry()
    design.joints[0].parent = "rail_x"
    failures = _failures(design)
    assert any("unconstrained body 'frame'" in failure for failure in failures)
    assert any("unconstrained body 'rail_x'" in failure for failure in failures)


@pytest.mark.req("BLD-022")
def test_overlapping_bodies_fail_naming_both() -> None:
    data = gantry_data()
    data["joints"][6]["placement"] = {"position": ["1 m", "0 m", "0 m"]}
    failures = _failures(filled_gantry(data))
    assert any("'rail_x' and 'x_motor_shaft'" in failure for failure in failures)


@pytest.mark.req("BLD-022")
def test_touching_bodies_pass_and_excluded_or_joined_pairs_are_skipped() -> None:
    data = gantry_data()
    data["joints"][6]["placement"] = {"position": ["1.1 m", "0 m", "0 m"]}
    assert _build(filled_gantry(data)).physics is not None
    data["joints"][6]["placement"] = {"position": ["1 m", "0 m", "0 m"]}
    data["environment"]["contact"]["exclusions"] = [["x_motor_shaft", "rail_x"]]
    assert _build(filled_gantry(data)).physics is not None
    joined = gantry_data()
    joined["joints"][1]["placement"] = {"position": ["0 m", "0 m", "0 m"]}
    joined["joints"][6]["placement"] = {"position": ["-1 m", "0 m", "0 m"]}
    assert _build(filled_gantry(joined)).physics is not None


@pytest.mark.req("BLD-022")
def test_rotated_shapes_overlap_by_their_turned_bounds() -> None:
    """A 0.1 x 0.1 x 1 box at x = 0.5 m reaches x = 1.0 m once turned 90 degrees about y, past rail_x's face at
    0.95 m; unturned it spans 0.45 m to 0.55 m."""
    data = gantry_data()
    data["bodies"][6]["geometry"]["shapes"][0]["size"] = ["0.1 m", "0.1 m", "1 m"]
    data["joints"][6]["placement"] = {"position": ["0.5 m", "0 m", "0 m"]}
    assert _build(filled_gantry(data)).physics is not None
    half = math.sqrt(0.5)
    data["bodies"][6]["geometry"]["shapes"][0]["orientation"] = [half, 0.0, half, 0.0]
    failures = _failures(filled_gantry(data))
    assert any("'rail_x' and 'x_motor_shaft'" in failure for failure in failures)


@pytest.mark.req("BLD-024")
def test_element_the_adapter_cannot_map_fails_naming_element_and_adapter() -> None:
    supported = {kind for kind, _name in element_kinds(_physics(filled_gantry()))}
    assert _build(filled_gantry(), supported_elements=supported, adapter="native").physics is not None
    failures = _failures(
        filled_gantry(),
        supported_elements=supported - {"transmission.belt_pulley", "joint.spherical"},
        adapter="native",
    )
    assert any("'lift_belt'" in failure and "'native'" in failure for failure in failures)
    assert any("'hook_swing'" in failure for failure in failures)


def test_nonzero_joint_backlash_has_no_neutral_form() -> None:
    data = gantry_data()
    data["joints"][2]["linear"][0]["backlash"] = "1 mm"
    failures = _failures(filled_gantry(data))
    assert any("joint 'x_axis'" in failure and "backlash" in failure for failure in failures)


@pytest.mark.req("BLD-028")
def test_unresolved_cross_domain_binding_fails() -> None:
    design = filled_gantry()
    design.components[0].mechanical[1].joint = "no_such_joint"
    design.components[9].mechanical[0].body = "no_such_body"
    failures = _failures(design)
    assert any("'x_motor' mount 'shaft'" in failure and "no_such_joint" in failure for failure in failures)
    assert any("'x_min_switch' mount 'body'" in failure and "no_such_body" in failure for failure in failures)


@pytest.mark.req("BLD-028")
def test_binding_to_an_unknown_mount_or_a_shaft_on_a_body_fails() -> None:
    design = filled_gantry()
    design.components[9].mechanical[0].mount = "no_mount"
    design.components[0].mechanical[1].joint = None
    design.components[0].mechanical[1].body = "frame"
    failures = _failures(design)
    assert any("mount 'no_mount'" in failure for failure in failures)
    assert any("shaft mount must bind a joint" in failure for failure in failures)


@pytest.mark.req("BLD-025")
def test_failed_build_lists_every_failure_and_returns_nothing() -> None:
    design = filled_gantry()
    design.joints = [joint for joint in design.joints if joint.name != "hook_swing"]
    design.pin_bindings = design.pin_bindings[1:]
    design.components[0].mechanical[1].joint = "no_such_joint"
    failures = _failures(design)
    assert any("unconstrained body 'hook'" in failure for failure in failures)
    assert any("'x_driver.step' has no pin binding" in failure for failure in failures)
    assert any("no_such_joint" in failure for failure in failures)
    with pytest.raises(BuildError) as raised:
        _build(design)
    assert str(raised.value).startswith("build failed:")
