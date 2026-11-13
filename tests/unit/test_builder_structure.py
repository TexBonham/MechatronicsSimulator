"""Builder structure (BLD-008, BLD-009, BLD-014, BLD-018, BLD-019, BLD-031, BLD-032, LIB-030): member list with
mounts, catalog sections, lumped chain statics and first mode, calibration, fastenings, ropes."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import pytest

from mechsim.builder import BuildError, BuildResult, build, structure
from mechsim.builder.payload import PAYLOAD_EDGE, with_payload
from mechsim.builder.payload import _pose as payload_pose
from mechsim.builder.payload import _q_rot as q_rot
from mechsim.builder.physics import element_kinds
from mechsim.design.schema import Design, shipped_materials
from mechsim.library.registry import default_registry
from mechsim.physics.neutral import PhysicsModel
from tests.unit.test_builder_support import MATERIAL, filled_gantry, gantry, gantry_data

EI = 2.0e11 * 0.01**4 / 12.0
RHO_A = 7850.0 * 1e-4


def _build(design: Design, **options: Any) -> BuildResult:
    return build(design, default_registry(), shipped_materials(), **options)


def _physics(design: Design) -> PhysicsModel:
    model = _build(design).physics
    assert model is not None
    return model


def _failures(design: Design) -> list[str]:
    with pytest.raises(BuildError) as raised:
        _build(design)
    return raised.value.failures


def _bar(supports: list[str], **flexible: Any) -> Design:
    return Design.model_validate(_bar_data(supports, **flexible))


def _bar_data(supports: list[str], **flexible: Any) -> dict[str, Any]:
    body: dict[str, Any] = {
        "name": "bar",
        "material": MATERIAL,
        "process": {"kind": "extrusion"},
        "geometry": {
            "kind": "primitives",
            "shapes": [{"shape": "box", "position": ["0.5 m", "0 m", "0 m"], "size": ["1 m", "0.01 m", "0.01 m"]}],
        },
        "catalog_part": {"type": "flexible_beam", "part": "steel_bar_10mm_1m"},
        "flexible": {"supports": supports, **flexible},
    }
    return {
        "name": "bar_design",
        "version": "1.0.0",
        "source_hash": "unset",
        "status": "release",
        "environment": {"gravity": ["0 m/s**2", "-9.81 m/s**2", "0 m/s**2"], "ambient_temperature": "293.15 K"},
        "bodies": [body],
        "joints": [
            {
                "name": "bar_mount",
                "type": "fixed",
                "parent": "world",
                "child": "bar",
                "placement": {"position": ["0 m", "0 m", "0 m"]},
            }
        ],
    }


# ===== Chain statics and first mode =====================================


@pytest.mark.req("BLD-014")
@pytest.mark.parametrize("segments", [1, 2, 4, 8, 16])
def test_chain_cantilever_tip_deflection_is_the_engine_chain_value(segments: int) -> None:
    # ENG-038: the midpoint-spring chain's tip deflection is P L^3 / (3 E I) (1 - 1 / (4 N^2)), exact in the chain.
    chain = structure.chain_layout("clamped", "free", segments, 1.0)
    measured = structure.chain_deflection(chain, EI, 1.0, 1.0)
    assert measured == pytest.approx(1.0 / (3.0 * EI) * (1.0 - 1.0 / (4.0 * segments**2)), rel=1e-12)


@pytest.mark.req("BLD-014")
@pytest.mark.parametrize("segments", [2, 4, 8, 16])
def test_chain_simply_supported_midspan_deflection_is_the_engine_chain_value(segments: int) -> None:
    # ENG-040: midspan deflection P L^3 / (48 E I) (1 - 1 / N^2) for even N, exact in the chain.
    chain = structure.chain_layout("pinned", "roller", segments, 1.0)
    measured = structure.chain_deflection(chain, EI, 1.0, 0.5)
    assert measured == pytest.approx(1.0 / (48.0 * EI) * (1.0 - 1.0 / segments**2), rel=1e-12)


@pytest.mark.req("BLD-014")
def test_chain_first_modes_converge_to_the_closed_forms() -> None:
    # Euler-Bernoulli first modes: 1.8751^2 and pi^2 times sqrt(E I / (rho A L^4)); the chain converges as N grows,
    # so the error at 16 segments is below that at 4.
    scale = math.sqrt(EI / RHO_A)
    errors: dict[str, list[float]] = {"cantilever": [], "supported": []}
    for segments in (4, 16):
        cantilever = structure.chain_layout("clamped", "free", segments, 1.0)
        supported = structure.chain_layout("pinned", "roller", segments, 1.0)
        errors["cantilever"].append(
            abs(structure.chain_first_frequency(cantilever, EI, RHO_A) / (1.8751040687119611**2 * scale) - 1.0)
        )
        errors["supported"].append(
            abs(structure.chain_first_frequency(supported, EI, RHO_A) / (math.pi**2 * scale) - 1.0)
        )
    for series in errors.values():
        assert series[1] < series[0]
        assert series[0] < 0.03


def test_chain_layout_spans_cover_the_member() -> None:
    clamped = structure.chain_layout("clamped", "roller", 4, 1.0)
    assert clamped.hinges == (0.125, 0.375, 0.625, 0.875)
    assert clamped.starts[0] == 0.125
    assert sum(clamped.spans) == pytest.approx(0.875, rel=1e-15)
    assert clamped.roller
    pinned = structure.chain_layout("pinned", "free", 4, 1.0)
    assert pinned.hinges[0] == 0.0
    assert pinned.springs == (False, True, True, True, True)
    assert sum(pinned.spans) == pytest.approx(1.0, rel=1e-15)
    assert not pinned.roller


def test_pinned_free_chain_is_a_mechanism() -> None:
    chain = structure.chain_layout("pinned", "free", 4, 1.0)
    with pytest.raises(structure.ChainError, match="mechanism"):
        structure.chain_deflection(chain, EI, 1.0, 1.0)


def test_load_point_is_the_tip_of_a_free_end_else_midspan() -> None:
    assert structure.load_point("free", 2.0) == 2.0
    assert structure.load_point("roller", 2.0) == 1.0


# ===== Calibration =========================================================


@pytest.mark.req("BLD-019")
def test_calibration_scales_are_exact_on_the_chain() -> None:
    # Deflection is linear in 1 / E I and omega is proportional to sqrt(E I / m): after scaling, the chain reproduces
    # both recorded values to rounding.
    chain = structure.chain_layout("clamped", "free", 4, 1.0)
    scales = structure.calibrate(chain, EI, RHO_A, 2.0, 3e-3, 7.0)
    assert structure.chain_deflection(chain, EI * scales.stiffness_scale, 2.0, 1.0) == pytest.approx(3e-3, rel=1e-12)
    omega = structure.chain_first_frequency(chain, EI * scales.stiffness_scale, RHO_A * scales.mass_scale)
    assert omega == pytest.approx(2.0 * math.pi * 7.0, rel=1e-10)


@pytest.mark.req("BLD-019")
def test_recorded_chain_values_give_unit_scales() -> None:
    chain = structure.chain_layout("pinned", "roller", 8, 1.0)
    deflection = structure.chain_deflection(chain, EI, 5.0, 0.5)
    frequency = structure.chain_first_frequency(chain, EI, RHO_A) / (2.0 * math.pi)
    scales = structure.calibrate(chain, EI, RHO_A, 5.0, deflection, frequency)
    assert scales.stiffness_scale == pytest.approx(1.0, rel=1e-12)
    assert scales.mass_scale == pytest.approx(1.0, rel=1e-10)


# Near the 1 m steel bar's own values (tip deflection 19.9 mm under 10 N, first mode 8.15 Hz): scales near 1.
CALIBRATION = {"source": "hand", "load": "10 N", "deflection": "19 mm", "first_natural_frequency": "8 Hz"}


@pytest.mark.req("BLD-019")
def test_build_reports_scales_and_applies_them_to_the_lumped_section_only() -> None:
    result = _build(_bar(["clamped", "free"], calibration=CALIBRATION))
    record = result.lumped[0]
    assert record.calibrated
    assert (record.calibration_source, record.calibration_load) == ("hand", 10.0)
    assert record.calibration_deflection == 0.019
    assert record.calibration_frequency == 8.0
    assert record.stiffness_scale is not None
    assert record.mass_scale is not None
    chain = structure.chain_layout("clamped", "free", 8, 1.0)
    expected = structure.calibrate(chain, EI, RHO_A, 10.0, 0.019, 8.0)
    assert record.stiffness_scale == pytest.approx(expected.stiffness_scale, rel=1e-12)
    assert record.mass_scale == pytest.approx(expected.mass_scale, rel=1e-12)
    assert result.physics is not None
    lumped = result.physics.lumped_members[0].section
    assert lumped.elastic_modulus == pytest.approx(2.0e11 * record.stiffness_scale, rel=1e-15)
    assert lumped.density == pytest.approx(7850.0 * record.mass_scale, rel=1e-15)
    member = result.members[0].section
    assert (member.elastic_modulus, member.density) == (2.0e11, 7850.0)
    assert record.node_stiffness == pytest.approx(2.0e11 * record.stiffness_scale * EI / 2.0e11 / 0.125, rel=1e-12)
    assert result.notes == ()


@pytest.mark.req("BLD-019")
def test_member_without_calibration_is_built_as_is_and_flagged() -> None:
    result = _build(_bar(["clamped", "free"]))
    record = result.lumped[0]
    assert not record.calibrated
    assert record.stiffness_scale is None
    assert record.mass_scale is None
    assert record.calibration_source is None
    assert result.physics is not None
    section = result.physics.lumped_members[0].section
    assert (section.elastic_modulus, section.density) == (2.0e11, 7850.0)
    assert result.notes == (
        "flexible body 'bar': no calibration data; the lumped beam uses its section and material as is (BLD-019)",
    )


@pytest.mark.req("BLD-019")
def test_calibrating_a_mechanism_fails_the_build() -> None:
    failures = _failures(_bar(["pinned", "free"], calibration=CALIBRATION))
    assert failures == [
        "flexible body 'bar': calibration failed: the member's supports leave it a mechanism; it has no static "
        "deflection (BLD-019)"
    ]


@pytest.mark.req("BLD-019")
@pytest.mark.req("BLD-033")
def test_rigid_build_needs_no_calibration_and_emits_no_lumped_record() -> None:
    result = _build(_bar(["clamped", "free"], calibration=CALIBRATION), rigid_members=True)
    assert result.lumped == ()
    assert len(result.members) == 1


@pytest.mark.req("BLD-018")
@pytest.mark.req("BLD-019")
def test_pinned_roller_default_segments_are_used_for_calibration() -> None:
    result = _build(_bar(["pinned", "roller"], calibration=CALIBRATION))
    assert result.physics is not None
    assert result.physics.lumped_members[0].segments == 8
    chain = structure.chain_layout("pinned", "roller", 8, 1.0)
    expected = structure.calibrate(chain, EI, RHO_A, 10.0, 0.019, 8.0)
    assert result.lumped[0].stiffness_scale == pytest.approx(expected.stiffness_scale, rel=1e-12)


# ===== Catalog sections (LIB-030) =======================================


@pytest.mark.req("LIB-030")
@pytest.mark.req("BLD-014")
def test_section_and_material_constants_come_from_the_catalog_part() -> None:
    member = _physics(_bar(["clamped", "free"])).lumped_members[0]
    section = member.section
    assert (section.area, section.length, section.elastic_modulus, section.density) == (1e-4, 1.0, 2.0e11, 7850.0)
    assert section.second_moment_of_area == pytest.approx(0.01**4 / 12.0, rel=1e-15)


@pytest.mark.req("DES-033")
@pytest.mark.req("BLD-014")
def test_design_section_shape_wins_over_the_part_and_takes_the_body_material() -> None:
    design = _bar(["clamped", "free"], section={"shape": {"kind": "circle", "diameter": "20 mm"}, "length": "0.8 m"})
    member = _physics(design).lumped_members[0]
    material = shipped_materials()[MATERIAL]
    assert member.section.area == pytest.approx(math.pi * 0.02**2 / 4.0, rel=1e-15)
    assert member.section.second_moment_of_area == pytest.approx(math.pi * 0.02**4 / 64.0, rel=1e-15)
    assert member.section.length == 0.8
    assert member.section.elastic_modulus == material.youngs_modulus.typ


# ===== Member list and mounts (BLD-008, BLD-009) ========================


@pytest.mark.req("BLD-008")
@pytest.mark.req("BLD-009")
def test_member_list_names_each_member_with_section_material_supports_and_mounts() -> None:
    result = _build(filled_gantry())
    members = {member.name: member for member in result.members}
    assert sorted(members) == ["rail_x", "rail_y"]
    assert members["rail_x"].mounts == ("rail_x_mount",)
    assert members["rail_y"].mounts == ()
    assert members["rail_x"].material == MATERIAL
    assert [support.condition for support in members["rail_y"].supports] == ["pinned", "roller"]
    assert result.physics is not None
    assert result.physics.structural_members == result.members


@pytest.mark.req("BLD-008")
def test_member_list_is_kept_in_a_rigid_build_and_in_the_canonical_json() -> None:
    result = _build(gantry(), rigid_members=True)
    assert [member.name for member in result.members] == ["rail_x", "rail_y"]
    assert '"members":[' in result.to_json()
    assert '"mounts":["rail_x_mount"]' in result.to_json()


# ===== Fastenings (BLD-031) ===============================================


@pytest.mark.req("BLD-031")
def test_every_fixed_joint_and_fastener_set_is_an_addressable_fastening() -> None:
    data = gantry_data()
    data["bodies"][1]["fasteners"] = [
        {"kind": "m5_bolt", "count": 4, "grade": "steel_8_8_fastener", "preload": "1 kN"},
        {"kind": "m3_bolt", "count": 2, "grade": "steel_8_8_fastener", "preload": "0.5 kN"},
        {"kind": "m3_bolt", "count": 2, "grade": "steel_8_8_fastener", "preload": "0.5 kN"},
    ]
    model = _physics(filled_gantry(data))
    fixed = [joint.name for joint in filled_gantry(data).joints if joint.type == "fixed"]
    mounts = [item for item in model.fastenings if item.kind == "mount"]
    assert [item.name for item in mounts] == fixed
    fasteners = [item for item in model.fastenings if item.kind == "fastener"]
    assert [item.name for item in fasteners] == ["rail_x_m5_bolt", "rail_x_m3_bolt_1", "rail_x_m3_bolt_2"]
    for item in fasteners:
        assert (item.body_a, item.body_b, item.joint) == ("frame", "rail_x", "rail_x_mount")


@pytest.mark.req("BLD-031")
def test_named_mount_maps_to_one_fastening_index_an_adapter_can_disable() -> None:
    model = _physics(filled_gantry())
    index = model.fastening_index("rail_x_mount")
    assert model.fastenings[index].name == "rail_x_mount"
    assert (model.fastenings[index].body_a, model.fastenings[index].body_b) == ("frame", "rail_x")
    assert [item.name for item in model.fastenings].count("rail_x_mount") == 1
    with pytest.raises(KeyError, match="no fastening 'rail_x'"):
        model.fastening_index("rail_x")


@pytest.mark.req("BLD-031")
def test_mount_on_the_world_is_a_fastening_and_no_fastener_adds_none() -> None:
    model = _physics(_bar(["clamped", "free"]))
    assert [(item.name, item.kind, item.body_a, item.body_b) for item in model.fastenings] == [
        ("bar_mount", "mount", "world", "bar")
    ]


# ===== Ropes (BLD-032) ===================================================


def _rope_design(to: dict[str, Any], **extra: Any) -> Design:
    data = gantry_data()
    data["ropes"] = [
        {"name": "test_rope", "part": "rope_3mm_steel_generic", "from": {"body": "hook"}, "to": to, **extra}
    ]
    return filled_gantry(data)


@pytest.mark.req("BLD-032")
@pytest.mark.req("LIB-020")
def test_rope_between_bodies_is_a_neutral_lumped_chain() -> None:
    model = _physics(_rope_design({"body": "frame", "point": ["0 m", "0 m", "1 m"]}, segments=6))
    rope = model.ropes[0]
    assert (rope.name, rope.from_body, rope.to_body, rope.segments) == ("test_rope", "hook", "frame", 6)
    assert rope.from_point == (0.0, 0.0, 0.0)
    assert rope.to_point == (0.0, 0.0, 1.0)
    assert (rope.axial_stiffness, rope.damping, rope.breaking_tension) == (5.0e5, 50.0, 5.0e3)
    assert rope.mass_per_length == pytest.approx(0.035, rel=1e-15)
    assert rope.rest_length == pytest.approx(0.3, rel=1e-15)
    assert ("rope", "test_rope") in element_kinds(model)


@pytest.mark.req("BLD-032")
def test_rope_rest_length_override_and_world_anchor() -> None:
    model = _physics(_rope_design({"body": "world"}, rest_length="0.75 m"))
    assert model.ropes[0].to_body == "world"
    assert model.ropes[0].rest_length == 0.75
    assert model.ropes[0].segments == 8


@pytest.mark.req("BLD-032")
def test_rope_to_a_case_payload_is_noted_and_not_emitted() -> None:
    result = _build(gantry(), rigid_members=True)
    assert result.physics is not None
    assert result.physics.ropes == ()
    assert result.notes == (
        "rope 'payload_rope': its to end is case payload 'payload'; not emitted until the case places the payload "
        "(BLD-032)",
    )


@pytest.mark.req("BLD-032")
def test_rope_to_a_case_payload_is_listed_for_the_run() -> None:
    result = _build(gantry(), rigid_members=True)
    assert len(result.payload_ropes) == 1
    rope = result.payload_ropes[0]
    assert (rope.name, rope.from_body, rope.to_body, rope.segments) == ("payload_rope", "hook", "payload", 8)
    assert rope.from_point == (0.0, 0.0, -0.02)
    assert rope.to_point == (0.0, 0.0, 0.0)
    assert rope.rest_length == pytest.approx(0.3, rel=1e-15)
    between = _build(_rope_design({"body": "frame"}), rigid_members=True)
    assert between.payload_ropes == ()


# The shipped rope part's end fitting (LIB-020): its default, 1 g.
SHACKLE_MASS = 1e-3


@pytest.mark.req("LIB-020", "BLD-036")
def test_the_rope_parts_end_fitting_mass_is_data_and_the_shackle_takes_it() -> None:
    values = default_registry().part("rope_3mm_steel_generic").values
    assert values["end_fitting_mass"] == SHACKLE_MASS
    result = _build(gantry(), rigid_members=True)
    assert result.physics is not None
    hung = result.payload_ropes[0]
    heavy = with_payload(result.physics, "payload", "hook", 4.5, True, hung, None, 0.25)
    assert heavy.bodies[-2].mass == 0.25
    for missing in (None, 0.0, -1.0):
        with pytest.raises(BuildError, match="needs a positive end fitting mass"):
            with_payload(result.physics, "payload", "hook", 4.5, True, hung, None, missing)


def _hung(starts: dict[str, float] | None = None, carrier: str = "hook", **rope: Any) -> PhysicsModel:
    result = _build(gantry(), rigid_members=True)
    assert result.physics is not None
    hung = result.payload_ropes[0].model_copy(update=rope)
    return with_payload(result.physics, "payload", carrier, 4.5, True, hung, starts, SHACKLE_MASS)


def _world(
    model: PhysicsModel, body: str, point: tuple[float, float, float], starts: dict[str, float] | None = None
) -> tuple[float, float, float]:
    origin, orientation = payload_pose(model, body, starts)
    offset = q_rot(orientation, point)
    return (origin[0] + offset[0], origin[1] + offset[1], origin[2] + offset[2])


@pytest.mark.req("BLD-032")
@pytest.mark.req("BLD-036")
@pytest.mark.parametrize("starts", [None, {"x_axis": 0.1, "y_axis": 0.2, "lift_axis": 0.3}])
def test_payload_hangs_on_its_rope_straight_at_rest_length_from_the_start_pose(
    starts: dict[str, float] | None,
) -> None:
    model = _hung(starts)
    assert [body.name for body in model.bodies][-2:] == ["payload_shackle", "payload"]
    shackle, payload = model.bodies[-2], model.bodies[-1]
    assert shackle.joint is not None
    assert payload.joint is not None
    assert (shackle.parent, shackle.joint.name, shackle.joint.type) == ("hook", "payload_fixed", "fixed")
    assert shackle.joint.position == (0.0, 0.0, -0.02)
    assert shackle.shapes == ()
    assert shackle.mass == SHACKLE_MASS
    assert (payload.parent, payload.joint.name, payload.joint.type) == ("world", "payload_free", "free")
    assert payload.mass == 4.5
    rope = model.ropes[-1]
    assert (rope.from_body, rope.from_point, rope.to_body) == ("payload_shackle", (0.0, 0.0, 0.0), "payload")
    assert rope.to_point == pytest.approx((0.0, 0.0, PAYLOAD_EDGE / 2.0), abs=1e-15)
    hook_point = _world(model, "hook", (0.0, 0.0, -0.02), starts)
    top = _world(model, "payload", rope.to_point)
    moved = [0.0, 0.0, 0.0]
    if starts is not None:
        moved = [starts["x_axis"], starts["y_axis"], starts["lift_axis"]]
    base = _world(_hung(None), "hook", (0.0, 0.0, -0.02))
    assert hook_point == pytest.approx(tuple(base[i] + moved[i] for i in range(3)), abs=1e-12)
    assert (hook_point[0] - top[0], hook_point[1] - top[1]) == pytest.approx((0.0, 0.0), abs=1e-12)
    assert hook_point[2] - top[2] == pytest.approx(0.3, abs=1e-12)


@pytest.mark.req("BLD-032")
def test_payload_rope_from_another_carrier_or_to_another_payload_is_a_build_error() -> None:
    with pytest.raises(BuildError) as carrier:
        _hung(carrier="lift_carriage")
    assert carrier.value.failures == [
        "payload 'payload': carried by 'lift_carriage', but rope 'payload_rope' hangs from 'hook'"
    ]
    with pytest.raises(BuildError) as other:
        _hung(to_body="crate")
    assert other.value.failures == ["payload 'payload': rope 'payload_rope' ends at 'crate', not at this payload"]


@pytest.mark.req("BLD-024")
@pytest.mark.req("BLD-032")
def test_adapter_without_ropes_refuses_the_rope_element() -> None:
    design = _rope_design({"body": "frame"})
    kinds = {kind for kind, _name in element_kinds(_physics(design)) if kind != "rope"}
    with pytest.raises(BuildError) as raised:
        _build(design, supported_elements=kinds, adapter="bridge")
    assert raised.value.failures == ["element 'test_rope' (rope): adapter 'bridge' cannot map it (BLD-024)"]


# ===== Neutral rope and fastening validation (BLD-031, BLD-032) ==========


def _model_data(**extra: Any) -> dict[str, Any]:
    model = _physics(filled_gantry())
    data: dict[str, Any] = model.model_dump(mode="json")
    data.update(extra)
    return data


ROPE = {
    "name": "line",
    "from_body": "hook",
    "from_point": ["0.0 m", "0.0 m", "0.0 m"],
    "to_body": "world",
    "to_point": ["0.0 m", "0.0 m", "2.0 m"],
    "segments": 4,
    "axial_stiffness": "1000.0 N",
    "damping": "1.0 N*s",
    "mass_per_length": "0.01 kg/m",
    "rest_length": "1.0 m",
    "breaking_tension": "100.0 N",
}


@pytest.mark.req("BLD-032")
def test_neutral_rope_round_trips() -> None:
    model = PhysicsModel.model_validate(_model_data(ropes=[ROPE]))
    assert model.ropes[0].axial_stiffness == 1000.0
    assert PhysicsModel.model_validate(model.model_dump(mode="json")) == model


@pytest.mark.req("BLD-032")
@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"from_body": "ghost"}, "rope 'line': unknown body 'ghost'"),
        ({"to_body": "hook"}, "rope 'line': both ends on body 'hook'"),
        ({"name": "hook"}, "duplicate name 'hook'"),
        ({"segments": 0}, "segments"),
        ({"axial_stiffness": "0 N"}, "axial_stiffness"),
        ({"damping": "-1 N*s"}, "damping"),
        ({"mass_per_length": "0 kg/m"}, "mass_per_length"),
        ({"rest_length": "0 m"}, "rest_length"),
        ({"breaking_tension": "0 N"}, "breaking_tension"),
        ({"damping": "1 N*s/m"}, "damping"),
        ({"mass_per_length": "1 kg"}, "mass_per_length"),
    ],
)
def test_neutral_rope_rules(change: dict[str, Any], message: str) -> None:
    with pytest.raises(ValueError, match=message.replace("(", r"\(").replace(")", r"\)")):
        PhysicsModel.model_validate(_model_data(ropes=[{**ROPE, **change}]))


@pytest.mark.req("BLD-031")
@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"body_a": "ghost"}, "fastening 'rail_x_mount': unknown body 'ghost'"),
        ({"joint": "ghost"}, "fastening 'rail_x_mount': unknown joint 'ghost'"),
        ({"kind": "glue"}, "kind"),
    ],
)
def test_neutral_fastening_rules(change: dict[str, Any], message: str) -> None:
    data = _model_data()
    index = next(i for i, item in enumerate(data["fastenings"]) if item["name"] == "rail_x_mount")
    data["fastenings"][index].update(change)
    with pytest.raises(ValueError, match=message):
        PhysicsModel.model_validate(data)


@pytest.mark.req("BLD-031")
def test_neutral_duplicate_fastening_rejected() -> None:
    data = _model_data()
    data["fastenings"].append(dict(data["fastenings"][0]))
    with pytest.raises(ValueError, match=f"duplicate fastening '{data['fastenings'][0]['name']}'"):
        PhysicsModel.model_validate(data)


@pytest.mark.req("BLD-008")
def test_neutral_member_mount_must_be_a_joint() -> None:
    data = _model_data()
    data["structural_members"][0]["mounts"] = ["ghost"]
    with pytest.raises(ValueError, match="structural member 'rail_x': unknown mount joint 'ghost'"):
        PhysicsModel.model_validate(data)


@pytest.mark.req("BLD-019")
@pytest.mark.parametrize("segments", [1, 4, 7])
def test_calibrated_member_below_eight_segments_fails_the_build(segments: int) -> None:
    # Ruling 20: the 2 percent tolerance holds for N >= 8 (midspan chain error -1/N^2: -1.56 % at 8, -6.25 % at 4).
    failures = _failures(_bar(["clamped", "free"], calibration=CALIBRATION, segments=segments))
    assert failures == [
        f"flexible body 'bar': calibrated member has {segments} segments; calibration needs at least 8 for supports "
        "(clamped, free) (the 2 percent tolerance holds for N >= 8, and N >= 16 clamped-clamped, rulings 20 and 32; "
        "BLD-018, BLD-019)"
    ]


@pytest.mark.req("BLD-019")
def test_uncalibrated_member_below_eight_segments_builds() -> None:
    assert _physics(_bar(["clamped", "free"], segments=4)).lumped_members[0].segments == 4


@pytest.mark.req("BLD-019")
@pytest.mark.parametrize(
    ("deflection", "frequency", "flagged"),
    [
        # Tip deflection 19.92 mm and 8.18 Hz for the chain at N = 8: halving the deflection doubles E (scale 2.0
        # on the bound, no note); a tenth gives 9.96; the frequency then sets the mass scale.
        ("9.9609375 mm", "11.57 Hz", []),
        ("1.9921875 mm", "25.88 Hz", ["stiffness_scale"]),
        ("19 mm", "1 Hz", ["mass_scale"]),
        ("1 mm", "1 Hz", ["stiffness_scale", "mass_scale"]),
    ],
)
def test_scale_outside_half_to_double_is_noted(deflection: str, frequency: str, flagged: list[str]) -> None:
    calibration = {**CALIBRATION, "deflection": deflection, "first_natural_frequency": frequency}
    result = _build(_bar(["clamped", "free"], calibration=calibration))
    record = result.lumped[0]
    assert record.stiffness_scale is not None
    assert record.mass_scale is not None
    named = [note.split(" ")[4] for note in result.notes]
    assert named == flagged
    for note in result.notes:
        assert note.startswith("flexible body 'bar': calibration ")
        assert note.endswith("lies outside [0.5, 2.0]; the lumped beam is an approximation (BLD-019)")


def test_approximation_notes_bounds_are_inclusive() -> None:
    assert structure.approximation_notes("m", structure.CalibrationScales(0.5, 2.0)) == []
    notes = structure.approximation_notes("m", structure.CalibrationScales(0.49, 2.01))
    assert notes == [
        "flexible body 'm': calibration stiffness_scale 0.49 lies outside [0.5, 2.0]; the lumped beam is an "
        "approximation (BLD-019)",
        "flexible body 'm': calibration mass_scale 2.01 lies outside [0.5, 2.0]; the lumped beam is an approximation "
        "(BLD-019)",
    ]


@pytest.mark.req("BLD-019")
def test_clamped_clamped_calibration_is_noted_as_owed_in_a_rigid_free_check() -> None:
    design = _bar(["clamped", "clamped"], calibration=CALIBRATION)
    result = _build(design, rigid_members=True)
    assert result.notes == ()
    data = gantry_data()
    data["bodies"][1]["flexible"]["supports"] = ["pinned", "clamped"]
    failures = _failures(filled_gantry(data))
    assert any("supports (pinned, clamped) have no lumped beam" in failure for failure in failures)


@pytest.mark.req("BLD-019", "BLD-017")
def test_clamped_clamped_chain_matches_the_engine_closed_forms() -> None:
    """The clamped-clamped chain the calibration uses is the engine's (ruling 16): midspan deflection
    P L^3 / (192 E I) (1 - 4 / N^2) exactly (the chain's closed form), and its first mode 22.3733 / L^2 sqrt(E I / mu)
    +0.61 percent at N = 16 (engine contract). Calibrating to the chain's own deflection gives a stiffness scale of
    exactly one; to the continuous first mode a mass scale (1.0061)^2 within the rounding of the stated 0.61 percent."""
    from mechsim.builder.physics import clamped_calibration

    length, rigidity, mu, load = 2.0, 1.0e4, 3.0, 100.0
    chain = load * length**3 / (192.0 * rigidity) * (1.0 - 4.0 / 16**2)
    continuous = 22.3733 / length**2 * math.sqrt(rigidity / mu) / (2.0 * math.pi)
    scales = clamped_calibration(16, length, rigidity, mu, load, chain, continuous)
    assert scales.stiffness_scale == pytest.approx(1.0, rel=1e-9)
    assert scales.mass_scale == pytest.approx(1.0061**2, rel=2e-4)
    halved = clamped_calibration(16, length, rigidity, mu, load, chain / 2.0, continuous)
    assert halved.stiffness_scale == pytest.approx(2.0, rel=1e-9)


@pytest.mark.req("BLD-019")
@pytest.mark.parametrize("segments", [8, 15])
def test_calibrated_clamped_clamped_member_below_sixteen_segments_fails_the_build(segments: int) -> None:
    failures = _failures(_bar(["clamped", "clamped"], calibration=CALIBRATION, segments=segments))
    assert failures == [
        f"flexible body 'bar': calibrated member has {segments} segments; calibration needs at least 16 for supports "
        "(clamped, clamped) (the 2 percent tolerance holds for N >= 8, and N >= 16 clamped-clamped, rulings 20 and 32; "
        "BLD-018, BLD-019)"
    ]


@pytest.mark.req("BLD-019")
def test_clamped_clamped_calibration_applies_at_sixteen_segments() -> None:
    """Ruling 32: at sixteen segments (the clamped-clamped default) the calibration scales E and density."""
    from mechsim.builder.physics import Collector, _flexible

    design = _bar(["clamped", "clamped"], calibration=CALIBRATION)
    collector = Collector()
    _member, _entry, scales = _flexible(
        design.bodies[0], shipped_materials()[MATERIAL], design, default_registry(), collector, False
    )
    assert scales is not None
    assert not any("no calibration chain layout" in note for note in collector.notes)
    record = _build(design).lumped[0]
    assert record.stiffness_scale == scales.stiffness_scale
    assert record.mass_scale == scales.mass_scale


@pytest.mark.req("BLD-032")
@pytest.mark.req("LIB-020")
def test_rope_without_breaking_tension_never_snaps() -> None:
    rope = _rope_design({"body": "frame"}).ropes[0]
    values = dict(default_registry().part("rope_3mm_steel_generic").values)
    del values["breaking_tension"]
    element = structure.rope_element(rope, values)
    assert element is not None
    assert "breaking_tension" not in element
    data = _model_data(ropes=[{**ROPE}])
    del data["ropes"][0]["breaking_tension"]
    assert PhysicsModel.model_validate(data).ropes[0].breaking_tension is None


@pytest.mark.req("MAT-020")
@pytest.mark.req("BLD-009")
def test_design_material_overrides_reach_the_member_list(tmp_path: Path) -> None:
    from ruamel.yaml import YAML

    from mechsim.design.schema import design_materials, load_design
    from mechsim.testing.core_driver import core_builder

    data = _bar_data(["clamped", "free"], section={"area": "1 cm**2", "second_moment": "1 cm**4", "length": "1 m"})
    data["materials"] = {"overrides": "overrides.yaml"}
    del data["bodies"][0]["catalog_part"]
    (tmp_path / "overrides.yaml").write_text(
        "material_overrides:\n"
        f"  {MATERIAL}:\n"
        "    yield_strength: {source: coupon test, value: 111 MPa}\n"
        "    density: {source: weighed, value: 2500 kg/m**3}\n",
        encoding="utf-8",
    )
    path = tmp_path / "design.yaml"
    YAML().dump(data, path)
    loaded = load_design(path)
    result = core_builder(loaded)
    member = result.members[0]
    assert member.material == MATERIAL
    assert member.section.density == 2500.0
    assert design_materials(loaded)[member.material].yield_strength.typ == 111e6
    assert shipped_materials()[MATERIAL].density.typ != 2500.0
