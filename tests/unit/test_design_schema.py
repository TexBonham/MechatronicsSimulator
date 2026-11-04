"""Unit tier: design file schema, cross-reference validation, loader, and the CAD mass cross-check (DES)."""

from __future__ import annotations

import copy
import functools
import io
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError
from ruamel.yaml import YAML

from mechsim.design import cad
from mechsim.design.chip import Chip, shipped_chips
from mechsim.design.schema import (
    Design,
    OpenValue,
    StepGeometry,
    blob_hash,
    design_identity,
    load_design,
    validate_design_file,
    yaml_loader,
)
from mechsim.library.registry import Registry, load_registry
from mechsim.library.schema import CATALOG_DIR
from mechsim.physics.neutral import ContactModel as NeutralContactModel
from mechsim.physics.neutral import ContactParameters as NeutralContactParameters

ROOT = Path(__file__).resolve().parents[2]
GANTRY = ROOT / "designs" / "gantry" / "design.yaml"
PENDULUM = ROOT / "designs" / "pendulum" / "design.yaml"


@functools.cache
def _raw(path: Path) -> Any:
    return YAML(typ="safe").load(path.read_text(encoding="utf-8"))


def gantry() -> dict[str, Any]:
    return copy.deepcopy(_raw(GANTRY))


def pendulum() -> dict[str, Any]:
    return copy.deepcopy(_raw(PENDULUM))


def rejected(data: Any, *fragments: str) -> None:
    with pytest.raises(ValidationError) as caught:
        Design.model_validate(data)
    text = str(caught.value)
    for fragment in fragments:
        assert fragment in text, text


def accepted(data: Any) -> Design:
    return Design.model_validate(data)


def component(data: dict[str, Any], name: str) -> dict[str, Any]:
    for item in data["components"]:
        if item["name"] == name:
            found: dict[str, Any] = item
            return found
    raise KeyError(name)


def joint(data: dict[str, Any], name: str) -> dict[str, Any]:
    for item in data["joints"]:
        if item["name"] == name:
            found: dict[str, Any] = item
            return found
    raise KeyError(name)


def link(data: dict[str, Any]) -> dict[str, Any]:
    found: dict[str, Any] = data["bodies"][0]
    return found


def pivot_dof(data: dict[str, Any]) -> dict[str, Any]:
    found: dict[str, Any] = data["joints"][0]["angular"][0]
    return found


def printed_process() -> dict[str, Any]:
    return {"kind": "printed", "build_direction": [0.0, 0.0, 1.0], "walls": 3, "infill": "20 percent"}


def step_geometry() -> dict[str, Any]:
    return {"kind": "step", "file": "cad/link.step", "mass_check_tolerance": "5 percent"}


# ===== Shipped examples ================================================


@pytest.mark.req("DES-011")
@pytest.mark.req("DES-026")
def test_pendulum_is_a_release_design_matching_the_engine_reference() -> None:
    design = accepted(pendulum())
    assert design.status == "release"
    assert design.open_parameters == []
    body = design.bodies[0]
    assert body.material == "aluminium_6063_t5_extrusion"
    assert body.mass_properties is not None
    assert not isinstance(body.mass_properties, OpenValue)
    assert body.mass_properties.mass == pytest.approx(1.0)
    assert design.joints[0].type == "revolute"
    assert design.components == []
    assert design.firmware is None


@pytest.mark.req("DES-011")
@pytest.mark.req("DES-025")
def test_gantry_is_a_draft_design_with_open_parameters() -> None:
    design = accepted(gantry())
    assert design.status == "draft"
    assert len(design.open_parameters) > 0
    assert isinstance(design.bodies[0].geometry, OpenValue)
    assert design.bodies[0].geometry.name == "cad_model"


@pytest.mark.req("DES-014")
@pytest.mark.parametrize("path", [GANTRY, PENDULUM], ids=["gantry", "pendulum"])
def test_round_trip_preserves_bytes_comments_and_order(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    loader = yaml_loader()
    data = loader.load(text)
    stream = io.StringIO()
    loader.dump(data, stream)
    assert stream.getvalue() == text
    assert list(data.keys())[:4] == ["name", "version", "source_hash", "status"]


@pytest.mark.req("DES-013")
def test_quantities_are_held_as_si_floats() -> None:
    design = accepted(pendulum())
    shape = design.bodies[0].geometry
    assert not isinstance(shape, OpenValue)
    assert shape.kind == "primitives"
    cylinder = shape.shapes[0]
    assert cylinder.shape == "cylinder"
    assert cylinder.radius == pytest.approx(0.01)
    assert design.joints[0].angular[0].damping == pytest.approx(0.04520785330006282)
    assert design.environment.gravity == [0.0, pytest.approx(-9.81), 0.0]
    assert design.environment.ambient_temperature == pytest.approx(298.15)
    gantry_design = accepted(gantry())
    assert gantry_design.firmware is not None
    assert gantry_design.firmware.tick_rate == pytest.approx(1000.0)
    assert gantry_design.structural is not None
    assert gantry_design.structural.accuracy_target == pytest.approx(0.01)
    assert gantry_design.structural.design_load.allowance == pytest.approx(0.15)


@pytest.mark.req("DES-013")
@pytest.mark.parametrize("value", [1.0, 1, "1", "", "kg", True, None])
def test_bare_or_missing_unit_rejected(value: Any) -> None:
    data = pendulum()
    link(data)["mass_properties"]["mass"] = value
    rejected(data, "mass_properties")


@pytest.mark.req("DES-013")
@pytest.mark.parametrize("value", ["1 m", "1 s", "1 N"])
def test_wrong_unit_rejected(value: str) -> None:
    data = pendulum()
    link(data)["mass_properties"]["mass"] = value
    rejected(data, "expected [mass]")


@pytest.mark.req("DES-013")
@pytest.mark.parametrize("value", ["1000 g", "2.2046226218 lb", "1 kg"])
def test_other_units_convert_to_si(value: str) -> None:
    data = pendulum()
    link(data)["mass_properties"]["mass"] = value
    properties = accepted(data).bodies[0].mass_properties
    assert properties is not None
    assert not isinstance(properties, OpenValue)
    assert properties.mass == pytest.approx(1.0, rel=1e-9)


# ===== Header and version ==============================================


@pytest.mark.req("DES-008")
@pytest.mark.parametrize("field", ["name", "version", "source_hash", "status", "environment", "bodies", "joints"])
def test_required_top_level_field_missing(field: str) -> None:
    data = pendulum()
    del data[field]
    rejected(data, field)


@pytest.mark.req("DES-008")
@pytest.mark.parametrize("version", ["1.0", "v1.0.0", "1.0.0-rc", "", 1])
def test_bad_semver_rejected(version: Any) -> None:
    data = pendulum()
    data["version"] = version
    rejected(data, "version")


@pytest.mark.req("DES-008")
@pytest.mark.parametrize("value", ["unset", "0" * 40, "a1b2c3d4e5" * 4])
def test_source_hash_accepted(value: str) -> None:
    data = pendulum()
    data["source_hash"] = value
    assert accepted(data).source_hash == value


@pytest.mark.req("DES-008")
@pytest.mark.parametrize("value", ["0" * 39, "0" * 41, "A" * 40, "g" * 40, "", "UNSET"])
def test_source_hash_rejected(value: str) -> None:
    data = pendulum()
    data["source_hash"] = value
    rejected(data, "source_hash")


@pytest.mark.req("DES-008")
def test_loader_exposes_version_and_git_blob_hash() -> None:
    version, blob = design_identity(PENDULUM)
    assert version == "0.1.0"
    expected = subprocess.run(
        ["git", "hash-object", str(PENDULUM)], capture_output=True, text=True, check=True, cwd=ROOT
    ).stdout.strip()
    assert blob == expected
    assert blob_hash(PENDULUM) == expected


@pytest.mark.req("DES-008")
@pytest.mark.parametrize("status", ["final", "", "Draft", None])
def test_bad_status_rejected(status: Any) -> None:
    data = pendulum()
    data["status"] = status
    rejected(data, "status")


@pytest.mark.req("DES-015")
@pytest.mark.parametrize(
    ("where", "key"),
    [("top", "colour"), ("body", "colour"), ("joint", "colour"), ("dof", "colour"), ("environment", "colour")],
)
def test_unknown_key_rejected(where: str, key: str) -> None:
    data = pendulum()
    targets = {
        "top": data,
        "body": link(data),
        "joint": data["joints"][0],
        "dof": pivot_dof(data),
        "environment": data["environment"],
    }
    targets[where][key] = "x"
    rejected(data, "Extra inputs are not permitted")


@pytest.mark.req("DES-015")
@pytest.mark.parametrize(
    ("field", "value"),
    [("bodies", {"link": {}}), ("joints", "pivot"), ("name", 3), ("open_parameters", "none"), ("components", {})],
)
def test_wrong_type_rejected(field: str, value: Any) -> None:
    data = pendulum()
    data[field] = value
    rejected(data, field)


@pytest.mark.req("DES-015")
@pytest.mark.parametrize("name", ["Pendulum", "1pendulum", "pend-ulum", "", "pendulum.v1"])
def test_bad_names_rejected(name: str) -> None:
    data = pendulum()
    data["name"] = name
    rejected(data, "name")
    data = pendulum()
    link(data)["name"] = name
    rejected(data, "bodies")


@pytest.mark.req("DES-001")
@pytest.mark.req("DES-002")
@pytest.mark.parametrize("field", ["bodies", "joints"])
def test_empty_bodies_or_joints_rejected(field: str) -> None:
    data = pendulum()
    data[field] = []
    rejected(data, field)


@pytest.mark.req("DES-015")
@pytest.mark.parametrize("field", ["components", "pin_bindings", "operator_messages", "open_parameters"])
def test_empty_optional_collections_accepted(field: str) -> None:
    data = pendulum()
    data[field] = []
    accepted(data)


# ===== Open parameters =================================================


@pytest.mark.req("DES-038")
def test_release_design_with_open_parameter_rejected() -> None:
    data = pendulum()
    data["open_parameters"] = [{"name": "rod_mass", "dimension": "[mass]", "owed_by": "owner", "note": "test"}]
    rejected(data, "a release design may not have open parameters (rod_mass)")


@pytest.mark.req("DES-038")
def test_release_design_with_open_reference_rejected() -> None:
    data = pendulum()
    link(data)["mass_properties"]["mass"] = "${rod_mass}"
    rejected(data, "undeclared open parameter ${rod_mass}")


@pytest.mark.req("DES-038")
def test_draft_with_declared_reference_accepted() -> None:
    data = pendulum()
    data["status"] = "draft"
    data["open_parameters"] = [{"name": "rod_mass", "dimension": "[mass]", "owed_by": "owner", "note": "test"}]
    link(data)["mass_properties"]["mass"] = "${rod_mass}"
    properties = accepted(data).bodies[0].mass_properties
    assert properties is not None
    assert not isinstance(properties, OpenValue)
    assert properties.mass == OpenValue("rod_mass", "[mass]")


@pytest.mark.req("DES-038")
def test_draft_with_undeclared_reference_rejected() -> None:
    data = gantry()
    data["bodies"][0]["material"] = "${frame_alloy}"
    rejected(data, "bodies[0].material: references undeclared open parameter ${frame_alloy}")


@pytest.mark.req("DES-038")
def test_open_reference_with_wrong_dimension_rejected() -> None:
    data = gantry()
    joint(data, "x_axis")["linear"][0]["limits"]["upper"] = "${rated_load}"
    rejected(data, "open parameter 'rated_load' has dimension [mass], expected [length]")


@pytest.mark.req("DES-038")
def test_choice_parameter_in_quantity_field_rejected() -> None:
    data = gantry()
    joint(data, "x_axis")["linear"][0]["friction"] = "${cad_model}"
    rejected(data, "open parameter 'cad_model' has dimension choice")


@pytest.mark.req("DES-038")
def test_quantity_parameter_in_choice_field_rejected() -> None:
    data = gantry()
    data["bodies"][0]["geometry"] = "${workspace_x}"
    rejected(data, "expected choice")


@pytest.mark.req("DES-038")
def test_duplicate_open_parameter_rejected() -> None:
    data = gantry()
    data["open_parameters"].append(copy.deepcopy(data["open_parameters"][0]))
    rejected(data, "duplicate open parameter 'workspace_x'")


@pytest.mark.req("DES-038")
@pytest.mark.parametrize("field", ["name", "dimension", "owed_by", "note"])
def test_open_parameter_field_missing(field: str) -> None:
    data = gantry()
    del data["open_parameters"][0][field]
    rejected(data, field)


@pytest.mark.req("DES-038")
@pytest.mark.parametrize("dimension", ["[wobble]", "", "furlongs per"])
def test_open_parameter_unknown_dimension_rejected(dimension: str) -> None:
    data = gantry()
    data["open_parameters"][0]["dimension"] = dimension
    rejected(data, "dimension")


@pytest.mark.req("DES-038")
@pytest.mark.parametrize("text", ["${}", "${Rated}", "$rated_load", "{rated_load}", "${rated load}"])
def test_malformed_open_reference_is_not_an_open_value(text: str) -> None:
    data = gantry()
    data["structural"]["design_load"]["rated_load"] = text
    rejected(data, "rated_load")


@pytest.mark.req("DES-038")
def test_unreferenced_open_parameter_allowed_in_draft() -> None:
    data = gantry()
    names = [item["name"] for item in data["open_parameters"]]
    assert "swing_limit_angle" in names
    accepted(data)


# ===== Bodies ==========================================================


@pytest.mark.req("DES-001")
@pytest.mark.req("DES-037")
@pytest.mark.parametrize("field", ["name", "material", "process", "geometry"])
def test_body_required_field_missing(field: str) -> None:
    data = pendulum()
    del link(data)[field]
    rejected(data, field)


@pytest.mark.req("DES-001")
@pytest.mark.req("DES-016")
def test_unknown_material_rejected() -> None:
    data = pendulum()
    link(data)["material"] = "unobtainium"
    rejected(data, "bodies[0].material: unknown material 'unobtainium'")


@pytest.mark.req("DES-001")
@pytest.mark.parametrize("kind", ["machined", "molded", "extrusion"])
def test_non_printed_processes_accepted(kind: str) -> None:
    data = pendulum()
    link(data)["process"] = {"kind": kind}
    accepted(data)


@pytest.mark.req("DES-001")
def test_printed_process_on_printed_material_accepted() -> None:
    data = pendulum()
    link(data)["material"] = "pla_printed"
    link(data)["process"] = printed_process()
    accepted(data)


@pytest.mark.req("DES-001")
def test_printed_process_on_non_printed_material_rejected() -> None:
    data = pendulum()
    link(data)["process"] = printed_process()
    rejected(data, "process 'printed' does not match material 'aluminium_6063_t5_extrusion'")


@pytest.mark.req("DES-001")
def test_printed_material_without_printed_process_rejected() -> None:
    data = pendulum()
    link(data)["material"] = "pla_printed"
    rejected(data, "process 'extrusion' does not match material 'pla_printed'")


@pytest.mark.req("DES-001")
@pytest.mark.parametrize("kind", ["cast", "", None])
def test_unknown_process_rejected(kind: Any) -> None:
    data = pendulum()
    link(data)["process"] = {"kind": kind}
    rejected(data, "process")


@pytest.mark.req("DES-001")
@pytest.mark.parametrize("field", ["build_direction", "walls", "infill"])
def test_printed_process_field_missing(field: str) -> None:
    data = pendulum()
    link(data)["material"] = "pla_printed"
    process = printed_process()
    del process[field]
    link(data)["process"] = process
    rejected(data, field)


@pytest.mark.req("DES-001")
@pytest.mark.parametrize(
    ("infill", "ok"),
    [
        ("0 percent", True),
        ("100 percent", True),
        ("50 percent", True),
        ("101 percent", False),
        ("-1 percent", False),
        ("0.5 m", False),
    ],
)
def test_printed_infill_bounds(infill: str, ok: bool) -> None:
    data = pendulum()
    link(data)["material"] = "pla_printed"
    process = printed_process()
    process["infill"] = infill
    link(data)["process"] = process
    if ok:
        accepted(data)
    else:
        rejected(data, "infill")


@pytest.mark.req("DES-001")
@pytest.mark.parametrize(("walls", "ok"), [(0, False), (1, True), (2, True), (-1, False), ("2", False)])
def test_printed_walls_bounds(walls: Any, ok: bool) -> None:
    data = pendulum()
    link(data)["material"] = "pla_printed"
    process = printed_process()
    process["walls"] = walls
    link(data)["process"] = process
    if ok:
        accepted(data)
    else:
        rejected(data, "walls")


@pytest.mark.req("DES-001")
@pytest.mark.parametrize("direction", [[0.0, 0.0, 2.0], [0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
def test_printed_build_direction_must_be_unit_three_vector(direction: list[float]) -> None:
    data = pendulum()
    link(data)["material"] = "pla_printed"
    process = printed_process()
    process["build_direction"] = direction
    link(data)["process"] = process
    rejected(data, "build_direction")


@pytest.mark.req("DES-001")
@pytest.mark.parametrize(
    "shape",
    [
        {"shape": "sphere", "position": ["0 m", "0 m", "0 m"], "radius": "1 cm"},
        {"shape": "box", "position": ["0 m", "0 m", "0 m"], "size": ["1 cm", "2 cm", "3 cm"]},
        {"shape": "capsule", "position": ["0 m", "0 m", "0 m"], "radius": "1 cm", "length": "4 cm"},
    ],
    ids=["sphere", "box", "capsule"],
)
def test_primitive_shapes_accepted(shape: dict[str, Any]) -> None:
    data = pendulum()
    link(data)["geometry"]["shapes"].append(shape)
    accepted(data)


@pytest.mark.req("DES-001")
@pytest.mark.parametrize(("radius", "ok"), [("0 m", False), ("-1 mm", False), ("1e-9 m", True)])
def test_shape_radius_bounds(radius: str, ok: bool) -> None:
    data = pendulum()
    link(data)["geometry"]["shapes"][0]["radius"] = radius
    if ok:
        accepted(data)
    else:
        rejected(data, "cylinder radius must be positive")


@pytest.mark.req("DES-001")
@pytest.mark.parametrize(("size", "ok"), [(["1 cm", "0 cm", "1 cm"], False), (["1 cm", "1 cm", "1 cm"], True)])
def test_box_size_bounds(size: list[str], ok: bool) -> None:
    data = pendulum()
    link(data)["geometry"]["shapes"] = [{"shape": "box", "position": ["0 m", "0 m", "0 m"], "size": size}]
    if ok:
        accepted(data)
    else:
        rejected(data, "box size must be positive")


@pytest.mark.req("DES-001")
def test_empty_primitive_shapes_rejected() -> None:
    data = pendulum()
    link(data)["geometry"]["shapes"] = []
    rejected(data, "shapes")


@pytest.mark.req("DES-001")
def test_non_unit_shape_orientation_rejected() -> None:
    data = pendulum()
    link(data)["geometry"]["shapes"][0]["orientation"] = [1.0, 1.0, 0.0, 0.0]
    rejected(data, "unit quaternion")


@pytest.mark.req("DES-022")
def test_step_geometry_needs_declared_mass_properties() -> None:
    data = pendulum()
    link(data)["geometry"] = step_geometry()
    del link(data)["mass_properties"]
    rejected(data, "STEP geometry needs declared mass_properties")


@pytest.mark.req("DES-022")
@pytest.mark.parametrize(("tolerance", "ok"), [("0 percent", False), ("-5 percent", False), ("5 percent", True)])
def test_step_mass_check_tolerance_bounds(tolerance: str, ok: bool) -> None:
    data = pendulum()
    geometry = step_geometry()
    geometry["mass_check_tolerance"] = tolerance
    link(data)["geometry"] = geometry
    if ok:
        accepted(data)
    else:
        rejected(data, "mass_check_tolerance must be positive")


@pytest.mark.req("DES-027")
@pytest.mark.parametrize("file", ["/abs/link.step", "../link.step", "cad/../../link.step", "cad/link.obj", ""])
def test_bad_step_paths_rejected(file: str) -> None:
    data = pendulum()
    geometry = step_geometry()
    geometry["file"] = file
    link(data)["geometry"] = geometry
    rejected(data, "file")


@pytest.mark.req("DES-027")
def test_step_file_resolved_relative_to_design_file(tmp_path: Path) -> None:
    data = pendulum()
    link(data)["geometry"] = step_geometry()
    path = tmp_path / "design.yaml"
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    path.write_text(stream.getvalue(), encoding="utf-8")
    errors = validate_design_file(path)
    assert errors == [f"{path}: bodies[0].geometry.file: missing CAD file 'cad/link.step'"]
    (tmp_path / "cad").mkdir()
    (tmp_path / "cad" / "link.step").write_text("ISO-10303-21;\n", encoding="utf-8")
    assert validate_design_file(path) == []
    design = load_design(path)
    assert isinstance(design.bodies[0].geometry, StepGeometry)


@pytest.mark.req("DES-001")
@pytest.mark.parametrize(("mass", "ok"), [("0 kg", False), ("-1 kg", False), ("1e-9 kg", True)])
def test_mass_bounds(mass: str, ok: bool) -> None:
    data = pendulum()
    link(data)["mass_properties"]["mass"] = mass
    if ok:
        accepted(data)
    else:
        rejected(data, "mass must be positive")


@pytest.mark.req("DES-001")
@pytest.mark.parametrize("term", ["ixx", "iyy", "izz"])
def test_principal_inertia_must_be_positive(term: str) -> None:
    data = pendulum()
    link(data)["mass_properties"]["inertia"][term] = "0 kg*m**2"
    rejected(data, f"{term} must be positive")


@pytest.mark.req("DES-001")
@pytest.mark.parametrize("field", ["mass", "com", "inertia"])
def test_mass_properties_field_missing(field: str) -> None:
    data = pendulum()
    del link(data)["mass_properties"][field]
    rejected(data, field)


@pytest.mark.req("DES-023")
def test_body_declared_as_catalog_part() -> None:
    data = pendulum()
    link(data)["catalog_part"] = {"type": "stepper_behavioral", "part": "nema17_class_generic"}
    design = accepted(data)
    assert design.bodies[0].catalog_part is not None
    assert design.bodies[0].catalog_part.part == "nema17_class_generic"


@pytest.mark.req("DES-016")
@pytest.mark.req("DES-023")
@pytest.mark.parametrize(
    ("reference", "message"),
    [
        ({"type": "warp_drive", "part": "nema17_class_generic"}, "unknown component type 'warp_drive'"),
        ({"type": "stepper_behavioral", "part": "nema99"}, "unknown catalog part 'nema99'"),
        ({"type": "limit_switch", "part": "nema17_class_generic"}, "is not of type 'limit_switch'"),
    ],
)
def test_dangling_body_catalog_part_rejected(reference: dict[str, str], message: str) -> None:
    data = pendulum()
    link(data)["catalog_part"] = reference
    rejected(data, message)


@pytest.mark.req("DES-033")
@pytest.mark.req("DES-034")
def test_flexible_member_with_hand_calibration_accepted() -> None:
    data = pendulum()
    link(data)["flexible"] = {
        "section": {"area": "1 cm**2", "second_moment": "1 cm**4", "length": "0.5 m"},
        "supports": ["clamped", "free"],
        "calibration": {"source": "hand", "load": "10 N", "deflection": "1 mm", "first_natural_frequency": "40 Hz"},
    }
    flexible = accepted(data).bodies[0].flexible
    assert flexible is not None
    assert flexible.segments == 4
    assert flexible.section.area == pytest.approx(1e-4)
    assert flexible.section.second_moment == pytest.approx(1e-8)


@pytest.mark.req("DES-035")
def test_fea_calibration_needs_design_version() -> None:
    data = pendulum()
    calibration = {"source": "fea", "load": "10 N", "deflection": "1 mm", "first_natural_frequency": "40 Hz"}
    link(data)["flexible"] = {
        "section": {"area": "1 cm**2", "second_moment": "1 cm**4", "length": "0.5 m"},
        "supports": ["clamped", "free"],
        "calibration": calibration,
    }
    rejected(data, "an FEA calibration must name the design_version")
    calibration["design_version"] = "0.1.0"
    accepted(data)


@pytest.mark.req("DES-033")
@pytest.mark.req("DES-034")
@pytest.mark.parametrize(
    ("where", "field", "value"),
    [
        ("section", "area", "0 m**2"),
        ("section", "second_moment", "-1 cm**4"),
        ("section", "length", "0 m"),
        ("calibration", "load", "0 N"),
        ("calibration", "deflection", "0 mm"),
        ("calibration", "first_natural_frequency", "0 Hz"),
        ("calibration", "source", "guess"),
        ("section", "area", "1 cm"),
    ],
)
def test_flexible_member_bounds(where: str, field: str, value: str) -> None:
    data = pendulum()
    link(data)["flexible"] = {
        "section": {"area": "1 cm**2", "second_moment": "1 cm**4", "length": "0.5 m"},
        "supports": ["clamped", "free"],
        "calibration": {"source": "hand", "load": "10 N", "deflection": "1 mm", "first_natural_frequency": "40 Hz"},
    }
    link(data)["flexible"][where][field] = value
    rejected(data, field)


@pytest.mark.req("DES-033")
@pytest.mark.parametrize(("segments", "ok"), [(0, False), (1, True), (8, True)])
def test_flexible_member_segments_bounds(segments: int, ok: bool) -> None:
    data = gantry()
    data["bodies"][1]["flexible"]["segments"] = segments
    if ok:
        accepted(data)
    else:
        rejected(data, "segments")


@pytest.mark.req("DES-033")
def test_gantry_rails_are_flexible() -> None:
    design = accepted(gantry())
    flexible = [body.name for body in design.bodies if body.flexible is not None]
    assert flexible == ["rail_x", "rail_y"]


@pytest.mark.req("DES-001")
@pytest.mark.req("DES-016")
def test_fasteners_validate_grade_count_and_preload() -> None:
    data = pendulum()
    fastener = {"kind": "m5_socket_head", "count": 4, "grade": "steel_8_8_fastener", "preload": "0 N"}
    link(data)["fasteners"] = [fastener]
    accepted(data)
    fastener["count"] = 0
    rejected(data, "count")
    fastener["count"] = 1
    fastener["preload"] = "-1 N"
    rejected(data, "fastener preload must not be negative")
    fastener["preload"] = "1 kN"
    fastener["grade"] = "mystery_steel"
    rejected(data, "bodies[0].fasteners[0].grade: unknown material 'mystery_steel'")


@pytest.mark.req("DES-016")
def test_duplicate_and_reserved_body_names_rejected() -> None:
    data = gantry()
    data["bodies"][1]["name"] = "frame"
    rejected(data, "duplicate body, joint or component name 'frame'")
    data = gantry()
    data["joints"][0]["name"] = "frame"
    rejected(data, "duplicate body, joint or component name 'frame'")
    data = pendulum()
    link(data)["name"] = "world"
    rejected(data, "'world' is reserved")


# ===== Joints ==========================================================


@pytest.mark.req("DES-002")
@pytest.mark.parametrize("field", ["name", "type", "parent", "child", "placement"])
def test_joint_required_field_missing(field: str) -> None:
    data = pendulum()
    del data["joints"][0][field]
    rejected(data, field)


@pytest.mark.req("DES-002")
@pytest.mark.parametrize("field", ["friction", "damping", "backlash"])
def test_dof_required_field_missing(field: str) -> None:
    data = pendulum()
    del pivot_dof(data)[field]
    rejected(data, field)


@pytest.mark.req("DES-002")
@pytest.mark.parametrize(
    ("field", "value", "ok"),
    [
        ("friction", "0 N*m", True),
        ("friction", "-0.1 N*m", False),
        ("damping", "-1 N*m*s/rad", False),
        ("backlash", "-1 deg", False),
        ("backlash", "1 deg", True),
        ("friction", "1 N", False),
    ],
)
def test_dof_bounds_and_units(field: str, value: str, ok: bool) -> None:
    data = pendulum()
    pivot_dof(data)[field] = value
    if ok:
        accepted(data)
    else:
        rejected(data, field)


@pytest.mark.req("DES-002")
@pytest.mark.parametrize("joint_type", ["prismatic", "fixed", "planar", "spherical", "free"])
def test_dof_count_must_match_joint_type(joint_type: str) -> None:
    data = pendulum()
    data["joints"][0]["type"] = joint_type
    rejected(data, f"of type {joint_type} needs")


@pytest.mark.req("DES-002")
def test_every_joint_type_accepted_with_its_dofs() -> None:
    linear = {"friction": "0 N", "damping": "0 N*s/m", "backlash": "0 m"}
    angular = {"friction": "0 N*m", "damping": "0 N*m*s/rad", "backlash": "0 rad"}
    shapes = {
        "revolute": ([], [angular], True),
        "prismatic": ([linear], [], True),
        "fixed": ([], [], False),
        "planar": ([linear, linear], [angular], True),
        "spherical": ([], [angular, angular, angular], False),
        "free": ([linear, linear, linear], [angular, angular, angular], False),
    }
    for joint_type, (linear_dofs, angular_dofs, has_axis) in shapes.items():
        data = pendulum()
        item = data["joints"][0]
        item["type"] = joint_type
        item["linear"] = linear_dofs
        item["angular"] = angular_dofs
        if not has_axis:
            del item["axis"]
        assert accepted(data).joints[0].type == joint_type


@pytest.mark.req("DES-002")
@pytest.mark.parametrize("joint_type", ["hinge", "ball", ""])
def test_unknown_joint_type_rejected(joint_type: str) -> None:
    data = pendulum()
    data["joints"][0]["type"] = joint_type
    rejected(data, "type")


@pytest.mark.req("DES-002")
def test_axis_required_for_revolute_and_forbidden_for_fixed() -> None:
    data = pendulum()
    del data["joints"][0]["axis"]
    rejected(data, "needs an axis")
    data = gantry()
    joint(data, "frame_mount")["axis"] = [1.0, 0.0, 0.0]
    rejected(data, "takes no axis")


@pytest.mark.req("DES-002")
@pytest.mark.parametrize("axis", [[0.0, 0.0, 2.0], [0.0, 0.0, 0.0], [0.0, 1.0]])
def test_axis_must_be_unit_three_vector(axis: list[float]) -> None:
    data = pendulum()
    data["joints"][0]["axis"] = axis
    rejected(data, "axis")


@pytest.mark.req("DES-002")
def test_limits_rejected_on_spherical_joint() -> None:
    data = gantry()
    joint(data, "hook_swing")["angular"][0]["limits"] = {"lower": "-10 deg", "upper": "10 deg"}
    rejected(data, "takes no limits")


@pytest.mark.req("DES-028")
@pytest.mark.parametrize(
    ("lower", "upper", "ok"),
    [("-90 deg", "90 deg", True), ("45 deg", "45 deg", True), ("91 deg", "90 deg", False)],
)
def test_angular_limit_order(lower: str, upper: str, ok: bool) -> None:
    data = pendulum()
    pivot_dof(data)["limits"] = {"lower": lower, "upper": upper}
    if ok:
        accepted(data)
    else:
        rejected(data, "exceeds upper")


@pytest.mark.req("DES-028")
@pytest.mark.parametrize(
    ("lower", "upper", "ok"),
    [("0 m", "1 m", True), ("1 m", "1 m", True), ("1.001 m", "1 m", False), ("0 m", "1 kg", False)],
)
def test_linear_limit_order(lower: str, upper: str, ok: bool) -> None:
    data = gantry()
    joint(data, "x_axis")["linear"][0]["limits"] = {"lower": lower, "upper": upper}
    if ok:
        accepted(data)
    else:
        rejected(data, "limits")


@pytest.mark.req("DES-002")
@pytest.mark.parametrize("missing", ["lower", "upper"])
def test_limits_need_both_bounds(missing: str) -> None:
    data = pendulum()
    limits = {"lower": "-1 rad", "upper": "1 rad"}
    del limits[missing]
    pivot_dof(data)["limits"] = limits
    rejected(data, missing)


@pytest.mark.req("DES-016")
@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("parent", "ghost", "joints[0].parent: unknown body 'ghost'"),
        ("child", "ghost", "joints[0].child: unknown body 'ghost'"),
    ],
)
def test_dangling_joint_body_rejected(field: str, value: str, message: str) -> None:
    data = pendulum()
    data["joints"][0][field] = value
    rejected(data, message)


@pytest.mark.req("DES-002")
def test_joint_tree_rules() -> None:
    data = pendulum()
    data["joints"][0]["parent"] = "link"
    rejected(data, "joins body 'link' to itself")
    data = gantry()
    joint(data, "x_motor_spin")["child"] = "rail_y"
    rejected(data, "body 'rail_y' is the child of more than one joint", "body 'x_motor_shaft' is the child of no joint")
    data = gantry()
    joint(data, "frame_mount")["parent"] = "hook"
    rejected(data, "is in a joint cycle")


@pytest.mark.req("DES-002")
def test_placement_quaternion_must_be_unit() -> None:
    data = pendulum()
    data["joints"][0]["placement"]["orientation"] = [0.5, 0.0, 0.0, 0.0]
    rejected(data, "unit quaternion")


# ===== Components ======================================================


@pytest.mark.req("DES-003")
@pytest.mark.parametrize("field", ["name", "type", "part"])
def test_component_required_field_missing(field: str) -> None:
    data = gantry()
    del component(data, "x_motor")[field]
    rejected(data, field)


@pytest.mark.req("DES-003")
@pytest.mark.req("DES-016")
@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("type", "warp_drive", "unknown component type 'warp_drive'"),
        ("part", "nema99", "unknown catalog part 'nema99'"),
        ("part", "microswitch_generic", "part 'microswitch_generic' is not of type 'stepper_behavioral'"),
    ],
)
def test_dangling_component_type_or_part_rejected(field: str, value: str, message: str) -> None:
    data = gantry()
    component(data, "x_motor")[field] = value
    rejected(data, message)


@pytest.mark.req("DES-030")
@pytest.mark.parametrize(
    ("value", "ok"), [("0.5", True), ("1", True), ("1.5", False), ("0.001", False), ("0.5 m", False), (True, False)]
)
def test_override_validated_against_type(value: Any, ok: bool) -> None:
    data = gantry()
    component(data, "x_belt")["overrides"] = {"efficiency": value}
    if ok:
        accepted(data)
    else:
        rejected(data, "components[2].overrides")


@pytest.mark.req("DES-030")
def test_override_of_unknown_parameter_rejected() -> None:
    data = gantry()
    component(data, "x_belt")["overrides"] = {"colour": "1 m"}
    rejected(data, "unknown parameter 'colour'")


@pytest.mark.req("DES-030")
@pytest.mark.req("DES-038")
def test_open_override_checked_against_parameter_dimension() -> None:
    data = gantry()
    component(data, "x_belt")["overrides"] = {"max_tension": "${axis_friction}"}
    accepted(data)
    component(data, "x_belt")["overrides"] = {"max_tension": "${axis_backlash}"}
    rejected(data, "open parameter 'axis_backlash' has dimension [length]")
    component(data, "x_belt")["overrides"] = {"max_tension": "${tension_unset}"}
    rejected(data, "undeclared open parameter ${tension_unset}")
    component(data, "x_belt")["overrides"] = {"self_locking": "${axis_friction}"}
    rejected(data, "flag parameter 'self_locking' cannot be an open value")
    component(data, "x_belt")["overrides"] = {"colour": "${axis_friction}"}
    rejected(data, "has no parameter 'colour'")


@pytest.mark.req("DES-004")
@pytest.mark.req("DES-016")
@pytest.mark.parametrize(
    ("binding", "message"),
    [
        ({"mount": "flange", "body": "frame"}, "type 'stepper_behavioral' has no mount 'flange'"),
        ({"mount": "shaft", "body": "frame"}, "shaft mount 'shaft' must bind a joint"),
        ({"mount": "face", "body": "ghost"}, "unknown body 'ghost'"),
        ({"mount": "shaft", "joint": "ghost"}, "unknown joint 'ghost'"),
        ({"mount": "face", "body": "frame", "joint": "x_axis"}, "binds exactly one of body or joint"),
        ({"mount": "face"}, "binds exactly one of body or joint"),
    ],
)
def test_bad_mechanical_binding_rejected(binding: dict[str, str], message: str) -> None:
    data = gantry()
    component(data, "x_motor")["mechanical"] = [binding]
    rejected(data, message)


@pytest.mark.req("DES-004")
def test_mount_bound_twice_rejected() -> None:
    data = gantry()
    bindings = component(data, "x_motor")["mechanical"]
    bindings.append(copy.deepcopy(bindings[0]))
    rejected(data, "mount 'face' is bound more than once")


@pytest.mark.req("DES-004")
def test_gantry_binds_motor_shafts_to_joints_and_housings_to_bodies() -> None:
    design = accepted(gantry())
    motor = next(item for item in design.components if item.name == "x_motor")
    assert [(item.mount, item.body, item.joint) for item in motor.mechanical] == [
        ("face", "frame", None),
        ("shaft", None, "x_motor_spin"),
    ]


@pytest.mark.req("DES-016")
def test_duplicate_component_name_rejected() -> None:
    data = gantry()
    component(data, "y_motor")["name"] = "x_motor"
    rejected(data, "duplicate body, joint or component name 'x_motor'")


# ===== Signals and pin bindings ========================================


@pytest.mark.req("DES-009")
@pytest.mark.parametrize("name", ["x_driver_step", "x_driver.step.extra", "X_driver.step", "x_driver.", ".step"])
def test_undotted_signal_name_rejected(name: str) -> None:
    data = gantry()
    component(data, "x_driver")["signals"][0]["name"] = name
    rejected(data, "signals")


@pytest.mark.req("DES-009")
def test_signal_must_carry_its_instance_name() -> None:
    data = gantry()
    component(data, "x_driver")["signals"][0]["name"] = "x_motor.step"
    data["pin_bindings"][0]["signal"] = "x_motor.step"
    rejected(data, "must start with its instance name 'x_driver'")


@pytest.mark.req("DES-010")
def test_duplicate_signal_rejected_naming_both_declarations() -> None:
    data = gantry()
    component(data, "x_driver")["signals"][1]["name"] = "x_driver.step"
    rejected(data, "duplicate signal 'x_driver.step', first declared at components[1].signals[0]")


@pytest.mark.req("DES-016")
@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"pin": "coil_z"}, "has no pin 'coil_z'"),
        ({"pin": "dir"}, "pin 'dir' carries more than one signal"),
        ({"direction": "emits"}, "pin 'step' is input and cannot emits"),
        ({"direction": "listens"}, "direction"),
    ],
)
def test_bad_signal_declaration_rejected(change: dict[str, str], message: str) -> None:
    data = gantry()
    component(data, "x_driver")["signals"][0].update(change)
    rejected(data, message)


@pytest.mark.req("DES-005")
def test_signal_bound_twice_rejected() -> None:
    data = gantry()
    data["pin_bindings"].append({"signal": "x_driver.step", "pin": "PD0", "mode": "gpio_output"})
    rejected(data, "signal 'x_driver.step' is bound more than once, first at pin_bindings[0]")


@pytest.mark.req("DES-005")
def test_unbound_signal_rejected() -> None:
    data = gantry()
    del data["pin_bindings"][0]
    rejected(data, "signal 'x_driver.step' has no pin binding")


@pytest.mark.req("DES-005")
@pytest.mark.req("DES-016")
def test_binding_of_unknown_signal_rejected() -> None:
    data = gantry()
    data["pin_bindings"].append({"signal": "ghost.out", "pin": "PD0", "mode": "gpio_input"})
    rejected(data, "unknown signal 'ghost.out'")


@pytest.mark.req("DES-005")
def test_duplicate_mcu_pin_rejected() -> None:
    data = gantry()
    data["pin_bindings"][1]["pin"] = "PA0"
    rejected(data, "MCU pin 'PA0' is bound more than once, first at pin_bindings[0]")


@pytest.mark.req("DES-005")
@pytest.mark.parametrize(
    ("index", "mode"), [(0, "gpio_input"), (0, "timer_encoder"), (9, "gpio_output"), (9, "timer_pwm")]
)
def test_peripheral_mode_must_suit_signal_direction(index: int, mode: str) -> None:
    data = gantry()
    data["pin_bindings"][index]["mode"] = mode
    rejected(data, f"mode '{mode}' does not suit signal")


@pytest.mark.req("DES-005")
@pytest.mark.parametrize(("field", "value"), [("pin", "pa0"), ("pin", ""), ("mode", "bitbang"), ("signal", "x")])
def test_bad_pin_binding_fields_rejected(field: str, value: str) -> None:
    data = gantry()
    data["pin_bindings"][0][field] = value
    rejected(data, field)


@pytest.mark.req("DES-005")
@pytest.mark.parametrize("field", ["signal", "pin", "mode"])
def test_pin_binding_field_missing(field: str) -> None:
    data = gantry()
    del data["pin_bindings"][0][field]
    rejected(data, field)


@pytest.mark.req("DES-005")
@pytest.mark.req("DES-040")
def test_signals_without_firmware_rejected() -> None:
    data = gantry()
    del data["firmware"]
    rejected(data, "a design with controller-facing signals must declare its firmware and chip")


@pytest.mark.req("DES-005")
def test_every_gantry_signal_bound_exactly_once() -> None:
    design = accepted(gantry())
    bound = sorted(binding.signal for binding in design.pin_bindings)
    assert bound == sorted(design.signal_names())
    assert len(bound) == 22


# ===== Power ===========================================================


@pytest.mark.req("DES-006")
@pytest.mark.req("DES-016")
@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"supply": "ghost"}, "power.rails[0].supply: unknown component instance 'ghost'"),
        ({"supply": "estop"}, "instance 'estop' is not a power component"),
        ({"gated_by": ["ghost"]}, "power.rails[0].gated_by[0]: unknown component instance 'ghost'"),
        ({"gated_by": ["motor_supply"]}, "instance 'motor_supply' is not a safety component"),
        ({"name": "logic"}, "duplicate rail 'logic'"),
    ],
)
def test_bad_rail_rejected(change: dict[str, Any], message: str) -> None:
    data = gantry()
    data["power"]["rails"][0].update(change)
    rejected(data, message)


@pytest.mark.req("DES-006")
def test_power_structure_rules() -> None:
    data = gantry()
    data["power"]["controller_rail"] = "ghost"
    rejected(data, "power.controller_rail: unknown rail 'ghost'")
    data = gantry()
    data["power"]["rails"] = []
    rejected(data, "rails")
    data = gantry()
    del data["power"]["controller_rail"]
    rejected(data, "controller_rail")
    design = accepted(gantry())
    assert design.power is not None
    assert design.power.rails[0].gated_by == ["estop", "watchdog"]


# ===== Environment, firmware, structural ===============================


@pytest.mark.req("DES-007")
@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("gravity", ["0 m/s**2", "-9.81 m/s**2"]),
        ("gravity", ["0 m", "0 m", "-9.81 m"]),
        ("ambient_temperature", "0 K"),
        ("ambient_temperature", "-300 degC"),
        ("ambient_temperature", "20 m"),
    ],
)
def test_bad_environment_rejected(field: str, value: Any) -> None:
    data = pendulum()
    data["environment"][field] = value
    rejected(data, field)


@pytest.mark.req("DES-007")
@pytest.mark.parametrize("field", ["gravity", "ambient_temperature"])
def test_environment_field_missing(field: str) -> None:
    data = pendulum()
    del data["environment"][field]
    rejected(data, field)


@pytest.mark.req("DES-040")
def test_gantry_firmware_is_data() -> None:
    design = accepted(gantry())
    assert design.firmware is not None
    assert design.firmware.path == "firmware"
    assert design.firmware.chip == "stm32_placeholder"


@pytest.mark.req("DES-040")
@pytest.mark.parametrize(
    ("field", "value"),
    [("tick_rate", "0 Hz"), ("tick_rate", "1 ms"), ("path", "/firmware"), ("path", "../firmware"), ("chip", "STM32")],
)
def test_bad_firmware_rejected(field: str, value: str) -> None:
    data = gantry()
    data["firmware"][field] = value
    rejected(data, field)


@pytest.mark.req("DES-040")
@pytest.mark.parametrize("field", ["path", "chip", "tick_rate"])
def test_firmware_field_missing(field: str) -> None:
    data = gantry()
    del data["firmware"][field]
    rejected(data, field)


@pytest.mark.req("DES-031")
@pytest.mark.req("DES-032")
@pytest.mark.parametrize(
    ("path", "value"),
    [
        (("design_load", "allowance"), "-1 percent"),
        (("design_load", "rated_load"), "0 kg"),
        (("accuracy_target",), "0 cm"),
        (("accuracy_target",), "1 kg"),
        (("deflection_target",), "quasi"),
        (("minimum_safety_factor",), "0"),
    ],
)
def test_bad_structural_values_rejected(path: tuple[str, ...], value: str) -> None:
    data = gantry()
    target = data["structural"]
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    rejected(data, path[-1])


@pytest.mark.req("DES-031")
@pytest.mark.req("DES-032")
def test_structural_values_accepted_when_set() -> None:
    data = gantry()
    data["structural"]["design_load"]["rated_load"] = "3 kg"
    data["structural"]["design_load"]["allowance"] = "0 percent"
    data["structural"]["deflection_target"] = "static"
    data["structural"]["minimum_safety_factor"] = "2"
    structural = accepted(data).structural
    assert structural is not None
    assert structural.design_load.rated_load == pytest.approx(3.0)
    assert structural.deflection_target == "static"
    assert structural.minimum_safety_factor == pytest.approx(2.0)


@pytest.mark.req("DES-031")
@pytest.mark.req("DES-032")
@pytest.mark.parametrize("field", ["design_load", "accuracy_target", "deflection_target"])
def test_structural_field_missing(field: str) -> None:
    data = gantry()
    del data["structural"][field]
    rejected(data, field)


# ===== Wiring and rail consumers =======================================


def wire(source: str, destination: str) -> dict[str, str]:
    return {"source": source, "destination": destination}


def bus_registry(directory: Path) -> Registry:
    """The shipped catalog plus a test-only type whose `bus` port is bidirectional."""
    for path in CATALOG_DIR.glob("*.yaml"):
        shutil.copy(path, directory / path.name)
    text = (CATALOG_DIR / "watchdog.yaml").read_text(encoding="utf-8").replace("watchdog", "bus_node")
    text = text.replace(
        "- name: tripped\n          direction: output", "- name: bus\n          direction: bidirectional"
    )
    (directory / "bus_node.yaml").write_text(text, encoding="utf-8")
    return load_registry(directory)


@pytest.mark.req("DES-041")
def test_gantry_wiring_connects_supply_safety_drivers_and_motors() -> None:
    design = accepted(gantry())
    pairs = [(item.source, item.destination) for item in design.wiring]
    assert len(pairs) == 15
    assert ("motor_supply.v_out", "estop.contact_in") in pairs
    assert ("estop.contact_out", "watchdog.contact_in") in pairs
    for axis in ("x", "y", "lift"):
        assert ("watchdog.contact_out", f"{axis}_driver.vmot") in pairs
        assert (f"{axis}_driver.coil_a", f"{axis}_motor.coil_a") in pairs
        assert (f"{axis}_driver.coil_b", f"{axis}_motor.coil_b") in pairs
    assert design.power is not None
    assert design.power.rails[0].consumers == ["x_driver", "y_driver", "lift_driver"]


@pytest.mark.req("DES-041")
@pytest.mark.req("LIB-019")
def test_gantry_heartbeat_is_a_digital_output_pin_bound_to_the_watchdog() -> None:
    """Covers LIB-019's watchdog half only: the load cell, amplifier and IMU types remain owed (LIB-019 stub)."""
    design = accepted(gantry())
    bindings = {binding.signal: binding for binding in design.pin_bindings}
    assert bindings["watchdog.heartbeat"].mode == "gpio_output"
    assert bindings["watchdog.heartbeat"].pin == "PB6"
    assert "heartbeat_wiring" not in [parameter.name for parameter in design.open_parameters]


@pytest.mark.req("DES-041")
def test_pendulum_without_wiring_chip_or_contact_stays_legal() -> None:
    design = accepted(pendulum())
    assert design.wiring == []
    assert design.firmware is None
    assert design.environment.contact is None


@pytest.mark.req("DES-041")
def test_empty_wiring_and_consumers_accepted() -> None:
    data = gantry()
    data["wiring"] = []
    for rail in data["power"]["rails"]:
        rail["consumers"] = []
    design = accepted(data)
    assert design.wiring == []
    del data["wiring"]
    accepted(data)


@pytest.mark.req("DES-041")
@pytest.mark.parametrize("end", ["source", "destination"])
def test_wire_to_unknown_instance_rejected(end: str) -> None:
    data = gantry()
    data["wiring"][0][end] = "ghost.v_out"
    rejected(data, f"wiring[0].{end}: unknown component instance 'ghost'")


@pytest.mark.req("DES-041")
@pytest.mark.parametrize("end", ["source", "destination"])
def test_wire_to_unknown_port_rejected(end: str) -> None:
    data = gantry()
    data["wiring"][0][end] = "estop.ghost"
    rejected(data, f"wiring[0].{end}: type 'estop_relay' has no port 'ghost'")


@pytest.mark.req("DES-041")
@pytest.mark.parametrize(
    ("source", "destination", "message"),
    [
        ("x_driver.vmot", "estop.contact_in", "cannot wire power_in port 'x_driver.vmot' to power_in"),
        ("x_driver.coil_a", "y_driver.coil_a", "cannot wire output port 'x_driver.coil_a' to output"),
        ("motor_supply.v_out", "estop.reset", "cannot wire power_out port 'motor_supply.v_out' to input"),
        ("motor_supply.gnd", "estop.contact_in", "cannot wire ground port 'motor_supply.gnd' to power_in"),
        ("estop.status", "x_driver.vmot", "cannot wire output port 'estop.status' to power_in"),
        ("estop.reset", "watchdog.heartbeat", "cannot wire input port 'estop.reset' to input"),
        ("motor_supply.v_out", "watchdog.gnd", "cannot wire power_out port 'motor_supply.v_out' to ground"),
    ],
)
def test_wire_direction_clash_rejected(source: str, destination: str, message: str) -> None:
    data = gantry()
    data["wiring"].append(wire(source, destination))
    rejected(data, message)


@pytest.mark.req("DES-041")
def test_signal_port_wired_twice_rejected() -> None:
    data = gantry()
    data["wiring"].append(wire("x_driver.coil_a", "y_motor.coil_b"))
    del data["wiring"][12]
    rejected(data, "signal port 'x_driver.coil_a' is connected more than once (wiring[9], wiring[14])")


@pytest.mark.req("DES-041")
def test_signal_port_driven_by_controller_and_wire_rejected() -> None:
    data = gantry()
    data["wiring"].append(wire("estop.status", "x_driver.step"))
    rejected(data, "signal port 'x_driver.step' is connected more than once (signal 'x_driver.step', wiring[15])")


@pytest.mark.req("DES-041")
def test_power_ports_may_be_shared() -> None:
    data = gantry()
    data["wiring"].append(wire("motor_supply.v_out", "watchdog.contact_in"))
    data["wiring"].append(wire("motor_supply.gnd", "x_encoder.gnd"))
    accepted(data)


@pytest.mark.req("DES-041")
def test_duplicate_wire_rejected() -> None:
    data = gantry()
    data["wiring"].append(wire("motor_supply.v_out", "estop.contact_in"))
    rejected(data, "wiring[15]: duplicate wire 'motor_supply.v_out' to 'estop.contact_in'")


@pytest.mark.req("DES-041")
@pytest.mark.parametrize(
    ("source", "destination"),
    [("output", "bidirectional"), ("bidirectional", "input"), ("bidirectional", "bidirectional")],
)
def test_bidirectional_ports_wire_to_inputs_and_each_other(tmp_path: Path, source: str, destination: str) -> None:
    registry = bus_registry(tmp_path)
    data = gantry()
    data["components"].append({"name": "node_a", "type": "bus_node", "part": "bus_node_24v_generic"})
    data["components"].append({"name": "node_b", "type": "bus_node", "part": "bus_node_24v_generic"})
    ends = {"output": "estop.status", "input": "node_b.heartbeat", "bidirectional": "node_a.bus"}
    target = ends[destination]
    if source == destination:
        target = "node_b.bus"
    data["wiring"].append(wire(ends[source], target))
    Design.model_validate(data, context={"registry": registry})


@pytest.mark.req("DES-041")
def test_bidirectional_port_cannot_drive_an_output(tmp_path: Path) -> None:
    registry = bus_registry(tmp_path)
    data = gantry()
    data["components"].append({"name": "node_a", "type": "bus_node", "part": "bus_node_24v_generic"})
    data["wiring"].append(wire("node_a.bus", "estop.status"))
    with pytest.raises(ValidationError, match=re.escape("cannot wire bidirectional port 'node_a.bus' to output")):
        Design.model_validate(data, context={"registry": registry})


@pytest.mark.req("DES-041")
@pytest.mark.parametrize(
    ("change", "fragment"),
    [
        ({"source": "x_driver"}, "source"),
        ({"destination": "X_motor.coil_a"}, "destination"),
        ({"source": "x_driver.coil_a.extra"}, "source"),
        ({"source": 1}, "source"),
        ({"gauge": "1 mm"}, "gauge"),
    ],
)
def test_bad_wire_fields_rejected(change: dict[str, Any], fragment: str) -> None:
    data = gantry()
    data["wiring"][0].update(change)
    rejected(data, fragment)


@pytest.mark.req("DES-041")
@pytest.mark.parametrize("field", ["source", "destination"])
def test_wire_field_missing(field: str) -> None:
    data = gantry()
    del data["wiring"][0][field]
    rejected(data, field)


@pytest.mark.req("DES-041")
@pytest.mark.parametrize(
    ("consumers", "message"),
    [
        (["ghost"], "power.rails[0].consumers[0]: unknown component instance 'ghost'"),
        (["x_driver", "x_driver"], "power.rails[0].consumers: consumer 'x_driver'"),
        (["X_driver"], "consumers"),
        ("x_driver", "consumers"),
        (["${logic_supply}"], "consumers"),
    ],
)
def test_bad_rail_consumers_rejected(consumers: Any, message: str) -> None:
    data = gantry()
    data["power"]["rails"][0]["consumers"] = consumers
    rejected(data, message)


# ===== Contact environment =============================================


def concrete_contact() -> dict[str, Any]:
    return {
        "default": {"stiffness": "1e5 N/m", "damping": "50 N*s/m", "friction": "0.5"},
        "pairs": [
            {
                "materials": ["aluminium_6063_t5_extrusion", "pla_printed"],
                "stiffness": "2e5 N/m",
                "damping": "10 N*s/m",
                "friction": "0.3",
            }
        ],
        "exclusions": [["link", "world"]],
        "floor": {"normal": [0.0, 1.0, 0.0], "offset": "-50 cm"},
    }


def with_contact(contact: dict[str, Any]) -> dict[str, Any]:
    data = pendulum()
    data["environment"]["contact"] = contact
    return data


@pytest.mark.req("DES-042")
def test_contact_values_held_as_si_and_mirror_the_neutral_model() -> None:
    contact = accepted(with_contact(concrete_contact())).environment.contact
    assert contact is not None
    assert contact.default.stiffness == pytest.approx(1e5)
    assert contact.default.damping == pytest.approx(50.0)
    assert contact.default.friction == pytest.approx(0.5)
    assert contact.pairs[0].materials == ["aluminium_6063_t5_extrusion", "pla_printed"]
    assert contact.pairs[0].friction == pytest.approx(0.3)
    assert contact.exclusions == [["link", "world"]]
    assert contact.floor is not None
    assert contact.floor.body == "world"
    assert contact.floor.offset == pytest.approx(-0.5)
    default_fields = set(type(contact.default).model_fields)
    assert default_fields == set(NeutralContactParameters.model_fields)
    assert set(NeutralContactModel.model_fields) <= set(type(contact).model_fields)


@pytest.mark.req("DES-042")
def test_gantry_contact_defaults_are_open_and_floor_is_the_world_plane() -> None:
    contact = accepted(gantry()).environment.contact
    assert contact is not None
    assert contact.default.stiffness == OpenValue("contact_stiffness", "[mass] / [time] ** 2")
    assert contact.floor is not None
    assert contact.floor.normal == [0.0, 0.0, 1.0]
    assert contact.floor.offset == pytest.approx(0.0)


@pytest.mark.req("DES-042")
def test_contact_minimal_accepted() -> None:
    contact = accepted(with_contact({"default": concrete_contact()["default"]})).environment.contact
    assert contact is not None
    assert contact.pairs == []
    assert contact.exclusions == []
    assert contact.floor is None
    accepted(with_contact({**concrete_contact(), "pairs": [], "exclusions": []}))


@pytest.mark.req("DES-042")
def test_contact_defaults_required() -> None:
    contact = concrete_contact()
    del contact["default"]
    rejected(with_contact(contact), "default")


@pytest.mark.req("DES-042")
@pytest.mark.parametrize("field", ["stiffness", "damping", "friction"])
def test_contact_default_field_missing(field: str) -> None:
    contact = concrete_contact()
    del contact["default"][field]
    rejected(with_contact(contact), field)


@pytest.mark.req("DES-042")
@pytest.mark.parametrize("field", ["materials", "stiffness", "damping", "friction"])
def test_contact_pair_field_missing(field: str) -> None:
    contact = concrete_contact()
    del contact["pairs"][0][field]
    rejected(with_contact(contact), field)


@pytest.mark.req("DES-042")
@pytest.mark.parametrize(
    ("field", "value", "ok"),
    [
        ("stiffness", "0 N/m", False),
        ("stiffness", "-1 N/m", False),
        ("stiffness", "1e-9 N/m", True),
        ("damping", "-1 N*s/m", False),
        ("damping", "0 N*s/m", True),
        ("damping", "1e-9 N*s/m", True),
        ("friction", "-0.1", False),
        ("friction", "0", True),
        ("friction", "1.5", True),
    ],
)
def test_contact_parameter_bounds(field: str, value: str, ok: bool) -> None:
    for where in ("default", "pair"):
        contact = concrete_contact()
        target = contact["default"]
        if where == "pair":
            target = contact["pairs"][0]
        target[field] = value
        if ok:
            accepted(with_contact(contact))
        else:
            rejected(with_contact(contact), field)


@pytest.mark.req("DES-042")
@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("stiffness", "1 N"),
        ("stiffness", 1e5),
        ("stiffness", "1e5"),
        ("damping", "1 N/m"),
        ("damping", 50),
        ("friction", "1 m"),
        ("friction", 0.5),
        ("friction", True),
    ],
)
def test_contact_parameter_units_rejected(field: str, value: Any) -> None:
    contact = concrete_contact()
    contact["default"][field] = value
    rejected(with_contact(contact), field)


@pytest.mark.req("DES-042")
def test_contact_pair_override_unknown_material_rejected() -> None:
    contact = concrete_contact()
    contact["pairs"][0]["materials"][1] = "unobtainium"
    rejected(with_contact(contact), "environment.contact.pairs[0].materials: unknown material 'unobtainium'")


@pytest.mark.req("DES-042")
def test_contact_pair_listed_twice_rejected_in_either_order() -> None:
    contact = concrete_contact()
    pair = copy.deepcopy(contact["pairs"][0])
    pair["materials"] = ["pla_printed", "aluminium_6063_t5_extrusion"]
    contact["pairs"].append(pair)
    rejected(with_contact(contact), "material pair aluminium_6063_t5_extrusion/pla_printed listed twice")


@pytest.mark.req("DES-042")
def test_contact_pair_of_one_material_with_itself_accepted() -> None:
    contact = concrete_contact()
    contact["pairs"][0]["materials"] = ["pla_printed", "pla_printed"]
    accepted(with_contact(contact))


@pytest.mark.req("DES-042")
@pytest.mark.parametrize("materials", [[], ["pla_printed"], ["pla_printed", "abs_printed", "petg_printed"], "pla"])
def test_contact_pair_must_name_two_materials(materials: Any) -> None:
    contact = concrete_contact()
    contact["pairs"][0]["materials"] = materials
    rejected(with_contact(contact), "materials")


@pytest.mark.req("DES-042")
def test_contact_exclusion_unknown_body_rejected() -> None:
    contact = concrete_contact()
    contact["exclusions"][0][1] = "ghost"
    rejected(with_contact(contact), "environment.contact.exclusions[0]: exclusion names unknown body 'ghost'")


@pytest.mark.req("DES-042")
def test_contact_exclusion_of_a_body_with_itself_rejected() -> None:
    contact = concrete_contact()
    contact["exclusions"][0] = ["link", "link"]
    rejected(with_contact(contact), "exclusion pairs body 'link' with itself")


@pytest.mark.req("DES-042")
def test_contact_exclusion_listed_twice_rejected_in_either_order() -> None:
    contact = concrete_contact()
    contact["exclusions"].append(["world", "link"])
    rejected(with_contact(contact), "exclusions[1]: exclusion link/world listed twice")


@pytest.mark.req("DES-042")
@pytest.mark.parametrize("exclusion", [[], ["link"], ["link", "world", "link"], "link"])
def test_contact_exclusion_must_name_two_bodies(exclusion: Any) -> None:
    contact = concrete_contact()
    contact["exclusions"][0] = exclusion
    rejected(with_contact(contact), "exclusions")


@pytest.mark.req("DES-042")
def test_floor_on_world_only() -> None:
    contact = concrete_contact()
    contact["floor"]["body"] = "link"
    rejected(with_contact(contact), "the floor plane is allowed only on the world, not on 'link'")
    contact["floor"]["body"] = "world"
    accepted(with_contact(contact))


@pytest.mark.req("DES-042")
@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("normal", [0.0, 0.0, 2.0]),
        ("normal", [0.0, 0.0, 0.0]),
        ("normal", [0.0, 1.0]),
        ("normal", [0.0, 0.0, 1.0, 0.0]),
        ("normal", "up"),
        ("offset", 0.0),
        ("offset", "0"),
        ("offset", "0 kg"),
    ],
)
def test_bad_floor_rejected(field: str, value: Any) -> None:
    contact = concrete_contact()
    contact["floor"][field] = value
    rejected(with_contact(contact), field)


@pytest.mark.req("DES-042")
@pytest.mark.parametrize("field", ["normal", "offset"])
def test_floor_field_missing(field: str) -> None:
    contact = concrete_contact()
    del contact["floor"][field]
    rejected(with_contact(contact), field)


@pytest.mark.req("DES-042")
@pytest.mark.parametrize("where", ["contact", "default", "pair", "floor"])
def test_contact_unknown_key_rejected(where: str) -> None:
    contact = concrete_contact()
    targets = {
        "contact": contact,
        "default": contact["default"],
        "pair": contact["pairs"][0],
        "floor": contact["floor"],
    }
    targets[where]["restitution"] = "0.5"
    rejected(with_contact(contact), "restitution")


# ===== Chip pin map ====================================================


@pytest.mark.req("DES-029")
def test_every_gantry_binding_uses_a_pin_and_mode_of_its_chip() -> None:
    design = accepted(gantry())
    modes = shipped_chips()["stm32_placeholder"].pin_modes()
    for binding in design.pin_bindings:
        assert binding.mode in modes[binding.pin]


@pytest.mark.req("DES-029")
def test_binding_to_pin_missing_from_chip_rejected() -> None:
    data = gantry()
    data["pin_bindings"][0]["pin"] = "PD0"
    rejected(data, "pin_bindings[0].pin: chip 'stm32_placeholder' has no pin 'PD0'")


@pytest.mark.req("DES-029")
@pytest.mark.parametrize(("index", "mode"), [(0, "timer_pwm"), (0, "dac"), (9, "adc"), (15, "adc")])
def test_binding_in_mode_the_chip_pin_lacks_rejected(index: int, mode: str) -> None:
    data = gantry()
    data["pin_bindings"][index]["mode"] = mode
    pin = data["pin_bindings"][index]["pin"]
    rejected(data, f"pin_bindings[{index}].mode: chip 'stm32_placeholder' pin '{pin}' does not support mode '{mode}'")


@pytest.mark.req("DES-029")
def test_unknown_chip_rejected() -> None:
    data = gantry()
    data["firmware"]["chip"] = "ghost_chip"
    rejected(data, "firmware.chip: unknown chip 'ghost_chip'")


@pytest.mark.req("DES-029")
def test_open_chip_skips_the_pin_map_check() -> None:
    data = gantry()
    data["firmware"]["chip"] = "${stm32_part}"
    data["pin_bindings"][0]["pin"] = "PD0"
    accepted(data)


@pytest.mark.req("DES-029")
def test_chips_from_context_replace_the_shipped_set() -> None:
    chip = Chip.model_validate(
        {
            "name": "stm32_placeholder",
            "family": "STM32",
            "status": "placeholder",
            "platform_description": "placeholder",
            "svd": "placeholder",
            "pins": [{"name": "PA0", "modes": ["gpio_output"]}],
        }
    )
    with pytest.raises(ValidationError, match="chip 'stm32_placeholder' has no pin 'PB6'"):
        Design.model_validate(gantry(), context={"chips": {"stm32_placeholder": chip}})
    with pytest.raises(ValidationError, match="unknown chip 'stm32_placeholder'"):
        Design.model_validate(gantry(), context={"chips": {}})


# ===== Operator messages ===============================================


@pytest.mark.req("SIG-027")
def test_gantry_operator_messages_use_requirements_labels_verbatim() -> None:
    design = accepted(gantry())
    labels = [message.label for message in design.operator_messages]
    assert labels == ["target (x, y)", "home", "current position", "ready / fault"]


@pytest.mark.req("SIG-027")
@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"source": "controller", "destination": "controller"}, "exactly one of source and destination"),
        ({"source": "operator", "destination": "operator"}, "exactly one of source and destination"),
        ({"source": "ghost"}, "operator_messages[0].source: unknown endpoint 'ghost'"),
        ({"label": "home"}, "duplicate operator message label 'home'"),
        ({"name": "home"}, "duplicate operator message 'home'"),
        ({"label": ""}, "label"),
    ],
)
def test_bad_operator_message_rejected(change: dict[str, str], message: str) -> None:
    data = gantry()
    data["operator_messages"][0].update(change)
    rejected(data, message)


@pytest.mark.req("SIG-027")
def test_heartbeat_is_not_an_operator_message() -> None:
    data = gantry()
    data["operator_messages"].append(
        {"name": "heartbeat", "label": "heartbeat", "source": "controller", "destination": "estop"}
    )
    rejected(data, "message 'heartbeat' is reserved: the operator link carries operator traffic only")


@pytest.mark.req("DES-033")
def test_gantry_rail_supports_are_the_planning_default() -> None:
    design = accepted(gantry())
    supports = [body.flexible.supports for body in design.bodies if body.flexible is not None]
    assert supports == [["clamped", "clamped"], ["pinned", "roller"]]


@pytest.mark.req("DES-033")
@pytest.mark.parametrize(
    "supports", [["clamped", "pinned"], ["pinned", "roller"], ["free", "free"], ["roller", "clamped"]]
)
def test_flexible_supports_accepted(supports: list[str]) -> None:
    data = gantry()
    data["bodies"][1]["flexible"]["supports"] = supports
    flexible = accepted(data).bodies[1].flexible
    assert flexible is not None
    assert flexible.supports == supports


@pytest.mark.req("DES-033")
@pytest.mark.parametrize(
    "supports", [["clamped"], ["clamped", "clamped", "free"], [], ["welded", "free"], ["clamped", ""], "clamped"]
)
def test_bad_flexible_supports_rejected(supports: Any) -> None:
    data = gantry()
    data["bodies"][1]["flexible"]["supports"] = supports
    rejected(data, "supports")


@pytest.mark.req("DES-033")
def test_flexible_supports_required() -> None:
    data = gantry()
    del data["bodies"][1]["flexible"]["supports"]
    rejected(data, "supports")


@pytest.mark.req("SIG-027")
def test_message_field_dimension_must_be_known() -> None:
    data = gantry()
    data["operator_messages"][0]["fields"][0]["dimension"] = "[wobble]"
    rejected(data, "unknown dimension")


# ===== Errors ==========================================================


@pytest.mark.req("DES-015")
@pytest.mark.req("DES-036")
def test_validation_returns_every_error_with_file_and_path(tmp_path: Path) -> None:
    data = gantry()
    data["bodies"][0]["material"] = "unobtainium"
    joint(data, "x_axis")["parent"] = "ghost"
    del data["pin_bindings"][0]
    path = tmp_path / "design.yaml"
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    path.write_text(stream.getvalue(), encoding="utf-8")
    errors = validate_design_file(path)
    assert f"{path}: bodies[0].material: unknown material 'unobtainium'" in errors
    assert f"{path}: joints[2].parent: unknown body 'ghost'" in errors
    assert f"{path}: components[1].signals[0]: signal 'x_driver.step' has no pin binding" in errors
    assert all(error.startswith(f"{path}: ") for error in errors)


@pytest.mark.req("DES-015")
@pytest.mark.req("DES-036")
def test_field_errors_name_their_path(tmp_path: Path) -> None:
    data = pendulum()
    link(data)["mass_properties"]["mass"] = "1 m"
    path = tmp_path / "design.yaml"
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    path.write_text(stream.getvalue(), encoding="utf-8")
    errors = validate_design_file(path)
    assert len(errors) == 1
    assert errors[0].startswith(f"{path}: bodies.0.mass_properties.")
    assert "expected [mass]" in errors[0]


@pytest.mark.req("DES-015")
def test_yaml_parse_error_names_file(tmp_path: Path) -> None:
    path = tmp_path / "design.yaml"
    path.write_text("name: [unclosed\n", encoding="utf-8")
    errors = validate_design_file(path)
    assert len(errors) == 1
    assert errors[0].startswith(f"{path}: YAML parse error")


@pytest.mark.req("DES-015")
def test_load_design_raises_with_every_error(tmp_path: Path) -> None:
    data = pendulum()
    link(data)["material"] = "unobtainium"
    path = tmp_path / "design.yaml"
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    path.write_text(stream.getvalue(), encoding="utf-8")
    with pytest.raises(ValueError, match="unknown material 'unobtainium'"):
        load_design(path)


# ===== CAD mass cross-check ============================================


def properties(mass: float, com: float, ixx: float) -> cad.MassPropertiesSI:
    return cad.MassPropertiesSI(mass=mass, center_of_mass=(com, 0.0, 0.0), inertia=(ixx, 1.0, 1.0, 0.0, 0.0, 0.0))


@pytest.mark.req("DES-022")
@pytest.mark.parametrize(
    ("recomputed", "ok"), [(2.0, True), (2.25, True), (2.5, True), (2.5001, False), (1.4999, False)]
)
def test_mass_cross_check_at_inside_and_outside_tolerance(recomputed: float, ok: bool) -> None:
    report = cad.cross_check_mass("link", properties(2.0, 0.0, 1.0), properties(recomputed, 0.0, 1.0), 0.25, 1.0)
    assert report.within_tolerance == ok
    assert report.mass_error == pytest.approx(abs(recomputed - 2.0) / 2.0)
    expected: list[str] = []
    if not ok:
        expected = ["mass"]
    assert [item.quantity for item in report.mismatches] == expected


@pytest.mark.req("DES-022")
@pytest.mark.parametrize(("offset", "ok"), [(0.0, True), (0.125, True), (0.25, True), (0.2501, False)])
def test_center_of_mass_cross_check_relative_to_body_size(offset: float, ok: bool) -> None:
    report = cad.cross_check_mass("link", properties(1.0, 0.0, 1.0), properties(1.0, offset, 1.0), 0.5, 0.5)
    assert report.within_tolerance == ok
    assert report.center_of_mass_error == pytest.approx(offset / 0.5)


@pytest.mark.req("DES-022")
@pytest.mark.parametrize(("ixx", "ok"), [(1.0, True), (1.25, True), (1.6, False)])
def test_inertia_cross_check_by_tensor_norm(ixx: float, ok: bool) -> None:
    declared = properties(1.0, 0.0, 1.0)
    report = cad.cross_check_mass("link", declared, properties(1.0, 0.0, ixx), 0.25 / 3.0**0.5 + 1e-12, 1.0)
    assert report.inertia_error == pytest.approx(abs(ixx - 1.0) / 3.0**0.5)
    assert report.within_tolerance == ok


@pytest.mark.req("DES-022")
def test_cross_check_reports_every_mismatch() -> None:
    report = cad.cross_check_mass("link", properties(1.0, 0.0, 1.0), properties(2.0, 1.0, 3.0), 0.1, 1.0)
    assert [item.quantity for item in report.mismatches] == ["mass", "center_of_mass", "inertia"]
    assert report.mismatches[0].error == pytest.approx(1.0)
    assert report.mismatches[0].tolerance == pytest.approx(0.1)


@pytest.mark.req("DES-022")
@pytest.mark.parametrize(("tolerance", "size"), [(0.0, 1.0), (-0.1, 1.0), (float("nan"), 1.0), (0.1, 0.0), (0.1, -1.0)])
def test_cross_check_rejects_bad_tolerance_or_size(tolerance: float, size: float) -> None:
    with pytest.raises(ValueError, match="must be positive"):
        cad.cross_check_mass("link", properties(1.0, 0.0, 1.0), properties(1.0, 0.0, 1.0), tolerance, size)


@pytest.mark.req("DES-022")
@pytest.mark.parametrize(("mass", "ixx"), [(0.0, 1.0), (-1.0, 1.0), (1.0, 0.0), (float("inf"), 1.0)])
def test_mass_properties_result_bounds(mass: float, ixx: float) -> None:
    with pytest.raises(ValidationError):
        properties(mass, 0.0, ixx)


def test_importers_are_abstract() -> None:
    with pytest.raises(TypeError):
        cad.StepImporter()  # type: ignore[abstract]
    with pytest.raises(TypeError):
        cad.UrdfImporter()  # type: ignore[abstract]


def test_import_result_types_validate() -> None:
    mass = properties(1.0, 0.0, 1.0)
    hull = cad.ConvexHull(vertices=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)])
    result = cad.StepImportResult(
        body="link", volume=1e-4, mass_properties=mass, hulls=[hull], needs_mesh_contact=False
    )
    assert result.hulls[0].vertices[1] == (1.0, 0.0, 0.0)
    with pytest.raises(ValidationError):
        cad.ConvexHull(vertices=[(0.0, 0.0, 0.0)])
    with pytest.raises(ValidationError):
        cad.UrdfImportResult(bodies=[], joints=[])
    with pytest.raises(ValidationError):
        cad.UrdfJoint(name="j", type="hinge", parent="a", child="b")
