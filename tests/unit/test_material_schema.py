"""Material record validation (MAT)."""

from __future__ import annotations

import copy
import io
import math
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

from mechsim.materials import DATA_DIR, Material, MaterialError, load_material_file
from mechsim.materials.schema import yaml_loader

SHIPPED = sorted(DATA_DIR.glob("*.yaml"))
PROPERTIES = [
    "density",
    "youngs_modulus",
    "poissons_ratio",
    "yield_strength",
    "ultimate_strength",
    "shear_strength",
    "thermal_conductivity",
    "specific_heat",
    "thermal_expansion",
    "max_service_temperature",
]
REQUIRED = ["name", "family", "source", "placeholder", "wide_spread_default", "sn_curve", *PROPERTIES]


def plain(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): plain(item) for key, item in value.items()}
    if isinstance(value, list):
        return [plain(item) for item in value]
    return value


def example() -> dict[str, Any]:
    data: dict[str, Any] = plain(yaml_loader().load((DATA_DIR / "pla_printed.yaml").read_text(encoding="utf-8")))
    return copy.deepcopy(data)


def rejects(data: dict[str, Any], *fragments: str) -> None:
    with pytest.raises(ValidationError) as caught:
        Material.model_validate(data)
    for fragment in fragments:
        assert fragment in str(caught.value)


def write(tmp_path: Path, data: dict[str, Any]) -> Path:
    path = tmp_path / "material.yaml"
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    path.write_text(stream.getvalue(), encoding="utf-8")
    return path


# ===== Worked examples =================================================


@pytest.mark.req("MAT-018")
@pytest.mark.parametrize("path", SHIPPED, ids=[path.stem for path in SHIPPED])
def test_shipped_material_validates_and_round_trips(path: Path) -> None:
    load_material_file(path)
    text = path.read_text(encoding="utf-8")
    yaml = yaml_loader()
    stream = io.StringIO()
    yaml.dump(yaml.load(text), stream)
    assert stream.getvalue() == text


@pytest.mark.req("MAT-001")
@pytest.mark.req("MAT-019")
def test_mechanical_properties_in_si() -> None:
    material = Material.model_validate(example())
    assert material.density.typ == 1240.0
    assert material.youngs_modulus.typ == 2.8e9
    assert material.poissons_ratio.typ == 0.33
    assert material.yield_strength.min == 20e6
    assert material.ultimate_strength.max == 65e6
    assert material.shear_strength.typ == 25e6


@pytest.mark.req("MAT-002")
@pytest.mark.req("MAT-019")
def test_thermal_properties_in_si() -> None:
    material = Material.model_validate(example())
    assert material.thermal_conductivity.typ == 0.13
    assert material.specific_heat.typ == 1500.0
    assert math.isclose(material.thermal_expansion.typ, 70e-6)
    assert material.max_service_temperature.typ == 323.0


@pytest.mark.req("MAT-019")
def test_no_unit_string_survives_load() -> None:
    for path in SHIPPED:
        material = load_material_file(path)
        for name in PROPERTIES:
            spread = getattr(material, name)
            for value in (spread.min, spread.typ, spread.max):
                assert type(value) is float
        for point in material.sn_curve:
            assert type(point.stress_amplitude) is float


@pytest.mark.req("MAT-013")
def test_shipped_metal_families() -> None:
    names = {load_material_file(path).name: load_material_file(path) for path in SHIPPED}
    assert names["aluminium_6063_t5_extrusion"].family == "aluminium"
    assert names["steel_8_8_fastener"].family == "steel"
    assert names["bearing_steel_52100"].family == "steel"


@pytest.mark.req("MAT-014")
@pytest.mark.req("MAT-015")
@pytest.mark.parametrize("name", ["pla_printed", "petg_printed", "abs_printed"])
def test_shipped_printed_materials_are_wide_spread_defaults(name: str) -> None:
    material = load_material_file(DATA_DIR / f"{name}.yaml")
    assert material.process is not None
    assert material.process.kind == "printed"
    assert material.wide_spread_default is True
    assert material.placeholder is True
    assert material.yield_strength.max >= 2.0 * material.yield_strength.min


# ===== Spreads =========================================================


@pytest.mark.req("MAT-004")
@pytest.mark.parametrize("name", PROPERTIES)
@pytest.mark.parametrize("bound", ["min", "typ", "max"])
def test_spread_needs_min_typ_max(name: str, bound: str) -> None:
    data = example()
    del data[name][bound]
    rejects(data, name, bound, "Field required")


@pytest.mark.req("MAT-005")
@pytest.mark.parametrize("name", PROPERTIES)
def test_spread_equal_values_accepted(name: str) -> None:
    data = example()
    data[name]["min"] = data[name]["typ"]
    data[name]["max"] = data[name]["typ"]
    Material.model_validate(data)


@pytest.mark.req("MAT-005")
@pytest.mark.parametrize("name", PROPERTIES)
@pytest.mark.parametrize(("low", "high"), [("min", "typ"), ("typ", "max"), ("min", "max")])
def test_spread_out_of_order_names_file_material_property(tmp_path: Path, name: str, low: str, high: str) -> None:
    data = example()
    data[name][low], data[name][high] = data[name][high], data[name][low]
    path = write(tmp_path, data)
    with pytest.raises(MaterialError) as caught:
        load_material_file(path)
    message = str(caught.value)
    for expected in (str(path), "material 'pla_printed'", name, "min <= typ <= max"):
        assert expected in message


@pytest.mark.req("MAT-019")
@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("density", "1240"),
        ("density", "1240 kg"),
        ("youngs_modulus", "2.8 m"),
        ("yield_strength", 40.0),
        ("thermal_conductivity", "0.13 W"),
        ("specific_heat", "1500 J"),
        ("thermal_expansion", "70e-6 m"),
        ("max_service_temperature", "323 s"),
        ("poissons_ratio", "0.33 m"),
    ],
)
def test_missing_or_wrong_unit(name: str, value: Any) -> None:
    data = example()
    data[name]["typ"] = value
    rejects(data, name, "typ")


@pytest.mark.req("MAT-019")
@pytest.mark.parametrize("value", ["1.24 g/cm**3", "1240 kg/m**3", "0.0448 lb/inch**3"])
def test_equivalent_units_accepted(value: str) -> None:
    data = example()
    data["density"]["typ"] = value
    assert math.isclose(Material.model_validate(data).density.typ, 1240.0, rel_tol=1e-3)


@pytest.mark.parametrize(
    ("name", "value", "valid"),
    [
        ("density", "0 kg/m**3", False),
        ("density", "0.001 kg/m**3", True),
        ("youngs_modulus", "0 GPa", False),
        ("poissons_ratio", "0.5", False),
        ("poissons_ratio", "0.499", True),
        ("poissons_ratio", "-1", False),
        ("poissons_ratio", "-0.999", True),
        ("thermal_conductivity", "0 W/(m*K)", False),
        ("specific_heat", "0 J/(kg*K)", False),
        ("max_service_temperature", "0 K", False),
        ("max_service_temperature", "1 K", True),
        ("thermal_expansion", "-1e-6 1/K", True),
    ],
)
def test_physical_bounds_on_min(name: str, value: str, valid: bool) -> None:
    data = example()
    data[name]["min"] = value
    if name == "poissons_ratio":
        data[name]["typ"] = value
        data[name]["max"] = value
    if valid:
        Material.model_validate(data)
    else:
        rejects(data, name)


@pytest.mark.req("MAT-006")
def test_temperature_dependence_parsed() -> None:
    table = Material.model_validate(example()).yield_strength.temperature_dependence
    assert table is not None
    assert [point.temperature for point in table] == [253.0, 296.0, 323.0]
    assert table[2].factor == 0.6


@pytest.mark.req("MAT-006")
@pytest.mark.parametrize(
    ("table", "fragment"),
    [
        ([], "temperature_dependence"),
        ([{"temperature": "296 K", "factor": 1.0}], "temperature_dependence"),
        ([{"temperature": "296 K", "factor": 1.0}, {"temperature": "296 K", "factor": 0.9}], "strictly increasing"),
        ([{"temperature": "300 K", "factor": 1.0}, {"temperature": "296 K", "factor": 0.9}], "strictly increasing"),
        ([{"temperature": "296 K", "factor": 1.0}, {"temperature": "300 K", "factor": 0.0}], "factor"),
        ([{"temperature": "296 K", "factor": 1.0}, {"temperature": "300 K", "factor": -0.1}], "factor"),
        ([{"temperature": "296", "factor": 1.0}, {"temperature": "300 K", "factor": 0.9}], "temperature"),
        ([{"temperature": "296 m", "factor": 1.0}, {"temperature": "300 K", "factor": 0.9}], "temperature"),
        ([{"temperature": "296 K", "factor": 1.0, "note": "x"}, {"temperature": "300 K", "factor": 0.9}], "Extra"),
    ],
)
def test_temperature_dependence_rejected(table: list[dict[str, Any]], fragment: str) -> None:
    data = example()
    data["density"]["temperature_dependence"] = table
    rejects(data, fragment)


# ===== Fatigue =========================================================


@pytest.mark.req("MAT-003")
def test_sn_curve_parsed() -> None:
    curve = Material.model_validate(example()).sn_curve
    assert [point.cycles for point in curve] == [1e3, 1e5, 1e6]
    assert curve[0].stress_amplitude == 25e6


@pytest.mark.req("MAT-003")
def test_sn_curve_flat_segment_accepted() -> None:
    data = example()
    data["sn_curve"][2]["stress_amplitude"] = data["sn_curve"][1]["stress_amplitude"]
    Material.model_validate(data)


@pytest.mark.req("MAT-003")
@pytest.mark.parametrize(
    ("index", "field", "value", "fragment"),
    [
        (2, "stress_amplitude", "30 MPa", "must not increase"),
        (1, "cycles", 1e3, "strictly increasing"),
        (1, "cycles", 1e2, "strictly increasing"),
        (0, "cycles", 0.5, "cycles"),
        (0, "stress_amplitude", "0 MPa", "must be positive"),
        (0, "stress_amplitude", "-5 MPa", "must be positive"),
        (0, "stress_amplitude", "25", "stress_amplitude"),
        (0, "stress_amplitude", "25 kg", "stress_amplitude"),
        (0, "cycles", "1000", "cycles"),
    ],
)
def test_sn_curve_rejected(index: int, field: str, value: Any, fragment: str) -> None:
    data = example()
    data["sn_curve"][index][field] = value
    rejects(data, fragment)


@pytest.mark.req("MAT-003")
def test_sn_curve_cycles_at_one_accepted() -> None:
    data = example()
    data["sn_curve"][0]["cycles"] = 1.0
    Material.model_validate(data)


@pytest.mark.req("MAT-003")
@pytest.mark.parametrize("length", [0, 1])
def test_sn_curve_needs_two_points(length: int) -> None:
    data = example()
    data["sn_curve"] = data["sn_curve"][:length]
    rejects(data, "sn_curve")


# ===== Processes =======================================================


@pytest.mark.req("MAT-009")
@pytest.mark.parametrize("process", [{"kind": "machined"}, {"kind": "molded"}, None])
def test_other_processes_accepted(process: dict[str, str] | None) -> None:
    data = example()
    if process is None:
        del data["process"]
    else:
        data["process"] = process
    Material.model_validate(data)


@pytest.mark.req("MAT-009")
def test_printed_process_attributes() -> None:
    process = Material.model_validate(example()).process
    assert process is not None
    assert process.kind == "printed"
    assert process.walls == 2
    assert process.infill == 0.2
    assert process.layer_height == 0.0002
    assert process.build_direction == [0.0, 0.0, 1.0]


@pytest.mark.req("MAT-009")
def test_unknown_process_kind() -> None:
    data = example()
    data["process"] = {"kind": "cast"}
    rejects(data, "process")


@pytest.mark.req("MAT-009")
@pytest.mark.parametrize("key", ["kind", "build_direction", "layer_height", "walls", "infill", "strength_factors"])
def test_printed_process_required_field_missing(key: str) -> None:
    data = example()
    del data["process"][key]
    rejects(data, key)


@pytest.mark.req("MAT-010")
def test_direction_dependent_strength_factors() -> None:
    process = Material.model_validate(example()).process
    assert process is not None
    assert process.kind == "printed"
    assert (process.strength_factors.in_layer, process.strength_factors.interlayer) == (0.7, 0.3)


@pytest.mark.req("MAT-010")
@pytest.mark.parametrize("axis", ["in_layer", "interlayer"])
@pytest.mark.parametrize(("value", "valid"), [(0.0, False), (0.01, True), (1.0, True), (1.01, False), (-0.5, False)])
def test_strength_factor_bounds(axis: str, value: float, valid: bool) -> None:
    data = example()
    data["process"]["strength_factors"][axis] = value
    if valid:
        Material.model_validate(data)
    else:
        rejects(data, axis)


@pytest.mark.req("MAT-010")
@pytest.mark.parametrize("axis", ["in_layer", "interlayer"])
def test_strength_factor_missing(axis: str) -> None:
    data = example()
    del data["process"]["strength_factors"][axis]
    rejects(data, axis, "Field required")


@pytest.mark.req("MAT-024")
@pytest.mark.parametrize(
    ("value", "valid"),
    [
        ("0 percent", True),
        ("100 percent", True),
        ("55 percent", True),
        ("0.55", True),
        ("100.1 percent", False),
        ("-1 percent", False),
        ("101 percent", False),
        ("20 mm", False),
        (20, False),
    ],
)
def test_infill_bounds(value: Any, valid: bool) -> None:
    data = example()
    data["process"]["infill"] = value
    if valid:
        Material.model_validate(data)
    else:
        rejects(data, "infill")


@pytest.mark.req("MAT-024")
@pytest.mark.parametrize(
    ("value", "valid"), [(0, False), (-1, False), (1, True), (12, True), (2.5, False), ("2", False)]
)
def test_wall_count_bounds(value: Any, valid: bool) -> None:
    data = example()
    data["process"]["walls"] = value
    if valid:
        Material.model_validate(data)
    else:
        rejects(data, "walls")


@pytest.mark.req("MAT-024")
@pytest.mark.parametrize(("value", "valid"), [("0 mm", False), ("-0.1 mm", False), ("0.01 mm", True), ("0.2", False)])
def test_layer_height_bounds(value: str, valid: bool) -> None:
    data = example()
    data["process"]["layer_height"] = value
    if valid:
        Material.model_validate(data)
    else:
        rejects(data, "layer_height")


@pytest.mark.req("MAT-024")
@pytest.mark.parametrize(
    ("value", "valid"),
    [
        ([0.0, 0.0, 1.0], True),
        ([0.6, 0.0, 0.8], True),
        ([0.0, 0.0, 0.0], False),
        ([0.0, 0.0, 2.0], False),
        ([0.0, 1.0], False),
    ],
)
def test_build_direction(value: list[float], valid: bool) -> None:
    data = example()
    data["process"]["build_direction"] = value
    if valid:
        Material.model_validate(data)
    else:
        rejects(data, "build_direction")


# ===== Structure =======================================================


@pytest.mark.req("MAT-001")
@pytest.mark.req("MAT-002")
@pytest.mark.req("MAT-003")
@pytest.mark.parametrize("key", REQUIRED)
def test_required_field_missing(key: str) -> None:
    data = example()
    del data[key]
    rejects(data, key, "Field required")


@pytest.mark.parametrize(
    "location",
    [(), ("density",), ("process",), ("process", "strength_factors"), ("sn_curve", 0)],
)
def test_unknown_key(location: tuple[Any, ...]) -> None:
    data = example()
    target: Any = data
    for step in location:
        target = target[step]
    target["surprise"] = 1
    rejects(data, "Extra inputs")


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("name", 5),
        ("name", "PLA"),
        ("name", ""),
        ("family", "wood"),
        ("placeholder", "yes"),
        ("wide_spread_default", 1),
        ("source", 3),
        ("density", "1240 kg/m**3"),
        ("sn_curve", {"cycles": 1}),
    ],
)
def test_wrong_type(key: str, value: Any) -> None:
    data = example()
    data[key] = value
    rejects(data, key)


def test_not_a_mapping_names_file(tmp_path: Path) -> None:
    path = tmp_path / "list.yaml"
    path.write_text("- 1\n", encoding="utf-8")
    with pytest.raises(MaterialError) as caught:
        load_material_file(path)
    assert str(path) in str(caught.value)


def test_yaml_parse_error_names_file(tmp_path: Path) -> None:
    path = tmp_path / "broken.yaml"
    path.write_text("name: [\n", encoding="utf-8")
    with pytest.raises(MaterialError) as caught:
        load_material_file(path)
    assert "YAML parse error" in str(caught.value)
