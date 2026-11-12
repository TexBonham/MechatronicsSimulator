"""Material overrides file, MaterialSet and the integrity view (MAT-016, 017, 020, 021, 023)."""

from __future__ import annotations

import copy
import io
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import jsonschema
import pytest

from mechsim.materials import (
    DATA_DIR,
    IntegrityMaterialView,
    MaterialError,
    MaterialSet,
    SpreadValues,
    load_overrides,
    load_shipped,
)
from mechsim.materials import overrides as overrides_module
from mechsim.materials.schema import yaml_loader

EXAMPLE_PATH = Path(__file__).resolve().parents[2] / "schemas" / "examples" / "material_overrides.example.yaml"
EXAMPLE_TEXT = EXAMPLE_PATH.read_text(encoding="utf-8")
MPA = 1.0e6
REL = 1e-12


def close(actual: float, expected: float) -> None:
    assert actual == pytest.approx(expected, rel=REL, abs=0.0)


def plain(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): plain(item) for key, item in value.items()}
    if isinstance(value, list):
        return [plain(item) for item in value]
    return value


@pytest.fixture(scope="module")
def example_file(tmp_path_factory: pytest.TempPathFactory) -> Path:
    path = tmp_path_factory.mktemp("overrides") / "material_overrides.yaml"
    path.write_text(EXAMPLE_TEXT, encoding="utf-8")
    return path


def example() -> dict[str, Any]:
    data: dict[str, Any] = plain(yaml_loader().load(EXAMPLE_TEXT))
    return copy.deepcopy(data)


def write(tmp_path: Path, data: Any) -> Path:
    path = tmp_path / "material_overrides.yaml"
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    path.write_text(stream.getvalue(), encoding="utf-8")
    return path


def single(entry: dict[str, Any], material: str = "pla_printed") -> dict[str, Any]:
    return {"material_overrides": {material: entry}}


def material_set(tmp_path: Path, data: Any) -> MaterialSet:
    return MaterialSet(load_shipped(), load_overrides(write(tmp_path, data)))


@dataclass
class Printed:
    kind: str = "printed"
    build_direction: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])


# ===== Applying overrides ==============================================


@pytest.mark.req("MAT-016")
def test_full_spread_override_replaces_the_shipped_spread(example_file: Path) -> None:
    materials = MaterialSet.from_files(example_file)
    cases: list[tuple[Any, float]] = [("min", 28.0), ("typ", 34.0), ("max", 38.0)]
    for spread, expected in cases:
        close(materials.lookup("pla_printed", "yield_strength", 296.0, spread), expected * MPA)
    assert materials.source("pla_printed", "yield_strength") == "coupon test 2026-11-01"


@pytest.mark.req("MAT-016")
@pytest.mark.parametrize("spread", ["min", "typ", "max"])
def test_value_override_sets_min_typ_and_max(example_file: Path, spread: Any) -> None:
    materials = MaterialSet.from_files(example_file)
    assert materials.lookup("pla_printed", "density", 296.0, spread) == pytest.approx(1210.0, rel=REL)
    assert materials.source("pla_printed", "density") == "weighed print 2026-11-01"


@pytest.mark.req("MAT-016")
def test_properties_not_overridden_keep_the_shipped_value_and_source(example_file: Path) -> None:
    shipped = load_shipped()
    materials = MaterialSet.from_files(example_file)
    for name in ["ultimate_strength", "shear_strength", "youngs_modulus", "thermal_expansion"]:
        assert materials.lookup("pla_printed", name, 296.0) == getattr(shipped["pla_printed"], name).typ
        assert materials.source("pla_printed", name) == shipped["pla_printed"].source
    assert materials.material("petg_printed") == shipped["petg_printed"]


@pytest.mark.req("MAT-016")
def test_shipped_database_files_are_untouched(example_file: Path) -> None:
    before = {path.name: path.read_bytes() for path in DATA_DIR.glob("*.yaml")}
    shipped = load_shipped()
    MaterialSet.from_files(example_file)
    assert {path.name: path.read_bytes() for path in DATA_DIR.glob("*.yaml")} == before
    assert load_shipped() == shipped
    close(shipped["pla_printed"].yield_strength.typ, 40.0 * MPA)


@pytest.mark.req("MAT-016")
def test_no_overrides_is_the_shipped_set() -> None:
    assert MaterialSet.from_files().records() == load_shipped()
    assert MaterialSet(load_shipped()).overrides is None


@pytest.mark.req("MAT-016")
@pytest.mark.req("MAT-006")
def test_override_without_temperature_table_keeps_the_shipped_table(example_file: Path) -> None:
    """Shipped PLA yield factor at 309.5 K is 0.8 (midpoint of 1.0 and 0.6); the override's typ 34 MPa scales by it."""
    materials = MaterialSet.from_files(example_file)
    close(materials.lookup("pla_printed", "yield_strength", 309.5), 34.0 * 0.8 * MPA)
    with pytest.raises(MaterialError, match="outside the stated range"):
        materials.lookup("pla_printed", "yield_strength", 330.0)


@pytest.mark.req("MAT-016")
@pytest.mark.req("MAT-007")
def test_override_temperature_table_replaces_the_shipped_table(tmp_path: Path) -> None:
    """Override table (280 K, 1.0), (340 K, 0.4): at 330 K the factor is 1.0 + (50 / 60) * (0.4 - 1.0) = 0.5."""
    entry = {
        "yield_strength": {
            "source": "coupon test 2026-11-01",
            "value": "30 MPa",
            "temperature_dependence": [
                {"temperature": "280 K", "factor": 1.0},
                {"temperature": "340 K", "factor": 0.4},
            ],
        }
    }
    materials = material_set(tmp_path, single(entry))
    close(materials.lookup("pla_printed", "yield_strength", 330.0), 15.0 * MPA)
    with pytest.raises(MaterialError, match=r"280\.0 K to 340\.0 K"):
        materials.lookup("pla_printed", "yield_strength", 260.0)


@pytest.mark.req("MAT-016")
@pytest.mark.req("MAT-011")
def test_strength_factor_override_changes_the_direction_strengths(example_file: Path) -> None:
    """Example factors in-layer 0.8, interlayer 0.4 on the overridden yield typ 34 MPa."""
    materials = MaterialSet.from_files(example_file)
    close(materials.strength("pla_printed", "yield", [0.0, 0.0, 1.0], None, 296.0), 34.0 * 0.4 * MPA)
    close(materials.strength("pla_printed", "yield", [1.0, 0.0, 0.0], Printed(), 296.0), 34.0 * 0.8 * MPA)
    close(materials.strength("pla_printed", "yield", [1.0, 0.0, 1.0], Printed(), 296.0), 34.0 * 0.6 * MPA)
    assert materials.source("pla_printed", "strength_factors") == "coupon test 2026-11-01, Z and XY bars"


@pytest.mark.req("MAT-016")
@pytest.mark.req("MAT-012")
def test_sn_curve_override_is_used_for_the_allowable(example_file: Path) -> None:
    """Override (1e4, 320), (2e6, 60) MPa: at the geometric midpoint sqrt(2e10) the amplitude is sqrt(320 * 60)."""
    materials = MaterialSet.from_files(example_file)
    close(materials.allowable_amplitude("steel_8_8_fastener", 2e10**0.5), (320.0 * 60.0) ** 0.5 * MPA)
    with pytest.raises(MaterialError, match="outside the data range"):
        materials.allowable_amplitude("steel_8_8_fastener", 2.1e6)
    assert materials.source("steel_8_8_fastener", "sn_curve") == "supplier datasheet rev C"


@pytest.mark.req("MAT-016")
@pytest.mark.parametrize(
    "name",
    [
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
    ],
)
def test_every_property_can_be_overridden(tmp_path: Path, name: str) -> None:
    shipped = getattr(load_shipped()["steel_8_8_fastener"], name)
    unit = {
        "density": "kg/m**3",
        "youngs_modulus": "Pa",
        "poissons_ratio": "",
        "yield_strength": "Pa",
        "ultimate_strength": "Pa",
        "shear_strength": "Pa",
        "thermal_conductivity": "W/(m*K)",
        "specific_heat": "J/(kg*K)",
        "thermal_expansion": "1/K",
        "max_service_temperature": "K",
    }[name]
    values = [shipped.min * 0.99, shipped.typ, shipped.max * 1.0]
    entry = {
        "source": "test",
        **{key: f"{value!r} {unit}".strip() for key, value in zip(["min", "typ", "max"], values, strict=True)},
    }
    materials = material_set(tmp_path, single({name: entry}, "steel_8_8_fastener"))
    close(materials.lookup("steel_8_8_fastener", name, 296.0, "min"), values[0])
    assert materials.source("steel_8_8_fastener", name) == "test"


# ===== Overrides file validation =======================================


@pytest.mark.req("MAT-016")
@pytest.mark.parametrize(
    ("entry", "fragments"),
    [
        ({"yield_strength": {"source": "x", "value": "30 MPa", "typ": "30 MPa"}}, ["yield_strength", "not both"]),
        ({"yield_strength": {"source": "x", "min": "30 MPa", "typ": "31 MPa"}}, ["yield_strength", "all of min"]),
        ({"yield_strength": {"source": "x"}}, ["yield_strength", "all of min"]),
        (
            {"yield_strength": {"source": "x", "min": "40 MPa", "typ": "30 MPa", "max": "50 MPa"}},
            ["yield_strength", "min <= typ <= max"],
        ),
        ({"yield_strength": {"source": "x", "value": "30 m"}}, ["yield_strength"]),
        ({"yield_strength": {"source": "x", "value": "-30 MPa"}}, ["yield_strength", "positive"]),
        ({"poissons_ratio": {"source": "x", "value": "0.5"}}, ["poissons_ratio", "(-1, 0.5)"]),
        ({"yield_strength": {"value": "30 MPa"}}, ["yield_strength.source", "Field required"]),
        ({"yield_strength": {"source": "", "value": "30 MPa"}}, ["yield_strength.source"]),
        ({"yield_strength": {"source": "x", "value": 30}}, ["yield_strength.value"]),
        ({"hardness": {"source": "x", "value": "30 MPa"}}, ["hardness", "Extra inputs"]),
        ({}, ["at least one property"]),
        ({"sn_curve": {"source": "x", "points": [{"cycles": 1e3, "stress_amplitude": "9 MPa"}]}}, ["sn_curve.points"]),
        (
            {
                "sn_curve": {
                    "source": "x",
                    "points": [
                        {"cycles": 1e3, "stress_amplitude": "9 MPa"},
                        {"cycles": 1e4, "stress_amplitude": "10 MPa"},
                    ],
                }
            },
            ["sn_curve.points", "must not increase"],
        ),
        ({"strength_factors": {"source": "x", "in_layer": 1.1, "interlayer": 0.3}}, ["strength_factors.in_layer"]),
        ({"strength_factors": {"source": "x", "in_layer": 0.7, "interlayer": 0.0}}, ["strength_factors.interlayer"]),
        ({"strength_factors": {"in_layer": 0.7, "interlayer": 0.3}}, ["strength_factors.source"]),
        (
            {
                "yield_strength": {
                    "source": "x",
                    "value": "30 MPa",
                    "temperature_dependence": [
                        {"temperature": "300 K", "factor": 1.0},
                        {"temperature": "290 K", "factor": 0.9},
                    ],
                }
            },
            ["yield_strength", "strictly increasing"],
        ),
    ],
)
def test_invalid_override_names_file_and_field(tmp_path: Path, entry: dict[str, Any], fragments: list[str]) -> None:
    path = write(tmp_path, single(entry))
    with pytest.raises(MaterialError) as caught:
        load_overrides(path)
    message = str(caught.value)
    assert str(path) in message
    assert "material_overrides.pla_printed" in message
    for fragment in fragments:
        assert fragment in message


@pytest.mark.req("MAT-016")
@pytest.mark.parametrize(
    ("data", "fragment"),
    [
        ({"material_overrides": {}}, "material_overrides"),
        ({}, "material_overrides"),
        ({"material_overrides": {"pla_printed": {"density": {"source": "x", "value": "1 kg/m**3"}}}, "x": 1}, "x"),
        (["pla_printed"], "MaterialOverridesFile"),
    ],
)
def test_invalid_file_shape_names_file(tmp_path: Path, data: Any, fragment: str) -> None:
    path = write(tmp_path, data)
    with pytest.raises(MaterialError) as caught:
        load_overrides(path)
    assert str(path) in str(caught.value)
    assert fragment in str(caught.value)


@pytest.mark.req("MAT-016")
def test_yaml_parse_error_names_file(tmp_path: Path) -> None:
    path = tmp_path / "broken.yaml"
    path.write_text("material_overrides: [\n", encoding="utf-8")
    with pytest.raises(MaterialError, match="YAML parse error"):
        load_overrides(path)


@pytest.mark.req("MAT-016")
def test_missing_file_is_an_error(tmp_path: Path) -> None:
    with pytest.raises(MaterialError, match=r"absent\.yaml"):
        load_overrides(tmp_path / "absent.yaml")


@pytest.mark.req("MAT-016")
def test_strength_factors_on_a_non_printed_material_is_rejected(tmp_path: Path) -> None:
    path = write(
        tmp_path,
        single({"strength_factors": {"source": "x", "in_layer": 0.7, "interlayer": 0.3}}, "steel_8_8_fastener"),
    )
    with pytest.raises(MaterialError) as caught:
        MaterialSet(load_shipped(), load_overrides(path))
    assert str(path) in str(caught.value)
    assert "material_overrides.steel_8_8_fastener.strength_factors" in str(caught.value)


@pytest.mark.req("MAT-023")
@pytest.mark.parametrize("name", ["unobtainium", "PLA", "pla printed", ""])
def test_override_of_an_unknown_material_names_the_reference_and_file(tmp_path: Path, name: str) -> None:
    path = write(tmp_path, single({"density": {"source": "x", "value": "1 kg/m**3"}}, name))
    with pytest.raises(MaterialError) as caught:
        MaterialSet(load_shipped(), load_overrides(path))
    assert str(path) in str(caught.value)
    assert f"unknown material {name!r}" in str(caught.value)


@pytest.mark.req("MAT-023")
@pytest.mark.req("MAT-022")
def test_lookup_of_an_unknown_material_is_an_error_naming_it() -> None:
    materials = MaterialSet.from_files()
    with pytest.raises(MaterialError, match="unknown material 'unobtainium'"):
        materials.lookup("unobtainium", "density", 296.0)
    with pytest.raises(MaterialError, match="unknown material 'unobtainium'"):
        materials.integrity_view("unobtainium")


@pytest.mark.req("MAT-022")
def test_material_set_lookup_of_an_absent_property_names_material_and_property() -> None:
    with pytest.raises(MaterialError, match="material 'pla_printed' has no property 'hardness'"):
        MaterialSet.from_files().lookup("pla_printed", "hardness", 296.0)


# ===== Reproducibility and schema ======================================


@pytest.mark.req("MAT-017")
def test_loaded_overrides_carry_the_exact_recorded_input(example_file: Path) -> None:
    first = load_overrides(example_file)
    second = load_overrides(example_file)
    assert first.text == example_file.read_text(encoding="utf-8")
    assert first.path == example_file
    assert len(first.sha256) == 64
    assert first.sha256 == second.sha256
    assert MaterialSet(load_shipped(), first).records() == MaterialSet(load_shipped(), second).records()


@pytest.mark.req("MAT-017")
def test_recorded_text_reproduces_the_material_values(example_file: Path, tmp_path: Path) -> None:
    """A run archive holds the overrides file's text; writing it back and loading it gives the same values."""
    original = load_overrides(example_file)
    copy_path = tmp_path / "recorded_overrides.yaml"
    copy_path.write_text(original.text, encoding="utf-8")
    restored = load_overrides(copy_path)
    assert restored.sha256 == original.sha256
    assert MaterialSet(load_shipped(), restored).records() == MaterialSet(load_shipped(), original).records()


@pytest.mark.req("MAT-017")
def test_different_overrides_give_a_different_digest(example_file: Path, tmp_path: Path) -> None:
    data = example()
    data["material_overrides"]["pla_printed"]["density"]["value"] = "1211 kg/m**3"
    assert load_overrides(write(tmp_path, data)).sha256 != load_overrides(example_file).sha256


@pytest.mark.req("MAT-017")
def test_design_field_and_metadata_key_names() -> None:
    assert overrides_module.DESIGN_FIELD == "materials.overrides"
    assert overrides_module.METADATA_KEY == "material_overrides"


@pytest.mark.req("MAT-017")
def test_overrides_schema_is_current() -> None:
    assert overrides_module.SCHEMA_PATH.name == "material_overrides.json"
    assert overrides_module.SCHEMA_PATH.read_text(encoding="utf-8") == overrides_module.schema_text()


@pytest.mark.req("MAT-017")
def test_overrides_schema_validates_the_example_and_rejects_bad_documents() -> None:
    schema = json.loads(overrides_module.SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    validator = jsonschema.Draft202012Validator(schema)
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert schema["$id"] == "material_overrides.json"
    assert list(validator.iter_errors(example())) == []
    bare = example()
    bare["material_overrides"]["pla_printed"]["density"]["value"] = 1210
    assert list(validator.iter_errors(bare)) != []
    assert list(validator.iter_errors({"material_overrides": {"pla_printed": {"hardness": {}}}})) != []


@pytest.mark.req("MAT-017")
def test_write_schema_main(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    target = tmp_path / "material_overrides.json"
    monkeypatch.setattr(overrides_module, "SCHEMA_PATH", target)
    assert overrides_module.main(["--write-schema"]) == 0
    assert json.loads(target.read_text(encoding="utf-8"))["$id"] == "material_overrides.json"


# ===== Builder and integrity supply ====================================


@pytest.mark.req("MAT-020")
def test_records_supply_density_with_overrides_applied(example_file: Path) -> None:
    shipped = load_shipped()
    records = MaterialSet.from_files(example_file).records()
    assert records["pla_printed"].density.typ == pytest.approx(1210.0, rel=REL)
    assert records["aluminium_6063_t5_extrusion"].density.typ == shipped["aluminium_6063_t5_extrusion"].density.typ
    assert set(records) == set(shipped)


@pytest.mark.req("MAT-021")
def test_integrity_view_supplies_strengths_with_spreads(example_file: Path) -> None:
    view = MaterialSet.from_files(example_file).integrity_view("pla_printed")
    assert isinstance(view, IntegrityMaterialView)
    assert view.yield_strength(296.0) == SpreadValues(28.0 * MPA, 34.0 * MPA, 38.0 * MPA)
    assert view.ultimate_strength(296.0) == SpreadValues(25.0 * MPA, 45.0 * MPA, 65.0 * MPA)
    assert view.shear_strength(296.0) == SpreadValues(10.0 * MPA, 25.0 * MPA, 40.0 * MPA)
    assert view.source("yield_strength") == "coupon test 2026-11-01"


@pytest.mark.req("MAT-021")
@pytest.mark.req("MAT-011")
def test_integrity_view_supplies_direction_dependent_strengths_with_spreads(example_file: Path) -> None:
    """Override factors interlayer 0.4 along +z, in-layer 0.8 across, on yield 28 / 34 / 38 MPa."""
    view = MaterialSet.from_files(example_file).integrity_view("pla_printed")
    along = view.strength_spread("yield", 296.0, [0.0, 0.0, 1.0])
    across = view.strength_spread("yield", 296.0, [0.0, 1.0, 0.0], Printed())
    for actual, base in [(along.min, 28.0), (along.typ, 34.0), (along.max, 38.0)]:
        close(actual, base * 0.4 * MPA)
    for actual, base in [(across.min, 28.0), (across.typ, 34.0), (across.max, 38.0)]:
        close(actual, base * 0.8 * MPA)
    shear = view.strength_spread("shear", 309.5, [0.0, 0.0, 1.0])
    close(shear.typ, 25.0 * 0.8 * 0.4 * MPA)


@pytest.mark.req("MAT-021")
@pytest.mark.req("MAT-003")
def test_integrity_view_supplies_the_sn_curve_and_allowable() -> None:
    view = MaterialSet.from_files().integrity_view("aluminium_6063_t5_extrusion")
    assert view.sn_curve() == ((1e3, 140.0 * MPA), (1e5, 100.0 * MPA), (1e7, 70.0 * MPA), (5e8, 55.0 * MPA))
    close(view.allowable_amplitude(1e4), (140.0 * 100.0) ** 0.5 * MPA)
    with pytest.raises(MaterialError, match="outside the data range"):
        view.allowable_amplitude(1e9)


@pytest.mark.req("MAT-021")
def test_integrity_view_isotropic_material_ignores_direction() -> None:
    view = MaterialSet.from_files().integrity_view("steel_8_8_fastener")
    assert view.strength_spread("yield", 296.0, [1.0, 2.0, 3.0]) == view.yield_strength(296.0)
    assert view.yield_strength(296.0) == SpreadValues(640.0 * MPA, 660.0 * MPA, 800.0 * MPA)
