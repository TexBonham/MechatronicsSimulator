"""Component type and catalog file validation (LIB)."""

from __future__ import annotations

import copy
import io
import math
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

from mechsim.library import (
    CATALOG_DIR,
    CatalogError,
    CatalogFile,
    CatalogPart,
    QuantityParameter,
    load_catalog_file,
)
from mechsim.library.schema import yaml_loader

SHIPPED = sorted(CATALOG_DIR.glob("*.yaml"))


def minimal() -> dict[str, Any]:
    return {
        "types": [
            {
                "name": "widget",
                "version": "1.0.0",
                "category": "actuator",
                "parameters": [
                    {"name": "mass", "dimension": "[mass]", "minimum": "0 kg", "maximum": "10 kg"},
                    {"name": "torque", "dimension": "[mass] * [length] ** 2 / [time] ** 2", "default": "1 N*m"},
                    {"name": "voltage", "dimension": "[mass] * [length] ** 2 / [time] ** 3 / [current]"},
                    {"name": "reversed", "kind": "flag", "default": False},
                    {"name": "damping", "dimension": "[mass] * [length] ** 2 / [time]", "default": "0 N*m*s"},
                ],
                "tiers": [
                    {"name": "basic", "parameters": ["mass", "torque", "voltage", "reversed"]},
                    {"name": "detailed", "parameters": ["damping"]},
                ],
                "mechanical": {
                    "mass": {"source": "parameter", "parameter": "mass"},
                    "mounts": [{"name": "face", "kind": "face", "offset": ["0 mm", "0 mm", "5 mm"]}],
                    "shaft_axis": [0.0, 0.0, 1.0],
                },
                "electrical": {
                    "pins": [{"name": "a", "direction": "input", "signal_kind": "digital", "voltage_rating": "voltage"}]
                },
                "ratings": {
                    "absolute_maximums": [{"name": "supply", "parameter": "voltage"}],
                    "holding_torque": "torque",
                },
                "thermal": {"held_constant": True, "temperature": "298.15 K"},
                "plugin": {"python": "my_plugins.widget:Widget"},
            }
        ],
        "parts": [
            {
                "name": "widget_small",
                "type": "widget",
                "tier": "basic",
                "values": {"mass": "2 kg", "voltage": "12 V"},
            }
        ],
    }


def component(data: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = data["types"][0]
    return result


def part(data: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = data["parts"][0]
    return result


def rejects(data: dict[str, Any], *fragments: str) -> None:
    with pytest.raises(ValidationError) as caught:
        CatalogFile.model_validate(data)
    for fragment in fragments:
        assert fragment in str(caught.value)


def write(tmp_path: Path, data: dict[str, Any]) -> Path:
    path = tmp_path / "widget.yaml"
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    path.write_text(stream.getvalue(), encoding="utf-8")
    return path


# ===== Worked examples =================================================


@pytest.mark.req("LIB-010")
@pytest.mark.req("LIB-011")
def test_minimal_example_validates() -> None:
    catalog = CatalogFile.model_validate(minimal())
    assert catalog.types[0].name == "widget"


@pytest.mark.req("LIB-010")
@pytest.mark.parametrize("path", SHIPPED, ids=[path.stem for path in SHIPPED])
def test_shipped_catalog_validates_and_round_trips(path: Path) -> None:
    load_catalog_file(path)
    text = path.read_text(encoding="utf-8")
    yaml = yaml_loader()
    stream = io.StringIO()
    yaml.dump(yaml.load(text), stream)
    assert stream.getvalue() == text


def test_shipped_catalog_holds_the_brief_types() -> None:
    names = set()
    for path in SHIPPED:
        for item in load_catalog_file(path).types:
            names.add(item.name)
    expected = {
        "stepper_behavioral",
        "limit_switch",
        "incremental_encoder",
        "step_dir_driver",
        "lead_screw",
        "belt_pulley",
        "power_supply",
        "estop_relay",
    }
    assert expected <= names


def test_nema17_part_marks_placeholders_and_has_curve() -> None:
    catalog = load_catalog_file(CATALOG_DIR / "stepper_behavioral.yaml")
    nema = catalog.parts[0]
    assert "holding_torque" in nema.placeholders
    assert nema.torque_speed_curve is not None
    assert len(nema.torque_speed_curve) >= 2


@pytest.mark.req("LIB-026")
def test_driver_is_a_separate_type_from_the_motor() -> None:
    driver = load_catalog_file(CATALOG_DIR / "step_dir_driver.yaml").types[0]
    motor = load_catalog_file(CATALOG_DIR / "stepper_behavioral.yaml").types[0]
    assert driver.category == "driver"
    assert motor.category == "actuator"
    assert driver.name != motor.name


# ===== Facets ==========================================================


@pytest.mark.req("LIB-001")
def test_mechanical_facet_holds_mass_mounts_shaft_axis() -> None:
    item = CatalogFile.model_validate(minimal()).types[0]
    assert item.mechanical.mass.source == "parameter"
    assert item.mechanical.mounts[0].offset == [0.0, 0.0, 0.005]
    assert item.mechanical.shaft_axis == [0.0, 0.0, 1.0]


@pytest.mark.req("LIB-001")
def test_mass_from_geometry_needs_no_parameter() -> None:
    data = minimal()
    component(data)["mechanical"]["mass"] = {"source": "geometry"}
    CatalogFile.model_validate(data)


@pytest.mark.req("LIB-001")
def test_mass_parameter_must_exist() -> None:
    data = minimal()
    component(data)["mechanical"]["mass"]["parameter"] = "weight"
    rejects(data, "references unknown parameter 'weight'")


@pytest.mark.req("LIB-001")
def test_mass_parameter_must_have_mass_dimension() -> None:
    data = minimal()
    component(data)["mechanical"]["mass"]["parameter"] = "torque"
    rejects(data, "must have dimension [mass]")


@pytest.mark.req("LIB-001")
@pytest.mark.parametrize("axis", [[0.0, 0.0, 1.0], [0.6, 0.8, 0.0], [0.0, 0.0, 1.0 + 1e-7]])
def test_unit_shaft_axis_accepted(axis: list[float]) -> None:
    data = minimal()
    component(data)["mechanical"]["shaft_axis"] = axis
    CatalogFile.model_validate(data)


@pytest.mark.req("LIB-001")
@pytest.mark.parametrize("axis", [[0.0, 0.0, 0.0], [0.0, 0.0, 2.0], [0.0, 0.0, 1.0 + 1e-5]])
def test_non_unit_shaft_axis_rejected(axis: list[float]) -> None:
    data = minimal()
    component(data)["mechanical"]["shaft_axis"] = axis
    rejects(data, "unit vector")


@pytest.mark.req("LIB-001")
@pytest.mark.parametrize("axis", [[0.0, 1.0], [0.0, 0.0, 1.0, 0.0]])
def test_shaft_axis_needs_three_components(axis: list[float]) -> None:
    data = minimal()
    component(data)["mechanical"]["shaft_axis"] = axis
    rejects(data, "shaft_axis")


@pytest.mark.req("LIB-001")
def test_shaft_axis_optional() -> None:
    data = minimal()
    del component(data)["mechanical"]["shaft_axis"]
    CatalogFile.model_validate(data)


@pytest.mark.req("LIB-002")
def test_electrical_facet_lists_pins_and_signal_kinds() -> None:
    pin = CatalogFile.model_validate(minimal()).types[0].electrical.pins[0]
    assert (pin.direction, pin.signal_kind) == ("input", "digital")


@pytest.mark.req("LIB-002")
@pytest.mark.parametrize(("field", "value"), [("direction", "sideways"), ("signal_kind", "smoke")])
def test_pin_enum_values_checked(field: str, value: str) -> None:
    data = minimal()
    component(data)["electrical"]["pins"][0][field] = value
    rejects(data, field)


@pytest.mark.req("LIB-002")
def test_pin_rating_must_reference_matching_dimension() -> None:
    data = minimal()
    component(data)["electrical"]["pins"][0]["current_rating"] = "voltage"
    rejects(data, "must have dimension [current]")


@pytest.mark.req("LIB-002")
def test_pin_rating_must_reference_known_parameter() -> None:
    data = minimal()
    component(data)["electrical"]["pins"][0]["voltage_rating"] = "volts"
    rejects(data, "unknown parameter 'volts'")


@pytest.mark.req("LIB-003")
def test_ratings_facet_holds_maximums_and_holding_torque() -> None:
    ratings = CatalogFile.model_validate(minimal()).types[0].ratings
    assert ratings.absolute_maximums[0].parameter == "voltage"
    assert ratings.holding_torque == "torque"


@pytest.mark.req("LIB-003")
def test_holding_torque_must_be_a_torque() -> None:
    data = minimal()
    component(data)["ratings"]["holding_torque"] = "mass"
    rejects(data, "holding_torque parameter 'mass' must have dimension")


@pytest.mark.req("LIB-003")
def test_absolute_maximum_cannot_reference_a_flag() -> None:
    data = minimal()
    component(data)["ratings"]["absolute_maximums"][0]["parameter"] = "reversed"
    rejects(data, "references flag parameter 'reversed'")


@pytest.mark.req("LIB-003")
def test_absolute_maximum_must_reference_known_parameter() -> None:
    data = minimal()
    component(data)["ratings"]["absolute_maximums"][0]["parameter"] = "nope"
    rejects(data, "unknown parameter 'nope'")


def curve_data() -> dict[str, Any]:
    data = minimal()
    component(data)["ratings"]["torque_speed_curve"] = "required"
    part(data)["torque_speed_curve"] = [
        {"speed": "0 rpm", "torque": "1 N*m"},
        {"speed": "600 rpm", "torque": "0.5 N*m"},
    ]
    return data


@pytest.mark.req("LIB-003")
def test_torque_speed_curve_parsed_to_si() -> None:
    curve = CatalogFile.model_validate(curve_data()).parts[0].torque_speed_curve
    assert curve is not None
    assert math.isclose(curve[1].speed, 20.0 * math.pi)


@pytest.mark.req("LIB-003")
def test_required_torque_speed_curve_missing() -> None:
    data = curve_data()
    del part(data)["torque_speed_curve"]
    rejects(data, "needs a torque_speed_curve")


@pytest.mark.req("LIB-003")
def test_torque_speed_curve_on_type_without_one() -> None:
    data = curve_data()
    component(data)["ratings"]["torque_speed_curve"] = "none"
    rejects(data, "takes no torque_speed_curve")


@pytest.mark.req("LIB-003")
def test_torque_speed_curve_speeds_increase() -> None:
    data = curve_data()
    part(data)["torque_speed_curve"][1]["speed"] = "0 rpm"
    rejects(data, "strictly increasing")


@pytest.mark.req("LIB-003")
def test_torque_speed_curve_torque_non_negative() -> None:
    data = curve_data()
    part(data)["torque_speed_curve"][1]["torque"] = "-0.1 N*m"
    rejects(data, "non-negative")


@pytest.mark.req("LIB-003")
@pytest.mark.parametrize("points", [[], [{"speed": "0 rpm", "torque": "1 N*m"}]])
def test_torque_speed_curve_needs_two_points(points: list[dict[str, str]]) -> None:
    data = curve_data()
    part(data)["torque_speed_curve"] = points
    rejects(data, "torque_speed_curve")


@pytest.mark.req("LIB-003")
@pytest.mark.parametrize(("field", "value"), [("speed", "100 N*m"), ("torque", "1 rpm"), ("speed", "100")])
def test_torque_speed_curve_units_checked(field: str, value: str) -> None:
    data = curve_data()
    part(data)["torque_speed_curve"][0][field] = value
    rejects(data, field)


@pytest.mark.req("LIB-004")
def test_thermal_slot_held_constant() -> None:
    thermal = CatalogFile.model_validate(minimal()).types[0].thermal
    assert thermal.held_constant is True
    assert thermal.temperature == 298.15


@pytest.mark.req("LIB-004")
def test_thermal_slot_cannot_be_variable_in_v1() -> None:
    data = minimal()
    component(data)["thermal"]["held_constant"] = False
    rejects(data, "held_constant")


@pytest.mark.req("LIB-004")
@pytest.mark.parametrize("value", ["298 kg", "298", 298.0])
def test_thermal_temperature_needs_temperature_unit(value: Any) -> None:
    data = minimal()
    component(data)["thermal"]["temperature"] = value
    rejects(data, "temperature")


# ===== Tiers ===========================================================


@pytest.mark.req("LIB-005")
def test_tiers_are_additive() -> None:
    item = CatalogFile.model_validate(minimal()).types[0]
    assert item.tier_parameters("basic") == ["mass", "torque", "voltage", "reversed"]
    assert item.tier_parameters("detailed") == ["mass", "torque", "voltage", "reversed", "damping"]
    with pytest.raises(KeyError):
        item.tier_parameters("ultra")


@pytest.mark.req("LIB-005")
def test_part_selects_a_named_tier() -> None:
    data = minimal()
    part(data)["tier"] = "ultra"
    rejects(data, "selects tier 'ultra'")


@pytest.mark.req("LIB-005")
def test_part_at_higher_tier_gets_its_defaults() -> None:
    data = minimal()
    part(data)["tier"] = "detailed"
    values = CatalogFile.model_validate(data).resolved_parts()[0].parameter_values()
    assert values["damping"] == 0.0


@pytest.mark.req("LIB-005")
def test_tier_referencing_unknown_parameter() -> None:
    data = minimal()
    component(data)["tiers"][1]["parameters"].append("stiffness")
    rejects(data, "tier 'detailed' references unknown parameter 'stiffness'")


@pytest.mark.req("LIB-005")
def test_parameter_in_two_tiers() -> None:
    data = minimal()
    component(data)["tiers"][1]["parameters"].append("mass")
    rejects(data, "more than one tier")


@pytest.mark.req("LIB-005")
def test_parameter_in_no_tier() -> None:
    data = minimal()
    component(data)["tiers"][1]["parameters"] = []
    rejects(data, "parameter 'damping' belongs to no tier")


@pytest.mark.req("LIB-005")
def test_duplicate_tier_names() -> None:
    data = minimal()
    component(data)["tiers"][1]["name"] = "basic"
    rejects(data, "duplicate type 'widget' tier 'basic'")


# ===== Parameters ======================================================


@pytest.mark.req("LIB-012")
@pytest.mark.req("LIB-033")
def test_values_become_si_floats() -> None:
    catalog = CatalogFile.model_validate(minimal())
    values = catalog.resolved_parts()[0].parameter_values()
    assert values == {"mass": 2.0, "voltage": 12.0, "torque": 1.0, "reversed": False}
    for value in values.values():
        assert isinstance(value, float | bool)
    for parameter in catalog.types[0].parameters:
        if isinstance(parameter, QuantityParameter):
            for bound in (parameter.default, parameter.minimum, parameter.maximum):
                assert bound is None or type(bound) is float


@pytest.mark.req("LIB-012")
def test_shipped_values_become_si_floats() -> None:
    nema = load_catalog_file(CATALOG_DIR / "stepper_behavioral.yaml").resolved_parts()[0].parameter_values()
    assert math.isclose(float(nema["rotor_inertia"]), 54e-7)
    assert math.isclose(float(nema["step_angle"]), math.radians(1.8))


@pytest.mark.req("LIB-033")
def test_no_unit_string_in_any_shipped_resolved_value() -> None:
    for path in SHIPPED:
        catalog = load_catalog_file(path)
        for item in catalog.resolved_parts():
            for value in item.parameter_values().values():
                assert not isinstance(value, str)


@pytest.mark.req("LIB-013")
@pytest.mark.parametrize(("value", "fragment"), [("2", "has no unit"), ("2 m", "has dimension"), ("kg", "must start")])
def test_wrong_or_missing_unit_names_file_parameter_dimension(tmp_path: Path, value: str, fragment: str) -> None:
    data = minimal()
    part(data)["values"]["mass"] = value
    path = write(tmp_path, data)
    with pytest.raises(CatalogError) as caught:
        load_catalog_file(path)
    message = str(caught.value)
    for expected in (str(path), "parameter 'mass'", "[mass]", fragment):
        assert expected in message


@pytest.mark.req("LIB-013")
@pytest.mark.parametrize("field", ["default", "minimum", "maximum"])
def test_type_bound_wrong_unit(field: str) -> None:
    data = minimal()
    component(data)["parameters"][0][field] = "1 m"
    rejects(data, f"parameter 'mass' {field}")


@pytest.mark.req("LIB-013")
@pytest.mark.parametrize("value", ["0 mm", 0.0, "0"])
def test_mount_offset_needs_length_units(value: Any) -> None:
    data = minimal()
    component(data)["mechanical"]["mounts"][0]["offset"][0] = value
    if value == "0 mm":
        CatalogFile.model_validate(data)
    else:
        rejects(data, "offset")


@pytest.mark.req("LIB-013")
def test_unknown_dimension_is_one_clear_error() -> None:
    data = minimal()
    component(data)["parameters"][0]["dimension"] = "[bogus]"
    with pytest.raises(ValidationError) as caught:
        CatalogFile.model_validate(data)
    assert caught.value.error_count() == 1
    assert "parameter 'mass': unknown dimension '[bogus]'" in str(caught.value)


# ===== Part resolution =================================================


@pytest.mark.req("LIB-033")
@pytest.mark.parametrize("context", [None, {}, {"component_type": "widget"}])
def test_part_without_its_type_does_not_validate(context: dict[str, Any] | None) -> None:
    with pytest.raises(ValidationError) as caught:
        CatalogPart.model_validate(part(minimal()), context=context)
    assert "resolves only against its component type" in str(caught.value)


@pytest.mark.req("LIB-033")
def test_part_resolved_against_its_type_holds_si_floats() -> None:
    catalog = CatalogFile.model_validate(minimal())
    resolved = CatalogPart.resolve(part(minimal()), catalog.component_type("widget"))
    assert resolved.values == {"mass": 2.0, "voltage": 12.0, "torque": 1.0, "reversed": False}
    for value in resolved.model_dump()["values"].values():
        assert not isinstance(value, str)
    assert resolved == catalog.resolved_parts()[0]


@pytest.mark.req("LIB-033")
def test_part_entry_resolves_too() -> None:
    catalog = CatalogFile.model_validate(minimal())
    resolved = CatalogPart.resolve(catalog.parts[0], catalog.component_type("widget"))
    assert resolved.values["mass"] == 2.0


@pytest.mark.req("LIB-033")
def test_override_re_resolves_against_the_same_type() -> None:
    catalog = CatalogFile.model_validate(minimal())
    widget = catalog.component_type("widget")
    resolved = CatalogPart.resolve(catalog.parts[0], widget, overrides={"mass": "2500 g", "reversed": True})
    assert resolved.values["mass"] == 2.5
    assert resolved.values["reversed"] is True
    assert resolved.values["voltage"] == 12.0
    assert catalog.resolved_parts()[0].values["mass"] == 2.0


@pytest.mark.req("LIB-015")
@pytest.mark.parametrize(
    ("overrides", "fragment"),
    [
        ({"mass": "11 kg"}, "above maximum"),
        ({"mass": "1 m"}, "expects dimension [mass]"),
        ({"colour": "1 m"}, "unknown"),
    ],
)
def test_override_is_validated(overrides: dict[str, str], fragment: str) -> None:
    catalog = CatalogFile.model_validate(minimal())
    with pytest.raises(ValidationError) as caught:
        CatalogPart.resolve(catalog.parts[0], catalog.component_type("widget"), overrides=overrides)
    assert fragment in str(caught.value)


def test_part_resolved_against_another_type() -> None:
    data = minimal()
    other = copy.deepcopy(component(data))
    other["name"] = "gadget"
    data["types"].append(other)
    catalog = CatalogFile.model_validate(data)
    with pytest.raises(ValidationError) as caught:
        CatalogPart.resolve(catalog.parts[0], catalog.component_type("gadget"))
    assert "resolved against 'gadget'" in str(caught.value)


def test_component_type_lookup_unknown() -> None:
    with pytest.raises(KeyError):
        CatalogFile.model_validate(minimal()).component_type("gizmo")


@pytest.mark.req("LIB-014")
def test_missing_required_parameter_names_file_type_parameter(tmp_path: Path) -> None:
    data = minimal()
    del part(data)["values"]["voltage"]
    path = write(tmp_path, data)
    with pytest.raises(CatalogError) as caught:
        load_catalog_file(path)
    message = str(caught.value)
    for expected in (str(path), "type 'widget'", "required parameter 'voltage'"):
        assert expected in message


@pytest.mark.req("LIB-014")
def test_empty_values_misses_required_parameter() -> None:
    data = minimal()
    part(data)["values"] = {}
    rejects(data, "missing required parameter 'mass'")


@pytest.mark.req("LIB-014")
def test_required_parameter_of_higher_tier_not_needed_at_lower_tier() -> None:
    data = minimal()
    del component(data)["parameters"][4]["default"]
    CatalogFile.model_validate(data)
    part(data)["tier"] = "detailed"
    rejects(data, "missing required parameter 'damping'")


@pytest.mark.req("LIB-015")
@pytest.mark.parametrize("value", ["0 kg", "10 kg", "5 kg", "0.000001 kg", "9999 g"])
def test_part_value_at_and_inside_bounds(value: str) -> None:
    data = minimal()
    part(data)["values"]["mass"] = value
    CatalogFile.model_validate(data)


@pytest.mark.req("LIB-015")
@pytest.mark.parametrize(
    ("value", "fragment"),
    [("-0.001 kg", "below minimum 0.0"), ("10.001 kg", "above maximum 10.0"), ("-2.5 kg", "below minimum")],
)
def test_part_value_outside_bounds_names_file_parameter_value_bound(tmp_path: Path, value: str, fragment: str) -> None:
    data = minimal()
    part(data)["values"]["mass"] = value
    path = write(tmp_path, data)
    with pytest.raises(CatalogError) as caught:
        load_catalog_file(path)
    message = str(caught.value)
    for expected in (str(path), "parameter 'mass'", value, fragment):
        assert expected in message


@pytest.mark.req("LIB-015")
@pytest.mark.parametrize(("default", "valid"), [("0 kg", True), ("10 kg", True), ("3 kg", True), ("11 kg", False)])
def test_type_default_within_bounds(default: str, valid: bool) -> None:
    data = minimal()
    component(data)["parameters"][0]["default"] = default
    if valid:
        CatalogFile.model_validate(data)
    else:
        rejects(data, "above maximum")


@pytest.mark.req("LIB-015")
def test_minimum_above_maximum() -> None:
    data = minimal()
    component(data)["parameters"][0]["minimum"] = "11 kg"
    rejects(data, "minimum 11.0 above maximum 10.0")


@pytest.mark.req("LIB-015")
def test_minimum_equal_maximum_accepted() -> None:
    data = minimal()
    component(data)["parameters"][0]["minimum"] = "10 kg"
    part(data)["values"]["mass"] = "10 kg"
    CatalogFile.model_validate(data)


def test_parameters_sweepable_by_default() -> None:
    item = CatalogFile.model_validate(minimal()).types[0]
    assert all(parameter.sweepable for parameter in item.parameters)
    data = minimal()
    component(data)["parameters"][0]["sweepable"] = False
    assert CatalogFile.model_validate(data).types[0].parameters[0].sweepable is False


def test_duplicate_parameter_names() -> None:
    data = minimal()
    component(data)["parameters"][1]["name"] = "mass"
    rejects(data, "duplicate type 'widget' parameter 'mass'")


def test_flag_parameter_values() -> None:
    data = minimal()
    part(data)["values"]["reversed"] = True
    assert CatalogFile.model_validate(data).resolved_parts()[0].parameter_values()["reversed"] is True
    part(data)["values"]["reversed"] = "1 kg"
    rejects(data, "flag parameter 'reversed' needs true or false")


def test_quantity_parameter_rejects_bool_value() -> None:
    data = minimal()
    part(data)["values"]["mass"] = True
    rejects(data, "needs a quantity string")


# ===== Parts ===========================================================


def test_part_references_unknown_type() -> None:
    data = minimal()
    part(data)["type"] = "gizmo"
    rejects(data, "references unknown component type 'gizmo'")


def test_part_sets_unknown_parameter() -> None:
    data = minimal()
    part(data)["values"]["colour"] = "1 m"
    rejects(data, "sets unknown parameter 'colour'")


def test_placeholder_must_name_a_set_value() -> None:
    data = minimal()
    part(data)["placeholders"] = ["torque"]
    rejects(data, "marks 'torque' as placeholder")


def test_duplicate_placeholder() -> None:
    data = minimal()
    part(data)["placeholders"] = ["mass", "mass"]
    rejects(data, "duplicate part 'widget_small' placeholder")


@pytest.mark.parametrize(
    ("collection", "fragment"),
    [("parts", "duplicate catalog part"), ("types", "duplicate component type")],
)
def test_duplicate_names_in_file(collection: str, fragment: str) -> None:
    data = minimal()
    data[collection].append(copy.deepcopy(data[collection][0]))
    rejects(data, fragment)


@pytest.mark.parametrize(
    ("path", "fragment"),
    [
        (("electrical", "pins"), "pin 'a'"),
        (("mechanical", "mounts"), "mount 'face'"),
        (("ratings", "absolute_maximums"), "absolute maximum 'supply'"),
    ],
)
def test_duplicate_facet_entries(path: tuple[str, str], fragment: str) -> None:
    data = minimal()
    entries = component(data)[path[0]][path[1]]
    entries.append(copy.deepcopy(entries[0]))
    rejects(data, f"duplicate type 'widget' {fragment}")


def test_duplicate_failure_modes() -> None:
    data = minimal()
    component(data)["failure_modes"] = ["stuck", "stuck"]
    rejects(data, "duplicate type 'widget' failure mode 'stuck'")


@pytest.mark.req("LIB-025")
@pytest.mark.parametrize("mode", ["stuck", "noisy", "dropped", "forced_level"])
def test_failure_modes_accepted(mode: str) -> None:
    data = minimal()
    component(data)["failure_modes"] = [mode]
    CatalogFile.model_validate(data)


@pytest.mark.req("LIB-025")
def test_unknown_failure_mode() -> None:
    data = minimal()
    component(data)["failure_modes"] = ["melted"]
    rejects(data, "failure_modes")


# ===== Category rules ==================================================


def sensor_data() -> dict[str, Any]:
    data = minimal()
    item = component(data)
    item["category"] = "sensor"
    item["failure_modes"] = ["stuck"]
    for name, dimension, default in (
        ("resolution", "[length]", "1 mm"),
        ("noise", "[length]", "0 mm"),
        ("delay", "[time]", "0 s"),
        ("sample_rate", "1 / [time]", "1 kHz"),
    ):
        item["parameters"].append({"name": name, "dimension": dimension, "default": default})
        item["tiers"][0]["parameters"].append(name)
    return data


@pytest.mark.req("LIB-025")
def test_sensor_type_carries_sensor_parameters() -> None:
    CatalogFile.model_validate(sensor_data())


@pytest.mark.req("LIB-025")
@pytest.mark.parametrize("missing", ["resolution", "noise", "delay", "sample_rate"])
def test_sensor_type_missing_parameter(missing: str) -> None:
    data = sensor_data()
    item = component(data)
    item["parameters"] = [entry for entry in item["parameters"] if entry["name"] != missing]
    item["tiers"][0]["parameters"].remove(missing)
    rejects(data, f"sensor type 'widget' must carry parameter '{missing}'")


@pytest.mark.req("LIB-025")
def test_sensor_type_needs_failure_modes() -> None:
    data = sensor_data()
    component(data)["failure_modes"] = []
    rejects(data, "must list the failure modes")


@pytest.mark.req("LIB-025")
def test_shipped_sensor_types_carry_sensor_parameters() -> None:
    for name in ("limit_switch", "incremental_encoder"):
        item = load_catalog_file(CATALOG_DIR / f"{name}.yaml").types[0]
        assert item.category == "sensor"
        assert len(item.failure_modes) > 0


def transmission_data() -> dict[str, Any]:
    data = minimal()
    item = component(data)
    item["category"] = "transmission"
    for entry in (
        {"name": "ratio", "dimension": "[length]", "default": "8 mm/turn"},
        {"name": "efficiency", "dimension": "dimensionless", "default": "0.9", "minimum": "0", "maximum": "1"},
        {"name": "stiffness", "dimension": "[mass] / [time] ** 2", "default": "1 kN/m"},
        {"name": "backlash", "dimension": "[length]", "default": "0 mm"},
        {"name": "self_locking", "kind": "flag", "default": False},
        {"name": "slip", "dimension": "dimensionless", "default": "0"},
    ):
        item["parameters"].append(entry)
        item["tiers"][0]["parameters"].append(entry["name"])
    return data


@pytest.mark.req("LIB-024")
def test_transmission_type_carries_transmission_parameters() -> None:
    values = CatalogFile.model_validate(transmission_data()).resolved_parts()[0].parameter_values()
    assert values["slip"] == 0.0
    assert values["self_locking"] is False


@pytest.mark.req("LIB-024")
@pytest.mark.parametrize("missing", ["ratio", "efficiency", "stiffness", "backlash", "self_locking", "slip"])
def test_transmission_type_missing_parameter(missing: str) -> None:
    data = transmission_data()
    item = component(data)
    item["parameters"] = [entry for entry in item["parameters"] if entry["name"] != missing]
    item["tiers"][0]["parameters"].remove(missing)
    rejects(data, f"transmission type 'widget' must carry parameter '{missing}'")


@pytest.mark.req("LIB-024")
@pytest.mark.parametrize("default", ["0.1", None])
def test_transmission_slip_defaults_to_zero(default: str | None) -> None:
    data = transmission_data()
    slip = component(data)["parameters"][-1]
    if default is None:
        del slip["default"]
    else:
        slip["default"] = default
    rejects(data, "'slip' must default to zero")


@pytest.mark.req("LIB-024")
def test_shipped_transmissions_default_slip_zero() -> None:
    for name in ("lead_screw", "belt_pulley"):
        values = load_catalog_file(CATALOG_DIR / f"{name}.yaml").resolved_parts()[0].parameter_values()
        assert values["slip"] == 0.0


# ===== Plugin reference ================================================


@pytest.mark.parametrize(
    "plugin",
    [{"python": "pkg.mod:Cls"}, {"cpp": "stepper_behavioral"}, {"python": "a:B", "cpp": "b"}],
)
def test_plugin_reference_forms(plugin: dict[str, str]) -> None:
    data = minimal()
    component(data)["plugin"] = plugin
    CatalogFile.model_validate(data)


@pytest.mark.parametrize(
    ("plugin", "fragment"),
    [({}, "needs a python import path"), ({"python": "pkg.mod.Cls"}, "python"), ({"cpp": "Bad-Name"}, "cpp")],
)
def test_plugin_reference_rejected(plugin: dict[str, str], fragment: str) -> None:
    data = minimal()
    component(data)["plugin"] = plugin
    rejects(data, fragment)


# ===== Structure =======================================================


@pytest.mark.req("LIB-014")
@pytest.mark.parametrize(
    "key",
    ["name", "version", "category", "parameters", "tiers", "mechanical", "electrical", "ratings", "thermal", "plugin"],
)
def test_type_required_field_missing(key: str) -> None:
    data = minimal()
    del component(data)[key]
    rejects(data, key, "Field required")


@pytest.mark.req("LIB-014")
@pytest.mark.parametrize(
    ("section", "key"),
    [
        ("mechanical", "mass"),
        ("mechanical", "mounts"),
        ("electrical", "pins"),
        ("ratings", "absolute_maximums"),
        ("thermal", "held_constant"),
        ("thermal", "temperature"),
    ],
)
def test_facet_required_field_missing(section: str, key: str) -> None:
    data = minimal()
    del component(data)[section][key]
    rejects(data, key, "Field required")


@pytest.mark.req("LIB-014")
def test_parameter_name_missing() -> None:
    data = minimal()
    del component(data)["parameters"][0]["name"]
    rejects(data, "name", "Field required")


@pytest.mark.req("LIB-014")
@pytest.mark.parametrize("value", [None, 3])
def test_parameter_dimension_missing_or_wrong_type_is_one_error(value: Any) -> None:
    data = minimal()
    if value is None:
        del component(data)["parameters"][0]["dimension"]
    else:
        component(data)["parameters"][0]["dimension"] = value
    with pytest.raises(ValidationError) as caught:
        CatalogFile.model_validate(data)
    assert caught.value.error_count() == 1
    assert "parameter 'mass' needs a dimension string" in str(caught.value)


@pytest.mark.req("LIB-014")
@pytest.mark.parametrize("key", ["name", "direction", "signal_kind"])
def test_pin_required_field_missing(key: str) -> None:
    data = minimal()
    del component(data)["electrical"]["pins"][0][key]
    rejects(data, key, "Field required")


@pytest.mark.req("LIB-014")
@pytest.mark.parametrize("key", ["name", "type", "tier", "values"])
def test_part_required_field_missing(key: str) -> None:
    data = minimal()
    del part(data)[key]
    rejects(data, key, "Field required")


@pytest.mark.parametrize("key", ["types", "parts"])
def test_file_required_field_missing(key: str) -> None:
    data = minimal()
    del data[key]
    rejects(data, key, "Field required")


@pytest.mark.parametrize(
    ("location", "fragment"),
    [
        ((), "Extra inputs"),
        (("types", 0), "Extra inputs"),
        (("types", 0, "parameters", 0), "Extra inputs"),
        (("types", 0, "parameters", 3), "Extra inputs"),
        (("types", 0, "mechanical"), "Extra inputs"),
        (("types", 0, "electrical", "pins", 0), "Extra inputs"),
        (("types", 0, "thermal"), "Extra inputs"),
        (("types", 0, "plugin"), "Extra inputs"),
        (("parts", 0), "Extra inputs"),
    ],
)
def test_unknown_key(location: tuple[Any, ...], fragment: str) -> None:
    data = minimal()
    target: Any = data
    for step in location:
        target = target[step]
    target["surprise"] = 1
    rejects(data, fragment)


@pytest.mark.parametrize(
    ("location", "key", "value"),
    [
        (("types", 0), "name", 5),
        (("types", 0), "version", 1.0),
        (("types", 0), "version", "1.0"),
        (("types", 0), "name", "Widget"),
        (("types", 0), "category", "toaster"),
        (("types", 0), "parameters", "mass"),
        (("types", 0, "parameters", 0), "sweepable", "yes"),
        (("types", 0, "parameters", 3), "default", "no"),
        (("types", 0, "mechanical"), "shaft_axis", ["0", "0", "1"]),
        (("types", 0), "failure_modes", "stuck"),
        (("parts", 0), "values", ["mass"]),
        (("parts", 0, "values"), "mass", 2),
        (("parts", 0, "values"), "mass", 2.0),
    ],
)
def test_wrong_type(location: tuple[Any, ...], key: str, value: Any) -> None:
    data = minimal()
    target: Any = data
    for step in location:
        target = target[step]
    target[key] = value
    with pytest.raises(ValidationError):
        CatalogFile.model_validate(data)


@pytest.mark.parametrize(
    ("location", "valid"),
    [
        (("types",), False),
        (("types", 0, "parameters"), False),
        (("types", 0, "tiers"), False),
        (("parts",), True),
        (("types", 0, "electrical", "pins"), True),
        (("types", 0, "mechanical", "mounts"), True),
        (("types", 0, "ratings", "absolute_maximums"), True),
        (("types", 0, "failure_modes"), True),
    ],
)
def test_empty_collections(location: tuple[Any, ...], valid: bool) -> None:
    data = minimal()
    if location[-1] == "pins" or location[-1] == "absolute_maximums":
        component(data)["electrical"]["pins"] = []
        component(data)["ratings"]["absolute_maximums"] = []
    target: Any = data
    for step in location[:-1]:
        target = target[step]
    target[location[-1]] = []
    if valid:
        CatalogFile.model_validate(data)
    else:
        rejects(data, location[-1])


def test_not_a_mapping_rejected(tmp_path: Path) -> None:
    path = tmp_path / "list.yaml"
    path.write_text("- 1\n- 2\n", encoding="utf-8")
    with pytest.raises(CatalogError) as caught:
        load_catalog_file(path)
    assert str(path) in str(caught.value)


def test_yaml_parse_error_names_file(tmp_path: Path) -> None:
    path = tmp_path / "broken.yaml"
    path.write_text("types: [\n", encoding="utf-8")
    with pytest.raises(CatalogError) as caught:
        load_catalog_file(path)
    assert "YAML parse error" in str(caught.value)
