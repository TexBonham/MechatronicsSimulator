"""Catalog alignment (brief M1-X1 item 5) and the milestone 2 types (brief M2-G): load cell, amplifier, IMU, brake."""

import copy
import json
import math
from typing import Any

import jsonschema
import pytest
from pydantic import ValidationError
from ruamel.yaml import YAML

from mechsim.library.registry import default_registry
from mechsim.library.schema import (
    CATALOG_DIR,
    SCHEMA_PATH,
    CatalogFile,
    FlagParameter,
    MassFromParameter,
    QuantityParameter,
    dimensions_equal,
    load_catalog_file,
)


# ===== Stepper and estop catalog entries ====================================
@pytest.mark.req("LIB-034")
def test_stepper_curve_starts_at_zero_speed_with_the_holding_torque() -> None:
    registry = default_registry()
    part = registry.resolve_part("nema17_class_generic")
    assert part.torque_speed_curve is not None
    assert part.torque_speed_curve[0].speed == 0.0
    assert part.torque_speed_curve[0].torque == part.values["holding_torque"]
    with pytest.raises(ValueError, match="not the holding torque"):
        registry.resolve_part("nema17_class_generic", {"holding_torque": "0.5 N*m"})


@pytest.mark.req("LIB-034")
def test_estop_relay_has_no_heartbeat_and_supply_output_is_v_out() -> None:
    registry = default_registry()
    relay_pins = [pin.name for pin in registry.component_type("estop_relay").electrical.pins]
    assert "heartbeat" not in relay_pins
    supply_pins = [pin.name for pin in registry.component_type("power_supply").electrical.pins]
    assert "v_out" in supply_pins


# ===== Milestone 2 catalog types (LIB-019, LIB-025, LIB-026, LIB-027) =====

VOLT = "[mass] * [length] ** 2 / [time] ** 3 / [current]"
FORCE = "[mass] * [length] / [time] ** 2"
ACCEL = "[length] / [time] ** 2"
RATE = "1 / [time]"
TORQUE = "[mass] * [length] ** 2 / [time] ** 2"

NEW_TYPES: dict[str, tuple[str, str, str]] = {
    "load_cell": ("load_cell.yaml", "load_cell_5kg_generic", "sensor"),
    "load_cell_amplifier": ("load_cell_amplifier.yaml", "load_cell_amplifier_generic", "front_end"),
    "imu": ("imu.yaml", "imu_6axis_generic", "sensor"),
    "brake": ("brake.yaml", "brake_24v_generic", "safety"),
}

# Per parameter: dimension, SI minimum, exclusive minimum, SI maximum, SI default; None where the brief sets none.
Declared = tuple[str, float | None, bool, float | None, float | None]
PARAMETERS: dict[str, dict[str, Declared]] = {
    "load_cell": {
        "mass": ("[mass]", 0.0, False, None, None),
        "capacity": (FORCE, 0.0, True, None, None),
        "sensitivity": ("dimensionless", 0.0, True, None, None),
        "excitation_voltage": (VOLT, 0.0, True, None, None),
        "resolution": (FORCE, 0.0, False, None, None),
        "noise": (FORCE, 0.0, False, None, 0.0),
        "delay": ("[time]", 0.0, False, None, 0.0),
        "sample_rate": (RATE, 1.0, False, None, None),
        "overload_limit": (FORCE, 0.0, True, None, None),
        "nonlinearity": ("dimensionless", 0.0, False, 1.0, 0.0),
    },
    "load_cell_amplifier": {
        "mass": ("[mass]", 0.0, False, None, None),
        "gain": ("dimensionless", 0.0, True, None, None),
        "output_full_scale": (VOLT, 0.0, True, None, None),
        "offset": (VOLT, None, False, None, 0.0),
        "bandwidth": (RATE, 0.0, True, None, None),
        "supply_voltage": (VOLT, 0.0, True, None, None),
        "supply_current": ("[current]", 0.0, False, None, None),
    },
    "imu": {
        "mass": ("[mass]", 0.0, False, None, None),
        "accel_range": (ACCEL, 0.0, True, None, None),
        "gyro_range": (RATE, 0.0, True, None, None),
        "resolution": (ACCEL, 0.0, False, None, None),
        "gyro_resolution": (RATE, 0.0, False, None, None),
        "noise": (ACCEL, 0.0, False, None, 0.0),
        "gyro_noise": (RATE, 0.0, False, None, 0.0),
        "delay": ("[time]", 0.0, False, None, 0.0),
        "sample_rate": (RATE, 1.0, False, None, None),
        "supply_voltage": (VOLT, 0.0, True, None, None),
        "supply_current": ("[current]", 0.0, False, None, None),
    },
    "brake": {
        "mass": ("[mass]", 0.0, False, None, None),
        "brake_torque": (TORQUE, 0.0, True, None, None),
        "engage_delay": ("[time]", 0.0, False, None, None),
        "release_delay": ("[time]", 0.0, False, None, None),
        "coil_voltage": (VOLT, 0.0, True, None, None),
        "coil_current": ("[current]", 0.0, False, None, None),
    },
}


SI_UNIT: dict[str, str] = {
    "[mass]": "kg",
    FORCE: "N",
    "dimensionless": "",
    VOLT: "V",
    "[time]": "s",
    RATE: "Hz",
    "[current]": "A",
    ACCEL: "m/s^2",
    TORQUE: "N*m",
}

EXPECTED_PART_VALUES: dict[str, dict[str, float | bool]] = {
    "load_cell_5kg_generic": {
        "capacity": 49.05,
        "sensitivity": 2e-3,
        "excitation_voltage": 5.0,
        "resolution": 0.01,
        "sample_rate": 80.0,
        "overload_limit": 1.5 * 49.05,
        "noise": 0.0,
        "delay": 0.0,
    },
    "load_cell_amplifier_generic": {
        "gain": 128.0,
        "output_full_scale": 3.3,
        "offset": 0.0,
        "bandwidth": 10.0,
        "supply_voltage": 5.0,
        "supply_current": 1.5e-3,
    },
    "imu_6axis_generic": {
        "accel_range": 4 * 9.81,
        "gyro_range": math.radians(500.0),
        "resolution": 1.2e-3,
        "gyro_resolution": math.radians(0.015),
        "sample_rate": 200.0,
        "noise": 0.0,
        "gyro_noise": 0.0,
        "delay": 0.0,
    },
    "brake_24v_generic": {
        "brake_torque": 0.5,
        "engage_delay": 0.02,
        "release_delay": 0.03,
        "coil_voltage": 24.0,
        "coil_current": 0.25,
        "power_off_engaged": True,
    },
}

EXPECTED_PINS: dict[str, list[tuple[str, str, str]]] = {
    "load_cell": [("excitation", "power_in", "power"), ("signal", "output", "analog"), ("ground", "ground", "ground")],
    "load_cell_amplifier": [
        ("signal_in", "input", "analog"),
        ("out", "output", "analog"),
        ("vcc", "power_in", "power"),
        ("ground", "ground", "ground"),
    ],
    "imu": [
        ("accel_x", "output", "analog"),
        ("accel_y", "output", "analog"),
        ("accel_z", "output", "analog"),
        ("gyro_x", "output", "analog"),
        ("gyro_y", "output", "analog"),
        ("gyro_z", "output", "analog"),
        ("vcc", "power_in", "power"),
        ("ground", "ground", "ground"),
    ],
    "brake": [("release", "input", "digital"), ("vcc", "power_in", "power"), ("ground", "ground", "ground")],
}


# ===== Parametrized case builders ===========================================
def raw(type_name: str) -> dict[str, Any]:
    data: dict[str, Any] = YAML(typ="safe").load((CATALOG_DIR / NEW_TYPES[type_name][0]).read_text(encoding="utf-8"))
    return copy.deepcopy(data)


def refused(data: Any, *fragments: str) -> None:
    with pytest.raises(ValidationError) as caught:
        CatalogFile.model_validate(data)
    text = str(caught.value)
    for fragment in fragments:
        assert fragment in text, text


def quantity(dimension: str, value: float) -> str:
    unit = SI_UNIT[dimension]
    if unit == "":
        return repr(value)
    return f"{value!r} {unit}"


def wrong_unit(dimension: str) -> str:
    if dimension == "[time]":
        return "1 kg"
    return "1 s"


def cases(builder: Any) -> list[Any]:
    built: list[Any] = []
    for type_name, parameters in PARAMETERS.items():
        for parameter_name, declared in parameters.items():
            for index, case in enumerate(builder(type_name, parameter_name, declared)):
                identifier = f"{type_name}-{parameter_name}-{index}"
                built.append(pytest.param(type_name, parameter_name, *case, id=identifier))
    return built


def bound_cases(type_name: str, parameter_name: str, declared: Declared) -> list[tuple[str, bool]]:
    dimension, minimum, exclusive, maximum, _ = declared
    built: list[tuple[str, bool]] = []
    if minimum is not None:
        built.append((quantity(dimension, minimum), not exclusive))
        built.append((quantity(dimension, minimum + 0.5), True))
        built.append((quantity(dimension, minimum - 0.5), False))
        built.append((quantity(dimension, minimum - 1e-9), False))
    if maximum is not None:
        built.append((quantity(dimension, maximum), True))
        built.append((quantity(dimension, maximum - 0.5), True))
        built.append((quantity(dimension, maximum + 1e-9), False))
        built.append((quantity(dimension, maximum + 0.5), False))
    if minimum is None and maximum is None:
        built.append((quantity(dimension, -1.0e3), True))
        built.append((quantity(dimension, 1.0e3), True))
    return built


def required_cases(type_name: str, parameter_name: str, declared: Declared) -> list[tuple[()]]:
    if declared[4] is None:
        return [()]
    return []


def default_cases(type_name: str, parameter_name: str, declared: Declared) -> list[tuple[float]]:
    if declared[4] is None:
        return []
    return [(declared[4],)]


def single(type_name: str, parameter_name: str, declared: Declared) -> list[tuple[str]]:
    return [(declared[0],)]


def every_type() -> list[str]:
    return list(NEW_TYPES)


def with_value(type_name: str, parameter_name: str, value: str | bool) -> dict[str, Any]:
    data = raw(type_name)
    data["parts"][0]["values"][parameter_name] = value
    if parameter_name not in data["parts"][0]["placeholders"]:
        data["parts"][0]["placeholders"].append(parameter_name)
    return data


# ===== Shipped files and placeholders =======================================
@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
@pytest.mark.req("LIB-027")
@pytest.mark.req("LIB-010")
@pytest.mark.parametrize("type_name", every_type())
def test_shipped_file_loads_and_validates_against_the_schema(type_name: str) -> None:
    file_name, part_name, category = NEW_TYPES[type_name]
    path = CATALOG_DIR / file_name
    catalog = load_catalog_file(path)
    assert [component.name for component in catalog.types] == [type_name]
    assert catalog.types[0].category == category
    assert [part.name for part in catalog.resolved_parts()] == [part_name]
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator(schema).validate(raw(type_name))


@pytest.mark.req("LIB-019")
@pytest.mark.parametrize("type_name", every_type())
def test_type_declares_the_briefed_parameters(type_name: str) -> None:
    component = load_catalog_file(CATALOG_DIR / NEW_TYPES[type_name][0]).types[0]
    quantities = {item.name: item for item in component.parameters if isinstance(item, QuantityParameter)}
    assert sorted(quantities) == sorted(PARAMETERS[type_name])
    for name, (dimension, minimum, exclusive, maximum, default) in PARAMETERS[type_name].items():
        parameter = quantities[name]
        assert dimensions_equal(parameter.dimension, dimension), name
        assert parameter.minimum == minimum, name
        assert parameter.minimum_exclusive is exclusive, name
        assert parameter.maximum == maximum, name
        assert parameter.default == default, name
    assert component.tiers[0].name == "ideal"
    pins = [(pin.name, pin.direction, pin.signal_kind) for pin in component.electrical.pins]
    assert pins == EXPECTED_PINS[type_name]


@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
@pytest.mark.req("LIB-027")
@pytest.mark.req("LIB-012")
@pytest.mark.parametrize("type_name", every_type())
def test_shipped_part_values_in_si_and_flagged_as_placeholders(type_name: str) -> None:
    catalog = load_catalog_file(CATALOG_DIR / NEW_TYPES[type_name][0])
    part = catalog.resolved_parts()[0]
    for name, expected in EXPECTED_PART_VALUES[part.name].items():
        assert part.values[name] == pytest.approx(expected, rel=1e-3), name
    entry = catalog.parts[0]
    assert sorted(entry.placeholders) == sorted(key for key in entry.values if not isinstance(entry.values[key], bool))
    assert entry.source == "placeholder values, no datasheet assigned"


# ===== Sensor types =========================================================
@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
@pytest.mark.parametrize("type_name", ["load_cell", "imu"])
def test_sensor_types_carry_resolution_noise_delay_rate_and_failure_modes(type_name: str) -> None:
    component = load_catalog_file(CATALOG_DIR / NEW_TYPES[type_name][0]).types[0]
    for name in ("resolution", "noise", "delay", "sample_rate"):
        component.parameter(name)
    assert component.failure_modes == ["stuck", "noisy", "dropped"]


@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
def test_load_cell_tiers_ratings_and_mount() -> None:
    component = load_catalog_file(CATALOG_DIR / "load_cell.yaml").types[0]
    assert [tier.name for tier in component.tiers] == ["ideal", "nonlinear"]
    assert component.tiers[1].parameters == ["nonlinearity"]
    assert [(item.name, item.parameter) for item in component.ratings.absolute_maximums] == [
        ("force", "overload_limit")
    ]
    assert [(mount.name, mount.kind) for mount in component.mechanical.mounts] == [("body", "bolt_pattern")]
    assert isinstance(component.mechanical.mass, MassFromParameter)
    assert component.mechanical.mass.parameter == "mass"


@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
def test_load_cell_nonlinear_tier_adds_nonlinearity() -> None:
    data = raw("load_cell")
    data["parts"][0]["tier"] = "nonlinear"
    part = CatalogFile.model_validate(data).resolved_parts()[0]
    assert part.values["nonlinearity"] == 0.0
    data["parts"][0]["values"]["nonlinearity"] = "0.02"
    assert CatalogFile.model_validate(data).resolved_parts()[0].values["nonlinearity"] == pytest.approx(0.02)
    ideal = raw("load_cell")
    assert "nonlinearity" not in CatalogFile.model_validate(ideal).resolved_parts()[0].values


@pytest.mark.req("LIB-027")
def test_brake_names_its_holding_torque_and_flag_defaults_true() -> None:
    component = load_catalog_file(CATALOG_DIR / "brake.yaml").types[0]
    assert component.category == "safety"
    assert component.ratings.holding_torque == "brake_torque"
    flag = component.parameter("power_off_engaged")
    assert isinstance(flag, FlagParameter)
    assert flag.default is True
    data = raw("brake")
    del data["parts"][0]["values"]["power_off_engaged"]
    assert CatalogFile.model_validate(data).resolved_parts()[0].values["power_off_engaged"] is True
    data["parts"][0]["values"]["power_off_engaged"] = False
    assert CatalogFile.model_validate(data).resolved_parts()[0].values["power_off_engaged"] is False


@pytest.mark.req("LIB-027")
@pytest.mark.parametrize("value", ["true", "1 V", 1, None])
def test_brake_flag_wrong_type_rejected(value: Any) -> None:
    refused(with_value("brake", "power_off_engaged", value), "power_off_engaged")


@pytest.mark.req("LIB-019")
@pytest.mark.parametrize("type_name", ["imu", "brake"])
def test_header_states_the_open_design_seam(type_name: str) -> None:
    header = (CATALOG_DIR / NEW_TYPES[type_name][0]).read_text(encoding="utf-8").split("types:")[0]
    if type_name == "imu":
        assert "no serial peripheral mode" in header
        assert "open ruling 2" in header
    else:
        assert "acts_on" in header
        assert "DES-043" in header


@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
@pytest.mark.req("LIB-027")
@pytest.mark.req("LIB-015")
@pytest.mark.parametrize(("type_name", "parameter_name", "value", "valid"), cases(bound_cases))
def test_parameter_bound_at_inside_and_outside(type_name: str, parameter_name: str, value: str, valid: bool) -> None:
    data = with_value(type_name, parameter_name, value)
    if valid:
        part = CatalogFile.model_validate(data).resolved_parts()[0]
        assert parameter_name in part.values
    else:
        refused(data, parameter_name)


@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
@pytest.mark.req("LIB-027")
@pytest.mark.req("LIB-014")
@pytest.mark.parametrize(("type_name", "parameter_name"), cases(required_cases))
def test_required_parameter_missing(type_name: str, parameter_name: str) -> None:
    data = raw(type_name)
    del data["parts"][0]["values"][parameter_name]
    data["parts"][0]["placeholders"].remove(parameter_name)
    refused(data, f"missing required parameter {parameter_name!r}")


@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
@pytest.mark.req("LIB-027")
@pytest.mark.req("LIB-014")
@pytest.mark.parametrize(("type_name", "parameter_name", "default"), cases(default_cases))
def test_defaulted_parameter_omitted_takes_its_default(type_name: str, parameter_name: str, default: float) -> None:
    data = raw(type_name)
    data["parts"][0]["values"].pop(parameter_name, None)
    data["parts"][0]["tier"] = data["types"][0]["tiers"][-1]["name"]
    assert CatalogFile.model_validate(data).resolved_parts()[0].values[parameter_name] == default


@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
@pytest.mark.req("LIB-027")
@pytest.mark.req("LIB-013")
@pytest.mark.parametrize(("type_name", "parameter_name", "dimension"), cases(single))
def test_wrong_dimension_rejected(type_name: str, parameter_name: str, dimension: str) -> None:
    refused(with_value(type_name, parameter_name, wrong_unit(dimension)), parameter_name, "expects dimension")


@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
@pytest.mark.req("LIB-027")
@pytest.mark.req("LIB-012")
@pytest.mark.parametrize(("type_name", "parameter_name", "dimension"), cases(single))
def test_missing_unit_or_wrong_type_rejected(type_name: str, parameter_name: str, dimension: str) -> None:
    if dimension != "dimensionless":
        refused(with_value(type_name, parameter_name, "1"), parameter_name)
    for text in ("", "unit", "V 1", True):
        refused(with_value(type_name, parameter_name, text), parameter_name)
    for value in (1.0, 1, None, [], {}):
        refused(with_value(type_name, parameter_name, value), "values")


@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
@pytest.mark.req("LIB-027")
@pytest.mark.req("LIB-010")
@pytest.mark.parametrize("type_name", every_type())
@pytest.mark.parametrize("where", ["root", "type", "parameter", "pin", "part", "values"])
def test_unknown_key_rejected(type_name: str, where: str) -> None:
    data = raw(type_name)
    if where == "root":
        data["colour"] = "red"
        refused(data, "colour")
    elif where == "type":
        data["types"][0]["colour"] = "red"
        refused(data, "colour")
    elif where == "parameter":
        data["types"][0]["parameters"][0]["colour"] = "red"
        refused(data, "colour")
    elif where == "pin":
        data["types"][0]["electrical"]["pins"][0]["colour"] = "red"
        refused(data, "colour")
    elif where == "part":
        data["parts"][0]["colour"] = "red"
        refused(data, "colour")
    else:
        data["parts"][0]["values"]["colour"] = "1 m"
        refused(data, "unknown parameter 'colour'")


@pytest.mark.req("LIB-019")
@pytest.mark.req("LIB-025")
@pytest.mark.req("LIB-027")
@pytest.mark.req("LIB-010")
@pytest.mark.parametrize("type_name", every_type())
@pytest.mark.parametrize("collection", ["parameters", "tiers", "pins", "values"])
def test_empty_collections(type_name: str, collection: str) -> None:
    data = raw(type_name)
    if collection == "parameters":
        data["types"][0]["parameters"] = []
        refused(data, "parameters")
    elif collection == "tiers":
        data["types"][0]["tiers"] = []
        refused(data, "tiers")
    elif collection == "pins":
        data["types"][0]["electrical"]["pins"] = []
        data["types"][0]["ratings"]["absolute_maximums"] = []
        assert CatalogFile.model_validate(data).types[0].electrical.pins == []
    else:
        data["parts"][0]["values"] = {}
        data["parts"][0]["placeholders"] = []
        refused(data, "missing required parameter")


@pytest.mark.req("LIB-026")
def test_front_end_category_accepted_and_unknown_category_refused() -> None:
    amplifier = load_catalog_file(CATALOG_DIR / "load_cell_amplifier.yaml").types[0]
    assert amplifier.category == "front_end"
    for category in ("frontend", "front-end", "amplifier", "", "FRONT_END"):
        data = raw("load_cell_amplifier")
        data["types"][0]["category"] = category
        refused(data, "category")


@pytest.mark.req("LIB-011")
def test_exclusive_minimum_needs_a_minimum_and_room_below_the_maximum() -> None:
    data = raw("load_cell")
    del data["types"][0]["parameters"][1]["minimum"]
    refused(data, "minimum_exclusive needs a minimum")
    data = raw("load_cell")
    data["types"][0]["parameters"][2]["maximum"] = "0"
    refused(data, "leaves no value")
    data = raw("load_cell")
    data["types"][0]["parameters"][2]["maximum"] = "2e-3"
    assert CatalogFile.model_validate(data).types[0].parameters[2].name == "sensitivity"


@pytest.mark.req("LIB-011")
@pytest.mark.parametrize(("default", "valid"), [("0 N", False), ("1e-9 N", True), ("-1 N", False)])
def test_default_at_an_exclusive_minimum_rejected(default: str, valid: bool) -> None:
    data = raw("load_cell")
    data["types"][0]["parameters"][1]["default"] = default
    if valid:
        assert CatalogFile.model_validate(data).types[0].parameters[1].name == "capacity"
    else:
        refused(data, "capacity")


@pytest.mark.req("LIB-011")
@pytest.mark.parametrize("value", ["true", 1, None])
def test_exclusive_minimum_wrong_type_rejected(value: Any) -> None:
    data = raw("load_cell")
    data["types"][0]["parameters"][1]["minimum_exclusive"] = value
    refused(data, "minimum_exclusive")


@pytest.mark.req("LIB-011")
def test_exclusive_minimum_defaults_false_so_the_bound_is_inclusive() -> None:
    data = raw("load_cell")
    del data["types"][0]["parameters"][1]["minimum_exclusive"]
    data["parts"][0]["values"]["capacity"] = "0 N"
    assert CatalogFile.model_validate(data).resolved_parts()[0].values["capacity"] == 0.0


# ===== Milestone 3 structural types (LIB-020, LIB-030) ======================

STRUCTURAL_TYPES: dict[str, tuple[str, str, list[str]]] = {
    "flexible_beam": ("flexible_beam.yaml", "body", ["steel_bar_10mm_1m", "extrusion_2020_500mm_generic"]),
    "hook": ("hook.yaml", "end_effector", ["hook_5kg_generic"]),
    "rope": ("rope.yaml", "rope", ["rope_3mm_steel_generic"]),
}
STRUCTURAL_PARAMETERS: dict[str, dict[str, str]] = {
    "flexible_beam": {
        "area": "[length] ** 2",
        "second_moment": "[length] ** 4",
        "length": "[length]",
        "elastic_modulus": "[mass] / [length] / [time] ** 2",
        "density": "[mass] / [length] ** 3",
    },
    "hook": {"mass": "[mass]", "capacity": "[mass]"},
    "rope": {
        "axial_stiffness": FORCE,
        "damping": "[mass] * [length] / [time]",
        "mass_per_length": "[mass] / [length]",
        "breaking_tension": FORCE,
        "rest_length": "[length]",
    },
}


def _structural_raw(type_name: str) -> dict[str, Any]:
    data: dict[str, Any] = YAML(typ="safe", pure=True).load(
        (CATALOG_DIR / STRUCTURAL_TYPES[type_name][0]).read_text(encoding="utf-8")
    )
    return data


@pytest.mark.req("LIB-020")
@pytest.mark.parametrize("type_name", list(STRUCTURAL_TYPES))
def test_structural_type_ships_validates_and_is_registered(type_name: str) -> None:
    file_name, category, parts = STRUCTURAL_TYPES[type_name]
    catalog = load_catalog_file(CATALOG_DIR / file_name)
    assert [component.name for component in catalog.types] == [type_name]
    assert catalog.types[0].category == category
    assert [part.name for part in catalog.resolved_parts()] == parts
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator(schema).validate(_structural_raw(type_name))
    registry = default_registry()
    assert registry.has_type(type_name)
    for part in parts:
        assert registry.part(part).type == type_name


@pytest.mark.req("LIB-020")
@pytest.mark.req("LIB-030")
@pytest.mark.parametrize("type_name", list(STRUCTURAL_TYPES))
def test_structural_type_parameters_are_positive_quantities(type_name: str) -> None:
    component = load_catalog_file(CATALOG_DIR / STRUCTURAL_TYPES[type_name][0]).types[0]
    quantities = {item.name: item for item in component.parameters if isinstance(item, QuantityParameter)}
    assert sorted(quantities) == sorted(STRUCTURAL_PARAMETERS[type_name])
    for name, dimension in STRUCTURAL_PARAMETERS[type_name].items():
        assert dimensions_equal(quantities[name].dimension, dimension), name
        assert quantities[name].minimum == 0.0, name
        assert quantities[name].default is None, name


@pytest.mark.req("LIB-030")
def test_flexible_beam_carries_section_and_material_on_top_of_the_rigid_body() -> None:
    component = load_catalog_file(CATALOG_DIR / "flexible_beam.yaml").types[0]
    assert component.mechanical.mass.source == "geometry"
    assert [mount.name for mount in component.mechanical.mounts] == ["root", "end"]
    part = default_registry().part("steel_bar_10mm_1m")
    assert part.values == pytest.approx(
        {"area": 1e-4, "second_moment": 0.01**4 / 12.0, "length": 1.0, "elastic_modulus": 2.0e11, "density": 7850.0},
        rel=1e-15,
    )


@pytest.mark.req("LIB-020")
def test_hook_is_an_attach_point_with_capacity_and_a_rope_mount() -> None:
    component = load_catalog_file(CATALOG_DIR / "hook.yaml").types[0]
    assert [mount.name for mount in component.mechanical.mounts] == ["attach", "rope"]
    assert component.events == ["attach", "release"]
    assert [item.parameter for item in component.ratings.absolute_maximums] == ["capacity"]
    assert default_registry().part("hook_5kg_generic").values["capacity"] == 5.0


@pytest.mark.req("LIB-020")
def test_rope_is_the_adr_0016_element_with_snap_in_its_own_tier() -> None:
    component = load_catalog_file(CATALOG_DIR / "rope.yaml").types[0]
    assert component.tier_parameters("ideal") == ["axial_stiffness", "damping", "mass_per_length", "rest_length"]
    assert component.tier_parameters("breaking")[-1] == "breaking_tension"
    assert [item.parameter for item in component.ratings.absolute_maximums] == ["breaking_tension"]
    values = default_registry().part("rope_3mm_steel_generic").values
    assert values == pytest.approx(
        {
            "axial_stiffness": 5.0e5,
            "damping": 50.0,
            "mass_per_length": 0.035,
            "breaking_tension": 5.0e3,
            "rest_length": 0.3,
        },
        rel=1e-15,
    )


@pytest.mark.req("LIB-020")
@pytest.mark.parametrize("missing", ["axial_stiffness", "damping", "mass_per_length", "rest_length"])
def test_rope_category_requires_each_rope_parameter(missing: str) -> None:
    data = _structural_raw("rope")
    component = data["types"][0]
    component["parameters"] = [item for item in component["parameters"] if item["name"] != missing]
    for tier in component["tiers"]:
        tier["parameters"] = [name for name in tier["parameters"] if name != missing]
    component["tiers"] = [tier for tier in component["tiers"] if len(tier["parameters"]) > 0]
    component["ratings"]["absolute_maximums"] = []
    data["parts"] = []
    with pytest.raises(ValidationError, match=f"must carry parameter '{missing}'"):
        CatalogFile.model_validate(data)


@pytest.mark.req("LIB-030")
@pytest.mark.parametrize(
    ("parameter", "value"), [("area", "0 m**2"), ("second_moment", "-1e-9 m**4"), ("density", "0 kg/m**3")]
)
def test_flexible_beam_part_rejects_non_positive_section_values(parameter: str, value: str) -> None:
    data = _structural_raw("flexible_beam")
    data["parts"][0]["values"][parameter] = value
    with pytest.raises(ValidationError):
        CatalogFile.model_validate(data)
