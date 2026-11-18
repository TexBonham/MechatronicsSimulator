"""Material lookups: temperature, spread, direction-dependent strength, S-N (MAT-006 to 012, 022, 024).

Every expected value below is a closed form of the scheme the spec states (linear interpolation in temperature,
cosine-squared direction blend, log-log linear S-N) evaluated on the file's own numbers, never a measured output. The
1e-12 relative bound is a few ulps of double rounding over at most a handful of operations on O(1e8) magnitudes.
"""

from __future__ import annotations

import copy
import io
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pytest

from mechsim.design.schema import MachinedProcess as DesignMachined
from mechsim.design.schema import PrintedProcess as DesignPrinted
from mechsim.materials import (
    DATA_DIR,
    Material,
    MaterialError,
    allowable_amplitude,
    direction_factor,
    load_material_file,
    lookup,
    strength,
)
from mechsim.materials.schema import SPREAD_PROPERTIES, StrengthFactors, yaml_loader

REL = 1e-12
MPA = 1.0e6


def plain(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): plain(item) for key, item in value.items()}
    if isinstance(value, list):
        return [plain(item) for item in value]
    return value


def raw(name: str = "pla_printed") -> dict[str, Any]:
    data: dict[str, Any] = plain(yaml_loader().load((DATA_DIR / f"{name}.yaml").read_text(encoding="utf-8")))
    return copy.deepcopy(data)


def shipped(name: str) -> Material:
    return load_material_file(DATA_DIR / f"{name}.yaml")


@dataclass
class Process:
    kind: str
    build_direction: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])


@dataclass
class BareProcess:
    kind: str


def close(actual: float, expected: float) -> None:
    assert actual == pytest.approx(expected, rel=REL, abs=0.0)


# ===== Temperature =====================================================

PLA_YIELD_TYP = 40.0 * MPA


@pytest.mark.req("MAT-006")
@pytest.mark.parametrize(
    ("temperature", "factor"),
    [
        (253.0, 1.1),
        (296.0, 1.0),
        (323.0, 0.6),
        (274.5, 1.05),
        (309.5, 0.8),
        (316.25, 0.7),
    ],
)
def test_temperature_table_is_linearly_interpolated(temperature: float, factor: float) -> None:
    """PLA yield table (253 K, 1.1), (296 K, 1.0), (323 K, 0.6): nodes return the node factor; 274.5 K and 309.5 K are
    segment midpoints, 316.25 K is three quarters along 296 to 323 K: 1.0 + 0.75 * (0.6 - 1.0) = 0.7."""
    close(lookup(shipped("pla_printed"), "yield_strength", temperature), PLA_YIELD_TYP * factor)


@pytest.mark.req("MAT-006")
@pytest.mark.parametrize("name", SPREAD_PROPERTIES)
def test_every_property_accepts_and_evaluates_a_temperature_dependence(name: str) -> None:
    """Table (300 K, 2.0), (400 K, 1.0): at 350 K the factor is 1.5, so the lookup is 1.5 * typ."""
    data = raw()
    data[name]["temperature_dependence"] = [
        {"temperature": "300 K", "factor": 2.0},
        {"temperature": "400 K", "factor": 1.0},
    ]
    material = Material.model_validate(data)
    typ = getattr(material, name).typ
    close(lookup(material, name, 350.0), 1.5 * typ)
    close(lookup(material, name, 300.0), 2.0 * typ)
    close(lookup(material, name, 400.0), typ)


@pytest.mark.req("MAT-006")
def test_temperature_given_in_celsius_in_the_file_is_held_in_kelvin() -> None:
    data = raw()
    data["density"]["temperature_dependence"] = [
        {"temperature": "0 degC", "factor": 1.0},
        {"temperature": "100 degC", "factor": 0.5},
    ]
    material = Material.model_validate(data)
    close(lookup(material, "density", 323.15), 0.75 * material.density.typ)


@pytest.mark.req("MAT-006")
@pytest.mark.parametrize("temperature", [1.0, 296.0, 5000.0])
def test_property_without_a_table_is_temperature_independent(temperature: float) -> None:
    material = shipped("steel_8_8_fastener")
    assert lookup(material, "yield_strength", temperature) == material.yield_strength.typ


@pytest.mark.req("MAT-007")
@pytest.mark.parametrize("temperature", [252.999, 323.001, 0.0, 1000.0])
def test_temperature_outside_the_table_is_an_error_naming_property_and_range(temperature: float) -> None:
    with pytest.raises(MaterialError) as caught:
        lookup(shipped("pla_printed"), "yield_strength", temperature)
    message = str(caught.value)
    assert "yield_strength" in message
    assert "pla_printed" in message
    assert "253.0 K to 323.0 K" in message


@pytest.mark.req("MAT-007")
@pytest.mark.parametrize("temperature", [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize("name", ["pla_printed", "steel_8_8_fastener"])
def test_non_finite_temperature_is_an_error(temperature: float, name: str) -> None:
    with pytest.raises(MaterialError, match="not finite"):
        lookup(shipped(name), "yield_strength", temperature)


# ===== Spread ==========================================================


@pytest.mark.req("MAT-008")
@pytest.mark.parametrize("name", SPREAD_PROPERTIES)
@pytest.mark.parametrize("spread", ["min", "typ", "max"])
def test_lookup_returns_the_requested_spread(name: str, spread: Any) -> None:
    material = shipped("steel_8_8_fastener")
    assert lookup(material, name, 296.0, spread) == getattr(getattr(material, name), spread)


@pytest.mark.req("MAT-008")
def test_spread_default_is_typical() -> None:
    material = shipped("aluminium_6063_t5_extrusion")
    assert lookup(material, "youngs_modulus", 296.0) == material.youngs_modulus.typ


@pytest.mark.req("MAT-008")
@pytest.mark.parametrize("spread", [("min", 20.0), ("typ", 40.0), ("max", 60.0)])
def test_temperature_factor_applies_to_each_spread(spread: tuple[Any, float]) -> None:
    close(lookup(shipped("pla_printed"), "yield_strength", 309.5, spread[0]), spread[1] * MPA * 0.8)


@pytest.mark.req("MAT-008")
@pytest.mark.parametrize("spread", ["mean", "MIN", ""])
def test_unknown_spread_is_an_error(spread: Any) -> None:
    with pytest.raises(MaterialError, match="spread must be one of"):
        lookup(shipped("pla_printed"), "density", 296.0, spread)


@pytest.mark.req("MAT-022")
@pytest.mark.parametrize("name", ["hardness", "sn_curve", "process", "name", ""])
def test_absent_property_is_an_error_naming_material_and_property(name: str) -> None:
    with pytest.raises(MaterialError) as caught:
        lookup(shipped("pla_printed"), name, 296.0)
    assert "'pla_printed'" in str(caught.value)
    assert repr(name) in str(caught.value)


# ===== Direction-dependent strength ====================================

PLA_IN_LAYER = 0.7
PLA_INTERLAYER = 0.3


@pytest.mark.req("MAT-010")
@pytest.mark.req("MAT-011")
@pytest.mark.parametrize(
    ("direction", "factor"),
    [
        ([0.0, 0.0, 1.0], PLA_INTERLAYER),
        ([0.0, 0.0, -1.0], PLA_INTERLAYER),
        ([0.0, 0.0, 5.0], PLA_INTERLAYER),
        ([1.0, 0.0, 0.0], PLA_IN_LAYER),
        ([0.0, 1.0, 0.0], PLA_IN_LAYER),
        ([1.0, 1.0, 0.0], PLA_IN_LAYER),
        ([1.0, 0.0, 1.0], 0.5 * (PLA_IN_LAYER + PLA_INTERLAYER)),
        ([0.0, -2.0, 2.0], 0.5 * (PLA_IN_LAYER + PLA_INTERLAYER)),
        ([1.0, 0.0, math.sqrt(3.0)], 0.75 * PLA_INTERLAYER + 0.25 * PLA_IN_LAYER),
    ],
)
def test_printed_strength_blends_interlayer_along_and_in_layer_across_build(
    direction: list[float], factor: float
) -> None:
    """Build direction +z. Along it (theta 0) the interlayer factor; across it (theta 90 degrees) the in-layer factor;
    at 45 degrees cos^2 = sin^2 = 1/2, the mean; at 30 degrees from the build direction cos^2 = 3/4."""
    material = shipped("pla_printed")
    close(strength(material, "yield", direction, None, 296.0), PLA_YIELD_TYP * factor)


@pytest.mark.req("MAT-011")
@pytest.mark.parametrize(
    ("build", "direction", "factor"),
    [
        ([1.0, 0.0, 0.0], [1.0, 0.0, 0.0], PLA_INTERLAYER),
        ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], PLA_IN_LAYER),
        ([0.0, 1.0, 0.0], [0.0, 1.0, 0.0], PLA_INTERLAYER),
        ([0.0, 1.0, 0.0], [1.0, 0.0, 0.0], PLA_IN_LAYER),
    ],
)
def test_body_process_build_direction_is_used(build: list[float], direction: list[float], factor: float) -> None:
    """The body's own build direction (body frame) governs, not the material record's default +z."""
    material = shipped("pla_printed")
    process = DesignPrinted.model_validate(
        {"kind": "printed", "build_direction": build, "walls": 2, "infill": "20 percent"}
    )
    close(strength(material, "yield", direction, process, 296.0), PLA_YIELD_TYP * factor)


@pytest.mark.req("MAT-010")
@pytest.mark.req("MAT-011")
@pytest.mark.parametrize(
    ("kind", "field"), [("yield", "yield_strength"), ("ultimate", "ultimate_strength"), ("shear", "shear_strength")]
)
@pytest.mark.parametrize("spread", ["min", "typ", "max"])
def test_each_strength_kind_and_spread_scales_by_the_direction_factor(kind: Any, field: str, spread: Any) -> None:
    material = shipped("petg_printed")
    base = getattr(getattr(material, field), spread)
    close(strength(material, kind, [0.0, 0.0, 1.0], None, 296.0, spread), base * PLA_INTERLAYER)
    close(strength(material, kind, [1.0, 0.0, 0.0], None, 296.0, spread), base * PLA_IN_LAYER)


@pytest.mark.req("MAT-011")
def test_direction_factor_combines_with_temperature() -> None:
    close(strength(shipped("pla_printed"), "yield", [0.0, 0.0, 1.0], None, 309.5), PLA_YIELD_TYP * 0.8 * 0.3)


@pytest.mark.req("MAT-011")
@pytest.mark.parametrize(
    "process",
    [None, Process("machined"), BareProcess("molded"), BareProcess("extrusion"), DesignMachined(kind="machined")],
)
@pytest.mark.parametrize("direction", [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [1.0, 2.0, 3.0]])
def test_non_printed_process_is_isotropic(process: Any, direction: list[float]) -> None:
    material = shipped("steel_8_8_fastener")
    assert strength(material, "ultimate", direction, process, 296.0) == material.ultimate_strength.typ


@pytest.mark.req("MAT-011")
def test_printed_process_on_a_material_without_strength_factors_is_an_error() -> None:
    with pytest.raises(MaterialError, match="no printed strength factors"):
        strength(shipped("steel_8_8_fastener"), "yield", [0.0, 0.0, 1.0], Process("printed"), 296.0)


@pytest.mark.req("MAT-011")
@pytest.mark.parametrize("kind", ["machined", "molded", "extrusion"])
def test_isotropic_process_on_a_printed_material_is_an_error(kind: str) -> None:
    with pytest.raises(MaterialError, match="is printed but the process is"):
        strength(shipped("pla_printed"), "yield", [0.0, 0.0, 1.0], BareProcess(kind), 296.0)


@pytest.mark.req("MAT-011")
def test_unknown_process_kind_is_an_error() -> None:
    with pytest.raises(MaterialError, match="unknown process kind 'cast'"):
        strength(shipped("pla_printed"), "yield", [0.0, 0.0, 1.0], BareProcess("cast"), 296.0)


@pytest.mark.req("MAT-011")
def test_printed_process_without_build_direction_is_an_error() -> None:
    with pytest.raises(MaterialError, match="no build_direction"):
        strength(shipped("pla_printed"), "yield", [0.0, 0.0, 1.0], BareProcess("printed"), 296.0)


@pytest.mark.req("MAT-011")
@pytest.mark.parametrize("build", [[0.0, 0.0, 0.0], [1.0, 0.0]])
def test_printed_process_with_degenerate_build_direction_is_an_error(build: list[float]) -> None:
    with pytest.raises(MaterialError, match="build_direction"):
        strength(shipped("pla_printed"), "yield", [0.0, 0.0, 1.0], Process("printed", build), 296.0)


@pytest.mark.req("MAT-011")
@pytest.mark.parametrize(
    ("direction", "fragment"),
    [
        ([0.0, 0.0, 0.0], "nonzero"),
        ([1.0, 0.0], "three components"),
        ([1.0, 0.0, 0.0, 0.0], "three components"),
        ([math.nan, 0.0, 1.0], "not finite"),
        ([math.inf, 0.0, 1.0], "not finite"),
    ],
)
@pytest.mark.parametrize("name", ["pla_printed", "steel_8_8_fastener"])
def test_degenerate_loading_direction_is_an_error(direction: list[float], fragment: str, name: str) -> None:
    with pytest.raises(MaterialError, match=fragment):
        strength(shipped(name), "yield", direction, None, 296.0)


@pytest.mark.req("MAT-011")
@pytest.mark.parametrize("kind", ["torsion", "Yield", ""])
def test_unknown_strength_kind_is_an_error(kind: Any) -> None:
    with pytest.raises(MaterialError, match="strength kind must be one of"):
        strength(shipped("pla_printed"), kind, [0.0, 0.0, 1.0], None, 296.0)


@pytest.mark.req("MAT-010")
def test_direction_factor_bounds_are_the_two_factors() -> None:
    """f = f_inter cos^2 + f_in sin^2 lies between the two factors for any direction."""
    factors = StrengthFactors(in_layer=0.9, interlayer=0.2)
    build = [0.0, 0.0, 1.0]
    for index in range(37):
        angle = math.radians(5.0 * index)
        value = direction_factor(factors, build, [math.sin(angle), 0.0, math.cos(angle)])
        close(value, 0.2 * math.cos(angle) ** 2 + 0.9 * math.sin(angle) ** 2)
        assert 0.2 - 1e-15 <= value <= 0.9 + 1e-15


# ===== S-N =============================================================


def two_point_curve() -> Material:
    data = raw("steel_8_8_fastener")
    data["sn_curve"] = [
        {"cycles": 1e3, "stress_amplitude": "200 MPa"},
        {"cycles": 1e6, "stress_amplitude": "50 MPa"},
    ]
    return Material.model_validate(data)


@pytest.mark.req("MAT-012")
@pytest.mark.req("MAT-003")
@pytest.mark.parametrize(
    ("cycles", "amplitude"),
    [(1e3, 200.0), (1e6, 50.0), (math.sqrt(1e9), 100.0), (1e4, 200.0 * 0.25 ** (1.0 / 3.0))],
)
def test_two_point_curve_is_log_log_linear(cycles: float, amplitude: float) -> None:
    """log S linear in log N: at the geometric midpoint sqrt(N0 N1) the amplitude is sqrt(S0 S1) = sqrt(200 * 50) =
    100 MPa; at 1e4 (one third of the log span) S = S0 (S1 / S0)^(1/3)."""
    close(allowable_amplitude(two_point_curve(), cycles), amplitude * MPA)


@pytest.mark.req("MAT-012")
@pytest.mark.parametrize("cycles", [999.0, 1e3 * (1.0 - 1e-12), 1.000001e6, 1e7, 0.0, -1.0, math.nan, math.inf])
def test_cycles_outside_the_curve_is_an_error(cycles: float) -> None:
    with pytest.raises(MaterialError) as caught:
        allowable_amplitude(two_point_curve(), cycles)
    assert "outside the data range 1000 to 1e+06" in str(caught.value)
    assert "steel_8_8_fastener" in str(caught.value)


@pytest.mark.req("MAT-012")
@pytest.mark.parametrize(
    ("cycles", "amplitude"),
    [(1e3, 25.0), (1e4, math.sqrt(25.0 * 15.0)), (1e5, 15.0), (math.sqrt(1e11), math.sqrt(15.0 * 10.0)), (1e6, 10.0)],
)
def test_shipped_three_point_curve(cycles: float, amplitude: float) -> None:
    """PLA (1e3, 25), (1e5, 15), (1e6, 10) MPa: interior node exact, segment geometric midpoints the geometric mean."""
    close(allowable_amplitude(shipped("pla_printed"), cycles), amplitude * MPA)


@pytest.mark.req("MAT-012")
def test_flat_segment_returns_the_flat_amplitude() -> None:
    data = raw("steel_8_8_fastener")
    data["sn_curve"] = [
        {"cycles": 1e3, "stress_amplitude": "80 MPa"},
        {"cycles": 1e6, "stress_amplitude": "80 MPa"},
    ]
    close(allowable_amplitude(Material.model_validate(data), 4.2e4), 80.0 * MPA)


# ===== Shipped printed materials and process validation ================


@pytest.mark.req("MAT-014")
@pytest.mark.req("MAT-015")
@pytest.mark.req("MAT-003")
@pytest.mark.parametrize("name", ["pla_printed", "petg_printed", "abs_printed"])
def test_shipped_printed_materials_carry_what_the_lookups_need(name: str) -> None:
    material = shipped(name)
    assert material.placeholder is True
    assert material.wide_spread_default is True
    assert len(material.sn_curve) >= 3
    process = material.process
    assert process is not None
    assert process.kind == "printed"
    factors = process.strength_factors
    assert factors.interlayer < factors.in_layer
    assert process.walls >= 1
    assert 0.0 <= process.infill <= 1.0
    kinds: list[tuple[Any, str]] = [
        ("yield", "yield_strength"),
        ("ultimate", "ultimate_strength"),
        ("shear", "shear_strength"),
    ]
    for kind, prop in kinds:
        spread = getattr(material, prop)
        assert spread.temperature_dependence is not None
        assert spread.max >= 2.0 * spread.min
        assert strength(material, kind, [0.0, 0.0, 1.0], None, 296.0) < lookup(material, prop, 296.0)


@pytest.mark.req("MAT-002")
@pytest.mark.parametrize("path", sorted(DATA_DIR.glob("*.yaml")), ids=lambda path: path.stem)
@pytest.mark.parametrize(
    "prop", ["thermal_conductivity", "specific_heat", "thermal_expansion", "max_service_temperature"]
)
def test_thermal_properties_are_looked_up_in_si(path: Path, prop: str) -> None:
    material = load_material_file(path)
    value = lookup(material, prop, 296.0)
    assert value == getattr(material, prop).typ
    assert value > 0.0


@pytest.mark.req("MAT-002")
def test_thermal_units_convert_to_si() -> None:
    """1 BTU/(hr ft delta_degF) = 1.730735 W/(m K); 1 cal/(g K) = 4184 J/(kg K); 1 1/delta_degF = 1.8 1/K;
    0 degC = 273.15 K."""
    data = raw()
    data["thermal_conductivity"] = {
        "min": "1 BTU/(hr*ft*delta_degF)",
        "typ": "1 BTU/(hr*ft*delta_degF)",
        "max": "1 BTU/(hr*ft*delta_degF)",
    }
    data["specific_heat"] = {"min": "1 cal/(g*K)", "typ": "1 cal/(g*K)", "max": "1 cal/(g*K)"}
    data["thermal_expansion"] = {"min": "1e-5 1/delta_degF", "typ": "1e-5 1/delta_degF", "max": "1e-5 1/delta_degF"}
    data["max_service_temperature"] = {"min": "0 degC", "typ": "50 degC", "max": "100 degC"}
    material = Material.model_validate(data)
    assert lookup(material, "thermal_conductivity", 296.0) == pytest.approx(1.730735, rel=1e-6)
    close(lookup(material, "specific_heat", 296.0), 4184.0)
    close(lookup(material, "thermal_expansion", 296.0), 1.8e-5)
    close(lookup(material, "max_service_temperature", 296.0, "min"), 273.15)
    close(lookup(material, "max_service_temperature", 296.0, "max"), 373.15)


def write(tmp_path: Path, data: dict[str, Any]) -> Path:
    path = tmp_path / "bad_material.yaml"
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    path.write_text(stream.getvalue(), encoding="utf-8")
    return path


@pytest.mark.req("MAT-024")
@pytest.mark.req("MAT-009")
@pytest.mark.parametrize(
    ("key", "value", "valid"),
    [
        ("infill", "0 percent", True),
        ("infill", "0.0001 percent", True),
        ("infill", "100 percent", True),
        ("infill", "99.999 percent", True),
        ("infill", "-0.0001 percent", False),
        ("infill", "100.0001 percent", False),
        ("walls", 1, True),
        ("walls", 2, True),
        ("walls", 0, False),
        ("layer_height", "0.0001 mm", True),
        ("layer_height", "0 mm", False),
        ("layer_height", "-0.0001 mm", False),
    ],
)
def test_process_bounds_at_inside_and_outside_name_file_and_field(
    tmp_path: Path, key: str, value: Any, valid: bool
) -> None:
    data = raw()
    data["process"][key] = value
    path = write(tmp_path, data)
    if valid:
        load_material_file(path)
        return
    with pytest.raises(MaterialError) as caught:
        load_material_file(path)
    assert str(path) in str(caught.value)
    assert key in str(caught.value)
