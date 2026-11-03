"""Quantity strings parse to SI floats and reject everything else (L5, ADR 0012)."""

from __future__ import annotations

import math

import pytest
from pydantic import BaseModel, ValidationError

from mechsim import units


class _Model(BaseModel):
    length: units.Length
    mass: units.Mass
    angle: units.Angle


@pytest.mark.parametrize(
    ("text", "dimension", "expected"),
    [
        ("2.5 kg", "[mass]", 2.5),
        ("1500 g", "[mass]", 1.5),
        ("40 cm", "[length]", 0.4),
        ("1 inch", "[length]", 0.0254),
        ("2 N*m", "[mass] * [length] ** 2 / [time] ** 2", 2.0),
        ("3 kN", "[mass] * [length] / [time] ** 2", 3000.0),
        ("70 GPa", "[mass] / [length] / [time] ** 2", 70e9),
        ("2700 kg/m**3", "[mass] / [length] ** 3", 2700.0),
        ("1 kHz", "1 / [time]", 1000.0),
        ("12 V", "[mass] * [length] ** 2 / [time] ** 3 / [current]", 12.0),
        ("  5 mm  ", "[length]", 0.005),
        ("-3 m", "[length]", -3.0),
        ("0 m", "[length]", 0.0),
        ("1e-3 s", "[time]", 1e-3),
    ],
)
def test_parse_quantity_to_si(text: str, dimension: str, expected: float) -> None:
    assert math.isclose(units.parse_quantity(text, dimension), expected, rel_tol=1e-12, abs_tol=1e-15)


def test_angle_dimensionless_accepts_radian_and_degree() -> None:
    assert math.isclose(units.parse_quantity("90 deg", "dimensionless"), math.pi / 2, rel_tol=1e-12)
    assert math.isclose(units.parse_quantity("1.5 rad", "dimensionless"), 1.5, rel_tol=1e-12)
    assert units.parse_quantity("0.25", "dimensionless") == 0.25


@pytest.mark.parametrize(
    ("text", "dimension"),
    [
        ("2.5", "[mass]"),
        ("", "[mass]"),
        ("   ", "[mass]"),
        ("2.5 m", "[mass]"),
        ("2.5 kg", "[length]"),
        ("kg", "[mass]"),
        ("two kg", "[mass]"),
        ("2.5 parsecs per fortnight squared", "[mass]"),
        ("nan kg", "[mass]"),
        ("inf m", "[length]"),
    ],
)
def test_parse_quantity_rejects(text: str, dimension: str) -> None:
    with pytest.raises(units.UnitError):
        units.parse_quantity(text, dimension)


@pytest.mark.parametrize("value", [2.5, 0, 3, True, None, ["2.5 kg"], {"value": 2.5}])
def test_parse_quantity_rejects_non_strings(value: object) -> None:
    with pytest.raises(units.UnitError):
        units.parse_quantity(value, "[mass]")


def test_pydantic_field_stores_si_magnitude() -> None:
    model = _Model(length="40 cm", mass="1500 g", angle="180 deg")
    assert math.isclose(model.length, 0.4, rel_tol=1e-12)
    assert math.isclose(model.mass, 1.5, rel_tol=1e-12)
    assert math.isclose(model.angle, math.pi, rel_tol=1e-12)


@pytest.mark.parametrize(
    "payload",
    [
        {"length": 0.4, "mass": "1 kg", "angle": "0 rad"},
        {"length": "40 cm", "mass": "1 m", "angle": "0 rad"},
        {"length": "40 cm", "mass": "1 kg", "angle": "1 kg"},
        {"length": "", "mass": "1 kg", "angle": "0 rad"},
    ],
)
def test_pydantic_field_rejects(payload: dict[str, object]) -> None:
    with pytest.raises(ValidationError):
        _Model.model_validate(payload)
