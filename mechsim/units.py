"""Quantity strings in data files, plain SI floats inside (ruling L5, ADR 0012)."""

from __future__ import annotations

from typing import Annotated, Any

import re

import pint
from pydantic import BeforeValidator

_registry = pint.UnitRegistry()
_LEADING_NUMBER = re.compile(r"^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?(\s|$)")


class UnitError(ValueError):
    """A quantity string could not be parsed or has the wrong dimension."""


def parse_quantity(text: Any, dimension: str) -> float:
    """Parse a quantity string such as "2.5 kg" into its SI base-unit magnitude.

    `dimension` is a pint dimensionality string such as "[length]" or "[mass] / [length] ** 3". A bare number is
    rejected: units are explicit in files.
    """
    if isinstance(text, bool) or not isinstance(text, str):
        raise UnitError(f"quantity must be a string with a unit, got {text!r}")
    stripped = text.strip()
    if stripped == "":
        raise UnitError("quantity string is empty")
    if _LEADING_NUMBER.match(stripped) is None:
        raise UnitError(f"quantity {text!r} must start with a number followed by its unit")
    try:
        quantity = _registry.Quantity(stripped)
    except Exception as error:
        raise UnitError(f"cannot parse quantity {text!r}: {error}") from error
    if not isinstance(quantity, _registry.Quantity) or quantity.dimensionless:
        if dimension != "dimensionless":
            raise UnitError(f"quantity {text!r} has no unit; expected {dimension}")
        return float(quantity)
    if not quantity.check(dimension):
        raise UnitError(f"quantity {text!r} has dimension {quantity.dimensionality}, expected {dimension}")
    magnitude = quantity.to_base_units().magnitude
    value = float(magnitude)
    if value != value or value in (float("inf"), float("-inf")):
        raise UnitError(f"quantity {text!r} is not finite")
    return value


def quantity_field(dimension: str) -> Any:
    """Annotated float type for a pydantic field: accepts a unit string, stores the SI magnitude."""

    def _validate(value: Any) -> float:
        if isinstance(value, float) and not isinstance(value, bool):
            raise UnitError(f"expected a quantity string with a unit, got bare number {value!r}")
        return parse_quantity(value, dimension)

    return Annotated[float, BeforeValidator(_validate)]


Length = quantity_field("[length]")
Mass = quantity_field("[mass]")
Time = quantity_field("[time]")
Angle = quantity_field("dimensionless")
Force = quantity_field("[mass] * [length] / [time] ** 2")
Torque = quantity_field("[mass] * [length] ** 2 / [time] ** 2")
Stiffness = quantity_field("[mass] / [time] ** 2")
RotationalStiffness = quantity_field("[mass] * [length] ** 2 / [time] ** 2")
Pressure = quantity_field("[mass] / [length] / [time] ** 2")
Density = quantity_field("[mass] / [length] ** 3")
Velocity = quantity_field("[length] / [time]")
Acceleration = quantity_field("[length] / [time] ** 2")
Voltage = quantity_field("[mass] * [length] ** 2 / [time] ** 3 / [current]")
Current = quantity_field("[current]")
Frequency = quantity_field("1 / [time]")
Inertia = quantity_field("[mass] * [length] ** 2")
Area = quantity_field("[length] ** 2")
SecondMomentOfArea = quantity_field("[length] ** 4")
Temperature = quantity_field("[temperature]")
