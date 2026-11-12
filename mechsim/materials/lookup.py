"""Property lookups on material records (MAT-006 to 008, 011, 012, 022).

Temperature tables and S-N curves are evaluated inside their stated data range only; outside it a lookup ends with a
`MaterialError` (invariant 7), never an extrapolated value.
"""

from __future__ import annotations

import bisect
import math
from collections.abc import Sequence
from typing import Literal, Protocol, get_args

from mechsim.materials.schema import (
    SPREAD_PROPERTIES,
    Material,
    MaterialError,
    PrintedProcess,
    Spread,
    StrengthFactors,
)

SpreadChoice = Literal["min", "typ", "max"]
StrengthKind = Literal["yield", "ultimate", "shear"]

SPREAD_CHOICES: tuple[str, ...] = get_args(SpreadChoice)
STRENGTH_KINDS: tuple[str, ...] = get_args(StrengthKind)
ISOTROPIC_PROCESSES: tuple[str, ...] = ("machined", "molded", "extrusion")


class ProcessLike(Protocol):
    """A body's process attributes: `kind`, and for `printed` a `build_direction` in the body frame."""

    @property
    def kind(self) -> str: ...


def property_spread(material: Material, property_name: str) -> Spread:
    """The record's spread for `property_name`; a name that is not a spread property ends with an error (MAT-022)."""
    if property_name not in SPREAD_PROPERTIES:
        raise MaterialError(f"material {material.name!r} has no property {property_name!r}")
    value = getattr(material, property_name)
    if not isinstance(value, Spread):
        raise MaterialError(f"material {material.name!r} has no property {property_name!r}")
    return value


def _check_spread(spread: str) -> None:
    if spread not in SPREAD_CHOICES:
        raise MaterialError(f"spread must be one of min, typ, max, got {spread!r}")


def temperature_factor(table: Spread, temperature: float, material: str, property_name: str) -> float:
    """Linear interpolation of the temperature table; 1.0 when the property states no temperature dependence."""
    if not math.isfinite(temperature):
        raise MaterialError(
            f"material {material!r} property {property_name!r}: temperature {temperature} K is not finite"
        )
    points = table.temperature_dependence
    if points is None:
        return 1.0
    low = points[0].temperature
    high = points[-1].temperature
    if temperature < low or temperature > high:
        raise MaterialError(
            f"material {material!r} property {property_name!r}: temperature {temperature} K is outside the stated "
            f"range {low} K to {high} K"
        )
    temperatures = [point.temperature for point in points]
    index = bisect.bisect_left(temperatures, temperature)
    if temperatures[index] == temperature:
        return points[index].factor
    before = points[index - 1]
    after = points[index]
    weight = (temperature - before.temperature) / (after.temperature - before.temperature)
    return before.factor + weight * (after.factor - before.factor)


def lookup(material: Material, property_name: str, temperature: float, spread: SpreadChoice = "typ") -> float:
    """The property's `spread` value (SI) times its temperature factor at `temperature` (kelvin)."""
    _check_spread(spread)
    table = property_spread(material, property_name)
    base: float = getattr(table, spread)
    return base * temperature_factor(table, temperature, material.name, property_name)


def _checked_norm_squared(direction: Sequence[float]) -> float:
    if len(direction) != 3:
        raise MaterialError(f"loading direction must have three components, got {len(direction)}")
    if not all(math.isfinite(component) for component in direction):
        raise MaterialError(f"loading direction {list(direction)} is not finite")
    norm_squared = sum(component * component for component in direction)
    if norm_squared == 0.0:
        raise MaterialError("loading direction must be nonzero")
    return norm_squared


def direction_factor(factors: StrengthFactors, build_direction: Sequence[float], direction: Sequence[float]) -> float:
    """Placeholder axes model (round 5 ruling 7, owed by the owner): with theta the angle between the loading direction
    and the build direction, f = f_inter cos^2 theta + f_in sin^2 theta. Along the build direction the layers are pulled
    apart (interlayer); across it the load runs in the layer plane (in-layer)."""
    load_norm_squared = _checked_norm_squared(direction)
    build_norm_squared = sum(component * component for component in build_direction)
    dot = sum(a * b for a, b in zip(direction, build_direction, strict=True))
    cos_squared = min(1.0, dot * dot / (load_norm_squared * build_norm_squared))
    sin_squared = 1.0 - cos_squared
    return factors.interlayer * cos_squared + factors.in_layer * sin_squared


def _strength_property(kind: str) -> str:
    if kind not in STRENGTH_KINDS:
        raise MaterialError(f"strength kind must be one of yield, ultimate, shear, got {kind!r}")
    return f"{kind}_strength"


def _process_factor(material: Material, direction: Sequence[float], process: ProcessLike | None) -> float:
    record_process = material.process
    printed_record = isinstance(record_process, PrintedProcess)
    if process is None:
        if isinstance(record_process, PrintedProcess):
            return direction_factor(record_process.strength_factors, record_process.build_direction, direction)
        return 1.0
    if process.kind in ISOTROPIC_PROCESSES:
        if printed_record:
            raise MaterialError(f"material {material.name!r} is printed but the process is {process.kind!r}")
        return 1.0
    if process.kind != "printed":
        raise MaterialError(f"material {material.name!r}: unknown process kind {process.kind!r}")
    if not isinstance(record_process, PrintedProcess):
        raise MaterialError(f"material {material.name!r} has no printed strength factors for a printed process")
    build_direction = getattr(process, "build_direction", None)
    if build_direction is None:
        raise MaterialError(f"material {material.name!r}: printed process has no build_direction")
    components = [float(component) for component in build_direction]
    if len(components) != 3 or sum(component * component for component in components) == 0.0:
        raise MaterialError(f"material {material.name!r}: build_direction {components} is not a nonzero 3-vector")
    return direction_factor(record_process.strength_factors, components, direction)


def strength(
    material: Material,
    kind: StrengthKind,
    direction_body: Sequence[float],
    process: ProcessLike | None,
    temperature: float,
    spread: SpreadChoice = "typ",
) -> float:
    """Strength (Pa) for loading along `direction_body` (body frame). `process` is the body's process attributes; None
    uses the material's own. Printed: the record's strength times the direction factor; other processes: isotropic."""
    _checked_norm_squared(direction_body)
    value = lookup(material, _strength_property(kind), temperature, spread)
    return value * _process_factor(material, direction_body, process)


def allowable_amplitude(material: Material, cycles: float) -> float:
    """Allowable stress amplitude (Pa) at `cycles`, log-log linear between the S-N points; outside the data range an
    error."""
    points = material.sn_curve
    low = points[0].cycles
    high = points[-1].cycles
    if not math.isfinite(cycles) or cycles < low or cycles > high:
        raise MaterialError(
            f"material {material.name!r} S-N curve: {cycles} cycles is outside the data range {low:g} to {high:g}"
        )
    counts = [point.cycles for point in points]
    index = bisect.bisect_left(counts, cycles)
    if counts[index] == cycles:
        return points[index].stress_amplitude
    before = points[index - 1]
    after = points[index]
    weight = math.log(cycles / before.cycles) / math.log(after.cycles / before.cycles)
    return math.exp(
        math.log(before.stress_amplitude) + weight * math.log(after.stress_amplitude / before.stress_amplitude)
    )
