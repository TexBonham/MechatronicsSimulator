"""Body geometry for display (VIE-015), from the design file bytes a run archive carries, through the design loader.
Reads only; writes nothing (VIE-017)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pydantic import ValidationError

from mechsim.design.schema import (
    Box,
    Capsule,
    Cylinder,
    Design,
    DesignError,
    PrimitiveGeometry,
    Sphere,
    StepGeometry,
    yaml_loader,
)
from mechsim.schema_base import format_errors


@dataclass(frozen=True)
class ShapeGeometry:
    """One shape in its body's frame; sizes in m (radius, length, box edges), None while an open parameter."""

    shape: str
    position: tuple[float | None, ...]
    orientation: tuple[float, ...]
    sizes: dict[str, float | tuple[float | None, ...] | None]


@dataclass(frozen=True)
class BodyGeometry:
    body: str
    kind: str
    shapes: tuple[ShapeGeometry, ...]
    file: str | None = None


def _number(value: Any) -> float | None:
    if isinstance(value, int | float):
        return float(value)
    return None


def _shape(shape: Sphere | Box | Cylinder | Capsule) -> ShapeGeometry:
    sizes: dict[str, float | tuple[float | None, ...] | None] = {}
    if isinstance(shape, Box):
        sizes["size"] = tuple(_number(value) for value in shape.size)
    else:
        sizes["radius"] = _number(shape.radius)
        if isinstance(shape, Cylinder | Capsule):
            sizes["length"] = _number(shape.length)
    return ShapeGeometry(
        shape.shape,
        tuple(_number(value) for value in shape.position),
        tuple(float(value) for value in shape.orientation),
        sizes,
    )


def body_geometry(design_file: bytes, base: Path | None = None) -> tuple[BodyGeometry, ...]:
    """Every body's geometry in design order; a STEP body names its file and carries no shapes."""
    if len(design_file) == 0:
        raise DesignError("the archive carries no design file")
    try:
        data = yaml_loader().load(design_file.decode("utf-8"))
    except Exception as error:
        raise DesignError(f"archived design file: YAML parse error: {error}") from error
    context: dict[str, Any] = {}
    if base is not None:
        context["base"] = base
    try:
        design = Design.model_validate(data, context=context)
    except ValidationError as error:
        raise DesignError("archived design file: " + "; ".join(format_errors(error))) from error
    bodies: list[BodyGeometry] = []
    for body in design.bodies:
        geometry = body.geometry
        if isinstance(geometry, PrimitiveGeometry):
            bodies.append(BodyGeometry(body.name, geometry.kind, tuple(_shape(shape) for shape in geometry.shapes)))
        elif isinstance(geometry, StepGeometry):
            bodies.append(BodyGeometry(body.name, geometry.kind, (), str(geometry.file)))
        else:
            bodies.append(BodyGeometry(body.name, "open", ()))
    return tuple(bodies)
