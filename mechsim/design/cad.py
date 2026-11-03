"""CAD import interface (DES-017 to DES-024): STEP and URDF importers, their result types, and the mass cross-check.

Only the interface lives here; Open CASCADE (OCP), CoACD and the URDF parser arrive with the M4 implementations. The
mass cross-check arithmetic is complete: declared values from Fusion or URDF are the design's values, the recomputed
values from STEP are compared with them (decisions log row 22).
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from mechsim.design.schema import JointType

Name = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_]*$")]
Vector3 = tuple[float, float, float]


class _Result(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, frozen=True)


def _finite(values: list[float], what: str) -> None:
    for value in values:
        if not math.isfinite(value):
            raise ValueError(f"{what} must be finite, got {value}")


class MassPropertiesSI(_Result):
    """Mass, center of mass and inertia about it in the body frame, SI units."""

    mass: float = Field(gt=0.0)
    center_of_mass: Vector3
    inertia: tuple[float, float, float, float, float, float] = Field(description="ixx, iyy, izz, ixy, ixz, iyz")

    @model_validator(mode="after")
    def _finite_values(self) -> MassPropertiesSI:
        _finite([self.mass, *self.center_of_mass, *self.inertia], "mass properties")
        for value in self.inertia[:3]:
            if value <= 0.0:
                raise ValueError(f"principal inertia terms must be positive, got {value}")
        return self


class ConvexHull(_Result):
    vertices: list[Vector3] = Field(min_length=4)


class StepImportResult(_Result):
    body: Name
    volume: float = Field(gt=0.0)
    mass_properties: MassPropertiesSI
    hulls: list[ConvexHull]
    needs_mesh_contact: bool = Field(description="True when the shape needs true mesh contact, outside v1 (DES-020)")


class UrdfBody(_Result):
    name: Name
    mass_properties: MassPropertiesSI


class UrdfJoint(_Result):
    name: Name
    type: JointType
    parent: Name
    child: Name
    axis: Vector3 | None = None


class UrdfImportResult(_Result):
    bodies: list[UrdfBody] = Field(min_length=1)
    joints: list[UrdfJoint]


class StepImporter(ABC):
    """Reads a STEP file for one body's geometry and mass properties (DES-017, DES-018, DES-019)."""

    @abstractmethod
    def import_body(self, path: Path, body: str, density: float) -> StepImportResult:
        """Geometry of `path` as `body`, with volume and mass properties from `density` (kg/m**3)."""

    @abstractmethod
    def export_collision_mesh(self, result: StepImportResult, directory: Path) -> list[Path]:
        """Write the convex hulls of `result` (from the convex decomposition library) into `directory`."""


class UrdfImporter(ABC):
    """Parses a URDF file directly into bodies and joints (DES-021); an unknown joint type is an error (DES-024)."""

    @abstractmethod
    def import_file(self, path: Path) -> UrdfImportResult:
        """The bodies and joints of the URDF at `path`."""


# ===== Mass cross-check ================================================


class MassMismatch(_Result):
    quantity: Literal["mass", "center_of_mass", "inertia"]
    error: float = Field(ge=0.0, description="Relative error: mass by mass, com offset by body size, inertia by norm")
    tolerance: float


class MassCheckReport(_Result):
    body: Name
    tolerance: float
    mass_error: float
    center_of_mass_error: float
    inertia_error: float
    mismatches: list[MassMismatch]

    @property
    def within_tolerance(self) -> bool:
        return len(self.mismatches) == 0


def cross_check_mass(
    body: str,
    declared: MassPropertiesSI,
    recomputed: MassPropertiesSI,
    tolerance: float,
    body_size: float,
) -> MassCheckReport:
    """Compare recomputed mass properties with the declared ones (DES-022); an error equal to `tolerance` passes.

    Mass error is |m_r - m_d| / m_d; center-of-mass error is the offset distance over `body_size` (a characteristic
    length of the body, m); inertia error is the Frobenius norm of the tensor difference over the declared norm.
    """
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError(f"tolerance must be positive and finite, got {tolerance}")
    if not math.isfinite(body_size) or body_size <= 0.0:
        raise ValueError(f"body_size must be positive and finite, got {body_size}")
    mass_error = abs(recomputed.mass - declared.mass) / declared.mass
    offset = math.dist(recomputed.center_of_mass, declared.center_of_mass)
    center_of_mass_error = offset / body_size
    inertia_error = _tensor_norm(_difference(recomputed.inertia, declared.inertia)) / _tensor_norm(declared.inertia)
    mismatches: list[MassMismatch] = []
    errors: list[tuple[Literal["mass", "center_of_mass", "inertia"], float]] = [
        ("mass", mass_error),
        ("center_of_mass", center_of_mass_error),
        ("inertia", inertia_error),
    ]
    for quantity, error in errors:
        if error > tolerance:
            mismatches.append(MassMismatch(quantity=quantity, error=error, tolerance=tolerance))
    return MassCheckReport(
        body=body,
        tolerance=tolerance,
        mass_error=mass_error,
        center_of_mass_error=center_of_mass_error,
        inertia_error=inertia_error,
        mismatches=mismatches,
    )


def _difference(left: tuple[float, ...], right: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _tensor_norm(terms: tuple[float, ...]) -> float:
    diagonal = terms[0] ** 2 + terms[1] ** 2 + terms[2] ** 2
    off_diagonal = terms[3] ** 2 + terms[4] ** 2 + terms[5] ** 2
    return math.sqrt(diagonal + 2.0 * off_diagonal)
