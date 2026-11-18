"""Shipped materials with a design's overrides applied (MAT-016, 017, 020, 021, 023)."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from mechsim.materials.lookup import (
    ProcessLike,
    SpreadChoice,
    StrengthKind,
    allowable_amplitude,
    lookup,
    strength,
)
from mechsim.materials.overrides import MaterialOverrides, load_overrides
from mechsim.materials.schema import DATA_DIR, Material, MaterialError, PrintedProcess, Spread, load_material_file


def load_shipped(data_dir: Path = DATA_DIR) -> dict[str, Material]:
    materials: dict[str, Material] = {}
    for path in sorted(data_dir.glob("*.yaml")):
        material = load_material_file(path)
        if material.name in materials:
            raise MaterialError(f"{path}: duplicate material name {material.name!r}")
        materials[material.name] = material
    return materials


@dataclass(frozen=True)
class SpreadValues:
    min: float
    typ: float
    max: float


def _merge(name: str, record: Material, overrides: MaterialOverrides) -> Material:
    entry = overrides.file.material_overrides[name]
    update: dict[str, object] = {}
    for field, spread in entry.spreads().items():
        if spread.temperature_dependence is None:
            shipped: Spread = getattr(record, field)
            spread = spread.model_copy(update={"temperature_dependence": shipped.temperature_dependence})
        update[field] = spread
    if entry.sn_curve is not None:
        update["sn_curve"] = list(entry.sn_curve.points)
    if entry.strength_factors is not None:
        if not isinstance(record.process, PrintedProcess):
            raise MaterialError(
                f"{overrides.path}: material_overrides.{name}.strength_factors: material {name!r} is not printed"
            )
        factors = entry.strength_factors
        process = record.process.model_copy(
            update={
                "strength_factors": record.process.strength_factors.model_copy(
                    update={"in_layer": factors.in_layer, "interlayer": factors.interlayer}
                )
            }
        )
        update["process"] = process
    return record.model_copy(update=update)


class MaterialSet:
    """Shipped records with an overrides file applied before the shipped values; lookups by material name."""

    def __init__(self, shipped: Mapping[str, Material], overrides: MaterialOverrides | None = None) -> None:
        self._records: dict[str, Material] = dict(shipped)
        self._sources: dict[tuple[str, str], str] = {}
        self.overrides = overrides
        if overrides is None:
            return
        for name, entry in overrides.file.material_overrides.items():
            if name not in self._records:
                raise MaterialError(f"{overrides.path}: material_overrides.{name}: unknown material {name!r}")
            self._records[name] = _merge(name, self._records[name], overrides)
            for field, source in entry.sources().items():
                self._sources[(name, field)] = source

    @classmethod
    def from_files(cls, overrides_path: Path | None = None, data_dir: Path = DATA_DIR) -> MaterialSet:
        overrides = None
        if overrides_path is not None:
            overrides = load_overrides(overrides_path)
        return cls(load_shipped(data_dir), overrides)

    def material(self, name: str) -> Material:
        if name not in self._records:
            raise MaterialError(f"unknown material {name!r}")
        return self._records[name]

    def records(self) -> dict[str, Material]:
        """Every record with overrides applied, keyed by name (the builder's density source, MAT-020)."""
        return dict(self._records)

    def source(self, name: str, field: str) -> str:
        """The override's `source` for an overridden field, else the shipped record's `source`."""
        record = self.material(name)
        return self._sources.get((name, field), record.source)

    def lookup(self, name: str, property_name: str, temperature: float, spread: SpreadChoice = "typ") -> float:
        return lookup(self.material(name), property_name, temperature, spread)

    def strength(
        self,
        name: str,
        kind: StrengthKind,
        direction_body: Sequence[float],
        process: ProcessLike | None,
        temperature: float,
        spread: SpreadChoice = "typ",
    ) -> float:
        return strength(self.material(name), kind, direction_body, process, temperature, spread)

    def allowable_amplitude(self, name: str, cycles: float) -> float:
        return allowable_amplitude(self.material(name), cycles)

    def integrity_view(self, name: str) -> IntegrityMaterialView:
        return IntegrityMaterialView(self, name)


class IntegrityMaterialView:
    """What integrity checks read from one material (MAT-021): strengths with spreads, direction-dependent strengths,
    the S-N curve and its allowable amplitude, and where each value came from. Temperatures in kelvin, SI out."""

    def __init__(self, materials: MaterialSet, name: str) -> None:
        self.material = materials.material(name)
        self._materials = materials
        self.name = name

    def strength_spread(
        self,
        kind: StrengthKind,
        temperature: float,
        direction_body: Sequence[float] | None = None,
        process: ProcessLike | None = None,
    ) -> SpreadValues:
        """min, typ, max strength; with `direction_body` the direction-dependent value for `process` (None: the
        material's own process), without it the record's isotropic value."""
        values: list[float] = []
        spreads: tuple[SpreadChoice, ...] = ("min", "typ", "max")
        for spread in spreads:
            if direction_body is None:
                values.append(lookup(self.material, f"{kind}_strength", temperature, spread))
            else:
                values.append(strength(self.material, kind, direction_body, process, temperature, spread))
        return SpreadValues(values[0], values[1], values[2])

    def yield_strength(self, temperature: float) -> SpreadValues:
        return self.strength_spread("yield", temperature)

    def ultimate_strength(self, temperature: float) -> SpreadValues:
        return self.strength_spread("ultimate", temperature)

    def shear_strength(self, temperature: float) -> SpreadValues:
        return self.strength_spread("shear", temperature)

    def sn_curve(self) -> tuple[tuple[float, float], ...]:
        """(cycles, stress amplitude in Pa) points, cycles strictly increasing."""
        return tuple((point.cycles, point.stress_amplitude) for point in self.material.sn_curve)

    def allowable_amplitude(self, cycles: float) -> float:
        return allowable_amplitude(self.material, cycles)

    def source(self, field: str) -> str:
        return self._materials.source(self.name, field)
