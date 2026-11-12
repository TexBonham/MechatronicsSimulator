"""One registry of every shipped component type and catalog part (LIB), the lookup design files resolve against.

Catalog files under `mechsim/library/catalog/` each validate on their own; the registry merges them and rejects a type
or part name declared in two files, so a design reference names exactly one thing.
"""

from __future__ import annotations

import functools
from pathlib import Path

from pydantic import ValidationError

from mechsim.library.schema import (
    CATALOG_DIR,
    CatalogPart,
    ComponentType,
    FlagParameter,
    PartEntry,
    QuantityParameter,
    load_catalog_file,
)
from mechsim.units import UnitError, parse_quantity


class RegistryError(ValueError):
    """The catalog files could not be merged into one registry, or a lookup failed."""


class Registry:
    """Component types and resolved catalog parts by name, merged from every catalog file in a directory."""

    def __init__(
        self,
        types: dict[str, ComponentType],
        entries: dict[str, PartEntry],
        parts: dict[str, CatalogPart],
    ) -> None:
        self._types = dict(types)
        self._entries = dict(entries)
        self._parts = dict(parts)

    def type_names(self) -> list[str]:
        return sorted(self._types)

    def part_names(self) -> list[str]:
        return sorted(self._parts)

    def has_type(self, name: str) -> bool:
        return name in self._types

    def has_part(self, name: str) -> bool:
        return name in self._parts

    def component_type(self, name: str) -> ComponentType:
        if name not in self._types:
            raise RegistryError(f"no component type {name!r} in the catalog")
        return self._types[name]

    def part(self, name: str) -> CatalogPart:
        if name not in self._parts:
            raise RegistryError(f"no catalog part {name!r} in the catalog")
        return self._parts[name]

    def resolve_part(self, name: str, overrides: dict[str, str | bool] | None = None) -> CatalogPart:
        """The part `name` resolved against its type with `overrides` (unit strings or flags) replacing its values."""
        if name not in self._entries:
            raise RegistryError(f"no catalog part {name!r} in the catalog")
        entry = self._entries[name]
        try:
            return CatalogPart.resolve(entry, self._types[entry.type], overrides)
        except ValidationError as error:
            messages = "; ".join(str(item["msg"]) for item in error.errors())
            raise RegistryError(f"part {name!r}: {messages}") from error

    def parameter_override(self, type_name: str, parameter: str, value: str) -> str | bool:
        """`value` checked as one run's override of `parameter` of `type_name` (LIB-016): a quantity's unit string
        must have the parameter's dimension; a flag takes `true` or `false`. Returns the value to store."""
        component = self.component_type(type_name)
        declared = {item.name: item for item in component.parameters}
        if parameter not in declared:
            raise RegistryError(f"component type {type_name!r} has no parameter {parameter!r}")
        definition = declared[parameter]
        if isinstance(definition, FlagParameter):
            if value not in ("true", "false"):
                raise RegistryError(f"{type_name}.{parameter} is a flag: give true or false, got {value!r}")
            return value == "true"
        if not isinstance(definition, QuantityParameter):
            raise RegistryError(f"{type_name}.{parameter}: unsupported parameter kind")
        try:
            parse_quantity(value, definition.dimension)
        except UnitError as error:
            raise RegistryError(f"{type_name}.{parameter} ({definition.dimension}): {error}") from error
        return value

    def substitute_part(self, type_name: str, part: str) -> CatalogPart:
        """The catalog part replacing an instance's part (LIB-017): it must exist and be of `type_name`."""
        if part not in self._parts:
            raise RegistryError(f"no catalog part {part!r} in the catalog (instance type {type_name!r})")
        found = self._parts[part]
        if found.type != type_name:
            raise RegistryError(f"part {part!r} is a {found.type!r}, not a {type_name!r}")
        return found


def load_registry(directory: Path = CATALOG_DIR) -> Registry:
    """Load every `*.yaml` catalog file in `directory` into one registry; a duplicate type or part name raises."""
    files = sorted(directory.glob("*.yaml"))
    if len(files) == 0:
        raise RegistryError(f"{directory}: no catalog files")
    types: dict[str, ComponentType] = {}
    type_files: dict[str, Path] = {}
    entries: dict[str, PartEntry] = {}
    parts: dict[str, CatalogPart] = {}
    part_files: dict[str, Path] = {}
    for path in files:
        catalog = load_catalog_file(path)
        for component in catalog.types:
            if component.name in types:
                raise RegistryError(
                    f"duplicate component type {component.name!r} in {type_files[component.name]} and {path}"
                )
            types[component.name] = component
            type_files[component.name] = path
        for entry, resolved in zip(catalog.parts, catalog.resolved_parts(), strict=True):
            if entry.name in entries:
                raise RegistryError(f"duplicate catalog part {entry.name!r} in {part_files[entry.name]} and {path}")
            entries[entry.name] = entry
            parts[entry.name] = resolved
            part_files[entry.name] = path
    return Registry(types, entries, parts)


@functools.cache
def default_registry() -> Registry:
    """The shipped catalog, loaded once per process."""
    return load_registry(CATALOG_DIR)
