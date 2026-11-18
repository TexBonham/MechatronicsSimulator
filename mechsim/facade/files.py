"""File kinds and validation behind the facade's validate, read and write operations (FAC-010, FAC-017, DES-036)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from pydantic import ValidationError

from mechsim.design.chip import validate_chip_file
from mechsim.design.schema import validate_design_file, yaml_loader
from mechsim.integrity.config import FILE_NAME as INTEGRITY_FILE_NAME
from mechsim.integrity.config import load_config
from mechsim.integrity.plugin import IntegrityError
from mechsim.library.schema import CatalogError, load_catalog_file
from mechsim.materials.database import MaterialSet
from mechsim.materials.schema import MaterialError, load_material_file
from mechsim.physics.neutral import from_yaml
from mechsim.schema_base import format_errors
from mechsim.testing.baseline import validate_baseline_file
from mechsim.testing.schema import ROOT, validate_suite_file, validate_test_case_file

MATERIALS_DIR = (ROOT / "mechsim" / "materials" / "data").resolve()
WRITABLE_KINDS = ("design", "case")


def kind_of_text(path: Path, text: str) -> str | None:
    """design, case, suite, catalog, chip, baseline, material, material overrides, physics model or integrity; None
    when it is none of them. Chips, materials, physics models and integrity configurations (`integrity.yaml`, INT-002)
    by location and name, the others by their top-level keys."""
    if path.name == "chip.yaml":
        return "chip"
    if path.name == INTEGRITY_FILE_NAME:
        return "integrity"
    if path.name.startswith("physics_model.") and path.suffix == ".yaml":
        return "physics_model"
    if path.resolve().parent == MATERIALS_DIR:
        return "material"
    data: Any = yaml_loader().load(text)
    if not isinstance(data, dict):
        return None
    keys = set(data)
    if "criteria" in keys:
        return "case"
    if {"suite", "entries"} <= keys:
        return "baseline"
    if {"cases", "baseline"} <= keys:
        return "suite"
    if {"types", "parts"} <= keys:
        return "catalog"
    if "material_overrides" in keys:
        return "material_overrides"
    if {"bodies", "joints"} <= keys:
        return "design"
    return None


def file_kind(path: Path) -> str | None:
    return kind_of_text(path, path.read_text(encoding="utf-8"))


def validate_file(path: Path, designs_root: Path) -> tuple[str | None, list[str]]:
    try:
        kind = file_kind(path)
    except Exception as error:
        return None, [f"{path}: YAML parse error: {error}"]
    if kind == "chip":
        return kind, validate_chip_file(path)
    if kind == "case":
        return kind, validate_test_case_file(path, designs_root)
    if kind == "baseline":
        return kind, validate_baseline_file(path)
    if kind == "suite":
        return kind, validate_suite_file(path)
    if kind == "design":
        return kind, validate_design_file(path)
    if kind == "material":
        try:
            load_material_file(path)
        except MaterialError as error:
            return kind, [str(error)]
        return kind, []
    if kind == "material_overrides":
        try:
            MaterialSet.from_files(path)
        except MaterialError as error:
            return kind, [str(error)]
        return kind, []
    if kind == "physics_model":
        try:
            from_yaml(path.read_text(encoding="utf-8"))
        except ValidationError as error:
            return kind, [f"{path}: {message}" for message in format_errors(error)]
        return kind, []
    if kind == "integrity":
        try:
            load_config(path)
        except IntegrityError as error:
            return kind, [str(error)]
        return kind, []
    if kind == "catalog":
        try:
            load_catalog_file(path)
        except CatalogError as error:
            return kind, [str(error)]
        return kind, []
    return None, [
        f"{path}: not a design, test case, suite, catalog, chip, baseline, material, physics model or integrity file"
    ]
