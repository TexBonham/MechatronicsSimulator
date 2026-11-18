"""Check registry (INT-001): every module in `mechsim.integrity.checks` that defines `CHECK`, plus the entry point
group `mechsim.integrity_checks`, so a new check is a module (or a distribution's entry point) and nothing else."""

from __future__ import annotations

import importlib
import pkgutil
from importlib.metadata import entry_points

from mechsim.integrity import checks
from mechsim.integrity.plugin import IntegrityCheck, IntegrityError

ENTRY_POINT_GROUP = "mechsim.integrity_checks"


def _add(found: dict[str, IntegrityCheck], candidate: object, origin: str) -> None:
    if not isinstance(candidate, type) or not issubclass(candidate, IntegrityCheck):
        raise IntegrityError(f"{origin}: CHECK is not an IntegrityCheck subclass")
    instance = candidate()
    if instance.name in found:
        raise IntegrityError(f"{origin}: duplicate integrity check name {instance.name!r}")
    found[instance.name] = instance


def discover() -> dict[str, IntegrityCheck]:
    """Every registered check by name, in name order."""
    found: dict[str, IntegrityCheck] = {}
    for module_info in sorted(pkgutil.iter_modules(checks.__path__), key=lambda info: info.name):
        if module_info.name.startswith("_"):
            continue
        module = importlib.import_module(f"{checks.__name__}.{module_info.name}")
        if hasattr(module, "CHECK"):
            _add(found, module.CHECK, module.__name__)
    for point in sorted(entry_points(group=ENTRY_POINT_GROUP), key=lambda item: item.name):
        _add(found, point.load(), f"entry point {point.name}")
    return dict(sorted(found.items()))
