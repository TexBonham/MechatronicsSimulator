"""The own engine's binding, ``mechsim._core_ext.engine`` (ADR 0001), re-exported for the native adapter only.

Invariant 5: engine types stay behind ``mechsim/physics/native/``; tools/check_imports.py rejects this import
anywhere else except tests.
"""

from types import ModuleType
from typing import Any

from mechsim._core import load_extension

engine: ModuleType = load_extension("mechsim._core_ext.engine")


def __getattr__(name: str) -> Any:
    return getattr(engine, name)


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(dir(engine)))
