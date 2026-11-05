"""Import shim for the compiled core runtime, ``mechsim._core_ext`` (ADR 0005: one binding for the whole core).

Everything the binding exposes is reachable as an attribute of this module, e.g. ``mechsim._core.make_kernel``.
The extension is built by ``uv sync`` through scikit-build-core from ``core/`` (``core/bindings/CLAUDE.md``).
"""

import importlib
from types import ModuleType
from typing import Any

EXTENSION = "mechsim._core_ext"

BUILD_HINT = (
    "the compiled core runtime is not built: run `uv sync` from the repository root (needs CMake >= 3.28, Ninja or "
    "Make and a C++20 compiler); after changing C++ sources run `uv sync --reinstall-package mechsim`"
)


class CoreNotBuiltError(ImportError):
    """The compiled extension could not be imported."""


def load_extension(name: str = EXTENSION) -> ModuleType:
    """Import the compiled extension, or raise CoreNotBuiltError explaining how to build it."""
    try:
        return importlib.import_module(name)
    except ImportError as error:
        raise CoreNotBuiltError(f"cannot import {name}: {BUILD_HINT}", name=name) from error


extension: ModuleType = load_extension()


ENGINE_SUBMODULE = "engine"


def __getattr__(name: str) -> Any:
    if name == ENGINE_SUBMODULE:
        raise AttributeError(
            "the own engine is not re-exported here (invariant 5): import mechsim.physics.native._engine, "
            "allowed only under mechsim/physics/native/ and tests/"
        )
    return getattr(extension, name)


def __dir__() -> list[str]:
    return sorted((set(globals()) | set(dir(extension))) - {ENGINE_SUBMODULE})
