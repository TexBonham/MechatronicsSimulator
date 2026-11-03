"""Import-boundary checker for invariants 5 (engines behind adapters) and 9 (headless core).

Usage: python3 tools/check_imports.py [--root ROOT] [paths...]

Parses every .py file under the given paths (default: the repository) with ast and reports each import that crosses a
boundary in RULES as ``path:line: message``. Exits 1 when any violation is found, 0 otherwise. Standard library only.
"""

from __future__ import annotations

import argparse
import ast
import sys
from collections.abc import Iterable, Iterator, Sequence
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Rule:
    """A module family and where it may be imported.

    A module matches when its dotted name equals an entry of ``modules`` or starts with that entry plus a dot. A file
    location matches a prefix when its repository-relative POSIX path starts with it. An empty ``allowed_under`` means
    the module is allowed everywhere except under ``forbidden_under``.
    """

    modules: tuple[str, ...]
    allowed_under: tuple[str, ...]
    forbidden_under: tuple[str, ...]
    reason: str


RULES: tuple[Rule, ...] = (
    Rule(
        modules=("mujoco",),
        allowed_under=("mechsim/physics/mujoco/",),
        forbidden_under=(),
        reason="MuJoCo stays behind its adapter (invariant 5)",
    ),
    Rule(
        modules=("PySide6", "PySide2", "PyQt5", "PyQt6", "OpenGL", "pyqtgraph", "tkinter", "vispy", "glfw", "pygame"),
        allowed_under=("mechsim/viewer/",),
        forbidden_under=(),
        reason="the core is headless; Qt and display modules belong to the viewer (invariant 9)",
    ),
    Rule(
        modules=("hppfcl", "fcl", "pycollision"),
        allowed_under=("tests/",),
        forbidden_under=(),
        reason="reference collision libraries are test-only differential oracles (R-05)",
    ),
    Rule(
        modules=("mechsim.viewer",),
        allowed_under=("mechsim/viewer/", "tests/"),
        forbidden_under=(),
        reason="only the viewer and the tests depend on the viewer (invariant 9)",
    ),
    Rule(
        modules=("tests",),
        allowed_under=(),
        forbidden_under=("mechsim/",),
        reason="the mechsim package never depends on its test suite",
    ),
)

SKIPPED_DIRS: frozenset[str] = frozenset({"build", "node_modules", "__pycache__"})

DYNAMIC_IMPORT_FUNCTIONS: frozenset[str] = frozenset({"__import__", "import_module"})


@dataclass(frozen=True, order=True)
class Violation:
    path: str
    line: int
    message: str

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: {self.message}"


def module_matches(name: str, module: str) -> bool:
    return name == module or name.startswith(module + ".")


def location_matches(location: str, prefixes: Iterable[str]) -> bool:
    return any(location.startswith(prefix) for prefix in prefixes)


def is_allowed(rule: Rule, location: str) -> bool:
    if location_matches(location, rule.forbidden_under):
        return False
    if not rule.allowed_under:
        return True
    return location_matches(location, rule.allowed_under)


def describe(rule: Rule) -> str:
    if rule.allowed_under:
        where = "allowed only under " + ", ".join(rule.allowed_under)
    else:
        where = "forbidden under " + ", ".join(rule.forbidden_under)
    return f"{where}: {rule.reason}"


def broken_rules(name: str, location: str, rules: Sequence[Rule]) -> list[Rule]:
    return [
        rule
        for rule in rules
        if any(module_matches(name, module) for module in rule.modules) and not is_allowed(rule, location)
    ]


def dynamic_import_target(node: ast.Call) -> str | None:
    func = node.func
    if isinstance(func, ast.Name):
        func_name = func.id
    elif isinstance(func, ast.Attribute):
        func_name = func.attr
    else:
        return None
    if func_name not in DYNAMIC_IMPORT_FUNCTIONS or not node.args:
        return None
    first = node.args[0]
    if not isinstance(first, ast.Constant) or not isinstance(first.value, str):
        return None
    if first.value.startswith("."):
        return None
    return first.value


def imported_names(node: ast.AST) -> Iterator[tuple[str, list[str]]]:
    """Yield (reported name, candidate dotted names) for each absolute module an import node pulls in."""
    if isinstance(node, ast.Import):
        for alias in node.names:
            yield alias.name, [alias.name]
    elif isinstance(node, ast.ImportFrom):
        if node.level > 0 or node.module is None:
            return
        yield node.module, [node.module] + [f"{node.module}.{alias.name}" for alias in node.names if alias.name != "*"]
    elif isinstance(node, ast.Call):
        target = dynamic_import_target(node)
        if target is not None:
            yield target, [target]


def check_source(source: str | bytes, location: str, rules: Sequence[Rule] = RULES) -> list[Violation]:
    try:
        tree = ast.parse(source, filename=location)
    except (SyntaxError, ValueError) as error:
        line = 0
        if isinstance(error, SyntaxError) and error.lineno is not None:
            line = error.lineno
        return [Violation(location, line, f"cannot parse file: {error}")]
    violations: set[Violation] = set()
    for node in ast.walk(tree):
        line = getattr(node, "lineno", 0)
        for reported, candidates in imported_names(node):
            for candidate in candidates:
                for rule in broken_rules(candidate, location, rules):
                    shown = reported
                    if not any(module_matches(reported, module) for module in rule.modules):
                        shown = candidate
                    violations.add(Violation(location, line, f"import of '{shown}' {describe(rule)}"))
    return sorted(violations)


def relative_location(path: Path, root: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(root.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def iter_python_files(path: Path) -> Iterator[Path]:
    if path.is_file():
        if path.suffix == ".py":
            yield path
        return
    for child in sorted(path.iterdir()):
        if child.is_dir():
            if child.name.startswith(".") or child.name in SKIPPED_DIRS:
                continue
            yield from iter_python_files(child)
        elif child.suffix == ".py":
            yield child


def check_file(path: Path, root: Path, rules: Sequence[Rule] = RULES) -> list[Violation]:
    location = relative_location(path, root)
    try:
        source = path.read_bytes()
    except OSError as error:
        return [Violation(location, 0, f"cannot read file: {error.strerror}")]
    return check_source(source, location, rules)


def check_paths(paths: Iterable[Path], root: Path, rules: Sequence[Rule] = RULES) -> list[Violation]:
    violations: list[Violation] = []
    for path in paths:
        if not path.exists():
            violations.append(Violation(relative_location(path, root), 0, "no such file or directory"))
            continue
        for file in iter_python_files(path):
            violations.extend(check_file(file, root, rules))
    return sorted(violations)


def default_root() -> Path:
    return Path(__file__).resolve().parent.parent


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check import boundaries (invariants 5 and 9).")
    parser.add_argument("paths", nargs="*", type=Path, help="files or directories to check (default: the repository)")
    parser.add_argument("--root", type=Path, default=default_root(), help="repository root that rule paths refer to")
    args = parser.parse_args(argv)
    root: Path = args.root
    paths: list[Path] = args.paths
    if not paths:
        paths = [root]
    violations = check_paths(paths, root)
    for violation in violations:
        print(violation)
    if violations:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
