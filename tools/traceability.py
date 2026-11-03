"""Requirement traceability checker and matrix generator (ADR 0008).

Usage: python3 tools/traceability.py [--write] [--root PATH]

Collects requirement IDs from the ``## Requirements`` tables of ``docs/spec/*.md``, Python citations
(``pytest.mark.req("ID")``) from every ``.py`` under ``tests/``, and C++ citations (Catch2 ``TEST_CASE``/``SCENARIO``
tags) from every ``.cpp``/``.hpp`` under ``core/`` and ``firmware/``. Reports violations as
``path:line: message`` and compares the
generated matrix with the committed ``docs/spec/traceability.md``.

Exit codes: 0 clean; 1 violations, or (without ``--write``) a stale matrix; 2 usage error. Standard library only.
"""

from __future__ import annotations

import argparse
import ast
import difflib
import re
import sys
from collections.abc import Iterator, Sequence
from dataclasses import dataclass, field
from pathlib import Path

ID_PATTERN = re.compile(r"^[A-Z]{2,5}-\d{3}$")
LOOKS_LIKE_ID = re.compile(r"^(?:[A-Za-z]+[-_]\d+[A-Za-z]?|[A-Z]{2,5}\d{3})$")
TAG_PATTERN = re.compile(r"\[([^\[\]]*)\]")
CPP_MACROS: frozenset[str] = frozenset({"TEST_CASE", "SCENARIO"})
CPP_SUFFIXES: frozenset[str] = frozenset({".cpp", ".hpp"})
SKIPPED_DIRS: frozenset[str] = frozenset({"build", "node_modules", "__pycache__"})
SPEC_DIR = "docs/spec"
MATRIX_NAME = "traceability.md"
NON_SPEC_FILES: frozenset[str] = frozenset({"0000-template.md", "README.md", MATRIX_NAME})
REQUIREMENTS_HEADING = "## Requirements"
REQUIREMENTS_COLUMNS: tuple[str, ...] = ("ID", "Requirement", "Source", "Milestone", "Tier", "Tolerance")
WITHDRAWN_PREFIX = "Withdrawn:"
REGENERATE_COMMAND = "python3 tools/traceability.py --write"


# ===== Data model ===========================================================


@dataclass(frozen=True, order=True)
class Violation:
    path: str
    line: int
    message: str

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: {self.message}"


@dataclass(frozen=True, order=True)
class Requirement:
    req_id: str
    path: str
    line: int
    text: str
    milestone: str
    tier: str

    @property
    def withdrawn(self) -> bool:
        return self.text.startswith(WITHDRAWN_PREFIX)

    @property
    def prefix(self) -> str:
        return self.req_id.split("-", 1)[0]


@dataclass(frozen=True, order=True)
class Citation:
    req_id: str
    path: str
    line: int

    @property
    def location(self) -> str:
        return f"{self.path}:{self.line}"


@dataclass
class Collection:
    requirements: list[Requirement] = field(default_factory=list)
    citations: list[Citation] = field(default_factory=list)
    violations: list[Violation] = field(default_factory=list)


# ===== File helpers =========================================================


def normalize_newlines(text: str) -> str:
    return text.replace("\r\n", "\n").replace("\r", "\n")


def relative_location(path: Path, root: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(root.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def iter_files(directory: Path, suffixes: frozenset[str]) -> Iterator[Path]:
    if not directory.is_dir():
        return
    for child in sorted(directory.iterdir()):
        if child.is_dir():
            if child.name.startswith(".") or child.name in SKIPPED_DIRS:
                continue
            yield from iter_files(child, suffixes)
        elif child.suffix in suffixes:
            yield child


def read_text(path: Path, location: str, violations: list[Violation]) -> str | None:
    try:
        raw = path.read_bytes()
    except OSError as error:
        violations.append(Violation(location, 0, f"cannot read file: {error.strerror}"))
        return None
    try:
        return normalize_newlines(raw.decode("utf-8-sig"))
    except UnicodeDecodeError as error:
        violations.append(Violation(location, 0, f"cannot decode file as UTF-8: {error.reason}"))
        return None


# ===== Spec parsing =========================================================


def split_row(line: str) -> list[str]:
    body = line.strip()
    if body.startswith("|"):
        body = body[1:]
    if body.endswith("|") and not body.endswith("\\|"):
        body = body[:-1]
    cells: list[str] = []
    current: list[str] = []
    index = 0
    while index < len(body):
        char = body[index]
        if char == "\\" and index + 1 < len(body) and body[index + 1] == "|":
            current.append("|")
            index += 2
            continue
        if char == "|":
            cells.append("".join(current).strip())
            current = []
        else:
            current.append(char)
        index += 1
    cells.append("".join(current).strip())
    return cells


def is_separator_row(cells: Sequence[str]) -> bool:
    return all(re.fullmatch(r":?-{3,}:?", cell) is not None for cell in cells)


def requirement_sections(lines: Sequence[str]) -> Iterator[tuple[int, list[tuple[int, str]]]]:
    """Yield (heading line, [(line number, table line)]) for each ``## Requirements`` section outside code fences."""
    in_fence = False
    heading_line = 0
    rows: list[tuple[int, str]] = []
    in_section = False
    for number, line in enumerate(lines, start=1):
        stripped = line.strip()
        if stripped.startswith("```") or stripped.startswith("~~~"):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        if re.match(r"^#{1,2}\s", line):
            if in_section:
                yield heading_line, rows
            in_section = line.rstrip() == REQUIREMENTS_HEADING
            heading_line = number
            rows = []
            continue
        if in_section and stripped.startswith("|"):
            rows.append((number, stripped))
    if in_section:
        yield heading_line, rows


def parse_spec(text: str, location: str) -> tuple[list[Requirement], list[Violation]]:
    requirements: list[Requirement] = []
    violations: list[Violation] = []
    for heading_line, rows in requirement_sections(normalize_newlines(text).split("\n")):
        if not rows:
            violations.append(Violation(location, heading_line, "'## Requirements' heading has no table rows"))
            continue
        header_line, header_text = rows[0]
        header = split_row(header_text)
        if tuple(header) != REQUIREMENTS_COLUMNS:
            expected = " | ".join(REQUIREMENTS_COLUMNS)
            violations.append(Violation(location, header_line, f"requirements table header must be '{expected}'"))
            continue
        data = [(number, split_row(row)) for number, row in rows[1:]]
        if data and is_separator_row(data[0][1]):
            data = data[1:]
        if not data:
            violations.append(Violation(location, heading_line, "'## Requirements' table has no requirement rows"))
            continue
        for number, cells in data:
            if len(cells) != len(REQUIREMENTS_COLUMNS):
                message = f"requirements row has {len(cells)} cells, expected {len(REQUIREMENTS_COLUMNS)}"
                violations.append(Violation(location, number, message))
                continue
            req_id = cells[0]
            if ID_PATTERN.fullmatch(req_id) is None:
                violations.append(Violation(location, number, f"requirement ID '{req_id}' does not match the pattern"))
                continue
            requirements.append(Requirement(req_id, location, number, cells[1], cells[3], cells[4]))
    return requirements, violations


def spec_files(root: Path) -> list[Path]:
    directory = root / SPEC_DIR
    if not directory.is_dir():
        return []
    return [path for path in sorted(directory.glob("*.md")) if path.is_file() and path.name not in NON_SPEC_FILES]


def collect_requirements(root: Path) -> tuple[list[Requirement], list[Violation]]:
    requirements: list[Requirement] = []
    violations: list[Violation] = []
    for path in spec_files(root):
        location = relative_location(path, root)
        text = read_text(path, location, violations)
        if text is None:
            continue
        found, problems = parse_spec(text, location)
        requirements.extend(found)
        violations.extend(problems)
    return requirements, violations


# ===== Python citations =====================================================


def is_req_marker(node: ast.expr) -> bool:
    """True for ``pytest.mark.req`` or ``mark.req`` (the attribute itself, not a call of it)."""
    if not isinstance(node, ast.Attribute) or node.attr != "req":
        return False
    owner = node.value
    if isinstance(owner, ast.Name):
        return owner.id == "mark"
    if isinstance(owner, ast.Attribute) and owner.attr == "mark":
        return isinstance(owner.value, ast.Name) and owner.value.id == "pytest"
    return False


def marker_ids(node: ast.expr, location: str, violations: list[Violation]) -> list[str] | None:
    """IDs a req marker expression cites, or None when the expression is not a req marker."""
    if isinstance(node, ast.Call) and is_req_marker(node.func):
        if not node.args or node.keywords:
            violations.append(Violation(location, node.lineno, "req marker takes one or more positional string IDs"))
        ids: list[str] = []
        for argument in node.args:
            if not isinstance(argument, ast.Constant) or not isinstance(argument.value, str):
                violations.append(Violation(location, argument.lineno, "req marker argument is not a string literal"))
                continue
            value = argument.value
            if ID_PATTERN.fullmatch(value) is None:
                violations.append(Violation(location, argument.lineno, f"req marker ID '{value}' fails the pattern"))
                continue
            ids.append(value)
        return ids
    if is_req_marker(node):
        violations.append(Violation(location, node.lineno, "req marker used without an ID"))
        return []
    return None


def collected_methods(node: ast.ClassDef) -> Iterator[ast.FunctionDef | ast.AsyncFunctionDef]:
    for child in node.body:
        if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)) and child.name.startswith("test"):
            yield child
        elif isinstance(child, ast.ClassDef):
            yield from collected_methods(child)


def python_citations(source: str | bytes, location: str) -> tuple[list[Citation], list[Violation]]:
    violations: list[Violation] = []
    try:
        tree = ast.parse(source, filename=location)
    except (SyntaxError, ValueError) as error:
        line = 0
        if isinstance(error, SyntaxError) and error.lineno is not None:
            line = error.lineno
        return [], [Violation(location, line, f"cannot parse file: {error}")]
    citations: set[Citation] = set()
    decorators: set[int] = set()
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        for decorator in node.decorator_list:
            ids = marker_ids(decorator, location, violations)
            if ids is None:
                continue
            decorators.add(id(decorator))
            if isinstance(node, ast.ClassDef):
                for method in collected_methods(node):
                    citations.update(Citation(req_id, location, method.lineno) for req_id in ids)
            else:
                citations.update(Citation(req_id, location, decorator.lineno) for req_id in ids)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or id(node) in decorators:
            continue
        ids = marker_ids(node, location, violations)
        if ids is not None:
            citations.update(Citation(req_id, location, node.lineno) for req_id in ids)
    return sorted(citations), sorted(set(violations))


def collect_python_citations(root: Path) -> tuple[list[Citation], list[Violation]]:
    citations: list[Citation] = []
    violations: list[Violation] = []
    for path in iter_files(root / "tests", frozenset({".py"})):
        location = relative_location(path, root)
        try:
            source = path.read_bytes()
        except OSError as error:
            violations.append(Violation(location, 0, f"cannot read file: {error.strerror}"))
            continue
        found, problems = python_citations(source, location)
        citations.extend(found)
        violations.extend(problems)
    return citations, violations


# ===== C++ citations ========================================================


@dataclass(frozen=True)
class Token:
    kind: str
    value: str
    line: int


def cpp_tokens(text: str) -> Iterator[Token]:
    """Identifiers, string literals (escapes resolved) and single punctuation characters; comments are dropped."""
    line = 1
    index = 0
    length = len(text)
    while index < length:
        char = text[index]
        if char == "\n":
            line += 1
            index += 1
        elif char.isspace():
            index += 1
        elif text.startswith("//", index):
            end = text.find("\n", index)
            if end < 0:
                end = length
            index = end
        elif text.startswith("/*", index):
            end = text.find("*/", index + 2)
            if end < 0:
                end = length
            else:
                end += 2
            line += text.count("\n", index, end)
            index = end
        elif char == "R" and text.startswith('R"', index):
            open_paren = text.find("(", index + 2)
            if open_paren < 0:
                yield Token("ident", "R", line)
                index += 1
                continue
            delimiter = text[index + 2 : open_paren]
            closing = ")" + delimiter + '"'
            end = text.find(closing, open_paren + 1)
            if end < 0:
                end = length
            yield Token("string", text[open_paren + 1 : end], line)
            stop = min(end + len(closing), length)
            line += text.count("\n", index, stop)
            index = stop
        elif char.isalpha() or char == "_":
            start = index
            while index < length and (text[index].isalnum() or text[index] == "_"):
                index += 1
            yield Token("ident", text[start:index], line)
        elif char == '"' or char == "'":
            start_line = line
            index += 1
            value: list[str] = []
            while index < length and text[index] != char and text[index] != "\n":
                if text[index] == "\\" and index + 1 < length:
                    if text[index + 1] == "\n":
                        line += 1
                    else:
                        value.append(text[index + 1])
                    index += 2
                    continue
                value.append(text[index])
                index += 1
            if index < length and text[index] == char:
                index += 1
            if char == '"':
                yield Token("string", "".join(value), start_line)
        else:
            yield Token("punct", char, line)
            index += 1


def macro_arguments(tokens: Sequence[Token], start: int) -> list[str]:
    """Concatenated string-literal text of each top-level argument of the call whose '(' is at ``start``."""
    arguments: list[list[str]] = [[]]
    depth = 0
    for token in tokens[start:]:
        if token.kind == "punct" and token.value in "([{":
            depth += 1
            if depth == 1:
                continue
        elif token.kind == "punct" and token.value in ")]}":
            depth -= 1
            if depth == 0:
                break
        elif token.kind == "punct" and token.value == "," and depth == 1:
            arguments.append([])
            continue
        if token.kind == "string" and depth == 1:
            arguments[-1].append(token.value)
    return ["".join(parts) for parts in arguments]


def cpp_citations(text: str, location: str) -> tuple[list[Citation], list[Violation]]:
    tokens = list(cpp_tokens(normalize_newlines(text)))
    citations: set[Citation] = set()
    violations: set[Violation] = set()
    for index, token in enumerate(tokens):
        if token.kind != "ident" or token.value not in CPP_MACROS:
            continue
        if index + 1 >= len(tokens) or tokens[index + 1].kind != "punct" or tokens[index + 1].value != "(":
            continue
        arguments = macro_arguments(tokens, index + 1)
        if len(arguments) < 2:
            continue
        for tag in TAG_PATTERN.findall(arguments[1]):
            if ID_PATTERN.fullmatch(tag) is not None:
                citations.add(Citation(tag, location, token.line))
            elif LOOKS_LIKE_ID.fullmatch(tag) is not None:
                violations.add(
                    Violation(location, token.line, f"Catch2 tag '[{tag}]' looks like an ID but fails the pattern")
                )
    return sorted(citations), sorted(violations)


CPP_SCAN_ROOTS = ("core", "firmware")


def collect_cpp_citations(root: Path) -> tuple[list[Citation], list[Violation]]:
    citations: list[Citation] = []
    violations: list[Violation] = []
    for scan_root in CPP_SCAN_ROOTS:
        for path in iter_files(root / scan_root, CPP_SUFFIXES):
            location = relative_location(path, root)
            text = read_text(path, location, violations)
            if text is None:
                continue
            found, problems = cpp_citations(text, location)
            citations.extend(found)
            violations.extend(problems)
    return citations, violations


# ===== Checks ===============================================================


def collect(root: Path) -> Collection:
    requirements, spec_violations = collect_requirements(root)
    python_found, python_violations = collect_python_citations(root)
    cpp_found, cpp_violations = collect_cpp_citations(root)
    collection = Collection(
        requirements=sorted(requirements),
        citations=sorted(python_found + cpp_found),
        violations=spec_violations + python_violations + cpp_violations,
    )
    collection.violations.extend(cross_check(collection.requirements, collection.citations))
    collection.violations = sorted(set(collection.violations))
    return collection


def unique_requirements(requirements: Sequence[Requirement]) -> dict[str, Requirement]:
    first: dict[str, Requirement] = {}
    for requirement in sorted(requirements, key=lambda item: (item.path, item.line)):
        if requirement.req_id not in first:
            first[requirement.req_id] = requirement
    return first


def cross_check(requirements: Sequence[Requirement], citations: Sequence[Citation]) -> list[Violation]:
    violations: list[Violation] = []
    first = unique_requirements(requirements)
    for requirement in requirements:
        original = first[requirement.req_id]
        if original is not requirement:
            message = f"requirement {requirement.req_id} is also defined at {original.path}:{original.line}"
            violations.append(Violation(requirement.path, requirement.line, message))
    cited = {citation.req_id for citation in citations}
    for requirement in first.values():
        if not requirement.withdrawn and requirement.req_id not in cited:
            message = f"requirement {requirement.req_id} has no test citing it"
            violations.append(Violation(requirement.path, requirement.line, message))
    for citation in citations:
        if citation.req_id not in first:
            message = f"test cites {citation.req_id}, which no spec defines"
            violations.append(Violation(citation.path, citation.line, message))
    return violations


# ===== Matrix ===============================================================


def escape_cell(text: str) -> str:
    return text.replace("|", "\\|")


def render_matrix(requirements: Sequence[Requirement], citations: Sequence[Citation]) -> str:
    first = unique_requirements(requirements)
    by_prefix: dict[str, list[Requirement]] = {}
    for requirement in first.values():
        by_prefix.setdefault(requirement.prefix, []).append(requirement)
    locations: dict[str, list[str]] = {}
    for citation in sorted(set(citations)):
        locations.setdefault(citation.req_id, []).append(citation.location)
    lines = [
        "# Traceability matrix",
        "",
        f"Generated by `tools/traceability.py`; do not edit by hand. Regenerate with `{REGENERATE_COMMAND}`.",
        "",
    ]
    summary: list[tuple[str, int, int, int, int]] = []
    for prefix in sorted(by_prefix):
        lines.extend([f"## {prefix}", "", "| ID | Requirement | Milestone | Tier | Tests |", "|---|---|---|---|---|"])
        covered = 0
        uncovered = 0
        withdrawn = 0
        for requirement in sorted(by_prefix[prefix], key=lambda item: item.req_id):
            tests = "none"
            if requirement.req_id in locations:
                tests = ", ".join(locations[requirement.req_id])
            if requirement.withdrawn:
                withdrawn += 1
            elif requirement.req_id in locations:
                covered += 1
            else:
                uncovered += 1
            cells = [requirement.req_id, requirement.text, requirement.milestone, requirement.tier, tests]
            lines.append("| " + " | ".join(escape_cell(cell) for cell in cells) + " |")
        lines.append("")
        summary.append((prefix, len(by_prefix[prefix]), covered, uncovered, withdrawn))
    lines.extend(
        [
            "## Summary",
            "",
            "| Prefix | Requirements | Covered | Uncovered | Withdrawn |",
            "|---|---|---|---|---|",
        ]
    )
    for prefix, total, covered, uncovered, withdrawn in summary:
        lines.append(f"| {prefix} | {total} | {covered} | {uncovered} | {withdrawn} |")
    totals = [
        sum(total for _, total, _, _, _ in summary),
        sum(covered for _, _, covered, _, _ in summary),
        sum(uncovered for _, _, _, uncovered, _ in summary),
        sum(withdrawn for _, _, _, _, withdrawn in summary),
    ]
    lines.append("| Total | " + " | ".join(str(value) for value in totals) + " |")
    return "\n".join(lines) + "\n"


def matrix_path(root: Path) -> Path:
    return root / SPEC_DIR / MATRIX_NAME


def matrix_diff(committed: str | None, generated: str) -> str:
    before: list[str] = []
    if committed is not None:
        before = normalize_newlines(committed).splitlines(keepends=True)
    after = generated.splitlines(keepends=True)
    name = f"{SPEC_DIR}/{MATRIX_NAME}"
    return "".join(difflib.unified_diff(before, after, fromfile=f"{name} (committed)", tofile=f"{name} (generated)"))


# ===== CLI ==================================================================


def default_root() -> Path:
    return Path(__file__).resolve().parent.parent


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check requirement traceability and regenerate the matrix.")
    parser.add_argument("--write", action="store_true", help=f"write {SPEC_DIR}/{MATRIX_NAME} instead of comparing")
    parser.add_argument("--root", type=Path, default=default_root(), help="repository root")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exit_request:
        if isinstance(exit_request.code, int):
            return exit_request.code
        return 2
    root: Path = args.root
    write: bool = args.write
    if not root.is_dir():
        print(f"error: --root {root} is not a directory", file=sys.stderr)
        return 2
    collection = collect(root)
    for violation in collection.violations:
        print(violation)
    generated = render_matrix(collection.requirements, collection.citations)
    target = matrix_path(root)
    stale = False
    if write:
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(generated, encoding="utf-8", newline="\n")
    else:
        committed: str | None = None
        if target.is_file():
            committed = target.read_text(encoding="utf-8")
        if committed is None or normalize_newlines(committed) != generated:
            stale = True
            print(f"{SPEC_DIR}/{MATRIX_NAME} is stale; regenerate with `{REGENERATE_COMMAND}`")
            print(matrix_diff(committed, generated), end="")
    if collection.violations or stale:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
