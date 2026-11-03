#!/usr/bin/env python3
"""PostToolUse hook: parse an edited data YAML file under designs/, test_cases/, mechsim/library/ or chips/
and validate it against schemas/<kind>.json. Expects the Claude Code hook JSON on stdin, shaped like
    {"tool_name": "Write", "tool_input": {"file_path": "/abs/or/relative/designs/x.yaml"}, "cwd": "..."}
Exit 2 on a parse or schema error so the model sees it; exit 0 in every other case."""

import json
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

KINDS = (("designs/", "design"), ("test_cases/", "test_case"), ("mechsim/library/", "component"), ("chips/", "chip"))


def edited_path(raw_stdin: str) -> Path | None:
    try:
        payload = json.loads(raw_stdin)
    except ValueError:
        return None
    if not isinstance(payload, dict) or not isinstance(payload.get("tool_input"), dict):
        return None
    raw = payload["tool_input"].get("file_path")
    if not isinstance(raw, str) or raw.strip() == "":
        return None
    path = Path(raw)
    cwd = payload.get("cwd")
    if not path.is_absolute() and isinstance(cwd, str) and cwd != "":
        path = Path(cwd) / path
    return path.resolve()


def repo_root(path: Path) -> Path:
    for candidate in path.parents:
        if (candidate / ".git").exists():
            return candidate
    return Path(__file__).resolve().parents[2]


def kind_for(relative: str) -> str | None:
    for prefix, kind in KINDS:
        if relative.startswith(prefix):
            return kind
    return None


def yaml_loader() -> Callable[[str], Any] | None:
    try:
        from ruamel.yaml import YAML

        return YAML(typ="safe", pure=True).load
    except ImportError:
        pass
    try:
        import yaml

        return yaml.safe_load
    except ImportError:
        return None


def schema_errors(schema_path: Path, data: Any) -> list[str] | None:
    if not schema_path.is_file():
        return None
    try:
        import jsonschema
    except ImportError:
        return None
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    validator = jsonschema.validators.validator_for(schema)(schema)
    lines = []
    for error in validator.iter_errors(data):
        location = "/".join(str(part) for part in error.absolute_path)
        if location == "":
            location = "<root>"
        lines.append(f"{location}: {error.message}")
    return sorted(lines)


def main() -> int:
    path = edited_path(sys.stdin.read())
    if path is None:
        print("yaml_check_hook: unreadable hook input, skipped", file=sys.stderr)
        return 0
    if path.suffix not in (".yaml", ".yml"):
        return 0
    root = repo_root(path)
    try:
        relative = path.relative_to(root).as_posix()
    except ValueError:
        return 0
    kind = kind_for(relative)
    if kind is None:
        return 0
    load = yaml_loader()
    if load is None:
        print("yaml_check_hook: no YAML parser available, skipped", file=sys.stderr)
        return 0
    try:
        data = load(path.read_text(encoding="utf-8"))
    except Exception as error:
        print(f"{relative}: YAML parse error: {error}", file=sys.stderr)
        return 2
    schema_path = root / "schemas" / f"{kind}.json"
    errors = schema_errors(schema_path, data)
    if errors is None:
        print(f"yaml_check_hook: {relative} parsed; schema validation pending", file=sys.stderr)
        return 0
    if len(errors) == 0:
        return 0
    for line in errors[:20]:
        print(f"{relative}: schema {kind}: {line}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    try:
        code = main()
    except Exception as error:
        print(f"yaml_check_hook: internal error, skipped: {error}", file=sys.stderr)
        code = 0
    sys.exit(code)
