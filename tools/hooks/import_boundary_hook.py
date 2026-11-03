#!/usr/bin/env python3
"""PostToolUse hook: run the import-boundary checker on an edited Python file.

Expects the Claude Code hook JSON on stdin, shaped like
    {"tool_name": "Edit", "tool_input": {"file_path": "/abs/or/relative/path.py"}, "cwd": "..."}
Exit 2 with violations on stderr so the model sees them; exit 0 in every other case.
"""

import json
import subprocess
import sys
from pathlib import Path

CHECKER = Path("tools") / "check_imports.py"
TIMEOUT_SECONDS = 60


def read_payload() -> dict | None:
    try:
        payload = json.loads(sys.stdin.read())
    except (ValueError, OSError):
        return None
    if not isinstance(payload, dict):
        return None
    return payload


def edited_path(payload: dict) -> Path | None:
    tool_input = payload.get("tool_input")
    if not isinstance(tool_input, dict):
        return None
    raw = tool_input.get("file_path")
    if not isinstance(raw, str) or raw == "":
        return None
    path = Path(raw)
    if not path.is_absolute():
        cwd = payload.get("cwd")
        if isinstance(cwd, str) and cwd != "":
            path = Path(cwd) / path
    return path.resolve()


def repo_root(path: Path) -> Path:
    for candidate in path.parents:
        if (candidate / ".git").exists():
            return candidate
    return Path(__file__).resolve().parents[2]


def main() -> int:
    payload = read_payload()
    if payload is None:
        print("import_boundary_hook: unreadable hook input, skipped", file=sys.stderr)
        return 0
    path = edited_path(payload)
    if path is None or path.suffix != ".py":
        return 0
    root = repo_root(path)
    checker = root / CHECKER
    if not checker.is_file():
        print("import_boundary_hook: tools/check_imports.py is not installed, skipped", file=sys.stderr)
        return 0
    try:
        relative = path.relative_to(root)
    except ValueError:
        return 0
    try:
        result = subprocess.run(
            [sys.executable, str(checker), str(relative)],
            cwd=root,
            capture_output=True,
            text=True,
            timeout=TIMEOUT_SECONDS,
        )
    except (OSError, subprocess.SubprocessError) as error:
        print(f"import_boundary_hook: checker did not run: {error}", file=sys.stderr)
        return 0
    if result.returncode == 0:
        return 0
    output = (result.stdout + result.stderr).strip()
    if result.returncode == 1:
        print(output, file=sys.stderr)
        return 2
    print(f"import_boundary_hook: checker exited {result.returncode}: {output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    try:
        code = main()
    except Exception as error:
        print(f"import_boundary_hook: internal error, skipped: {error}", file=sys.stderr)
        code = 0
    sys.exit(code)
