"""Client code holds no domain logic (FAC-016, invariant tier): a client's only simulator import is the facade."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def _mechsim_imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names if alias.name.startswith("mechsim"))
        elif isinstance(node, ast.ImportFrom) and node.module is not None and node.module.startswith("mechsim"):
            names.add(node.module)
        elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "import_module":
            for argument in node.args:
                if isinstance(argument, ast.Constant) and str(argument.value).startswith("mechsim"):
                    names.add(str(argument.value))
    return names


@pytest.mark.req("FAC-016")
def test_clients_import_only_the_facade() -> None:
    clients = [ROOT / "mechsim" / "cli.py", *sorted((ROOT / "web").rglob("*.py"))]
    for client in clients:
        for module in _mechsim_imports(client):
            assert module == "mechsim.facade" or module.startswith("mechsim.facade."), f"{client} imports {module}"


@pytest.mark.req("FAC-016")
def test_the_import_scan_catches_a_client_reaching_past_the_facade(tmp_path: Path) -> None:
    client = tmp_path / "client.py"
    client.write_text(
        "import mechsim.scoring.score\nfrom mechsim.facade.service import Facade\n"
        "import importlib\nimportlib.import_module('mechsim.record')\n",
        encoding="utf-8",
    )
    assert _mechsim_imports(client) == {"mechsim.scoring.score", "mechsim.facade.service", "mechsim.record"}
