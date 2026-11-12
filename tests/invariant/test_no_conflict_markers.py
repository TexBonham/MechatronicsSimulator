"""Invariant tier: no merge-conflict marker survives in a tracked text file (a merge-back once committed two)."""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
MARKERS = ("<<<<<<< ", "=======\n", ">>>>>>> ")
TEXT_SUFFIXES = {
    ".md",
    ".py",
    ".yaml",
    ".yml",
    ".json",
    ".cpp",
    ".hpp",
    ".c",
    ".h",
    ".txt",
    ".toml",
    ".cmake",
    ".js",
    ".html",
    ".css",
}


def _tracked_text_files() -> list[Path]:
    listed = subprocess.run(["git", "ls-files"], cwd=ROOT, capture_output=True, text=True, check=True).stdout
    return [ROOT / line for line in listed.splitlines() if Path(line).suffix in TEXT_SUFFIXES]


@pytest.mark.req("INV-003")
def test_no_conflict_markers_in_tracked_files() -> None:
    offenders: list[str] = []
    for path in _tracked_text_files():
        if path.name == "test_no_conflict_markers.py":
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for marker in MARKERS:
            if marker == "=======\n":
                if "\n=======\n" in text and "<<<<<<< " in text:
                    offenders.append(f"{path.relative_to(ROOT)}: {marker.strip()}")
            elif marker in text:
                offenders.append(f"{path.relative_to(ROOT)}: {marker.strip()}")
    assert offenders == []
