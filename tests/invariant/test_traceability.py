"""Invariant tier: every requirement is traced to a test, every citation to a spec, and the matrix is current."""

from __future__ import annotations

import pytest

from traceability import default_root, main


def test_repository_traceability_is_clean_and_matrix_is_current(capsys: pytest.CaptureFixture[str]) -> None:
    status = main(["--root", str(default_root())])
    assert (status, capsys.readouterr().out) == (0, "")
