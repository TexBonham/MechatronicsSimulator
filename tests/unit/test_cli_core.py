"""The CLI over the real services: `mechsim run --design`, replay from the archive, exit codes (TST-033, TST-035)."""

from __future__ import annotations

import io
from pathlib import Path

import pytest

from mechsim.cli import EXIT_ERROR, EXIT_PASS, default_services, main
from mechsim.testing.schema import ROOT

P1 = ROOT / "test_cases" / "pendulum" / "P1_period.yaml"
PENDULUM = ROOT / "designs" / "pendulum" / "design.yaml"


def _call(argv: list[str]) -> tuple[int, str, str]:
    out = io.StringIO()
    err = io.StringIO()
    code = main(argv, default_services(), out, err)
    return code, out.getvalue(), err.getvalue()


@pytest.mark.req("TST-035")
@pytest.mark.req("TST-033")
def test_run_with_design_then_replay(tmp_path: Path) -> None:
    code, out, _ = _call(["run", str(P1), "--design", str(PENDULUM), "--out", str(tmp_path)])
    assert code == EXIT_PASS
    assert "Case P1: PASS" in out
    archive = tmp_path / "P1-seed0.h5"
    assert f"archive: {archive}" in out
    code, out, _ = _call(["replay", str(archive)])
    assert code == EXIT_PASS
    assert out.startswith("Run P1-seed0: case P1, design pendulum 0.1.0, seed 0, controller mode none\n")
    assert out.endswith("ended: end_time at 24.4 s\n")


@pytest.mark.req("TST-035")
def test_design_option_must_name_a_design_file(tmp_path: Path) -> None:
    code, _, err = _call(["run", str(P1), "--design", str(tmp_path / "other.yaml")])
    assert code == EXIT_ERROR
    assert "not a <designs>/<name>/design.yaml file" in err


@pytest.mark.req("TST-035")
def test_gantry_g3_with_an_ill_dimensioned_binding_is_scored_not_run(tmp_path: Path) -> None:
    code, out, _ = _call(
        [
            "run",
            str(ROOT / "test_cases" / "gantry" / "G3.yaml"),
            "--design",
            str(ROOT / "designs" / "gantry" / "design.yaml"),
            "--set",
            "run_time=1 kg",
            "--out",
            str(tmp_path),
        ]
    )
    assert code == EXIT_ERROR
    assert "Case G3: ERROR" in out
    assert "not run" in out
