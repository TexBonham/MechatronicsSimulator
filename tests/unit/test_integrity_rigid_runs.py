"""A gantry milestone 2 case with `designs/gantry/integrity.yaml` present, end to end through the CLI: on the bridge
(members built rigid, ruling 29) the run and its criteria are unaffected, the members' end-load and node-pose checks
are reported not evaluated with one line each, and a criterion reading one is an unevaluable error (rulings 34, 60);
on the native adapter the same case evaluates every member."""

from __future__ import annotations

import io
import json
import math
from pathlib import Path

import pytest

from mechsim.cli import default_services, main
from mechsim.testing.schema import ROOT

G8 = ROOT / "test_cases" / "gantry" / "G8.yaml"
GANTRY = ROOT / "designs" / "gantry" / "design.yaml"
CRITERION = """  - name: safety_factor_rail_x
    check: value
    quantity: safety_factor:rail_x
    statistic: minimum
    comparison: at_least
    limit: "1.5"
"""
ORIGINAL = ("motor_power_off", "x_no_steps", "y_no_steps", "lift_no_steps")


def _run(case: Path, out: Path) -> str:
    stdout = io.StringIO()
    main(["run", str(case), "--design", str(GANTRY), "--out", str(out)], default_services(), stdout, io.StringIO())
    return stdout.getvalue()


def _report(out: Path) -> dict[str, object]:
    (path,) = out.glob("*.integrity.json")
    data: dict[str, object] = json.loads(path.read_text(encoding="utf-8"))
    return data


@pytest.mark.req("INT-005", "INT-006", "SCO-029")
def test_a_bridge_run_reports_rigid_members_not_evaluated_and_scores_its_criteria(tmp_path: Path) -> None:
    case = tmp_path / "G8.yaml"
    case.write_text(
        G8.read_text(encoding="utf-8").replace("\ncriteria:\n", "\ncriteria:\n" + CRITERION), encoding="utf-8"
    )
    out = _run(case, tmp_path)
    assert "run error" not in out
    for name in ORIGINAL:
        assert f"  pass  {name}:" in out
    assert "  error safety_factor_rail_x:" in out
    assert "safety_factor:rail_x: not evaluated by integrity" in out
    report = _report(tmp_path)
    for member in ("rail_x", "rail_y"):
        lines = [note for note in report["notes"] if f"member {member!r}" in note]  # type: ignore[attr-defined]
        assert lines == [
            f"member {member!r}: strength, deflection, buckling, impact not evaluated, the member is built rigid and "
            "the run recorded no end-loads or node poses (rulings 29, 34)"
        ]
    assert list(tmp_path.glob("*.load_case.json")) == []


@pytest.mark.req("INT-005", "INT-006")
def test_the_same_case_on_the_native_adapter_evaluates_every_member(tmp_path: Path) -> None:
    text = G8.read_text(encoding="utf-8").replace(
        "controller_mode: native\n", "controller_mode: native\nphysics:\n  adapter: native\n"
    )
    case = tmp_path / "G8.yaml"
    case.write_text(text, encoding="utf-8")
    out = _run(case, tmp_path)
    assert "Case G8: PASS" in out
    report = _report(tmp_path)
    for note in report["notes"]:  # type: ignore[attr-defined]
        assert note.startswith("strength of 'rail_")
        assert ": conservative" in note
        assert "carried couple" in note
    table = report["table"]["members"]  # type: ignore[index]
    for row in table:
        assert row["flags"] in ([], ["safety_factor_below_minimum"])
        assert math.isfinite(row["safety_factor"])
        assert math.isfinite(row["peak_stress"])
        assert math.isfinite(row["deflection"])
    assert len(list(tmp_path.glob("*.load_case.json"))) == 1
