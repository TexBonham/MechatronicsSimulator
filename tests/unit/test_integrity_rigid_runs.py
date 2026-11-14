"""A gantry milestone 2 case with `designs/gantry/integrity.yaml` present, end to end through the CLI: on the bridge
(members built rigid, ruling 29) the run and its criteria are unaffected, the members' end-load and node-pose checks
are reported not evaluated with one line each, and a criterion reading one is an unevaluable error (rulings 34, 60);
on the native adapter the same case evaluates every member."""

from __future__ import annotations

import io
import json
import math
import re
from pathlib import Path
from typing import Any

import numpy as np
import pytest

import mechsim.integrity.service as integrity_service
from mechsim.cli import default_services, main
from mechsim.integrity.beam import STUB, carried_at
from mechsim.integrity.plugin import RunInputs
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
    assert report["notes"] == [
        f"member {member!r}: strength, deflection, buckling, impact not evaluated, the member is built rigid and "
        "the run recorded no end-loads or node poses (rulings 29, 34)"
        for member in ("rail_x", "rail_y")
    ]
    assert list(tmp_path.glob("*.load_case.json")) == []


def _stub_bound(inputs: RunInputs, member: str) -> float:
    """The stub bound by hand (INT-005, `beam.py` header): one carried load at s in a clamped root's half segment
    [0, l / 2) (k = 0) has no clean sample left of it; the bracketing samples are a = 0 and b = x_2 = 3 l / 2, so
    U = abs(F) (b - a) / 4 + abs(C) plus the stub's abs(F) l / 8 + abs(C) = abs(F) l / 2 + 2 abs(C), at the design
    load; the largest over the rows whose load sits in the stub."""
    chosen = inputs.member(member)
    assert chosen.nodes is not None
    assert inputs.bodies is not None
    assert chosen.nodes.root == "clamped"
    assert len(chosen.carried) == 1
    segment = chosen.nodes.segment_length
    s, force, couple = carried_at(chosen, inputs.bodies)
    stub = np.clip(s[:, 0], 0.0, chosen.section.length) < segment / 2.0
    bound = inputs.load_factor * (np.abs(force[:, 0]) * segment / 2.0 + 2.0 * np.abs(couple[:, 0]))
    return float(bound[stub].max())


@pytest.mark.req("INT-005", "INT-006")
def test_the_same_case_on_the_native_adapter_evaluates_every_member(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    text = G8.read_text(encoding="utf-8").replace(
        "controller_mode: native\n", "controller_mode: native\nphysics:\n  adapter: native\n"
    )
    case = tmp_path / "G8.yaml"
    case.write_text(text, encoding="utf-8")
    seen: list[RunInputs] = []
    original = integrity_service.evaluate

    def capture(config: Any, inputs: RunInputs, *rest: Any) -> Any:
        seen.append(inputs)
        return original(config, inputs, *rest)

    monkeypatch.setattr(integrity_service, "evaluate", capture)
    out = _run(case, tmp_path)
    assert "Case G8: PASS" in out
    report = _report(tmp_path)
    notes = report["notes"]
    assert isinstance(notes, list)
    stubs = [note for note in notes if STUB in note]
    assert len(stubs) == 1
    (note,) = stubs
    match = re.fullmatch(
        r"strength of 'rail_x': conservative, each sample's read adds the bound of what the carried-load rule does "
        r"not resolve, (.+): up to (\S+) N m at the design load \(INT-005, ruling 75\)",
        note,
    )
    assert match is not None
    assert match.group(1) == STUB
    (inputs,) = seen
    assert float(match.group(2)) == pytest.approx(_stub_bound(inputs, "rail_x"), rel=1e-5)
    assert all("not evaluated" not in line for line in notes)
    table = report["table"]["members"]  # type: ignore[index]
    for row in table:
        assert row["flags"] in ([], ["safety_factor_below_minimum"])
        assert math.isfinite(row["safety_factor"])
        assert math.isfinite(row["peak_stress"])
        assert math.isfinite(row["deflection"])
    assert len(list(tmp_path.glob("*.load_case.json"))) == 1
