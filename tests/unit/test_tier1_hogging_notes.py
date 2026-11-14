"""The tier 1 party's own-weight lines reach the run's notes (ruling 79, INT-005, INT-023; brief M3-D2d item 2): a
real run through `CoreDriver` of the data-only carried bar, its carriage's weight offset along the bar so the carried
load puts a couple at the foot point."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from mechsim.builder import build
from mechsim.design.schema import Design, shipped_materials
from mechsim.library.registry import default_registry
from mechsim.testing.core_driver import CoreDriver, core_builder
from mechsim.testing.runner import Services, run_case
from tests.unit.test_carried_members import _design_data

CASE = """id: HG1
title: Carried bar, own-weight line
design:
  name: carried_bar
  version: 1.0.0
seed: 0
initial_state:
  joints: []
rates:
  physics_step: 1 ms
physics:
  adapter: native
end:
  at: 0.02 s
  max_duration: 1 s
criteria:
  - name: bar_safety_factor
    check: value
    quantity: safety_factor:bar
    statistic: minimum
    comparison: at_least
    limit: "0 1"
"""
OFFSET = 0.02
GRAVITY = 9.81
SEGMENTS = 16
HOGGING = "may read low under a carried couple at a foot point where the own weight's moment hogs"


def _data(supports: tuple[str, str], position: float) -> dict[str, Any]:
    data = _design_data(["slide"], position=position, limits=(-0.05, 0.05))
    data["bodies"][0]["flexible"]["supports"] = list(supports)
    data["bodies"][1]["geometry"]["shapes"][0]["size"] = ["0.15 m", "0.15 m", "0.15 m"]
    data["bodies"][1]["geometry"]["shapes"][0]["position"] = [f"{OFFSET} m", "0 m", "0 m"]
    data["structural"] = {
        "design_load": {"rated_load": "1 kg", "allowance": "0 percent"},
        "accuracy_target": "1 m",
        "deflection_target": "static",
    }
    return data


def _run(tmp_path: Path, data: dict[str, Any]) -> tuple[str, ...]:
    designs = tmp_path / "designs"
    folder = designs / "carried_bar"
    folder.mkdir(parents=True)
    (folder / "design.yaml").write_text(json.dumps(data, indent=1), encoding="utf-8")
    (folder / "integrity.yaml").write_text("schema_version: 1\nchecks:\n  - check: strength\n", encoding="utf-8")
    case = tmp_path / "HG1.yaml"
    case.write_text(CASE, encoding="utf-8")
    result = run_case(case, Services(lambda: CoreDriver(designs_root=designs), core_builder, None, designs), None)
    assert result.report is not None, result.score.error
    assert not result.report.errored, result.report.error
    return tuple(result.report.notes)


@pytest.mark.req("INT-005", "INT-023")
@pytest.mark.parametrize(
    ("supports", "position", "present"),
    [
        (("clamped", "clamped"), 0.1, True),
        (("clamped", "clamped"), 0.5, False),
        (("pinned", "roller"), 0.1, False),
    ],
)
def test_the_partys_own_weight_line_is_in_the_run_notes_only_when_its_condition_holds(
    tmp_path: Path, supports: tuple[str, str], position: float, present: bool
) -> None:
    """The carriage's weight P offset by e = 20 mm along the bar puts a couple P e at its foot point, far above the
    couple floor 1e-9 P L. Under its own weight a clamped-clamped 1 m span hogs on [0, 0.2113 m] and [0.7887 m, 1 m]
    (the roots of 6 L x - 6 x^2 - L^2 = 0, x = L (1 -+ 1 / sqrt(3)) / 2), so at a = 0.1 m the line is present and at
    0.5 m (sagging) absent; a pinned-roller span never hogs, so at 0.1 m it is absent. The line's bound is
    3 w l^2 with w = rho A g and l = L / 16, the party's own-weight figure (ruling 79)."""
    data = _data(supports, position)
    lines = [note for note in _run(tmp_path, data) if HOGGING in note]
    if not present:
        assert lines == []
        return
    assert len(lines) == 1
    built = build(Design.model_validate(data), default_registry(), shipped_materials())
    assert built.physics is not None
    section = built.physics.lumped_members[0].section
    bound = 3.0 * float(section.density) * float(section.area) * GRAVITY * (float(section.length) / SEGMENTS) ** 2
    print(f"own-weight bound 3 w l^2: closed form {bound:.6g} N m; line: {lines[0]}")
    assert lines[0].startswith("strength of 'bar': ")
    assert f"by at most 3 w l^2 = {bound:.6g} N m" in lines[0]
