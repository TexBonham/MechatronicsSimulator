"""The evaluation start gates a rope snap end to end (INT-031, PHY-036, ENG-047, ruling 75): `CoreDriver` writes the
case's `integrity.evaluate_from` as `rope_snap_evaluation_start` beside `carried_evaluation_start`, and a rope whose
tension exceeded its breaking tension before the start gets a line instead of a snap.

Fixture (the tier 1 tests' `stretched_rope_run`, stretched further): the rope_spring bob released 0.1 m below the
one segment's rest length, so the rope starts at T0 = EA (0.1 m) / L0 = 50 kN, over the 5 kN breaking tension. At the
case's 1 ms step backward Euler on omega^2 = 5e5 N/m / 1.0175 kg (omega h = 0.70) gives the stretch x1 = x0 / (1 +
omega^2 h^2) = 0.67 x0 after the first step and x2 = (x1 + h v1) / (1 + omega^2 h^2) = 0.23 x0 after the second (the
rope's damping, zeta 0.035, lowers both a little): about 33 kN and 11 kN, both above 5 kN; the rope then goes slack
and the bob flies about 2v / g, over a second, so a run to 20 ms sees the exceedance only in its first two steps."""

from __future__ import annotations

from pathlib import Path

import pytest

from mechsim.integrity.tier1 import Tier1Service
from mechsim.physics.native.translate import translate
from mechsim.testing.core_driver import CoreDriver, core_builder
from mechsim.testing.runner import Services, run_case
from tests.reference.test_phy_native_beams import beam_model
from tests.unit.test_integrity_tier1 import SNAP_CASE, SPRING, config


def _with_start(seconds: str) -> str:
    return SNAP_CASE.replace("physics:\n", f"integrity:\n  evaluate_from: {seconds}\nphysics:\n")


def _run(tmp_path: Path, start: str) -> tuple[list[str], tuple[str, ...]]:
    """`stretched_rope_run` of the tier 1 tests with the case's evaluation start set and rope snap on."""
    root = tmp_path / "designs"
    (root / "rope_spring").mkdir(parents=True)
    text = SPRING.read_text(encoding="utf-8").replace('default: "-1 m"', 'default: "-1.1 m"')
    (root / "rope_spring" / "design.yaml").write_text(text, encoding="utf-8")
    case = tmp_path / "S9.yaml"
    case.write_text(_with_start(start), encoding="utf-8")
    drivers: list[CoreDriver] = []

    def factory() -> CoreDriver:
        driver = CoreDriver(designs_root=root, tier1=Tier1Service(config=config({"check": "rope_snap"})))
        drivers.append(driver)
        return driver

    result = run_case(case, Services(factory, core_builder, None, designs_root=root), None)
    assert result.report is not None, result.score.error
    assert not result.report.errored, result.report.error
    driver = drivers[0]
    kinds = [event.detail.split(" ")[0] for event in driver.recording().events() if event.kind == "model_change"]
    return kinds, result.report.approximations


@pytest.mark.req("INT-031", "PHY-036")
def test_an_exceedance_before_the_start_does_not_snap_and_gets_its_line(tmp_path: Path) -> None:
    kinds, lines = _run(tmp_path, "0.01 s")
    assert kinds == []
    held = [line for line in lines if "before the evaluation start" in line]
    assert len(held) == 1
    assert held[0].startswith("rope 'line': before the evaluation start 0.01 s its tension reached ")
    reached = float(held[0].split("reached ")[1].split(" ")[0])
    assert 5000.0 < reached <= 50000.0


@pytest.mark.req("INT-031", "PHY-036")
def test_the_same_exceedance_after_the_start_snaps(tmp_path: Path) -> None:
    """Start at 0.5 ms: the first step (from t = 0) is held back with its line, the step from 1 ms (about 11 kN)
    snaps."""
    kinds, lines = _run(tmp_path, "0.0005 s")
    assert kinds == ["rope_snapped", "load_released"]
    assert any("before the evaluation start 0.0005 s" in line for line in lines)


@pytest.mark.req("PHY-036")
@pytest.mark.parametrize(("start", "written"), [(0.0, False), (0.25, True)])
def test_the_document_writes_both_evaluation_starts_above_zero(start: float, written: bool) -> None:
    document = translate(beam_model("free", 8, 0.0), 1e-3, evaluation_start=start).text
    assert ('"rope_snap_evaluation_start"' in document) == written
    assert ('"carried_evaluation_start"' in document) == written
