"""Rope flags, single-axis sections, agreement with the viewer overlay, the file kind and the core wiring
(INT-002, INT-005, INT-022, FAC-013)."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from mechsim.facade.files import file_kind, validate_file
from mechsim.integrity.beam import normal_stress
from mechsim.integrity.config import IntegrityConfig
from mechsim.integrity.inputs import run_inputs, section_properties
from mechsim.integrity.service import IntegrityService
from mechsim.integrity.table import evaluate
from mechsim.record.tick import Wrench
from mechsim.testing.driver import MemberSection
from mechsim.viewer.overlays import member_stress
from tests.reference.test_integrity_ropes import hanging
from tests.unit.test_integrity import AREA, I_Z, LENGTH, _member, _recorded, _section, _views

ROOT = Path(__file__).resolve().parents[2]
EXAMPLE = ROOT / "schemas" / "examples" / "integrity.yaml"


@pytest.mark.req("INT-022")
@pytest.mark.parametrize(
    ("breaking", "allowance", "peak", "flags"),
    [
        (None, "0", 1000.0, ("design_load",)),
        (500.0, None, 600.0, ("snap",)),
        (500.0, None, 500.0, ()),
        (500.0, "0", 600.0, ("design_load", "snap")),
        (None, None, 1e6, ()),
        (2000.0, "3/20", 2.0 * 9.81 * 1.15, ()),
    ],
)
def test_rope_flags_follow_the_breaking_tension_and_the_design_load(
    breaking: float | None, allowance: str | None, peak: float, flags: tuple[str, ...]
) -> None:
    # Design-load tension: rated load 2 kg (the test design) times (1 + allowance) times |g| = 9.81 m/s^2.
    run = hanging([0.0, peak, 0.5 * peak], breaking, allowance)
    (rope,) = evaluate(IntegrityConfig(schema_version=1), run_inputs(run.build, run.view, {})).ropes
    assert (rope.peak_tension, rope.peak_tension_time, rope.flags) == (peak, 0.001, flags)


@pytest.mark.req("INT-022")
def test_a_rope_missing_from_the_recording_is_an_error() -> None:
    run = hanging([1.0])
    run.build.physics.ropes = (SimpleNamespace(name="ghost", breaking_tension=None),)
    with pytest.raises(ValueError, match="rope 'ghost'"):
        run_inputs(run.build, run.view, {})
    no_series = SimpleNamespace(design=run.build.design, members=(), physics=run.build.physics)
    with pytest.raises(ValueError, match="physics/ropes/max_tension"):
        run_inputs(no_series, _recorded([(Wrench(), Wrench())], names=()), {})


@pytest.mark.req("INT-005")
def test_strength_agrees_with_the_viewer_overlay_on_a_two_axis_section() -> None:
    # Both evaluate |N| / A + |M_y| c_z / I_y + |M_z| c_y / I_z (ruling 25) per end and sample.
    start = Wrench((0.3, 1.5, -2.0), (40.0, 3.0, 1.0))
    end = Wrench((0.0, -0.7, 4.0), (-40.0, 2.0, 0.0))
    view = _recorded([(start, end), (end, start)])
    section = SimpleNamespace(**vars(_section()), second_moment_y=2e-8, fibre_y=0.02, fibre_z=0.01)
    props = section_properties(section)
    rail = SimpleNamespace(**{**vars(_member()), "section": section})
    build = SimpleNamespace(design=hanging([1.0]).build.design, members=(rail,))
    loads = run_inputs(build, view, _views()).members[0].loads
    ours = normal_stress(loads, props)
    (theirs,) = member_stress(view, (MemberSection("rail", "petg_printed", AREA, I_Z, LENGTH, 2e-8, 0.02, 0.01),))
    assert ours[:, 0] == pytest.approx(np.asarray(theirs.start), rel=1e-12)
    assert ours[:, 1] == pytest.approx(np.asarray(theirs.end), rel=1e-12)


@pytest.mark.req("INT-005")
def test_a_single_axis_section_is_evaluated_and_flagged_never_refused() -> None:
    view = _recorded([(Wrench((0.0, 1.0, 1.0), (0.0, 0.0, 0.0)), Wrench())])
    build = SimpleNamespace(design=hanging([1.0], allowance=None).build.design, members=(_member(),))
    table = evaluate(IntegrityConfig(schema_version=1), run_inputs(build, view, _views()))
    assert table.member("rail").flags == (
        "approximation: single-axis section",
        "approximation: static deflection from end loads",
    )
    assert table.member("rail").peak_stress > 0.0


@pytest.mark.req("INT-002", "FAC-013")
def test_validate_knows_the_integrity_file_kind(tmp_path: Path) -> None:
    assert file_kind(EXAMPLE) == "integrity"
    assert validate_file(EXAMPLE, ROOT / "designs") == ("integrity", [])
    bad = tmp_path / "integrity.yaml"
    bad.write_text("schema_version: 1\nchecks:\n  - check: nope\n", encoding="utf-8")
    kind, errors = validate_file(bad, ROOT / "designs")
    assert kind == "integrity"
    assert len(errors) == 1


@pytest.mark.req("INT-022")
def test_core_services_wire_integrity_from_the_design_folder(tmp_path: Path) -> None:
    core_driver = pytest.importorskip("mechsim.testing.core_driver")
    services = core_driver.core_services(tmp_path)
    assert isinstance(services.integrity, IntegrityService)
    assert services.integrity.designs_root == tmp_path
    folder = tmp_path / "machine"
    folder.mkdir()
    build = SimpleNamespace(design=hanging([1.0]).build.design, members=(_member(),))
    view = _recorded([(Wrench(), Wrench())])
    assert services.integrity(build, None, view) is None
    (folder / "integrity.yaml").write_text("schema_version: 1\n", encoding="utf-8")
    report = services.integrity(build, None, view)
    assert report is not None
    assert [entry.member for entry in report.members] == ["rail"]
