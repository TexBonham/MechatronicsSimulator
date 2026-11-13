"""Integrity package: plugin interface, registry, configuration, tiers, design-load rule, flags, the per-member
table, the wear seam and the thermal slot (INT-001 to INT-003, INT-019 to INT-022, INT-036, INT-037)."""

from __future__ import annotations

import json
import math
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from typing import Any, ClassVar

import jsonschema
import numpy as np
import pytest

from mechsim.integrity import config as config_module
from mechsim.integrity import registry as registry_module
from mechsim.integrity.beam import deflection, effective_length_factor, first_frequency, peak
from mechsim.integrity.checks.buckling import Buckling, BucklingParameters
from mechsim.integrity.checks.fastener import Fastener, FastenerParameters
from mechsim.integrity.checks.fatigue import cycles_to_failure, rainflow, reversals
from mechsim.integrity.checks.impact import Impact, ImpactParameters, events
from mechsim.integrity.checks.resonance import Resonance, ResonanceParameters
from mechsim.integrity.checks.strength import Strength
from mechsim.integrity.config import IntegrityConfig, config_for, load_config, parse_config
from mechsim.integrity.export import load_case, write_beside
from mechsim.integrity.inputs import member_axes, run_inputs, section_properties
from mechsim.integrity.plugin import (
    CheckResult,
    IntegrityCheck,
    IntegrityError,
    JointLoads,
    NoParameters,
    RunInputs,
    Wrenches,
)
from mechsim.integrity.service import IntegrityService
from mechsim.integrity.table import THERMAL_NOTE, IntegrityReport, evaluate
from mechsim.record.layout import MEMBER_SERIES, Columns
from mechsim.record.stream import Recorder
from mechsim.record.tick import JointForce, MemberEndLoads, PhysicsOutputs, TickRecord, Wrench
from mechsim.schema_base import StrictModel
from mechsim.testing.driver import IntegrityResults, MemberSection, Series
from tests.reference.test_integrity_checks import (
    AREA,
    C_Y,
    I_Z,
    LENGTH,
    MODULUS,
    SECTION,
    TEMPERATURE,
    FakeMaterial,
    cantilever,
    constant,
    inputs,
    member,
)

EXAMPLE = Path(__file__).resolve().parents[2] / "schemas" / "examples" / "integrity.yaml"
STATIC = "approximation: static deflection from end loads"
SHIPPED = ("buckling", "deflection", "fastener", "fatigue", "impact", "resonance", "strength")


# ===== Plugin interface and registry (INT-001) ==========================


class _Probe(IntegrityCheck):
    name = "probe"
    tiers: ClassVar[tuple[int, ...]] = (1, 3)

    def evaluate(self, inputs: RunInputs, parameters: StrictModel, tier: int) -> CheckResult:
        self.check_tier(tier)
        return CheckResult(self.name, tier, ())


class _Point:
    def __init__(self, name: str, loaded: object) -> None:
        self.name = name
        self.loaded = loaded

    def load(self) -> object:
        return self.loaded


@pytest.mark.req("INT-001")
def test_the_registry_discovers_every_shipped_check_module() -> None:
    found = registry_module.discover()
    assert tuple(found) == SHIPPED
    assert all(isinstance(check, IntegrityCheck) for check in found.values())


@pytest.mark.req("INT-001")
def test_a_new_check_is_a_plugin_class_and_gets_its_schema_entry(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(registry_module, "entry_points", lambda group: [_Point("probe", _Probe)])
    found = registry_module.discover()
    assert "probe" in found
    entry = config_module._entry_model(found["probe"])
    assert entry.model_validate({"check": "probe", "tier": 3}).model_dump() == {
        "check": "probe",
        "tier": 3,
        "parameters": {},
    }
    assert entry.model_json_schema()["properties"]["tier"]["enum"] == [1, 3]


@pytest.mark.req("INT-001")
@pytest.mark.parametrize("loaded", [object, int])
def test_an_entry_point_that_is_not_a_check_is_an_error(monkeypatch: pytest.MonkeyPatch, loaded: object) -> None:
    monkeypatch.setattr(registry_module, "entry_points", lambda group: [_Point("bad", loaded)])
    with pytest.raises(IntegrityError, match="not an IntegrityCheck"):
        registry_module.discover()


@pytest.mark.req("INT-001")
def test_a_duplicate_check_name_is_an_error(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(registry_module, "entry_points", lambda group: [_Point("strength", type("S", (Strength,), {}))])
    with pytest.raises(IntegrityError, match="duplicate integrity check name 'strength'"):
        registry_module.discover()


@pytest.mark.req("INT-001")
def test_a_tier_a_check_does_not_offer_is_an_error() -> None:
    with pytest.raises(IntegrityError, match="tier 2 not offered"):
        _Probe().evaluate(inputs([], 1), NoParameters(), 2)


# ===== Configuration from data (INT-002) ================================


CONFIG_TEXT = """\
schema_version: 1
checks:
  - check: strength
    tier: 2
  - check: rope_snap
  - check: impact
    parameters:
      impact_factor: 2.0
      step_threshold: 50 N
      window: 20 ms
  - check: resonance
    parameters:
      band: 0.2
      excitations:
        - series: joint:x_axis.velocity
          hz_per_unit: 125.0
"""


@pytest.mark.req("INT-002")
def test_checks_tiers_and_parameters_come_from_the_configuration_file(tmp_path: Path) -> None:
    path = tmp_path / "integrity.yaml"
    path.write_text(CONFIG_TEXT, encoding="utf-8")
    config = load_config(path)
    assert config.rope_snap()
    entries = {check.name: (tier, parameters) for check, tier, parameters in config.entries()}
    assert list(entries) == ["deflection", "strength", "impact", "resonance"]
    assert entries["strength"][0] == 2
    impact = entries["impact"][1]
    assert isinstance(impact, ImpactParameters)
    assert (impact.impact_factor, impact.step_threshold, impact.window) == (2.0, 50.0, pytest.approx(0.02))
    assert config_for(tmp_path) == config


@pytest.mark.req("INT-002")
def test_a_design_without_a_configuration_runs_the_table_checks_only(tmp_path: Path) -> None:
    config = config_for(tmp_path)
    assert config == IntegrityConfig(schema_version=1)
    assert [(check.name, tier) for check, tier, _ in config.entries()] == [("strength", 1), ("deflection", 1)]
    assert not config.rope_snap()


@pytest.mark.req("INT-002")
@pytest.mark.parametrize(
    ("data", "message"),
    [
        ({"schema_version": 1, "checks": [{"check": "thermal"}]}, "checks.0"),
        ({"schema_version": 1, "checks": [{"check": "strength", "tier": 3}]}, "tier"),
        (
            {"schema_version": 1, "checks": [{"check": "impact", "parameters": {"impact_factor": 2.0}}]},
            "step_threshold",
        ),
        (
            {
                "schema_version": 1,
                "checks": [
                    {"check": "impact", "parameters": {"impact_factor": 2.0, "step_threshold": "5", "window": "1 s"}}
                ],
            },
            "step_threshold",
        ),
        ({"schema_version": 1, "checks": [{"check": "fatigue"}, {"check": "fatigue"}]}, "more than once: fatigue"),
        ({"schema_version": 2}, "schema_version"),
        ({"schema_version": 1, "extra": 1}, "extra"),
        ({"schema_version": 1, "halt_on_crossing": True}, "halt_on_crossing"),
        ({"schema_version": 1, "checks": [{"check": "rope_snap", "tier": 1}]}, "tier"),
        ({"schema_version": 1, "checks": [{"check": "rope_snap", "parameters": {}}]}, "parameters"),
        (
            {"schema_version": 1, "checks": [{"check": "rope_snap"}, {"check": "rope_snap"}]},
            "more than once: rope_snap",
        ),
        ({"schema_version": 1, "checks": [{"check": "resonance", "parameters": {"band": 1.5}}]}, "band"),
    ],
)
def test_an_invalid_configuration_is_an_error_naming_the_field(data: dict[str, Any], message: str) -> None:
    with pytest.raises(IntegrityError, match=message):
        parse_config(data, "integrity.yaml")


@pytest.mark.req("INT-002")
def test_an_unreadable_configuration_is_an_error(tmp_path: Path) -> None:
    path = tmp_path / "integrity.yaml"
    path.write_text("checks: [unclosed\n", encoding="utf-8")
    with pytest.raises(IntegrityError, match="cannot read"):
        load_config(path)


@pytest.mark.req("INT-001", "INT-002")
def test_the_schema_is_current_and_validates_the_example() -> None:
    assert config_module.SCHEMA_PATH.read_text(encoding="utf-8") == config_module.schema_text()
    schema = json.loads(config_module.schema_text())
    example = config_module.yaml_loader().load(EXAMPLE.read_text(encoding="utf-8"))
    jsonschema.validate(json.loads(json.dumps(example)), schema)
    load_config(EXAMPLE)
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate({"schema_version": 1, "checks": [{"check": "nope"}]}, schema)


# ===== Tiers are additive (INT-003) =====================================


@pytest.mark.req("INT-003")
def test_a_higher_tier_adds_rows_and_never_changes_a_lower_tiers_rows() -> None:
    joint = JointLoads(
        "mount", "mount", "base", np.tile([1.0, 2.0, 3.0], (3, 1)), np.tile([0.1, 0.0, 0.0], (3, 1)), member="rail"
    )
    run = inputs([member(cantilever(70.0))], 3, joints=(joint,), minimum_safety_factor=2.0)
    first = Strength().evaluate(run, NoParameters(), 1)
    second = Strength().evaluate(run, NoParameters(), 2)
    assert second.rows_of_tier(1) == first.rows
    assert len(second.rows) > len(first.rows)
    assert {row.tier for row in second.rows} == {1, 2}
    table_one = evaluate(parse_config({"schema_version": 1}, "c"), run)
    table_two = evaluate(parse_config({"schema_version": 1, "checks": [{"check": "strength", "tier": 2}]}, "c"), run)
    assert table_two.member("rail").peak_stress == table_one.member("rail").peak_stress
    assert table_two.member("rail").deflection == table_one.member("rail").deflection


# ===== Design-load rule (INT-019) =======================================


def _design(allowance: str | None, minimum: str | None = None, target: float = 0.01) -> Any:
    structural = None
    if allowance is not None:
        safety = None
        if minimum is not None:
            safety = Fraction(minimum)
        rule = SimpleNamespace(rated_load=2.0, allowance=Fraction(allowance))
        structural = SimpleNamespace(design_load=rule, accuracy_target=target, minimum_safety_factor=safety)
    return SimpleNamespace(
        name="machine",
        materials=None,
        structural=structural,
        environment=SimpleNamespace(ambient_temperature=TEMPERATURE, gravity=[0.0, 0.0, -9.81]),
    )


def _section() -> Any:
    return SimpleNamespace(area=AREA, second_moment_of_area=I_Z, elastic_modulus=MODULUS, density=1270.0, length=LENGTH)


def _member(name: str = "rail", mounts: tuple[str, ...] = ()) -> Any:
    supports = (SimpleNamespace(at=0.0, condition="clamped"),)
    return SimpleNamespace(name=name, material="petg_printed", section=_section(), supports=supports, mounts=mounts)


class _View:
    """A RecordView over a recorder recording."""

    def __init__(self, recording: Any, series: dict[str, Series] | None = None) -> None:
        self.recording = recording
        self._times = tuple(float(t) for t in recording.series["time/seconds"].values)
        self._series = series or {}

    def times(self) -> tuple[float, ...]:
        return self._times

    def series(self, reference: str) -> Series:
        return self._series[reference]

    def controller_states(self) -> tuple[str, ...]:
        return ()

    def events(self) -> tuple[Any, ...]:
        return ()


def _recorded(
    loads: list[tuple[Wrench, Wrench]], names: tuple[str, ...] = ("rail",), joints: tuple[Wrench, ...] = ()
) -> _View:
    columns = {path: Columns(names) for path in MEMBER_SERIES}
    recorder = Recorder(Fraction(1000), columns)
    for tick, (start, end) in enumerate(loads):
        physics = PhysicsOutputs(
            steps=tick + 1,
            members=tuple(MemberEndLoads(start, end) for _ in names),
            joints=tuple(JointForce((0.0,), reaction) for reaction in joints),
        )
        recorder.write_row(TickRecord(time=tick, physics=physics))
    return _View(recorder.recording())


def _cantilever_rows(load: float, rows: int = 3) -> list[tuple[Wrench, Wrench]]:
    start = Wrench((0.0, 0.0, load * LENGTH), (0.0, load, 0.0))
    end = Wrench((0.0, 0.0, 0.0), (0.0, -load, 0.0))
    return [(start, end)] * rows


def _views() -> dict[str, Any]:
    return {"petg_printed": FakeMaterial()}


@pytest.mark.req("INT-019")
def test_the_design_load_rule_comes_from_the_design_file() -> None:
    view = _recorded(_cantilever_rows(10.0))
    base = run_inputs(SimpleNamespace(design=_design("0"), members=(_member(),)), view, _views())
    scaled = run_inputs(SimpleNamespace(design=_design("3/20"), members=(_member(),)), view, _views())
    doubled = run_inputs(SimpleNamespace(design=_design("1"), members=(_member(),)), view, _views())
    assert (base.load_factor, scaled.load_factor, doubled.load_factor) == (1.0, 1.15, 2.0)
    config = IntegrityConfig(schema_version=1)
    stress = evaluate(config, base).member("rail").peak_stress
    assert stress == pytest.approx(10.0 * LENGTH * C_Y / I_Z)
    assert evaluate(config, scaled).member("rail").peak_stress == pytest.approx(1.15 * stress)
    assert evaluate(config, doubled).member("rail").deflection == pytest.approx(
        2.0 * evaluate(config, base).member("rail").deflection
    )


@pytest.mark.req("INT-019")
def test_a_design_without_a_structural_section_applies_no_rule() -> None:
    run = run_inputs(
        SimpleNamespace(design=_design(None), members=(_member(),)), _recorded(_cantilever_rows(1.0)), _views()
    )
    assert (run.load_factor, run.minimum_safety_factor, run.deflection_limit) == (1.0, None, None)
    assert evaluate(IntegrityConfig(schema_version=1), run).member("rail").flags == (
        "approximation: single-axis section",
        STATIC,
    )


@pytest.mark.req("INT-019", "INT-022")
def test_run_inputs_read_sections_supports_mounts_and_joint_reactions() -> None:
    reaction = Wrench((0.0, 0.0, 1.0), (3.0, 4.0, 0.0))
    view = _recorded(_cantilever_rows(1.0), joints=(Wrench(), reaction))
    bodies = (
        SimpleNamespace(joint=None),
        SimpleNamespace(joint=SimpleNamespace(name="slide")),
        SimpleNamespace(joint=SimpleNamespace(name="base")),
    )
    fastenings = (
        SimpleNamespace(name="base", kind="mount", body_a="frame", body_b="rail_body", joint="base"),
        SimpleNamespace(name="bolts", kind="fastener", body_a="frame", body_b="rail_body", joint="slide"),
    )
    lumped = (SimpleNamespace(member="rail", calibrated=True, calibration_frequency=31.0),)
    build = SimpleNamespace(
        design=_design("0", "3/2"),
        members=(_member(mounts=("base",)),),
        lumped=lumped,
        physics=SimpleNamespace(bodies=bodies, fastenings=fastenings),
    )
    run = run_inputs(build, view, _views())
    (rail,) = run.members
    assert rail.supports == ((0.0, "clamped"),)
    assert rail.natural_frequency == 31.0
    assert rail.section.c_y == pytest.approx(math.sqrt(3.0 * I_Z / AREA))
    assert rail.loads.start_force[0].tolist() == [0.0, 1.0, 0.0]
    mount, bolts = run.joints
    assert (mount.kind, mount.member, mount.force[0].tolist(), mount.moment[0].tolist()) == (
        "mount",
        "rail",
        [3.0, 4.0, 0.0],
        [0.0, 0.0, 1.0],
    )
    assert (bolts.kind, bolts.member, bolts.force[0].tolist()) == ("fastener", None, [0.0, 0.0, 0.0])
    assert run.minimum_safety_factor == 1.5


@pytest.mark.req("INT-022")
def test_section_properties_prefer_both_axes_when_the_section_states_them() -> None:
    # Ruling 25: I_z = second_moment_of_area, I_y = second_moment_y, c_y = fibre_y, c_z = fibre_z.
    section = SimpleNamespace(**vars(_section()), second_moment_y=1e-9, fibre_y=0.03, fibre_z=0.01)
    props = section_properties(section)
    assert (props.i_y, props.i_z, props.c_y, props.c_z, props.single_axis) == (1e-9, I_Z, 0.03, 0.01, False)
    # Single-axis fallback: I_y = I_z, c_y = c_z = sqrt(3 I / A) (solid rectangle), flagged.
    bare = section_properties(_section())
    fibre = math.sqrt(3.0 * I_Z / AREA)
    assert (bare.i_y, bare.c_y, bare.c_z, bare.single_axis) == (I_Z, fibre, fibre, True)
    partial = section_properties(SimpleNamespace(**vars(_section()), second_moment_y=None, fibre_y=0.025, fibre_z=None))
    assert (partial.i_y, partial.c_y, partial.c_z, partial.single_axis) == (I_Z, 0.025, 0.025, True)
    only_z = section_properties(SimpleNamespace(**vars(_section()), second_moment_y=None, fibre_y=None, fibre_z=0.01))
    assert (only_z.c_y, only_z.c_z) == (0.01, 0.01)


@pytest.mark.req("INT-022")
def test_run_inputs_errors_name_what_is_missing() -> None:
    build = SimpleNamespace(design=_design("0"), members=(_member("other"),))
    with pytest.raises(IntegrityError, match="member 'other': no recorded end-loads"):
        run_inputs(build, _recorded(_cantilever_rows(1.0)), _views())
    bare = SimpleNamespace(times=lambda: (0.0,), series=lambda ref: None)
    with pytest.raises(IntegrityError, match="carries no member end-loads"):
        run_inputs(SimpleNamespace(design=_design("0"), members=(_member(),)), bare, _views())  # type: ignore[arg-type]
    physics = SimpleNamespace(
        bodies=(SimpleNamespace(joint=None),),
        fastenings=(SimpleNamespace(name="m", kind="mount", body_a="a", body_b="b", joint="ghost"),),
    )
    with pytest.raises(IntegrityError, match="joint 'ghost' has no recorded reaction"):
        run_inputs(
            SimpleNamespace(design=_design("0"), members=(), physics=physics), _recorded(_cantilever_rows(1.0)), {}
        )
    with pytest.raises(IntegrityError, match="material 'petg_printed': not in the run's material set"):
        inputs([member(cantilever(1.0))], 3).material("petg_printed")
    with pytest.raises(IntegrityError, match="no structural member 'ghost'"):
        inputs([], 1).member("ghost")


# ===== Flags (INT-020, INT-021) ==========================================


@pytest.mark.req("INT-020")
def test_members_and_mounts_below_the_minimum_safety_factor_are_flagged() -> None:
    # Yield min 40 MPa; stress = P L c / I. P chosen for SF 1.0 against a minimum of 1.5: flagged; SF 2.0: not.
    flagged_load = 40e6 * I_Z / (LENGTH * C_Y)
    weak = JointLoads("mount", "mount", "base", np.tile([1e6, 0.0, 0.0], (3, 1)), np.zeros((3, 3)), member="rail")
    run = inputs([member(cantilever(flagged_load))], 3, joints=(weak,), minimum_safety_factor=1.5)
    table = evaluate(IntegrityConfig(schema_version=1), run)
    assert table.member("rail").safety_factor == pytest.approx(1.0)
    assert table.member("rail").flags == (STATIC, "safety_factor_below_minimum")
    assert table.member("mount").flags == ("safety_factor_below_minimum",)
    safe = inputs([member(cantilever(flagged_load / 2.0))], 3, minimum_safety_factor=1.5)
    assert evaluate(IntegrityConfig(schema_version=1), safe).member("rail").flags == (STATIC,)
    unset = inputs([member(cantilever(flagged_load))], 3)
    assert evaluate(IntegrityConfig(schema_version=1), unset).member("rail").flags == (STATIC,)


@pytest.mark.req("INT-021")
def test_members_over_the_deflection_target_are_flagged() -> None:
    load = 50.0
    tip = load * LENGTH**3 / (3.0 * MODULUS * I_Z)
    over = evaluate(
        IntegrityConfig(schema_version=1), inputs([member(cantilever(load))], 2, deflection_limit=tip * 0.99)
    )
    assert over.member("rail").flags == (STATIC, "deflection_over_target")
    under = evaluate(
        IntegrityConfig(schema_version=1), inputs([member(cantilever(load))], 2, deflection_limit=tip * 1.01)
    )
    assert under.member("rail").flags == (STATIC,)


# ===== Per-member table (INT-022) =======================================


@pytest.mark.req("INT-022")
def test_the_table_has_one_row_per_member_and_mount_in_the_results_shape() -> None:
    rows = 4
    loads = cantilever(20.0, rows)
    loads.start_force[2] *= 3.0
    loads.start_moment[2] *= 3.0
    loads.end_force[2] *= 3.0
    mount = JointLoads(
        "mount", "mount", "base", np.tile([5.0, 0.0, 0.0], (rows, 1)), np.zeros((rows, 3)), member="rail"
    )
    loose = JointLoads("loose", "mount", "x", np.zeros((rows, 3)), np.zeros((rows, 3)))
    table = evaluate(
        IntegrityConfig(schema_version=1),
        inputs([member(loads), member(constant(rows), name="idle")], rows, joints=(mount, loose)),
    )
    assert isinstance(table, IntegrityResults)
    assert [entry.member for entry in table.members] == ["rail", "idle", "mount"]
    rail = table.member("rail")
    assert rail.peak_stress == pytest.approx(60.0 * LENGTH * C_Y / I_Z)
    assert (rail.peak_stress_time, rail.deflection_time, rail.safety_factor_time) == (0.002, 0.002, 0.002)
    assert rail.deflection == pytest.approx(60.0 * LENGTH**3 / (3.0 * MODULUS * I_Z))
    assert rail.safety_factor == pytest.approx(40e6 / rail.peak_stress)
    idle = table.member("idle")
    assert (idle.peak_stress, idle.deflection, idle.safety_factor) == (0.0, 0.0, math.inf)
    assert table.member("mount").peak_stress == pytest.approx(5.0 / AREA)
    assert table.member("mount").deflection == 0.0
    assert IntegrityResults.from_json(table.to_json()) == IntegrityResults(table.members)


@pytest.mark.req("INT-022")
def test_the_report_and_load_case_are_written_beside_the_archive(tmp_path: Path) -> None:
    view = _recorded(_cantilever_rows(10.0))
    run = run_inputs(SimpleNamespace(design=_design("0"), members=(_member(),)), view, _views())
    table = evaluate(IntegrityConfig(schema_version=1), run)
    archive = tmp_path / "M1-seed0.h5"
    metadata = SimpleNamespace(
        members=(MemberSection("rail", "petg_printed", AREA, I_Z, LENGTH),),
        design_name="machine",
        design_version="1.2.0",
        run_id="M1-seed0",
    )
    written = write_beside(archive, table, view, metadata)
    assert [path.name for path in written] == [
        "M1-seed0.integrity.json",
        "M1-seed0.load_case.json",
        "M1-seed0.load_case.csv",
    ]
    report = json.loads(written[0].read_text(encoding="utf-8"))
    assert report["table"] == json.loads(table.to_json())
    assert report["load_factor"] == 1.0
    case = json.loads(written[1].read_text(encoding="utf-8"))
    assert case["design"] == {"name": "machine", "version": "1.2.0"}
    assert written[2].read_text(encoding="utf-8").startswith("# design machine 1.2.0; run M1-seed0;")
    plain = write_beside(tmp_path / "plain.h5", IntegrityResults(table.members), view, SimpleNamespace(members=()))
    assert [path.name for path in plain] == ["plain.integrity.json"]
    assert json.loads(plain[0].read_text(encoding="utf-8")) == json.loads(table.to_json())


@pytest.mark.req("INT-022")
def test_the_load_case_gives_peak_force_and_moment_per_member_end() -> None:
    start = Wrench((0.0, 0.0, 5.0), (0.0, 1.0, 0.0))
    view = _recorded([(Wrench(), Wrench()), (start, Wrench((0.0, -6.0, 8.0), (3.0, 4.0, 0.0)))])
    case = load_case(view, (MemberSection("rail", "petg_printed", AREA, I_Z, LENGTH),), "machine", "1.0.0", "r")
    by_key = {(entry.end, entry.quantity): entry for entry in case.entries}
    assert by_key[("end", "force")].magnitude == 5.0
    assert by_key[("end", "force")].direction == (0.6, 0.8, 0.0)
    assert (by_key[("end", "force")].x, by_key[("end", "force")].time) == (LENGTH, 0.001)
    assert by_key[("end", "moment")].magnitude == 10.0
    assert by_key[("start", "moment")].direction == (0.0, 0.0, 1.0)
    assert by_key[("start", "force")].x == 0.0
    rows = case.to_csv().splitlines()
    assert rows[1] == "member,end,x,quantity,unit,time,magnitude,dx,dy,dz"
    assert len(rows) == 6
    with pytest.raises(IntegrityError, match="member 'ghost'"):
        load_case(view, (MemberSection("ghost", "m", 1.0, 1.0, 1.0),), "machine", "1.0.0", "r")
    with pytest.raises(IntegrityError, match="carries no member end-loads"):
        load_case(SimpleNamespace(), (), "machine", "1.0.0", "r")  # type: ignore[arg-type]
    still = load_case(_recorded([(Wrench(), Wrench())]), (MemberSection("rail", "m", 1.0, 1.0, 1.0),), "m", "1", "r")
    assert all(entry.magnitude == 0.0 and entry.direction == (0.0, 0.0, 0.0) for entry in still.entries)


@pytest.mark.req("INT-022")
def test_the_service_evaluates_a_build_with_the_designs_folder_configuration(tmp_path: Path) -> None:
    folder = tmp_path / "machine"
    folder.mkdir()
    (folder / "integrity.yaml").write_text(
        "schema_version: 1\nchecks:\n  - check: strength\n    tier: 2\n", encoding="utf-8"
    )
    build = SimpleNamespace(design=_design("0"), members=(_member(),))
    report = IntegrityService(designs_root=tmp_path)(build, None, _recorded(_cantilever_rows(1.0)))  # type: ignore[arg-type]
    assert isinstance(report, IntegrityReport)
    strength = next(result for result in report.checks if result.check == "strength")
    assert strength.tier == 2
    assert report.member("rail").safety_factor < report.member("rail").peak_stress
    absent = IntegrityService(design_dir=lambda design: tmp_path / "nowhere")
    assert absent(build, None, _recorded(_cantilever_rows(1.0))) is None  # type: ignore[arg-type]
    given = IntegrityService(tmp_path / "nowhere", config=IntegrityConfig(schema_version=1))
    report = given(build, None, _recorded(_cantilever_rows(1.0)))  # type: ignore[arg-type]
    assert report is not None
    assert [result.check for result in report.checks] == ["strength", "deflection"]


# ===== Thermal slot and wear seam (INT-036, INT-037) ====================


@pytest.mark.req("INT-036")
def test_the_thermal_slot_is_accepted_and_not_evaluated() -> None:
    config = parse_config({"schema_version": 1, "thermal": {"parameters": {"limit": "80 degC"}}}, "c")
    assert config.thermal is not None
    report = evaluate(config, inputs([member(cantilever(1.0))], 2))
    assert THERMAL_NOTE in report.notes
    assert [result.check for result in report.checks] == ["strength", "deflection"]
    assert "thermal" not in report.report_json().split('"checks"')[1].split('"cycle_counts"')[0]


@pytest.mark.req("INT-037")
def test_cycle_counts_are_recorded_for_the_wear_seam() -> None:
    rows = 41
    loads = constant(rows)
    loads.start_moment[:, 2] = np.cos(2.0 * math.pi * np.arange(rows) / 10.0)
    config = parse_config({"schema_version": 1, "checks": [{"check": "fatigue"}]}, "c")
    report = evaluate(config, inputs([member(loads)], rows))
    assert report.cycle_counts() == {"rail": 4.0}
    assert json.loads(report.report_json())["cycle_counts"] == {"rail": 4.0}


# ===== Check edge cases =================================================


@pytest.mark.req("INT-008")
def test_rainflow_matches_the_astm_e1049_example() -> None:
    # ASTM E1049-85 figure 6 history; table: range 3 half, 4 one and a half, 6 half, 8 one, 9 half.
    counted: dict[float, float] = {}
    for stress_range, count in rainflow([-2.0, 1.0, -3.0, 5.0, -1.0, 3.0, -4.0, 4.0, -2.0]):
        counted[stress_range] = counted.get(stress_range, 0.0) + count
    assert counted == {3.0: 0.5, 4.0: 1.5, 6.0: 0.5, 8.0: 1.0, 9.0: 0.5}


@pytest.mark.req("INT-008")
def test_reversals_drop_repeats_and_monotone_runs() -> None:
    assert reversals([0.0, 0.0, 1.0, 2.0, 2.0, 1.0, 3.0]) == [0.0, 2.0, 1.0, 3.0]
    assert reversals([]) == []
    assert rainflow([1.0]) == []


@pytest.mark.req("INT-008")
def test_cycles_to_failure_covers_every_part_of_the_curve() -> None:
    curve = ((1e3, 100e6), (1e6, 40e6), (1e7, 40e6))
    assert cycles_to_failure(curve, 39e6) == math.inf
    assert cycles_to_failure(curve, 0.0) == math.inf
    assert cycles_to_failure(curve, 40e6) == 1e7
    assert cycles_to_failure(curve, 100e6) == pytest.approx(1e3)
    exponent = math.log(1e6 / 1e3) / math.log(100e6 / 40e6)
    assert cycles_to_failure(curve, 200e6) == pytest.approx(1e3 * 0.5**exponent)
    assert cycles_to_failure(((1e3, 50e6), (1e6, 50e6)), 60e6) == 1e3
    with pytest.raises(IntegrityError, match="two points"):
        cycles_to_failure(((1e3, 50e6),), 1.0)


@pytest.mark.req("INT-008")
def test_fatigue_flags_damage_at_the_limit() -> None:
    rows = 3
    loads = constant(rows)
    loads.start_moment[:, 2] = np.asarray([0.0, 100e6 * I_Z / C_Y, 0.0])
    # 0 -> S -> 0 is two half cycles of range S = 100 MPa (amplitude 50 MPa): N = 1e3 * 1e3^(ln 2 / ln 2.5), about
    # 1.86e5, so D is about 5.4e-6: over a limit of 1e-6.
    config = parse_config(
        {"schema_version": 1, "checks": [{"check": "fatigue", "parameters": {"damage_limit": 1e-6}}]}, "c"
    )
    report = evaluate(config, inputs([member(loads)], rows))
    assert "fatigue_damage_over_limit" in report.member("rail").flags
    quiet = evaluate(config, inputs([member(constant(rows))], rows))
    assert quiet.member("rail").flags == (STATIC,)


@pytest.mark.req("INT-006")
def test_deflection_needs_supports_that_hold_the_member() -> None:
    with pytest.raises(IntegrityError, match="free to move"):
        deflection(member(constant(1), ((0.0, "pinned"),)), constant(1))


@pytest.mark.req("INT-007", "INT-012")
def test_closed_forms_need_end_supports() -> None:
    interior = member(constant(1), ((0.0, "clamped"), (0.2, "pinned")))
    with pytest.raises(IntegrityError, match="interior"):
        effective_length_factor(interior)
    pinned_free = member(constant(1), ((0.0, "pinned"),))
    with pytest.raises(IntegrityError, match="no effective length"):
        effective_length_factor(pinned_free)
    with pytest.raises(IntegrityError, match="no natural frequency"):
        first_frequency(pinned_free)


@pytest.mark.req("INT-007")
def test_buckling_skips_tension_members_and_honours_an_override() -> None:
    tension = constant(2, start_force=(-5.0, 0.0, 0.0), end_force=(5.0, 0.0, 0.0))
    result = Buckling().evaluate(inputs([member(tension)], 2), BucklingParameters(), 1)
    assert result.rows == ()
    assert result.notes == ("buckling: member 'rail' is never in compression",)
    squeeze = constant(2, start_force=(5.0, 0.0, 0.0), end_force=(-5.0, 0.0, 0.0))
    run = inputs([member(squeeze)], 2, minimum_safety_factor=1e9)
    (row,) = Buckling().evaluate(run, BucklingParameters(effective_length_factors={"rail": 1.0}), 1).rows
    assert dict(row.detail)["effective_length_factor"] == 1.0
    assert row.flags == ("buckling_margin_below_minimum",)


@pytest.mark.req("INT-010")
def test_fastener_edges() -> None:
    rows = 2
    force = np.tile([0.0, 0.0, 300.0], (rows, 1))
    over = JointLoads("bolts", "fastener", "j", force, np.zeros((rows, 3)))
    rated = FastenerParameters.model_validate({"ratings": {"bolts": "200 N", "idle": "200 N"}})
    (row,) = Fastener().evaluate(inputs([], rows, joints=(over,)), rated, 1).rows
    assert row.flags == ("fastener_margin_below_minimum",)
    idle = JointLoads("idle", "fastener", "j", np.zeros((rows, 3)), np.zeros((rows, 3)))
    (row,) = Fastener().evaluate(inputs([], rows, joints=(idle,)), rated, 1).rows
    assert row.margin is not None
    assert row.margin.typ == math.inf
    within = JointLoads("bolts", "fastener", "j", force, np.zeros((rows, 3)))
    (row,) = (
        Fastener()
        .evaluate(
            inputs([], rows, joints=(within,), minimum_safety_factor=2.0),
            FastenerParameters.model_validate({"ratings": {"bolts": "500 N"}}),
            1,
        )
        .rows
    )
    assert row.flags == ("fastener_margin_below_minimum",)
    bare = JointLoads("bare", "fastener", "j", force, np.zeros((rows, 3)))
    with pytest.raises(IntegrityError, match="fastener 'bare': no load rating"):
        Fastener().evaluate(inputs([], rows, joints=(bare,)), FastenerParameters(), 1)


@pytest.mark.req("INT-011")
def test_impact_events_are_one_per_window_and_include_mounts() -> None:
    times = np.arange(10) * 1e-3
    load = np.asarray([0.0, 10.0, 20.0, 30.0, 30.0, 30.0, 0.0, 50.0, 50.0, 50.0])
    assert events(load, times, 5.0, 2e-3) == [1, 7]
    assert events(load, times, 5.0, 0.0) == [1, 2, 3, 7]
    assert events(load, times, 100.0, 1.0) == []
    rows = 6
    force = np.zeros((rows, 3))
    force[3:, 0] = 40.0
    mount = JointLoads("mount", "mount", "base", force, np.zeros((rows, 3)), member="rail")
    parameters = ImpactParameters.model_validate({"impact_factor": 1.5, "step_threshold": "10 N", "window": "1 s"})
    rows_out = Impact().evaluate(inputs([member(constant(rows))], rows, joints=(mount,)), parameters, 1).rows
    assert [(row.subject, row.time) for row in rows_out] == [("mount", 0.003)]
    assert rows_out[0].value == pytest.approx(1.5 * 40.0 / AREA)


@pytest.mark.req("INT-012")
def test_resonance_needs_its_series_and_skips_empty_ones() -> None:
    def missing(reference: str) -> Any:
        raise KeyError(reference)

    run = RunInputs(np.zeros(1), (member(constant(1)),), {}, TEMPERATURE, series=missing)
    parameters = ResonanceParameters.model_validate(
        {"band": 0.1, "excitations": [{"series": "joint:x.velocity", "hz_per_unit": 1.0}]}
    )
    with pytest.raises(IntegrityError, match=r"no series 'joint:x\.velocity'"):
        Resonance().evaluate(run, parameters, 1)
    empty = RunInputs(
        np.zeros(1), (member(constant(1), frequency=5.0),), {}, TEMPERATURE, series=lambda _: (np.zeros(0), np.zeros(0))
    )
    assert Resonance().evaluate(empty, parameters, 1).rows == ()


@pytest.mark.req("INT-022")
def test_peak_of_an_empty_run() -> None:
    assert peak(np.zeros(0), np.zeros(0)) == (0.0, 0.0)
    assert peak(np.zeros((0, 2)), np.asarray([]).reshape(0)) == (0.0, 0.0)
    assert peak(np.zeros(0), np.asarray([2.0])) == (0.0, 2.0)
    assert Wrenches(*(np.zeros((0, 3)) for _ in range(4))).scaled(2.0).start_force.shape == (0, 3)
    assert SECTION.i_min() == SECTION.i_y


@pytest.mark.req("INT-011", "INT-020")
def test_impact_rows_carry_their_own_margin_flag() -> None:
    rows = 4
    loads = cantilever(40e6 * I_Z / (LENGTH * C_Y), rows)
    for array in (loads.start_force, loads.start_moment, loads.end_force):
        array[:2] = 0.0
    parameters = ImpactParameters.model_validate({"impact_factor": 1.0, "step_threshold": "1 N", "window": "1 s"})
    (row,) = Impact().evaluate(inputs([member(loads)], rows, minimum_safety_factor=2.0), parameters, 1).rows
    assert row.flags == ("impact_margin_below_minimum",)
    (row,) = Impact().evaluate(inputs([member(loads)], rows), parameters, 1).rows
    assert row.flags == ()


@pytest.mark.req("INT-005")
def test_the_member_axis_comes_from_the_lumped_placement() -> None:
    # Placement turned +90 deg about z: q = (cos 45, 0, 0, sin 45) maps x to y.
    half = math.sqrt(0.5)
    lumped = (
        SimpleNamespace(name="rail_chain", structural_member="rail", orientation=(half, 0.0, 0.0, half)),
        SimpleNamespace(name="other", structural_member=None, orientation=(1.0, 0.0, 0.0, 0.0)),
    )
    axes = member_axes(SimpleNamespace(physics=SimpleNamespace(lumped_members=lumped)))
    assert axes["rail"] == pytest.approx((0.0, 1.0, 0.0), abs=1e-15)
    assert axes["other"] == (1.0, 0.0, 0.0)
    assert member_axes(SimpleNamespace(physics=None)) == {}
    build = SimpleNamespace(
        design=_design("0"),
        members=(_member(),),
        physics=SimpleNamespace(lumped_members=lumped, bodies=(), fastenings=()),
    )
    assert run_inputs(build, _recorded(_cantilever_rows(1.0)), _views()).members[0].axis_body == pytest.approx(
        (0.0, 1.0, 0.0), abs=1e-15
    )
    rigid = run_inputs(
        SimpleNamespace(design=_design("0"), members=(_member(),)), _recorded(_cantilever_rows(1.0)), _views()
    )
    assert rigid.members[0].axis_body == (1.0, 0.0, 0.0)
