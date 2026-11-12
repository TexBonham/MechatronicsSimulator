"""Tier 1 integrity party from Python: the limits table (INT-023, invariant 2), the cross-check against the post-run
strength check, the case halt field (INT-026, TST-041), rope snap routing (INT-031) and the core wiring (INT-024,
INT-025, INT-026, ruling 34)."""

from __future__ import annotations

import json
import math
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import jsonschema
import numpy as np
import pytest

from mechsim import _core as core
from mechsim.integrity.checks.strength import Strength
from mechsim.integrity.config import IntegrityConfig, parse_config
from mechsim.integrity.inputs import run_inputs
from mechsim.integrity.plugin import IntegrityError
from mechsim.integrity.tier1 import (
    FORMAT,
    NOT_EVALUATED,
    YIELD_RULE,
    Tier1Service,
    enabled,
    first_springs,
    limits_table,
    tier1_enabled,
)
from mechsim.materials.database import MaterialSet
from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.neutral import PhysicsModel, from_yaml
from mechsim.record.layout import MEMBER_SERIES
from mechsim.record.stream import Recorder
from mechsim.testing.core_driver import CoreDriver, core_builder, model_columns, tick_record
from mechsim.testing.driver import INTEGRITY_KIND, EndSpec, IntegrityCrossing, RunReport
from mechsim.testing.runner import Services, run_case
from mechsim.testing.schema import Case
from tests.unit.test_integrity import _View
from tests.unit.test_native_model_changes import BREAKING, G, record, snapping_rope
from tests.unit.test_test_case_schema import accepted, pendulum_case, rejected

ROOT = Path(__file__).resolve().parents[2]
PENDULUM = (ROOT / "schemas/examples/physics_model.pendulum.yaml").read_text()
MATERIAL = "aluminium_6063_t5_extrusion"
E = 6.9e10
I_Z = 2e-9
I_Y = 8e-9
AREA = 1e-4
FIBRE_Y = 0.005
FIBRE_Z = 0.01
LENGTH = 0.8
SEGMENTS = 8


def cantilever_model(joint: str = "fixed") -> PhysicsModel:
    """The pendulum example's link held fixed to the world (its frame turned -90 deg about z), carrying a clamped
    aluminium cantilever turned +90 deg so it lies along world x, bending about z under gravity along -y."""
    data = from_yaml(PENDULUM).model_dump(mode="json")
    data["signals"] = {}
    link = data["bodies"][0]
    link["joint"]["type"] = joint
    if joint == "fixed":
        link["joint"]["angular"] = []
    half = math.sqrt(0.5)
    data["lumped_members"] = [
        {
            "name": "arm",
            "parent": "link",
            "position": ("0 m", "0 m", "0 m"),
            "orientation": (half, 0.0, 0.0, half),
            "section": {
                "elastic_modulus": f"{E} Pa",
                "second_moment_of_area": f"{I_Z} m**4",
                "density": "2700 kg/m**3",
                "area": f"{AREA} m**2",
                "length": f"{LENGTH} m",
                "second_moment_y": f"{I_Y} m**4",
                "fibre_y": f"{FIBRE_Y} m",
                "fibre_z": f"{FIBRE_Z} m",
            },
            "root": "clamped",
            "end": "free",
            "segments": SEGMENTS,
        }
    ]
    return PhysicsModel.model_validate(data)


def build_of(model: PhysicsModel, structural: Any = None) -> Any:
    member = model.lumped_members[0]
    support = SimpleNamespace(at=0.0, condition="clamped")
    arm = SimpleNamespace(name="arm", material=MATERIAL, section=member.section, supports=(support,), mounts=())
    design = SimpleNamespace(
        name="bench", structural=structural, environment=SimpleNamespace(ambient_temperature=293.15), materials=None
    )
    return SimpleNamespace(design=design, members=(arm,), lumped=(), physics=model)


def views() -> dict[str, Any]:
    return {MATERIAL: MaterialSet.from_files(None).integrity_view(MATERIAL)}


def config(*entries: dict[str, Any]) -> IntegrityConfig:
    return parse_config({"schema_version": 1, "checks": list(entries)}, "test")


# ===== Limits table (INT-023) ===========================================


@pytest.mark.req("INT-023", "INT-028")
@pytest.mark.parametrize(
    ("entries", "on"),
    [
        ((), ()),
        (({"check": "fatigue"},), ()),
        (({"check": "strength"},), ("stress",)),
        (({"check": "strength", "tier": 2},), ("stress", "yield_offset")),
        (({"check": "buckling"}, {"check": "deflection"}), ("buckling", "deflection")),
        (({"check": "fastener"},), ("fastener",)),
    ],
)
def test_listed_checks_turn_the_tier_1_quantities_on(entries: tuple[dict[str, Any], ...], on: tuple[str, ...]) -> None:
    flags = enabled(config(*entries))
    assert sorted(name for name, value in flags.items() if value) == sorted(on)
    assert tier1_enabled(config(*entries)) == any(name != "yield_offset" for name in on)


@pytest.mark.req("INT-023")
def test_first_springs_count_joint_dofs_then_member_nodes() -> None:
    # Spring order of the native adapter: revolute and prismatic one each, planar three, others none; then N per member.
    bodies = [SimpleNamespace(joint=SimpleNamespace(type=kind)) for kind in ("revolute", "fixed", "planar", "free")]
    bodies.append(SimpleNamespace(joint=None))
    members = [SimpleNamespace(segments=8), SimpleNamespace(segments=16), SimpleNamespace(segments=4)]
    model = SimpleNamespace(bodies=bodies, lumped_members=members)
    assert first_springs(model) == [4, 12, 28]
    assert first_springs(cantilever_model("revolute")) == [1]


@pytest.mark.req("INT-023")
def test_the_limits_table_carries_the_member_data_and_conservative_allowable() -> None:
    model = cantilever_model()
    structural = SimpleNamespace(minimum_safety_factor=2.0, accuracy_target=0.01)
    table = limits_table(build_of(model, structural), model, config({"check": "strength"}), views())
    assert table["format"] == FORMAT
    (member,) = table["members"]
    yield_min = MaterialSet.from_files(None).integrity_view(MATERIAL).strength_spread("yield", 293.15, (1, 0, 0)).min
    assert member["allowable_stress"] == pytest.approx(float(yield_min) / 2.0, rel=1e-15)
    assert member["yield_strength"] == pytest.approx(float(yield_min), rel=1e-15)
    assert (member["i_y"], member["i_z"], member["c_y"], member["c_z"]) == pytest.approx((I_Y, I_Z, FIBRE_Y, FIBRE_Z))
    assert member["segment_length"] == pytest.approx(LENGTH / SEGMENTS)
    assert member["node_stiffness"] == pytest.approx(E * I_Z * SEGMENTS / LENGTH)
    assert (member["first_spring"], member["segments"], member["index"]) == (0, SEGMENTS, 0)
    assert member["effective_length_factor"] == 2.0
    assert member["deflection_limit"] == 0.01
    assert table["minimum_safety_factor"] == 2.0
    core.check_integrity_limits(json.dumps(table))
    bad = dict(table, members=[dict(member, area=-1.0)])
    with pytest.raises(ValueError, match="area"):
        core.check_integrity_limits(json.dumps(bad))


@pytest.mark.req("INT-023", "INT-010")
def test_a_fastening_without_a_rating_is_a_build_error() -> None:
    model = cantilever_model()
    data = model.model_dump(mode="json")
    data["fastenings"] = [{"name": "bolts", "kind": "fastener", "body_a": "link", "body_b": "link", "joint": "pivot"}]
    rated = PhysicsModel.model_validate(data)
    with pytest.raises(IntegrityError, match="bolts"):
        limits_table(build_of(rated), rated, config({"check": "fastener"}), views())
    table = limits_table(
        build_of(rated), rated, config({"check": "fastener", "parameters": {"ratings": {"bolts": "500 N"}}}), views()
    )
    assert table["fastenings"] == [{"name": "bolts", "joint": 0, "load_limit": 500.0}]


@pytest.mark.req("INT-023", "INT-028")
def test_the_service_needs_the_file_and_a_tier_1_check(tmp_path: Path) -> None:
    model = cantilever_model()
    build = build_of(model)
    folder = tmp_path / "bench"
    folder.mkdir()
    service = Tier1Service(tmp_path)
    assert service(build, model) is None
    (folder / "integrity.yaml").write_text("schema_version: 1\nchecks:\n  - check: fatigue\n", encoding="utf-8")
    assert service(build, model) is None
    (folder / "integrity.yaml").write_text(
        "schema_version: 1\nhalt_on_crossing: true\nchecks:\n  - check: strength\n    tier: 2\n", encoding="utf-8"
    )
    setup = service(build, model)
    assert setup is not None
    assert setup.yield_offset and setup.halt_on_crossing
    assert setup.approximations == (YIELD_RULE,)
    assert json.loads(setup.limits)["checks"]["yield_offset"] is True


# ===== Cross-check against the post-run strength check (INT-023, INT-005) =====


def _run_party(model: PhysicsModel, limits: str, ticks: int) -> tuple[Any, Any]:
    prepared = prepare_model(model, None, 1e-3)
    adapter = core.make_physics_adapter("native")
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(1000))
    physics = core.PhysicsPartyConfig()
    physics.rate = core.Rate(1000)
    physics.inputs = []
    kernel = core.make_kernel(core.RunConfig())
    kernel.register_party(core.PhysicsParty(adapter, physics))
    trace = core.IntegrityTrace()
    settings = core.IntegrityPartyConfig()
    settings.rate = core.Rate(1000)
    kernel.register_party(core.IntegrityParty(limits, settings, trace))
    sink = core.InMemoryRecorderSink()
    kernel.run(core.EndCondition(core.Seconds(ticks, 1000)), sink)
    recorder = Recorder(Fraction(1000), model_columns(model))
    for row in sink.drain():
        recorder.write_row(tick_record(row))
    return trace, recorder.recording()


@pytest.mark.req("INT-023", "INT-005")
def test_the_party_peak_stress_equals_the_post_run_strength_peak() -> None:
    """A cantilever released straight under gravity vibrates about its sag; the party evaluates every tick what the
    recorder writes. Its free end carries no load, so the resolved interior point has zero moment and the ends govern:
    the party's peak equals strength.py's peak over the recorded end-loads (load factor 1, no structural section)."""
    model = cantilever_model()
    build = build_of(model)
    table = limits_table(build, model, config({"check": "strength"}), views())
    trace, recording = _run_party(model, json.dumps(table), 400)
    assert all(path in recording.series for path in MEMBER_SERIES)
    inputs = run_inputs(build, _View(recording), views())
    (row,) = Strength().evaluate(inputs, Strength.parameters(), 1).rows
    print(f"cross-check: party peak {trace.peak_stress[0]!r} Pa, strength.py peak {row.value!r} Pa")
    assert row.value > 0.0
    assert trace.peak_stress[0] == pytest.approx(row.value, rel=1e-9)
    # Static sag stress at the root, rho A g L^2 / 2 c_y / I_z, is below the vibrating peak (dynamic factor up to 2).
    static = 2700.0 * AREA * 9.81 * LENGTH**2 / 2.0 * FIBRE_Y / I_Z
    assert static < row.value < 2.0 * static * 1.01


# ===== Case field (INT-026, TST-041) =====================================


@pytest.mark.req("INT-026", "TST-041")
def test_the_case_halt_field_defaults_false_and_is_strictly_boolean() -> None:
    absent = accepted(pendulum_case())
    assert absent.integrity is None
    for value in (False, True):
        data = pendulum_case()
        data["integrity"] = {"halt_on_crossing": value}
        case = accepted(data)
        assert case.integrity is not None
        assert case.integrity.halt_on_crossing is value
    data = pendulum_case()
    data["integrity"] = {}
    settings = accepted(data).integrity
    assert settings is not None
    assert settings.halt_on_crossing is False
    for wrong in ("yes", 1, None):
        data = pendulum_case()
        data["integrity"] = {"halt_on_crossing": wrong}
        rejected(data, "halt_on_crossing")
    data = pendulum_case()
    data["integrity"] = {"halt": True}
    rejected(data, "halt")


@pytest.mark.req("TST-041")
def test_the_generated_schema_carries_the_halt_field() -> None:
    schema = json.loads((ROOT / "schemas" / "test_case.json").read_text(encoding="utf-8"))
    data = pendulum_case()
    data["integrity"] = {"halt_on_crossing": True}
    jsonschema.validate(data, schema)
    data["integrity"] = {"halt_on_crossing": "true"}
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(data, schema)
    assert Case.model_json_schema()["$defs"]["IntegritySettings"]["properties"]["halt_on_crossing"]["default"] is False


# ===== Rope snap routing (INT-031) =======================================


@pytest.mark.req("INT-031", "ENG-034")
def test_a_snapped_rope_drops_its_load_in_free_fall() -> None:
    """The engine lists RopeSnapped then LoadReleased in the snap tick (ruling 40) and both reach the recorder as
    model_change events (ruling 26); from the next step the load is free. Semi-implicit Euler under constant g gives
    z_{n+1} - 2 z_n + z_{n-1} = -g h^2 exactly, so the second difference over h^2 is -g to rounding (1e-9 relative,
    the free-fall reference tolerance of the engine's exact recurrence)."""
    recording = record(snapping_rope(), 300)
    changes = [(tick, event) for tick, event in recording.events if event.kind == "model_change"]
    assert [event.detail.split(" ")[0] for _, event in changes] == ["rope_snapped", "load_released"]
    assert changes[0][0] == changes[1][0]
    assert float(changes[0][1].detail.rsplit(" ", 1)[1]) > BREAKING
    assert all(event.kind != "integrity" for _, event in recording.events)
    snap = changes[0][0]
    times = np.asarray(recording.series["time/seconds"].values, dtype=np.float64)
    row = int(np.searchsorted(np.rint(times * 1000.0), snap))
    z = np.asarray(recording.series["physics/bodies/position"].values, dtype=np.float64)[:, 0, 2]
    h = 1e-3
    tail = z[row + 2 :]
    acceleration = (tail[2:] - 2.0 * tail[1:-1] + tail[:-2]) / h**2
    print(f"rope snap: free-fall acceleration {acceleration.min()!r} to {acceleration.max()!r}, closed form {-G}")
    assert acceleration.size > 100
    assert acceleration == pytest.approx(np.full(acceleration.shape, -G), rel=1e-9)


# ===== Core wiring (INT-024, INT-025, INT-026, ruling 34) ================

G1 = ROOT / "test_cases" / "gantry" / "G1.yaml"
TICKS = 20


class ShortDriver(CoreDriver):
    def run(self, end: EndSpec) -> RunReport:
        return super().run(EndSpec(max_tick=TICKS, at_tick=TICKS))


def mount_config(rating: str = "1e-9 N") -> IntegrityConfig:
    """The fastener check with a vanishing rating on the gantry's X rail mount: its reaction (the rail's weight) passes
    it in the first tick."""
    return config({"check": "fastener", "parameters": {"ratings": {"rail_x_mount": rating}}})


def gantry_run(
    tmp_path: Path, adapter: str, tier1: IntegrityConfig | None, extra: str = ""
) -> tuple[Any, CoreDriver]:
    path = G1
    if extra:
        path = tmp_path / G1.name
        path.write_text(G1.read_text(encoding="utf-8").rstrip() + "\n" + extra + "\n", encoding="utf-8")
    drivers: list[CoreDriver] = []

    def factory() -> CoreDriver:
        service = None
        if tier1 is not None:
            service = Tier1Service(config=tier1)
        driver = ShortDriver(tier1=service)
        drivers.append(driver)
        return driver

    services = Services(factory, core_builder, None)
    result = run_case(path, services, {"physics.adapter": adapter})
    assert result.report is not None
    assert not result.report.errored, result.report.error
    return result, drivers[0]


def integrity_events(driver: CoreDriver) -> list[Any]:
    return [event for event in driver.recording().events() if event.kind == INTEGRITY_KIND]


@pytest.mark.req("INT-023")
def test_without_tier_1_no_party_is_registered(tmp_path: Path) -> None:
    result, driver = gantry_run(tmp_path, "native", None)
    assert driver.integrity_trace is None
    assert integrity_events(driver) == []
    assert not any(line.startswith("integrity") for line in result.report.approximations)
    off, other = gantry_run(tmp_path, "native", config({"check": "fatigue"}))
    assert other.integrity_trace is None
    assert off.report.approximations == result.report.approximations


@pytest.mark.req("INT-023")
def test_the_bridge_registers_no_party_and_says_so(tmp_path: Path) -> None:
    result, driver = gantry_run(tmp_path, "mujoco", mount_config())
    assert driver.integrity_trace is None
    assert integrity_events(driver) == []
    assert NOT_EVALUATED in result.report.approximations


@pytest.mark.req("INT-024", "INT-025", "REC-010", "SCO-015")
def test_a_native_crossing_is_one_integrity_event_and_fails_the_case(tmp_path: Path) -> None:
    result, driver = gantry_run(tmp_path, "native", mount_config())
    events = integrity_events(driver)
    assert len(events) == 1
    (event,) = events
    crossing = IntegrityCrossing.parse(event.detail)
    assert (crossing.member, crossing.quantity, crossing.limit, crossing.tier) == ("rail_x_mount", "fastener_load", 1e-9, 1)
    assert event.detail == crossing.detail()
    assert event.source == "integrity"
    # The first row is stamped at the end of the first step (1 ms): the mount carries the rail from the start.
    assert event.time == pytest.approx(1e-3, abs=1e-15)
    assert driver.integrity_trace.crossings[0].value == crossing.value
    assert result.score.status == "fail"
    assert len(result.score.crossings) == 1
    assert result.report.end_tick == TICKS


@pytest.mark.req("INT-026", "TST-041", "KER-017")
@pytest.mark.parametrize(("extra", "halts"), [("", False), ("integrity:\n  halt_on_crossing: false", False),
                                              ("integrity:\n  halt_on_crossing: true", True)])
def test_halting_ends_the_run_at_the_crossing(tmp_path: Path, extra: str, halts: bool) -> None:
    result, driver = gantry_run(tmp_path, "native", mount_config(), extra)
    assert len(integrity_events(driver)) == 1
    assert result.score.status == "fail"
    halt_lines = [line for line in result.report.approximations if line.startswith("integrity halt:")]
    if halts:
        assert result.report.end_tick == 1
        assert result.report.reason == "end_time"
        assert halt_lines == ["integrity halt: the run ended at 0.001 s on its first tier 1 crossing (INT-026)"]
    else:
        assert result.report.end_tick == TICKS
        assert halt_lines == []


@pytest.mark.req("INT-026")
def test_the_file_mirror_halts_when_the_case_says_nothing(tmp_path: Path) -> None:
    mirrored = parse_config(
        {
            "schema_version": 1,
            "halt_on_crossing": True,
            "checks": [{"check": "fastener", "parameters": {"ratings": {"rail_x_mount": "1e-9 N"}}}],
        },
        "test",
    )
    result, _ = gantry_run(tmp_path, "native", mirrored)
    assert result.report.end_tick == 1
    kept, _ = gantry_run(tmp_path, "native", mirrored, "integrity:\n  halt_on_crossing: false")
    assert kept.report.end_tick == TICKS
