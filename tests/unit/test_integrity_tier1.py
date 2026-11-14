"""Tier 1 integrity party from Python: the limits table (INT-023, invariant 2), the cross-check against the post-run
strength check, the case halt field (INT-026, TST-041), rope snap routing (INT-031) and the core wiring (INT-024,
INT-025, INT-026, ruling 34)."""

from __future__ import annotations

import itertools
import json
import math
import re
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import jsonschema
import numpy as np
import pytest

from mechsim import _core as core
from mechsim.facade.protocol import RunEndedPayload
from mechsim.facade.server import run_end_reason
from mechsim.facade.service import Facade, FacadeServices
from mechsim.integrity.checks.strength import Strength
from mechsim.integrity.config import IntegrityConfig, parse_config
from mechsim.integrity.inputs import run_inputs
from mechsim.integrity.plugin import IntegrityError
from mechsim.integrity.table import window_note
from mechsim.integrity.tier1 import (
    BEFORE_EVALUATION_KIND,
    FORMAT,
    MODEL_REQUESTS,
    NOT_EVALUATED,
    PARTY,
    YIELD_RULE,
    Tier1Service,
    enabled,
    first_springs,
    limits_table,
    published_names,
    tier1_enabled,
)
from mechsim.materials.database import MaterialSet
from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.neutral import PhysicsModel, from_yaml
from mechsim.record.layout import MEMBER_SERIES
from mechsim.record.reader import RecordArchiveReader
from mechsim.record.stream import Recorder
from mechsim.scoring.score import SuiteScore, report_text
from mechsim.testing.core_driver import (
    ROPE_SNAP_OFF,
    CoreDriver,
    RecordArchiver,
    _rope_snap_gate,
    core_builder,
    model_columns,
    tick_record,
)
from mechsim.testing.driver import INTEGRITY_KIND, EndSpec, IntegrityCrossing, RunReport
from mechsim.testing.runner import Services, run_case
from mechsim.testing.schema import DESIGNS_ROOT, Case
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


def native_of(model: PhysicsModel) -> dict[str, Any]:
    document: dict[str, Any] = json.loads(prepare_model(model, None, 1e-3).model_json)
    return document


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


def _document(kinds: tuple[str, ...], neutral: int, segments: tuple[int, ...]) -> dict[str, Any]:
    bodies = [{"name": f"b{i}", "joint": {"type": kind}} for i, kind in enumerate(kinds)]
    return {"bodies": bodies, "neutral_bodies": neutral, "members": [{"segments": n} for n in segments]}


@pytest.mark.req("INT-023")
def test_first_springs_come_from_the_prepared_native_model() -> None:
    # Spring order of the native adapter (core/physics/native/CLAUDE.md): revolute and prismatic one each, planar
    # three, fixed, spherical and free none, neutral bodies only (the world anchor after them adds nothing); then N
    # per member. Expected [0 + 1 + 0 + 3 + 0 + 1 + 0, +8, +16] = [5, 13, 29].
    kinds = ("revolute", "fixed", "planar", "free", "prismatic", "spherical", "revolute")
    assert first_springs(_document(kinds, 6, (8, 16, 4))) == [5, 13, 29]
    assert first_springs(_document((), 0, ())) == []
    assert first_springs(native_of(cantilever_model("revolute"))) == [1]
    assert first_springs(native_of(cantilever_model())) == [0]


@pytest.mark.req("INT-023")
def test_an_unknown_joint_type_in_the_spring_order_is_an_error() -> None:
    with pytest.raises(IntegrityError, match="'b1': joint type 'helical'"):
        first_springs(_document(("revolute", "helical"), 2, (8,)))


@pytest.mark.req("INT-023")
def test_a_native_member_list_that_differs_from_the_model_is_an_error() -> None:
    model = cantilever_model()
    native = native_of(model)
    native["members"] = []
    with pytest.raises(IntegrityError, match="native model lists 0 members, the physics model 1"):
        limits_table(build_of(model), model, config({"check": "strength"}), views(), native)


@pytest.mark.req("INT-023")
def test_the_limits_table_carries_the_member_data_and_conservative_allowable() -> None:
    model = cantilever_model()
    structural = SimpleNamespace(minimum_safety_factor=2.0, accuracy_target=0.01)
    table = limits_table(build_of(model, structural), model, config({"check": "strength"}), views(), native_of(model))
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


@pytest.mark.req("INT-006", "INT-023")
def test_node_indices_and_the_body_count_come_from_the_prepared_native_model() -> None:
    # The prepared document lists the build's bodies plus two a case added at run time (a payload and its shackle)
    # before the member's nodes: the chain starts after all of them, its parent is found by name, and the table states
    # the exact published count, neutral bodies plus N node bodies.
    model = cantilever_model()
    structural = SimpleNamespace(minimum_safety_factor=2.0, accuracy_target=0.01)
    native = native_of(model)
    neutral = int(native["neutral_bodies"])
    plain = limits_table(build_of(model, structural), model, config({"check": "deflection"}), views(), native)
    assert plain["bodies"] == neutral + SEGMENTS
    assert plain["members"][0]["nodes"]["first_body"] == neutral
    extra = [{"name": "payload_shackle", "joint": {"type": "fixed"}}, {"name": "payload", "joint": {"type": "free"}}]
    native["bodies"] = [*native["bodies"][:neutral], *extra, *native["bodies"][neutral:]]
    native["neutral_bodies"] = neutral + 2
    names = published_names(model, native)
    assert names[neutral : neutral + 2] == ("payload_shackle", "payload")
    assert names[neutral + 2 :] == tuple(f"{model.lumped_members[0].name}.node{k}" for k in range(SEGMENTS))
    table = limits_table(build_of(model, structural), model, config({"check": "deflection"}), views(), native)
    assert table["bodies"] == neutral + 2 + SEGMENTS
    nodes = table["members"][0]["nodes"]
    assert (nodes["first_body"], nodes["count"]) == (neutral + 2, SEGMENTS)
    assert nodes["parent_body"] == plain["members"][0]["nodes"]["parent_body"]
    core.check_integrity_limits(json.dumps(table))


@pytest.mark.req("INT-006", "INT-023")
def test_the_node_names_are_counted_from_the_native_document_not_the_model() -> None:
    # The document's member entry is what the adapter loads: its `segments`, plus the pin segment's node 0 when its
    # `ends` is simply_supported (a pinned root), sets the published node count; the model's member names them.
    model = cantilever_model()
    native = native_of(model)
    neutral = int(native["neutral_bodies"])
    name = model.lumped_members[0].name
    native["members"][0]["segments"] = 3
    assert published_names(model, native)[neutral:] == tuple(f"{name}.node{k}" for k in range(3))
    native["members"][0]["ends"] = "simply_supported"
    assert published_names(model, native)[neutral:] == tuple(f"{name}.node{k}" for k in range(4))
    native["members"] = []
    with pytest.raises(IntegrityError, match="the native model lists 0 members, the physics model 1"):
        published_names(model, native)


@pytest.mark.req("INT-023", "INT-010")
def test_a_fastening_without_a_rating_is_a_build_error() -> None:
    model = cantilever_model()
    data = model.model_dump(mode="json")
    data["fastenings"] = [{"name": "bolts", "kind": "fastener", "body_a": "link", "body_b": "link", "joint": "pivot"}]
    rated = PhysicsModel.model_validate(data)
    with pytest.raises(IntegrityError, match="bolts"):
        limits_table(build_of(rated), rated, config({"check": "fastener"}), views(), native_of(rated))
    rating = config({"check": "fastener", "parameters": {"ratings": {"bolts": "500 N"}}})
    table = limits_table(build_of(rated), rated, rating, views(), native_of(rated))
    assert table["fastenings"] == [{"name": "bolts", "joint": 0, "load_limit": 500.0, "member": None}]
    # The rule of the post-run check (`fastener.rating`): a mount is rated when given a rating, else left out.
    data["fastenings"].append({"name": "base", "kind": "mount", "body_a": "link", "body_b": "link", "joint": "pivot"})
    mounted = PhysicsModel.model_validate(data)
    table = limits_table(build_of(mounted), mounted, rating, views(), native_of(mounted))
    assert [entry["name"] for entry in table["fastenings"]] == ["bolts"]
    both = config({"check": "fastener", "parameters": {"ratings": {"bolts": "500 N", "base": "2 kN"}}})
    table = limits_table(build_of(mounted), mounted, both, views(), native_of(mounted))
    assert table["fastenings"] == [
        {"name": "bolts", "joint": 0, "load_limit": 500.0, "member": None},
        {"name": "base", "joint": 0, "load_limit": 2000.0, "member": None},
    ]


@pytest.mark.req("INT-023", "INT-028")
def test_the_service_needs_the_file_and_a_tier_1_check(tmp_path: Path) -> None:
    model = cantilever_model()
    build = build_of(model)
    folder = tmp_path / "bench"
    folder.mkdir()
    native = native_of(model)
    service = Tier1Service(tmp_path)
    assert service.configuration(build) is None
    assert service(build, model, native) is None
    (folder / "integrity.yaml").write_text("schema_version: 1\nchecks:\n  - check: fatigue\n", encoding="utf-8")
    assert service.configuration(build) is not None
    assert service(build, model, native) is None
    (folder / "integrity.yaml").write_text(
        "schema_version: 1\nchecks:\n  - check: strength\n    tier: 2\n", encoding="utf-8"
    )
    setup = service(build, model, native)
    assert setup is not None
    assert setup.yield_offset
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
    table = limits_table(build, model, config({"check": "strength"}), views(), native_of(model))
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


# ===== Yielding cantilever on the native adapter (INT-024, INT-027, INT-028) =====


def yielding_cantilever() -> tuple[PhysicsModel, str]:
    """The cantilever with I_z / 100 and a fastener rated 1 N on the link's fixed joint, with strength at tier 2 and
    the fastener check: released straight under gravity, the mount reaction (the link and beam weight, about 9.9 N)
    crosses at tick 1, and the root stress (static sag rho A g L^2 c_y / (2 I_z) = 2.1e8 Pa, dynamic up to twice that)
    passes the 1.1e8 Pa allowable and then the root spring's moment passes M_y, so the yield offset grows each tick
    while the first swing deepens. Returns the model and its limits table (JSON)."""
    data = cantilever_model().model_dump(mode="json")
    data["lumped_members"][0]["section"]["second_moment_of_area"] = f"{I_Z / 100} m**4"
    data["fastenings"] = [{"name": "bolts", "kind": "fastener", "body_a": "link", "body_b": "link", "joint": "pivot"}]
    model = PhysicsModel.model_validate(data)
    checks = config(
        {"check": "strength", "tier": 2}, {"check": "fastener", "parameters": {"ratings": {"bolts": "1 N"}}}
    )
    table = limits_table(build_of(model), model, checks, views(), native_of(model))
    return model, json.dumps(table)


def run_yielding(ticks: int) -> tuple[Any, list[tuple[int, str, str, str]]]:
    """The yielding cantilever through the kernel with the physics party wired to the party's requests (KER-028);
    every recorded event as (row tick, source, kind, detail)."""
    model, limits = yielding_cantilever()
    prepared = prepare_model(model, None, 1e-3)
    adapter = core.make_physics_adapter("native")
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(1000))
    physics = core.PhysicsPartyConfig()
    physics.rate = core.Rate(1000)
    physics.inputs = []
    physics.model_requests = MODEL_REQUESTS
    kernel = core.make_kernel(core.RunConfig())
    kernel.register_party(core.PhysicsParty(adapter, physics))
    trace = core.IntegrityTrace()
    settings = core.IntegrityPartyConfig()
    settings.name = PARTY
    settings.rate = core.Rate(1000)
    kernel.register_party(core.IntegrityParty(limits, settings, trace))
    sink = core.InMemoryRecorderSink()
    kernel.run(core.EndCondition(core.Seconds(ticks, 1000)), sink)
    events = [
        (int(row.time.count()), event.source, event.kind, event.detail) for row in sink.drain() for event in row.events
    ]
    return trace, events


@pytest.mark.req("INT-028", "KER-028")
def test_a_rising_peak_gives_one_model_change_per_request_with_a_monotone_offset() -> None:
    """Ruling 35: the offset grows only with the running peak, so under the deepening first swing every request is
    larger in magnitude than the one before, keeps one sign, goes to the root spring 0, and comes back exactly once as
    a model_change event in the row after the request's (applied before the next step, ruling 40)."""
    trace, events = run_yielding(300)
    requests = list(trace.requests)
    assert len(requests) > 1
    changes = [(tick, detail) for tick, _, kind, detail in events if kind == "model_change"]
    assert len(changes) == len(requests)
    values = [request.value for request in requests]
    assert all(value < 0.0 for value in values)
    assert all(abs(b) > abs(a) for a, b in itertools.pairwise(values))
    for request, (_, detail) in zip(requests, changes, strict=True):
        assert request.index == 0
        assert detail.startswith("spring_offset spring 0 value ")
        assert float(detail.rsplit(" ", 1)[1]) == request.value
    ticks = [tick for tick, _ in changes]
    assert ticks == sorted(set(ticks))


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


def gantry_run(tmp_path: Path, adapter: str, tier1: IntegrityConfig | None, extra: str = "") -> tuple[Any, CoreDriver]:
    path = G1
    if extra:
        path = tmp_path / G1.name
        path.write_text(G1.read_text(encoding="utf-8").rstrip() + "\n" + extra + "\n", encoding="utf-8")
    drivers: list[CoreDriver] = []

    def factory() -> CoreDriver:
        # Without tier 1 is explicit: a designs folder holding no integrity.yaml, never the shipped
        # designs/gantry/integrity.yaml.
        service = Tier1Service(tmp_path / "no-integrity")
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
    # The limit is the rating over the gantry's minimum safety factor 1.5 (the fastener check's margin rule).
    assert (crossing.member, crossing.quantity, crossing.tier) == ("rail_x_mount", "fastener_load", 1)
    assert crossing.limit == pytest.approx(1e-9 / 1.5, rel=1e-15)
    assert event.detail == crossing.detail()
    assert event.source == "integrity"
    # The first row is stamped at the end of the first step (1 ms): the mount carries the rail from the start.
    assert event.time == pytest.approx(1e-3, abs=1e-15)
    assert driver.integrity_trace.crossings[0].value == crossing.value
    assert result.score.status == "fail"
    assert len(result.score.crossings) == 1
    assert result.report.end_tick == TICKS


def archived_run(tmp_path: Path, tier1: IntegrityConfig, extra: str) -> Any:
    """A native gantry run with the archiver, its case written beside the archives the archiver searches."""
    path = tmp_path / "cases" / G1.name
    path.parent.mkdir()
    path.write_text(G1.read_text(encoding="utf-8").rstrip() + "\n" + extra + "\n", encoding="utf-8")

    def factory() -> CoreDriver:
        return ShortDriver(tier1=Tier1Service(config=tier1))

    services = Services(factory, core_builder, RecordArchiver(DESIGNS_ROOT, path.parent))
    return path, services


HALT_LINE = re.compile(
    r"^integrity halt: the run ended at 0\.001 s on the fastener_load of 'rail_x_mount' at (\S+) against the limit "
    r"(\S+) at 0\.001 s \(INT-026\)$"
)


@pytest.mark.req("INT-026", "TST-041", "KER-017", "SCO-015")
@pytest.mark.parametrize(
    ("extra", "halts"),
    [
        ("", False),
        ("integrity: {}", False),
        ("integrity:\n  halt_on_crossing: false", False),
        ("integrity:\n  halt_on_crossing: true", True),
    ],
)
def test_only_the_case_field_true_halts_at_the_crossing(tmp_path: Path, extra: str, halts: bool) -> None:
    # Section absent, empty or false: the run goes to its end time and fails on the crossing (INT-025); true: it ends
    # at the crossing row with end reason integrity_halt and the halt line among the report notes, never among the
    # approximations (rulings 50, 51).
    result, driver = gantry_run(tmp_path, "native", mount_config(), extra)
    assert len(integrity_events(driver)) == 1
    assert result.score.status == "fail"
    assert not any(line.startswith("integrity halt") for line in result.report.approximations)
    if halts:
        assert result.report.end_tick == 1
        assert result.report.reason == "integrity_halt"
        assert result.score.end_reason == "integrity_halt"
        (line,) = result.report.notes
        match = HALT_LINE.match(line)
        assert match is not None, line
        crossing = driver.integrity_trace.crossings[0]
        assert float(match.group(1)) == pytest.approx(crossing.value, rel=1e-11)
        assert float(match.group(2)) == pytest.approx(1e-9 / 1.5, rel=1e-11)
        # The halt line reaches the score and the printed report as a note, never an approximation (invariant 7).
        assert result.score.notes == (line,)
        text = report_text(SuiteScore("s", result.score.design, (result.score,)))
        assert f"  note: {line}" in text.splitlines()
        assert f"  approximation: {line}" not in text.splitlines()
    else:
        assert result.report.end_tick == TICKS
        assert result.report.reason == "end_time"
        assert result.report.notes == ()
        assert result.score.notes == ()


@pytest.mark.req("TST-044", "INT-024", "INT-025", "SCO-015")
def test_a_crossing_before_the_evaluation_start_is_recorded_and_does_not_fail_the_case(tmp_path: Path) -> None:
    # The mount's vanishing rating is crossed from tick 1 on. From 5 ms: the tick-1 crossing is one
    # integrity_before_evaluation event and the first counted crossing is tick 5, which fails the case. From 30 ms
    # (past the 20-tick run): the early event only, and the case scores as the run without tier 1 does.
    result, driver = gantry_run(tmp_path, "native", mount_config(), "integrity:\n  evaluate_from: 5 ms")
    early = [event for event in driver.recording().events() if event.kind == BEFORE_EVALUATION_KIND]
    assert [round(event.time * 1000) for event in early] == [1]
    assert [round(event.time * 1000) for event in integrity_events(driver)] == [5]
    assert IntegrityCrossing.parse(early[0].detail).member == "rail_x_mount"
    assert len(driver.integrity_trace.before_evaluation) == 1
    assert result.score.status == "fail"
    assert window_note(0.005) in result.report.approximations
    late, quiet = gantry_run(tmp_path, "native", mount_config(), "integrity:\n  evaluate_from: 30 ms")
    assert integrity_events(quiet) == []
    assert len([event for event in quiet.recording().events() if event.kind == BEFORE_EVALUATION_KIND]) == 1
    assert len(quiet.integrity_trace.crossings) == 0
    assert list(quiet.integrity_trace.peak_fastener_load) == [0.0]
    plain, _ = gantry_run(tmp_path, "native", None)
    assert late.score.status == plain.score.status


@pytest.mark.req("TST-043", "TST-044")
def test_the_case_integrity_fields_default_and_validate() -> None:
    settings = accepted(pendulum_case())
    assert settings.integrity is None
    data = pendulum_case()
    data["integrity"] = {}
    empty = accepted(data).integrity
    assert empty is not None
    assert (empty.design_load_carried, empty.evaluate_from) == (False, 0.0)
    data["integrity"] = {"design_load_carried": True, "evaluate_from": "250 ms"}
    given = accepted(data).integrity
    assert given is not None
    assert (given.design_load_carried, given.evaluate_from) == (True, 0.25)
    for field, wrong in (
        ("design_load_carried", "yes"),
        ("design_load_carried", 1),
        ("evaluate_from", "-1 s"),
        ("evaluate_from", "2 m"),
        ("evaluate_from", 0.5),
    ):
        data = pendulum_case()
        data["integrity"] = {field: wrong}
        rejected(data, field)
    schema = json.loads((ROOT / "schemas" / "test_case.json").read_text(encoding="utf-8"))
    data = pendulum_case()
    data["integrity"] = {"design_load_carried": True, "evaluate_from": "2 s"}
    jsonschema.validate(data, schema)
    properties = Case.model_json_schema()["$defs"]["IntegritySettings"]["properties"]
    assert properties["design_load_carried"]["default"] is False
    assert properties["evaluate_from"]["default"] == 0.0


@pytest.mark.req("INT-026", "REC-010", "FAC-005")
def test_the_halt_end_reason_reaches_the_archive_and_the_facade(tmp_path: Path) -> None:
    path, services = archived_run(tmp_path, mount_config(), "integrity:\n  halt_on_crossing: true")
    facade = Facade(FacadeServices(runner=services, reader=RecordArchiveReader()))
    summary = facade.run_case(path, None, {"physics.adapter": "native"}, tmp_path / "runs")
    assert summary.end_reason == "integrity_halt"
    assert run_end_reason(summary) == "integrity_halt"
    ended = RunEndedPayload(run_id="G1-seed1", t=0.001, reason=run_end_reason(summary), message="run ended")
    assert ended.reason == "integrity_halt"
    assert summary.archive is not None
    metadata, _ = RecordArchiveReader().open(Path(summary.archive))
    assert metadata.report.reason == "integrity_halt"
    assert len(metadata.report.notes) == 1
    assert HALT_LINE.match(metadata.report.notes[0]) is not None
    assert summary.notes == list(metadata.report.notes)
    assert f"  note: {metadata.report.notes[0]}" in summary.report_text.splitlines()


@pytest.mark.req("INT-026")
def test_a_halt_value_in_integrity_yaml_is_refused() -> None:
    with pytest.raises(IntegrityError, match="halt_on_crossing"):
        parse_config({"schema_version": 1, "halt_on_crossing": True, "checks": []}, "test")


class _CountingService(Tier1Service):
    def __init__(self, config: IntegrityConfig) -> None:
        super().__init__(config=config)
        self.tables = 0

    def __call__(self, build: Any, model: Any, native: Any) -> Any:
        self.tables += 1
        raise IntegrityError("fastener 'bolts': no load rating")


@pytest.mark.req("INT-023")
def test_the_bridge_never_builds_the_limits_table(tmp_path: Path) -> None:
    """Ruling 34: tier 1 runs on the native adapter only, so a table error (an unrated fastener) cannot fail a bridge
    run, which carries the not-evaluated line instead; the same configuration on the native adapter is a run error."""
    services_of: list[_CountingService] = []

    def factory() -> CoreDriver:
        service = _CountingService(mount_config())
        services_of.append(service)
        return ShortDriver(tier1=service)

    bridge = run_case(G1, Services(factory, core_builder, None), {"physics.adapter": "mujoco"})
    assert bridge.report is not None
    assert not bridge.report.errored
    assert NOT_EVALUATED in bridge.report.approximations
    assert services_of[-1].tables == 0
    native = run_case(G1, Services(factory, core_builder, None), {"physics.adapter": "native"})
    assert native.score.status == "error"
    assert native.score.error is not None
    assert "integrity tier 1: fastener 'bolts': no load rating" in native.score.error
    assert services_of[-1].tables == 1


# ===== Rope snap gating (INT-031, ruling 53) =============================


@pytest.mark.req("INT-031")
def test_without_a_rope_snap_entry_no_rope_snaps() -> None:
    model = snapping_rope()
    for integrity in (None, config(), config({"check": "strength", "tier": 2})):
        gated, lines = _rope_snap_gate(model, integrity)
        assert [rope.breaking_tension for rope in gated.ropes] == [None]
        assert lines == (ROPE_SNAP_OFF.format("line"),)
        recording = record(gated, 300)
        assert [event for _, event in recording.events if event.kind == "model_change"] == []


@pytest.mark.req("INT-031")
def test_a_rope_snap_entry_keeps_the_breaking_tension_and_the_rope_snaps() -> None:
    model = snapping_rope()
    gated, lines = _rope_snap_gate(model, config({"check": "rope_snap"}))
    assert gated == model
    assert lines == ()
    recording = record(gated, 300)
    kinds = [event.detail.split(" ")[0] for _, event in recording.events if event.kind == "model_change"]
    assert kinds == ["rope_snapped", "load_released"]


@pytest.mark.req("INT-031")
def test_a_rope_without_a_breaking_tension_adds_no_line() -> None:
    model = cantilever_model()
    gated, lines = _rope_snap_gate(model, None)
    assert gated is model
    assert lines == ()


SPRING = ROOT / "designs" / "rope_spring" / "design.yaml"
SNAP_CASE = """id: S9
title: Stretched rope released
design:
  name: rope_spring
  version: 1.0.0
seed: 0
initial_state:
  joints: []
physics:
  adapter: native
end:
  at: 0.02 s
  max_duration: 1 s
criteria:
  - name: settle
    check: value
    quantity: pose:bob.y
    statistic: final
    comparison: below
    limit: 0 m
"""


def stretched_rope_run(tmp_path: Path, tier1: IntegrityConfig) -> tuple[Any, CoreDriver]:
    """The rope_spring design with its bob placed 0.02 m below the rope's rest length: the one segment starts at
    T = EA (0.02 m) / L0 = 10 kN, over the part's 5 kN breaking tension, so the rope snaps in the first tick when
    rope_snap is on (INT-031) and the bob oscillates on it when off."""
    root = tmp_path / "designs"
    (root / "rope_spring").mkdir(parents=True)
    text = SPRING.read_text(encoding="utf-8").replace('default: "-1 m"', 'default: "-1.02 m"')
    (root / "rope_spring" / "design.yaml").write_text(text, encoding="utf-8")
    case = tmp_path / "S9.yaml"
    case.write_text(SNAP_CASE, encoding="utf-8")
    drivers: list[CoreDriver] = []

    def factory() -> CoreDriver:
        driver = CoreDriver(designs_root=root, tier1=Tier1Service(config=tier1))
        drivers.append(driver)
        return driver

    result = run_case(case, Services(factory, core_builder, None, designs_root=root), None)
    assert result.report is not None, result.score.error
    assert not result.report.errored, result.report.error
    return result, drivers[0]


@pytest.mark.req("INT-031")
@pytest.mark.parametrize("entry", [True, False])
def test_the_rope_snap_gate_reaches_the_run_and_its_report_through_the_driver(tmp_path: Path, entry: bool) -> None:
    entries: tuple[dict[str, Any], ...] = ()
    if entry:
        entries = ({"check": "rope_snap"},)
    result, driver = stretched_rope_run(tmp_path, config(*entries))
    kinds = [event.detail.split(" ")[0] for event in driver.recording().events() if event.kind == "model_change"]
    off = ROPE_SNAP_OFF.format("line")
    if entry:
        assert kinds == ["rope_snapped", "load_released"]
        assert off not in result.report.approximations
        assert off not in result.score.approximations
    else:
        assert kinds == []
        assert off in result.report.approximations
        assert f"  approximation: {off}" in report_text(SuiteScore("s", result.score.design, (result.score,)))
