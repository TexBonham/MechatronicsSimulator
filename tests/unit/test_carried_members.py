"""Load-path coupling of a carried joint on a flexible member (ENG-052, BLD-037, DES-033, PHY-035; ruling 63): the
design field and its checks, the build's travel check and mapping, the neutral references, the native translation and
a run of a small data-only design through the native adapter and the kernel."""

from __future__ import annotations

import json
from fractions import Fraction
from typing import Any

import numpy as np
import pytest
from pydantic import ValidationError

from mechsim import _core as core
from mechsim.builder import BuildError, build
from mechsim.design.schema import Design, shipped_materials
from mechsim.integrity.beam import node_deflection
from mechsim.integrity.inputs import BODY_ORIENTATION, BODY_POSITION, MEMBER_SERIES, node_chains
from mechsim.integrity.plugin import BodyPoses, MemberInputs, SectionProperties, Wrenches
from mechsim.library.registry import default_registry
from mechsim.physics.native import translate
from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.neutral import PhysicsModel
from mechsim.record.layout import Recording
from mechsim.record.stream import Recorder
from mechsim.testing.core_driver import model_columns, tick_record
from tests.unit.test_builder_support import MATERIAL

AREA = 4e-4
SECOND_MOMENT = 0.02**4 / 12.0
CUBE = 0.04


def _design_data(
    carries: list[str], position: float = 0.46875, limits: tuple[float, float] | None = (-0.1, 0.1)
) -> dict[str, Any]:
    """A 1 m aluminium bar (20 mm square section, pinned root, roller end, sixteen segments) fixed to the world along x,
    gravity -y in its bending plane (ruling 65), and a 40 mm cube carriage on a prismatic joint along the bar whose
    parent is the bar itself (the lumped chain's parent)."""
    linear: dict[str, Any] = {"friction": "0 N", "damping": "0 N*s/m", "backlash": "0 m"}
    if limits is not None:
        linear["limits"] = {"lower": f"{limits[0]} m", "upper": f"{limits[1]} m"}
    box = {"shape": "box", "position": ["0 m", "0 m", "0 m"], "size": [f"{CUBE} m", f"{CUBE} m", f"{CUBE} m"]}
    bar_box = {"shape": "box", "position": ["0.5 m", "0 m", "0 m"], "size": ["1 m", "0.02 m", "0.02 m"]}
    return {
        "name": "carried_bar",
        "version": "1.0.0",
        "source_hash": "unset",
        "status": "release",
        "environment": {"gravity": ["0 m/s**2", "-9.81 m/s**2", "0 m/s**2"], "ambient_temperature": "293.15 K"},
        "bodies": [
            {
                "name": "bar",
                "material": MATERIAL,
                "process": {"kind": "extrusion"},
                "geometry": {"kind": "primitives", "shapes": [bar_box]},
                "flexible": {
                    "section": {"area": f"{AREA} m**2", "second_moment": f"{SECOND_MOMENT} m**4", "length": "1 m"},
                    "supports": ["pinned", "roller"],
                    "segments": 16,
                    "carries": carries,
                },
            },
            {
                "name": "carriage",
                "material": MATERIAL,
                "process": {"kind": "extrusion"},
                "geometry": {"kind": "primitives", "shapes": [box]},
            },
        ],
        "joints": [
            {
                "name": "bar_mount",
                "type": "fixed",
                "parent": "world",
                "child": "bar",
                "placement": {"position": ["0 m", "0 m", "0 m"]},
            },
            {
                "name": "slide",
                "type": "prismatic",
                "parent": "bar",
                "child": "carriage",
                "placement": {"position": [f"{position} m", "0 m", "0 m"]},
                "axis": [1.0, 0.0, 0.0],
                "linear": [linear],
            },
        ],
    }


def _design(carries: list[str], **options: Any) -> Design:
    return Design.model_validate(_design_data(carries, **options))


def _physics(design: Design) -> PhysicsModel:
    result = build(design, default_registry(), shipped_materials())
    assert result.physics is not None
    return result.physics


def _failures(design: Design) -> list[str]:
    with pytest.raises(BuildError) as raised:
        build(design, default_registry(), shipped_materials())
    return raised.value.failures


# ===== Design and build =================================================


@pytest.mark.req("DES-033", "ENG-052")
def test_the_design_refuses_an_unknown_joint_a_wrong_parent_and_a_joint_carried_twice() -> None:
    with pytest.raises(ValidationError, match="unknown joint 'nowhere'"):
        _design(["nowhere"])
    with pytest.raises(ValidationError, match="a carried joint's parent is the member's own body 'bar'"):
        _design(["bar_mount"])
    with pytest.raises(ValidationError, match="joint 'slide' is already carried by 'bar'"):
        _design(["slide", "slide"])
    assert _design(["slide"]).bodies[0].flexible is not None


@pytest.mark.req("BLD-037", "ENG-052")
def test_the_build_maps_a_carried_joint_to_its_child_body_on_the_neutral_member() -> None:
    member = _physics(_design(["slide"])).lumped_members[0]
    assert member.carries == ("carriage",)
    assert member.parent == "bar"
    assert _physics(_design([])).lumped_members[0].carries == ()
    rigid = build(_design(["slide"]), default_registry(), shipped_materials(), rigid_members=True)
    assert rigid.physics is not None
    assert rigid.physics.lumped_members == ()


@pytest.mark.req("BLD-037", "ENG-052")
@pytest.mark.parametrize(
    ("position", "limits", "fragment"),
    [
        (0.95, (-0.1, 0.1), "over x in [0.85, 1.05] m, outside the member's span [0, 1] m by 0.05 m"),
        (0.05, (-0.1, 0.1), "over x in [-0.05, 0.15] m, outside the member's span [0, 1] m by 0.05 m"),
        (0.5, None, "travel along the member is unbounded (no limits)"),
    ],
)
def test_the_build_refuses_a_carried_joint_whose_travel_leaves_the_span(
    position: float, limits: tuple[float, float] | None, fragment: str
) -> None:
    failures = _failures(_design(["slide"], position=position, limits=limits))
    assert len(failures) == 1
    assert "carried joint 'slide'" in failures[0]
    assert fragment in failures[0]


@pytest.mark.req("BLD-037")
def test_travel_exactly_to_both_ends_of_the_span_is_accepted() -> None:
    assert _physics(_design(["slide"], position=0.5, limits=(-0.5, 0.5))).lumped_members[0].carries == ("carriage",)


@pytest.mark.req("ENG-052")
def test_the_neutral_model_refuses_a_wrong_parent_an_unknown_body_and_a_body_carried_twice() -> None:
    data = _physics(_design(["slide"])).model_dump(mode="json")
    data["lumped_members"][0]["carries"] = ["bar"]
    with pytest.raises(ValidationError, match="carried body 'bar' is a child of 'world'"):
        PhysicsModel.model_validate(data)
    data["lumped_members"][0]["carries"] = ["ghost"]
    with pytest.raises(ValidationError, match="unknown carried body 'ghost'"):
        PhysicsModel.model_validate(data)
    data["lumped_members"][0]["carries"] = ["carriage", "carriage"]
    with pytest.raises(ValidationError, match="'carriage' is carried by more than one lumped member"):
        PhysicsModel.model_validate(data)


# ===== Native translation ===============================================


@pytest.mark.req("PHY-035", "ENG-052")
def test_the_translation_writes_the_carried_body_index_and_the_approximation_lines() -> None:
    model = _physics(_design(["slide"]))
    out = translate.translate(model, step=1e-3)
    document = json.loads(out.text)
    names = [body["name"] for body in document["bodies"]]
    assert document["members"][0]["carried"] == [names.index("carriage")]
    assert any("carries 'carriage' by load-path coupling" in line for line in out.report)
    assert any("out-of-plane bending and torsion are rigid (ruling 65)" in line for line in out.report)
    plain = translate.translate(_physics(_design([])), step=1e-3)
    assert "carried" not in json.loads(plain.text)["members"][0]
    assert not any("load-path coupling" in line for line in plain.report)
    assert any("out-of-plane bending and torsion are rigid" in line for line in plain.report)


# ===== A run through the native adapter =================================


def _record(model: PhysicsModel, ticks: int, document: str | None = None) -> Recording:
    prepared = prepare_model(model, None, 1e-3)
    text = prepared.model_json
    if document is not None:
        text = document
    adapter = core.make_physics_adapter("native")
    adapter.load(core.PreparedModel(text, "", prepared.version), core.Rate(1000))
    party_config = core.PhysicsPartyConfig()
    party_config.rate = core.Rate(1000)
    party_config.inputs = []
    kernel = core.make_kernel(core.RunConfig())
    kernel.register_party(core.PhysicsParty(adapter, party_config))
    sink = core.InMemoryRecorderSink()
    kernel.run(core.EndCondition(core.Seconds(ticks, 1000)), sink)
    recorder = Recorder(Fraction(1000), model_columns(model))
    for row in sink.drain():
        recorder.write_row(tick_record(row))
    return recorder.recording()


def _last(recording: Recording, path: str, width: int) -> np.ndarray:
    values = np.asarray(recording.series[path].values, dtype=np.float64)
    return values.reshape(-1, width)[-1]


def _deflection(model: PhysicsModel, recording: Recording) -> float:
    chain = node_chains(model)["bar"]
    positions = np.asarray(recording.series[BODY_POSITION].values, dtype=np.float64)
    orientations = np.asarray(recording.series[BODY_ORIENTATION].values, dtype=np.float64)
    count = len(model.bodies) + chain.count
    poses = BodyPoses(positions.reshape(-1, count, 3)[-1:], orientations.reshape(-1, count, 4)[-1:])
    zero = np.zeros((1, 3))
    section = SectionProperties(AREA, 1.0, 1.0, 1.0, SECOND_MOMENT, SECOND_MOMENT, 0.01, 0.01)
    member = MemberInputs("bar", MATERIAL, section, ((0.0, "pinned"),), Wrenches(zero, zero, zero, zero), nodes=chain)
    return float(node_deflection(member, poses)[0])


def _chain_compliance(segments: int, a: float) -> float:
    """The pinned-pinned chain's deflection at a under a unit load at a, times E I (lumped_beam.hpp unit-load method,
    the lever rule onto the two load nodes around a; reciprocity gives the deflection under the load)."""
    span = 1.0 / segments
    nodes = [0.0] + [(j + 0.5) * span for j in range(segments)] + [1.0]
    k = 0
    while k + 2 < len(nodes) and nodes[k + 1] <= a:
        k += 1
    w = (nodes[k + 1] - a) / (nodes[k + 1] - nodes[k])
    loads = ((nodes[k], w), (nodes[k + 1], 1.0 - w))

    def moment(x: float) -> float:
        total = 0.0
        for at, share in loads:
            if x <= at:
                total += share * (1.0 - at) * x
            else:
                total += share * at * (1.0 - x)
        return total

    return sum(moment((j + 0.5) * span) ** 2 * span for j in range(segments))


@pytest.mark.req("ENG-052", "PHY-035", "INT-006")
def test_a_carried_carriage_loads_the_member_through_the_native_adapter() -> None:
    """The carriage (weight P) rests at a = 7.5 l = 0.46875 m (l = L / 16), exactly on spring node 7, on a pinned-roller
    bar under its own weight. Carried minus uncarried isolates P: the support shears differ by P b / L at the pin and P
    a / L at the roller, and the integrity check's node-pose deflection (the largest node distance from the pin-to-far-
    end chord; self weight symmetric about midspan, so nodes 7 and 8 hold it equally and node 7 is the largest in both
    runs) by the deflection under the load, the chain value P C(a) / (E I) and the continuum P a^2 b^2 / (3 E I L)
    within the 2 percent row (the chain at N = 16, a = 0.46875: +0.78 percent; at N = 8 on node 3, a = 0.4375, it is
    +3.2 percent, outside the row; ruling 57).
    Settling: no node damping; backward Euler damps the first mode (omega = pi^2 sqrt(E I / (rho A L^4)), 106 rad/s)
    by zeta = omega h / 2 = 0.053, so after 2 s the start transient is e^(-zeta omega t) = e^(-11) of the static
    deflection, 1.7e-5 relative: the deflection and shear asserts take 1e-4 relative against the chain and statics.
    Geometry: total deflection about 2e-4 m, (delta / L)^2 = 4e-8 relative, below that."""
    carried = _physics(_design(["slide"]))
    plain = _physics(_design([]))
    section = carried.lumped_members[0].section
    ei = float(section.elastic_modulus) * float(section.second_moment_of_area)
    weight = float(next(body.mass for body in carried.bodies if body.name == "carriage")) * 9.81
    a = 0.46875
    b = 1.0 - a
    first = _record(carried, 2000)
    reference = _record(plain, 2000)
    loaded = _deflection(carried, first) - _deflection(plain, reference)
    shears = [
        _last(first, MEMBER_SERIES[0], 3)[1] - _last(reference, MEMBER_SERIES[0], 3)[1],
        _last(first, MEMBER_SERIES[2], 3)[1] - _last(reference, MEMBER_SERIES[2], 3)[1],
    ]
    chain = weight * _chain_compliance(16, a) / ei
    continuum = weight * a * a * b * b / (3.0 * ei)
    print(f"carried through the native adapter: deflection {loaded!r}, chain {chain!r}, continuum {continuum!r}")
    print(f"  shears {shears!r}, P b / L {weight * b!r}, P a / L {weight * a!r}")
    assert loaded == pytest.approx(chain, rel=1e-4)
    assert loaded == pytest.approx(continuum, rel=0.02)
    assert shears[0] == pytest.approx(weight * b, rel=1e-4)
    assert shears[1] == pytest.approx(weight * a, rel=1e-4)
    second = _record(carried, 2000)
    assert sorted(second.series) == sorted(first.series)
    for path, series in first.series.items():
        left = np.asarray(series.values)
        right = np.asarray(second.series[path].values)
        assert left.dtype == right.dtype, path
        if left.dtype.kind in "fiub":
            assert left.tobytes() == right.tobytes(), path
        else:
            assert left.tolist() == right.tolist(), path
    assert second.events == first.events


@pytest.mark.req("PHY-035")
def test_the_adapter_records_the_run_largest_carried_deflection() -> None:
    prepared = prepare_model(_physics(_design(["slide"])), None, 1e-3)
    adapter = core.NativeAdapter()
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(1000))
    assert adapter.carried_deflection() == [0.0]
    for _ in range(20):
        adapter.step()
    assert adapter.carried_deflection()[0] > 0.0


@pytest.mark.req("PHY-035", "ENG-052")
def test_a_run_reports_each_carrying_member_s_deflection_and_lag_after_the_run() -> None:
    """The shared record outlives the adapter the party takes (as CoreDriver reads it at run end): the carrying member's
    largest deflection is the adapter's own value, and the parent fixed to the world gives the feed-forward line."""
    model = _physics(_design(["slide"]))
    prepared = prepare_model(model, None, 1e-3)
    adapter = core.NativeAdapter()
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(1000))
    record = adapter.carried_record()
    assert record.values() == [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, 0.0)]
    for _ in range(50):
        adapter.step()
    deflection = adapter.carried_deflection()[0]
    assert deflection > 0.0
    assert record.values()[0][:4] == (deflection, 0.0, 0.0, 0.0)
    lines = translate.carried_member_lines(model, record.values())
    assert len(lines) == 1
    assert lines[0].startswith("lumped member 'bar_beam': largest deflection at a carried joint over the run")
    assert f"{deflection:.6g} m" in lines[0]
    assert "parent fixed across the member" in lines[0]
    assert translate.carried_member_lines(_physics(_design([])), [(0.0,) * 10]) == ()
    free = translate.carried_member_lines(model, [(1e-4, 0.25, 0.1, 0.999)])
    assert "lag ratio r 0.25 against its boundary (1 + x^2) / 2 = 0.505 at omega h 0.1" in free[0]
    assert "largest per-step factor 0.999" in free[0]


@pytest.mark.req("PHY-035", "ENG-052")
def test_a_run_reports_each_carrying_member_s_work_ratio_and_validity() -> None:
    """Brief M3-E7c (ruling 69): the record carries the engine's work, largest force, quasi-static deflection and
    ratio; the run-end lines state them per carrying member, and a flagged member gets the not-valid line with its
    time. The resting carriage loads the undamped member released from straight under gravity: positive work. Judged
    from the release (evaluation start 0) it is flagged: the ratio is formed relative to the own-weight shape g (ruling
    74, member_loads.hpp Definition (b)), and the own weight released from straight rings about it, so abs(d - g)
    reaches 2 C W + g against the base C W, a ratio near 2 + g / (C W), above 2.04 whenever g > 0.04 C W (the 1 m bar
    outweighs the 40 mm cube several times). That is the release the evaluation start exists for (Definition (d)):
    from a start after the run's 50 steps the ratio stays 0 and nothing is flagged. (Under the E7c and E7e base,
    abs(g + C W), the same run read below the limit.)"""
    assert translate.CARRIED_RATIO_LIMIT == core.CARRIED_RATIO_LIMIT
    model = _physics(_design(["slide"]))
    late = prepare_model(model, None, 1e-3, None, 1, 0.1)
    held = core.NativeAdapter()
    held.load(core.PreparedModel(late.model_json, "", late.version), core.Rate(1000))
    for _ in range(50):
        held.step()
    assert held.carried_record().values()[0][7:] == (0.0, False, 0.0)
    prepared = prepare_model(model, None, 1e-3)
    adapter = core.NativeAdapter()
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(1000))
    record = adapter.carried_record()
    for _ in range(50):
        adapter.step()
    value = record.values()[0]
    work, force, quasi_static, ratio, unresolved, since = value[4:]
    assert work > 0.0
    assert force > 0.0
    assert quasi_static > 0.0
    assert ratio > core.CARRIED_RATIO_LIMIT
    assert unresolved is True
    assert 0.0 <= since < 0.05
    lines = translate.carried_validity_lines(model, record.values())
    assert len(lines) == 2
    assert lines[0].startswith("lumped member 'bar_beam': the carried load-path pair did")
    assert f"{work:.6g} J" in lines[0]
    assert f"static deflection at the foot) {ratio:.6g} against the limit 2.04" in lines[0]
    assert "from the evaluation start the deflection relative to the own-weight shape stayed within 2.04" in lines[0]
    flagged = translate.carried_validity_lines(model, [(1e-3, 0.0, 0.0, 0.0, 10.7, 50.0, 1e-3, 13.5, True, 2.125)])
    assert len(flagged) == 2
    assert "static deflection at the foot) 13.5 against" in flagged[0]
    assert "quasi-static deflection of its carried loads within the window and the carried weight's" in flagged[1]
    assert flagged[1].startswith(
        "lumped member 'bar_beam': the one-way carried-load result is not valid from t = 2.125 s"
    )
    assert "ruling 69" in flagged[1]
    assert translate.carried_validity_lines(_physics(_design([])), [(0.0,) * 10]) == ()


@pytest.mark.req("PHY-035")
def test_a_member_carrying_nothing_records_identically_with_the_field_absent_or_empty() -> None:
    """The no-carry path is inert (PHY-035): `carried` absent (as the translation writes it) and `carried: []` give
    bit-identical recordings."""
    model = _physics(_design([]))
    absent = prepare_model(model, None, 1e-3).model_json
    document = json.loads(absent)
    assert "carried" not in document["members"][0]
    document["members"][0]["carried"] = []
    first = _record(model, 300)
    second = _record(model, 300, json.dumps(document))
    assert sorted(second.series) == sorted(first.series)
    for path, series in first.series.items():
        left = np.asarray(series.values)
        right = np.asarray(second.series[path].values)
        if left.dtype.kind in "fiub":
            assert left.tobytes() == right.tobytes(), path
        else:
            assert left.tolist() == right.tolist(), path
    assert second.events == first.events
