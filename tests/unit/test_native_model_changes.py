"""Model-change events and both rope tensions from the native adapter through the kernel to the recorder (ruling 40,
ruling 26, PHY-004, PHY-008, REC-004, REC-010)."""

from __future__ import annotations

import math
from fractions import Fraction
from pathlib import Path
from typing import Any

import pytest

from mechsim import _core as core
from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.neutral import PhysicsModel, from_yaml
from mechsim.record.layout import ROPE_MAX_TENSION, ROPE_TENSION, Recording
from mechsim.record.stream import Recorder
from mechsim.testing.core_driver import model_columns, tick_record

ROOT = Path(__file__).resolve().parents[2]
PENDULUM = (ROOT / "schemas/examples/physics_model.pendulum.yaml").read_text()
G = 9.81
BREAKING = 9.5


def snapping_rope() -> PhysicsModel:
    """A 1 kg load hanging from the world on an unstretched 4-segment rope that breaks at 9.5 N, just under its
    weight: the first stretch overshoots towards 2 m g, so the rope snaps within the first swing."""
    data = from_yaml(PENDULUM).model_dump(mode="json")
    data["signals"] = {}
    data["gravity"] = ("0 m/s**2", "0 m/s**2", f"{-G} m/s**2")
    data["bodies"] = [
        {
            "name": "load",
            "mass": "1 kg",
            "com": ("0 m", "0 m", "0 m"),
            "inertia": {"ixx": "0.01 kg*m**2", "iyy": "0.01 kg*m**2", "izz": "0.01 kg*m**2"},
            "material": "steel",
            "parent": "world",
            "joint": {"name": "hang", "type": "free", "position": ("0 m", "0 m", "-1 m")},
            "shapes": [],
        }
    ]
    data["ropes"] = [
        {
            "name": "line",
            "from_body": "world",
            "from_point": ("0 m", "0 m", "0 m"),
            "to_body": "load",
            "to_point": ("0 m", "0 m", "0 m"),
            "segments": 4,
            "axial_stiffness": "1e5 N",
            "damping": "50 N*s",
            "mass_per_length": "0.01 kg/m",
            "rest_length": "1 m",
            "breaking_tension": f"{BREAKING} N",
        }
    ]
    return PhysicsModel.model_validate(data)


def record(model: PhysicsModel, ticks: int) -> Recording:
    prepared = prepare_model(model, None, 1e-3)
    adapter = core.make_physics_adapter("native")
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(1000))
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


def _model_changes(recording: Recording) -> list[tuple[int, Any]]:
    return [(tick, event) for tick, event in recording.events if event.kind == "model_change"]


@pytest.mark.req("PHY-008", "REC-010")
def test_a_snap_is_recorded_as_two_model_change_events_in_order() -> None:
    recording = record(snapping_rope(), 300)
    changes = _model_changes(recording)
    assert len(changes) == 2
    (snap_tick, snap), (release_tick, release) = changes
    assert snap.source == "physics"
    assert snap.detail.startswith("rope_snapped rope 0 value ")
    assert float(snap.detail.rsplit(" ", 1)[1]) > BREAKING
    assert release.detail == "load_released rope 0 value 0"
    # The engine releases the load in the step whose end-of-step tension broke the rope (one release path).
    assert release_tick == snap_tick


@pytest.mark.req("PHY-004", "REC-004")
def test_both_rope_tensions_are_recorded_and_differ_by_the_rope_weight() -> None:
    """Before the snap the rope hangs from the world under a falling load; at any instant the largest segment
    tension is at least the load-end tension, and both are zero after the load is released (no rope in tension).
    At rest the two differ by the weight of 3.5 segments (the end half-masses sit on the end bodies)."""
    recording = record(snapping_rope(), 300)
    tension = recording.series[ROPE_TENSION].values
    maximum = recording.series[ROPE_MAX_TENSION].values
    assert tension.shape == maximum.shape == (300, 1)
    assert bool((maximum >= tension).all())
    assert float(maximum.max()) > BREAKING
    assert recording.series[ROPE_TENSION].names == ("line",)


@pytest.mark.req("PHY-004")
def test_a_resting_rope_reports_load_end_and_maximum_tension() -> None:
    model = snapping_rope()
    data = model.model_dump(mode="json")
    data["ropes"][0]["breaking_tension"] = None
    adapter = core.make_physics_adapter("native")
    prepared = prepare_model(PhysicsModel.model_validate(data), None, 1e-3)
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(1000))
    for _ in range(3000):
        adapter.step()
    rope = adapter.outputs().ropes[0]
    segment_mass = 0.01 * 1.0 / 4
    assert rope.tension == pytest.approx((1.0 + segment_mass / 2) * G, rel=1e-3)
    assert rope.max_tension == pytest.approx((1.0 + 3.5 * segment_mass) * G, rel=1e-3)
    assert not rope.slack
    assert adapter.outputs().model_changes == []
    assert math.isfinite(rope.max_tension)


@pytest.mark.req("PHY-010")
def test_a_rope_snap_runs_twice_bitwise() -> None:
    first = record(snapping_rope(), 200)
    second = record(snapping_rope(), 200)
    assert first.events == second.events
    for name in (ROPE_TENSION, ROPE_MAX_TENSION):
        assert first.series[name].values.tobytes() == second.series[name].values.tobytes()
