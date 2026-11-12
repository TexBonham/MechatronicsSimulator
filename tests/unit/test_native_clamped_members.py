"""Clamped-clamped members reach the own engine (BLD-017, BLD-018; rulings 16, 29, 31, 32): the neutral end value,
the builder's segment defaults and calibration floor, the translation's `clamped` ends, the native build flexible and
the bridge build rigid, and the member node names (X1)."""

from __future__ import annotations

import json
from typing import Any

import pytest
from pydantic import ValidationError

from mechsim.builder import BuildError, build
from mechsim.design.schema import shipped_materials
from mechsim.library.registry import default_registry
from mechsim.physics.native import translate
from mechsim.physics.neutral import PhysicsModel
from mechsim.testing.core_driver import core_builder
from mechsim.testing.runner import prepare
from mechsim.testing.schema import ROOT
from tests.unit.test_builder_structure import CALIBRATION, _bar

G3 = ROOT / "test_cases" / "gantry" / "G3.yaml"


def _model(supports: list[str], **flexible: Any) -> PhysicsModel:
    result = build(_bar(supports, **flexible), default_registry(), shipped_materials())
    assert result.physics is not None
    return result.physics


@pytest.mark.req("BLD-018")
@pytest.mark.parametrize(
    ("supports", "segments"), [(["clamped", "clamped"], 16), (["clamped", "free"], 8), (["pinned", "roller"], 8)]
)
def test_the_default_segment_count_follows_the_support_condition(supports: list[str], segments: int) -> None:
    member = _model(supports).lumped_members[0]
    assert (member.root, member.end) == (supports[0], supports[1])
    assert member.segments == segments


@pytest.mark.req("BLD-018")
def test_a_stated_segment_count_wins() -> None:
    assert _model(["clamped", "clamped"], segments=20).lumped_members[0].segments == 20
    assert _model(["clamped", "free"], segments=12).lumped_members[0].segments == 12


@pytest.mark.req("BLD-018")
def test_a_calibrated_clamped_clamped_member_below_sixteen_segments_is_a_build_error() -> None:
    with pytest.raises(BuildError) as raised:
        build(
            _bar(["clamped", "clamped"], calibration=CALIBRATION, segments=8), default_registry(), shipped_materials()
        )
    assert len(raised.value.failures) == 1
    assert "calibrated member has 8 segments; calibration needs at least 16" in raised.value.failures[0]
    result = build(_bar(["clamped", "clamped"], calibration=CALIBRATION), default_registry(), shipped_materials())
    assert result.physics is not None
    assert result.physics.lumped_members[0].segments == 16
    assert any("no calibration chain layout yet" in note for note in result.notes)


@pytest.mark.req("BLD-018")
def test_a_calibrated_cantilever_keeps_the_floor_of_eight() -> None:
    with pytest.raises(BuildError) as raised:
        build(_bar(["clamped", "free"], calibration=CALIBRATION, segments=7), default_registry(), shipped_materials())
    assert "calibration needs at least 8" in raised.value.failures[0]


@pytest.mark.req("BLD-017")
def test_a_clamped_end_needs_a_clamped_root() -> None:
    with pytest.raises(BuildError) as raised:
        build(_bar(["pinned", "clamped"]), default_registry(), shipped_materials())
    assert "supports (pinned, clamped) have no lumped beam" in raised.value.failures[0]
    data = _model(["clamped", "clamped"]).model_dump(mode="json")
    data["lumped_members"][0]["root"] = "pinned"
    with pytest.raises(ValidationError, match="end 'clamped' needs root 'clamped'"):
        PhysicsModel.model_validate(data)
    del data["lumped_members"][0]["segments"]
    data["lumped_members"][0]["root"] = "clamped"
    assert PhysicsModel.model_validate(data).lumped_members[0].segments == 16


@pytest.mark.req("BLD-017", "PHY-020")
def test_the_translation_emits_the_engines_clamped_ends() -> None:
    document = json.loads(translate.translate(_model(["clamped", "clamped"])).text)
    assert document["members"][0]["ends"] == "clamped"
    assert document["members"][0]["segments"] == 16
    document = json.loads(translate.translate(_model(["clamped", "free"])).text)
    assert document["members"][0]["ends"] == "cantilever"


@pytest.mark.req("BLD-033", "TST-040")
def test_the_native_build_is_flexible_and_the_bridge_build_rigid() -> None:
    design = prepare(G3).design
    native = core_builder(design, "native")
    bridge = core_builder(design)
    assert native.physics is not None
    assert bridge.physics is not None
    assert len(bridge.physics.lumped_members) == 0
    ends = {
        member.structural_member: (member.root, member.end, member.segments) for member in native.physics.lumped_members
    }
    assert ends["rail_x"] == ("clamped", "clamped", 16)
    assert ends["rail_y"] == ("pinned", "roller", 8)
    assert [member.name for member in native.members] == [member.name for member in bridge.members]


@pytest.mark.req("BLD-017")
def test_member_node_names_follow_the_published_order() -> None:
    model = _model(["clamped", "clamped"])
    name = model.lumped_members[0].name
    assert translate.member_node_names(model) == tuple(f"{name}.node{k}" for k in range(16))
    pinned = _model(["pinned", "roller"])
    assert len(translate.member_node_names(pinned)) == 9
