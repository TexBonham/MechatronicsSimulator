"""The builder's note on a horizontal flexible member whose bending plane gravity misses entirely (BLD-038, ruling 65):
the lumped chain bends about its frame z only, so with gravity along the frame z the member cannot sag; and the carried
joint travel check's rounding tolerance (BLD-037)."""

from __future__ import annotations

from typing import Any

import pytest

from mechsim.builder import BuildError, build
from mechsim.design.schema import Design, shipped_materials
from mechsim.library.registry import default_registry
from tests.unit.test_carried_members import _design_data

HALF = 0.7071067811865476
FRAGMENT = "gravity entirely outside its bending plane"


def _notes(gravity: tuple[float, float, float], orientation: list[float] | None = None, rigid: bool = False) -> list[str]:
    """The bar of the carried-member fixture (along world x from the origin, frame y the bending direction) under
    `gravity`, its mount turned by `orientation` (w-first) when given; the notes naming the bar's bending plane."""
    data: dict[str, Any] = _design_data([])
    data["environment"]["gravity"] = [f"{value} m/s**2" for value in gravity]
    if orientation is not None:
        data["joints"][0]["placement"]["orientation"] = orientation
    result = build(Design.model_validate(data), default_registry(), shipped_materials(), rigid_members=rigid)
    return [note for note in result.notes if FRAGMENT in note]


@pytest.mark.req("BLD-038")
def test_gravity_along_the_frame_z_of_a_horizontal_member_adds_one_note_naming_it() -> None:
    notes = _notes((0.0, 0.0, -9.81))
    assert len(notes) == 1
    assert notes[0].startswith("flexible body 'bar': horizontal")
    assert "state the frame with y vertical" in notes[0]
    assert "BLD-038" in notes[0]
    assert len(_notes((0.0, 0.0, 9.81))) == 1


@pytest.mark.req("BLD-038")
@pytest.mark.parametrize(
    "gravity",
    [
        (0.0, -9.81, 0.0),
        (0.0, 9.81, 0.0),
        (0.0, -6.0, -6.0),
        (-9.81, 0.0, 0.0),
        (-6.0, 0.0, -6.0),
        (0.0, 0.0, 0.0),
    ],
    ids=["in_plane_down", "in_plane_up", "partly_in_plane", "along_the_member", "member_not_horizontal", "no_gravity"],
)
def test_no_note_when_gravity_has_a_component_in_the_bending_plane_or_the_member_is_not_horizontal(
    gravity: tuple[float, float, float],
) -> None:
    assert _notes(gravity) == []


@pytest.mark.req("BLD-038")
def test_the_check_uses_the_member_frame_at_the_reference_configuration() -> None:
    """Turned 90 degrees about x, the bar's frame y is world z: vertical gravity lies in its plane, no note; gravity
    along world y is then along its frame z, one note."""
    turned = [HALF, HALF, 0.0, 0.0]
    assert _notes((0.0, 0.0, -9.81), turned) == []
    assert len(_notes((0.0, -9.81, 0.0), turned)) == 1


@pytest.mark.req("BLD-038", "BLD-033")
def test_a_rigid_build_adds_no_note() -> None:
    assert _notes((0.0, 0.0, -9.81), rigid=True) == []


def _turned_slide(limits: tuple[float, float]) -> Design:
    """The carried slide placed with a rotation about y by 90 degrees (w-first entries 1 / sqrt 2, not exact in
    binary) and its axis along the child's z, so its travel runs along the bar's x with the rotation's rounding."""
    data = _design_data(["slide"], position=0.5, limits=limits)
    data["joints"][1]["placement"]["orientation"] = [HALF, 0.0, HALF, 0.0]
    data["joints"][1]["axis"] = [0.0, 0.0, 1.0]
    return Design.model_validate(data)


@pytest.mark.req("BLD-037")
def test_travel_to_the_span_ends_through_a_rounded_rotation_is_accepted() -> None:
    result = build(_turned_slide((-0.5, 0.5)), default_registry(), shipped_materials())
    assert result.physics is not None
    assert result.physics.lumped_members[0].carries == ("carriage",)


@pytest.mark.req("BLD-037")
def test_travel_past_the_span_by_more_than_the_rounding_tolerance_is_refused() -> None:
    with pytest.raises(BuildError) as raised:
        build(_turned_slide((-0.5, 0.500001)), default_registry(), shipped_materials())
    assert len(raised.value.failures) == 1
    assert "carried joint 'slide'" in raised.value.failures[0]
    assert "outside the member's span [0, 1] m by 1e-06 m" in raised.value.failures[0]
