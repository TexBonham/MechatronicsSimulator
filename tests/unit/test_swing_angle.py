"""The swing angle quantity (SCO-030, ACC-025, ruling 60): the angle between the line through two bodies' origins and
the gravity direction, scored with the existing statistics, against a rigid pendulum's closed form."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import pytest
from pydantic import TypeAdapter, ValidationError

from mechsim.design.schema import Design, Senses, load_design
from mechsim.scoring.score import _SwingView, _UnevaluableError
from mechsim.scoring.statistics import statistic
from mechsim.testing.driver import Series
from mechsim.testing.schema import (
    CASE_PAYLOAD_BODY,
    ROOT,
    Case,
    Criterion,
    QuantityRef,
    case_payload_body,
    cross_check,
    load_test_case,
    reference_dimension,
)

# Rigid pendulum: pivot body at the origin, bob at L (sin theta, -cos theta, 0) under gravity (0, -g, 0), released at
# rest from theta_0, small-angle motion theta(t) = theta_0 cos(2 pi t / T) with T = 2 pi sqrt(L / g) (the closed form
# the criteria score; the motion is given exactly, so the bounds below are the scoring's own error only).
G = 9.81
L = 1.0
THETA_0 = 0.05
PERIOD = 2.0 * math.pi * math.sqrt(L / G)
STEP = 1e-3
ROWS = 10_001


@dataclass(frozen=True)
class _Kinematics:
    bodies: tuple[str, ...]
    reference: tuple[tuple[tuple[float, float, float], tuple[float, float, float, float]], ...]


class _Pendulum:
    """Like the record view (TST-038): `pose:<body>.<x,y,z>` is the displacement from the body's reference-configuration
    origin (the bob hanging straight down), and `kinematics` gives those origins."""

    def __init__(self, pivot: tuple[float, float, float] = (0.0, 0.0, 0.0), bob_offset: float = L) -> None:
        self._times = tuple(index * STEP for index in range(ROWS))
        angles = [THETA_0 * math.cos(2.0 * math.pi * t / PERIOD) for t in self._times]
        hanging = (pivot[0], pivot[1] - bob_offset, pivot[2])
        identity = (1.0, 0.0, 0.0, 0.0)
        self.kinematics = _Kinematics(("pivot", "bob"), ((pivot, identity), (hanging, identity)))
        self._poses = {
            "pivot": [(0.0, 0.0, 0.0)] * ROWS,
            "bob": [(bob_offset * math.sin(a), bob_offset * (1.0 - math.cos(a)), 0.0) for a in angles],
        }

    def times(self) -> Sequence[float]:
        return self._times

    def series(self, reference: str) -> Series:
        kind, _, rest = reference.partition(":")
        body, _, axis = rest.partition(".")
        if kind != "pose" or body not in self._poses:
            raise KeyError(reference)
        index = "xyz".index(axis)
        return Series(self._times, tuple(pose[index] for pose in self._poses[body]), "m")

    def controller_states(self) -> Sequence[str]:
        return ()

    def events(self) -> Sequence[object]:
        return ()


def _swing(reference: str, gravity: Sequence[float] | None = (0.0, -G, 0.0), view: object = None) -> Series:
    if view is None:
        view = _Pendulum()
    return _SwingView(view, gravity).series(reference)  # type: ignore[arg-type]


@pytest.mark.req("SCO-030", "ACC-025")
def test_the_signed_swing_of_a_rigid_pendulum_has_its_release_angle_and_period() -> None:
    # Peak: the release sample t = 0 is theta_0 exactly; atan2 of the exact geometry adds rounding only (1e-12).
    # Period: upward zero crossings by linear interpolation; for theta_0 cos(w t) the interpolation error of a crossing
    # is below w^2 h^3 / 8 relative to the swing (h = 1 ms, w h = 3.1e-3), so the period over four periods is exact to
    # far below 1e-6 relative.
    series = _swing("swing:pivot.bob.x")
    peak, when = statistic("peak_absolute", series.times, series.values)
    print(f"swing peak: closed form {THETA_0} rad, measured {peak}; period: closed form {PERIOD} s")
    assert peak == pytest.approx(THETA_0, rel=1e-12)
    assert when == 0.0
    period, _ = statistic("period_about_mean", series.times, series.values)
    print(f"swing period: measured {period} s")
    assert period == pytest.approx(PERIOD, rel=1e-6)
    assert series.unit == "rad"
    # The reversed line (bob to pivot) points up: atan2(-sin theta, -cos theta) = theta - pi. z is across the swing
    # plane, so the line has no z part: 0.
    assert _swing("swing:pivot.bob.x").values[0] == pytest.approx(_swing("swing:bob.pivot.x").values[0] + math.pi)
    assert max(abs(value) for value in _swing("swing:pivot.bob.z").values) == pytest.approx(0.0, abs=1e-15)


@pytest.mark.req("SCO-030")
def test_the_unsigned_swing_is_the_angle_from_the_gravity_line() -> None:
    # acos(d . g_hat / |d|) = |theta| for the hanging bob; a pivot moved and a longer line change nothing.
    series = _swing("swing:pivot.bob", view=_Pendulum((0.3, 1.2, -0.4), 2.5))
    assert max(series.values) == pytest.approx(THETA_0, rel=1e-9)
    assert min(series.values) >= 0.0
    settled = _swing("swing:pivot.bob", gravity=(0.0, -2.0 * G, 0.0))
    assert settled.values == pytest.approx(_swing("swing:pivot.bob").values, rel=1e-12)


@pytest.mark.req("SCO-030", "SCO-029")
@pytest.mark.parametrize(
    ("reference", "gravity", "message"),
    [
        ("swing:pivot.bob", None, "no gravity input"),
        ("swing:pivot.bob", (0.0, 0.0, 0.0), "gravity is zero"),
        ("swing:pivot.bob.y", (0.0, -G, 0.0), "axis y is along gravity"),
        ("swing:pivot.pivot", (0.0, -G, 0.0), "origins coincide at 0 s"),
    ],
)
def test_a_swing_without_a_direction_or_a_line_is_unevaluable(
    reference: str, gravity: Sequence[float] | None, message: str
) -> None:
    with pytest.raises(_UnevaluableError, match=message):
        _swing(reference, gravity)


@pytest.mark.req("SCO-030")
def test_a_swing_of_an_unrecorded_body_is_not_recorded() -> None:
    with pytest.raises(KeyError):
        _swing("swing:pivot.ghost")


class _Unplaced(_Pendulum):
    def __init__(self) -> None:
        super().__init__()
        del self.kinematics


@pytest.mark.req("SCO-030", "TST-038")
def test_a_swing_needs_the_reference_poses_the_displacements_are_taken_from() -> None:
    with pytest.raises(_UnevaluableError, match="no reference pose for body 'pivot'"):
        _swing("swing:pivot.bob", view=_Unplaced())


@pytest.mark.req("SCO-030", "TST-038")
def test_the_swing_reads_world_origins_not_displacements() -> None:
    # At the reference configuration every displacement is zero; the line is the reference origins' (straight down),
    # so the angle is exactly zero, a value; the displacements alone would give coincident origins.
    view = _Pendulum()
    view._poses["bob"] = [(0.0, 0.0, 0.0)] * ROWS
    series = _swing("swing:pivot.bob.x", view=view)
    assert set(series.values) == {0.0}
    assert set(_swing("swing:pivot.bob", view=view).values) == {0.0}


@pytest.mark.req("SCO-030")
@pytest.mark.parametrize("reference", ["swing:hook.payload", "swing:hook.payload.x", "swing:a.b.z"])
def test_the_case_schema_accepts_swing_references(reference: str) -> None:
    assert TypeAdapter(QuantityRef).validate_python(reference) == reference
    assert reference_dimension(reference) == "dimensionless"


@pytest.mark.req("SCO-030")
@pytest.mark.parametrize("reference", ["swing:hook", "swing:hook.payload.w", "swing:hook.payload.x.y", "swing:"])
def test_the_case_schema_refuses_malformed_swing_references(reference: str) -> None:
    with pytest.raises(ValidationError):
        TypeAdapter(QuantityRef).validate_python(reference)


def _senses(**target: str) -> Senses:
    return Senses(**target)  # type: ignore[arg-type]


@pytest.mark.req("SCO-030")
@pytest.mark.parametrize(
    ("senses", "expected"),
    [
        ((), CASE_PAYLOAD_BODY),
        ((None, _senses(joint="pivot")), CASE_PAYLOAD_BODY),
        ((None, _senses(payload="crate")), "crate"),
        ((_senses(joint_reaction="cargo"),), "cargo"),
        ((_senses(joint_reaction="pivot"),), CASE_PAYLOAD_BODY),
        ((_senses(joint_reaction="pivot"), _senses(payload="crate")), "crate"),
        ((_senses(payload="first"), _senses(payload="second")), "first"),
        ((_senses(body="hook"),), CASE_PAYLOAD_BODY),
    ],
)
def test_the_case_payload_body_is_the_name_the_run_gives_it(senses: tuple[Senses | None, ...], expected: str) -> None:
    assert case_payload_body(senses, {"pivot"}) == expected


def _gantry_with(tmp_path: Path, old: str, new: str) -> Design:
    text = (ROOT / "designs" / "gantry" / "design.yaml").read_text(encoding="utf-8")
    assert old in text
    folder = tmp_path / "gantry"
    folder.mkdir()
    (folder / "design.yaml").write_text(text.replace(old, new, 1), encoding="utf-8")
    return load_design(folder / "design.yaml")


def _swing_case(body: str) -> Case:
    case = load_test_case(ROOT / "test_cases" / "gantry" / "G1.yaml")
    criterion = TypeAdapter(Criterion).validate_python(
        {
            "name": "sway",
            "check": "value",
            "quantity": f"swing:hook.{body}",
            "statistic": "maximum",
            "comparison": "at_most",
            "limit": "0.1 rad",
        }
    )
    return case.model_copy(update={"criteria": (*case.criteria, criterion)})


@pytest.mark.req("SCO-030")
@pytest.mark.parametrize(
    ("old", "new", "body"),
    [
        ("joint_reaction: payload", "joint_reaction: payload", "payload"),
        ("joint_reaction: payload", "joint_reaction: cargo", "cargo"),
    ],
)
def test_the_case_schema_accepts_the_payload_under_the_name_the_run_gives_it(
    tmp_path: Path, old: str, new: str, body: str
) -> None:
    design = _gantry_with(tmp_path, old, new)
    assert case_payload_body((c.senses for c in design.components), {j.name for j in design.joints}) == body
    assert [error for error in cross_check(_swing_case(body), design) if "sway" in error] == []
    if body != CASE_PAYLOAD_BODY:
        errors = cross_check(_swing_case(CASE_PAYLOAD_BODY), design)
        assert any("unknown body 'payload'" in error for error in errors), errors
    assert any("unknown body 'ghost'" in error for error in cross_check(_swing_case("ghost"), design))
