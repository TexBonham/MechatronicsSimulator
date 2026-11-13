"""The swing angle quantity (SCO-030, ACC-025, ruling 60): the angle between the line through two bodies' origins and
the gravity direction, scored with the existing statistics, against a rigid pendulum's closed form."""

from __future__ import annotations

import math
from collections.abc import Sequence

import pytest
from pydantic import TypeAdapter, ValidationError

from mechsim.scoring.score import _SwingView, _UnevaluableError
from mechsim.scoring.statistics import statistic
from mechsim.testing.driver import Series
from mechsim.testing.schema import QuantityRef, reference_dimension

# Rigid pendulum: pivot body at the origin, bob at L (sin theta, -cos theta, 0) under gravity (0, -g, 0), released at
# rest from theta_0, small-angle motion theta(t) = theta_0 cos(2 pi t / T) with T = 2 pi sqrt(L / g) (the closed form
# the criteria score; the motion is given exactly, so the bounds below are the scoring's own error only).
G = 9.81
L = 1.0
THETA_0 = 0.05
PERIOD = 2.0 * math.pi * math.sqrt(L / G)
STEP = 1e-3
ROWS = 10_001


class _Pendulum:
    def __init__(self, pivot: tuple[float, float, float] = (0.0, 0.0, 0.0), bob_offset: float = L) -> None:
        self._times = tuple(index * STEP for index in range(ROWS))
        angles = [THETA_0 * math.cos(2.0 * math.pi * t / PERIOD) for t in self._times]
        self._poses = {
            "pivot": [pivot] * ROWS,
            "bob": [
                (pivot[0] + bob_offset * math.sin(a), pivot[1] - bob_offset * math.cos(a), pivot[2]) for a in angles
            ],
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
        ("swing:pivot.bob", None, "states no gravity direction"),
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
