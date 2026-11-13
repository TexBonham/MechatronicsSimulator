"""Statistics of a recorded series over a window (SCO-020, SCO-027, SCO-028): final, minimum, maximum, peak absolute,
period, and the oscillation statistics mean, amplitude, period about the mean and frequency about the mean.

Each returns the value and the simulated time of its worst case: the sample time for final, minimum, maximum and peak
absolute (the first such sample on a tie), the last upward crossing used for a period or frequency, the window's last
sample for the mean, and the later of the two extreme samples for the amplitude.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Literal

Statistic = Literal[
    "final",
    "minimum",
    "maximum",
    "peak_absolute",
    "period",
    "mean",
    "amplitude",
    "period_about_mean",
    "frequency_about_mean",
]


class StatisticError(ValueError):
    """The statistic cannot be computed from the samples given (empty window, too few zero crossings, a sample that is
    not a number)."""


def window_indices(times: Sequence[float], start: float | None, end: float | None) -> list[int]:
    """Indices of the samples with start <= t <= end; an absent bound is open."""
    selected: list[int] = []
    for index, time in enumerate(times):
        if start is not None and time < start:
            continue
        if end is not None and time > end:
            continue
        selected.append(index)
    return selected


def upward_crossings(times: Sequence[float], values: Sequence[float]) -> list[float]:
    """Times of upward zero crossings, before < 0 and after >= 0, linearly interpolated between samples, as the
    engine reference tests count them (`core/engine/tests/beam_helpers.hpp`)."""
    crossings: list[float] = []
    for index in range(1, len(values)):
        before = values[index - 1]
        after = values[index]
        if before < 0.0 and after >= 0.0:
            t0 = times[index - 1]
            step = times[index] - t0
            crossings.append(t0 + step * before / (before - after))
    return crossings


def statistic(name: Statistic, times: Sequence[float], values: Sequence[float]) -> tuple[float, float]:
    """The statistic `name` of the samples and the time of its worst case."""
    if len(values) == 0:
        raise StatisticError("no samples in the window")
    for index, value in enumerate(values):
        if math.isnan(value):
            raise StatisticError(f"not evaluated: the sample at {times[index]:.12g} s is not a number")
    if name == "final":
        return values[-1], times[-1]
    if name == "period":
        return _period(times, values, "upward zero crossings")
    if name == "mean":
        return math.fsum(values) / len(values), times[-1]
    if name == "amplitude":
        return _amplitude(times, values)
    if name in ("period_about_mean", "frequency_about_mean"):
        mean = math.fsum(values) / len(values)
        period, last = _period(times, [value - mean for value in values], "upward crossings of the window's mean")
        if name == "period_about_mean":
            return period, last
        return 1.0 / period, last
    best = 0
    for index in range(1, len(values)):
        if _better(name, values[index], values[best]):
            best = index
    if name == "peak_absolute":
        return abs(values[best]), times[best]
    return values[best], times[best]


def _period(times: Sequence[float], values: Sequence[float], about: str) -> tuple[float, float]:
    crossings = upward_crossings(times, values)
    if len(crossings) < 2:
        raise StatisticError(f"period needs at least 2 {about}, found {len(crossings)}")
    return (crossings[-1] - crossings[0]) / (len(crossings) - 1), crossings[-1]


def _amplitude(times: Sequence[float], values: Sequence[float]) -> tuple[float, float]:
    """Half the peak-to-peak excursion; independent of the level the series swings about."""
    low = 0
    high = 0
    for index in range(1, len(values)):
        if values[index] < values[low]:
            low = index
        if values[index] > values[high]:
            high = index
    worst = times[high]
    if times[low] > worst:
        worst = times[low]
    return 0.5 * (values[high] - values[low]), worst


def _better(name: Statistic, candidate: float, current: float) -> bool:
    if name == "minimum":
        return candidate < current
    if name == "maximum":
        return candidate > current
    return abs(candidate) > abs(current)


def compare(measured: float, comparison: str, limit: float) -> bool:
    """below <, at_most <=, above >, at_least >= (the named forms, never implied)."""
    if comparison == "below":
        return measured < limit
    if comparison == "at_most":
        return measured <= limit
    if comparison == "above":
        return measured > limit
    if comparison == "at_least":
        return measured >= limit
    raise ValueError(f"unknown comparison {comparison!r}")


def within(measured: float, target: float, tolerance: float) -> bool:
    """|measured - target| <= tolerance: an error equal to the tolerance passes."""
    return abs(measured - target) <= tolerance
