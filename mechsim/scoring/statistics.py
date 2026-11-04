"""Statistics of a recorded series over a window (SCO-020): final, minimum, maximum, peak absolute, period.

Each returns the value and the simulated time of its worst case: the sample time for final, minimum, maximum and peak
absolute (the first such sample on a tie), and the last upward zero crossing used for the period.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Literal

Statistic = Literal["final", "minimum", "maximum", "peak_absolute", "period"]


class StatisticError(ValueError):
    """The statistic cannot be computed from the samples given (empty window, too few zero crossings)."""


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
    if name == "final":
        return values[-1], times[-1]
    if name == "period":
        crossings = upward_crossings(times, values)
        if len(crossings) < 2:
            raise StatisticError(f"period needs at least 2 upward zero crossings, found {len(crossings)}")
        return (crossings[-1] - crossings[0]) / (len(crossings) - 1), crossings[-1]
    best = 0
    for index in range(1, len(values)):
        if _better(name, values[index], values[best]):
            best = index
    if name == "peak_absolute":
        return abs(values[best]), times[best]
    return values[best], times[best]


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
