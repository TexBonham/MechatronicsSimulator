"""Replay cursor over a run's recorded sample times (VIE-007 to VIE-009). Pure state: no clock, no engine (VIE-016)."""

from __future__ import annotations

import bisect
import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Literal

Direction = Literal["forward", "backward"]


class ReplayError(ValueError):
    """A cursor request that cannot be honoured (no samples, a negative or non-finite time)."""


@dataclass(frozen=True)
class CursorState:
    index: int
    t: float
    paused: bool
    samples: int


class ReplayCursor:
    """Starts paused at the first recorded sample; every position is a recorded sample index. Playback keeps a
    continuous play time beside the index, so time `advance` moves past the shown sample is carried, not dropped
    (VIE-009); scrub and step set it to the time asked or the sample reached."""

    def __init__(self, times: Sequence[float]) -> None:
        if len(times) == 0:
            raise ReplayError("the recording has no samples to replay")
        self._times = tuple(float(value) for value in times)
        self._index = 0
        self._paused = True
        self._play = self._times[0]

    @property
    def times(self) -> tuple[float, ...]:
        return self._times

    def state(self) -> CursorState:
        return CursorState(self._index, self._times[self._index], self._paused, len(self._times))

    def scrub(self, t: float) -> CursorState:
        """The latest recorded sample at or before t; a time before the first sample lands on the first."""
        _check_time(t, "scrub time")
        self._seek(t)
        return self.state()

    def _seek(self, t: float) -> None:
        self._play = min(max(t, self._times[0]), self._times[-1])
        self._index = max(bisect.bisect_right(self._times, self._play) - 1, 0)

    def pause(self, paused: bool) -> CursorState:
        self._paused = paused
        return self.state()

    def step(self, direction: Direction) -> CursorState:
        """One recorded sample; stays put at either end."""
        if direction == "forward":
            self._index = min(self._index + 1, len(self._times) - 1)
        else:
            self._index = max(self._index - 1, 0)
        self._play = self._times[self._index]
        return self.state()

    def advance(self, dt: float) -> CursorState:
        """A client play loop's elapsed simulated time; no movement while paused."""
        _check_time(dt, "advance interval")
        if self._paused:
            return self.state()
        self._seek(self._play + dt)
        return self.state()


def _check_time(value: float, what: str) -> None:
    if not math.isfinite(value) or value < 0.0:
        raise ReplayError(f"{what} must be finite and >= 0 s, got {value}")
