"""Service-side decimation (FAC-007, FAC-018): which recorded samples of a window reach a client. Selects sample
indices only; the recording is never touched."""

from __future__ import annotations

import math
from collections.abc import Sequence


def bucket_of(t: float, t_start: float, t_end: float, points: int) -> int:
    """Bucket k of `points` equal buckets covers [t_start + k w, t_start + (k + 1) w), the last one closed at t_end."""
    width = (t_end - t_start) / points
    return min(math.floor((t - t_start) / width), points - 1)


def window(times: Sequence[float], t_start: float, t_end: float) -> list[int]:
    return [index for index, t in enumerate(times) if t_start <= t <= t_end]


def bucket_select(
    times: Sequence[float], t_start: float, t_end: float, points: int, first: int = 0, stop: int | None = None
) -> list[int]:
    """Per bucket in [first, stop) the window sample nearest the bucket's centre (the earlier one on a tie); an empty
    bucket contributes nothing."""
    if stop is None:
        stop = points
    width = (t_end - t_start) / points
    best: dict[int, tuple[float, int]] = {}
    for index in window(times, t_start, t_end):
        bucket = bucket_of(times[index], t_start, t_end, points)
        if bucket < first or bucket >= stop:
            continue
        distance = abs(times[index] - (t_start + (bucket + 0.5) * width))
        held = best.get(bucket)
        if held is None or distance < held[0]:
            best[bucket] = (distance, index)
    return [best[bucket][1] for bucket in sorted(best)]


def decimate(times: Sequence[float], t_start: float, t_end: float, points: int) -> list[int]:
    """Every window sample when `points` covers them all; otherwise one sample per bucket, nearest its centre. Never
    more than `points` indices, in time order."""
    if points < 1 or not t_start < t_end:
        raise ValueError("decimation needs points >= 1 and t_start < t_end (FAC-018)")
    kept = window(times, t_start, t_end)
    if points >= len(kept):
        return kept
    return bucket_select(times, t_start, t_end, points)
