"""Tier 1 integrity events are deterministic (INT-027, invariant 4): the same design, case, seed and mode raise the
same events at the same simulated times, compared bitwise."""

from __future__ import annotations

from pathlib import Path

import pytest

from tests.unit.test_integrity_tier1 import gantry_run, mount_config


@pytest.mark.req("INT-027")
def test_a_gantry_run_with_tier_1_raises_the_same_events_twice(tmp_path: Path) -> None:
    config = mount_config("2 N")
    first, one = gantry_run(tmp_path, "native", config)
    second, two = gantry_run(tmp_path, "native", config)
    a = [(event.time.hex(), event.source, event.kind, event.detail) for event in one.recording().events()]
    b = [(event.time.hex(), event.source, event.kind, event.detail) for event in two.recording().events()]
    assert any(kind == "integrity" for _, _, kind, _ in a)
    assert a == b
    assert [c.value.hex() for c in one.integrity_trace.crossings] == [c.value.hex() for c in two.integrity_trace.crossings]
    assert first.report == second.report
