"""Tier 1 integrity events are deterministic (INT-027, invariant 4): the same design, case, seed and mode raise the
same events at the same simulated times, compared bitwise."""

from __future__ import annotations

from pathlib import Path

import pytest

from tests.unit.test_integrity_tier1 import gantry_run, mount_config, run_yielding


@pytest.mark.req("INT-027")
def test_a_gantry_run_with_tier_1_raises_the_same_events_twice(tmp_path: Path) -> None:
    config = mount_config("2 N")
    first, one = gantry_run(tmp_path, "native", config)
    second, two = gantry_run(tmp_path, "native", config)
    a = [(event.time.hex(), event.source, event.kind, event.detail) for event in one.recording().events()]
    b = [(event.time.hex(), event.source, event.kind, event.detail) for event in two.recording().events()]
    assert any(kind == "integrity" for _, _, kind, _ in a)
    assert a == b
    assert [c.value.hex() for c in one.integrity_trace.crossings] == [
        c.value.hex() for c in two.integrity_trace.crossings
    ]
    assert first.report == second.report


@pytest.mark.req("INT-027", "INT-028")
def test_member_crossings_and_yield_changes_repeat_bitwise() -> None:
    """The yielding cantilever (a fastener, a member stress crossing and a run of yield offsets) twice: the same
    events at the same ticks with the same details, and the same requests bit for bit (invariant 4)."""
    first_trace, first = run_yielding(300)
    second_trace, second = run_yielding(300)
    details = [detail for _, _, kind, detail in first if kind == "integrity"]
    assert any('"quantity": "fastener_load"' in detail for detail in details)
    assert any('"quantity": "stress"' in detail for detail in details)
    assert sum(1 for _, _, kind, _ in first if kind == "model_change") > 1
    assert first == second
    assert [r.value.hex() for r in first_trace.requests] == [r.value.hex() for r in second_trace.requests]
    assert [c.value.hex() for c in first_trace.crossings] == [c.value.hex() for c in second_trace.crossings]
