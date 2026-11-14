"""Never silently low (ruling 79, INT-005): where a carried couple is applied at a foot point inside the stretch where
the member's own weight moment hogs, the rule's read can be low by up to 3 w l^2; both tiers say so with one line per
member (the strength check's notes, the party's trace)."""

from __future__ import annotations

import json
import math
from types import SimpleNamespace

import pytest

from mechsim.integrity.beam import hogging_intervals, own_weight_bound
from mechsim.integrity.checks.strength import hogging_note
from mechsim.integrity.plugin import MemberInputs, NodeChain, SectionProperties
from mechsim.integrity.tier1 import node_record, trace_notes

LENGTH = 2.0
SECTION = SectionProperties(4e-4, 7e10, 2700.0, LENGTH, 1e-8, 1e-8, 0.01, 0.01)
CHAIN = NodeChain(0, 8, "clamped", None, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), LENGTH / 8.0, carries=True)


def _member(*supports: tuple[float, str]) -> MemberInputs:
    return MemberInputs("bar", "alu", SECTION, tuple(supports), None, nodes=CHAIN)  # type: ignore[arg-type]


def _clamped_clamped(x: float) -> float:
    # Uniform w = 1 on a clamped-clamped span, sagging positive: M = (6 L x - 6 x^2 - L^2) / 12.
    return (6.0 * LENGTH * x - 6.0 * x * x - LENGTH * LENGTH) / 12.0


def _clamped_pinned(x: float) -> float:
    # Clamp at 0, pin at L, w = 1: the pin's reaction 3 L / 8, M = (3 L / 8) (L - x) - (L - x)^2 / 2.
    return 3.0 * LENGTH / 8.0 * (LENGTH - x) - (LENGTH - x) ** 2 / 2.0


@pytest.mark.req("INT-005")
def test_the_hogging_stretches_are_where_the_uniform_load_moment_has_the_clamps_sign() -> None:
    both = hogging_intervals(_member((0.0, "clamped"), (LENGTH, "clamped")))
    assert len(both) == 2
    (lo, first), (second, hi) = both
    assert (lo, hi) == (0.0, LENGTH)
    for root in (first, second):
        assert _clamped_clamped(root) == pytest.approx(0.0, abs=1e-15)
    assert _clamped_clamped(first / 2.0) < 0.0
    assert _clamped_clamped(LENGTH / 2.0) > 0.0
    ((lo, hi),) = hogging_intervals(_member((0.0, "clamped"), (LENGTH, "roller")))
    assert lo == 0.0
    assert _clamped_pinned(hi) == pytest.approx(0.0, abs=1e-15)
    assert _clamped_pinned(hi / 2.0) < 0.0
    ((lo, hi),) = hogging_intervals(_member((0.0, "pinned"), (LENGTH, "clamped")))
    assert (lo, hi) == pytest.approx((0.75 * LENGTH, LENGTH))
    assert hogging_intervals(_member((0.0, "pinned"), (LENGTH, "pinned"))) == ()
    assert hogging_intervals(_member((0.0, "clamped"))) == ((0.0, LENGTH),)
    assert hogging_intervals(_member((LENGTH, "clamped"))) == ((0.0, LENGTH),)
    # Any other layout is taken to hog everywhere (conservative): no support, one pin, an interior support.
    assert hogging_intervals(_member()) == ((0.0, LENGTH),)
    assert hogging_intervals(_member((0.0, "pinned"))) == ((0.0, LENGTH),)
    assert hogging_intervals(_member((0.0, "clamped"), (1.0, "pinned"), (LENGTH, "clamped"))) == ((0.0, LENGTH),)


@pytest.mark.req("INT-005")
def test_the_own_weight_bound_is_three_w_l_squared() -> None:
    weight = 2700.0 * 4e-4 * 9.81
    assert own_weight_bound(_member((0.0, "clamped")), -9.81) == pytest.approx(3.0 * weight * (LENGTH / 8.0) ** 2)
    assert own_weight_bound(_member((0.0, "clamped")), 0.0) == 0.0
    rigid = MemberInputs("bar", "alu", SECTION, ((0.0, "clamped"),), None)
    assert own_weight_bound(rigid, 9.81) == 0.0


@pytest.mark.req("INT-005", "INT-023")
def test_the_limits_table_carries_the_bound_and_stretches_of_a_carrying_chain_only() -> None:
    member = _member((0.0, "clamped"), (LENGTH, "pinned"))
    record = node_record(CHAIN, member, 9.81)
    assert record is not None
    assert record["hogging"]["bound"] == own_weight_bound(member, 9.81)
    assert record["hogging"]["intervals"] == [[0.0, 0.25 * LENGTH]]
    plain = NodeChain(0, 8, "clamped", None, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), LENGTH / 8.0)
    record = node_record(plain, member, 9.81)
    assert record is not None
    assert "hogging" not in record
    assert node_record(None, member, 9.81) is None


@pytest.mark.req("INT-005", "INT-023")
def test_the_party_trace_gives_one_line_per_member_in_the_strength_checks_words() -> None:
    limits = json.dumps({"members": [{"name": "rail_a"}, {"name": "rail_b"}, {"name": "rail_c"}]})
    trace = SimpleNamespace(hogging_bound=[0.0, 2.0, 1.5], hogging_read=[0.0, 8.0, 0.0])
    lines = trace_notes(limits, trace)
    assert lines == (hogging_note("rail_b", 2.0, 8.0), hogging_note("rail_c", 1.5, 0.0))
    assert lines[0] == (
        "strength of 'rail_b': may read low under a carried couple at a foot point where the own weight's moment "
        "hogs, by at most 3 w l^2 = 2 N m (static own weight; more while the member rings), 0.25 of the largest read "
        "there, 8 N m (INT-005, ruling 79)"
    )
    assert f"{math.inf:.3g} of the largest read" in lines[1]
    assert trace_notes(limits, SimpleNamespace(hogging_bound=[0.0] * 3, hogging_read=[0.0] * 3)) == ()
