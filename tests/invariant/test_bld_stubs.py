"""Invariant tier tests for the builder spec (BLD): run-twice determinism."""

from __future__ import annotations

from collections.abc import Callable

import pytest

from mechsim.builder import build
from mechsim.design.schema import Design, shipped_materials
from mechsim.library.registry import default_registry
from mechsim.physics.neutral import to_yaml
from tests.unit.test_builder_support import filled_gantry, gantry, pendulum


@pytest.mark.req("BLD-026")
@pytest.mark.parametrize("load", [pendulum, gantry, filled_gantry])
def test_bld_026(load: Callable[[], Design]) -> None:
    """Building the same design with the same inputs twice shall produce an identical neutral model, wiring, chip
    configuration and member list.
    """
    first = build(load(), default_registry(), shipped_materials(), rigid_members=load is gantry)
    second = build(load(), default_registry(), shipped_materials(), rigid_members=load is gantry)
    assert first.to_json().encode() == second.to_json().encode()
    assert first == second
    if first.physics is not None and second.physics is not None:
        assert to_yaml(first.physics).encode() == to_yaml(second.physics).encode()
    if first.chip is not None and second.chip is not None:
        assert first.chip.channels_header.encode() == second.chip.channels_header.encode()
