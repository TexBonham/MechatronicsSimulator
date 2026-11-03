"""Invariant tier stubs for the component library spec (LIB), strict xfail until converted."""

import pytest


@pytest.mark.req("LIB-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_006() -> None:
    """Adding a higher fidelity tier to a component type shall leave the results of every lower tier of that type
    unchanged.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_010() -> None:
    """Every shipped catalog file shall be YAML and shall validate against the JSON Schema for its component type."""
    raise NotImplementedError


@pytest.mark.req("LIB-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_021() -> None:
    """Every shipped component type shall have a reference test that compares it with an analytical result or a
    datasheet curve within a tolerance stated in this spec.
    """
    raise NotImplementedError
