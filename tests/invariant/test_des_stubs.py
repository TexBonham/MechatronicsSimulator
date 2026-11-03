"""Invariant tier stubs for the design spec (DES), strict xfail until converted."""

import pytest


@pytest.mark.req("DES-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_011() -> None:
    """Every shipped design file shall be YAML and shall validate against the design JSON Schema."""
    raise NotImplementedError


@pytest.mark.req("DES-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_012() -> None:
    """The design schema shall be generated from pydantic v2 models, checked in under `schemas/`, and a staleness test
    shall fail when the checked-in schema differs from a fresh generation.
    """
    raise NotImplementedError
