"""Invariant tier stubs for the component library spec (LIB), strict xfail until converted. LIB-021 is converted for
the milestone 2 types only; its stub stays for the rest of the catalog."""

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
# Catalog type -> the C++ built-in class its reference tests construct.
M2_BUILT_INS = {"load_cell": "LoadCell", "load_cell_amplifier": "LoadCellAmplifier", "imu": "Imu", "brake": "Brake"}


def reference_test_cases() -> list[str]:
    """Every Catch2 TEST_CASE in a reference-tier source (`test_reference_*.cpp`, the targets labelled reference)."""
    cases: list[str] = []
    for source in sorted((ROOT / "core").rglob("tests/test_reference_*.cpp")):
        cases += source.read_text(encoding="utf-8").split("TEST_CASE(")[1:]
    return cases


@pytest.mark.req("LIB-021")
@pytest.mark.parametrize(("component_type", "built_in"), sorted(M2_BUILT_INS.items()))
def test_lib_021_milestone_2_types_have_a_reference_test(component_type: str, built_in: str) -> None:
    """Each milestone 2 type has a reference-tier test tagged LIB-021 that constructs its built-in and compares it with
    a closed form (the tolerances sit in the test's derivation comment, LIB-021)."""
    catalog = (ROOT / "mechsim" / "library" / "catalog" / f"{component_type}.yaml").read_text(encoding="utf-8")
    assert f"cpp: {component_type}" in catalog
    constructs = re.compile(rf"\b{built_in} \w+\(")
    tagged = [case for case in reference_test_cases() if "[LIB-021]" in case.split(") {")[0]]
    assert any(constructs.search(case) for case in tagged), f"no LIB-021 reference test constructs {built_in}"


@pytest.mark.req("LIB-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_006() -> None:
    """Adding a higher fidelity tier to a component type shall leave the results of every lower tier of that type
    unchanged.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_021() -> None:
    """Every shipped component type shall have a reference test that compares it with an analytical result or a
    datasheet curve within a tolerance stated in this spec.
    """
    raise NotImplementedError
