"""Strict-xfail stubs for the INV spec (invariant tier)."""

import pytest


@pytest.mark.req("INV-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_001() -> None:
    """The controller never sees true state. Everything between physics and controller crosses the signal
    layer as electrical signals through the pin-binding table. No convenience path from controller code
    to physics state, in either mode.
    """
    raise NotImplementedError


@pytest.mark.req("INV-002")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_002() -> None:
    """Nothing design-specific in code. The gantry exists only under designs/, test_cases/ and firmware/. A
    requirement that can only be met by special-casing a design is a gap in the library and is flagged,
    not patched.
    """
    raise NotImplementedError


@pytest.mark.req("INV-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_003() -> None:
    """Data files are the source of truth. Every YAML file validates against its schema; SI units
    internally, units explicit in files; the GUI editor, when it exists, writes the same files.
    """
    raise NotImplementedError


@pytest.mark.req("INV-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_004() -> None:
    """Determinism. The same design, test case, seed and controller mode give an identical recording. Any
    source of nondeterminism is seeded or removed.
    """
    raise NotImplementedError


@pytest.mark.req("INV-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_006() -> None:
    """Fidelity tiers are additive. Adding a higher tier to a component never changes the lower tier's
    results.
    """
    raise NotImplementedError


@pytest.mark.req("INV-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_007() -> None:
    """Failures are errors. Solver drift, solver failure and emulator faults end the run with a report;
    they never produce silent numbers.
    """
    raise NotImplementedError


@pytest.mark.req("INV-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_008() -> None:
    """The board interface is the only coupling to firmware. Firmware application code includes the
    interface header and nothing else from the simulator or any vendor HAL.
    """
    raise NotImplementedError


@pytest.mark.req("INV-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_010() -> None:
    """Every behavior change shall cite the requirement IDs it implements or changes and carry its spec
    edit in the same commit.
    """
    raise NotImplementedError


@pytest.mark.req("INV-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_011() -> None:
    """The traceability check shall pass: every non-withdrawn requirement has a citing test, every citation
    names a defined requirement, and the committed matrix equals the regenerated one.
    """
    raise NotImplementedError


@pytest.mark.req("INV-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_012() -> None:
    """Every shipped YAML file shall validate against its schema in `schemas/`."""
    raise NotImplementedError


@pytest.mark.req("INV-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_013() -> None:
    """Every schema in `schemas/` shall equal the schema generated from its pydantic model."""
    raise NotImplementedError


@pytest.mark.req("INV-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_014() -> None:
    """Running each shipped design and test case twice with the same seed and controller mode shall give
    identical recordings.
    """
    raise NotImplementedError


@pytest.mark.req("INV-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_015() -> None:
    """An executable import-boundary check over the whole repository shall fail on any engine or emulator
    type outside its adapter and on any Qt or display dependency outside viewer code.
    """
    raise NotImplementedError


@pytest.mark.req("INV-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_016() -> None:
    """Every requirement shall have a test stub committed as a strict expected failure before any code is
    written for it.
    """
    raise NotImplementedError


@pytest.mark.req("INV-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_017() -> None:
    """A stub that passes without being converted shall fail the test run."""
    raise NotImplementedError


@pytest.mark.req("INV-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_018() -> None:
    """Every test tolerance shall be stated in a spec, and a tolerance shall change only by a reviewed spec
    edit.
    """
    raise NotImplementedError


@pytest.mark.req("INV-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_inv_019() -> None:
    """CI shall run lint, type checks and every test tier headless on Ubuntu."""
    raise NotImplementedError
