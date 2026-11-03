"""Strict-xfail stubs for the FAC spec (unit tier)."""

import pytest


@pytest.mark.req("FAC-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_001() -> None:
    """Every client shall drive the simulator through the one facade and through no other entry point."""
    raise NotImplementedError


@pytest.mark.req("FAC-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_005() -> None:
    """The facade shall ship a client-independent conformance suite that every client, the browser client
    and any standalone app alike, must pass.
    """
    raise NotImplementedError


@pytest.mark.req("FAC-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_006() -> None:
    """The service shall stream run data to clients from the recorder's streaming interface."""
    raise NotImplementedError


@pytest.mark.req("FAC-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_007() -> None:
    """The service shall decide what reaches a client and at what rate, decimating in the service, and
    decimation shall never change the recording.
    """
    raise NotImplementedError


@pytest.mark.req("FAC-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_008() -> None:
    """Replay through the facade shall read run archives only, never live engine state."""
    raise NotImplementedError


@pytest.mark.req("FAC-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_009() -> None:
    """The facade service shall be Python in `mechsim/facade/`, calling the core runtime through its
    pybind11 binding.
    """
    raise NotImplementedError


@pytest.mark.req("FAC-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_010() -> None:
    """The facade shall expose the validate, run, suite and replay operations the CLI commands use."""
    raise NotImplementedError


@pytest.mark.req("FAC-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_011() -> None:
    """The facade shall expose an archive's recorded series and events, and the value of any body, joint,
    pin or signal at any recorded time.
    """
    raise NotImplementedError


@pytest.mark.req("FAC-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_012() -> None:
    """The facade shall expose pass/fail reports and regression diffs."""
    raise NotImplementedError


@pytest.mark.req("FAC-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_013() -> None:
    """The facade shall expose integrity results and the load-case export."""
    raise NotImplementedError


@pytest.mark.req("FAC-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_014() -> None:
    """The facade shall return every run error (solver failure, drift, emulator fault) to the client as a
    structured error report.
    """
    raise NotImplementedError


@pytest.mark.req("FAC-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_015() -> None:
    """A run started through the facade by any client shall produce the same archive as the same run
    started from the CLI.
    """
    raise NotImplementedError


@pytest.mark.req("FAC-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_017() -> None:
    """The facade shall expose reading and writing design and test case files, validating every write
    against the file's schema.
    """
    raise NotImplementedError
