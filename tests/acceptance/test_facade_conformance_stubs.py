"""Strict-xfail stubs for the facade conformance suite (FAC, ADR 0007, ADR 0015 proposed).

Each stub describes client-observable behaviour over the protocol; the suite drives the service as a client would.
"""

import pytest


@pytest.mark.req("FAC-005")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_005() -> None:
    """A client-independent conformance suite drives the service over the protocol and every client must pass it."""
    raise NotImplementedError


@pytest.mark.req("FAC-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_006() -> None:
    """A client subscribed to a running run receives tick batches sourced from the recorder's streaming interface."""
    raise NotImplementedError


@pytest.mark.req("FAC-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_007() -> None:
    """A subscription for N points per series over a window yields at most N points per series, and the run's
    archive is identical with and without the subscription.
    """
    raise NotImplementedError


@pytest.mark.req("FAC-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_008() -> None:
    """Replay opened through the protocol serves an archive's data with no engine loaded."""
    raise NotImplementedError


@pytest.mark.req("FAC-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_010() -> None:
    """The validate, run, suite and replay operations are reachable through protocol requests and answer with ok or
    error responses.
    """
    raise NotImplementedError


@pytest.mark.req("FAC-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_011() -> None:
    """A client reads an archive's recorded series and events, and the value of any body, joint, pin or signal at
    any recorded time, through protocol requests.
    """
    raise NotImplementedError


@pytest.mark.req("FAC-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_012() -> None:
    """A client reads pass/fail reports and regression diffs through protocol requests."""
    raise NotImplementedError


@pytest.mark.req("FAC-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_013() -> None:
    """A client reads integrity results and the load-case export through protocol requests."""
    raise NotImplementedError


@pytest.mark.req("FAC-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_014() -> None:
    """A run that ends in solver failure, drift or emulator fault reaches the client as a run-ended event and a
    structured error report.
    """
    raise NotImplementedError


@pytest.mark.req("FAC-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_015() -> None:
    """A run started through the protocol produces the same archive as the same run started from the CLI."""
    raise NotImplementedError


@pytest.mark.req("FAC-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_017() -> None:
    """A client reads and writes design and test case files through the protocol, and an invalid write is refused
    with an error response.
    """
    raise NotImplementedError
