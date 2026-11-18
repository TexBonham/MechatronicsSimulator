"""Strict-xfail stubs for the REC spec (unit tier)."""

import pytest


@pytest.mark.req("REC-001")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_001() -> None:
    """The recorder shall be the single sink of run data: integrity, scoring, the viewer and exports read the recording
    and never live run state, except the tier 1 integrity party, which reads live member loads through the kernel.
    """
    raise NotImplementedError
