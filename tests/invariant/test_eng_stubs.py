"""ENG stubs, invariant tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("ENG-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_018() -> None:
    """The engine shall not link the reference collision library; that library shall be used only inside the test
    suite.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_023() -> None:
    """Given the same model, inputs and step on the same machine and build, the engine shall produce bitwise
    identical state after every step.
    """
    raise NotImplementedError
