"""SIG stubs, reference tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("SIG-030")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_030() -> None:
    """A component on the Python slow path and its C++ built-in counterpart shall implement the same interface and
    produce identical results on the shared reference tests.
    """
    raise NotImplementedError
