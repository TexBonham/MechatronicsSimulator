"""Session-wide fixtures shared by the test tiers."""

from collections.abc import Mapping
from pathlib import Path

import pytest

from mechsim.testing.core_driver import CoreDriver, core_services
from mechsim.testing.runner import RunResult, Services, run_case


class CaseRuns:
    """One run per case file, adapter bindings and overrides for the whole session (brief M3-X2d): a case several
    test modules read through the real stack is run once, on first use, with no archive written. A test that needs a
    run of its own (run-twice determinism, a run into a folder, a run of an edited copy) calls `run_case` itself and
    never goes through this cache."""

    def __init__(self) -> None:
        self._runs: dict[tuple[str, tuple[tuple[str, str], ...]], tuple[RunResult, CoreDriver]] = {}

    def get(self, case: Path, bindings: Mapping[str, str] | None = None) -> tuple[RunResult, CoreDriver]:
        given: Mapping[str, str] = {}
        if bindings is not None:
            given = bindings
        key = (str(case.resolve()), tuple(sorted(given.items())))
        if key not in self._runs:
            driver = CoreDriver()
            services = core_services()
            result = run_case(case, Services(lambda: driver, services.builder), bindings, None)
            self._runs[key] = (result, driver)
        return self._runs[key]


@pytest.fixture(scope="session")
def case_runs() -> CaseRuns:
    """The session's cache of whole-case runs (`CaseRuns`)."""
    return CaseRuns()
