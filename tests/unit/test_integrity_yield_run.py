"""A native run with tier 2 strength (the yield offset, INT-028) through `CoreDriver` (brief I3 item 3): the integrity
party's model-change requests reach physics and the row's coupling column; their Python form is a read-only value
copy (REC-020, REC-022), never a "no Python form" error."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from mechsim import _core as core
from mechsim.integrity.service import IntegrityService
from mechsim.integrity.tier1 import MODEL_REQUESTS
from mechsim.record.layout import COUPLING_DIGEST
from mechsim.testing.core_driver import CoreDriver, RecordArchiver, core_builder
from mechsim.testing.runner import Services, run_case
from mechsim.testing.schema import ROOT

# The B1 bar under its own weight: root moment w L^2 / 2 = 3.85 N m. With the fibre at 1 m (the dynamics read only
# E I) the root fibre stress M c / I = 3.85 / 8.333e-10 = 4.6e9 Pa passes any steel yield, so the party requests a
# spring offset as soon as the root moment passes M_y = sigma_y I / c (below 1 N m for sigma_y under 1.2 GPa).
DESIGN = (
    (ROOT / "designs" / "cantilever_beam" / "design.yaml")
    .read_text(encoding="utf-8")
    .replace("name: cantilever_beam\n", "name: cantilever_yield\n", 1)
    .replace("fibre_y: 5 mm", "fibre_y: 1 m", 1)
)
CASE = """id: Y1
title: Cantilever yielding under its own weight
design:
  name: cantilever_yield
  version: 1.0.0
seed: 0
initial_state:
  joints: []
rates:
  physics_step: 1 ms
physics:
  adapter: native
end:
  at: 0.2 s
  max_duration: 1 s
criteria:
  - name: tip_below_root
    check: value
    quantity: pose:bar.y
    statistic: final
    comparison: at_most
    limit: 0 m
"""


@pytest.mark.req("INT-028", "REC-022")
def test_a_native_run_with_the_yield_offset_records_its_requests(tmp_path: Path) -> None:
    folder = tmp_path / "designs" / "cantilever_yield"
    folder.mkdir(parents=True)
    (folder / "design.yaml").write_text(DESIGN, encoding="utf-8")
    (folder / "integrity.yaml").write_text(
        "schema_version: 1\nchecks:\n  - check: strength\n    tier: 2\n", encoding="utf-8"
    )
    case = tmp_path / "Y1.yaml"
    case.write_text(CASE, encoding="utf-8")
    designs = tmp_path / "designs"
    driver = CoreDriver(designs_root=designs)
    services = Services(
        lambda: driver, core_builder, RecordArchiver(designs), designs, integrity=IntegrityService(designs)
    )
    result: Any = run_case(case, services, None, tmp_path / "runs")
    assert result.report is not None, result.score.error
    assert not result.report.errored, result.report.error
    assert len(driver.integrity_trace.requests) > 0
    digests = driver.recording().recording.series[f"{COUPLING_DIGEST}/{MODEL_REQUESTS}"].values
    assert any(len(digest) > 0 for digest in digests)


@pytest.mark.req("INT-028", "REC-020")
def test_the_requests_payload_is_a_read_only_value() -> None:
    first = core.ModelChangeRequest(core.ModelChangeKind.SpringOffset, 3, 1e-4)
    batch = core.ModelChangeRequests(7, [first])
    assert (batch.sequence, len(batch.requests)) == (7, 1)
    (copy,) = batch.requests
    assert (copy.kind, copy.index, copy.value) == (core.ModelChangeKind.SpringOffset, 3, 1e-4)
    batch.requests.append(first)
    assert len(batch.requests) == 1
    with pytest.raises(AttributeError):
        batch.sequence = 8  # type: ignore[misc]
    with pytest.raises(AttributeError):
        batch.requests = []  # type: ignore[misc]
    with pytest.raises(AttributeError):
        copy.value = 0.0  # type: ignore[misc]
