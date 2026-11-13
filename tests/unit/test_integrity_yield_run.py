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


@pytest.mark.req("INT-028", "INT-005")
def test_a_yielded_spring_reads_k_times_its_elastic_rotation(tmp_path: Path) -> None:
    """Item 2 of brief I4b (ruling 35): the post-run check reads the offsets the party requested from the recorded
    `model_change` events and takes k (total rotation - offset) at each spring. Oracle: the root spring's moment by
    statics over the stub, M_n = -M_z0 + (l / 2) F_y0 from the recorded start wrench (the root joint's reaction: its
    spring torque, no node damping in this design), equal to k times the elastic rotation at every row within 1e-9
    relative; the total rotation reads k times the offset more. The placeholder rule (ruling 35) relieves no moment in
    a statically determinate member: the root moment stays the statics w L^2 / 2, so the stress there stays
    M_n c_y / I_z, above the yield stress by (|M_n| - M_y) c_y / I_z; the offset changes the shape, not the moment."""
    import numpy as np

    from mechsim.integrity.beam import node_moments
    from mechsim.integrity.inputs import BODY_ORIENTATION, BODY_POSITION, MEMBER_SERIES, node_chains, spring_offsets
    from mechsim.integrity.plugin import BodyPoses, MemberInputs, SectionProperties, Wrenches

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
    real = IntegrityService(designs)
    seen: dict[str, Any] = {}

    def integrity(build: Any, run: Any, recording: Any) -> Any:
        seen["build"] = build
        seen["recording"] = recording
        return real(build, run, recording)

    services = Services(lambda: driver, core_builder, RecordArchiver(designs), designs, integrity=integrity)
    result: Any = run_case(case, services, None, tmp_path / "runs")
    assert result.report is not None, result.score.error
    assert not result.report.errored, result.report.error
    model = seen["build"].physics
    raw = seen["recording"].recording
    rows = len(seen["recording"].times())
    offsets = spring_offsets(model, tuple(raw.events), rows)
    (name,) = offsets
    chain = node_chains(model)[name]
    count = len(model.bodies) + chain.count
    poses = BodyPoses(
        np.asarray(raw.series[BODY_POSITION].values, dtype=np.float64).reshape(rows, count, 3),
        np.asarray(raw.series[BODY_ORIENTATION].values, dtype=np.float64).reshape(rows, count, 4),
    )
    wrenches = [np.asarray(raw.series[path].values, dtype=np.float64).reshape(rows, 3) for path in MEMBER_SERIES]
    section = SectionProperties(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    member = MemberInputs(name, "steel", section, ((0.0, "clamped"),), Wrenches(*wrenches), nodes=chain)
    elastic, _ = node_moments(member, poses, offsets[name])
    total, _ = node_moments(member, poses)
    statics = -wrenches[1][:, 2] + 0.5 * chain.segment_length * wrenches[0][:, 1]
    yielded = np.nonzero(offsets[name][:, 0])[0]
    assert yielded.size > 0
    first = int(yielded[0])
    print(f"root spring after the first offset (row {first}): statics {statics[-1]!r}, elastic {elastic[-1, 0]!r}")
    assert elastic[first:, 0] == pytest.approx(statics[first:], rel=1e-9)
    assert total[first:, 0] - elastic[first:, 0] == pytest.approx(chain.stiffness * offsets[name][first:, 0], rel=1e-9)
    assert not np.any(offsets[name][:, 1:])
