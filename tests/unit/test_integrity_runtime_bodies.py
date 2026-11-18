"""Node indexing with bodies a case adds at run time (INT-006, brief I3 item 2): a cantilever lumped member plus a case
payload hung on a rope from a separate post. On the native adapter the payload and its shackle are published after
the build's bodies and before the member's nodes, so a node index taken from the build's layout is two short. Tier 2
reads the nodes by the recorder's body names, tier 1 by the prepared native model's, and the party requires the
published body count exactly."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import pytest

from mechsim.integrity.service import IntegrityService
from mechsim.testing.core_driver import CoreDriver, RecordArchiver, core_builder
from mechsim.testing.runner import Services, run_case
from mechsim.testing.schema import ROOT

# The B1 bar (designs/cantilever_beam): E I = 170.8333 N m^2, w = rho A g = 7.70085 N/m, L = 1 m, clamped at the
# world origin, gravity -y. Static tip deflection of the continuum w L^4 / (8 E I) = 5.634768e-3 m; first mode
# f_1 = 8.255100 Hz. The post, its rope and the payload carry no load to the bar.
STATIC = 5.634768e-3
FIRST_MODE = 8.255100
WINDOW_START = 0.25
END = 8.25

DESIGN = (
    (ROOT / "designs" / "cantilever_beam" / "design.yaml")
    .read_text(encoding="utf-8")
    .replace("name: cantilever_beam\n", "name: cantilever_payload\n", 1)
    + """  - name: post_mount
    type: fixed
    parent: world
    child: post
    placement:
      position: [-1 m, 1 m, 0 m]
ropes:
  - name: hanger
    part: rope_3mm_steel_generic
    from:
      body: post
      point: [0 m, 0 m, 0 m]
    to:
      payload: payload
    segments: 8
    rest_length: 0.3 m
structural:
  design_load:
    rated_load: 1 kg
    allowance: 0 percent
  accuracy_target: 5 cm
  deflection_target: static
  minimum_safety_factor: "1.5"
"""
)
POST = """  - name: post
    material: steel_8_8_fastener
    process:
      kind: extrusion
    geometry:
      kind: primitives
      shapes:
        - shape: sphere
          position: [0 m, 0 m, 0 m]
          radius: 0.02 m
joints:
"""
CASE = f"""id: C1
title: Cantilever beside a payload on a rope
design:
  name: cantilever_payload
  version: 1.0.0
seed: 0
initial_state:
  joints: []
  payload:
    attached: true
    body: post
    mass: 1 kg
rates:
  physics_step: 1 ms
physics:
  adapter: native
  substeps: 10
end:
  at: {END} s
  max_duration: 10 s
criteria:
  - name: settled_deflection
    check: value
    quantity: deflection:bar
    statistic: mean
    window:
      start: {WINDOW_START} s
    comparison: within
    target: {STATIC} m
    relative_limit: "0.02"
"""


def _write(root: Path) -> Path:
    folder = root / "designs" / "cantilever_payload"
    folder.mkdir(parents=True)
    (folder / "design.yaml").write_text(DESIGN.replace("joints:\n", POST, 1), encoding="utf-8")
    (folder / "integrity.yaml").write_text("schema_version: 1\nchecks:\n  - check: deflection\n", encoding="utf-8")
    case = root / "C1.yaml"
    case.write_text(CASE, encoding="utf-8")
    return case


@pytest.mark.req("INT-006", "INT-023")
def test_run_time_bodies_shift_no_node_index_in_either_tier(tmp_path: Path) -> None:
    """Tier 2: the mean of the node-pose deflection over [0.25 s, 8.25 s] is the chain's static value to within
    A / (pi f_1 T) for an oscillation of amplitude A about it (the bar is released straight, so A <= the static value
    when undamped): 1 / (pi 8.2551 8) = 0.48 percent; the chain's static value is within the INT-006 row's 2 percent
    of the continuum. The criterion asserts the raw row (2 percent); a node series shifted by the two run-time bodies
    reads the post, the shackle or the payload instead and misses it by orders of magnitude (1.38 m on G15). Tier 1:
    the party computes the same node-pose deflection from the published poses each tick, with no design-load factor
    (the allowance is 0 here), so its peak equals the post-run table's peak to the INT-006 row's 1e-6 relative of the
    chain's own value; the run would fault on a published body count other than the table's."""
    case = _write(tmp_path)
    designs = tmp_path / "designs"
    driver = CoreDriver(designs_root=designs)
    services = Services(
        lambda: driver, core_builder, RecordArchiver(designs), designs, integrity=IntegrityService(designs)
    )
    result: Any = run_case(case, services, None, tmp_path / "runs")
    assert result.report is not None, result.score.error
    assert not result.report.errored, result.report.error
    (criterion,) = result.score.criteria
    report = result.metadata.integrity
    print(f"settled deflection: closed form {STATIC} m, measured {criterion.measured}")
    assert criterion.status == "pass", criterion
    tier2 = report.member("bar").deflection
    trace = driver.integrity_trace
    assert trace is not None
    (tier1,) = trace.peak_deflection
    print(f"peak deflection: tier 1 {tier1} m, tier 2 {tier2} m")
    assert math.isfinite(tier2)
    assert tier1 == pytest.approx(tier2, rel=1e-6)
    assert list(trace.crossings) == []
