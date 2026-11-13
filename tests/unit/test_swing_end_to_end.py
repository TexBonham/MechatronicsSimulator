"""The swing angle on a real recording (SCO-030, ACC-025, brief I3b items 1 and 2): a rigid pendulum design run
through `CoreDriver` and the runner, so the angle is formed from the record view's `pose:` displacements (TST-038) and
the design's gravity reaches scoring through the runner."""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from mechsim.testing.core_driver import CoreDriver, RecordArchiver, core_builder
from mechsim.testing.runner import Services, design_gravity, run_case

# A point bob on a light arm: the arm (1 g, inertia 1e-9 kg m^2) turns on an undamped revolute joint at the world
# origin with its centre of mass on the axis; the bob (1 kg, inertia 1e-9 kg m^2 about its centre) is fixed to the arm
# L = 1 m below, straight down at joint coordinate zero, which is the reference configuration. Gravity (0, -9.81, 0)
# m/s^2. Pivot inertia I = 1 + 2e-9 kg m^2, restoring moment m g L sin theta, so T = 2 pi sqrt(I / (m g L)) =
# 2 pi sqrt(L / g) (1 + 1e-9) at small amplitude. Native adapter, physics step h = 1 ms (one recorded row per step,
# the first at t = h), n = 10 engine substeps of h_s = h / n; omega = sqrt(g / L), x = omega h = 3.13e-3,
# x_s = omega h_s = 3.13e-4.
#
# Peak (SCO-030, recorded tier): the semi-implicit Euler orbit of an undamped swing released at rest from theta_0 never
# exceeds theta_0 / sqrt(1 - x_s^2 / 4), 1.2e-8 relative above it; a recorded row falls within h / 2 of each turning
# point, at most (x / 2)^2 / 2 = 1.23e-6 relative below the orbit's maximum. So the peak is theta_0 to
# x^2 / 8 + x_s^2 / 8 = 1.238e-6 relative. The release itself is not a recorded row: the first row (t = h) is the
# scheme's theta_0 (1 - x^2 (1 + 1 / n) / 2), 5.4e-6 below, which a later turning point beats.
# Period about the mean, against 2 pi sqrt(L / g): finite amplitude theta_0^2 / 16 = 6.3e-8 (theta_0 = 1e-3 rad), the
# integrator's detuning x_s^2 / 24 = 4.1e-9, the arm and bob inertia 1e-9, the crossing interpolation below
# omega^2 h^3 / 8 per crossing; sum 7e-8, inside the SCO-030 row's 1e-6 relative. (A damped swing would bias the mean
# toward the release side and shift each crossing about it as the amplitude decays, so the swing is undamped.)
# At rest (theta_0 = 0) the torque m g L sin 0 is zero, every displacement stays zero and the line is the reference
# origins' (0, -L, 0), along gravity: atan2(0, L) = acos(1) = 0 exactly.
G = 9.81
L = 1.0
THETA_0 = 1e-3
PERIOD = 2.0 * math.pi * math.sqrt(L / G)
STEP = 1e-3
SUBSTEPS = 10
X = math.sqrt(G / L) * STEP
PEAK_LIMIT = X * X / 8.0 + (X / SUBSTEPS) ** 2 / 8.0
END = 8.5

DESIGN = f"""name: swing_pendulum
version: 1.0.0
source_hash: unset
status: release
description: A point bob on a light arm, the swing angle's end-to-end reference.
environment:
  gravity: [0 m/s**2, -{G} m/s**2, 0 m/s**2]
  ambient_temperature: 298.15 K
bodies:
  - name: arm
    material: aluminium_6063_t5_extrusion
    process:
      kind: extrusion
    geometry:
      kind: primitives
      shapes:
        - shape: sphere
          position: [0 m, 0 m, 0 m]
          radius: 0.005 m
    mass_properties:
      mass: 0.001 kg
      com: [0 m, 0 m, 0 m]
      inertia:
        ixx: 1e-9 kg*m**2
        iyy: 1e-9 kg*m**2
        izz: 1e-9 kg*m**2
  - name: bob
    material: aluminium_6063_t5_extrusion
    process:
      kind: extrusion
    geometry:
      kind: primitives
      shapes:
        - shape: sphere
          position: [0 m, 0 m, 0 m]
          radius: 0.01 m
    mass_properties:
      mass: 1 kg
      com: [0 m, 0 m, 0 m]
      inertia:
        ixx: 1e-9 kg*m**2
        iyy: 1e-9 kg*m**2
        izz: 1e-9 kg*m**2
joints:
  - name: pivot
    type: revolute
    parent: world
    child: arm
    placement:
      position: [0 m, 0 m, 0 m]
    axis: [0.0, 0.0, 1.0]
    angular:
      - friction: 0 N*m
        damping: 0 N*m*s/rad
        backlash: 0 rad
  - name: bob_mount
    type: fixed
    parent: arm
    child: bob
    placement:
      position: [0 m, -{L} m, 0 m]
"""


def _case(identifier: str, start: float, criteria: str) -> str:
    return f"""id: {identifier}
title: Swing of a point pendulum
design:
  name: swing_pendulum
  version: 1.0.0
seed: 0
initial_state:
  joints:
    - joint: pivot
      position: {start!r} rad
      velocity: 0 rad/s
rates:
  physics_step: {STEP!r} s
physics:
  adapter: native
  substeps: {SUBSTEPS}
end:
  at: {END} s
  max_duration: 10 s
criteria:
{criteria}"""


SWING_CRITERIA = f"""  - name: release_peak
    check: value
    quantity: swing:arm.bob.x
    statistic: peak_absolute
    comparison: within
    target: {THETA_0!r} rad
    relative_limit: "{PEAK_LIMIT!r}"
  - name: unsigned_peak
    check: value
    quantity: swing:arm.bob
    statistic: maximum
    comparison: within
    target: {THETA_0!r} rad
    relative_limit: "{PEAK_LIMIT!r}"
  - name: period
    check: value
    quantity: swing:arm.bob.x
    statistic: period_about_mean
    comparison: within
    target: {PERIOD!r} s
    relative_limit: "1e-6"
"""
REST_CRITERIA = """  - name: rest_unsigned
    check: value
    quantity: swing:arm.bob
    statistic: maximum
    comparison: at_most
    limit: 0 rad
  - name: rest_signed
    check: value
    quantity: swing:arm.bob.x
    statistic: peak_absolute
    comparison: at_most
    limit: 0 rad
"""


def _run(root: Path, identifier: str, start: float, criteria: str) -> Any:
    folder = root / "designs" / "swing_pendulum"
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "design.yaml").write_text(DESIGN, encoding="utf-8")
    case = root / f"{identifier}.yaml"
    case.write_text(_case(identifier, start, criteria), encoding="utf-8")
    designs = root / "designs"
    services = Services(lambda: CoreDriver(designs_root=designs), core_builder, RecordArchiver(designs), designs)
    result: Any = run_case(case, services, None, root / "runs")
    assert result.report is not None, result.score.error
    assert not result.report.errored, result.report.error
    return result


@pytest.mark.req("SCO-030", "ACC-025", "TST-038")
def test_a_recorded_pendulum_swings_at_its_release_angle_and_closed_form_period(tmp_path: Path) -> None:
    result = _run(tmp_path, "S1", THETA_0, SWING_CRITERIA)
    measured = {criterion.name: criterion for criterion in result.score.criteria}
    print(f"swing peak: closed form {THETA_0} rad, measured {measured['release_peak'].measured}")
    print(f"unsigned peak: closed form {THETA_0} rad, measured {measured['unsigned_peak'].measured}")
    print(f"swing period: closed form {PERIOD} s, measured {measured['period'].measured}")
    for criterion in result.score.criteria:
        assert criterion.status == "pass", criterion
    assert result.score.status == "pass"


@pytest.mark.req("SCO-030", "TST-038")
def test_a_recorded_pendulum_at_rest_has_a_swing_of_exactly_zero(tmp_path: Path) -> None:
    result = _run(tmp_path, "S2", 0.0, REST_CRITERIA)
    for criterion in result.score.criteria:
        print(f"{criterion.name}: closed form 0 rad, measured {criterion.measured}")
        assert criterion.status == "pass", criterion
    assert result.score.status == "pass"


@pytest.mark.req("SCO-030")
def test_the_runner_gives_scoring_the_design_gravity_or_none_while_it_is_open() -> None:
    resolved = SimpleNamespace(environment=SimpleNamespace(gravity=[0.0, -G, 0.0]))
    assert design_gravity(resolved) == (0.0, -G, 0.0)  # type: ignore[arg-type]
    open_value = SimpleNamespace(environment=SimpleNamespace(gravity=[0.0, object(), 0.0]))
    assert design_gravity(open_value) is None  # type: ignore[arg-type]
