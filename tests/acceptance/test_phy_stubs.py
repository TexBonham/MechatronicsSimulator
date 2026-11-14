"""Acceptance tier stubs for the physics adapter spec (PHY), strict xfail until converted."""

import re
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

from mechsim.testing.baseline import load_baseline

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.req("PHY-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_015() -> None:
    """The MuJoCo adapter shall carry the milestone 1 acceptance cases, G1 to G4 and the pendulum."""
    raise NotImplementedError


@pytest.mark.req("PHY-023")
def test_phy_023() -> None:
    """The MuJoCo retirement check shall pass only when the native adapter passes the whole analytical suite.

    The check enumerates the reference tier (`tests/reference`) and runs it on the native adapter in a fresh
    interpreter: every adapter-parametrized test must have a native variant, the bridge's own variants (`mujoco` in the
    test id) are the only ones left out, and the run must end with no failure and no error. Strict expected failures
    are unconverted stubs, not results, and are listed by the run (PHY-023 row)."""
    collect = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/reference", "--collect-only", "-q", "-p", "no:cacheprovider"],
        capture_output=True,
        text=True,
        check=False,
        cwd=ROOT,
    )
    assert collect.returncode == 0, collect.stdout
    ids = [line for line in collect.stdout.splitlines() if "::" in line]
    bridge_only = [item for item in ids if "mujoco" in item]
    for item in bridge_only:
        assert item.replace("mujoco", "native") in ids, item
    run = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/reference", "-q", "-k", "not mujoco", "-p", "no:cacheprovider"],
        capture_output=True,
        text=True,
        check=False,
        cwd=ROOT,
    )
    summary = run.stdout.strip().splitlines()[-1]
    print(f"reference tier on native: {summary}")
    assert run.returncode == 0, run.stdout[-4000:]
    assert re.search(r"\b\d+ passed\b", summary) is not None
    assert re.search(r"\b\d+ failed\b", summary) is None
    assert re.search(r"\b\d+ errors?\b", summary) is None


FULL_STEP_TRAVEL = 0.04 / 200.0
LARGEST_BELT_LOAD = (5.5 + 0.44) * 9.81
BELT_STIFFNESS = 1.0e5
LENGTH_TOLERANCE = FULL_STEP_TRAVEL + LARGEST_BELT_LOAD / BELT_STIFFNESS
TIME_TOLERANCE = 3.0e-3


def differential_misses(bridge: Any, native: Any) -> list[str]:
    """Entries of a native baseline outside the differential tolerance of the bridge's (PHY-024 row, ruling 41)."""
    natives = {(entry.case, entry.criterion): entry for entry in native.entries}
    misses: list[str] = []
    for entry in bridge.entries:
        other = natives.get((entry.case, entry.criterion))
        if other is None or other.status != entry.status:
            status = "missing"
            if other is not None:
                status = other.status
            misses.append(f"{entry.case} {entry.criterion}: status {entry.status} on the bridge, {status} on native")
            continue
        if entry.measured is None or other.measured is None:
            continue
        unit = entry.measured.dimension
        a = entry.measured.value
        b = other.measured.value
        if unit == "[length]" and abs(a - b) > LENGTH_TOLERANCE:
            misses.append(f"{entry.case} {entry.criterion}: {a:.6g} m against {b:.6g} m")
        if unit == "[time]" and abs(a - b) > TIME_TOLERANCE:
            misses.append(f"{entry.case} {entry.criterion}: {a:.6g} s against {b:.6g} s")
        if unit == "dimensionless":
            if a != b:
                misses.append(f"{entry.case} {entry.criterion}: {a:g} against {b:g}")
            if entry.worst_time is not None and other.worst_time is not None:
                shift = abs(float(entry.worst_time) - float(other.worst_time))
                if shift > TIME_TOLERANCE:
                    misses.append(
                        f"{entry.case} {entry.criterion}: at {entry.worst_time} s against {other.worst_time} s"
                    )
    return misses


@pytest.mark.req("PHY-024")
@pytest.mark.xfail(
    strict=True,
    reason=(
        "not met (brief M3-X2): G5 and G6 error on native (the adapter refuses a mass change on the rope-hung "
        "payload); "
        "G7 hook_held and G9 load_held differ by 1.03 mm (payload on its rope and compliant lift belt against the "
        "bridge's fixed payload and rigid transmission); G10's stall, re-home and slip events come 8 to 20 ms earlier "
        "(compliant belts move the encoder-mismatch crossing)"
    ),
)
def test_phy_024() -> None:
    """The MuJoCo retirement check shall pass only when the native adapter matches the reference-design baselines within
    the differential tolerance.

    Each fixed bridge baseline (PHY-025) against the suite's native baseline: same status per criterion; lengths within
    one full motor step times the axis ratio (1.8 degrees on the 40 mm per turn pulley, 0.2 mm) plus the computed
    coupling sag, taken once for the suite at its largest (the lift belt under the 5.5 kg payload of G5 and G6 and the
    0.44 kg carriage, hook and cell: 58.3 N on 1e5 N/m, 0.583 mm); times, and the time of every event a criterion
    reads, within three control ticks (3 ms at 1 kHz); counts and presences exactly (ruling 41)."""
    native = load_baseline(ROOT / "test_cases" / "gantry" / "baseline.yaml")
    misses: list[str] = []
    for path in sorted((ROOT / "test_cases" / "gantry" / "bridge_baselines").glob("*.yaml")):
        misses.extend(differential_misses(load_baseline(path), native))
    print("\n".join(misses))
    assert misses == []
