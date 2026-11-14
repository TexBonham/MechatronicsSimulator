"""Acceptance tier stubs for the physics adapter spec (PHY), strict xfail until converted."""

import re
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from mechsim.testing.baseline import load_baseline
from mechsim.testing.schema import OpenValue, load_suite_file, load_test_case
from mechsim.units import parse_quantity

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


# ===== Differential tolerance (PHY-024 row, ruling 41 as reworded by brief M3-X2d) ==========================
# Each baseline entry is compared at the tolerance of the quantity its criterion reads, never a pooled one.
#
# Positions on a driven axis (x_axis, y_axis, lift_axis; the hook's pose x and y read the x and y axes):
#   one full motor step times the axis ratio, 1.8 degrees of a 40 mm per turn GT2 pulley: 0.04 m / 200 = 0.2 mm
#   (a homing latch shifts by whole quanta within one full step, ruling 41), plus THAT axis's belt stretch under the
#   load the belt carries at rest in THAT case, F / k with k = 1e5 N/m (gt2_20t_6mm_generic). The bridge's
#   transmission is rigid, so the stretch is the whole static difference the compliant belt adds.
#   - lift: F = (m_payload + m_carried) g, m_carried = 0.4064 kg (lift carriage, hook and load cell: the lift belt's
#     driven mass at build time, printed by every run, BLD-041), m_payload the largest payload the case hangs on the
#     hook (initial or by payload_change; the load on the belt while the lift stands): no payload 0.2 + 0.040 =
#     0.240 mm, 2 kg (G7, G9) 0.436 mm, 5.5 kg (G5, G6) 0.779 mm.
#   - x and y: F = 0. The rails are level, so at rest no external force acts along the axis; the carriage is not
#     accelerating, so the belt's force on it, m a, is zero, and the stopped stepper exerts holding torque only in
#     proportion to a displacement off its detent, which no load produces: x and y are 0.2 mm.
#   A criterion over several joints (hold_band on x, y and lift) takes the smallest of its joints' tolerances: its
#   value is the largest deviation of any of them, so the smallest tolerance is the one no joint's value can borrow.
# Times (an interval's duration) and the time of every event a criterion reads (the worst_time of a count or a
#   presence): three control ticks at 1 kHz, 3 ms (a latch or a detection takes up to three ticks). The worst_time of
#   a length statistic is the instant of an extreme inside a window, often a plateau, not an event: not compared.
# Counts and presences (dimensionless): exact.
# Any other dimension (none occurs in the fixed bridge baselines: force, angle and the rest appear only in G12, G15
#   and G16, which have no bridge baseline): exact, and reported with "no derived tolerance".
FULL_STEP_TRAVEL = 0.04 / 200.0
BELT_STIFFNESS = 1.0e5
LIFT_CARRIED_MASS = 0.4064
GRAVITY = 9.81
TIME_TOLERANCE = 3.0e-3
AXIS_OF_JOINT = {"x_axis": "x", "y_axis": "y", "lift_axis": "lift"}
AXIS_OF_POSE = {"hook.x": "x", "hook.y": "y", "hook.z": "lift"}


def _case_payload(case: Any) -> float:
    """The largest payload mass the case hangs on the hook, initial or by payload_change, open values resolved."""
    values = {parameter.name: parameter.default for parameter in case.open_parameters}
    values.update(case.parameters)
    masses = [0.0]
    payload = case.initial_state.payload
    if payload is not None and payload.attached and payload.mass is not None:
        masses.append(float(payload.mass))
    for fault in case.faults:
        if fault.kind != "payload_change":
            continue
        mass = fault.mass
        if isinstance(mass, OpenValue):
            masses.append(parse_quantity(values[mass.name], "[mass]"))
        else:
            masses.append(float(mass))
    return max(masses)


def axis_tolerance(axis: str, case: Any) -> float:
    """The position tolerance of one driven axis in one case (header derivation)."""
    if axis == "lift":
        load = (_case_payload(case) + LIFT_CARRIED_MASS) * GRAVITY
        return FULL_STEP_TRAVEL + load / BELT_STIFFNESS
    return FULL_STEP_TRAVEL


def _criterion_axes(criterion: Any) -> list[str]:
    joints = getattr(criterion, "joints", None)
    if joints is not None:
        return [AXIS_OF_JOINT[joint] for joint in joints if joint in AXIS_OF_JOINT]
    quantity = str(getattr(criterion, "quantity", ""))
    kind, _, rest = quantity.partition(":")
    if kind == "joint" and rest.endswith(".position"):
        joint = rest.removesuffix(".position")
        if joint in AXIS_OF_JOINT:
            return [AXIS_OF_JOINT[joint]]
    if kind == "pose" and rest in AXIS_OF_POSE:
        return [AXIS_OF_POSE[rest]]
    return []


def _length_tolerance(criterion: Any, case: Any) -> tuple[float, str] | None:
    axes = _criterion_axes(criterion)
    if len(axes) == 0:
        return None
    tolerances = sorted((axis_tolerance(axis, case), axis) for axis in axes)
    return tolerances[0][0], f"{tolerances[0][1]} axis"


def differential_misses(bridge: Any, native: Any, cases: dict[str, Any]) -> list[str]:
    """Entries of a native baseline outside the differential tolerance of the bridge's, each naming the criterion,
    both values, the tolerance applied and the axis or rule it came from (PHY-024 row)."""
    natives = {(entry.case, entry.criterion): entry for entry in native.entries}
    misses: list[str] = []
    for entry in bridge.entries:
        name = f"{entry.case} {entry.criterion}"
        other = natives.get((entry.case, entry.criterion))
        if other is None or other.status != entry.status:
            status = "missing"
            if other is not None:
                status = other.status
            misses.append(f"{name}: status {entry.status} on the bridge, {status} on native")
            continue
        if entry.measured is None or other.measured is None:
            continue
        case = cases[entry.case]
        criterion = next(item for item in case.criteria if item.name == entry.criterion)
        unit = entry.measured.dimension
        a = entry.measured.value
        b = other.measured.value
        if unit == "[length]":
            found = _length_tolerance(criterion, case)
            if found is None:
                if a != b:
                    misses.append(f"{name}: {a!r} m on the bridge, {b!r} m on native, no derived tolerance (exact)")
                continue
            tolerance, axis = found
            if abs(a - b) > tolerance:
                misses.append(f"{name}: {a:.6g} m on the bridge, {b:.6g} m on native, tolerance {tolerance:.4g} m ({axis})")
        elif unit == "[time]":
            if abs(a - b) > TIME_TOLERANCE:
                misses.append(f"{name}: {a:.6g} s on the bridge, {b:.6g} s on native, tolerance 0.003 s (three ticks)")
        elif unit == "dimensionless":
            if a != b:
                misses.append(f"{name}: {a:g} on the bridge, {b:g} on native, exact (count)")
            if entry.worst_time is not None and other.worst_time is not None:
                shift = abs(float(entry.worst_time) - float(other.worst_time))
                if shift > TIME_TOLERANCE:
                    misses.append(
                        f"{name}: event at {entry.worst_time} s on the bridge, {other.worst_time} s on native, "
                        "tolerance 0.003 s (three ticks)"
                    )
        elif a != b:
            misses.append(f"{name}: {a!r} on the bridge, {b!r} on native ({unit}), no derived tolerance (exact)")
    return misses


def gantry_misses() -> list[str]:
    folder = ROOT / "test_cases" / "gantry"
    suite = load_suite_file(folder / "suite.yaml")
    cases = {case.id: case for case in (load_test_case(folder / name) for name in suite.cases)}
    native = load_baseline(folder / "baseline.yaml")
    misses: list[str] = []
    for path in sorted((folder / "bridge_baselines").glob("*.yaml")):
        misses.extend(differential_misses(load_baseline(path), native, cases))
    return sorted(misses)


# ===== Misses of record (brief M3-X2d item 3; archives of both adapters, 2026-11-14) ==========================
# G7 hook_held and G9 load_held, EXPLAINED (prediction within 1 percent of the observed 1.031 mm difference). Worst
#   deviation of lift_axis: bridge 2.177 mm at 12.050 s, native 1.146 mm at 12.044 s (G7; G9 the same 20 ms earlier
#   in its own window). Power leaves the lift stepper at 12.021 s and the brake (0.5 N m, 20 ms engage delay) holds
#   from 20 ms later. Moving inertia at the belt: payload 2 kg + driven mass 0.4074 kg + rotor J / c^2 = 2.8e-5 /
#   (0.04 / 2 pi)^2 = 0.691 kg, M = 3.098 kg. Bridge (rigid drive, the fall undamped: recorded 7.6 m/s^2):
#   a = m g / M = 7.62 m/s^2, 20 ms: 1.524 mm at 0.152 m/s; brake 0.5 / c - m g = 54.9 N on M, 17.7 m/s^2, stop
#   0.656 mm; predicted 2.180 mm. Native: the stepper's viscous damping 8e-3 N m s/rad acts on the unpowered rotor,
#   b = 8e-3 / c^2 = 197 N s/m (the bridge's fall shows none), efficiency 0.95 backdriving: v_t = 0.95 m g / b =
#   0.114 m/s, tau = M / b = 15.7 ms, 20 ms: 0.988 mm at 0.082 m/s; the brake stops the rotor alone within a tick and
#   the 0.4074 kg carriage, hook and cell run on into the belt (the payload hangs on its rope): 0.082 sqrt(0.4074 /
#   1e5) = 0.165 mm; predicted 1.153 mm. Predicted difference 1.027 mm. The cause is the rotor's damping applied on
#   native and absent from the bridge's unpowered fall, not the belt's compliance (which adds 0.165 mm on native).
# G10 stall_fault (8 ms) and slip_fault (20 ms), FINDING (prediction under 70 percent of the observed). The x belt's
#   stretch under the drive torque at the fault, 0.0316 N m / c = 4.96 N on 1e5 N/m = 0.0496 mm (recorded rotor
#   to carriage 0.048 mm), starts the mismatch that much ahead; at the threshold it grows at the commanded speed
#   (stall: 28.2 mm/s) or at 0.3 of it (slip: 10.5 mm/s), a crossing 1.7 ms (stall) and 4.7 ms (slip) earlier,
#   and the event, detection + v / a with v still rising at a, moves twice that: 3.4 ms and 9.4 ms predicted
#   against 8 ms and 20 ms observed. The crossings recomputed from the archives (rest reference) move 2 ms and 5 ms,
#   as predicted; the rest of the event shift is not explained (a homing latch three quanta apart, G2, is a
#   candidate not shown).
# G10 stall_rehomed (14 ms) and slip_carriage_stopped (0.73 mm), consequences of the two above: the re-home starts
#   from the earlier stall stop (8 ms) plus a homing on native 2 ms shorter (G2 ready); the slip stop starts
#   earlier, and the commanded stop distance moves 2 v per second of detection: 2 x 0.036 m/s x 10 ms = 0.72 mm.
MISSES_OF_RECORD: dict[str, str] = {
    "G10 slip_carriage_stopped: 0.0211615 m on the bridge, 0.0204295 m on native, tolerance 0.0002 m (x axis)": (
        "the slip stop starts 10 ms earlier (slip_fault): 2 v x 10 ms = 0.72 mm; consequence of a finding"
    ),
    "G10 slip_fault: event at 31.427 s on the bridge, 31.407 s on native, tolerance 0.003 s (three ticks)": (
        "finding: x belt stretch 0.0496 mm predicts 9.4 ms of the 20 ms"
    ),
    "G10 stall_fault: event at 9.161 s on the bridge, 9.153 s on native, tolerance 0.003 s (three ticks)": (
        "finding: x belt stretch 0.0496 mm predicts 3.4 ms of the 8 ms"
    ),
    "G10 stall_rehomed: event at 12.044 s on the bridge, 12.03 s on native, tolerance 0.003 s (three ticks)": (
        "the re-home follows the stall stop 8 ms earlier and a homing 2 ms shorter; consequence of a finding"
    ),
    "G7 hook_held: 0.00217742 m on the bridge, 0.00114624 m on native, tolerance 0.0004361 m (lift axis)": (
        "explained: unpowered rotor damping on native, none on the bridge; 2.180 against 1.153 mm predicted"
    ),
    "G9 load_held: 0.00217752 m on the bridge, 0.00114618 m on native, tolerance 0.0004361 m (lift axis)": (
        "explained: as G7 hook_held, the same brake hold after the power loss"
    ),
}


@pytest.mark.req("PHY-024")
@pytest.mark.xfail(
    strict=True,
    reason=(
        "not met (ruling 83, owner decision at the gate): the six misses of record, MISSES_OF_RECORD; G7 and G9 "
        "explained (unpowered rotor damping on native only), G10's stall and slip events a finding"
    ),
)
def test_phy_024() -> None:
    """The MuJoCo retirement check shall pass only when the native adapter matches the reference-design baselines within
    the differential tolerance.

    Each fixed bridge baseline (PHY-025) against the suite's native baseline at the tolerance of the quantity each
    criterion reads (module header, the differential tolerance row): no miss at all."""
    misses = gantry_misses()
    print("\n".join(misses))
    assert misses == []


@pytest.mark.req("PHY-024")
def test_phy_024_misses_of_record() -> None:
    """The differential comparison's misses are exactly the list of record, so a new miss or a vanished one fails
    the suite (brief M3-X2d); each carries its cause in `MISSES_OF_RECORD`."""
    misses = gantry_misses()
    for miss in misses:
        print(f"{miss} -- {MISSES_OF_RECORD.get(miss, 'NOT OF RECORD')}")
    assert misses == sorted(MISSES_OF_RECORD)


@pytest.mark.req("PHY-024")
def test_tolerance_per_axis_and_case() -> None:
    """The per-axis tolerances of the header derivation, case by case (PHY-024 row)."""
    folder = ROOT / "test_cases" / "gantry"
    expected = {"G3": 0.2e-3 + 0.4064 * 9.81 / 1e5, "G7": 0.2e-3 + 2.4064 * 9.81 / 1e5}
    expected["G6"] = 0.2e-3 + 5.9064 * 9.81 / 1e5
    expected["G5"] = expected["G6"]
    for case_id, lift in expected.items():
        case = load_test_case(folder / f"{case_id}.yaml")
        assert axis_tolerance("lift", case) == pytest.approx(lift, rel=1e-12)
        assert axis_tolerance("x", case) == pytest.approx(0.2e-3, rel=1e-12)
        assert axis_tolerance("y", case) == pytest.approx(0.2e-3, rel=1e-12)


def _entry(criterion: str, dimension: str | None, value: float, worst: float | None, status: str = "pass") -> Any:
    measured = None
    if dimension is not None:
        measured = SimpleNamespace(dimension=dimension, value=value)
    return SimpleNamespace(case="G7", criterion=criterion, status=status, measured=measured, worst_time=worst)


@pytest.mark.req("PHY-024")
def test_differential_rules_per_dimension() -> None:
    """Each rule of the header on synthetic entries against G7 (lift 0.436 mm, x and y 0.2 mm): the hold band over
    x, y and lift takes the x tolerance; a length with no axis, a count, an event time and an unknown dimension."""
    case = load_test_case(ROOT / "test_cases" / "gantry" / "G7.yaml")
    case = case.model_copy(
        update={
            "criteria": [
                SimpleNamespace(name="lift", joints=["lift_axis"]),
                SimpleNamespace(name="all", joints=["x_axis", "y_axis", "lift_axis"]),
                SimpleNamespace(name="hook_x", quantity="pose:hook.x"),
                SimpleNamespace(name="other_length", quantity="pose:frame.x"),
                SimpleNamespace(name="count", quantity="signal:x_driver.step"),
                SimpleNamespace(name="span", quantity=None),
                SimpleNamespace(name="force", quantity="signal:load_cell.force"),
                SimpleNamespace(name="gone", quantity=None),
                SimpleNamespace(name="flipped", quantity=None),
                SimpleNamespace(name="errored", quantity=None),
            ]
        }
    )
    bridge = SimpleNamespace(
        entries=[
            _entry("lift", "[length]", 0.0, 1.0),
            _entry("all", "[length]", 0.0, 1.0),
            _entry("hook_x", "[length]", 0.0, 1.0),
            _entry("other_length", "[length]", 0.0, 1.0),
            _entry("count", "dimensionless", 1.0, 2.0),
            _entry("span", "[time]", 1.0, None),
            _entry("force", "[force]", 1.0, None),
            _entry("gone", "[time]", 1.0, None),
            _entry("flipped", "[time]", 1.0, None),
            _entry("errored", None, 0.0, None, "error"),
        ]
    )
    native = SimpleNamespace(
        entries=[
            _entry("lift", "[length]", 0.4e-3, 9.0),
            _entry("all", "[length]", 0.25e-3, 1.0),
            _entry("hook_x", "[length]", 0.19e-3, 1.0),
            _entry("other_length", "[length]", 1e-9, 1.0),
            _entry("count", "dimensionless", 2.0, 2.004),
            _entry("span", "[time]", 1.0035, None),
            _entry("force", "[force]", 1.0 + 1e-12, None),
            _entry("flipped", "[time]", 1.0, None, "fail"),
            _entry("errored", None, 0.0, None, "error"),
        ]
    )
    misses = differential_misses(bridge, native, {"G7": case})
    assert misses == [
        "G7 all: 0 m on the bridge, 0.00025 m on native, tolerance 0.0002 m (x axis)",
        "G7 other_length: 0.0 m on the bridge, 1e-09 m on native, no derived tolerance (exact)",
        "G7 count: 1 on the bridge, 2 on native, exact (count)",
        "G7 count: event at 2.0 s on the bridge, 2.004 s on native, tolerance 0.003 s (three ticks)",
        "G7 span: 1 s on the bridge, 1.0035 s on native, tolerance 0.003 s (three ticks)",
        "G7 force: 1.0 on the bridge, 1.000000000001 on native ([force]), no derived tolerance (exact)",
        "G7 gone: status pass on the bridge, missing on native",
        "G7 flipped: status pass on the bridge, fail on native",
    ]
