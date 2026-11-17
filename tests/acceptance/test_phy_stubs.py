"""Acceptance tier stubs for the physics adapter spec (PHY), strict xfail until converted."""

import re
import subprocess
import sys
from functools import cache
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from mechsim.library.registry import default_registry
from mechsim.physics.driven import driven_mass
from mechsim.testing.baseline import load_baseline
from mechsim.testing.core_driver import core_builder
from mechsim.testing.runner import prepare
from mechsim.testing.schema import OpenValue, load_suite_file, load_test_case
from mechsim.units import parse_quantity
from tests.conftest import CaseRuns

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.req("PHY-023")
@pytest.mark.long_run
def test_phy_023() -> None:
    """The MuJoCo retirement check shall pass only when the native adapter passes the whole analytical suite.

    Met at retirement (ADR 0004, 2026-11-15) and kept as the native adapter's standing check: the whole reference tier
    (`tests/reference`), every test on the native adapter (the only one), runs in a fresh interpreter and must end with
    no failure and no error. Strict expected failures
    are unconverted stubs, not results, and are listed by the run (PHY-023 row)."""
    run = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/reference", "-q", "-p", "no:cacheprovider"],
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
#     driven mass at build time, printed by every run, BLD-041; read from the gantry build by `lift_masses`, not
#     written here), m_payload the largest payload the case hangs on the hook (initial or by payload_change; the load
#     on the belt while the lift stands): no payload 0.2 + 0.040 = 0.240 mm, 2 kg (G7, G9) 0.436 mm, 5.5 kg (G5, G6)
#     0.779 mm. A native run that hangs a payload adds the rope's end fitting under the hook (1 g, the run-time
#     payload_shackle, so the run's damping line reads 0.4074 kg); the row takes the build's driven mass and leaves
#     the fitting's 0.1 um of stretch out.
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
GRAVITY = 9.81
TIME_TOLERANCE = 3.0e-3
AXIS_OF_JOINT = {"x_axis": "x", "y_axis": "y", "lift_axis": "lift"}
AXIS_OF_POSE = {"hook.x": "x", "hook.y": "y", "hook.z": "lift"}


@cache
def lift_masses() -> tuple[float, float]:
    """The lift belt's driven mass at build time (BLD-041: the subtree of the belt's driven joint, lift carriage, hook
    and load cell, as the build's damping line states it) and the payload rope's end fitting (LIB-020
    `end_fitting_mass`, the run-time `<payload>_shackle` body on the hook whenever a native run hangs a payload), kg,
    from the gantry build and its rope part: no literal."""
    prepared = prepare(ROOT / "test_cases" / "gantry" / "G7.yaml")
    model = core_builder(prepared.design, "native").physics
    assert model is not None
    belt = next(item for item in model.transmissions if item.name == "lift_belt")
    driven = next(body for body in model.bodies if body.joint is not None and body.joint.name == belt.driven_joint)
    rope = next(item for item in prepared.design.ropes if item.name == "payload_rope")
    fitting = float(default_registry().resolve_part(rope.part, {}).values["end_fitting_mass"])
    return driven_mass(model, driven.name), fitting


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
        load = (_case_payload(case) + lift_masses()[0]) * GRAVITY
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
                values = f"{a:.6g} m on the bridge, {b:.6g} m on native"
                misses.append(f"{name}: {values}, tolerance {tolerance:.4g} m ({axis})")
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


NATIVE_AT_RETIREMENT = "native_at_retirement.yaml"


def gantry_misses() -> list[str]:
    folder = ROOT / "test_cases" / "gantry"
    suite = load_suite_file(folder / "suite.yaml")
    cases = {case.id: case for case in (load_test_case(folder / name) for name in suite.cases)}
    # The comparison of record is frozen (ruling 99 (c)): the native baseline at retirement, never the live
    # baseline.yaml, which tracks the design.
    native = load_baseline(folder / "bridge_baselines" / NATIVE_AT_RETIREMENT)
    misses: list[str] = []
    for path in sorted((folder / "bridge_baselines").glob("*.yaml")):
        if path.name == NATIVE_AT_RETIREMENT:
            continue
        misses.extend(differential_misses(load_baseline(path), native, cases))
    return sorted(misses)


# ===== Misses of record (brief M3-X2d item 3; mechanism of G7 and G9 brief M3-X2g item 1; list M3-X2h) ==========
# Four misses, all G10 (brief M3-X2h: the native baseline regenerated whole after E8). G7 hook_held and G9
# load_held are no longer misses: explained as a modelling difference (below), inside their 0.436 mm tolerance
# since E8: bridge 2.17742 and 2.17752 mm, native 1.82075 and 1.82062 mm, 0.357 mm apart.
#   Bridge 2.177 mm at 12.050 s, native of record 1.146 mm at 12.044 s (G9 the same in its own window). The lift
#   stepper is disabled from 12.022 s (SIG-033: a disabled drive, K = D = 0, cleared by the native adapter), the
#   brake (0.5 N m, 20 ms engage delay) acts from 12.042 s. Masses at the belt (n = 0.04 / 2 pi m/rad): rotor
#   J / n^2 = 2.8e-5 / 4.053e-5 = 0.691 kg; carriage, hook and load cell 0.4064 kg (the build's driven mass, BLD-041)
#   plus the rope's end fitting 1 g (`end_fitting_mass`, the run's payload_shackle): 0.4074 kg; payload 2 kg.
#   Bridge (rigid drive, payload welded to the hook): a = 2.4074 g / 3.098 = 7.62 m/s^2 (recorded 7.60) for 20 ms,
#   1.524 mm at 0.152 m/s; the brake, 0.5 / n - 2.4074 g = 54.9 N on 3.098 kg, stops it in 0.656 mm: 2.180 mm.
#   Native: three bodies, rotor - belt (k 1e5 N/m, D 4.03 N s/m) - carriage - rope (E A / L = 1.667e6 N/m, 167 N s/m,
#   unilateral) - payload, backward Euler at 1 ms. Two engine mechanisms, no rotor damping (the drive is cleared):
#   (a) the belt's efficiency loss (ruling 47, brief M3-E6) is an implicit shaft damping c = alpha L / max(omega,
#       omega_s) frozen at the step's START rate; released from rest, omega_s = 1e-4 rad/s gives c = 0.0526 x 0.151 /
#       1e-4 = 79 N m s/rad, and while omega grows twentyfold a step the loss is most of L, not 5 percent: the
#       recorded rotor rate 0.002, 0.038, 0.655, 3.86 rad/s in the first four steps, where the 0.95 efficiency gives
#       0.95 L / J = 5100 rad/s^2, 5 rad/s after one step. The release is held back about 2 ms.
#   (b) the rope: the belt unloads first, the light carriage is pulled down past the payload and the rope goes slack
#       at 12.030 s; before brief M3-E8 a rope segment's slack state was decided at the step's start and a segment at
#       its rest length counted slack, so the rope did not take up again inside the window: the brake stopped the
#       rotor and the 0.4074 kg carriage alone ran 0.15 mm into the belt while the payload fell free.
#   A reduced model of exactly these terms (three masses, the springs and dampers above, backward Euler at 1 ms, c
#   frozen per step as in (a), the brake as a Coulomb torque then held) gives native 1.140 mm against the recorded
#   1.146 mm, and the bridge's 2.180 mm against 2.177 mm: difference 1.040 mm predicted against 1.031 mm observed.
#   Its steps: compliant belt and rope, lossless 2.418 mm (+0.238); with (a) 1.811 mm (-0.607); with (b) 1.140 mm
#   (-0.670). The engine at the head (E8 merged, 8803705: the rope takes up within the step) records 1.821 mm
#   (model 1.811 mm), so G7 now differs from the bridge by 0.357 mm, inside its 0.436 mm tolerance; the baseline
#   regenerated after E8 (M3-X2h) holds 1.82075 mm (G9 1.82062 mm). With the engine of 3c06a8e built in place, G7 on
#   native records 1.14623 mm, the pre-E8 value of record to 2 nm (brief M3-X2g run). Mechanism (a) remains, bounded
#   and published per run (brief M3-X2i, engine transmission.hpp (iv); G7 lift belt ratio 19.97), not a miss.
#   Ruled out: the stepper's viscous damping (SIG-033 publishes a disabled drive with D = 0; the reduced model needs
#   none), the belt's modal damping (4.03 N s/m on the stretch rate, 0.01 of critical), the brake's engagement (the
#   same 20 ms on both adapters; it only ends the fall).
# G10 stall_fault (8 ms) and slip_fault (20 ms), EXPLAINED (brief M3-X2i: the firmware's mismatch rebuilt from both
#   archives). The firmware (FW-020) compares the x encoder's counts since the homing latch, floor(x / 16.67 um)
#   less its value at the latch, with the steps issued since, x 60000 / 80000, faults over 120 counts, begins its
#   stop at 0.05 m/s^2 and latches the fault when the profile reaches zero, so the event is detection + v / a.
#   Rebuilt mismatch, native against the bridge (counts; continuous, from the recorded x joint and step pins):
#   (1) homing settle: the counts are zeroed at the latch; afterwards the native carriage settles 71.0 um (stall
#       run) and 72.7 um (slip run, the re-home) back as the belt lets go of the homing drive's tension (5.12 N,
#       0.051 mm of stretch at the latch), the bridge's rigid belt 22.9 and 26.0 um: 2.89 and 2.80 counts ahead.
#   (2) drive stretch (M3-X2d's term): the x belt under the drive, 0.0499 mm at 8.499 and 30.499 s: 2.99 counts.
#       Offset at the fault's start: 5.93 counts (stall) and 5.85 (slip), (1) + (2) and 0.05 counts of rotor lag.
#   (3) coast: the stalled carriage coasts 253.8 um on native, 253.3 um on the bridge (-0.02 counts); under the
#       slip the carriage speed differs slightly (+0.26 counts by the crossing). Offset at the crossing 5.91 and 6.11.
#   Crossing shift = offset / the mismatch's growth rate (the commanded less the carriage speed: stall 28.9 mm/s =
#   1.73 counts/ms; slip 0.3 x 35.3 mm/s = 0.64 counts/ms: 5.91 / 1.73 = 3.4 ms and 6.11 / 0.64 = 9.5 ms predicted):
#   measured continuous crossings 8.5777 against 8.5812 s (3.44 ms) and 30.7040 against 30.7139 s (9.93 ms); the
#   integer counts cross at ticks 8578, 8582 and 30705, 30715.
#   (4) stop phase: v / a with v still rising at a, so a detection earlier by d stops d sooner: 575 against 579 ms
#       and 702 against 712 ms observed, the crossing shift again. Predicted lead 2 x 3.44 = 6.9 ms and 2 x 9.93 =
#       19.9 ms against 8 and 20 ms (86 and 99 percent; the remainder the integer count and tick quantisation, the
#       observed shifts 2 x 4 and 2 x 10 ticks). M3-X2d's recomputed 2 and 5 ms used a rest reference that drops
#       term (1). A model difference (compliant against rigid x belt at the homing latch), not an engine fault.
# G10 stall_rehomed (14 ms) and slip_carriage_stopped (0.73 mm), consequences of the two above: the re-home starts
#   from the earlier stall stop (8 ms) plus a homing on native 2 ms shorter (G2 ready); the slip stop starts
#   earlier, and the commanded stop distance moves 2 v per second of detection: 2 x 0.036 m/s x 10 ms = 0.72 mm.
MISSES_OF_RECORD: dict[str, str] = {
    "G10 slip_carriage_stopped: 0.0211615 m on the bridge, 0.0204295 m on native, tolerance 0.0002 m (x axis)": (
        "the slip stop starts 10 ms earlier (slip_fault): 2 v x 10 ms = 0.72 mm; consequence of slip_fault"
    ),
    "G10 slip_fault: event at 31.427 s on the bridge, 31.407 s on native, tolerance 0.003 s (three ticks)": (
        "explained: homing settle 2.80 + drive stretch 2.99 + slip coast 0.26 counts, crossing 9.93 ms, event 19.9 ms"
    ),
    "G10 stall_fault: event at 9.161 s on the bridge, 9.153 s on native, tolerance 0.003 s (three ticks)": (
        "explained: homing settle 2.89 + drive stretch 2.99 - coast 0.02 counts, crossing 3.44 ms, event 6.9 ms"
    ),
    "G10 stall_rehomed: event at 12.044 s on the bridge, 12.03 s on native, tolerance 0.003 s (three ticks)": (
        "the re-home follows the stall stop 8 ms earlier and a homing 2 ms shorter; consequence of stall_fault"
    ),
}


@pytest.mark.req("PHY-024")
@pytest.mark.xfail(
    strict=True,
    reason=(
        "not met (ruling 83, owner decision at the gate): the four misses of record, MISSES_OF_RECORD, all G10: "
        "stall_fault and slip_fault explained (brief M3-X2i), stall_rehomed and slip_carriage_stopped their "
        "consequences"
    ),
)
def test_phy_024() -> None:
    """The MuJoCo retirement check shall pass only when the native adapter matches the reference-design baselines within
    the differential tolerance.

    Each fixed bridge baseline (PHY-025) against the fixed native baseline at retirement
    (`bridge_baselines/native_at_retirement.yaml`, ruling 99 (c)) at the tolerance of the quantity each
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
    carried, fitting = lift_masses()
    print(f"lift belt driven mass at build time {carried!r} kg, rope end fitting {fitting!r} kg")
    assert carried > 0.0
    assert fitting > 0.0
    expected = {"G3": 0.2e-3 + carried * 9.81 / 1e5, "G7": 0.2e-3 + (2.0 + carried) * 9.81 / 1e5}
    expected["G6"] = 0.2e-3 + (5.5 + carried) * 9.81 / 1e5
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


# ===== E9 approximation line on G7 (PHY-031, ruling 94, brief M3-X2i) =========================================


@pytest.mark.req("PHY-031")
@pytest.mark.long_run
def test_g7_on_native_states_the_lift_belt_efficiency_damping(case_runs: CaseRuns) -> None:
    """G7 on native releases the lift rotor from rest (the drive cleared at 12.022 s, and again around the home): the
    frozen efficiency damping (engine transmission.hpp (iv)) then removes far more than the loss it stands for, and
    the run states it, one line per transmission at most, with the run's largest ratio above the factor 2 and its
    time. The ratio's value against its derivation is the engine adapter's test; the hold itself is unchanged by the
    line (hook_held 1.821 mm, the native baseline)."""
    result, driver = case_runs.get(ROOT / "test_cases" / "gantry" / "G7.yaml")
    assert result.report is not None
    assert not result.report.errored, result.report.error
    lines = [line for line in driver.approximations if "efficiency loss applied as shaft damping" in line]
    for name in ("x_belt", "y_belt", "lift_belt"):
        assert len([line for line in lines if line.startswith(f"transmission '{name}':")]) <= 1
    (lift,) = [line for line in lines if line.startswith("transmission 'lift_belt':")]
    pattern = r"removed up to ([0-9.e+-]+) times the loss it stands for in one step, at t = ([0-9.e+-]+) s"
    found = re.search(pattern, lift)
    assert found is not None
    print(f"G7 lift_belt efficiency damping: ratio {found.group(1)} at {found.group(2)} s")
    assert float(found.group(1)) > 2.0
