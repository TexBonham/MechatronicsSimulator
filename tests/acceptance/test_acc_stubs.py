"""Strict-xfail stubs for the ACC spec (acceptance tier)."""

import itertools
import math
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mechsim.integrity.beam import node_deflection
from mechsim.integrity.inputs import node_chains, section_properties
from mechsim.integrity.plugin import BodyPoses, MemberInputs
from mechsim.testing.core_driver import CoreDriver, core_services
from mechsim.testing.runner import RunResult, Services, prepare, run_case
from mechsim.testing.schema import ROOT

PENDULUM_CASE = ROOT / "test_cases" / "pendulum" / "P1_period.yaml"
GANTRY_CASES = ROOT / "test_cases" / "gantry"
_VIEWS: dict[str, CoreDriver] = {}
GANTRY_RUNS = ("G1", "G2", "G3", "G4", "G5", "G6", "G7", "G8", "G9", "G10", "G11", "G13", "G14")


# ===== Fixtures and helpers =================================================
@pytest.fixture(scope="module")
def gantry() -> dict[str, RunResult]:
    """G1 to G14 (G12 not yet shipped) once each through the real stack (gantry design, builder, MuJoCo bridge,
    kernel, signal layer, native gantry firmware, recorder, scoring); placeholder defaults for every open parameter
    (DES-044). A case whose run errors fails its own tests (`_passes`), not every case's."""
    results: dict[str, RunResult] = {}
    services = core_services()
    for case in GANTRY_RUNS:
        driver = CoreDriver()
        _VIEWS[case] = driver
        results[case] = run_case(
            GANTRY_CASES / f"{case}.yaml", Services(lambda d=driver: d, services.builder), None, None
        )
        assert results[case].report is not None, results[case].score
    return results


def _criterion(result: RunResult, name: str) -> Any:
    for criterion in result.score.criteria:
        if criterion.name == name:
            return criterion
    raise KeyError(name)


def _status_after(result: RunResult, start: float) -> tuple[str, str]:
    """The first status message at or after `start`, its fields named by the design's code tables (DES-045)."""
    build = core_services().builder(prepare(GANTRY_CASES / f"{result.score.case_id}.yaml").design)
    firmware = build.design.firmware
    assert firmware is not None
    rate = _VIEWS[result.score.case_id].recording().recording.base_rate
    for _, event in _VIEWS[result.score.case_id].recording().recording.events:
        # Position lines (FW-029) share the kind; only the status message carries state and fault.
        if not event.detail.startswith("ready / fault "):
            continue
        if event.kind == "operator_status" and float(event.time / rate) >= start:
            fields = dict(item.split("=") for item in event.detail.removeprefix("ready / fault ").split())
            state = firmware.state_name(int(fields["state"]))
            fault = firmware.fault_name(int(fields["fault"]))
            assert state is not None
            assert fault is not None
            return state, fault
    raise AssertionError(f"no status message after {start} s")


def result_series(result: RunResult, reference: str) -> Any:
    return _VIEWS[result.score.case_id].recording().series(reference)


# ===== ACC-001 to ACC-008 ===================================================
@pytest.mark.req("ACC-001")
def test_acc_001(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G1: when target (40, 25) is sent at power on before homing, no actuator
    shall move.

    Criterion (M1-X6 ruling 2, acc.md row): no step pulse on any axis for the whole run (every recorded step count
    zero) and every axis within its power-on hold band, placeholder 1 mm owed by the gantry team: the stepper hold sags
    and rings by micrometres, which is not motion.
    """
    result = gantry["G1"]
    for axis, driver in (("x", "x_driver"), ("y", "y_driver"), ("lift", "lift_driver")):
        assert _criterion(result, f"no_step_pulses_{axis}").status == "pass"
        assert len(result_series(result, f"signal:{driver}.step").values) > 0
    band = _criterion(result, "hold_band")
    print(f"G1 hold band: measured {band.measured} m (placeholder band 1 mm)")
    assert band.status == "pass"
    assert band.measured < 0.001


@pytest.mark.req("ACC-002")
def test_acc_002(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G1: the refusal shall be reported with status "not homed".

    Tolerance not set (owed by gantry team, ACC-002 row): the behavior is asserted. The status message after the
    target carries state not_homed and fault not_homed by the design's code tables (DES-045).
    """
    assert _criterion(gantry["G1"], "refusal_reported").status == "pass"
    assert _status_after(gantry["G1"], 0.0) == ("not_homed", "not_homed")


@pytest.mark.req("ACC-003")
def test_acc_003(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G2: after homing, each axis's origin shall lie within the tolerance of
    its true origin.

    Tolerance 1 cm (ACC-003 row). The true origin is the min switch at 0 m on each axis joint (DES-043); the homed axis
    stops on its switch, so the joint coordinate when homing has finished is the origin error.
    """
    result = gantry["G2"]
    for name in ("origin_x", "origin_y"):
        criterion = _criterion(result, name)
        print(f"G2 {name}: target 0 m, measured {criterion.measured} m ({criterion.limit})")
        assert criterion.status == "pass", criterion


@pytest.mark.req("ACC-004")
def test_acc_004(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G2: after homing, the status shall be "ready".

    Tolerance not set (owed by gantry team, ACC-004 row): the behavior is asserted. The controller enters ready and
    its status message reports state ready with fault none (DES-045).
    """
    assert _criterion(gantry["G2"], "ready").status == "pass"
    assert _criterion(gantry["G2"], "status_reported").status == "pass"
    assert _status_after(gantry["G2"], 0.0) == ("ready", "none")
    assert gantry["G2"].score.status == "pass"


@pytest.mark.req("ACC-005")
def test_acc_005(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G3: after target (40, 25) is sent following homing, the hook's final
    position shall lie within the tolerance of the target.

    Tolerance 1 cm (ACC-005 row), in the homed origin frame (G3.yaml); the hook's displacement from its reference pose
    is that frame because homing puts the origin at each joint's zero.
    """
    result = gantry["G3"]
    for name, target in (("final_x", 0.40), ("final_y", 0.25)):
        criterion = _criterion(result, name)
        print(f"G3 {name}: target {target} m, measured {criterion.measured} m ({criterion.limit})")
        assert criterion.status == "pass", criterion


@pytest.mark.req("ACC-006")
def test_acc_006(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G3: no travel limit shall be exceeded during the move.

    Tolerance not set (owed by gantry team, ACC-006 row): the behavior is asserted, every joint inside its design
    limits, ends included, from the target on.
    """
    result = gantry["G3"]
    for criterion in result.score.criteria:
        if criterion.name.endswith(("_upper", "_lower")):
            print(f"G3 {criterion.name}: measured {criterion.measured} m ({criterion.limit})")
            assert criterion.status == "pass", criterion
    assert result.score.status == "pass"


@pytest.mark.req("ACC-007")
def test_acc_007(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G4: for a target outside the workspace, no actuator shall move.

    Criterion as ACC-001 from the target on (after homing): no step pulse on any axis and every axis within the
    placeholder 1 mm hold band of its position when the target arrives.
    """
    result = gantry["G4"]
    for axis, driver in (("x", "x_driver"), ("y", "y_driver"), ("lift", "lift_driver")):
        assert _criterion(result, f"no_step_pulses_{axis}").status == "pass"
        assert len(result_series(result, f"signal:{driver}.step").values) > 0
    band = _criterion(result, "hold_band")
    print(f"G4 hold band: measured {band.measured} m (placeholder band 1 mm)")
    assert band.status == "pass"
    assert band.measured < 0.001


@pytest.mark.req("ACC-008")
def test_acc_008(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G4: the target shall be rejected with a reported fault, not clipped.

    Tolerance not set (owed by gantry team, ACC-008 row): the behavior is asserted. Rejected: the status message after
    the target reports fault outside_workspace (DES-045); not clipped: the carriage never moves toward the workspace
    edge, its x joint stays within 1 cm (the position accuracy) of the homed origin from the target on.
    """
    result = gantry["G4"]
    assert _criterion(result, "rejection_reported").status == "pass"
    commands = prepare(GANTRY_CASES / "G4.yaml").case.commands
    homing = float(next(command.at for command in commands if command.message == "target"))  # type: ignore[arg-type]
    assert _status_after(result, homing) == ("ready", "outside_workspace")
    series = result_series(result, "joint:x_axis.position")
    after = [value for time, value in zip(series.times, series.values, strict=True) if time >= homing]
    print(f"G4 x after the target: max {max(after)} m, min {min(after)} m")
    assert max(abs(value) for value in after) < 0.01


# ===== Refusal helpers ======================================================
def _passes(result: RunResult, *names: str) -> None:
    report = result.report
    assert report is not None, result.score
    assert not report.errored, f"{result.score.case_id}: {report.error}"
    for name in names:
        criterion = _criterion(result, name)
        print(f"{result.score.case_id} {name}: measured {criterion.measured} ({criterion.limit})")
        assert criterion.status == "pass", criterion


def _refusals(result: RunResult, start: float, end: float) -> int:
    """Firmware `refused` events in [start, end); their fault is read from the status message that follows."""
    events = _VIEWS[result.score.case_id].recording().events()
    return len([event for event in events if event.kind == "refused" and start <= event.time < end])


def _refused_with(result: RunResult, start: float, end: float, fault: str) -> None:
    assert _refusals(result, start, end) >= 1
    assert _status_after(result, start)[1] == fault


# ===== ACC-009 to ACC-024 ===================================================
@pytest.mark.req("ACC-009")
@pytest.mark.req("FW-018")
def test_acc_009(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G5: with a load over the limit attached before a move, a lift command
    shall be ignored.

    Tolerance not set (owed by gantry team): the behavior is asserted. With 5.5 kg on the hook (over the 5.216 kg
    limit, rated 10 lb + 15 percent, gantry team 2026-11-11) `lift 0.4` at 18 s is refused with fault overload, and the
    lift stays within the 1 cm hold band (the position accuracy) until the next command at 20 s.
    """
    result = gantry["G5"]
    _passes(result, "lift_up_refused", "lift_up_ignored")
    _refused_with(result, 18.0, 20.0, "overload")


@pytest.mark.req("ACC-010")
@pytest.mark.req("FW-018")
def test_acc_010(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G5: with a load over the limit attached, a lower command shall work.

    `lift 0.1` at 20 s from 0.3 m with 5.5 kg: no refusal to the end at 26 s, and the lift ends within 1 cm (the
    position accuracy) of 0.1 m.
    """
    result = gantry["G5"]
    _passes(result, "lowered")
    assert _refusals(result, 20.0, 26.0) == 0


@pytest.mark.req("ACC-011")
@pytest.mark.req("FW-019")
@pytest.mark.req("TST-020")
def test_acc_011(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G6: when the load becomes overweight mid-lift, the lift shall stop.

    The payload becomes 5.5 kg 2 s into the lift (TST-020 routes the change to physics). Bound from the gantry team's
    limits of 2026-11-11 and ruling 35 (G6 header): 0.1 m risen at the change, at most 1.12 cm more during detection
    (12.5 ms cell sample, 97.3 ms for the 2 Hz amplifier to cross 5.216 kg, 2 ms filter, at 0.1 m/s), v^2 / 2a = 0.1 m
    to stop at 0.05 m/s^2: the lift stays under 21.1 cm, short of the 45 cm commanded.
    """
    _passes(gantry["G6"], "lift_stopped")


@pytest.mark.req("ACC-012")
@pytest.mark.req("FW-019")
def test_acc_012(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G6: after an overweight stop, the load shall be able to be set down.

    `lift 0.05` after the stop, still at 5.5 kg: the lift ends within 1 cm of 5 cm.
    """
    _passes(gantry["G6"], "set_down")


@pytest.mark.req("ACC-013")
@pytest.mark.req("FW-024")
def test_acc_013(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G7: when the E-stop is pressed mid-move with a load, motor power shall be
    cut within the relay delay.

    Pressed at 12 s with the hook lifted to 30 cm; the relay's 20 ms cutoff_delay (estop_relay_24v_generic) opens the
    contacts in the layer step at 12.020 s, whose row is stamped 12.021 s (KER-009): `rail:motor_power` (ruling 27)
    and the relay status read 0 from 12.021 s to the release, and the rail's first off sample is exactly that row.
    """
    result = gantry["G7"]
    _passes(result, "power_cut", "relay_open")
    series = result_series(result, "signal:motor_power.on")
    off = [time for time, value in zip(series.times, series.values, strict=True) if value == 0.0]
    print(f"G7 motor rail first off at {off[0]} s (press 12 s + 20 ms, row stamped 12.021 s)")
    assert off[0] == pytest.approx(12.021, abs=1e-9)


@pytest.mark.req("ACC-014")
def test_acc_014(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G7: after the E-stop, the hook shall not drop.

    The hook is lifted to 30 cm first, so the brake and not the lift's lower joint limit holds it (X review): the lift
    stays within the 1 cm hold band (position accuracy) of its position at the press at 12 s until the home at 15 s;
    the power-off brake engages when the motor rail drops (ruling 1).
    """
    _passes(gantry["G7"], "hook_held")


@pytest.mark.req("ACC-015")
@pytest.mark.req("FW-024")
def test_acc_015(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G7: after the E-stop, motion shall resume only after a reset and a new
    command.

    No step pulse on X or Y from the tick after the cutoff (12.022 s) until the `home` at 15 s, the `reset` at 14 s
    included (ruling 7: reset returns to not_homed), and the hook reaches (40, 25) within 1 cm after the new target.
    """
    _passes(gantry["G7"], "no_steps_x_until_command", "no_steps_y_until_command", "resumed_x", "resumed_y")


@pytest.mark.req("ACC-016")
@pytest.mark.req("FW-025")
@pytest.mark.req("TST-018")
def test_acc_016(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G8: when the controller freezes mid-move, the motors shall stop within
    the heartbeat timeout.

    Freeze at 9 s (TST-018 routes it to the controller runtime): the heartbeat stops, the watchdog trips after its
    50 ms timeout and opens after its 10 ms cutoff delay; with one 10 ms heartbeat period of slack the motor rail is
    off and no driver steps from 9.07 s to the end of the freeze.
    """
    _passes(gantry["G8"], "motor_power_off", "x_no_steps", "y_no_steps", "lift_no_steps")


@pytest.mark.req("ACC-017")
@pytest.mark.req("FW-017")
@pytest.mark.req("TST-017")
def test_acc_017(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G9: after power is lost and restored, no motion shall occur until the
    gantry is homed.

    Both rails off from 12 s to 12.5 s (TST-017), the hook lifted to 30 cm before: the logic rail under the brownout
    threshold resets the firmware to not_homed; the target at 13.5 s is refused with fault not_homed, no driver steps
    from the restore to the home at 14 s, and homing starts after it.
    """
    result = gantry["G9"]
    _passes(
        result,
        "not_homed_after_restore",
        "target_refused",
        "no_steps_x_until_home",
        "no_steps_y_until_home",
        "no_steps_lift_until_home",
        "homing_after_home",
    )
    _refused_with(result, 13.5, 14.0, "not_homed")


@pytest.mark.req("ACC-018")
def test_acc_018(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G9: during the power loss and restore, the load shall not drop.

    The 2 kg load, lifted to 30 cm so the brake and not the lift's lower joint limit holds it (X review), stays within
    the 1 cm hold band from the loss at 12 s to the home at 14 s: the brake coil is on the motor rail (power-off
    brake, ruling 1).
    """
    _passes(gantry["G9"], "load_held")


def _travel_after(result: RunResult, start: float, end: float) -> float:
    """Largest |x(t) - x(start)| of the X carriage over [start, end]."""
    series = result_series(result, "joint:x_axis.position")
    times = np.asarray(series.times, dtype=float)
    values = np.asarray(series.values, dtype=float)
    inside = (times >= start) & (times <= end)
    assert inside.any()
    origin = values[np.argmax(times >= start)]
    return float(np.max(np.abs(values[inside] - origin)))


@pytest.mark.req("ACC-019")
@pytest.mark.req("FW-020")
@pytest.mark.req("TST-015")
@pytest.mark.req("TST-016")
def test_acc_019(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G10: on a motor stall or belt slip during a move, motion shall stop.

    Tolerance not set (owed by gantry team); the bound is the gantry team's limits of 2026-11-11 (G10 header): the 2 mm
    mismatch threshold is crossed within the 0.5 s fault window, by then the speed is at most 0.05 m/s^2 x 1 s =
    0.05 m/s and the stop takes v / a = 1 s, so no driver step from fault start + 1.5 s, and the carriage travels at
    most 0.05 m/s x 0.5 s + v^2 / (2 a) = 0.025 m + 0.025 m = 0.05 m past its position at the fault start. The
    frame-side encoders (ruling 32) see both the stall and the slip.
    """
    result = gantry["G10"]
    _passes(result, "stall_stopped_x", "stall_stopped_y", "slip_stopped_x", "slip_stopped_y", "slip_carriage_stopped")
    bound = 0.05 * 0.5 + 0.05**2 / (2.0 * 0.05)
    for start in (8.5, 30.5):
        travel = _travel_after(result, start, start + 2.0)
        print(f"G10 X travel from {start} s over 2 s: {travel} m (bound {bound} m)")
        assert travel <= bound


@pytest.mark.req("ACC-020")
@pytest.mark.req("FW-020")
@pytest.mark.req("TST-016")
def test_acc_020(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G10: on a motor stall or belt slip during a move, an encoder-mismatch
    fault shall be reported.

    Stall on the X motor from 8.5 s (TST-016) and slip on the X belt from 30.5 s (TST-015): each enters the fault
    state within the 0.5 s window plus the 1 s stop, and the refusals of the targets at 10.5 s and 32.5 s carry fault
    encoder_mismatch.
    """
    result = gantry["G10"]
    _passes(result, "stall_fault", "slip_fault")
    _refused_with(result, 10.5, 11.0, "encoder_mismatch")
    _refused_with(result, 32.5, 33.0, "encoder_mismatch")


@pytest.mark.req("ACC-021")
@pytest.mark.req("FW-020")
def test_acc_021(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G10: after a stall or slip fault, re-homing shall be required before
    further motion.

    After each fault the target is refused and no driver steps until `home`; homing then runs (ready again after the
    stall's re-home).
    """
    _passes(
        gantry["G10"],
        "stall_stopped_x",
        "stall_stopped_y",
        "stall_target_refused",
        "stall_rehomed",
        "slip_stopped_x",
        "slip_stopped_y",
        "slip_target_refused",
        "slip_rehoming",
    )


@pytest.mark.req("ACC-022")
@pytest.mark.req("FW-021")
def test_acc_022(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G11: when a limit switch trips mid-workspace, motion shall stop.

    x_max_switch forced high at 8.5 s for 0.2 s, 6.25 mm into the move: no driver steps from 8.5 + 0.2 + 0.7 s (stop
    at 0.05 m/s^2 from at most 0.035 m/s, G11 header) until the home.
    """
    _passes(gantry["G11"], "stopped_x", "stopped_y")


@pytest.mark.req("ACC-023")
@pytest.mark.req("FW-021")
def test_acc_023(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G11: when a limit switch trips mid-workspace, a fault shall be reported.

    The firmware enters the fault state and the next refusal carries fault limit_trip.
    """
    result = gantry["G11"]
    _passes(result, "limit_fault")
    _refused_with(result, 10.0, 10.5, "limit_trip")


@pytest.mark.req("ACC-024")
@pytest.mark.req("FW-021")
def test_acc_024(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G11: after a mid-workspace limit trip, re-homing shall be required before
    further motion.

    The target at 10 s is refused; homing starts after `home` at 10.5 s.
    """
    _passes(gantry["G11"], "target_refused", "rehoming")


# ===== ACC-025 to ACC-028 ===================================================
# G12 and G15 on the native adapter (rope payload, 100 microseconds): each runs once per module, archived, so the
# load case export and the archive digest are checked; the determinism test runs each a second time (invariant 4).
SWING_A, SWING_G, SWING_L, SWING_V = 0.05, 9.81, 0.3, 0.1


@pytest.fixture(scope="module")
def native_gantry(tmp_path_factory: pytest.TempPathFactory) -> dict[str, tuple[RunResult, CoreDriver]]:
    out = tmp_path_factory.mktemp("native_gantry")
    return {case: _native_run(case, out) for case in ("G12", "G15")}


def _native_run(case: str, out: Path) -> tuple[RunResult, CoreDriver]:
    services = core_services()
    driver = CoreDriver()
    result = run_case(GANTRY_CASES / f"{case}.yaml", replace(services, driver=lambda: driver), None, out)
    assert result.report is not None, result.score
    assert not result.report.errored, result.report.error
    return result, driver


def _swing(driver: CoreDriver) -> tuple[Any, Any]:
    """Times and the swing angle of the payload about the hook: the line from the hook's origin (the hook_swing pivot)
    to the payload's centre against the vertical, from the recorded body poses (indexed by name)."""
    recording = driver.recording().recording
    positions = recording.series["physics/bodies/position"]
    names = list(positions.names)
    values = np.asarray(positions.values, dtype=np.float64)
    line = values[:, names.index("payload"), :] - values[:, names.index("hook"), :]
    angle = np.arctan2(np.hypot(line[:, 0], line[:, 1]), -line[:, 2])
    times = np.asarray(driver.recording().series("pose:hook.x").times, dtype=np.float64)
    return times, angle


def _peak(times: Any, values: Any, start: float, end: float) -> float:
    inside = (times >= start) & (times <= end)
    return float(np.max(np.abs(values[inside])))


@pytest.mark.req("ACC-025")
def test_acc_025(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """Reference design (gantry), G12: after a fast move and after an E-stop, the peak load swing angle
    shall be within its limit.

    Limit 0.1 rad (swing_limit_angle, placeholder). Oracles (G12 header; L = 0.3 m is shorter than the true pendulum,
    so each bound is an upper one): from rest, accelerating at a, theta <= 2 a / g over the first ramp (8 s to 10 s);
    each of the six step changes of acceleration in the trapezoid with its slow zone adds a free swing of at most a / g
    about an offset of at most a / g, so theta <= 7 a / g over the move (8 s to 20 s); the E-stop from v in a time short
    against the period adds at most v / sqrt(g L) to the free swing left by the second move's two steps (2 a / g), so
    theta <= 2 a / g + v / sqrt(g L) after the press (22.5 s to 30 s). The angle is measured from the body poses; the
    recorded body count is checked first (neutral bodies with the shackle and the payload, then the member nodes).
    """
    result, driver = native_gantry["G12"]
    names = list(driver.recording().recording.series["physics/bodies/position"].names)
    assert len(names) == 11 + 16 + 9
    assert names[9:11] == ["payload_shackle", "payload"]
    assert names[11] == "rail_x_beam.node0"
    times, angle = _swing(driver)
    ramp = 2.0 * SWING_A / SWING_G
    move = 7.0 * SWING_A / SWING_G
    stop = 2.0 * SWING_A / SWING_G + SWING_V / math.sqrt(SWING_G * SWING_L)
    measured = (_peak(times, angle, 8.0, 10.0), _peak(times, angle, 8.0, 20.0), _peak(times, angle, 22.5, 30.0))
    print(f"G12 swing peaks: first ramp {measured[0]!r} rad (bound 2 a / g = {ramp!r})")
    print(f"G12 swing peaks: move {measured[1]!r} rad (bound 7 a / g = {move!r})")
    print(f"G12 swing peaks: E-stop {measured[2]!r} rad (bound 2 a / g + v / sqrt(g L) = {stop!r})")
    assert measured[0] <= ramp
    assert measured[1] <= move
    assert measured[2] <= stop
    assert max(measured) <= 0.1
    for name in ("move_swing_x", "move_swing_y", "estop_swing_x"):
        criterion = _criterion(result, name)
        print(f"G12 {name}: hook tilt {criterion.measured} rad ({criterion.limit})")
        assert criterion.status == "pass", criterion


@pytest.mark.req("ACC-026")
@pytest.mark.xfail(
    strict=True,
    reason="finding M3-X1a: the gantry model has no swing damping (hook_swing 0, rope axial only); settle owed",
)
def test_acc_026(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """Reference design (gantry), G12: after a fast move and after an E-stop, the swing settle time shall
    be within its limit.

    Limit 5 s (swing_settle_time) into the 0.01 rad band (swing_settle_band), both placeholders. The model's physical
    swing loss is zero to first order (G12 header), so the swing decays only by backward Euler's
    (1 + x^2)^(-pi / atan x) per period, x = omega h = sqrt(g / L) 1e-4 s: from the E-stop's swing A0 the expected
    settle time is ln(band / A0) / ln(kept) periods of 2 pi / omega, printed beside the measured swing at the run's
    end. Strict expected failure until the gantry team gives the swing damping (the limit and the physics are not
    tuned to pass).
    """
    result, driver = native_gantry["G12"]
    omega = math.sqrt(SWING_G / SWING_L)
    x = omega * 1e-4
    kept = (1.0 + x * x) ** (-math.pi / math.atan(x))
    times, angle = _swing(driver)
    start = _peak(times, angle, 22.5, 30.0)
    expected = math.log(0.01 / start) / math.log(kept) * 2.0 * math.pi / omega
    print(f"G12 settle: scheme keeps {kept!r} per period; from {start!r} rad expected {expected!r} s (limit 5 s)")
    print(f"G12 settle: swing over the last period {_peak(times, angle, 30.0 - 2.0 * math.pi / omega, 30.0)!r} rad")
    for name in ("move_settle_x", "move_settle_y", "estop_settle_x"):
        criterion = _criterion(result, name)
        print(f"G12 {name}: {criterion.status}, measured {criterion.measured} s ({criterion.limit})")
    for name in ("move_settle_x", "move_settle_y", "estop_settle_x"):
        assert _criterion(result, name).status == "pass"


@pytest.mark.req("ACC-027")
@pytest.mark.req("FW-027")
@pytest.mark.req("SCO-022")
def test_acc_027(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G13: a lift command while the gantry is moving shall be handled by the
    defined policy (queue or reject) in every run.

    Policy: the firmware placeholder, reject (gantry_config.h, owed by the gantry team). `lift 0.3` at 8.5 s, inside
    the move to (40, 25) (0.4 m of X at no more than 0.1 m/s lasts at least 4 s): refused with fault busy
    (`fault:busy`, ruling 30) and a `refused` event, no lift step pulse and the lift within the 1 cm band to 10 s; the
    move still arrives within 1 cm. The status after the refusal names busy.
    """
    result = gantry["G13"]
    _passes(result, "lift_refused_busy", "lift_refused_event", "no_lift_steps", "lift_held", "arrived_x", "arrived_y")
    _refused_with(result, 8.5, 9.0, "busy")


@pytest.mark.req("ACC-028")
@pytest.mark.req("FW-026")
@pytest.mark.req("FW-028")
@pytest.mark.req("SCO-021")
@pytest.mark.req("SCO-025")
def test_acc_028(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G14: under rapid-fire and conflicting commands, no two actuator commands
    shall overlap.

    Fifty commands from 8 s to 12.9 s: `no_overlap` on every driver's `motion_start` and `motion_end` events (each
    axis has profiles to check: homing starts all three at t = 0), the first move's Y profile between 0.5 s and 2.3 s
    after its target (cruise and slow-zone bounds, G14.yaml header), the malformed target refused with fault
    malformed. FW-026: every target sent while moving is refused (reject placeholder), so the first move's Y profile
    is the only one between its target and its end.
    """
    result = gantry["G14"]
    _passes(
        result,
        "no_overlap_x",
        "no_overlap_y",
        "no_overlap_lift",
        "first_move_not_too_fast",
        "first_move_not_too_slow",
        "malformed_refused",
    )
    events = _VIEWS["G14"].recording().events()
    for driver in ("x_driver", "y_driver", "lift_driver"):
        starts = [event for event in events if event.kind == "motion_start" and event.detail == driver]
        assert len(starts) >= 1, driver
    first_start = min(event.time for event in events if event.kind == "motion_start" and event.time >= 8.0)
    first_end = min(event.time for event in events if event.kind == "motion_end" and event.time >= 8.0)
    starts_inside = [event for event in events if event.kind == "motion_start" and first_start < event.time < first_end]
    assert starts_inside == []
    assert _refusals(result, 8.0, 13.0) >= 1


# ===== ACC-029 to ACC-032 ===================================================
# G15 oracles (G15 header): design-load weight P on mid-span of each rail at mid-travel, calibrated mid-span stiffness
# 100 N / 1 mm = 1e5 N/m on each rail (so P L^3 / (192 E I) and P L^3 / (48 E I) are both P / 1e5 N/m), the stated
# section c = 0.02 m, I = 2e-8 m^4; a suddenly applied load at most doubles the static value.
G15_LOAD = (5.2164 + 0.035 * 0.3) * 9.81
G15_STIFFNESS = 100.0 / 1e-3
G15_MOMENT = {"rail_x": G15_LOAD * 0.5 / 8.0, "rail_y": G15_LOAD * 0.5 / 4.0}


@pytest.mark.req("ACC-029")
@pytest.mark.xfail(
    strict=True,
    reason="finding M3-X1a: tier 1 fastener crossings at the E-stop (3.0 kN on frame_mount against 1.33 kN) and "
    "the carriage loads bypass the lumped rails (engine gap); see the report",
)
def test_acc_029(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """Reference design (gantry), G15: under maximum acceleration with rated load and an E-stop with a
    swinging load, every member shall stay above the minimum safety factor.

    Minimum 1.5 (design, gantry team 2026-11-11). Each member's measured peak stress is printed beside the static
    oracle M c / I (M = P L / 8 clamped-clamped, P L / 4 pinned) and asserted within twice it (suddenly applied load)
    and at least the static share it must carry; no tier 1 crossing; the load case is exported beside the archive.
    """
    result, _ = native_gantry["G15"]
    assert result.metadata is not None
    assert result.metadata.integrity is not None
    for member in ("rail_x", "rail_y"):
        row = result.metadata.integrity.member(member)
        static = G15_MOMENT[member] * 0.02 / 2e-8
        print(
            f"G15 {member}: safety factor {row.safety_factor!r} (minimum 1.5), peak stress {row.peak_stress!r} Pa at "
            f"{row.peak_stress_time!r} s (static oracle M c / I {static!r} Pa)"
        )
        assert _criterion(result, f"safety_factor_{member}").status == "pass"
        assert static <= row.peak_stress <= 2.0 * static
    assert result.archive is not None
    for suffix in (".load_case.json", ".load_case.csv", ".integrity.json"):
        assert result.archive.with_suffix(suffix).is_file(), suffix
    print(f"G15 crossings: {result.score.crossings}")
    assert result.score.crossings == ()
    assert result.score.status == "pass"


@pytest.mark.req("ACC-030")
@pytest.mark.xfail(
    strict=True,
    reason="finding M3-X1a: post-run node-pose deflection indexes the build's bodies, not the run's (payload "
    "bodies shift the node series: 1.38 m on rail_x); the carriage loads bypass the lumped rails",
)
def test_acc_030(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """Reference design (gantry), G15: under the same worst cases, member deflection shall stay under the
    target.

    Target 1 cm (ACC-030 row), the dynamic node-pose peak (the conservative reading). Each rail's measured peak is
    printed beside its static oracle P / k (k the calibrated mid-span stiffness, 1e5 N/m on both rails) and asserted
    at least the static and at most twice it (suddenly applied load).
    """
    result, _ = native_gantry["G15"]
    assert result.metadata is not None
    assert result.metadata.integrity is not None
    static = G15_LOAD / G15_STIFFNESS
    for member in ("rail_x", "rail_y"):
        row = result.metadata.integrity.member(member)
        print(f"G15 {member}: deflection {row.deflection!r} m at {row.deflection_time!r} s (static oracle {static!r})")
        assert _criterion(result, f"deflection_{member}").status == "pass"
        assert static <= row.deflection <= 2.0 * static
        assert row.deflection < 0.01


@pytest.mark.req("INV-014")
@pytest.mark.parametrize("case", ["G12", "G15"])
def test_native_gantry_cases_repeat_bit_identically(
    case: str, native_gantry: dict[str, tuple[RunResult, CoreDriver]], tmp_path: Path
) -> None:
    """G12 and G15 run twice with the same seed give the same archive bytes (invariant 4)."""
    first, _ = native_gantry[case]
    second, _ = _native_run(case, tmp_path)
    assert first.score.run is not None
    assert second.score.run is not None
    assert first.score.run.archive_sha256 == second.score.run.archive_sha256
    assert first.report == second.report


@pytest.mark.req("ACC-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_031() -> None:
    """Reference design (gantry), G16: a sweep of NEMA 17 against NEMA 23 with the rated load shall report
    the torque margin per motor.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-032")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_032() -> None:
    """Reference design (gantry), G16: the same sweep shall report any missed steps per motor."""
    raise NotImplementedError


# ===== ACC-033 to ACC-034 ===================================================
@pytest.mark.req("ACC-033")
def test_acc_033(tmp_path: Path) -> None:
    """Pendulum design, rigid link: the small-amplitude swing period shall match the damped small-angle period
    for the link, T0 / sqrt(1 - zeta^2) with T0 = 2 pi sqrt(I / (m g d)), within the period tolerance.

    Closed form (P1_period.yaml): I = m L^2 / 3 = 1/12 kg m^2, m = 1 kg, g = 9.81 m/s^2, d = 0.25 m give
    T0 = 1.1582031 s; zeta = 0.05 gives T_d = 1.1596536 s. Tolerance 0.1 percent (ACC-033 row, PHY-028), scored by
    the case's own criterion through the real stack: YAML case, builder, MuJoCo bridge, kernel, recorder, scoring.
    """
    result = run_case(PENDULUM_CASE, core_services(), None, tmp_path)
    assert result.report is not None
    assert not result.report.errored, result.report.error
    period = result.score.criteria[0]
    print(f"P1 period: closed form 1.1596536 s, measured {period.measured} s ({period.limit})")
    assert period.name == "period"
    assert period.status == "pass"
    assert result.score.status == "pass"
    assert result.archive is not None
    assert result.archive.is_file()


@pytest.mark.req("ACC-034")
def test_acc_034(tmp_path: Path) -> None:
    """Pendulum design, rigid link: the swing amplitude decay shall match the closed-form decay for the
    link's stated joint damping within the decay tolerance.

    The decay tolerance is not set (owed by the owner, ACC-034 row), so the numeric match is printed and not asserted;
    the behavior is asserted: released from 0.05 rad at rest the swing starts at the release angle and every positive
    peak is lower than the one before. Closed form of the envelope: theta0 exp(-zeta omega0 t), zeta = 0.05,
    omega0 = 2 pi / T0 = 5.42494 rad/s, so peak n of the damped swing sits near theta0 exp(-zeta omega0 n T_d).
    """
    driver = CoreDriver()
    services = core_services()
    result = run_case(PENDULUM_CASE, Services(lambda: driver, services.builder), None, None)
    assert result.report is not None
    assert not result.report.errored
    series = driver.recording().series("joint:pivot.position")
    values = np.asarray(series.values)
    times = np.asarray(series.times)
    assert abs(values[0] - 0.05) < 0.05 * 0.01
    inner = values[1:-1]
    peaks = np.nonzero((inner > values[:-2]) & (inner >= values[2:]) & (inner > 0.0))[0] + 1
    assert len(peaks) >= 10
    zeta, period0, period_d, theta0 = 0.05, 1.1582031, 1.1596536, 0.05
    omega0 = 2.0 * math.pi / period0
    for index in peaks[:3]:
        closed = theta0 * math.exp(-zeta * omega0 * times[index])
        print(f"peak at {times[index]:.4f} s: closed-form envelope {closed:.6e} rad, measured {values[index]:.6e} rad")
    print(f"decay per period: closed form {math.exp(-zeta * omega0 * period_d):.6f}")
    assert all(values[later] < values[earlier] for earlier, later in itertools.pairwise(peaks))


# ===== ACC-035 to ACC-045 ===================================================
@pytest.mark.req("ACC-035")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_035() -> None:
    """Acceptance criterion 1: the simulator shall build the reference design from data files or an
    imported URDF with no design-specific code.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-036")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_036() -> None:
    """Acceptance criterion 2: the simulator shall run G1 to G16 in one batch and produce the pass/fail
    report.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-037")
def test_acc_037() -> None:
    """Acceptance criterion 3, rewritten: the rope pendulum's period and swing decay shall match their
    closed-form values within tolerance.

    Closed form (ADR 0016, `test_cases/rope_pendulum/R1_period.yaml`): T_N = 2 pi sqrt(I_N / (g S)) with the loaded
    length L = L0 + g L0 (M + m_r / 2) / EA, I_N = M L^2 + m_r L^2 (1/3 + 1 / (6 N^2)), S = M L + m_r L / 2. The rope
    damps only axially, so the undamped swing keeps its amplitude: every positive peak of the bob's x over 20 periods
    lies within 2 percent of the release value L sin(0.02) (the scheme keeps (1 + x^2)^(-pi / atan x) per period at
    x = omega h = 1.57e-4: 1 percent over 20 periods, ruling 45). Tolerances: ACC-037 row (proposed, owner).
    """
    m, m_r, l0, ea, g, n = 1.0, 0.035, 1.0, 5e5, 9.81, 8
    length = l0 + g * l0 * (m + m_r / 2.0) / ea
    inertia = m * length**2 + m_r * length**2 * (1.0 / 3.0 + 1.0 / (6.0 * n * n))
    moment = m * length + m_r * length / 2.0
    period_n = 2.0 * math.pi * math.sqrt(inertia / (g * moment))
    result, driver, _ = _reference_run(ROPE_PENDULUM_CASE)
    series = driver.recording().series("pose:bob.x")
    times = np.asarray(series.times, dtype=np.float64)
    # pose:bob.x is the free joint's coordinate from its placement (L sin 0.02, -L cos 0.02): the swing is about
    # -L sin(0.02), and its amplitude is L sin(0.02).
    release = length * math.sin(0.02)
    values = np.asarray(series.values, dtype=np.float64) + release
    crossings = [
        times[i - 1] + values[i - 1] / (values[i - 1] - values[i]) * (times[i] - times[i - 1])
        for i in range(1, len(values))
        if values[i - 1] < 0.0 <= values[i]
    ]
    assert len(crossings) >= 21
    measured = (crossings[20] - crossings[0]) / 20.0
    print(f"ACC-037 period: closed form T_N {period_n!r} s, measured {measured!r} s over 20 periods")
    assert abs(measured / period_n - 1.0) <= 1e-3
    inner = values[1:-1]
    peaks = np.nonzero((inner > values[:-2]) & (inner >= values[2:]) & (inner > 0.0))[0] + 1
    peaks = peaks[times[peaks] <= crossings[20]]
    print(f"ACC-037 amplitude: release {release!r} m, peaks {values[peaks].min()!r} to {values[peaks].max()!r} m")
    assert len(peaks) >= 20
    assert np.all(np.abs(values[peaks] / release - 1.0) <= 0.02)
    assert result.score.status == "pass"


@pytest.mark.req("ACC-038")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_038() -> None:
    """Acceptance criterion 3, rewritten: the stepper's pull-out torque shall match its published curve
    within tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-039")
def test_acc_039() -> None:
    """Acceptance criterion 3, rewritten: the settle time of a step response shall match its closed-form
    value within tolerance.

    Mass on a spring-damper (`designs/rope_spring`, `test_cases/rope_spring/S1_settle.yaml`): M = 1.0175 kg (bob plus
    the rope's lower half-mass), k = EA / L0 = 500 kN/m, c = 50 N s/m, released at the rest length under gravity
    (the step), x_s = M g / k. The underdamped deviation is bounded by the envelope x_s e^(-sigma t) / sqrt(1 - zeta^2);
    it enters the 2 percent band for good at t_env = ln(50 / sqrt(1 - zeta^2)) / sigma. Continuous sigma = zeta
    omega_n; backward Euler at step h maps each mode s to 1 / (1 - h s), so the run decays at sigma_h = ln|1 - h s| / h
    (ruling 45). The last exit from the band lies at a peak under the envelope, at most half a damped period
    pi / omega_d before t_env, and the scorer reports the first sample after it (+ h). Proposed tolerance (ACC-039
    row): t_env - pi / omega_d - h <= measured <= t_env + 2 percent.
    """
    m, k, c, h = 1.0175, 5e5, 50.0, 1e-5
    omega = math.sqrt(k / m)
    zeta = c / (2.0 * m * omega)
    damped = omega * math.sqrt(1.0 - zeta * zeta)
    pole = complex(-zeta * omega, damped)
    sigma_h = math.log(abs(1.0 - h * pole)) / h
    reach = math.log(50.0 / math.sqrt(1.0 - zeta * zeta))
    continuous = reach / (zeta * omega)
    corrected = reach / sigma_h
    result, _, _ = _reference_run(ROPE_SPRING_CASE)
    settle = _criterion(result, "settle")
    measured = float(settle.measured)
    print(
        f"ACC-039 settle time: closed form {continuous!r} s, scheme-corrected {corrected!r} s, measured {measured!r} s"
    )
    assert corrected - math.pi / damped - h <= measured <= corrected * 1.02
    assert result.score.status == "pass"


@pytest.mark.req("ACC-040")
def test_acc_040() -> None:
    """Acceptance criterion 4, rewritten: a cantilever beam's static deflection shall match the closed-form
    value within tolerance.

    Through the full stack on the native adapter (`test_cases/cantilever_beam/B1_cantilever.yaml`): the bar released
    straight under its own weight w = rho A g settles under backward Euler's per-period decay (ruling 45) to the static
    shape by the last sample, whose node-pose deflection (INT-006, ruling 28: distance from the root tangent) is the
    static value. Closed form w L^4 / (8 E I); tolerance the lumped chain's 2 percent at N = 8 (rulings 31, 32; the
    chain's own value under the distributed load is 0.78 percent low).
    """
    static = BEAM_W * BEAM_LENGTH**4 / (8.0 * BEAM_EI)
    measured = _settled_deflection(CANTILEVER_CASE)
    print(f"ACC-040 cantilever static deflection: closed form {static!r} m, measured {measured!r} m")
    assert abs(measured / static - 1.0) <= 0.02


@pytest.mark.req("ACC-041")
def test_acc_041() -> None:
    """Acceptance criterion 4, rewritten: a cantilever beam's first natural frequency shall match the
    closed-form value within tolerance.

    The released bar swings about its static shape; the first-mode frequency is counted from the upward zero crossings
    of the node pose of the tip segment about its final (settled) value, over at least ten periods after the first two
    (the higher modes decay faster under backward Euler). Step 1 ms: x = omega_1 h = 0.0512, frequency ratio atan(x) / x
    = 0.99913, inside 0.1 percent (ruling 45). Closed form omega_1 = 1.8751^2 sqrt(E I / (rho A L^4)); tolerance the
    lumped chain's 2 percent at N = 8 (rulings 31, 32).
    """
    closed = 1.8751040687119611**2 * math.sqrt(BEAM_EI / (BEAM_RHO_A * BEAM_LENGTH**4)) / (2.0 * math.pi)
    measured, periods = _first_frequency(CANTILEVER_CASE, -1)
    print(f"ACC-041 cantilever first frequency: closed form {closed!r} Hz, measured {measured!r} Hz over {periods}")
    assert periods >= 10
    assert abs(measured / closed - 1.0) <= 0.02


@pytest.mark.req("ACC-042")
def test_acc_042() -> None:
    """Acceptance criterion 4, rewritten: a simply supported beam's static deflection shall match the closed-form
    value within tolerance.

    Through the full stack on the native adapter (`test_cases/simple_beam/B2_simply_supported.yaml`): the bar released
    straight under its own weight w = rho A g settles under backward Euler's per-period decay (ruling 45) to the static
    shape by the last sample, whose node-pose deflection (INT-006, ruling 28: distance from the chord through the pin
    and the far end) is the static value. Closed form 5 w L^4 / (384 E I); tolerance the lumped chain's 2 percent at N =
    8 (rulings 31, 32; the chain's own value under the distributed load is 0.625 percent low).
    """
    static = 5.0 * BEAM_W * BEAM_LENGTH**4 / (384.0 * BEAM_EI)
    measured = _settled_deflection(SIMPLE_BEAM_CASE)
    print(f"ACC-042 simply supported static deflection: closed form {static!r} m, measured {measured!r} m")
    assert abs(measured / static - 1.0) <= 0.02


@pytest.mark.req("ACC-043")
def test_acc_043() -> None:
    """Acceptance criterion 4, rewritten: a simply supported beam's first natural frequency shall match the
    closed-form value within tolerance.

    The released bar swings about its static shape; the first-mode frequency is counted from the upward zero crossings
    of the node pose of the midspan segment about its final (settled) value, over at least ten periods after the first
    two (the higher modes decay faster under backward Euler). Step 0.25 ms: x = omega_1 h = 0.0360, frequency ratio
    atan(x) / x = 0.99957, inside 0.1 percent (ruling 45). Closed form omega_1 = pi^2 sqrt(E I / (rho A L^4)); tolerance
    the lumped chain's 2 percent at N = 8 (rulings 31, 32).
    """
    closed = math.pi**2 * math.sqrt(BEAM_EI / (BEAM_RHO_A * BEAM_LENGTH**4)) / (2.0 * math.pi)
    measured, periods = _first_frequency(SIMPLE_BEAM_CASE, 4)
    print(
        f"ACC-043 simply supported first frequency: closed form {closed!r} Hz, measured {measured!r} Hz over {periods}"
    )
    assert periods >= 10
    assert abs(measured / closed - 1.0) <= 0.02


@pytest.mark.req("ACC-044")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_044() -> None:
    """Acceptance criterion 5: a second, different design (the two-joint arm) shall be built and tested
    using only the library and data files.
    """
    raise NotImplementedError


@pytest.mark.req("ACC-045")
@pytest.mark.xfail(strict=True, reason="stub")
def test_acc_045() -> None:
    """Acceptance criterion 6: rerunning any case with the same seed shall give identical results."""
    raise NotImplementedError


# ===== PHY-016 ==============================================================
@pytest.mark.req("PHY-016")
def test_phy_016(gantry: dict[str, RunResult]) -> None:
    """The MuJoCo adapter shall carry the milestone 2 acceptance cases, G5 to G11, G13 and G14.

    Each case runs on the MuJoCo bridge through the whole stack without a run error and every criterion of it passes
    (the per-requirement tests above assert what each criterion means).
    """
    for case in ("G5", "G6", "G7", "G8", "G9", "G10", "G11", "G13", "G14"):
        result = gantry[case]
        report = result.report
        assert report is not None, result.score
        assert not report.errored, f"{case}: {report.error}"
        failing = [criterion.name for criterion in result.score.criteria if criterion.status != "pass"]
        assert failing == [], f"{case}: {failing}"


# ===== Native references (ACC-037, ACC-039 to ACC-043) =======================
ROPE_PENDULUM_CASE = ROOT / "test_cases" / "rope_pendulum" / "R1_period.yaml"
ROPE_SPRING_CASE = ROOT / "test_cases" / "rope_spring" / "S1_settle.yaml"
CANTILEVER_CASE = ROOT / "test_cases" / "cantilever_beam" / "B1_cantilever.yaml"
SIMPLE_BEAM_CASE = ROOT / "test_cases" / "simple_beam" / "B2_simply_supported.yaml"
# steel_bar_10mm_1m (catalog): E I = 200 GPa * 8.3333e-10 m^4, rho A = 7850 kg/m^3 * 1e-4 m^2, L = 1 m.
BEAM_EI = 200e9 * 8.333333333333334e-10
BEAM_RHO_A = 7850.0 * 1e-4
BEAM_LENGTH = 1.0
BEAM_W = BEAM_RHO_A * 9.81
_REFERENCE_RUNS: dict[Path, tuple[RunResult, CoreDriver, Any]] = {}


def _reference_run(case: Path) -> tuple[RunResult, CoreDriver, Any]:
    """One run per case through the CLI's path (case file, builder, native adapter, kernel, recorder, integrity,
    scoring) with the shipped services; the driver kept for its recording, the native build for its node chains."""
    if case not in _REFERENCE_RUNS:
        driver = CoreDriver()
        services = core_services()
        result = run_case(case, replace(services, driver=lambda: driver), None, None)
        assert result.report is not None, result.score
        assert not result.report.errored, result.report.error
        build = services.builder(prepare(case).design, "native")
        _REFERENCE_RUNS[case] = (result, driver, build)
    return _REFERENCE_RUNS[case]


def _poses(case: Path) -> tuple[BodyPoses, Any, np.ndarray]:
    _, driver, build = _reference_run(case)
    recording = driver.recording().recording
    rows = len(recording.series["time/seconds"].values)
    positions = np.asarray(recording.series["physics/bodies/position"].values, dtype=np.float64).reshape(rows, -1, 3)
    orientations = np.asarray(recording.series["physics/bodies/orientation"].values, dtype=np.float64).reshape(
        rows, -1, 4
    )
    times = np.asarray(recording.series["time/seconds"].values, dtype=np.float64)
    return BodyPoses(positions, orientations), node_chains(build.physics)["bar"], times


def _settled_deflection(case: Path) -> float:
    bodies, chain, _ = _poses(case)
    _, _, build = _reference_run(case)
    (member,) = build.members
    section = section_properties(member.section)
    view = MemberInputs("bar", str(member.material), section, (), None, nodes=chain)  # type: ignore[arg-type]
    last = BodyPoses(bodies.positions[-1:], bodies.orientations[-1:])
    return float(node_deflection(view, last)[0])


def _first_frequency(case: Path, node: int) -> tuple[float, int]:
    """Hz from upward crossings (linear interpolation) of a node's y about its final value in the run's first half
    (the swing still above 2 percent of the step there, so rounding near the settled value adds no crossing), dropping
    the first two; (crossings - 1) / (last - first) periods per second, and the period count."""
    bodies, chain, times = _poses(case)
    y = bodies.positions[:, chain.first_body + (node % chain.count), 1]
    y = y - y[-1]
    crossings: list[float] = []
    half = int(np.searchsorted(times, 0.5 * times[-1]))
    for index in range(1, half):
        if y[index - 1] < 0.0 <= y[index]:
            fraction = y[index - 1] / (y[index - 1] - y[index])
            crossings.append(times[index - 1] + fraction * (times[index] - times[index - 1]))
    kept = crossings[2:]
    periods = len(kept) - 1
    return periods / (kept[-1] - kept[0]), periods
