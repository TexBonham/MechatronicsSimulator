"""Strict-xfail stubs for the ACC spec (acceptance tier)."""

import itertools
import math
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest

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


@pytest.mark.req("BLD-039")
def test_bridge_keeps_the_rope_payload_on_its_carrier_with_one_line(gantry: dict[str, RunResult]) -> None:
    """G7 on the bridge (milestone 2, a 2 kg payload attached to the hook from the start, the design's integrity file
    present): the line is emitted only for a case payload (G3 has none), so G7's payload on payload_rope stays fixed
    to the hook with one stand-in line naming the rope and the carrier, tier 1 is reported not evaluated, and the
    case still passes (G12 and G15 hang the payload on the rope on native: `native_gantry`)."""
    result = gantry["G7"]
    assert result.report is not None
    lines = result.report.approximations
    stand_in = [line for line in lines if line.startswith("payload rope 'payload_rope': not modeled")]
    assert len(stand_in) == 1
    assert "fixed to 'hook'" in stand_in[0]
    assert any(line.startswith("integrity tier 1: not evaluated on this adapter") for line in lines)
    assert result.score.status == "pass"


# ===== ACC-025 to ACC-028 ===================================================
# G12 and G15 on the native adapter (rope payload, 100 microseconds): each runs once per module, archived, so the
# load case export and the archive digest are checked; the determinism test runs each a second time (invariant 4).
# The swing's pendulum length L: the hook's rope fitting 0.02 m below the hook origin, the rope's 0.3 m rest length and
# half the 0.1 m payload box to its centre (the swing is measured from the hook origin to the payload centre): 0.37 m.
SWING_A, SWING_G, SWING_V = 0.05, 9.81, 0.1
SWING_L = 0.02 + 0.3 + 0.05
HOOK_ON_LIMIT = 0.54


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


def _swing(driver: CoreDriver) -> tuple[Any, Any, Any]:
    """Times and the swing of the payload about the hook per axis: the line from the hook's origin (the hook_swing
    pivot) to the payload's centre, its angle from the vertical in the x-z and the y-z planes, from the recorded body
    poses indexed by name."""
    recording = driver.recording().recording
    positions = recording.series["physics/bodies/position"]
    names = list(positions.names)
    values = np.asarray(positions.values, dtype=np.float64)
    line = values[:, names.index("payload"), :] - values[:, names.index("hook"), :]
    times = np.asarray(driver.recording().series("pose:hook.x").times, dtype=np.float64)
    return times, np.arctan2(line[:, 0], -line[:, 2]), np.arctan2(line[:, 1], -line[:, 2])


def _peak(times: Any, values: Any, start: float, end: float) -> float:
    inside = (times >= start) & (times <= end)
    return float(np.max(np.abs(values[inside])))


def _residual(times: Any, values: Any, start: float) -> float:
    """The undamped free swing already present at `start`: its peak over the last rope period 2 pi sqrt(L / g) before
    `start`. The true pendulum is longer, so the window can miss the true peak: R0 reads low and the bound is tighter,
    never looser."""
    return _peak(times, values, start - 2.0 * math.pi * math.sqrt(SWING_L / SWING_G), start)


def _check_g12(driver: CoreDriver) -> None:
    names = list(driver.recording().recording.series["physics/bodies/position"].names)
    assert len(names) == 11 + 16 + 9
    assert names[9:11] == ["payload_shackle", "payload"]
    assert names[11] == "rail_x_beam.node0"
    assert names[27] == "rail_y_beam.node0"


@pytest.mark.req("ACC-025")
def test_acc_025(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """Reference design (gantry), G12: after a fast move and after an E-stop, the peak load swing angle
    shall be within its limit.

    The fast move (8 s to 20 s). Limit 0.1 rad (swing_limit_angle, placeholder), scored by the case on the swing angle.
    Oracle (G12 header), per axis: each of the six step changes of acceleration (a = 0.05 m/s^2) in the trapezoid with
    its slow zone adds a free swing of at most a / g about an offset of at most a / g, on top of the free swing R0 left
    by homing (measured over the last rope period before 8 s; no damping removes it): theta <= R0 + 7 a / g. The angle
    is measured from the body poses after checking the recorded body list (neutral bodies with the shackle and the
    payload, then the member nodes by name). The E-stop part: `test_acc_025_bounds`.
    """
    result, driver = native_gantry["G12"]
    _check_g12(driver)
    times, swing_x, swing_y = _swing(driver)
    for axis, swing in (("x", swing_x), ("y", swing_y)):
        bound = _residual(times, swing, 8.0) + 7.0 * SWING_A / SWING_G
        measured = _peak(times, swing, 8.0, 20.0)
        print(f"G12 move swing {axis}: {measured!r} rad (bound R0 + 7 a / g = {bound!r}, limit 0.1)")
        assert measured <= bound
        assert measured <= 0.1
    for name in ("move_swing_x", "move_swing_y"):
        criterion = _criterion(result, name)
        print(f"G12 {name}: swing {criterion.measured} rad ({criterion.limit})")
        assert criterion.status == "pass", criterion


def _hook_path(driver: CoreDriver, axis: int) -> Any:
    recording = driver.recording().recording
    positions = recording.series["physics/bodies/position"]
    return np.asarray(positions.values, dtype=np.float64)[:, list(positions.names).index("hook"), axis]


def _follower_peak(zeta: float) -> float:
    """Peak acceleration, per unit commanded acceleration, of a second-order follower (a carriage on a belt of
    stiffness k with the damper c in parallel, BLD-041) when the commanded position starts a ramp in rate: the input's
    acceleration steps from 0 to a. With H(s) = (2 zeta w s + w^2) / (s^2 + 2 zeta w s + w^2) the acceleration is
    a [1 - exp(-zeta w t) (cos w_d t - zeta / sqrt(1 - zeta^2) sin w_d t)], w_d = w sqrt(1 - zeta^2); its first maximum
    lies within the first damped period and does not depend on w, so it is taken from the closed form on a fine grid of
    w_d t over one period. At zeta = 0 it is exactly 2; it falls with zeta."""
    root = math.sqrt(1.0 - zeta * zeta)
    phase = np.linspace(0.0, 2.0 * math.pi, 200001)
    response = 1.0 - np.exp(-zeta * phase / root) * (np.cos(phase) - zeta / root * np.sin(phase))
    return float(np.max(response))


def _realised_ratio(result: RunResult, transmission: str) -> float:
    """The smallest of the damping ratios the run prints for `transmission` (stated, at build time, in this run;
    BLD-041): a smaller ratio gives the follower a larger peak, so the allowance stays an upper bound."""
    assert result.report is not None
    for line in result.report.approximations:
        if line.startswith(f"transmission '{transmission}': damping ratio stated "):
            numbers = [
                float(line.split("damping ratio stated ")[1].split(" ")[0]),
                float(line.split("realised at build time ")[1].split(" ")[0]),
                float(line.split("realised in this run ")[1].split(" ")[0]),
            ]
            return min(numbers)
    raise AssertionError(f"no damping line for {transmission}")


def _hook_acceleration(times: Any, path: Any, start: float, end: float, span: float) -> float:
    """The hook's largest acceleration over [start, end] as the second difference of its recorded position across
    `span` seconds (the acceleration averaged over that span)."""
    inside = (times >= start - span) & (times <= end + span)
    t = times[inside]
    x = path[inside]
    step = round(span / float(t[1] - t[0]))
    accel = (x[2 * step :] - 2.0 * x[step:-step] + x[: -2 * step]) / (span * span)
    centre = t[step:-step]
    keep = (centre >= start) & (centre <= end)
    return float(np.max(np.abs(accel[keep])))


def _forced_peak(times: Any, path: Any, start: float, end: float) -> float:
    """The largest swing a pendulum of length L hung from a pivot following `path` from rest at `start` can be forced
    to (Duhamel, integrated by parts): theta(t) = -(omega^2 / g) integral v(s) cos(omega (t - s)) ds, omega =
    sqrt(g / L); with v = a t it peaks at 2 a / g. The free swing already present adds on top (`_residual`)."""
    omega = math.sqrt(SWING_G / SWING_L)
    inside = (times >= start) & (times <= end)
    t = times[inside]
    velocity = np.gradient(path[inside] - path[inside][0], t)
    forced = [
        abs(np.trapezoid(velocity[: i + 1] * np.cos(omega * (t[i] - t[: i + 1])), t[: i + 1])) for i in range(len(t))
    ]
    return float(max(forced)) * omega * omega / SWING_G


@pytest.mark.req("ACC-025")
def test_acc_025_bounds(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """The first ramp and the E-stop (G12 header). Per axis: over the first ramp (8 s to 10 s) theta <= R0 + the
    swing the hook's recorded motion forces (2 a / g for an exact ramp), and a priori theta <= R0 + 2 a_max / g with
    a_max = a p(zeta) the peak a second-order follower of a ramp in rate permits (`_follower_peak`, zeta the smallest
    ratio the run prints for the axis belt; brief M3-X1f item 1; the recorded acceleration against a_max is
    `test_acc_025_first_ramp_allowance`); after the E-stop at full
    speed (22.5 s to 30 s), stopped in a time short against the period, theta <= R1 + v / sqrt(g L) with R1 the free
    swing at the press; the case's swing criterion within 0.1 rad; the lift on its lower limit through the stop (E6):
    the hook never above its drive-held height before the press, never below its height with the lift on the limit
    (0.54 m, design header; the limit row admits no penetration)."""
    result, driver = native_gantry["G12"]
    times, swing_x, swing_y = _swing(driver)
    for index, (axis, swing) in enumerate((("x", swing_x), ("y", swing_y))):
        residual = _residual(times, swing, 8.0)
        ramp = residual + _forced_peak(times, _hook_path(driver, index), 8.0, 10.0)
        stop = _residual(times, swing, 22.5) + SWING_V / math.sqrt(SWING_G * SWING_L)
        measured = (_peak(times, swing, 8.0, 10.0), _peak(times, swing, 22.5, 30.0))
        a_max = SWING_A * _follower_peak(_realised_ratio(result, f"{axis}_belt"))
        prior = residual + 2.0 * a_max / SWING_G
        print(f"G12 first ramp {axis}: {measured[0]!r} rad (R0 + forced {ramp!r}; R0 + 2 a_max / g {prior!r})")
        print(f"G12 E-stop {axis}: {measured[1]!r} rad (bound R1 + v / sqrt(g L) = {stop!r})")
        assert measured[0] <= ramp
        assert measured[0] <= prior
        assert measured[1] <= stop
    lift = _hook_path(driver, 2)
    held = lift[(times >= 22.5) & (times <= 30.0)]
    before = lift[(times >= 20.0) & (times < 22.5)]
    print(f"G12 hook height over the stop: {held.min()!r} to {held.max()!r} m (before {before.max()!r} m)")
    assert held.max() <= before.max()
    assert held.min() >= HOOK_ON_LIMIT - 1e-9
    criterion = _criterion(result, "estop_swing_x")
    print(f"G12 estop_swing_x: {criterion.measured} rad ({criterion.limit})")
    assert criterion.status == "pass", criterion


@pytest.mark.req("ACC-025")
@pytest.mark.xfail(
    strict=True,
    reason="brief M3-X1f item 1 finding: the firmware commands a step train (80 000 steps/m), not a ramp in rate, and "
    "each step rings the axis's drive and belt mode: over the first ramp the hook's recorded acceleration (1 ms second "
    "differences) peaks at 3.32 m/s^2 in y (8.777 s) and 0.56 m/s^2 in x against the follower allowances 0.0994 and "
    "0.0991 m/s^2; the swing stays within R0 + 2 a_max / g because the pendulum does not follow the ring",
)
def test_acc_025_first_ramp_allowance(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """The first ramp's drive allowance, a priori (brief M3-X1f item 1): the trolley follows the commanded ramp in rate
    through its belt, a second-order follower (stiffness with its damper in parallel, BLD-041), so its acceleration is
    at most a p(zeta), p the follower's peak response to a step of its input's acceleration (`_follower_peak`; 2 at
    zeta = 0), zeta the smallest damping ratio the run prints for the axis belt (a smaller ratio, a larger peak). The
    hook rides the trolley, so its recorded acceleration over 8 s to 10 s, as 1 ms second differences of its recorded
    position, is asserted within that allowance per axis."""
    result, driver = native_gantry["G12"]
    times, _, _ = _swing(driver)
    for index, axis in enumerate(("x", "y")):
        allowance = SWING_A * _follower_peak(_realised_ratio(result, f"{axis}_belt"))
        measured = _hook_acceleration(times, _hook_path(driver, index), 8.0, 10.0, 1e-3)
        print(f"G12 first ramp hook acceleration {axis}: {measured!r} m/s^2 (allowance a p(zeta) = {allowance!r})")
    for index, axis in enumerate(("x", "y")):
        allowance = SWING_A * _follower_peak(_realised_ratio(result, f"{axis}_belt"))
        assert _hook_acceleration(times, _hook_path(driver, index), 8.0, 10.0, 1e-3) <= allowance


@pytest.mark.req("ACC-026")
@pytest.mark.xfail(
    strict=True,
    reason="ruling 64: the gantry model has no swing damping (hook_swing 0, rope axial only); the scheme's 0.16 "
    "percent per period (L = 0.37 m, 100 microseconds) gives a derived settle of about 1135 s from the E-stop's 0.0450 "
    "rad into the 0.01 rad band against the 5 s placeholder (move_settle_x 5.208 s, estop_settle_x not settled by "
    "30 s); damping, limit and band owed by the gantry team",
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
    times, angle, _ = _swing(driver)
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
# G15 oracles (G15 header, brief M3-X1d item 1, brief M3-X1f): the calibrated chains' E I (N m^2) and w (N/m), the
# design load P with rope and shackle, the carried weights P_c (rail_y and all on it) and P_t (trolley, lift, hook),
# mid-travel on both rails. Static deflection w L^4 / (384 E I) + (P_c + P) L^3 / (192 E I) clamped-clamped (rail_x),
# 5 w L^4 / (384 E I) + (P_t + P) L^3 / (48 E I) pinned-roller (rail_y); static moment (P_c + P) L / 8 + w L^2 / 12 and
# (P_t + P) L / 4 + w L^2 / 8, stress M c / I. The case carries the design load (`integrity.design_load_carried`,
# TST-043), so the table's factor is 1, and integrity counts from 8 s (`evaluate_from`, TST-044). Both tiers read the
# exact moment under a carried load from the clean spring samples each side of the foot (INT-005, ruling 75), so
# the lower oracle of a peak is the continuum static stress less the chain's 2 percent. The upper oracle is what
# an unflagged member's validity bounds (rulings 73, 77): its deflection relative to its own-weight shape, and with it
# the bending stress, within 2.04 times the static carried value; a flagged member's results are not valid (ruling 69),
# never pass or fail. Ruling 73's three conditions for a result of record are checked by the acceptance test: (a) the
# verdicts restated with deflection and stress scaled by 2.04 over the member's peak dynamic ratio, (b) the verdicts
# reproduced at the step sizes run (10, 20 and 40 substeps, brief M3-X1f, G15 header), (c) the loaded first frequency
# against sqrt(k / (m_c + f m_beam)) (G15 header).
G15_SPAN = 0.5
G15_LOAD = 51.286
G15_TABLE_FACTOR = 1.0
G15_CHAIN = 0.02
G15_FLAG_LIMIT = 2.04
G15_RAILS = {
    "rail_x": (64.087, 51.641, 43.856, 192.0, 384.0 / 1.0, 8.0, 12.0),
    "rail_y": (256.35, 39.710, 9.000, 48.0, 384.0 / 5.0, 4.0, 8.0),
}


def _g15_oracle(member: str) -> tuple[float, float]:
    """(static deflection m, static stress Pa) of `member` at mid-travel under its own weight, its carried weight and
    the design load, before the table's factor."""
    stiffness, weight, carried, point, distributed, point_moment, distributed_moment = G15_RAILS[member]
    span = G15_SPAN
    deflection = weight * span**4 / (distributed * stiffness) + (carried + G15_LOAD) * span**3 / (point * stiffness)
    moment = (carried + G15_LOAD) * span / point_moment + weight * span**2 / distributed_moment
    return deflection, moment * 0.02 / 2e-8


def _g15_flags(result: RunResult) -> dict[str, tuple[float, bool]]:
    """Each lumped member's largest dynamic ratio from the evaluation start and whether the engine flagged it, from the
    run's approximation lines (ENG-052)."""
    assert result.report is not None
    found: dict[str, tuple[float, bool]] = {}
    for line in result.report.approximations:
        for member in ("rail_x", "rail_y"):
            prefix = f"lumped member '{member}_beam': the carried load-path pair did "
            if line.startswith(prefix):
                text = line.split("largest dynamic ratio from the evaluation start")[1].split(" against the limit")[0]
                found[member] = (float(text.rsplit(" ", 1)[1]), False)
    for line in result.report.approximations:
        for member in ("rail_x", "rail_y"):
            if line.startswith(f"lumped member '{member}_beam': the one-way carried-load result is not valid from"):
                found[member] = (found[member][0], True)
    assert sorted(found) == ["rail_x", "rail_y"], found
    return found


@pytest.mark.req("ACC-029")
@pytest.mark.xfail(
    strict=True,
    reason="placeholder rail section and PETG strength (gantry team): rail_x's safety factor is 1.424, 1.414, 1.280 "
    "at 10, 20, 40 substeps (16.549 s, when the E-stop drops the lift onto its lower limit; 9.616 s at 40) against the "
    "1.5 minimum (13.71 MPa on the 40 x 40 mm, 2e-8 m^4 placeholder section; the stop raises rail_x's root moment from "
    "7.0 to 9.6 N m); and rail_y is not valid from 8.31 s at every step size run (dynamic ratio 2.76, 3.63, 4.11 at "
    "10, 20, 40 substeps, against 2.04: the trolley starts 24 mm from rail_y's support; rulings 69, 78), so its "
    "safety factor is not evaluated",
)
def test_acc_029(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """Reference design (gantry), G15: under maximum acceleration with rated load and an E-stop with a
    swinging load, every member shall stay above the minimum safety factor.

    Minimum 1.5 (design, gantry team 2026-11-11). Per rail: the table's peak stress from 8 s (the carried design load)
    beside the static oracles, the engine's peak dynamic ratio and flag, and the verdict restated with the stress
    scaled by 2.04 over that ratio (ruling 73 (a): all that no flag bounds; a verdict that flips is not established).
    Asserted: neither rail flagged; each peak stress at least the continuum static stress less the chain's 2 percent
    and at most 2.04 times it (the validity bound); the case's safety-factor criteria pass, also when scaled; no tier 1
    crossing; the load case exported beside the archive.
    """
    result, _ = native_gantry["G15"]
    assert result.metadata is not None
    assert result.metadata.integrity is not None
    assert result.archive is not None
    for suffix in (".load_case.json", ".load_case.csv", ".integrity.json"):
        assert result.archive.with_suffix(suffix).is_file(), suffix
    flags = _g15_flags(result)
    print(f"G15 crossings: {result.score.crossings}")
    for member in ("rail_x", "rail_y"):
        row = result.metadata.integrity.member(member)
        static = _g15_oracle(member)[1] * G15_TABLE_FACTOR
        ratio, flagged = flags[member]
        scaled = row.safety_factor * ratio / G15_FLAG_LIMIT
        print(
            f"G15 {member}: safety factor {row.safety_factor!r} at {row.safety_factor_time!r} s (minimum 1.5), peak "
            f"stress {row.peak_stress!r} Pa at {row.peak_stress_time!r} s (continuum static {static!r} Pa); dynamic "
            f"ratio {ratio!r}, flagged {flagged}; scaled by {G15_FLAG_LIMIT} / ratio: safety factor {scaled!r}"
        )
    for member in ("rail_x", "rail_y"):
        row = result.metadata.integrity.member(member)
        static = _g15_oracle(member)[1] * G15_TABLE_FACTOR
        ratio, flagged = flags[member]
        assert not flagged
        assert _criterion(result, f"safety_factor_{member}").status == "pass"
        assert row.safety_factor * ratio / G15_FLAG_LIMIT >= 1.5
        assert row.peak_stress >= (1.0 - G15_CHAIN) * static
        assert row.peak_stress <= G15_FLAG_LIMIT * static
    assert result.score.crossings == ()
    assert result.score.status == "pass"


@pytest.mark.req("ACC-030")
@pytest.mark.xfail(
    strict=True,
    reason="rail_y is not valid from 8.31 s at every step size run (dynamic ratio 2.76, 3.63, 4.11 at 10, 20, 40 "
    "substeps against 2.04; rulings 69, 78: placeholder rails and belts at the fixed 0.01 damping), so its "
    "deflection criteria are not evaluated; rail_x passes at every step size (peak 1.134, 1.141, 1.119 mm; scaled by "
    "2.04 over its ratio 2.25, 1.91, 1.46 mm; settled 1.0806 mm against 1.0977 mm)",
)
def test_acc_030(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """Reference design (gantry), G15: under the same worst cases, member deflection shall stay under the
    target.

    Target 1 cm (ACC-030 row), the dynamic node-pose peak from 8 s (the conservative reading). Settled (14 s to 15 s,
    both carriages at mid-travel): the case's own criterion within 2 percent of the closed form (G15 header; the
    table's factor is 1, the case carrying the design load). Peak: at most 2.04 times the static value (the validity
    bound of an unflagged member) and under the target, also when scaled by 2.04 over the member's peak dynamic ratio
    (ruling 73 (a)); neither rail flagged.
    """
    result, _ = native_gantry["G15"]
    assert result.metadata is not None
    assert result.metadata.integrity is not None
    flags = _g15_flags(result)
    for member in ("rail_x", "rail_y"):
        static = _g15_oracle(member)[0] * G15_TABLE_FACTOR
        settled = _criterion(result, f"settled_deflection_{member}")
        peak = _criterion(result, f"deflection_{member}")
        print(
            f"G15 {member}: settled {settled.measured} m ({settled.status}), peak from 8 s {peak.measured} m at "
            f"{peak.worst_time} s ({peak.status}); static oracle {static!r} m; dynamic ratio, flagged {flags[member]}"
        )
    for member in ("rail_x", "rail_y"):
        static = _g15_oracle(member)[0] * G15_TABLE_FACTOR
        ratio, flagged = flags[member]
        assert not flagged
        assert _criterion(result, f"settled_deflection_{member}").status == "pass"
        peak = _criterion(result, f"deflection_{member}")
        assert peak.status == "pass"
        assert peak.measured <= G15_FLAG_LIMIT * static
        assert peak.measured * G15_FLAG_LIMIT / ratio < 0.01


@pytest.mark.req("INV-014")
@pytest.mark.parametrize("case", ["G12", "G15"])
def test_native_gantry_cases_repeat_bit_identically(
    case: str, native_gantry: dict[str, tuple[RunResult, CoreDriver]], tmp_path: Path
) -> None:
    """G12 and G15 run twice with the same seed give the same recording, compared bitwise (invariant 4: every series'
    values and times, every event), and the same run report. The archive files are not compared: they also carry the
    archive's own path and write metadata."""
    first, one = native_gantry[case]
    second, two = _native_run(case, tmp_path)
    a = one.recording().recording
    b = two.recording().recording
    assert sorted(a.series) == sorted(b.series)
    for path, series in a.series.items():
        assert np.asarray(series.values).tobytes() == np.asarray(b.series[path].values).tobytes(), path
    assert a.events == b.events
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
    damps only axially, so the undamped swing keeps its release amplitude L sin(0.02) but for the scheme's loss,
    (1 + x^2)^(-pi / atan x) kept per period at x = omega h = 1.57e-4, 1 percent over 20 periods (ruling 45).
    Tolerances (ACC-037 row, proposed): period 0.1 percent, amplitude after 20 periods 2 percent. The case's own
    criteria carry both; a rope 5 percent longer (placed at its own loaded length) lengthens the period by
    sqrt(1.05) - 1 = 2.5 percent and must fail the period criterion.
    """
    m, m_r, l0, ea, g, n = 1.0, 0.035, 1.0, 5e5, 9.81, 8
    length = l0 + g * l0 * (m + m_r / 2.0) / ea
    inertia = m * length**2 + m_r * length**2 * (1.0 / 3.0 + 1.0 / (6.0 * n * n))
    moment = m * length + m_r * length / 2.0
    period_n = 2.0 * math.pi * math.sqrt(inertia / (g * moment))
    release = length * math.sin(0.02)
    result = _reference_run(ROPE_PENDULUM_CASE)
    period = _criterion(result, "period")
    amplitude = _criterion(result, "amplitude_after_20_periods")
    print(f"ACC-037 period: closed form T_N {period_n!r} s, measured {period.measured!r} s")
    print(f"ACC-037 amplitude after 20 periods: release {release!r} m, measured {amplitude.measured!r} m")
    assert abs(float(period.measured) / period_n - 1.0) <= 1e-3
    assert abs(float(amplitude.measured) / release - 1.0) <= 0.02
    assert result.score.status == "pass", result.score
    longer = 1.05 + g * 1.05 * (m + m_r * 1.05 / 2.0) / ea
    perturbed = _reference_run(
        ROPE_PENDULUM_CASE,
        (
            ("bob_x", f"{longer * math.sin(0.02)!r} m"),
            ("bob_y", f"{-longer * math.cos(0.02)!r} m"),
            ("rope_length", "1.05 m"),
        ),
    )
    print(f"ACC-037 perturbed period: {_criterion(perturbed, 'period').measured!r} s")
    assert _criterion(perturbed, "period").status == "fail"
    assert perturbed.score.status == "fail"


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
    the rope's lower half-mass), k = EA / L0 = 500 kN/m, c = 50 N s / L0, released at the rest length under gravity
    (the step), x_s = M g / k. Continuous closed form (ruling 59): t_s = ln(50) / (zeta omega_n) = 2 M ln(50) / c. The
    run's step h = 1 us makes backward Euler's decay sigma_h = ln|1 - h s| / h faster by k h / c = 1 percent (ruling
    45), inside the tolerance; that corrected value is printed beside it. The last exit from the band lies at a peak
    under the envelope, at most half a damped period pi / omega_d before the envelope reaches the band. Tolerance
    (ACC-039 row, proposed): t_s (1 - 0.02) - pi / omega_d <= measured <= t_s (1 + 0.02), both bounds in the case. A
    rope 5 percent longer lowers k and c by 1 / 1.05: x_s grows 5 percent (2.5 band widths), so the series never
    settles about the stated target and the upper settle criterion fails.
    """
    m, k, c, h = 1.0175, 5e5, 50.0, 1e-6
    omega = math.sqrt(k / m)
    zeta = c / (2.0 * m * omega)
    damped = omega * math.sqrt(1.0 - zeta * zeta)
    sigma_h = math.log(abs(1.0 - h * complex(-zeta * omega, damped))) / h
    continuous = math.log(50.0) / (zeta * omega)
    corrected = math.log(50.0) / sigma_h
    result = _reference_run(ROPE_SPRING_CASE)
    late = _criterion(result, "settle_not_late")
    measured = float(late.measured)
    print(
        f"ACC-039 settle time: closed form {continuous!r} s, scheme-corrected {corrected!r} s (x = omega_n h = "
        f"{omega * h!r}), measured {measured!r} s"
    )
    assert abs(corrected / continuous - 1.0) <= 0.02
    assert continuous * 0.98 - math.pi / damped <= measured <= continuous * 1.02
    assert _criterion(result, "settle_not_early").measured == late.measured
    assert result.score.status == "pass", result.score
    perturbed = _reference_run(ROPE_SPRING_CASE, (("bob_y", "-1.05 m"), ("rope_length", "1.05 m")))
    print(f"ACC-039 perturbed: {_criterion(perturbed, 'settle_not_late')}")
    assert _criterion(perturbed, "settle_not_late").status == "fail"
    assert perturbed.score.status == "fail"


def _beam(case: Path, static: float, frequency: float, perturb: bool) -> None:
    """Settled deflection and first frequency of the case against the closed forms within the lumped chain's 2
    percent (rulings 31, 32); with `perturb`, a second moment 5 percent larger lowers the static deflection by
    1 - 1 / 1.05 = 4.8 percent and must fail the settled-deflection criterion."""
    result = _reference_run(case)
    settled = _criterion(result, "settled_deflection")
    first = _criterion(result, "first_frequency")
    print(f"{case.stem}: static closed form {static!r} m, settled {settled.measured!r} m")
    print(f"{case.stem}: first frequency closed form {frequency!r} Hz, measured {first.measured!r} Hz")
    assert abs(float(settled.measured) / static - 1.0) <= 0.02
    assert abs(float(first.measured) / frequency - 1.0) <= 0.02
    assert result.score.status == "pass", result.score
    if perturb:
        stiffer = _reference_run(case, (("bar_second_moment", f"{BEAM_I * 1.05!r} m**4"),))
        print(f"{case.stem} perturbed: settled {_criterion(stiffer, 'settled_deflection').measured!r} m")
        assert _criterion(stiffer, "settled_deflection").status == "fail"
        assert stiffer.score.status == "fail"


@pytest.mark.req("ACC-040")
def test_acc_040() -> None:
    """Acceptance criterion 4, rewritten: a cantilever beam's static deflection shall match the closed-form
    value within tolerance.

    Through the full stack on the native adapter (`test_cases/cantilever_beam/B1_cantilever.yaml`): the bar released
    straight under its own weight w = rho A g settles under backward Euler's per-period decay (ruling 45); the case's
    `settled_deflection` is the mean of the node-pose deflection (INT-006: distance from the root tangent) over the
    final second (SCO-028). Closed form w L^4 / (8 E I); tolerance the lumped chain's 2 percent at N = 8 (rulings 31,
    32; the chain's own value under the distributed load is 0.78 percent low).
    """
    _beam(CANTILEVER_CASE, BEAM_W * BEAM_LENGTH**4 / (8.0 * BEAM_EI), _cantilever_frequency(), True)


@pytest.mark.req("ACC-041")
def test_acc_041() -> None:
    """Acceptance criterion 4, rewritten: a cantilever beam's first natural frequency shall match the
    closed-form value within tolerance.

    The case's `first_frequency` counts upward crossings of the deflection about its mean from 0.25 s to 1.75 s,
    twelve periods (SCO-027). Step 1 ms: x = omega_1 h = 0.0519, frequency ratio atan(x) / x = 0.99910, inside 0.1
    percent (ruling 45). Closed form omega_1 = 1.8751^2 sqrt(E I / (rho A L^4)); tolerance the lumped chain's 2
    percent at N = 8 (rulings 31, 32).
    """
    x = 2.0 * math.pi * _cantilever_frequency() * 1e-3
    assert 1.0 - math.atan(x) / x <= 1e-3
    _beam(CANTILEVER_CASE, BEAM_W * BEAM_LENGTH**4 / (8.0 * BEAM_EI), _cantilever_frequency(), False)


@pytest.mark.req("ACC-042")
def test_acc_042() -> None:
    """Acceptance criterion 4, rewritten: a simply supported beam's static deflection shall match the closed-form
    value within tolerance.

    Through the full stack on the native adapter (`test_cases/simple_beam/B2_simply_supported.yaml`): the case's
    `settled_deflection` is the mean of the node-pose deflection (INT-006: distance from the chord through the pin and
    the far end) over the final half second (SCO-028). Closed form 5 w L^4 / (384 E I); tolerance the lumped chain's 2
    percent at N = 8 (rulings 31, 32; the chain's own value under the distributed load is 0.625 percent low).
    """
    _beam(SIMPLE_BEAM_CASE, 5.0 * BEAM_W * BEAM_LENGTH**4 / (384.0 * BEAM_EI), _simple_frequency(), True)


@pytest.mark.req("ACC-043")
def test_acc_043() -> None:
    """Acceptance criterion 4, rewritten: a simply supported beam's first natural frequency shall match the
    closed-form value within tolerance.

    The case's `first_frequency` counts upward crossings of the deflection about its mean from 0.1 s to 0.7 s,
    thirteen periods (SCO-027). Step 0.25 ms: x = omega_1 h = 0.0364, frequency ratio atan(x) / x = 0.99956, inside
    0.1 percent (ruling 45). Closed form omega_1 = pi^2 sqrt(E I / (rho A L^4)); tolerance the lumped chain's 2 percent
    at N = 8 (rulings 31, 32).
    """
    x = 2.0 * math.pi * _simple_frequency() * 2.5e-4
    assert 1.0 - math.atan(x) / x <= 1e-3
    _beam(SIMPLE_BEAM_CASE, 5.0 * BEAM_W * BEAM_LENGTH**4 / (384.0 * BEAM_EI), _simple_frequency(), False)


def _cantilever_frequency() -> float:
    return 1.8751040687119611**2 * math.sqrt(BEAM_EI / (BEAM_RHO_A * BEAM_LENGTH**4)) / (2.0 * math.pi)


def _simple_frequency() -> float:
    return math.pi**2 * math.sqrt(BEAM_EI / (BEAM_RHO_A * BEAM_LENGTH**4)) / (2.0 * math.pi)


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
# The beam designs: a 10 mm square bar of steel_8_8_fastener, E 205 GPa and rho 7850 kg/m^3 typical, I = b^4 / 12,
# A = 1e-4 m^2, L = 1 m.
BEAM_I = 0.01**4 / 12.0
BEAM_EI = 205e9 * BEAM_I
BEAM_RHO_A = 7850.0 * 1e-4
BEAM_LENGTH = 1.0
BEAM_W = BEAM_RHO_A * 9.81
_REFERENCE_RUNS: dict[tuple[Path, tuple[tuple[str, str], ...]], RunResult] = {}


def _reference_run(case: Path, bindings: tuple[tuple[str, str], ...] = ()) -> RunResult:
    """One run per case and `--set` bindings through the CLI's path (case file, builder, native adapter, kernel,
    recorder, integrity, scoring) with the shipped services."""
    key = (case, bindings)
    if key not in _REFERENCE_RUNS:
        result = run_case(case, core_services(), dict(bindings), None)
        assert result.report is not None, result.score
        assert not result.report.errored, result.report.error
        _REFERENCE_RUNS[key] = result
    return _REFERENCE_RUNS[key]
