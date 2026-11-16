"""Strict-xfail stubs for the ACC spec (acceptance tier)."""

import copy
import io
import itertools
import math
import re
import shutil
import time
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from ruamel.yaml import YAML

from mechsim.cli import default_services, main
from mechsim.design.schema import shipped_materials
from mechsim.library.registry import Registry, default_registry, load_registry
from mechsim.library.schema import CATALOG_DIR
from mechsim.scoring.score import CriterionResult, available_torque, score_run
from mechsim.testing.core_driver import CoreDriver, core_services
from mechsim.testing.runner import RunResult, Services, prepare, run_case, stepper_drives, sweep_runs
from mechsim.testing.schema import ROOT, ValueCriterion, load_suite_file
from tests.conftest import CaseRuns

PENDULUM_CASE = ROOT / "test_cases" / "pendulum" / "P1_period.yaml"
GANTRY_CASES = ROOT / "test_cases" / "gantry"
_VIEWS: dict[str, CoreDriver] = {}
BRIDGE = {"physics.adapter": "mujoco"}
# Cases whose acceptance runs stay on the bridge, each with its reason; empty since brief M3-X2c (G5 and G6 run on
# native once brief M3-N5 let the adapter change the mass of a rope-hung payload).
BRIDGE_RUNS: tuple[str, ...] = ()


# ===== Fixtures and helpers =================================================
class _Runs(dict[str, Any]):
    """Runs a case on first access and keeps it for the module, so a filtered run (`-k`) runs only the cases its
    tests read."""

    def __init__(self, run: Callable[[str], Any]) -> None:
        super().__init__()
        self._run = run

    def __missing__(self, case: str) -> Any:
        self[case] = self._run(case)
        return self[case]


def _cached(case_runs: CaseRuns, case: str, bindings: dict[str, str] | None) -> RunResult:
    result, driver = case_runs.get(GANTRY_CASES / f"{case}.yaml", bindings)
    _VIEWS[case] = driver
    assert result.report is not None, result.score
    return result


@pytest.fixture(scope="module")
def gantry(case_runs: CaseRuns) -> dict[str, RunResult]:
    """G1 to G14 (G12 is `native_gantry`'s), each once and on first use, through the real stack (gantry design,
    builder, the default native adapter, kernel, signal layer, native gantry firmware, recorder, scoring); placeholder
    defaults for every open parameter (DES-044); `BRIDGE_RUNS` (now none) on the bridge. Runs come from the session
    cache (`tests/conftest.py`), so a case another module also reads runs once. A case whose run errors fails its own
    tests (`_passes`), not every case's. The fast tier reads only G1 here; the acceptance checks of G2 to G11, G13 and
    G14 are full native runs of 40 to 130 s each and carry the long-run marker (brief M3-X2d)."""

    def run(case: str) -> RunResult:
        bindings = None
        if case in BRIDGE_RUNS:
            bindings = BRIDGE
        return _cached(case_runs, case, bindings)

    return _Runs(run)


@pytest.fixture(scope="module")
def bridge_gantry(case_runs: CaseRuns) -> dict[str, RunResult]:
    """The milestone 2 cases on the MuJoCo bridge (`physics.adapter: mujoco`, TST-040), each once and on first use,
    for the bridge's own requirement (PHY-016) while the bridge stays (ADR 0004); session-cached like `gantry`."""

    def run(case: str) -> RunResult:
        result, _ = case_runs.get(GANTRY_CASES / f"{case}.yaml", BRIDGE)
        assert result.report is not None, result.score
        return result

    return _Runs(run)


@pytest.fixture(scope="module")
def bridge_g7(case_runs: CaseRuns) -> RunResult:
    """G7 on the bridge, for the bridge's own rope stand-in (BLD-039); session-cached."""
    result, _ = case_runs.get(GANTRY_CASES / "G7.yaml", BRIDGE)
    assert result.report is not None, result.score
    return result


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
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
def test_acc_009(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G5: with a load over the limit attached before a move, a lift command
    shall be ignored.

    Tolerance not set (owed by gantry team): the behavior is asserted. With 5.5 kg on the hook (over the 5.216 kg
    limit, rated 10 lb + 15 percent, gantry team 2026-11-11) `lift 0.4` at 18 s is refused with fault overload, and the
    lift stays within the 1 cm hold band (the position accuracy) until the next command at 20 s.

    On native the payload hangs on its rope; the load cell reads the shackle's reaction, 3308 counts at 5.5 kg
    against the bridge's 3301 on its fixed payload, so the firmware's overload decision is the bridge's (the lift
    held within 2.2e-7 m). The refusal's reason is asserted, not only its presence: without the suite's 9 s
    evaluation start the release transient snaps the rope at 0.012 s (6011 N against 5000 N), the cell reads empty,
    `lift 0.4` is accepted and a `busy` refusal of the 20 s command would satisfy `lift_up_refused` alone (M3-X2c).
    """
    result = gantry["G5"]
    _passes(result, "lift_up_refused", "lift_up_ignored")
    _refused_with(result, 18.0, 20.0, "overload")


@pytest.mark.req("ACC-010")
@pytest.mark.req("FW-018")
@pytest.mark.long_run
def test_acc_010(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G5: with a load over the limit attached, a lower command shall work.

    `lift 0.1` at 20 s from 0.3 m with 5.5 kg: no refusal to the end at 26 s, and the lift ends within 1 cm (the
    position accuracy) of 0.1 m. On native it ends at 0.09961 m (the bridge 0.10003 m).
    """
    result = gantry["G5"]
    _passes(result, "lowered")
    assert _refusals(result, 20.0, 26.0) == 0


@pytest.mark.req("ACC-011")
@pytest.mark.req("FW-019")
@pytest.mark.req("TST-020")
@pytest.mark.long_run
def test_acc_011(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G6: when the load becomes overweight mid-lift, the lift shall stop.

    The payload becomes 5.5 kg 2 s into the lift (TST-020 routes the change to physics). Bound from the gantry team's
    limits of 2026-11-11 and ruling 35 (G6 header): 0.1 m risen at the change, at most 1.12 cm more during detection
    (12.5 ms cell sample, 97.3 ms for the 2 Hz amplifier to cross 5.216 kg, 2 ms filter, at 0.1 m/s), v^2 / 2a = 0.1 m
    to stop at 0.05 m/s^2: the lift stays under 21.1 cm, short of the 45 cm commanded. On native it peaks at
    0.20885 m (the bridge 0.20825 m).
    """
    _passes(gantry["G6"], "lift_stopped")


@pytest.mark.req("ACC-012")
@pytest.mark.req("FW-019")
@pytest.mark.long_run
def test_acc_012(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G6: after an overweight stop, the load shall be able to be set down.

    `lift 0.05` after the stop, still at 5.5 kg: the lift ends within 1 cm of 5 cm. On native it ends at 0.04996 m
    (the bridge 0.05005 m).
    """
    _passes(gantry["G6"], "set_down")


@pytest.mark.req("ACC-013")
@pytest.mark.req("FW-024")
@pytest.mark.long_run
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
@pytest.mark.long_run
def test_acc_014(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G7: after the E-stop, the hook shall not drop.

    The hook is lifted to 30 cm first, so the brake and not the lift's lower joint limit holds it (X review): the lift
    stays within the 1 cm hold band (position accuracy) of its position at the press at 12 s until the home at 15 s;
    the power-off brake engages when the motor rail drops (ruling 1).
    """
    _passes(gantry["G7"], "hook_held")


@pytest.mark.req("ACC-015")
@pytest.mark.req("FW-024")
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
def test_acc_022(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G11: when a limit switch trips mid-workspace, motion shall stop.

    x_max_switch forced high at 8.5 s for 0.2 s, 6.25 mm into the move: no driver steps from 8.5 + 0.2 + 0.7 s (stop
    at 0.05 m/s^2 from at most 0.035 m/s, G11 header) until the home.
    """
    _passes(gantry["G11"], "stopped_x", "stopped_y")


@pytest.mark.req("ACC-023")
@pytest.mark.req("FW-021")
@pytest.mark.long_run
def test_acc_023(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G11: when a limit switch trips mid-workspace, a fault shall be reported.

    The firmware enters the fault state and the next refusal carries fault limit_trip.
    """
    result = gantry["G11"]
    _passes(result, "limit_fault")
    _refused_with(result, 10.0, 10.5, "limit_trip")


@pytest.mark.req("ACC-024")
@pytest.mark.req("FW-021")
@pytest.mark.long_run
def test_acc_024(gantry: dict[str, RunResult]) -> None:
    """Reference design (gantry), G11: after a mid-workspace limit trip, re-homing shall be required before
    further motion.

    The target at 10 s is refused; homing starts after `home` at 10.5 s.
    """
    _passes(gantry["G11"], "target_refused", "rehoming")


@pytest.mark.req("BLD-039")
def test_bridge_keeps_the_rope_payload_on_its_carrier_with_one_line(bridge_g7: RunResult) -> None:
    """G7 on the bridge (milestone 2, a 2 kg payload attached to the hook from the start, the design's integrity file
    present): the line is emitted only for a case payload (G3 has none), so G7's payload on payload_rope stays fixed
    to the hook with one stand-in line naming the rope and the carrier, tier 1 is reported not evaluated, and the
    case still passes (on the default native adapter the payload hangs on the rope)."""
    result = bridge_g7
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
    return _Runs(lambda case: _native_run(case, out))


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
@pytest.mark.long_run
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
@pytest.mark.long_run
def test_acc_025_bounds(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """The first ramp and the E-stop (G12 header). Per axis: over the first ramp (8 s to 10 s) theta <= R0 + the
    swing the hook's recorded motion forces (2 a / g for an exact ramp), and a priori theta <= R0 + 2 a_max / g with
    a_max = a p(zeta) the peak a second-order follower of a ramp in rate permits (`_follower_peak`, zeta the smallest
    ratio the run prints for the axis belt; brief M3-X1f item 1; the recorded acceleration against a_max is
    `test_acc_025_first_ramp_allowance`); after the E-stop at full speed (22.5 s to 30 s), stopped in a time short
    against the period, theta <= R1 + v / sqrt(g L) with R1 the free swing at the press; two-sided on the first ramp
    (ruling 60): theta is the free part plus the forced part, so its peak is at least the forced peak less R0; the
    case's swing criterion within 0.1 rad; the lift on its lower limit through the stop (E6): the hook never above
    its drive-held height before the press, never below its height with the lift on the limit (0.54 m, design header;
    the limit row admits no penetration)."""
    result, driver = native_gantry["G12"]
    times, swing_x, swing_y = _swing(driver)
    for index, (axis, swing) in enumerate((("x", swing_x), ("y", swing_y))):
        residual = _residual(times, swing, 8.0)
        forced = _forced_peak(times, _hook_path(driver, index), 8.0, 10.0)
        ramp = residual + forced
        stop = _residual(times, swing, 22.5) + SWING_V / math.sqrt(SWING_G * SWING_L)
        measured = (_peak(times, swing, 8.0, 10.0), _peak(times, swing, 22.5, 30.0))
        a_max = SWING_A * _follower_peak(_realised_ratio(result, f"{axis}_belt"))
        prior = residual + 2.0 * a_max / SWING_G
        print(f"G12 first ramp {axis}: {measured[0]!r} rad (R0 + forced {ramp!r}; R0 + 2 a_max / g {prior!r})")
        print(f"G12 E-stop {axis}: {measured[1]!r} rad (bound R1 + v / sqrt(g L) = {stop!r})")
        assert measured[0] <= ramp
        assert measured[0] >= forced - residual
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
    raises=AssertionError,
    reason="brief M3-X1g item 5: the firmware commands a step train (80 000 steps/m, q = 12.5 micrometres), not a "
    "ramp in rate; each step rings the axis's belt mode by q omega^2: x 0.25 m/s^2 on the measured 22.6 Hz mode "
    "(the carriage with rail_y's beam), y 1.36 m/s^2 on 52.5 Hz (trolley 0.918 kg, 1e5 N/m), against the allowances "
    "0.098 and 0.098 m/s^2; measured 0.56 in x and 3.32 in y at 8.777 s, where the per-tick step-count beat "
    "abs(f_s - 3000 Hz) crosses the 52.5 Hz belt mode (8.737 and 8.763 s); no gantry value in the data converts it "
    "(the drive's step size or microstepping, or belt damping, would)",
)
@pytest.mark.long_run
def test_acc_025_first_ramp_allowance(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """The first ramp's drive allowance, a priori (brief M3-X1f item 1): the trolley follows the commanded ramp in rate
    through its belt, a second-order follower (stiffness with its damper in parallel, BLD-041), so its acceleration is
    at most a p(zeta), p the follower's peak response to a step of its input's acceleration (`_follower_peak`; 2 at
    zeta = 0), zeta the smallest damping ratio the run prints for the axis belt (a smaller ratio, a larger peak). The
    hook rides the trolley, so its recorded acceleration over 8 s to 10 s, as 1 ms second differences of its recorded
    position, is asserted within that allowance per axis.

    Why it fails (brief M3-X1g item 5), from the data: the firmware does not command a ramp in rate but whole steps
    of q = 1 / 80 000 m = 12.5 micrometres. A step of the belt's input by q starts a free ring of the belt mode with
    acceleration amplitude q omega^2. y: m = 0.918 kg (trolley, lift, hook, shackle; the payload hangs on the rope, a
    pendulum to this mode), k = 1e5 N/m, omega^2 = 1.089e5 s^-2 (52.5 Hz), q omega^2 = 1.36 m/s^2; x: the carriage
    carries rail_y's beam chain (4.47 kg, the measured ring 22.6 Hz with the drive's own compliance in series),
    q omega^2 = 0.25 m/s^2. Both exceed a p(zeta) = 0.098 m/s^2
    by far. Measured: x 0.56 m/s^2 (rings of successive steps in phase), y 3.32 m/s^2 at 8.777 s: at v = a (t - 8 s)
    the step rate is f_s = 80 000 v, 3000 steps/s at 8.75 s, and the per-tick step count beats at abs(f_s - 3000 Hz),
    which crosses 52.5 Hz at 8.737 s and 8.763 s, driving the belt mode at resonance. No gantry value in the data
    today converts it: a smaller step (microstepping or the drive's step size) or belt damping would."""
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
    raises=AssertionError,
    reason="ruling 64: the gantry model has no swing damping (hook_swing 0, rope axial only); the scheme's 0.16 "
    "percent per period (L = 0.37 m, 100 microseconds) gives a derived settle of about 1135 s from the E-stop's 0.0450 "
    "rad into the 0.01 rad band against the 5 s placeholder (move_settle_x 5.208 s, estop_settle_x not settled by "
    "30 s); damping, limit and band owed by the gantry team",
)
@pytest.mark.long_run
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
@pytest.mark.long_run
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
@pytest.mark.long_run
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
# the lower oracle of a peak is the continuum static stress less the chain's 2 percent. The upper oracle is what an
# unflagged member's validity bounds (ENG-052, rulings 73, 77): from 8 s its deflection relative to its own-weight
# shape, anywhere on the span, within 2.04 times the larger of the windowed peak of its carried loads' largest nodal
# quasi-static deflection (under the actual loads, the lift's landing in the stop included) and the worst-case static
# carried deflection over the travel; the engine's dynamic ratio is that quotient, so the bound is asserted as the
# printed ratio at most 2.04 with no flag. No fixed multiple of the mid-travel static value is a bound: the base is
# larger than it whenever the loads peak (brief M3-X1g item 4). A flagged member's results are not valid (ruling
# 69), never pass or fail. Ruling 73's conditions for a result of record: (a) the verdicts restated with deflection
# and stress scaled by 2.04 over the member's peak dynamic ratio, checked here; (b) the verdicts reproduced at 10, 20
# and 40 substeps with no member flagged at any of them: the fixture runs 10 substeps only, so the other two are the
# record of the native baseline (`_g15_record`, ruling 99 (b); the gate report's 20 and 40 substep rows were the
# placeholder rails'), asserted here: the 10-substep run must match its row,
# and every row must be unflagged and agree, so a 10-substep pass alone cannot convert the stubs; (c) the loaded
# first frequency is reported in the G15 header (the chain's eigenfrequency with the engine's carried mass).
G15_TABLE_FACTOR = 1.0
G15_CHAIN = 0.02
G15_FLAG_LIMIT = 2.04
G15_CASE = ROOT / "test_cases" / "gantry" / "G15.yaml"
G15_BASELINE = ROOT / "test_cases" / "gantry" / "baseline.yaml"
# Per support pair: (point-load deflection divisor, distributed-load deflection divisor, point-load moment divisor,
# distributed-load moment divisor) at mid-span; closed forms, not data.
G15_SUPPORTS = {
    ("clamped", "clamped"): (192.0, 384.0, 8.0, 12.0),
    ("pinned", "roller"): (48.0, 384.0 / 5.0, 4.0, 8.0),
}


def _g15_rails(result: RunResult) -> dict[str, tuple[float, ...]]:
    """Ruling 99 (a), (b): every G15 oracle input from the data, never a literal. Per rail: span L, E I and w = rho A g
    from the design's section and the library material's typical E and rho, the carried weight from the engine's
    carried mass the run prints (`realised at build time ... (carried mass m kg)`, BLD-040), the support divisors, and
    the fibre distance and second moment of the bending plane (`fibre_y`, `second_moment_z`). Also the design load P:
    the case's `design_load_mass` plus the rope (mass per length times rest length) and its end fitting, from the
    catalog part the design names on payload_rope."""
    assert result.report is not None
    prepared = prepare(G15_CASE)
    design = prepared.design
    registry = default_registry()
    materials = shipped_materials()
    case = YAML(typ="safe", pure=True).load(G15_CASE.read_text(encoding="utf-8"))
    load_text = next(item["default"] for item in case["open_parameters"] if item["name"] == "design_load_mass")
    rope = registry.part(next(item.part for item in design.ropes if item.name == "payload_rope")).values
    load = (
        float(str(load_text).split(" ")[0]) + rope["mass_per_length"] * rope["rest_length"] + rope["end_fitting_mass"]
    )
    gravity = 9.81
    rails: dict[str, tuple[float, ...]] = {}
    for body in design.bodies:
        if body.flexible is None:
            continue
        section = body.flexible.section
        material = materials[str(body.material)]
        stiffness = material.youngs_modulus.typ * float(section.second_moment_z)
        weight = material.density.typ * float(section.area) * gravity
        line = next(
            item for item in result.report.approximations if item.startswith(f"flexible body '{body.name}': damping")
        )
        carried = float(line.split("realised at build time ")[1].split("(carried mass ")[1].split(" kg")[0]) * gravity
        supports = (str(body.flexible.supports[0]), str(body.flexible.supports[1]))
        rails[body.name] = (
            float(section.length),
            stiffness,
            weight,
            carried,
            *G15_SUPPORTS[supports],
            float(section.fibre_y),
            float(section.second_moment_z),
            load * gravity,
        )
    assert sorted(rails) == ["rail_x", "rail_y"]
    return rails


def _g15_record() -> dict[str, tuple[tuple[int, bool, bool | None, bool | None], ...]]:
    """(substeps, flagged, safety factor verdict, peak deflection verdict) per rail from the native baseline of record
    (`baseline.yaml`, regenerated with the gantry data; ruling 99 (b)): a criterion in error is the flagged member's
    (not valid, ruling 69), its verdict None. The case runs 10 substeps; the 20 and 40 substep rows of the gate report
    (brief M3-X1f) were the placeholder rails' and are not carried over to the 2040 section."""
    data = YAML(typ="safe", pure=True).load(G15_BASELINE.read_text(encoding="utf-8"))
    status = {item["criterion"]: item["status"] for item in data["entries"] if item["case"] == "G15"}
    record = {}
    for member in ("rail_x", "rail_y"):
        verdicts: list[bool | None] = []
        for name in (f"safety_factor_{member}", f"deflection_{member}"):
            verdict = None
            if status[name] in ("pass", "fail"):
                verdict = status[name] == "pass"
            verdicts.append(verdict)
        flagged = status[f"deflection_{member}"] == "error"
        record[member] = ((10, flagged, verdicts[0], verdicts[1]),)
    return record


def _g15_oracle(member: str, result: RunResult) -> tuple[float, float]:
    """(static deflection m, static stress Pa) of `member` at mid-travel under its own weight, its carried weight and
    the design load, before the table's factor."""
    span, stiffness, weight, carried, point, distributed, point_moment, distributed_moment, fibre, second, load = (
        _g15_rails(result)[member]
    )
    deflection = weight * span**4 / (distributed * stiffness) + (carried + load) * span**3 / (point * stiffness)
    moment = (carried + load) * span / point_moment + weight * span**2 / distributed_moment
    return deflection, moment * fibre / second


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


def _g15_reproduced(member: str, flagged: bool, verdict: bool | None, column: int) -> None:
    """Ruling 73 (b) against the hand record: this 10-substep run matches its row, and the verdict reproduces at
    every recorded step size with the member unflagged at each (fails while any member is flagged or any verdict
    differs)."""
    rows = _g15_record()[member]
    assert rows[0][0] == 10
    assert (rows[0][1], rows[0][column]) == (flagged, verdict), (member, rows[0])
    for _, recorded_flag, *_ in rows:
        assert not recorded_flag, (member, rows)
    assert len({row[column] for row in rows}) == 1, (member, rows)


@pytest.mark.req("ACC-029")
@pytest.mark.long_run
def test_acc_029(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """Reference design (gantry), G15: under maximum acceleration with rated load and an E-stop with a
    swinging load, every member shall stay above the minimum safety factor.

    Minimum 1.5 (design, gantry team 2026-11-11). Per rail: the table's peak stress from 8 s (the carried design load)
    beside the static oracles, the engine's peak dynamic ratio and flag, and the verdict restated with the stress
    scaled by 2.04 over that ratio (ruling 73 (a): all that no flag bounds; a verdict that flips is not established).
    Asserted: neither rail flagged and each printed dynamic ratio at most 2.04 (the upper oracle as the engine defines
    it, ENG-052); each peak stress at least the continuum static stress less the chain's 2 percent; the case's
    safety-factor criteria pass, also when scaled; ruling 73 (b) against the hand record of 20 and 40 substeps
    (`_g15_reproduced`); no tier 1 crossing; the load case exported beside the archive.

    Converted by brief M3-GV2 (ruling 99 (b)): passes on the sourced 2040 aluminium rails (stiffer and lighter, neither
    rail flagged). The strict expected failure's reason, kept as history:
    placeholder rail section and PETG strength (gantry team): rail_x's safety factor is 1.424, 1.414, 1.280 at 10,
    20, 40 substeps (16.549 s, when the E-stop drops the lift onto its lower limit; 9.616 s at 40) against the 1.5
    minimum (13.71 MPa on the 40 x 40 mm, 2e-8 m^4 placeholder section; the stop raises rail_x's root moment from
    7.0 to 9.6 N m); and rail_y is not valid from 8.31 s at every step size run (dynamic ratio 2.76, 3.63, 4.11 at
    10, 20, 40 substeps, against 2.04: the trolley starts 24 mm from rail_y's support; rulings 69, 78), so its
    safety factor is not evaluated; the 20 and 40 substep verdicts are the gate report's hand record (G15_RECORD),
    which a 10-substep pass alone cannot change
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
        static = _g15_oracle(member, result)[1] * G15_TABLE_FACTOR
        ratio, flagged = flags[member]
        scaled = row.safety_factor * ratio / G15_FLAG_LIMIT
        print(
            f"G15 {member}: safety factor {row.safety_factor!r} at {row.safety_factor_time!r} s (minimum 1.5), peak "
            f"stress {row.peak_stress!r} Pa at {row.peak_stress_time!r} s (continuum static {static!r} Pa); dynamic "
            f"ratio {ratio!r}, flagged {flagged}; scaled by {G15_FLAG_LIMIT} / ratio: safety factor {scaled!r}"
        )
    for member in ("rail_x", "rail_y"):
        row = result.metadata.integrity.member(member)
        static = _g15_oracle(member, result)[1] * G15_TABLE_FACTOR
        ratio, flagged = flags[member]
        criterion = _criterion(result, f"safety_factor_{member}")
        verdict = None
        if criterion.status in ("pass", "fail"):
            verdict = criterion.status == "pass"
        _g15_reproduced(member, flagged, verdict, 2)
        assert not flagged
        assert ratio <= G15_FLAG_LIMIT
        assert criterion.status == "pass"
        assert row.safety_factor * ratio / G15_FLAG_LIMIT >= 1.5
        assert row.peak_stress >= (1.0 - G15_CHAIN) * static
    assert result.score.crossings == ()
    assert result.score.status == "pass"


@pytest.mark.req("ACC-030")
@pytest.mark.long_run
def test_acc_030(native_gantry: dict[str, tuple[RunResult, CoreDriver]]) -> None:
    """Reference design (gantry), G15: under the same worst cases, member deflection shall stay under the
    target.

    Target 1 cm (ACC-030 row), the dynamic node-pose peak from 8 s (the conservative reading). Settled (14 s to 15 s,
    both carriages at mid-travel): the case's own criterion within 2 percent of the closed form (G15 header; the
    table's factor is 1, the case carrying the design load). Peak: the engine's printed dynamic ratio at most 2.04
    with no flag (what no flag bounds, ENG-052: the base is the larger of the windowed quasi-static peak under the
    actual loads and the worst-case static carried deflection, not the mid-travel static value), at least the static
    value less the chain's 2 percent, and under the target, also when scaled by 2.04 over the member's peak dynamic
    ratio (ruling 73 (a)); ruling 73 (b) against the hand record (`_g15_reproduced`).

    Converted by brief M3-GV2 (ruling 99 (b)): passes on the sourced 2040 aluminium rails (stiffer and lighter, neither
    rail flagged). The strict expected failure's reason, kept as history:
    rail_y is not valid from 8.31 s at every step size run (dynamic ratio 2.76, 3.63, 4.11 at 10, 20, 40 substeps
    against 2.04; rulings 69, 78: placeholder rails and belts at the fixed 0.01 damping), so its deflection
    criteria are not evaluated; rail_x passes at every step size (peak 1.134, 1.141, 1.119 mm; scaled by 2.04 over
    its ratio 2.25, 1.91, 1.46 mm; settled 1.0806 mm against 1.0977 mm); the 20 and 40 substep verdicts are the
    gate report's hand record (G15_RECORD), which a 10-substep pass alone cannot change
    """
    result, _ = native_gantry["G15"]
    assert result.metadata is not None
    assert result.metadata.integrity is not None
    flags = _g15_flags(result)
    for member in ("rail_x", "rail_y"):
        static = _g15_oracle(member, result)[0] * G15_TABLE_FACTOR
        settled = _criterion(result, f"settled_deflection_{member}")
        peak = _criterion(result, f"deflection_{member}")
        print(
            f"G15 {member}: settled {settled.measured} m ({settled.status}), peak from 8 s {peak.measured} m at "
            f"{peak.worst_time} s ({peak.status}); static oracle {static!r} m; dynamic ratio, flagged {flags[member]}"
        )
    for member in ("rail_x", "rail_y"):
        static = _g15_oracle(member, result)[0] * G15_TABLE_FACTOR
        ratio, flagged = flags[member]
        peak = _criterion(result, f"deflection_{member}")
        verdict = None
        if peak.status in ("pass", "fail"):
            verdict = peak.status == "pass"
        _g15_reproduced(member, flagged, verdict, 3)
        assert not flagged
        assert ratio <= G15_FLAG_LIMIT
        assert _criterion(result, f"settled_deflection_{member}").status == "pass"
        assert peak.status == "pass"
        assert peak.measured >= (1.0 - G15_CHAIN) * static
        assert peak.measured * G15_FLAG_LIMIT / ratio < 0.01


@pytest.mark.req("INV-014")
@pytest.mark.parametrize("case", ["G12", "G15"])
@pytest.mark.long_run
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


# ===== G16, motor sizing (ACC-031, ACC-032, LIB-017) ========================
G16_CASE = GANTRY_CASES / "G16.yaml"
G16_MOTORS = ("lift_motor", "x_motor", "y_motor")
# Every G16 value from the data (ruling 99 (a), (d)): each motor's belt (the catalog part the design names: ratio c in
# m/rad, efficiency eta, the steady gain n / eta on the native adapter, ruling 66), the firmware's limits
# (`gantry_config.h`: a, the cruise v, the homing speed; omega_c = v / c, alpha = a / c), the axis friction (design
# joints x_axis and y_axis; none on the lift), what hangs below the hook (G16's `payload_mass`, the rope part's mass per
# length over its rest length and its end fitting), each part's rotor inertia, holding torque and published curve.
G16_FIRMWARE = ROOT / "firmware" / "app" / "gantry" / "gantry_config.h"
G16_BELTS = {"lift_motor": "lift_belt", "x_motor": "x_belt", "y_motor": "y_belt"}
G16_G = 9.81


def _g16_define(name: str) -> float:
    text = G16_FIRMWARE.read_text(encoding="utf-8")
    found = re.search(rf"#define {name} (\d+)", text)
    assert found is not None, name
    return float(found.group(1))


G16_A = _g16_define("GANTRY_MAX_ACCEL_UM_S2") * 1e-6
G16_CRUISE = _g16_define("GANTRY_MAX_SPEED_UM_S") * 1e-6
G16_HOMING = _g16_define("GANTRY_HOMING_SPEED_UM_S") * 1e-6


def _g16_design() -> Any:
    return prepare(G16_CASE).design


def _g16_belt(motor: str) -> tuple[float, float]:
    """(c m/rad, eta) of the belt part the design names on `motor`'s transmission."""
    part = next(item.part for item in _g16_design().components if item.name == G16_BELTS[motor])
    values = default_registry().part(part).values
    return float(values["ratio"]), float(values["efficiency"])


def _g16_friction(motor: str) -> float:
    if motor == "lift_motor":
        return 0.0
    joint = next(item for item in _g16_design().joints if item.name == f"{motor.removesuffix('_motor')}_axis")
    return float(joint.linear[0].friction)


def _g16_hanging() -> float:
    case = YAML(typ="safe", pure=True).load(G16_CASE.read_text(encoding="utf-8"))
    payload = next(item["default"] for item in case["open_parameters"] if item["name"] == "payload_mass")
    rope_part = next(item.part for item in _g16_design().ropes if item.name == "payload_rope")
    rope = default_registry().part(rope_part).values
    return float(str(payload).split(" ")[0]) + rope["mass_per_length"] * rope["rest_length"] + rope["end_fitting_mass"]


def _g16_nominal() -> str:
    parts = {item.part for item in _g16_design().components if item.name in G16_MOTORS}
    assert len(parts) == 1, parts
    return parts.pop()


def _g16_swept() -> set[str]:
    case = YAML(typ="safe", pure=True).load(G16_CASE.read_text(encoding="utf-8"))
    return {value for sweep in case["sweeps"] for value in sweep["values"]}


def _g16_curve(part: str) -> tuple[tuple[float, float], ...]:
    curve = default_registry().part(part).torque_speed_curve
    assert curve is not None
    return tuple((float(point.speed), float(point.torque)) for point in curve)


def _g16_available(part: str, motor: str) -> tuple[float, float]:
    """(least torque, at speed rad/s) of the part's published curve over every shaft speed the motor runs, from rest to
    the cruise omega_c = v / c, the homing speed among them: the curve is linear between its points, so the least is at
    a point at or below omega_c or at omega_c itself."""
    c, _ = _g16_belt(motor)
    cruise = G16_CRUISE / c
    curve = _g16_curve(part)
    speeds = [speed for speed, _ in curve if speed <= cruise] + [cruise, G16_HOMING / c]
    values = [(available_torque(curve, speed), speed) for speed in speeds]
    return min(values)


def _g16_rotor(part: str) -> float:
    return float(default_registry().part(part).values["rotor_inertia"])


def _g16_holding(part: str) -> float:
    return float(default_registry().part(part).values["holding_torque"])


# Windows (G16 header timeline): the lift ramps from 6.05 s (after the 50 ms brake release) to 0.1 m/s by 8.05 s
# having moved 0.1 m and cruises until its slow-zone deceleration 0.3 - 0.05 - 0.075 = 0.175 m from home, 8.8 s;
# x ramps from 15 s to 17 s and cruises until 0.4 - 0.125 = 0.275 m, 18.75 s. Each cruise window keeps 0.15 s off
# both ends.
G16_LIFT_CRUISE = (8.2, 8.65)
G16_X_CRUISE = (17.15, 18.6)


def _g16_evaluation_start() -> float:
    """G16's `integrity.evaluate_from`: the sizing criteria judge from the last rest at or before it (rulings 91,
    93)."""
    case = YAML(typ="safe", pure=True).load(G16_CASE.read_text(encoding="utf-8"))
    return float(str(case["integrity"]["evaluate_from"]).split(" ")[0])


G16_WINDOWS = {"lift_motor": (6.0, 15.0), "x_motor": (15.0, 26.0), "y_motor": (15.0, 26.0)}


@pytest.fixture(scope="module")
def g16() -> list[tuple[RunResult, CoreDriver]]:
    """G16 through the CLI's path: the nominal run (NEMA 23 on every axis) and its sweep runs, each substituting the
    NEMA 17 into one motor instance (LIB-017)."""
    services = core_services()
    drivers: list[CoreDriver] = []

    def driver() -> CoreDriver:
        made = CoreDriver()
        drivers.append(made)
        return made

    chosen = Services(driver, services.builder)
    results = [run_case(G16_CASE, chosen, None, None)]
    results.extend(sweep_runs(G16_CASE, chosen, None, None))
    assert len(results) == len(drivers) == 4
    for result in results:
        assert result.report is not None, result.score
        assert not result.report.errored, result.report.error
    return list(zip(results, drivers, strict=True))


def _g16_parts(result: RunResult) -> dict[str, str]:
    parts = dict.fromkeys(G16_MOTORS, _g16_nominal())
    if result.score.sweep is not None:
        parameter, value = result.score.sweep
        parts[parameter.removeprefix("instance:").removesuffix(".part")] = value
    return parts


def _g16_masses(parts: dict[str, str]) -> dict[str, float]:
    """The mass each motor's belt moves, from the built model with the run's parts (a motor's mass sits on its mount
    body): the lift raises the lift carriage, the hook and what hangs below it; y moves the trolley and the lift; x
    moves rail_y with all of that."""
    bindings = {f"instance:{motor}.part": part for motor, part in parts.items()}
    built = core_services().builder(prepare(G16_CASE, bindings).design)
    assert built.physics is not None
    body = {item.name: item.mass for item in built.physics.bodies}
    lift = body["lift_carriage"] + body["hook"] + _g16_hanging()
    y = lift + body["trolley"]
    return {"lift_motor": lift, "y_motor": y, "x_motor": y + body["rail_y"]}


def _g16_torque(driver: CoreDriver, motor: str) -> tuple[Any, Any]:
    recording = driver.recording()
    times = np.asarray(recording.series("joint:lift_axis.position").times, dtype=np.float64)
    values = np.asarray(recording.recording.series[f"signals/actuator_efforts/{motor}.torque"].values)
    return times, values


@pytest.mark.req("ACC-031")
@pytest.mark.req("LIB-017")
@pytest.mark.long_run
def test_acc_031(g16: list[tuple[RunResult, CoreDriver]]) -> None:
    """Reference design (gantry), G16: a sweep of NEMA 17 against NEMA 23 with the rated load shall report
    the torque margin per motor.

    Demand at the commanded speed (G16 header): lift T = m_L (g + a) c / eta + J alpha, x and y T = (m a + F_f) c / eta
    + J alpha, m from the built model with the run's parts; margin = the part's curve at 150 rpm less T, printed per
    motor and run. Assertions, each from the drive's steady state (the drive torque read back is the spring torque
    that carries the load; the stepper's damping acts about the moving rest, so it adds nothing in steady motion):
    - lift cruise: the mean drive torque is m_L g c / eta, within m_L a c / eta (the ramp's step of the load, which an
      undamped spring-mass carries on as a ring of at most that amplitude; a mean over the window only shrinks it);
    - x cruise: the mean is F_f c / eta within m_x a c / eta, the same argument;
    - every motor's peak over its window reaches its cruise demand (the drive carries at least the steady load).
    A motor whose margin is negative by the derivation must show lost steps: the firmware's encoder mismatch fault
    or a failing criterion (the report's missed steps among them). The peak above the demand (the brake hand-over on
    the lift and the step train ringing the lightly damped belts on x and y, ruling 81) is printed, not bounded.

    The report's own margin (`torque_margin_<motor>`, SCO-032: the part's curve at the recorded shaft speed less the
    recorded shaft load torque, its minimum over the span from the last rest at or before 9 s, rulings 91, 93) is
    asserted against this derivation: it names the run's part, passes exactly when positive, is negative when the
    derivation's margin is and positive otherwise (every derived margin here is positive, the NEMA 17 lift's about
    +0.053 N m), and (lift and x, which have a cruise
    window) is at most T(w_min) - (cruise - m a c / eta), T(w_min) the curve at the least shaft speed in the cruise
    window. That bound is one-sided and derived, not fitted: the run's minimum is at most the margin at the cruise
    sample of largest load; that load is at least the window mean, which lies within m a c / eta of the cruise load
    (the assertion above), and the curve there is at most T(w_min) since it falls with speed. Within the window the
    shaft load torque (PHY-033) and the drive torque differ only by the stepper's damping about the moving rest, which
    the mean does not see."""
    seen: set[tuple[str, str]] = set()
    for result, driver in g16:
        parts = _g16_parts(result)
        masses = _g16_masses(parts)
        lost = any(event == "fault:encoder_mismatch" for event in _g16_faults(driver)) or any(
            criterion.status != "pass" for criterion in result.score.criteria
        )
        for motor in G16_MOTORS:
            part = parts[motor]
            mass = masses[motor]
            reported, measured = _g16_reported(result, f"torque_margin_{motor}", part)
            c, eta = _g16_belt(motor)
            friction = _g16_friction(motor)
            alpha_term = _g16_rotor(part) * G16_A / c
            if motor == "lift_motor":
                cruise = mass * G16_G * c / eta
                demand = mass * (G16_G + G16_A) * c / eta + alpha_term
            else:
                cruise = friction * c / eta
                demand = (mass * G16_A + friction) * c / eta + alpha_term
            available, at_speed = _g16_available(part, motor)
            margin = available - demand
            times, torque = _g16_torque(driver, motor)
            start, end = G16_WINDOWS[motor]
            inside = (times >= start) & (times <= end)
            peak = float(np.max(np.abs(torque[inside])))
            print(
                f"G16 sweep {result.score.sweep}: {motor} {part}: moved mass {mass:.4f} kg, demand {demand:.5f} N m, "
                f"least available {available:.4f} N m at {at_speed * 60.0 / (2.0 * math.pi):.1f} rpm, margin "
                f"{margin:+.5f} N m; measured peak "
                f"{peak:.5f} N m ({peak / demand:.3f} of the demand); lost steps shown: {lost}; report: margin "
                f"{measured:+.5f} N m at {reported.worst_time} s ({reported.status})"
            )
            assert (reported.status == "pass") == (measured > 0.0)
            if margin < 0.0:
                assert lost, f"{motor} {part}: margin {margin} N m and no lost step shown"
                assert measured < 0.0
            else:
                assert measured > 0.0, f"{motor} {part}: margin over the span {measured} N m"
            seen.add((motor, part))
            if lost:
                # A run that lost steps stops at the fault: no motor of it cruises, so the cruise checks do not apply.
                continue
            assert peak >= cruise
            if motor == "lift_motor":
                window = (times >= G16_LIFT_CRUISE[0]) & (times <= G16_LIFT_CRUISE[1])
                mean = float(np.mean(torque[window]))
                print(f"  lift cruise mean {mean:.6f} N m against m_L g c / eta = {cruise:.6f}")
                assert abs(mean - cruise) <= mass * G16_A * c / eta
                _g16_margin_bound(driver, motor, part, G16_LIFT_CRUISE, cruise - mass * G16_A * c / eta, measured)
            if motor == "x_motor":
                window = (times >= G16_X_CRUISE[0]) & (times <= G16_X_CRUISE[1])
                mean = float(np.mean(torque[window]))
                print(f"  x cruise mean {mean:.6f} N m against F_f c / eta = {cruise:.6f}")
                assert abs(mean - cruise) <= mass * G16_A * c / eta
                _g16_margin_bound(driver, motor, part, G16_X_CRUISE, cruise - mass * G16_A * c / eta, measured)
    assert seen == {(motor, part) for motor in G16_MOTORS for part in {_g16_nominal(), *_g16_swept()}}


G16_SHAFTS = {"lift_motor": "lift_motor_spin", "x_motor": "x_motor_spin", "y_motor": "y_motor_spin"}


def _g16_reported(result: RunResult, name: str, part: str) -> tuple[CriterionResult, float]:
    """The report's criterion `name`, which must carry a measured value and name the run's part (SCO-031, SCO-032)."""
    found = [criterion for criterion in result.score.criteria if criterion.name == name]
    assert len(found) == 1, name
    assert found[0].measured is not None, found[0].note
    assert found[0].note == f"part {part}"
    return found[0], found[0].measured


def _g16_margin_bound(
    driver: CoreDriver, motor: str, part: str, window: tuple[float, float], least_load: float, measured: float
) -> None:
    view = driver.recording()
    times = np.asarray(view.times(), dtype=np.float64)
    speed = np.asarray(view.series(f"joint:{G16_SHAFTS[motor]}.velocity").values, dtype=np.float64)
    inside = (times >= window[0]) & (times <= window[1])
    curve = default_registry().part(part).torque_speed_curve
    assert curve is not None
    points = tuple((float(point.speed), float(point.torque)) for point in curve)
    bound = available_torque(points, float(np.min(np.abs(speed[inside])))) - least_load
    print(f"  report margin {measured:+.6f} N m against the cruise bound {bound:+.6f} N m")
    assert measured <= bound


def _g16_faults(driver: CoreDriver) -> list[str]:
    return [f"fault:{event.detail}" for event in driver.recording().events() if event.kind == "controller_fault"]


# Missed steps from the record (brief M3-X2c): the driver's step pin carries the microsteps applied each tick and its
# dir pin their sense (step_dir_driver_generic, 16 microsteps per 1.8 degree full step), the shaft joint the rotor.
G16_MICROSTEP = 2.0 * math.pi / 3200.0
G16_FULL_STEP = 2.0 * math.pi / 200.0
G16_DRIVERS = {"lift_motor": "lift_driver", "x_motor": "x_driver", "y_motor": "y_driver"}
G16_FIRST_MOVE = 6.0


def _g16_missed(driver: CoreDriver, motor: str, holding: float) -> tuple[Any, Any]:
    """Missed full steps of `motor` at every sample: commanded angle (the signed sum of the step pin's microsteps)
    less the rotor's travel, in full steps, less the drive's own lead. A pull-out re-syncs the commanded angle to the
    step nearest the rotor (SIG-033), so what is left after the lead is a whole number of steps; the lead is the sine
    law's inverse at the drive torque read back, (2 / pi) asin(tau / T_hold) full steps; the count relative to the
    first sample (SCO-031)."""
    view = driver.recording()
    pin = G16_DRIVERS[motor]
    steps = np.asarray(view.series(f"signal:{pin}.step").values, dtype=np.float64)
    sense = np.where(np.asarray(view.series(f"signal:{pin}.dir").values) > 0.5, 1.0, -1.0)
    rotor = np.asarray(view.series(f"actuator:{motor}.position").values, dtype=np.float64)
    commanded = np.cumsum(steps * sense) * G16_MICROSTEP
    times, torque = _g16_torque(driver, motor)
    lead = 2.0 / math.pi * np.arcsin(np.clip(torque / holding, -1.0, 1.0))
    raw = (commanded - (rotor - rotor[0])) / G16_FULL_STEP - lead
    return times, raw - raw[0]


def _g16_stalls(driver: CoreDriver, motor: str) -> list[float]:
    return [event.time for event in driver.recording().events() if event.source == f"{motor}.stalled"]


def _g16_margin(motor: str, part: str, mass: float) -> float:
    """ACC-031's derived margin: the part's least published torque from rest to the cruise less the ramp's demand,
    lift m (g + a) c / eta + J alpha, x and y (m a + F_f) c / eta + J alpha."""
    c, eta = _g16_belt(motor)
    alpha_term = _g16_rotor(part) * G16_A / c
    if motor == "lift_motor":
        demand = mass * (G16_G + G16_A) * c / eta + alpha_term
    else:
        demand = (mass * G16_A + _g16_friction(motor)) * c / eta + alpha_term
    return _g16_available(part, motor)[0] - demand


@pytest.mark.req("ACC-032")
@pytest.mark.req("SIG-033")
@pytest.mark.long_run
def test_acc_032(g16: list[tuple[RunResult, CoreDriver]]) -> None:
    """Reference design (gantry), G16: the same sweep shall report any missed steps per motor.

    Per motor and run, from the record (`_g16_missed`), at rest before the first move (6 s) and at the end (26 s).
    Each count must be a whole number of steps to within a quarter step (a half step is what identifies it uniquely;
    at rest the remainder is only the drive's predicted lead, which the sine law gives from the recorded torque).
    Derivation of which motor must lose steps: the payload is released at t = 0 on its unstretched rope and snatches;
    the recorded end tension F peaks in the first tens of milliseconds, before the lift has moved. Holding the hook
    against it needs at least (F eta + m_c g) c on the lift shaft (back-driven through the 0.95 belt, the lower of the
    two efficiencies; m_c the lift carriage and hook); a lift motor whose holding torque is below that has a negative
    margin there and must lose steps. Every other motor and part has a positive margin by that bound (the x and y
    belts carry no part of a vertical snatch) and by the cruise margins of ACC-031 (all positive), so it loses none,
    and no motor loses a step from the first move to the end. The tolerance on the count is not set (ACC-032 row): the
    counts are printed with the recorded stall events of each motor.

    The report's own count (`missed_steps_<motor>`, SCO-031, final) judges the span from the last rest at or before
    the 9 s evaluation start (rulings 91, 93; for the lift the rest before its first move, so its move is judged
    whole): it names the run's part, is 0 for every motor and part (every stall event precedes the rest), and equals
    the test's count at the end less its count at that rest within 1e-6 steps. The steps lost before the rest are on
    the report's before-span note, which must give the test's count at the rest and, for a lift motor that must lose
    steps by the bound above, at least one step and a negative least margin before it (the snatch load on the shaft
    exceeds the holding torque, which no point of the curve exceeds). Both are the same
    quantity from the same samples with the same constants (the catalog's 1.8 degree step, 16 microsteps, holding
    torque); they differ only in summation order over at most 26 000 samples of commanded steps, whose rounding is
    below 26 000 x 2.2e-16 relative of a few thousand steps, about 1e-8 steps."""
    seen: set[tuple[str, str]] = set()
    for result, driver in g16:
        parts = _g16_parts(result)
        masses = _g16_masses(parts)
        recording = driver.recording().recording
        tension = np.asarray(recording.series["physics/ropes/tension"].values, dtype=np.float64)
        snatch = float(np.max(tension.reshape(len(tension), -1)[:, 0]))
        carriage = masses["lift_motor"] - _g16_hanging()
        c, eta = _g16_belt("lift_motor")
        bound = (snatch * eta + carriage * G16_G) * c
        for motor in G16_MOTORS:
            part = parts[motor]
            holding = _g16_holding(part)
            times, missed = _g16_missed(driver, motor, holding)
            line = _g16_before_line(result, motor, part)
            rest_time, lost, least, least_time = _g16_before_values(line)
            rest = int(np.argmin(np.abs(times - rest_time)))
            before = float(missed[int(np.searchsorted(times, G16_FIRST_MOVE)) - 1])
            at_rest = float(missed[rest])
            after = float(missed[-1])
            stalls = _g16_stalls(driver, motor)
            print(
                f"G16 sweep {result.score.sweep}: {motor} {part}: missed full steps {before:.4f} before the first "
                f"move, {at_rest:.4f} at the span's rest {rest_time} s, {after:.4f} at the end; stall events at "
                f"{stalls}; release snatch {snatch:.3f} N, lift demand bound {bound:.4f} N m against holding "
                f"{holding} N m"
            )
            print(f"  note: {line}")
            reported, measured = _g16_reported(result, f"missed_steps_{motor}", part)
            print(f"  report: missed steps {measured:.6f} over the span ({reported.status})")
            assert rest_time <= _g16_evaluation_start()
            assert abs(measured - (after - at_rest)) <= 1e-6
            seen.add((motor, part))
            if _g16_margin(motor, part, masses[motor]) < 0.0:
                # Ruling 99 (d): a negative derived margin (ACC-031) loses steps in the move: the report counts at
                # least one in the span and fails, and the record's count is whole up to the first move and rises by
                # at least one step after it (a pull-out while stepping is counted, not a stall event, SIG-033). The
                # drive is disabled at the fault, so the count at the end is not a whole number of steps and is
                # printed, not rounded.
                assert measured >= 1.0
                assert reported.status == "fail"
                assert abs(before - round(before)) <= 0.25
                assert round(before) == round(at_rest)
                assert after - before >= 1.0
                continue
            assert round(measured) == 0
            assert reported.status == "pass"
            assert all(time < rest_time for time in stalls)
            assert abs(before - round(before)) <= 0.25
            assert abs(at_rest - round(at_rest)) <= 0.25
            assert round(before) == round(at_rest)
            # The note prints 6 significant digits: within 5e-6 of a count near 1 (1e-5 allows the rounding).
            assert abs(lost - at_rest) <= 1e-5
            assert least_time < rest_time
            if motor == "lift_motor" and bound > holding:
                assert abs(round(at_rest)) >= 1, f"{motor} {part}: demand bound {bound} N m and no missed step"
                assert least < 0.0
            else:
                assert round(at_rest) == 0
    assert seen == {(motor, part) for motor in G16_MOTORS for part in {_g16_nominal(), *_g16_swept()}}


def _g16_before_line(result: RunResult, motor: str, part: str) -> str:
    """The report's before-span note for `motor` (SCO-031, SCO-032, rulings 91, 93)."""
    head = f"motor '{motor}' (part {part}): before the rest at "
    found = [line for line in result.score.notes if line.startswith(head)]
    assert len(found) == 1, result.score.notes
    return found[0]


def _g16_before_values(line: str) -> tuple[float, float, float, float]:
    """The span's rest time (s), the count there, the least margin before it (N m) and its time (s)."""
    pattern = (
        r"before the rest at (\S+) s \(evaluation start [^)]*\), missed steps (\S+?)[ ,].*"
        r"least torque margin (\S+) N m at (\S+) s;"
    )
    match = re.search(pattern, line)
    assert match is not None, line
    return tuple(float(match.group(index)) for index in range(1, 5))  # type: ignore[return-value]


# ===== In motion, evaluation start 0 (ACC-032, rulings 93, 95) ==============
# The suite's cases judge from the rest at or before 9 s, so G16's and G10's losses (homing, G10's 8.5 s stall) fall
# before the span. Ruling 95: the in-motion demonstration runs test-local copies of the cases evaluated from the
# start, data only; the suite's cases keep their derived start. The record's first sample is at one physics step, 1 ms,
# and the span starts at a rest sample at or before the evaluation start (ruling 93), so `evaluate_from: 0 s` has no
# sample to start from (unevaluable); the start is the first sample, 1 ms, where every motor is still at rest (homing
# applies its first microstep tens of ticks later).
G16_FIRST_SAMPLE = "1 ms"


def _start_at_zero(case: Path, directory: Path) -> Path:
    text = case.read_text()
    stated = f"evaluate_from: {YAML(typ='safe', pure=True).load(text)['integrity']['evaluate_from']}"
    assert text.count(stated) == 1, case
    copied = directory / case.name
    copied.write_text(text.replace(stated, f"evaluate_from: {G16_FIRST_SAMPLE}"))
    return copied


# A motor that must lose steps in motion. Not the lift: a gravity-loaded axis cannot be 20 percent short at cruise and
# still hold at rest (ACC-032 row, ruling 93). The x axis carries no gravity load: a test-local part, the NEMA 17
# entry with its holding torque and every curve point scaled by G16_WEAK_FACTOR, supplied through the registry the run
# is prepared and built with (`core_services(registry=...)`, LIB-016, BLD-027); the shipped catalog is unchanged.
G16_WEAK_PART = "nema17_weak_test"
G16_WEAK_FACTOR = 0.05


def _g16_weak_registry(directory: Path) -> Registry:
    catalog = directory / "catalog"
    shutil.copytree(CATALOG_DIR, catalog)
    yaml = YAML(typ="safe")
    path = catalog / "stepper_behavioral.yaml"
    source = yaml.load(path.read_text())
    found = [item for item in source["parts"] if item["name"] == "nema17_class_generic"]
    assert len(found) == 1
    entry = copy.deepcopy(found[0])
    entry["name"] = G16_WEAK_PART
    entry["values"]["holding_torque"] = f"{_g16_holding('nema17_class_generic') * G16_WEAK_FACTOR!r} N*m"
    for point in entry["torque_speed_curve"]:
        torque = float(point["torque"].split(" ")[0])
        point["torque"] = f"{torque * G16_WEAK_FACTOR!r} N*m"
    source["parts"].append(entry)
    with path.open("w") as handle:
        yaml.dump(source, handle)
    return load_registry(catalog)


def _losses(driver: CoreDriver, motor: str, holding: float) -> list[float]:
    """The times at which `motor`'s whole missed-step count from the record (`_g16_missed`, the commanded angle less
    the rotor's travel less the drive's lead, SCO-031) grows: each pull-out loses whole steps (SIG-033)."""
    times, missed = _g16_missed(driver, motor, holding)
    whole = np.abs(np.round(missed))
    rises = np.nonzero(np.diff(whole) > 0.0)[0] + 1
    return [float(times[index]) for index in rises]


def _stepping(driver: CoreDriver, motor: str, at: float) -> bool:
    """Whether `motor`'s driver applied commanded microsteps in the 5 ms up to `at` (the axis is commanded to move:
    homing at 0.02 m/s is 1600 microsteps per second, more than one per 1 ms tick)."""
    view = driver.recording()
    times = np.asarray(view.times(), dtype=np.float64)
    steps = np.asarray(view.series(f"signal:{G16_DRIVERS[motor]}.step").values, dtype=np.float64)
    inside = (times >= at - 0.005) & (times <= at)
    return bool(np.any(steps[inside] != 0.0))


@pytest.mark.req("ACC-032")
@pytest.mark.req("SCO-031")
@pytest.mark.req("SCO-032")
@pytest.mark.req("SIG-033")
@pytest.mark.long_run
def test_acc_032_weak_x_motor_loses_steps_in_motion(tmp_path: Path) -> None:
    """G16 nominal, evaluated from the first sample (test-local copy, ruling 95), with the x motor replaced by the NEMA
    17 scaled by G16_WEAK_FACTOR = 0.05 (holding 0.02 N m, the curve 0.0198 N m at 30 rpm, 0.019 N m at 150 rpm):
    the x motor loses steps while it is commanded to move, and the report says so.

    Derivation (ACC-031's demand, against the scaled curve): x demands T = (m_x a + F_f) c / eta + J alpha, m_x from
    the built model, F_f = 5 N, c = 0.04 / 2 pi m, eta = 0.95, J = 5.4e-6 kg m^2, alpha = a / c. In homing at the
    constant 0.02 m/s (omega = 0.02 / c = 3.14 rad/s, 30 rpm) the demand is at least F_f c / eta = 0.0335 N m
    against 0.0198 N m available, 41 percent short; at the 0.1 m/s cruise (a = 0.05 m/s^2 on the ramps) at least
    that plus m_x a c / eta against 0.019 N m, more than the 20 percent short ruling 91 asks (asserted). At rest the
    axis needs no torque (no gravity load; Coulomb friction only resists motion), so the hold is ample and every
    loss is in motion. Asserted: both shortfalls; the whole missed-step count from the record (`_g16_missed`) rising
    at least once, each rise and any stall event while the x driver is stepping (SIG-033 raises `stalled`, the stall
    event, only on a tick with no commanded microstep, so a pull-out while stepping is counted, not flagged; the
    events are printed); the report's missed steps over the span from the first sample at least one whole step (a
    count within 0.25 of a whole one is the report's own criterion; at least one means abs(count) above 0.75) and
    failing, its least margin negative at a time the driver is stepping and failing, both naming the part."""
    case = _start_at_zero(G16_CASE, tmp_path)
    registry = _g16_weak_registry(tmp_path)
    drivers: list[CoreDriver] = []
    services = core_services(registry=registry)

    def driver() -> CoreDriver:
        made = CoreDriver()
        drivers.append(made)
        return made

    chosen = Services(driver, services.builder, registry=registry)
    result = run_case(case, chosen, {"instance:x_motor.part": G16_WEAK_PART}, None)
    assert result.report is not None
    assert not result.report.errored, result.report.error
    mass = _g16_masses(dict.fromkeys(G16_MOTORS, _g16_nominal()))["x_motor"]
    c, eta = _g16_belt("x_motor")
    friction = _g16_friction("x_motor")
    curve = registry.part(G16_WEAK_PART).torque_speed_curve
    assert curve is not None
    points = tuple((float(point.speed), float(point.torque)) for point in curve)
    homing_demand = friction * c / eta
    homing_available = available_torque(points, G16_HOMING / c)
    cruise_demand = (mass * G16_A + friction) * c / eta + _g16_rotor("nema17_class_generic") * G16_A / c
    cruise_available = available_torque(_g16_curve("nema17_class_generic"), G16_CRUISE / c) * G16_WEAK_FACTOR
    print(
        f"weak x motor: moved mass {mass:.4f} kg; homing demand {homing_demand:.5f} N m against {homing_available:.5f}"
        f" N m; cruise demand {cruise_demand:.5f} N m against {cruise_available:.5f} N m"
    )
    assert homing_demand > homing_available
    assert cruise_demand >= 1.2 * cruise_available
    stalls = _g16_stalls(drivers[0], "x_motor")
    losses = _losses(drivers[0], "x_motor", _g16_holding("nema17_class_generic") * G16_WEAK_FACTOR)
    print(f"  stall events {len(stalls)}: first {stalls[:5]}, last {stalls[-3:]}")
    print(f"  stepper lost-step rises {len(losses)}: first {losses[:5]}, last {losses[-3:]}")
    for line in result.score.notes:
        print(f"  note: {line}")
    missed, missed_value = _g16_reported(result, "missed_steps_x_motor", G16_WEAK_PART)
    margin, margin_value = _g16_reported(result, "torque_margin_x_motor", G16_WEAK_PART)
    print(f"  report: {missed.name} {missed_value:.6f} ({missed.status}, {missed.note})")
    print(f"  report: {margin.name} {margin_value:+.6f} N m at {margin.worst_time} s ({margin.status}, {margin.note})")
    assert len(losses) >= 1
    for loss in losses:
        assert _stepping(drivers[0], "x_motor", loss), loss
    for stall in stalls:
        assert _stepping(drivers[0], "x_motor", stall), stall
    assert abs(missed_value) > 0.75
    assert missed.status == "fail"
    assert margin_value < 0.0
    assert margin.status == "fail"
    assert margin.worst_time is not None
    assert _stepping(drivers[0], "x_motor", margin.worst_time)


@pytest.mark.req("ACC-032")
@pytest.mark.req("SCO-031")
@pytest.mark.long_run
def test_acc_032_g10_stalled_motor_loses_steps_in_motion(tmp_path: Path) -> None:
    """G10, evaluated from the first sample (test-local copy, ruling 95): its x motor is stalled for 0.5 s from 8.5 s
    during its move (TST-016); with no available torque the rotor cannot follow its commanded steps, so the report's
    count for the stalled motor over the span from the first sample is at least one whole step (abs(count) above
    0.75) and fails. Scored from the copy's run with one added criterion on `missed_steps:x_motor` (final 0 within
    0.25, ACC-032's form)."""
    case = _start_at_zero(GANTRY_CASES / "G10.yaml", tmp_path)
    drivers: list[CoreDriver] = []
    services = core_services()

    def driver() -> CoreDriver:
        made = CoreDriver()
        drivers.append(made)
        return made

    result = run_case(case, Services(driver, services.builder), None, None)
    assert result.report is not None
    assert not result.report.errored, result.report.error
    prepared = prepare(case, {})
    extra = {
        "name": "missed_steps_x_motor",
        "check": "value",
        "quantity": "missed_steps:x_motor",
        "statistic": "final",
        "comparison": "within",
        "target": "0",
        "limit": "0.25",
    }
    scored = prepared.case.model_copy(update={"criteria": [ValueCriterion.model_validate(extra)]})
    drives = stepper_drives(core_services().builder(prepared.design))
    score = score_run(scored, drivers[0].recording(), result.report, stepper_drives=drives)
    reported = score.criteria[0]
    stalls = _g16_stalls(drivers[0], "x_motor")
    print(f"G10 from the first sample: x motor stall events {len(stalls)}: first {stalls[:3]}, last {stalls[-3:]}")
    print(f"  report: missed steps {reported.measured} ({reported.status}, {reported.note}); notes {score.notes}")
    assert reported.measured is not None, reported.note
    assert abs(reported.measured) > 0.75
    assert reported.status == "fail"


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
@pytest.mark.req("TST-024")
@pytest.mark.long_run
def test_acc_036(tmp_path: Path) -> None:
    """Acceptance criterion 2: the simulator shall run G1 to G16 in one batch and produce the pass/fail
    report.

    One `mechsim suite` call on `test_cases/gantry/suite.yaml` (native, the default adapter). The cases and the run
    count come from the suite file and the cases' own sweeps and variations, not from a list in the test (brief
    M3-X2d): the printed report and the written Markdown report each carry every case with its verdict and every
    criterion of it, and both count every run (one nominal run per case, one per sweep value, `count` per
    variations block). Verdicts are the cases' own (G12 and G15 are recorded, not passed, rulings 64, 81); this test
    checks the report, not the verdicts."""
    suite = load_suite_file(GANTRY_CASES / "suite.yaml")
    cases = [prepare(GANTRY_CASES / name).case for name in suite.cases]
    runs = 0
    for case in cases:
        runs += 1 + sum(len(sweep.values) for sweep in case.sweeps)
        if case.variations is not None:
            runs += case.variations.count
    stdout = io.StringIO()
    started = time.monotonic()
    code = main(
        ["suite", str(GANTRY_CASES / "suite.yaml"), "--out", str(tmp_path), "--report", str(tmp_path / "report.md")],
        default_services(),
        stdout,
        io.StringIO(),
    )
    print(f"batch of {len(cases)} cases, {runs} runs, on native: {time.monotonic() - started:.0f} s, exit {code}")
    text = stdout.getvalue()
    written = (tmp_path / "report.md").read_text(encoding="utf-8")
    assert len(cases) == len(suite.cases)
    for case in cases:
        assert re.search(rf"^Case {case.id}: (PASS|FAIL|ERROR)$", text, re.MULTILINE) is not None, case.id
        assert re.search(rf"^## Case {case.id}: (PASS|FAIL|ERROR)$", written, re.MULTILINE) is not None, case.id
        for criterion in case.criteria:
            name = re.escape(criterion.name)
            assert re.search(rf"^  (pass|fail|error) +{name}:", text, re.MULTILINE), (case.id, criterion.name)
            assert re.search(rf"^\| {name} \| (pass|fail|error) \|", written, re.MULTILINE), (case.id, criterion.name)
    assert re.search(rf"^{runs} runs:", text, re.MULTILINE) is not None
    assert re.search(rf"^- Runs: {runs}:", written, re.MULTILINE) is not None


@pytest.mark.req("ACC-037")
@pytest.mark.long_run
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
@pytest.mark.xfail(
    strict=True,
    reason="not convertible (brief M3-X2c): the shipped parts' torque_speed_curve points are placeholders with no "
    "datasheet assigned (stepper_behavioral.yaml: nema17_class_generic, nema23_class_generic), so there is no "
    "published curve to match; and the behavioral stepper takes that table as its pull-out limit (SIG-033: pull-out "
    "when the drive torque exceeds table(abs(omega)) or the lead exceeds two step angles), so a measured pull-out "
    "torque reads the input table back and cannot test it against an independent curve; the only design with a "
    "stepper is the gantry, which has no case that loads a shaft at constant speed to pull-out. Owed: a datasheet "
    "curve (owner) and a measurement independent of the table (the electrical tier, phase current against back EMF)",
)
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
@pytest.mark.long_run
def test_phy_016(bridge_gantry: dict[str, RunResult]) -> None:
    """The MuJoCo adapter shall carry the milestone 2 acceptance cases, G5 to G11, G13 and G14.

    Each case runs on the MuJoCo bridge (`physics.adapter: mujoco`; the native adapter is the default since M3-X2)
    through the whole stack without a run error and every criterion of it passes (the per-requirement tests above
    assert what each criterion means, on the default adapter).
    """
    for case in ("G5", "G6", "G7", "G8", "G9", "G10", "G11", "G13", "G14"):
        result = bridge_gantry[case]
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
