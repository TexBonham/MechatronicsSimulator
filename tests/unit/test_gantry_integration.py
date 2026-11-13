"""Gantry end to end, the parts below acceptance: sensor placement and open-parameter defaults in the design (DES-043,
DES-044), their resolution (TST-039) and build (BLD-035), the generated channel header (FW-031), scheduled command and
fault delivery (KER-023), operator labels in the record (REC-009), and run-report errors (TST-012)."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from pydantic import ValidationError

from mechsim.design.schema import Design, cross_check, shipped_materials
from mechsim.library.registry import default_registry
from mechsim.record.archive import RunArchive
from mechsim.testing import core_wiring
from mechsim.testing.core_driver import CoreDriver, RecordArchiveReader, core_builder, core_services
from mechsim.testing.runner import RunnerError, Services, prepare, resolve_open_values, run_case
from mechsim.testing.schema import ROOT

GANTRY = ROOT / "designs" / "gantry" / "design.yaml"
G3 = ROOT / "test_cases" / "gantry" / "G3.yaml"
PENDULUM = ROOT / "test_cases" / "pendulum" / "P1_period.yaml"


def _design_data() -> dict[str, Any]:
    from mechsim.design.schema import yaml_loader

    data: dict[str, Any] = yaml_loader().load(GANTRY.read_text(encoding="utf-8"))
    return data


def _errors(data: dict[str, Any]) -> list[str]:
    try:
        design = Design.model_validate(data, context={"base": GANTRY.parent})
    except ValidationError as error:
        return [str(error)]
    return cross_check(design, default_registry(), shipped_materials())


def _component(data: dict[str, Any], name: str) -> dict[str, Any]:
    for component in data["components"]:
        if component["name"] == name:
            return dict(component)
    raise KeyError(name)


def _replace(data: dict[str, Any], name: str, component: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(data)
    components = []
    for item in out["components"]:
        if item["name"] == name:
            components.append(component)
        else:
            components.append(item)
    out["components"] = components
    return out


# ===== Sensor placement (DES-043) ======================================


@pytest.mark.req("DES-043")
def test_shipped_gantry_places_every_sensor() -> None:
    data = _design_data()
    assert _errors(data) == []
    switch = _component(data, "x_min_switch")["senses"]
    assert switch == {"joint": "x_axis", "position": "0 m", "actuate_above": False}
    assert _component(data, "x_encoder")["senses"] == {"joint": "x_axis"}


@pytest.mark.req("DES-043")
@pytest.mark.parametrize(
    ("name", "senses", "message"),
    [
        ("x_min_switch", None, "must name the joint it senses"),
        ("x_min_switch", {"joint": "nowhere", "position": "0 m", "actuate_above": False}, "unknown joint 'nowhere'"),
        ("x_min_switch", {"joint": "x_axis", "actuate_above": False}, "senses.position: required"),
        ("x_min_switch", {"joint": "x_axis", "position": "0 m"}, "senses.actuate_above: required"),
        ("x_min_switch", {"joint": "x_axis", "position": "0 m", "actuate_above": True, "reversed": True}, "takes no"),
        ("x_encoder", {"joint": "x_motor_spin", "position": "0 m"}, "takes no position"),
        ("x_motor", {"joint": "x_motor_spin"}, "only sensor instances carry senses"),
    ],
)
def test_senses_rules(name: str, senses: dict[str, Any] | None, message: str) -> None:
    data = _design_data()
    component = _component(data, name)
    component.pop("senses", None)
    if senses is not None:
        component["senses"] = senses
    errors = _errors(_replace(data, name, component))
    assert any(message in error for error in errors), errors


@pytest.mark.req("DES-043")
def test_senses_joint_may_be_open() -> None:
    data = _design_data()
    component = _component(data, "x_encoder")
    component["senses"] = {"joint": "${encoder_placement}"}
    assert _errors(_replace(data, "x_encoder", component)) == []


@pytest.mark.req("DES-043")
def test_encoder_reversed_is_accepted() -> None:
    data = _design_data()
    component = _component(data, "x_encoder")
    component["senses"] = {"joint": "x_motor_spin", "reversed": True}
    assert _errors(_replace(data, "x_encoder", component)) == []


# ===== Open-parameter defaults (DES-044, TST-039) ======================


@pytest.mark.req("DES-044")
@pytest.mark.parametrize(
    ("dimension", "default", "message"),
    [
        ("[length]", "3 kg", "default of 'p'"),
        ("[length]", "${other}", "may not be an open value"),
        ("[length]", "", "is empty"),
        ("[length]", {"source": "hand"}, "a mapping is only a choice value"),
    ],
)
def test_default_must_fit_its_dimension(dimension: str, default: Any, message: str) -> None:
    data = _design_data()
    data["open_parameters"].append(
        {"name": "p", "dimension": dimension, "owed_by": "x", "note": "x", "default": default}
    )
    with pytest.raises(ValueError, match=message.replace("$", r"\$").replace("{", r"\{")):
        Design.model_validate(data, context={"base": GANTRY.parent})


@pytest.mark.req("DES-044")
def test_every_gantry_open_parameter_has_a_default() -> None:
    data = _design_data()
    assert all("default" in parameter for parameter in data["open_parameters"])


@pytest.mark.req("DES-044")
@pytest.mark.req("TST-039")
def test_resolution_order_set_then_case_then_default() -> None:
    from mechsim.design.schema import OpenParameter

    declared = {
        name: OpenParameter(name=name, dimension="[length]", owed_by="x", note="x", default="1 m")
        for name in ("a", "b", "c")
    }
    declared["d"] = OpenParameter(name="d", dimension="choice", owed_by="x", note="x")
    values = resolve_open_values(declared, {"a": "2 m", "b": "2 m"}, {"a": "3 m"})
    assert values == {"a": "3 m", "b": "2 m", "c": "1 m"}


@pytest.mark.req("TST-039")
def test_case_parameters_and_set_bind_design_values() -> None:
    prepared = prepare(G3, {"workspace_x": "0.9 m", "encoder_placement": "rail_x"})
    bindings = dict(prepared.bindings)
    assert bindings["workspace_x"] == "0.9 m"
    assert bindings["encoder_placement"] == "rail_x"
    assert bindings["start_x"] == "0.1 m"
    assert prepared.case.criteria[2].limit.value == pytest.approx(0.9)


@pytest.mark.req("TST-039")
def test_unknown_or_ill_dimensioned_parameters_are_errors(tmp_path: Path) -> None:
    with pytest.raises(RunnerError, match="binding 'nope'"):
        prepare(G3, {"nope": "1 m"})
    with pytest.raises(RunnerError, match="binding 'workspace_x'"):
        prepare(G3, {"workspace_x": "1 kg"})
    text = G3.read_text(encoding="utf-8").replace("seed: 3\n", "seed: 3\nparameters:\n  workspace_x: 2 s\n")
    case = tmp_path / "G3.yaml"
    case.write_text(text, encoding="utf-8")
    with pytest.raises(RunnerError, match="parameters 'workspace_x'"):
        prepare(case)


@pytest.mark.req("TST-039")
def test_parameter_without_any_value_is_an_error(tmp_path: Path) -> None:
    text = G3.read_text(encoding="utf-8").replace('    default: "25 s"\n', "")
    case = tmp_path / "G3.yaml"
    case.write_text(text, encoding="utf-8")
    with pytest.raises(RunnerError, match=r"unresolved \$\{run_time\}"):
        prepare(case)


# ===== Build (BLD-035) and channel header (FW-031) =====================


@pytest.mark.req("BLD-035")
def test_builder_maps_senses_and_torque_curve() -> None:
    build = core_builder(prepare(G3).design)
    instances = {instance.name: instance for instance in build.wiring.instances}
    switch = instances["x_max_switch"].senses
    assert switch is not None
    assert (switch.joint, switch.position, switch.actuate_above, switch.reversed) == ("x_axis", 0.5, True, False)
    encoder = instances["y_encoder"].senses
    assert encoder is not None
    assert (encoder.joint, encoder.position, encoder.reversed) == ("y_axis", None, False)
    assert instances["x_motor"].senses is None
    curve = instances["x_motor"].torque_speed
    assert curve[0] == (0.0, pytest.approx(1.26))
    assert curve[1][0] == pytest.approx(300.0 * 2.0 * np.pi / 60.0)


@pytest.mark.req("BLD-035")
def test_switch_position_reads_along_the_joint_from_its_neutral_placement() -> None:
    """The layer reads the child body origin in the parent frame: placement (0, 0.1, 0.04) plus q along +x (rail_x's
    frame starts at the rail's root end, M3-X1a), so a switch at joint coordinate q sits at q in the reading (DES-043
    positions are joint coordinates)."""
    build = core_builder(prepare(G3).design)
    assert build.physics is not None
    places, dofs = core_wiring.joint_places(build.physics)
    instances = {instance.name: instance for instance in build.wiring.instances}
    params = core_wiring.component_parameters(instances["x_max_switch"], places, build.physics)
    assert params["position"] == pytest.approx(0.5)
    assert params["actuate_above"] is True
    assert places["x_axis"].offset == pytest.approx(0.0)
    assert dofs == sum(1 for name in ("x_axis", "y_axis", "lift_axis")) + 3 + 3


@pytest.mark.req("FW-031")
def test_committed_channel_header_is_the_builder_output() -> None:
    build = core_builder(prepare(G3).design)
    assert build.chip is not None
    committed = (ROOT / "firmware" / "app" / "gantry" / "channels.h").read_text(encoding="utf-8")
    assert committed == build.chip.channels_header


# ===== Delivery (KER-023), labels (REC-009), report errors =============

DELIVERY = """id: K1
title: Delivery of scheduled commands and faults
design:
  name: gantry
  version: 0.1.0
controller_mode: native
seed: 11
initial_state:
  joints:
    - joint: x_axis
      position: 0.1 m
      velocity: 0 m/s
    - joint: y_axis
      position: 0.1 m
      velocity: 0 m/s
    - joint: lift_axis
      position: 0.1 m
      velocity: 0 m/s
commands:
  - at: 0.1005 s
    message: target
    payload:
      x: 40 cm
      y: 25 cm
faults:
  - kind: sensor_forced
    target: x_min_switch
    level: high
    start: 0.5 s
    duration: 0.2 s
  - kind: estop_press
    target: estop
    start: 1 s
    duration: 0.5 s
end:
  at: 2 s
  max_duration: 2 s
criteria:
  - name: refusal
    check: event
    event: message:status
    expect: present
"""


def delivery_case(directory: Path) -> Path:
    path = directory / "K1.yaml"
    path.write_text(DELIVERY, encoding="utf-8")
    return path


@pytest.fixture(scope="module")
def delivered(tmp_path_factory: pytest.TempPathFactory) -> tuple[CoreDriver, Any, Path]:
    directory = tmp_path_factory.mktemp("delivery")
    driver = CoreDriver()
    services = core_services()
    result = run_case(
        delivery_case(directory),
        Services(lambda: driver, services.builder, services.archiver, services.designs_root),
        None,
        directory,
    )
    assert result.report is not None
    assert not result.report.errored, result.report.error
    assert result.archive is not None
    return driver, result, result.archive


def _changes(driver: CoreDriver, reference: str) -> list[tuple[float, float]]:
    series = driver.recording().series(reference)
    values = np.asarray(series.values)
    times = np.asarray(series.times)
    return [(round(float(times[i + 1]), 9), float(values[i + 1])) for i in np.nonzero(np.diff(values))[0]]


@pytest.mark.req("KER-023")
def test_forced_switch_level_holds_for_its_window(delivered: tuple[CoreDriver, Any, Path]) -> None:
    """Forced high for 0.5 s <= t < 0.7 s; rows are stamped at the end of their step (KER-009), so the 1 kHz layer's
    step at tick 500 shows at 0.501 s and the first step past the window (tick 700) at 0.701 s. The switch is far from
    its position (x = 0.1 m), so its level is low outside the window."""
    driver, _, _ = delivered
    assert _changes(driver, "signal:x_min_switch.out") == [(0.501, 1.0), (0.701, 0.0)]


@pytest.mark.req("KER-023")
def test_estop_press_opens_motor_power_within_the_cutoff(delivered: tuple[CoreDriver, Any, Path]) -> None:
    """Pressed at 1 s: the relay opens floor(0.02 s / 1 ms) = 20 samples after the press is seen at tick 1000, so the
    rail is off from the row of tick 1020, stamped 1.021 s. The gantry relay runs `manual_reset: false` (round 4
    ruling 25): the loop closes at the release, tick 1500, and the contacts close in that step, stamped 1.501 s."""
    driver, _, _ = delivered
    assert _changes(driver, "signal:motor_power.on") == [(1.021, 0.0), (1.501, 1.0)]


@pytest.mark.req("KER-023")
@pytest.mark.req("TST-013")
def test_command_reaches_the_link_at_the_first_step_at_or_after_its_time(
    delivered: tuple[CoreDriver, Any, Path],
) -> None:
    """0.1005 s rounds up to tick 101 (TST timing); the runner party posts it at its step at tick 101, and the layer
    stamps the event with that step's start time, 0.101 s."""
    driver, _, _ = delivered
    commands = [event for event in driver.recording().events() if event.detail == "target"]
    assert [event.time for event in commands] == [pytest.approx(0.101)]


@pytest.mark.req("REC-009")
def test_operator_labels_recorded_verbatim(delivered: tuple[CoreDriver, Any, Path]) -> None:
    _, _, archive = delivered
    with RunArchive(archive) as opened:
        events = [(event.kind, event.detail) for _, event in opened.recording().events]
    assert ("operator_command", "target (x, y) x=0.4 y=0.25") in events
    assert ("operator_status", "ready / fault state=1 fault=1") in events
    _, view = RecordArchiveReader().open(archive)
    names = [(event.kind, event.detail) for event in view.events() if event.kind == "operator_message"]
    assert names[:2] == [("operator_message", "target"), ("operator_message", "status")]


@pytest.mark.req("TST-012")
def test_raising_on_fault_hook_is_reported(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    def boom(self: Any, outputs: Any) -> None:
        raise ValueError("write failed")

    def bad_hook(self: Any, fault: Any) -> None:
        raise RuntimeError("hook failed")

    monkeypatch.setattr(core_wiring.RunnerParty, "write_outputs", boom)
    monkeypatch.setattr(core_wiring.RunnerState, "note_fault", bad_hook)
    driver = CoreDriver()
    result = run_case(delivery_case(tmp_path), Services(lambda: driver, core_services().builder), None, None)
    assert result.report is not None
    assert result.report.errored
    assert result.report.error is not None
    assert "write failed" in result.report.error
    assert result.report.hook_errors == ("runner.on_fault: RuntimeError: hook failed",)


@pytest.mark.req("TST-012")
@pytest.mark.req("REC-019")
def test_sink_errors_reach_the_report_and_the_archive(tmp_path: Path) -> None:
    def subscriber(row: Any) -> None:
        raise ValueError("subscriber down")

    driver = CoreDriver([subscriber])
    services = core_services()
    result = run_case(
        PENDULUM, Services(lambda: driver, services.builder, services.archiver, services.designs_root), None, tmp_path
    )
    assert result.report is not None
    assert result.report.errored
    assert len(result.report.sink_errors) == 1
    assert "subscriber down" in result.report.sink_errors[0]
    assert result.archive is not None
    metadata, _ = RecordArchiveReader().open(result.archive)
    assert metadata.report.sink_errors == result.report.sink_errors


# ===== Firmware code tables (DES-045) ==================================


@pytest.mark.req("DES-045")
def test_gantry_names_its_state_and_fault_codes() -> None:
    design = prepare(G3).design
    assert design.firmware is not None
    assert design.firmware.state_name(1) == "not_homed"
    assert design.firmware.fault_name(2) == "outside_workspace"
    assert design.firmware.state_name(9) is None


@pytest.mark.req("DES-045")
@pytest.mark.parametrize(
    ("table", "message"),
    [
        ({"x": "a"}, "is not an integer 0..255"),
        ({"256": "a"}, "is not an integer"),
        ({"1": "a", "2": "a"}, "duplicate"),
    ],
)
def test_code_tables_reject_bad_codes_and_repeated_names(table: dict[str, str], message: str) -> None:
    data = _design_data()
    data["firmware"]["state_codes"] = table
    with pytest.raises(ValueError, match=message):
        Design.model_validate(data, context={"base": GANTRY.parent})


@pytest.mark.req("DES-045")
@pytest.mark.req("TST-039")
def test_precondition_names_a_declared_state(tmp_path: Path) -> None:
    for old, new, message in (
        ("state: not_homed", "state: flying", "'flying' is not in firmware.state_codes"),
        ("state: not_homed", "homed: false", "reports only state and fault"),
    ):
        case = tmp_path / "G3.yaml"
        case.write_text(G3.read_text(encoding="utf-8").replace(old, new), encoding="utf-8")
        with pytest.raises(RunnerError, match=message):
            prepare(case)


@pytest.mark.req("DES-045")
@pytest.mark.req("TST-012")
def test_power_on_state_is_reported_by_name() -> None:
    from fractions import Fraction

    from mechsim.testing.driver import RunConfig

    prepared = prepare(G3)
    driver = CoreDriver()
    driver.load(core_builder(prepared.design), prepared.case)
    driver.configure(RunConfig(Fraction(1000), Fraction(1000), Fraction(1000), 0, (), "native"))
    assert driver.reported_controller_state() == {"state": "not_homed", "fault": "none"}


@pytest.mark.req("CTL-007")
def test_host_build_dir_is_keyed_by_firmware_tree(tmp_path: Path) -> None:
    """Two checkouts of the firmware tree never share a CMake build directory, and one tree always gets the same."""
    first = tmp_path / "a" / "firmware"
    second = tmp_path / "b" / "firmware"
    first.mkdir(parents=True)
    second.mkdir(parents=True)
    one = core_wiring.host_build_dir(first, 1000, "mechsim_fw_gantry")
    assert one == core_wiring.host_build_dir(first, 1000, "mechsim_fw_gantry")
    assert one != core_wiring.host_build_dir(second, 1000, "mechsim_fw_gantry")
    assert one != core_wiring.host_build_dir(first, 500, "mechsim_fw_gantry")
    assert one.parent == core_wiring.NATIVE_BUILD_DIR
