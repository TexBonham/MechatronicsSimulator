"""Test runner (TST) against the scripted in-memory fake driver."""

from __future__ import annotations

import io
import json
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mechsim.builder import build
from mechsim.design import schema as design_schema
from mechsim.design.schema import shipped_materials, yaml_loader
from mechsim.integrity.table import IntegrityReport
from mechsim.library.registry import Registry, default_registry, load_registry
from mechsim.library.schema import CATALOG_DIR
from mechsim.testing.baseline import load_baseline
from mechsim.testing.driver import MESSAGE_KIND, IntegrityResults, RecordedEvent
from mechsim.testing.runner import (
    FAULT_OWNERS,
    RunnerError,
    Services,
    case_files,
    command_timeline,
    draw_variation,
    end_spec,
    exact_tick,
    fault_timeline,
    get_design_path,
    module_seeds,
    next_tick,
    prepare,
    rational_lcm,
    run_case,
    run_suite,
    run_variation,
    set_design_path,
    sweep_runs,
    variation_runs,
    variation_seed,
)
from mechsim.testing.schema import Case, SuiteFile
from tests.unit.test_runner_fakes import (
    MACHINE_CASE,
    FakeArchiver,
    FakeBuilder,
    InMemoryFakeDriver,
    Row,
    unrun_baseline,
    write_case,
    write_designs,
)

BIND = {"link_mass": "1 kg"}
ANGLE = "joint:pivot.position"


def steady(n: int, t: float) -> Row:
    return Row({ANGLE: 0.05})


def machine(tmp_path: Path, text: str = MACHINE_CASE) -> tuple[Path, Path]:
    designs = write_designs(tmp_path)
    return designs, write_case(tmp_path / "cases", text)


def services(designs: Path, driver: InMemoryFakeDriver, builder: FakeBuilder | None = None, **extra: Any) -> Services:
    return Services(lambda: driver, builder or FakeBuilder(), designs_root=designs, **extra)


def fake(**options: Any) -> InMemoryFakeDriver:
    options.setdefault("script", steady)
    options.setdefault("units", {ANGLE: "rad"})
    options.setdefault("reported", {"homed": False})
    return InMemoryFakeDriver(**options)


def minimal_case(**fields: Any) -> Case:
    data: dict[str, Any] = {
        "id": "X1",
        "title": "x",
        "design": {"name": "machine", "version": "0.1.0"},
        "seed": 0,
        "initial_state": {},
        "end": {"at": "1 s", "max_duration": "1 s"},
        "criteria": [
            {"name": "c", "check": "event", "event": "message:home", "expect": "present"},
        ],
    }
    data.update(fields)
    return Case.model_validate(data)


# ===== Loading, design version, resolution ==============================


@pytest.mark.req("TST-011")
def test_a_design_version_mismatch_is_an_error_before_the_build(tmp_path: Path) -> None:
    designs, case = machine(tmp_path, MACHINE_CASE.replace("version: 0.1.0", "version: 0.2.0"))
    builder = FakeBuilder()
    driver = fake()
    result = run_case(case, services(designs, driver, builder), BIND)
    assert result.score.status == "error"
    assert not result.score.ran
    assert "design.version: case was written against 0.2.0, design is 0.1.0" in str(result.score.error)
    assert builder.built == []
    assert driver.calls == []


@pytest.mark.req("TST-011")
def test_the_builder_builds_the_resolved_design_the_case_names(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    builder = FakeBuilder()
    run_case(case, services(designs, fake(), builder), BIND)
    assert len(builder.built) == 1
    assert builder.built[0].name == "machine"
    assert builder.built[0].bodies[0].mass_properties.mass == 1.0


@pytest.mark.req("TST-009")
def test_an_invalid_case_file_is_rejected_naming_the_field(tmp_path: Path) -> None:
    designs, case = machine(tmp_path, MACHINE_CASE.replace("      angle: 0.25 rad", "      angle: 0.25"))
    with pytest.raises(RunnerError, match=r"commands\.1\.payload\.angle"):
        prepare(case, BIND, designs)


def test_an_unresolved_open_value_is_an_error_naming_its_path(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    with pytest.raises(RunnerError, match=r"design.bodies\[0\].mass_properties.mass: unresolved \$\{link_mass\}"):
        prepare(case, {}, designs)
    result = run_case(case, services(designs, fake()), {})
    assert result.score.status == "error"
    assert "unresolved ${link_mass}" in str(result.score.error)


def test_a_binding_must_name_an_open_parameter_and_match_its_dimension(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    with pytest.raises(RunnerError, match="binding 'nothing': not an open parameter"):
        prepare(case, {**BIND, "nothing": "1 m"}, designs)
    with pytest.raises(RunnerError, match="binding 'link_mass'"):
        prepare(case, {"link_mass": "1 m"}, designs)


def test_a_case_open_parameter_resolves_and_keeps_its_written_limit_text(tmp_path: Path) -> None:
    text = MACHINE_CASE.replace(
        "initial_state:",
        "open_parameters:\n  - name: tolerance\n    dimension: dimensionless\n"
        "    owed_by: t\n    note: n\ninitial_state:",
    ).replace("    limit: 0.001 rad", "    limit: ${tolerance}")
    designs, case = machine(tmp_path, text)
    prepared = prepare(case, {**BIND, "tolerance": "0.002 rad"}, designs)
    assert prepared.limits["final_angle"] == "final within ${tolerance} of 0.05 rad"
    result = run_case(case, services(designs, fake()), {**BIND, "tolerance": "0.002 rad"})
    assert result.score.criteria[0].limit == "final within ${tolerance} of 0.05 rad"
    assert result.score.status == "pass"


# ===== Initial state and controller precondition ========================


@pytest.mark.req("TST-012")
def test_the_initial_state_is_set_before_the_run_starts(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    driver = fake()
    run_case(case, services(designs, driver), BIND)
    assert driver.calls.index("set_initial_state") < driver.calls.index("run")
    assert driver.initial is not None
    assert driver.initial.joints[0].joint == "pivot"
    assert driver.initial.joints[0].position.value == 0.05  # type: ignore[union-attr]


@pytest.mark.parametrize(
    ("reported", "message"),
    [
        ({"homed": True}, "expected False, firmware reports True"),
        ({}, "the firmware reports no 'homed'"),
        ({"homed": "false"}, "expected False, firmware reports 'false'"),
    ],
)
def test_a_controller_precondition_mismatch_is_an_error_before_t0(
    tmp_path: Path, reported: dict[str, Any], message: str
) -> None:
    designs, case = machine(tmp_path)
    driver = fake(reported=reported)
    result = run_case(case, services(designs, driver), BIND)
    assert result.score.status == "error"
    assert not result.score.ran
    assert message in str(result.score.error)
    assert "run" not in driver.calls
    assert driver.calls.index("reported_controller_state") > driver.calls.index("load")


def test_a_met_controller_precondition_lets_the_run_start(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    driver = fake(reported={"homed": False, "other": "idle"})
    result = run_case(case, services(designs, driver), BIND)
    assert result.score.ran
    assert "run" in driver.calls


# ===== Command timeline ================================================


@pytest.mark.req("TST-013")
def test_commands_go_through_the_operator_link_at_the_next_tick(tmp_path: Path) -> None:
    # 0.0015 s at 1 kHz is tick 1.5 -> delivered at tick 2; 3 ms is on tick 3, delivered there.
    designs, case = machine(tmp_path)
    driver = fake()
    result = run_case(case, services(designs, driver), BIND)
    assert [(command.tick, command.message) for command in driver.commands] == [(2, "home"), (3, "target")]
    assert driver.commands[1].payload == (("angle", 0.25),)
    delivered = [(event.time, event.detail) for event in driver.recording().events() if event.kind == MESSAGE_KIND]
    assert delivered == [(0.002, "home"), (0.003, "target")]
    assert result.commands == tuple(driver.commands)


@pytest.mark.parametrize(
    ("seconds", "rate", "tick"),
    [
        (0.0, Fraction(1000), 0),
        (0.001, Fraction(1000), 1),
        (0.0015, Fraction(1000), 2),
        (0.0010001, Fraction(1000), 2),
        (0.003, Fraction(1000), 3),
        (0.0011, Fraction(1_000_000), 1100),
        (0.0015, Fraction(3000), 5),
        (1.0 / 3.0, Fraction(3), 1),
        (0.1 + 0.2, Fraction(10), 3),
    ],
)
def test_the_delivery_rule_rounds_up_to_the_next_base_tick(seconds: float, rate: Fraction, tick: int) -> None:
    # Times are taken to 12 significant digits first: 0.0011 s (float 0.0011000000000000001 after a ms conversion)
    # stays tick 1100 at 1 MHz, and 0.1 + 0.2 = 0.30000000000000004 stays tick 3 at 10 Hz, not 4.
    assert next_tick(seconds, rate) == tick


def test_unequal_rates_give_the_lcm_base_tick(tmp_path: Path) -> None:
    assert rational_lcm(Fraction(1000), Fraction(400)) == Fraction(2000)
    assert rational_lcm(Fraction(1000), Fraction(1, 10)) == Fraction(1000)
    text = MACHINE_CASE.replace("seed: 7", "seed: 7\nrates:\n  control_rate: 3 kHz")
    designs, case = machine(tmp_path, text)
    driver = fake()
    run_case(case, services(designs, driver), BIND)
    assert driver.config is not None
    assert driver.config.base_rate == Fraction(3000)
    assert driver.config.physics_rate == Fraction(1000)
    assert [command.tick for command in driver.commands] == [5, 9]


def test_an_end_time_off_the_base_tick_is_an_error() -> None:
    with pytest.raises(RunnerError, match=r"end\.at"):
        end_spec(minimal_case(end={"at": "1.5 ms", "max_duration": "2 ms"}), Fraction(1000))
    assert exact_tick(0.002, Fraction(1000), "x") == 2


# ===== Fault timeline ==================================================


def _fault_case(*faults: dict[str, Any]) -> Case:
    return minimal_case(faults=list(faults))


@pytest.mark.req("TST-014")
@pytest.mark.parametrize("kind", ["sensor_stuck", "sensor_noisy", "sensor_dropped"])
def test_sensor_faults_go_to_the_signal_layer_for_their_window(kind: str) -> None:
    fault: dict[str, Any] = {"kind": kind, "target": "encoder", "start": "0.1 s", "duration": "0.2 s"}
    if kind == "sensor_noisy":
        fault["standard_deviation"] = "2 mrad"
    (scheduled,) = fault_timeline(_fault_case(fault), Fraction(1000))
    assert (scheduled.owner, scheduled.kind, scheduled.target) == ("signals", kind, "encoder")
    assert (scheduled.start_tick, scheduled.end_tick) == (100, 300)
    if kind == "sensor_noisy":
        assert scheduled.parameters == (("standard_deviation", pytest.approx(0.002)),)


@pytest.mark.req("TST-037")
@pytest.mark.parametrize("level", ["high", "low"])
def test_a_forced_level_fault_goes_to_the_signal_layer(level: str) -> None:
    fault = {"kind": "sensor_forced", "target": "switch", "level": level, "start": "0.0105 s", "end": "0.02 s"}
    (scheduled,) = fault_timeline(_fault_case(fault), Fraction(1000))
    assert (scheduled.owner, scheduled.start_tick, scheduled.end_tick) == ("signals", 11, 20)
    assert scheduled.parameters == (("level", level),)


@pytest.mark.req("TST-015")
def test_a_slip_fault_goes_to_physics() -> None:
    fault = {"kind": "slip", "target": "belt", "fraction": "0.5", "start": "0 s", "end": "1 ms"}
    (scheduled,) = fault_timeline(_fault_case(fault), Fraction(1000))
    assert (scheduled.owner, scheduled.target, scheduled.start_tick, scheduled.end_tick) == ("physics", "belt", 0, 1)
    assert scheduled.parameters == (("fraction", 0.5),)


@pytest.mark.req("TST-016")
def test_a_motor_stall_goes_to_the_signal_layer() -> None:
    fault = {"kind": "motor_stall", "target": "motor", "available_torque": "0.25", "start": "2 ms", "duration": "3 ms"}
    (scheduled,) = fault_timeline(_fault_case(fault), Fraction(1000))
    assert (scheduled.owner, scheduled.start_tick, scheduled.end_tick) == ("signals", 2, 5)
    assert scheduled.parameters == (("available_torque", 0.25),)


@pytest.mark.req("TST-017")
def test_a_power_loss_goes_to_the_signal_layer_on_its_rail() -> None:
    fault = {"kind": "power_loss", "rail": "motor_rail", "start": "1 ms", "duration": "1 ms"}
    (scheduled,) = fault_timeline(_fault_case(fault), Fraction(1000))
    assert (scheduled.owner, scheduled.target, scheduled.start_tick, scheduled.end_tick) == (
        "signals",
        "motor_rail",
        1,
        2,
    )


@pytest.mark.req("TST-018")
def test_a_controller_freeze_goes_to_the_controller_runtime(tmp_path: Path) -> None:
    text = MACHINE_CASE.replace("end:", "faults:\n  - kind: controller_freeze\n    start: 4 ms\n    end: 6 ms\nend:")
    designs, case = machine(tmp_path, text)
    driver = fake()
    run_case(case, services(designs, driver), BIND)
    (scheduled,) = driver.faults
    assert (scheduled.owner, scheduled.target, scheduled.start_tick, scheduled.end_tick) == ("controller", None, 4, 6)


@pytest.mark.req("TST-019")
def test_an_estop_press_goes_to_the_signal_layer_and_sends_the_controller_nothing() -> None:
    case = _fault_case({"kind": "estop_press", "target": "estop", "start": "1 ms", "duration": "5 ms"})
    (scheduled,) = fault_timeline(case, Fraction(1000))
    assert (scheduled.owner, scheduled.target, scheduled.start_tick, scheduled.end_tick) == ("signals", "estop", 1, 6)
    assert FAULT_OWNERS["estop_press"] == "signals"


@pytest.mark.req("TST-019")
def test_an_estop_fault_schedules_no_operator_command() -> None:
    case = _fault_case({"kind": "estop_press", "target": "estop", "start": "1 ms", "duration": "5 ms"})
    assert command_timeline(case, Fraction(1000)) == []
    assert len(fault_timeline(case, Fraction(1000))) == 1


@pytest.mark.req("TST-020")
def test_a_payload_change_goes_to_physics_at_its_tick(tmp_path: Path) -> None:
    text = MACHINE_CASE.replace(
        "end:", "faults:\n  - kind: payload_change\n    target: link\n    start: 2.5 ms\n    mass: 0.5 kg\nend:"
    )
    designs, case = machine(tmp_path, text)
    driver = fake()
    run_case(case, services(designs, driver), BIND)
    (scheduled,) = driver.faults
    assert (scheduled.owner, scheduled.target, scheduled.start_tick, scheduled.end_tick) == ("physics", "link", 3, None)
    assert scheduled.parameters == (("mass", 0.5),)


def test_a_fault_window_inside_one_tick_is_an_error() -> None:
    case = _fault_case({"kind": "sensor_stuck", "target": "s", "start": "0.1 ms", "end": "0.2 ms"})
    with pytest.raises(RunnerError, match="covers no base tick"):
        fault_timeline(case, Fraction(1000))


def test_every_fault_kind_has_an_owner() -> None:
    assert set(FAULT_OWNERS) == {
        "sensor_stuck",
        "sensor_noisy",
        "sensor_dropped",
        "sensor_forced",
        "slip",
        "motor_stall",
        "power_loss",
        "controller_freeze",
        "estop_press",
        "payload_change",
    }


# ===== End conditions and run errors ===================================


@pytest.mark.req("TST-021")
def test_the_run_ends_at_the_end_time(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    driver = fake()
    result = run_case(case, services(designs, driver), BIND)
    assert driver.end is not None
    assert (driver.end.at_tick, driver.end.max_tick) == (10, 20)
    assert result.report is not None
    assert (result.report.reason, result.report.end_tick) == ("end_time", 10)
    assert result.score.end_reason == "end_time"


def _done_at(tick: int) -> Any:
    def script(n: int, t: float) -> Row:
        events: list[RecordedEvent] = []
        if n == tick:
            events.append(RecordedEvent(t, "controller", MESSAGE_KIND, "done"))
        return Row({ANGLE: 0.05}, "idle", events)

    return script


@pytest.mark.req("TST-021")
def test_the_run_ends_on_an_observed_message(tmp_path: Path) -> None:
    designs, case = machine(tmp_path, MACHINE_CASE.replace("  at: 10 ms\n", "  message: done\n"))
    driver = fake(script=_done_at(7))
    result = run_case(case, services(designs, driver), BIND)
    assert result.report is not None
    assert (result.report.reason, result.report.end_tick) == ("message", 7)


@pytest.mark.req("TST-021")
def test_the_run_ends_on_a_controller_state(tmp_path: Path) -> None:
    designs, case = machine(tmp_path, MACHINE_CASE.replace("  at: 10 ms\n", "  controller_state: ready\n"))

    def script(n: int, t: float) -> Row:
        state = "idle"
        if n >= 12:
            state = "ready"
        return Row({ANGLE: 0.05}, state)

    result = run_case(case, services(designs, fake(script=script)), BIND)
    assert result.report is not None
    assert (result.report.reason, result.report.end_tick) == ("controller_state", 12)


@pytest.mark.req("TST-021")
def test_the_run_stops_at_max_duration_when_the_trigger_never_comes(tmp_path: Path) -> None:
    designs, case = machine(tmp_path, MACHINE_CASE.replace("  at: 10 ms\n", "  message: done\n"))
    result = run_case(case, services(designs, fake()), BIND)
    assert result.report is not None
    assert (result.report.reason, result.report.end_tick) == ("max_duration", 20)
    assert result.score.end_time == 0.02


@pytest.mark.req("TST-022")
@pytest.mark.req("TST-023")
def test_a_run_error_ends_the_run_and_is_still_scored_as_errored(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    archiver = FakeArchiver()
    driver = fake(error_tick=4, error="physics: constraint drift beyond limit")
    result = run_case(case, services(designs, driver, archiver=archiver), BIND, tmp_path / "runs")
    assert result.report is not None
    assert (result.report.reason, result.report.end_tick) == ("run_error", 4)
    assert result.score.status == "error"
    assert result.score.ran
    assert result.score.end_reason == "run_error"
    assert result.score.end_time == 0.004
    assert result.score.error == "physics: constraint drift beyond limit"
    assert [criterion.status for criterion in result.score.criteria] == ["error"]
    assert len(archiver.written) == 1


# ===== Configuration, seeds, mode, dry ================================


@pytest.mark.req("TST-028")
def test_each_module_instance_gets_its_own_child_seed_in_registration_order(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    driver = fake()
    run_case(case, services(designs, driver, FakeBuilder(["physics", "sensors", "controller"])), BIND)
    assert driver.config is not None
    children = np.random.SeedSequence(7).spawn(3)
    expected = [int(child.generate_state(1, dtype=np.uint64)[0]) for child in children]
    assert driver.config.module_seeds == tuple(zip(["physics", "sensors", "controller"], expected, strict=True))
    assert driver.config.seed == 7
    assert len(set(expected)) == 3


@pytest.mark.req("TST-028")
def test_seeds_are_reproducible_and_follow_the_order() -> None:
    first = module_seeds(3, ["a", "b"])
    assert module_seeds(3, ["a", "b"]) == first
    assert module_seeds(3, ["b", "a"]) == (("b", first[0][1]), ("a", first[1][1]))
    assert module_seeds(4, ["a", "b"]) != first
    assert module_seeds(3, []) == ()
    with pytest.raises(RunnerError, match="unique"):
        module_seeds(3, ["a", "a"])


@pytest.mark.req("TST-036")
@pytest.mark.parametrize("mode", ["native", "emulated"])
def test_the_controller_mode_is_a_run_input_passed_to_the_driver(tmp_path: Path, mode: str) -> None:
    designs, case = machine(tmp_path, MACHINE_CASE.replace("controller_mode: native", f"controller_mode: {mode}"))
    driver = fake()
    result = run_case(case, services(designs, driver), BIND)
    assert driver.config is not None
    assert driver.config.controller_mode == mode
    assert result.score.controller_mode == mode


def test_a_design_without_firmware_runs_with_no_mode_and_no_control_rate(tmp_path: Path) -> None:
    designs = write_designs(tmp_path)
    shipped = Path(__file__).resolve().parents[2] / "test_cases" / "pendulum" / "P1_period.yaml"
    driver = fake(reported={})
    result = run_case(shipped, services(designs, driver), {})
    assert driver.config is not None
    assert (driver.config.controller_mode, driver.config.control_rate) == (None, None)
    assert driver.config.base_rate == Fraction(1000)
    assert "reported_controller_state" not in driver.calls
    assert result.score.status == "error"
    assert "zero crossings" in str(result.score.criteria[0].note)


def test_a_dry_run_configures_and_schedules_without_running(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    driver = fake()
    result = run_case(case, services(designs, driver), BIND, dry=True)
    assert "run" not in driver.calls
    assert result.config is not None
    assert result.config.dry
    assert len(result.commands) == 2
    assert result.report is None


@pytest.mark.req("TST-031")
def test_a_run_writes_its_archive_with_the_run_identity(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    archiver = FakeArchiver()
    result = run_case(case, services(designs, fake(), archiver=archiver), BIND, tmp_path / "runs")
    assert result.archive == tmp_path / "runs" / "M1-seed7.h5"
    assert result.archive.is_file()
    assert result.score.run is not None
    assert result.score.run.run_id == "M1-seed7"
    assert len(result.score.run.archive_sha256) == 64
    ((_, metadata),) = archiver.written
    assert (metadata.seed, metadata.controller_mode, metadata.bindings) == (7, "native", (("link_mass", "1 kg"),))


@pytest.mark.req("INT-026", "INT-005")
def test_the_integrity_report_notes_reach_the_run_report_and_the_score(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    note = "member 'arm': not evaluated, built rigid"

    def integrity(build: Any, case: Any, recording: Any) -> IntegrityReport:
        return IntegrityReport(members=(), notes=(note,))

    result = run_case(case, services(designs, fake(), integrity=integrity), BIND)
    assert result.report is not None
    assert result.report.notes[-1] == note
    assert result.metadata is not None
    assert result.metadata.report.notes[-1] == note
    assert result.score.notes[-1] == note


@pytest.mark.req("INT-026")
def test_integrity_results_without_notes_leave_the_run_report_notes_alone(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)

    def integrity(build: Any, case: Any, recording: Any) -> IntegrityResults:
        return IntegrityResults(members=())

    plain = run_case(case, services(designs, fake()), BIND)
    result = run_case(case, services(designs, fake(), integrity=integrity), BIND)
    assert result.score.notes == plain.score.notes


# ===== Suites and sweeps ===============================================


def _failing(n: int, t: float) -> Row:
    return Row({ANGLE: 0.06})


@pytest.mark.req("TST-024")
def test_a_suite_runs_every_case_in_one_batch(tmp_path: Path) -> None:
    designs = write_designs(tmp_path)
    suite = tmp_path / "suite"
    write_case(suite, MACHINE_CASE)
    write_case(suite, MACHINE_CASE.replace("id: M1", "id: M10"), "M10.yaml")
    write_case(suite, MACHINE_CASE.replace("id: M1", "id: M2"), "M2_second.yaml")
    (suite / "notes.yaml").write_text("just: notes\n", encoding="utf-8")
    assert [path.name for path in case_files(suite)] == ["M1.yaml", "M2_second.yaml", "M10.yaml"]
    run = run_suite(suite, services(designs, fake()), BIND)
    assert [case.case_id for case in run.score.cases] == ["M1", "M2", "M10"]
    assert run.score.status == "pass"
    assert run.errors == ()


@pytest.mark.req("TST-025")
def test_a_sweep_runs_once_per_value_recording_the_value(tmp_path: Path) -> None:
    text = MACHINE_CASE + "sweeps:\n  - parameter: ${link_mass}\n    values: [2 kg, 3 kg]\n"
    designs = write_designs(tmp_path)
    suite = tmp_path / "suite"
    write_case(suite, text)
    builder = FakeBuilder()
    run = run_suite(suite, services(designs, fake(), builder), BIND)
    assert [case.sweep for case in run.score.cases] == [None, ("${link_mass}", "2 kg"), ("${link_mass}", "3 kg")]
    assert [design.bodies[0].mass_properties.mass for design in builder.built] == [1.0, 2.0, 3.0]
    assert run.runs[1].metadata is not None
    assert run.runs[1].metadata.sweep == ("${link_mass}", "2 kg")
    assert run.runs[2].metadata is not None
    assert run.runs[2].metadata.run_id == "M1-seed7-sweep0-1-3_kg"


@pytest.mark.req("TST-025")
def test_a_sweep_over_a_design_field_patches_the_design(tmp_path: Path) -> None:
    designs = write_designs(tmp_path)
    shipped = Path(__file__).resolve().parents[2] / "test_cases" / "pendulum" / "P1_period.yaml"
    text = shipped.read_text(encoding="utf-8") + (
        "sweeps:\n  - parameter: bodies.link.mass_properties.mass\n    values: [2 kg]\n"
    )
    suite = tmp_path / "suite"
    write_case(suite, text, "P1_period.yaml")
    builder = FakeBuilder()
    run = run_suite(suite, services(designs, fake(reported={}), builder))
    assert [design.bodies[0].mass_properties.mass for design in builder.built] == [1.0, 2.0]
    assert run.score.cases[1].sweep == ("bodies.link.mass_properties.mass", "2 kg")


@pytest.mark.req("TST-027")
def test_a_suite_rerun_is_diffed_against_its_baseline(tmp_path: Path) -> None:
    designs = write_designs(tmp_path)
    suite = tmp_path / "suite"
    write_case(suite, MACHINE_CASE + "baseline: baseline.yaml\n")
    write_case(suite, MACHINE_CASE.replace("id: M1", "id: M2") + "baseline: baseline.yaml\n", "M2.yaml")
    (suite / "baseline.yaml").write_text(
        unrun_baseline("suite", ["M1", "M2"]),
        encoding="utf-8",
    )
    load_baseline(suite / "baseline.yaml")
    run = run_suite(suite, services(designs, fake(), archiver=FakeArchiver()), BIND, tmp_path / "runs")
    assert run.diff is not None
    assert [(change.case, change.new) for change in run.diff.newly_passing] == [("M1", "pass"), ("M2", "pass")]
    assert run.baseline is not None
    assert run.baseline_path == (suite / "baseline.yaml").resolve()


def test_a_suite_without_an_archive_cannot_produce_a_baseline_and_says_so(tmp_path: Path) -> None:
    designs = write_designs(tmp_path)
    suite = tmp_path / "suite"
    write_case(suite, MACHINE_CASE + "baseline: baseline.yaml\n")
    (suite / "baseline.yaml").write_text(
        unrun_baseline("suite", ["M1"]),
        encoding="utf-8",
    )
    run = run_suite(suite, services(designs, fake()), BIND)
    assert run.diff is None
    assert any("archive identity" in message for message in run.errors)


def test_a_suite_with_no_cases_is_an_error(tmp_path: Path) -> None:
    designs = write_designs(tmp_path)
    (tmp_path / "empty").mkdir()
    run = run_suite(tmp_path / "empty", services(designs, fake()))
    assert run.errors == ("suite empty: no test case files",)
    assert run.score.status == "fail"


def test_a_failing_case_fails_the_suite_and_both_still_run(tmp_path: Path) -> None:
    designs = write_designs(tmp_path)
    suite = tmp_path / "suite"
    write_case(suite, MACHINE_CASE)
    write_case(
        suite, MACHINE_CASE.replace("id: M1", "id: M2").replace("target: 0.05 rad", "target: 0.07 rad"), "M2.yaml"
    )
    run = run_suite(suite, services(designs, fake()), BIND)
    assert [case.status for case in run.score.cases] == ["pass", "fail"]
    assert run.score.status == "fail"


# ===== Suite files =====================================================


def _suite_yaml(cases: list[str], baseline: str = "baseline.yaml", version: str = "0.1.0") -> str:
    lines = ["name: nightly", "design:", "  name: machine", f"  version: {version}", "cases:"]
    lines.extend(f"  - {case}" for case in cases)
    lines.append(f"baseline: {baseline}")
    return "\n".join(lines) + "\n"


def _suite_folder(tmp_path: Path) -> tuple[Path, Path]:
    designs = write_designs(tmp_path)
    suite = tmp_path / "suite"
    write_case(suite, MACHINE_CASE)
    write_case(suite, MACHINE_CASE.replace("id: M1", "id: M2"), "M2.yaml")
    write_case(suite, MACHINE_CASE.replace("id: M1", "id: M3"), "M3.yaml")
    (suite / "baseline.yaml").write_text(unrun_baseline("nightly", ["M1", "M2"]), encoding="utf-8")
    return designs, suite


@pytest.mark.req("TST-008", "TST-027")
def test_a_suite_file_names_its_cases_and_the_baseline_it_is_diffed_against(tmp_path: Path) -> None:
    designs, suite = _suite_folder(tmp_path)
    (suite / "suite.yaml").write_text(_suite_yaml(["M2.yaml", "M1.yaml"]), encoding="utf-8")
    for path in (suite / "suite.yaml", suite):
        run = run_suite(path, services(designs, fake(), archiver=FakeArchiver()), BIND, tmp_path / "runs")
        assert run.errors == ()
        assert run.score.name == "nightly"
        assert [case.case_id for case in run.score.cases] == ["M2", "M1"]
        assert run.baseline_path == (suite / "baseline.yaml").resolve()
        assert run.diff is not None
        assert [(change.case, change.old, change.new) for change in run.diff.newly_passing] == [
            ("M1", "not_run", "pass"),
            ("M2", "not_run", "pass"),
        ]
        assert run.baseline is not None
        assert [entry.case for entry in run.baseline.entries] == ["M1", "M2"]


@pytest.mark.req("TST-027")
def test_a_folder_without_a_suite_file_uses_its_baseline_yaml(tmp_path: Path) -> None:
    designs, suite = _suite_folder(tmp_path)
    (suite / "baseline.yaml").write_text(unrun_baseline("suite", ["M1", "M2", "M3"]), encoding="utf-8")
    run = run_suite(suite, services(designs, fake(), archiver=FakeArchiver()), BIND, tmp_path / "runs")
    assert run.errors == ()
    assert run.baseline_path == (suite / "baseline.yaml").resolve()
    assert run.diff is not None
    assert [change.case for change in run.diff.newly_passing] == ["M1", "M2", "M3"]
    (suite / "baseline.yaml").unlink()
    run = run_suite(suite, services(designs, fake(), archiver=FakeArchiver()), BIND, tmp_path / "runs")
    assert (run.errors, run.baseline_path, run.diff) == ((), None, None)


@pytest.mark.req("TST-008")
def test_a_suite_file_that_does_not_load_is_an_error(tmp_path: Path) -> None:
    designs, suite = _suite_folder(tmp_path)
    path = suite / "suite.yaml"
    path.write_text(_suite_yaml(["M1.yaml", "M9.yaml"]), encoding="utf-8")
    run = run_suite(path, services(designs, fake()), BIND)
    assert run.errors == (f"{path}: cases[1]: missing test case file 'M9.yaml'",)
    assert (run.score.cases, run.runs, run.score.status) == ((), (), "fail")


@pytest.mark.req("TST-008")
def test_a_case_of_another_design_version_is_an_error(tmp_path: Path) -> None:
    designs, suite = _suite_folder(tmp_path)
    (suite / "suite.yaml").write_text(_suite_yaml(["M1.yaml"], version="0.2.0"), encoding="utf-8")
    run = run_suite(suite / "suite.yaml", services(designs, fake(), archiver=FakeArchiver()), BIND, tmp_path / "r")
    assert run.errors[0] == (
        "suite nightly: case M1 is written against machine 0.1.0, the suite file names machine 0.2.0"
    )


@pytest.mark.req("TST-008")
def test_a_case_naming_another_baseline_than_its_suite_file_is_an_error(tmp_path: Path) -> None:
    designs, suite = _suite_folder(tmp_path)
    write_case(suite, MACHINE_CASE + "baseline: other.yaml\n")
    (suite / "other.yaml").write_text(unrun_baseline("nightly", ["M1"]), encoding="utf-8")
    (suite / "suite.yaml").write_text(_suite_yaml(["M1.yaml"]), encoding="utf-8")
    run = run_suite(suite, services(designs, fake(), archiver=FakeArchiver()), BIND, tmp_path / "runs")
    assert run.errors == (
        f"suite nightly: a case names baseline {(suite / 'other.yaml').resolve()}, "
        f"the suite file names {(suite / 'baseline.yaml').resolve()}",
    )


@pytest.mark.req("TST-027")
def test_a_baseline_that_does_not_load_is_reported(tmp_path: Path) -> None:
    designs, suite = _suite_folder(tmp_path)
    (suite / "baseline.yaml").write_text("design: [\n", encoding="utf-8")
    (suite / "suite.yaml").write_text(_suite_yaml(["M1.yaml"]), encoding="utf-8")
    run = run_suite(suite, services(designs, fake(), archiver=FakeArchiver()), BIND, tmp_path / "runs")
    assert len(run.errors) == 1
    assert run.errors[0].startswith("baseline: ")
    assert "YAML parse error" in run.errors[0]
    assert (run.baseline, run.diff) == (None, None)


@pytest.mark.req("TST-027")
def test_a_baseline_of_another_suite_gives_no_diff(tmp_path: Path) -> None:
    designs, suite = _suite_folder(tmp_path)
    (suite / "baseline.yaml").write_text(unrun_baseline("weekly", ["M1"]), encoding="utf-8")
    (suite / "suite.yaml").write_text(_suite_yaml(["M1.yaml"]), encoding="utf-8")
    run = run_suite(suite, services(designs, fake(), archiver=FakeArchiver()), BIND, tmp_path / "runs")
    assert run.errors == ("baseline: cannot diff suite 'nightly' against a baseline of suite 'weekly'",)
    assert run.baseline is not None
    assert run.diff is None


# ===== Fault routing through the driver ================================


GANTRY_DESIGNS = Path(__file__).resolve().parents[2] / "designs"
GANTRY_CASE = Path(__file__).resolve().parents[2] / "test_cases" / "gantry" / "G1.yaml"


def _gantry_case(tmp_path: Path, extra: str) -> Path:
    text = GANTRY_CASE.read_text(encoding="utf-8").replace("baseline: baseline.yaml\n", "")
    return write_case(tmp_path / "cases", text + extra, "G1.yaml")


def _gantry_services(driver: InMemoryFakeDriver, builder: FakeBuilder | None = None, **extra: Any) -> Services:
    root = Path(__file__).resolve().parents[2] / "designs"
    return Services(lambda: driver, builder or FakeBuilder(), designs_root=root, **extra)


@pytest.mark.req("TST-014", "TST-016", "TST-019", "TST-037")
def test_signal_faults_reach_the_driver_with_their_windows(tmp_path: Path) -> None:
    # 1 kHz base tick: a window [start, start + duration) of 0.1 s + 0.2 s is ticks [100, 300).
    faults = """faults:
  - {kind: sensor_stuck, target: x_min_switch, start: 0.1 s, duration: 0.2 s}
  - {kind: sensor_noisy, target: x_encoder, standard_deviation: "0.1", start: 0.1 s, duration: 0.2 s}
  - {kind: sensor_dropped, target: y_encoder, start: 0.1 s, duration: 0.2 s}
  - {kind: sensor_forced, target: y_min_switch, level: high, start: 0.5 s, end: 0.75 s}
  - {kind: motor_stall, target: x_motor, available_torque: "0.25", start: 1 s, duration: 0.5 s}
  - {kind: estop_press, target: estop, start: 1.5 s, end: 1.6 s}
"""
    driver = fake(reported={"state": "not_homed"})
    result = run_case(_gantry_case(tmp_path, faults), _gantry_services(driver), dry=True)
    assert result.score.error == "dry run: configured, not run"
    assert [(fault.owner, fault.kind, fault.target, fault.start_tick, fault.end_tick) for fault in driver.faults] == [
        ("signals", "sensor_stuck", "x_min_switch", 100, 300),
        ("signals", "sensor_noisy", "x_encoder", 100, 300),
        ("signals", "sensor_dropped", "y_encoder", 100, 300),
        ("signals", "sensor_forced", "y_min_switch", 500, 750),
        ("signals", "motor_stall", "x_motor", 1000, 1500),
        ("signals", "estop_press", "estop", 1500, 1600),
    ]
    assert driver.faults[3].parameters == (("level", "high"),)
    assert driver.faults[4].parameters == (("available_torque", 0.25),)
    assert all(command.message != "estop" for command in driver.commands)


# ===== Overrides and part substitution =================================


def _registry_with_second_stepper(tmp_path: Path) -> Registry:
    """The shipped catalog plus `nema23_test`, a stepper of 1.2 N*m holding torque (its curve starts there, as the
    catalog requires)."""
    catalog = tmp_path / "catalog"
    catalog.mkdir()
    for path in CATALOG_DIR.glob("*.yaml"):
        (catalog / path.name).write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    stepper = catalog / "stepper_behavioral.yaml"
    data = yaml_loader().load(stepper.read_text(encoding="utf-8"))
    part = json.loads(json.dumps(data["parts"][0]))
    part["name"] = "nema23_test"
    part["values"]["holding_torque"] = "1.2 N*m"
    part["torque_speed_curve"] = [{"speed": "0 rpm", "torque": "1.2 N*m"}, {"speed": "600 rpm", "torque": "0.6 N*m"}]
    data["parts"].append(part)
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    stepper.write_text(stream.getvalue(), encoding="utf-8")
    return load_registry(catalog)


def _instance(design: Any, name: str) -> Any:
    return next(component for component in design.components if component.name == name)


@pytest.mark.req("LIB-017")
def test_a_substituted_stepper_reaches_the_built_instance_parameters(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    registry = _registry_with_second_stepper(tmp_path)
    # The builder revalidates its design against the shipped registry; the test part lives only in this one.
    monkeypatch.setattr(design_schema, "default_registry", lambda: registry)
    prepared = prepare(
        _gantry_case(tmp_path, ""), {"instance:x_motor.part": "nema23_test"}, GANTRY_DESIGNS, registry=registry
    )
    assert (_instance(prepared.design, "x_motor").part, _instance(prepared.design, "y_motor").part) == (
        "nema23_test",
        "nema23_class_generic",
    )
    assert ("instance:x_motor.part", "nema23_test") in prepared.bindings
    built = build(prepared.design, registry, shipped_materials(), rigid_members=True)
    torques = {
        item.name: item.parameters["holding_torque"]
        for item in built.wiring.instances
        if item.type == "stepper_behavioral"
    }
    assert torques == {"x_motor": 1.2, "y_motor": 1.26, "lift_motor": 1.26}


@pytest.mark.req("LIB-016")
def test_an_instance_parameter_is_overridden_for_one_run_in_si(tmp_path: Path) -> None:
    # 60 g*cm**2 = 60e-3 kg * 1e-4 m**2 = 6e-6 kg*m**2.
    prepared = prepare(
        _gantry_case(tmp_path, ""),
        {"instance:x_motor.rotor_inertia": "60 g*cm**2", "instance:estop.manual_reset": "false"},
        GANTRY_DESIGNS,
    )
    assert _instance(prepared.design, "x_motor").overrides == {"rotor_inertia": "60 g*cm**2"}
    assert _instance(prepared.design, "estop").overrides == {"manual_reset": False}
    built = build(prepared.design, default_registry(), shipped_materials(), rigid_members=True)
    inertia = {
        item.name: item.parameters["rotor_inertia"]
        for item in built.wiring.instances
        if item.type == "stepper_behavioral"
    }
    assert inertia["x_motor"] == pytest.approx(6e-6, rel=1e-12)
    assert inertia["y_motor"] == pytest.approx(280e-7, rel=1e-12)
    assert ("instance:x_motor.rotor_inertia", "60 g*cm**2") in prepared.bindings


@pytest.mark.req("LIB-016")
def test_set_overrides_the_cases_instance_parameter_and_the_archive_records_it(tmp_path: Path) -> None:
    case = _gantry_case(tmp_path, "parameters:\n  instance:x_motor.rotor_inertia: 50 g*cm**2\n")
    from_case = prepare(case, {}, GANTRY_DESIGNS)
    assert _instance(from_case.design, "x_motor").overrides == {"rotor_inertia": "50 g*cm**2"}
    driver = fake(reported={"state": "not_homed"}, script=lambda n, t: Row({}))
    result = run_case(
        case, _gantry_services(driver), {"instance:x_motor.rotor_inertia": "70 g*cm**2"}, tmp_path / "runs"
    )
    assert result.metadata is not None
    assert ("instance:x_motor.rotor_inertia", "70 g*cm**2") in result.metadata.bindings


@pytest.mark.req("LIB-016", "LIB-017")
def test_a_sweep_over_an_instance_part_runs_once_per_part(tmp_path: Path) -> None:
    registry = _registry_with_second_stepper(tmp_path)
    case = _gantry_case(
        tmp_path, "sweeps:\n  - parameter: instance:x_motor.part\n    values: [nema23_test, nema17_class_generic]\n"
    )
    builder = FakeBuilder()
    driver = fake(reported={"state": "not_homed"}, script=lambda n, t: Row({}))
    runs = sweep_runs(case, _gantry_services(driver, builder, registry=registry), None, None)
    assert [run.score.sweep for run in runs] == [
        ("instance:x_motor.part", "nema23_test"),
        ("instance:x_motor.part", "nema17_class_generic"),
    ]
    assert [_instance(design, "x_motor").part for design in builder.built] == ["nema23_test", "nema17_class_generic"]


@pytest.mark.req("LIB-016", "LIB-017")
@pytest.mark.parametrize(
    ("key", "value", "fragment"),
    [
        (
            "instance:z_motor.part",
            "nema17_class_generic",
            "instance:z_motor.part: unknown component instance 'z_motor'",
        ),
        (
            "instance:x_motor.colour",
            "red",
            "instance:x_motor.colour: instance 'x_motor': component type 'stepper_behavioral' has no parameter "
            "'colour'",
        ),
        (
            "instance:x_motor.rotor_inertia",
            "60 g",
            "instance:x_motor.rotor_inertia: instance 'x_motor': stepper_behavioral.rotor_inertia "
            "([mass] * [length] ** 2)",
        ),
        ("instance:estop.manual_reset", "yes", "estop_relay.manual_reset is a flag: give true or false, got 'yes'"),
        (
            "instance:x_motor.part",
            "nema99",
            "instance:x_motor.part: instance 'x_motor': no catalog part 'nema99' in the catalog "
            "(instance type 'stepper_behavioral')",
        ),
        (
            "instance:x_motor.part",
            "gt2_20t_6mm_generic",
            "instance:x_motor.part: instance 'x_motor': part 'gt2_20t_6mm_generic' is a 'belt_pulley', "
            "not a 'stepper_behavioral'",
        ),
        ("instance:x_motor", "1 kg", "instance:x_motor: expected instance:<name>.<parameter> or instance:<name>.part"),
        ("instance:x_motor.a.b", "1 kg", "expected instance:<name>.<parameter>"),
        ("instance:.part", "x", "expected instance:<name>.<parameter>"),
    ],
    ids=[
        "unknown_instance",
        "unknown_parameter",
        "dimension",
        "flag",
        "unknown_part",
        "other_type",
        "no_field",
        "nested",
        "no_instance",
    ],
)
def test_a_bad_override_is_an_error_naming_it(tmp_path: Path, key: str, value: str, fragment: str) -> None:
    case = _gantry_case(tmp_path, "")
    with pytest.raises(RunnerError) as raised:
        prepare(case, {key: value}, GANTRY_DESIGNS)
    assert fragment in str(raised.value)
    assert str(raised.value).startswith(f"{case}: ")
    result = run_case(case, _gantry_services(fake(reported={"state": "not_homed"})), {key: value})
    assert (result.score.status, result.score.ran) == ("error", False)


@pytest.mark.req("LIB-016")
def test_a_bad_override_in_the_cases_parameters_is_an_error(tmp_path: Path) -> None:
    case = _gantry_case(tmp_path, "parameters:\n  instance:x_motor.rotor_inertia: 5 kg\n")
    with pytest.raises(RunnerError, match=r"stepper_behavioral\.rotor_inertia"):
        prepare(case, {}, GANTRY_DESIGNS)


# ===== Seeded variations and suite-file sweeps (brief R) ================

VARIATIONS = """variations:
  count: 3
  parameters:
    - parameter: ${link_mass}
      distribution: normal
      spread: 0.1 kg
"""


class FreshDrivers:
    """A new fake driver per run, kept for inspection."""

    def __init__(self) -> None:
        self.made: list[InMemoryFakeDriver] = []

    def __call__(self) -> InMemoryFakeDriver:
        driver = fake()
        self.made.append(driver)
        return driver


def _expected_seed(case_seed: int, index: int) -> int:
    return int(np.random.SeedSequence(case_seed, spawn_key=(index,)).generate_state(1, dtype=np.uint64)[0])


@pytest.mark.req("TST-026")
def test_each_variation_runs_with_its_own_seed_and_drawn_value(tmp_path: Path) -> None:
    # Variation k's seed is SeedSequence(7) child k (D1); its value is nominal 1 kg + 0.1 kg times the first standard
    # normal draw of default_rng(seed), derived here independently of the runner.
    designs, case = machine(tmp_path, MACHINE_CASE + VARIATIONS)
    drivers = FreshDrivers()
    builder = FakeBuilder()
    results = variation_runs(case, Services(drivers, builder, designs_root=designs), BIND, None)
    seeds = [_expected_seed(7, index) for index in range(3)]
    assert [variation_seed(7, index) for index in range(3)] == seeds
    assert len(set(seeds)) == 3
    masses = [1.0 + 0.1 * float(np.random.default_rng(seed).standard_normal()) for seed in seeds]
    assert [design.bodies[0].mass_properties.mass for design in builder.built] == pytest.approx(masses, rel=1e-15)
    assert [driver.config.seed for driver in drivers.made if driver.config is not None] == seeds
    for result, seed, mass in zip(results, seeds, masses, strict=True):
        assert result.metadata is not None
        assert result.metadata.run_id == f"M1-variation-seed{seed}"
        assert result.metadata.seed == seed
        assert result.metadata.variation == (("${link_mass}", f"{mass:.17g} kg"),)
        assert result.score.seed == seed
        assert result.score.variation == result.metadata.variation
        assert result.score.sweep is None


@pytest.mark.req("TST-026")
def test_a_variation_reruns_alone_with_an_identical_result(tmp_path: Path) -> None:
    designs, case = machine(tmp_path, MACHINE_CASE + VARIATIONS)
    batch = variation_runs(case, Services(FreshDrivers(), FakeBuilder(), FakeArchiver(), designs), BIND, tmp_path / "a")
    seed = variation_seed(7, 1)
    alone = run_variation(
        case, Services(FreshDrivers(), FakeBuilder(), FakeArchiver(), designs), seed, BIND, tmp_path / "b"
    )
    assert alone.score == batch[1].score
    assert alone.metadata == batch[1].metadata
    assert batch[1].archive is not None
    assert alone.archive is not None
    assert alone.archive.read_bytes() == batch[1].archive.read_bytes()


@pytest.mark.req("TST-026")
def test_a_uniform_variation_stays_within_its_half_width(tmp_path: Path) -> None:
    designs, case = machine(tmp_path, MACHINE_CASE + VARIATIONS.replace("normal", "uniform").replace("3", "20", 1))
    builder = FakeBuilder()
    variation_runs(case, Services(FreshDrivers(), builder, designs_root=designs), BIND, None)
    masses = [design.bodies[0].mass_properties.mass for design in builder.built]
    assert len(masses) == 20
    assert all(0.9 <= mass <= 1.1 for mass in masses)


@pytest.mark.req("TST-026")
def test_a_variation_over_a_design_field_varies_about_the_design_value(tmp_path: Path) -> None:
    designs = write_designs(tmp_path)
    shipped = Path(__file__).resolve().parents[2] / "test_cases" / "pendulum" / "P1_period.yaml"
    text = shipped.read_text(encoding="utf-8") + (
        "variations:\n  count: 1\n  parameters:\n    - parameter: joints.pivot.angular.damping\n"
        "      distribution: normal\n      spread: 0.001 N*m*s/rad\n"
    )
    case = write_case(tmp_path / "cases", text, "P1_period.yaml")
    builder = FakeBuilder()
    driver = fake(reported={})
    result = variation_runs(case, services(designs, driver, builder), None, None)[0]
    seed = variation_seed(0, 0)
    expected = 0.04520785330006282 + 0.001 * float(np.random.default_rng(seed).standard_normal())
    assert builder.built[0].joints[0].angular[0].damping == pytest.approx(expected, rel=1e-15)
    assert result.score.ran


@pytest.mark.req("TST-026")
@pytest.mark.parametrize(
    ("variations", "message"),
    [
        (VARIATIONS.replace("0.1 kg", "0.1 m"), "spread '0.1 m' does not match nominal '1 kg'"),
        (
            VARIATIONS.replace("${link_mass}", "instance:pivot_motor.inertia"),
            "unknown component instance 'pivot_motor'",
        ),
        (
            VARIATIONS.replace("${link_mass}", "bodies.link.missing"),
            "'bodies.link.missing' does not name a design field",
        ),
    ],
)
def test_a_variation_that_cannot_be_drawn_is_an_errored_unrun_case(
    tmp_path: Path, variations: str, message: str
) -> None:
    designs, case = machine(tmp_path, MACHINE_CASE + variations)
    results = variation_runs(case, services(designs, fake()), BIND, None)
    assert len(results) == 3
    for result in results:
        assert (result.score.status, result.score.ran) == ("error", False)
        assert message in str(result.score.error)
        assert result.score.variation == ()


@pytest.mark.req("TST-026")
def test_a_case_without_variations_has_no_variation_runs(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    assert variation_runs(case, services(designs, fake()), BIND, None) == []
    alone = run_variation(case, services(designs, fake()), 5, BIND)
    assert (alone.score.status, alone.score.seed) == ("error", 5)
    assert "the case defines no variations" in str(alone.score.error)


@pytest.mark.req("TST-007")
@pytest.mark.req("TST-025")
@pytest.mark.req("TST-026")
def test_a_suite_file_sweeps_and_varies_every_case(tmp_path: Path) -> None:
    designs = write_designs(tmp_path)
    suite = tmp_path / "suite"
    write_case(suite, MACHINE_CASE)
    write_case(suite, MACHINE_CASE.replace("id: M1", "id: M2") + VARIATIONS.replace("count: 3", "count: 1"), "M2.yaml")
    (suite / "baseline.yaml").write_text(unrun_baseline("demo", ["M1", "M2"]), encoding="utf-8")
    (suite / "suite.yaml").write_text(
        "name: demo\ndesign:\n  name: machine\n  version: 0.1.0\ncases: [M1.yaml, M2.yaml]\nbaseline: baseline.yaml\n"
        "sweeps:\n  - parameter: ${link_mass}\n    values: [2 kg]\n"
        "variations:\n  count: 2\n  parameters:\n    - parameter: ${link_mass}\n      distribution: uniform\n"
        "      spread: 0.5 kg\n",
        encoding="utf-8",
    )
    services_ = Services(FreshDrivers(), FakeBuilder(), FakeArchiver(), designs)
    run = run_suite(suite / "suite.yaml", services_, BIND, tmp_path / "runs")
    shape = [(case.case_id, case.sweep, case.variation is not None) for case in run.score.cases]
    assert shape == [
        ("M1", None, False),
        ("M1", ("${link_mass}", "2 kg"), False),
        ("M1", None, True),
        ("M1", None, True),
        ("M2", None, False),
        ("M2", ("${link_mass}", "2 kg"), False),
        ("M2", None, True),
    ]
    assert [case.seed for case in run.score.cases if case.variation is not None] == [
        variation_seed(7, 0),
        variation_seed(7, 1),
        variation_seed(7, 0),
    ]
    assert run.errors == ()


@pytest.mark.req("TST-007")
def test_a_suite_file_rejects_a_repeated_sweep_parameter(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="duplicate sweep parameter"):
        SuiteFile.model_validate(
            {
                "name": "demo",
                "design": {"name": "machine", "version": "0.1.0"},
                "cases": ["M1.yaml"],
                "baseline": "baseline.yaml",
                "sweeps": [{"parameter": "${a}", "values": ["1 kg"]}, {"parameter": "${a}", "values": ["2 kg"]}],
            }
        )


@pytest.mark.req("TST-025")
def test_set_and_get_design_path_reach_every_per_dof_entry() -> None:
    data: dict[str, Any] = {
        "joints": [{"name": "j", "angular": [{"damping": "1 N*m*s/rad"}, {"damping": "1 N*m*s/rad"}]}]
    }
    assert get_design_path(data, "joints.j.angular.damping") == "1 N*m*s/rad"
    set_design_path(data, "joints.j.angular.damping", "2 N*m*s/rad")
    assert [entry["damping"] for entry in data["joints"][0]["angular"]] == ["2 N*m*s/rad", "2 N*m*s/rad"]
    data["joints"][0]["angular"][1]["damping"] = "3 N*m*s/rad"
    with pytest.raises(RunnerError, match="the entries differ"):
        get_design_path(data, "joints.j.angular.damping")
    with pytest.raises(RunnerError, match="no joints item 'k'"):
        set_design_path(data, "joints.k.angular.damping", "1 N*m*s/rad")
    with pytest.raises(RunnerError, match="the design has no bodies"):
        get_design_path(data, "bodies.b.mass")


@pytest.mark.req("TST-025")
@pytest.mark.req("SCO-018")
def test_the_pendulum_damping_sweep_on_the_core(tmp_path: Path) -> None:
    # P2 sweeps the pivot damping d = zeta * 2 I omega0 (I = 1/12 kg m^2, omega0 = 2 pi / T0, T0 = 1.1582031 s) over
    # zeta = 0.02, 0.05, 0.1; the damped period is T0 / sqrt(1 - zeta^2). Each run's period is held to ACC-033's 0.1
    # percent of its own closed form; backward Euler's period error (omega h)^2 / 3 = 1e-5 at 1 ms is far inside it.
    from mechsim.scoring.score import SuiteScore, report_text
    from mechsim.testing.core_driver import core_services

    case = Path(__file__).resolve().parents[2] / "test_cases" / "pendulum" / "P2_damping_sweep.yaml"
    runs = sweep_runs(case, core_services(), None, tmp_path)
    t0 = 2.0 * np.pi * np.sqrt((1.0 / 12.0) / (1.0 * 9.81 * 0.25))
    for run, zeta in zip(runs, (0.02, 0.05, 0.1), strict=True):
        expected = t0 / np.sqrt(1.0 - zeta**2)
        measured = run.score.criteria[0].measured
        print(f"zeta {zeta}: closed form {expected:.7f} s, measured {measured} s")
        assert measured is not None
        assert abs(measured - expected) <= 1e-3 * expected
        assert run.metadata is not None
        assert run.metadata.sweep is not None
        assert run.metadata.run_id.endswith(run.metadata.sweep[1].replace(" ", "_").replace("*", "_").replace("/", "_"))
    text = report_text(SuiteScore("pendulum", runs[0].score.design, tuple(run.score for run in runs)))
    assert "Sweep P2 over joints.pivot.angular.damping\n| criterion | 0.018083141320025128 N*m*s/rad |" in text


@pytest.mark.req("TST-026")
def test_drawing_a_variation_needs_a_nominal_and_a_resolved_matching_spread() -> None:
    variations = minimal_case(
        variations={
            "count": 1,
            "parameters": [{"parameter": "instance:m.inertia", "distribution": "normal", "spread": "1 kg*m^2"}],
        }
    ).variations
    assert variations is not None
    with pytest.raises(RunnerError, match="no nominal value to vary about"):
        draw_variation(variations, 1, {})
    with pytest.raises(RunnerError, match="does not match nominal"):
        draw_variation(variations, 1, {"instance:m.inertia": "1 kg"})
    drawn = draw_variation(variations, 1, {"instance:m.inertia": "2 kg*m^2"})
    expected = 2.0 + float(np.random.default_rng(1).standard_normal())
    assert drawn == (("instance:m.inertia", f"{expected:.17g} kg * m ** 2"),)
