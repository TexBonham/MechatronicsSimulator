"""Test runner (TST) against the scripted in-memory fake driver."""

from __future__ import annotations

from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mechsim.testing.baseline import load_baseline
from mechsim.testing.driver import MESSAGE_KIND, RecordedEvent
from mechsim.testing.runner import (
    FAULT_OWNERS,
    RunnerError,
    Services,
    case_files,
    command_timeline,
    end_spec,
    exact_tick,
    fault_timeline,
    module_seeds,
    next_tick,
    prepare,
    rational_lcm,
    run_case,
    run_suite,
)
from mechsim.testing.schema import Case
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
    assert run.runs[2].metadata.run_id == "M1-seed7-sweep0-1"


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
