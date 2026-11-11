"""Typed driver errors scored as run errors (TST-023, invariant 7), approximation lines in the score (SCO-013), the
stepper's rotor inertia checked from the Python side (SIG-037, BLD-034), the archived control rate (TST-033)."""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
from pathlib import Path
from typing import Any

import pytest

from mechsim import _core as core
from mechsim.builder.result import BuildResult
from mechsim.design.schema import Design
from mechsim.physics.mujoco.translate import UnmappableElementError
from mechsim.physics.neutral import LeadScrew
from mechsim.record.archive import RunArchive
from mechsim.scoring.score import SuiteScore, report_text
from mechsim.testing import core_driver, core_wiring
from mechsim.testing.core_driver import CoreDriver, DriverError, RecordArchiver, RecordArchiveReader, core_builder
from mechsim.testing.driver import EndSpec, RunReport
from mechsim.testing.runner import RunResult, Services, prepare, run_case
from mechsim.testing.schema import DESIGNS_ROOT, ROOT

G3 = ROOT / "test_cases" / "gantry" / "G3.yaml"
P1 = ROOT / "test_cases" / "pendulum" / "P1_period.yaml"
TICKS = 5


class ShortDriver(CoreDriver):
    """The real driver, stopped after a few ticks: these tests need the setup path and a short recording only."""

    def run(self, end: EndSpec) -> RunReport:
        return super().run(EndSpec(max_tick=TICKS, at_tick=TICKS))


def _services(builder: Any = core_builder, driver: Any = ShortDriver) -> Services:
    return Services(driver=driver, builder=builder)


def _run_failed(result: RunResult, *fragments: str) -> None:
    score = result.score
    assert score.status == "error"
    assert score.end_reason == "run_error"
    assert not score.ran
    assert result.report is None
    assert score.error is not None
    for fragment in fragments:
        assert fragment in score.error
    for criterion in score.criteria:
        assert criterion.status == "error"
    text = report_text(SuiteScore("s", score.design, (score,)))
    assert "ended: run_error, no recording" in text
    assert f"error: {score.error}" in text


def _lashed(design: Design) -> BuildResult:
    built = core_builder(design)
    assert built.physics is not None
    locked = tuple(
        item.model_copy(update={"self_locking": True}) if isinstance(item, LeadScrew) else item
        for item in built.physics.transmissions
    )
    return built.model_copy(update={"physics": built.physics.model_copy(update={"transmissions": locked})})


def _off_inertia(design: Design) -> BuildResult:
    """The x shaft body's axial inertia 1e-6 relative off the motor's rotor inertia, over the 1e-9 tolerance."""
    built = core_builder(design)
    assert built.physics is not None
    bodies = []
    for body in built.physics.bodies:
        if body.name == "x_motor_shaft":
            inertia = body.inertia.model_copy(update={"izz": body.inertia.izz * (1.0 + 1e-6)})
            body = body.model_copy(update={"inertia": inertia})
        bodies.append(body)
    return built.model_copy(update={"physics": built.physics.model_copy(update={"bodies": tuple(bodies)})})


def _prepared_driver(case: Path, builder: Any = core_builder) -> CoreDriver:
    prepared = prepare(case)
    result = run_case(case, _services(builder), dry=True)
    assert result.config is not None
    driver = CoreDriver()
    driver.load(builder(prepared.design), prepared.case)
    driver.configure(result.config)
    driver.set_initial_state(prepared.case.initial_state)
    for command in result.commands:
        driver.schedule_command(command)
    for fault in result.faults:
        driver.schedule_fault(fault)
    return driver


# ===== Typed errors from run (TST-023) =================================


@pytest.mark.req("TST-023")
@pytest.mark.req("PHY-019")
def test_an_unmappable_design_is_a_driver_error_and_a_run_error_score() -> None:
    driver = _prepared_driver(G3, _lashed)
    with pytest.raises(DriverError, match="self-locking") as raised:
        driver.run(EndSpec(max_tick=TICKS, at_tick=TICKS))
    assert isinstance(raised.value.__cause__, UnmappableElementError)
    _run_failed(run_case(G3, _services(_lashed)), "self-locking")


@pytest.mark.req("TST-023")
def test_a_failing_adapter_load_is_a_driver_error_and_a_run_error_score(monkeypatch: pytest.MonkeyPatch) -> None:
    class Refusing:
        def load(self, prepared: Any, rate: Any) -> None:
            raise core.AdapterError("adapter refused the model")

    monkeypatch.setattr(core_driver.core, "make_physics_adapter", lambda name: Refusing())
    driver = _prepared_driver(P1)
    with pytest.raises(DriverError, match="AdapterError: adapter refused the model") as raised:
        driver.run(EndSpec(max_tick=TICKS, at_tick=TICKS))
    assert isinstance(raised.value.__cause__, core.AdapterError)
    _run_failed(run_case(P1, _services()), "AdapterError", "adapter refused the model")


@pytest.mark.req("TST-023")
def test_a_duplicate_party_name_is_a_driver_error_and_a_run_error_score(monkeypatch: pytest.MonkeyPatch) -> None:
    physics = core.PhysicsParty(core.make_physics_adapter("mujoco"), core.PhysicsPartyConfig()).name()

    class Clashing(core_wiring.RunnerParty):  # type: ignore[misc]
        def name(self) -> str:
            return str(physics)

    monkeypatch.setattr(core_driver, "RunnerParty", Clashing)
    driver = _prepared_driver(G3)
    with pytest.raises(DriverError, match="KernelConfigError") as raised:
        driver.run(EndSpec(max_tick=TICKS, at_tick=TICKS))
    assert isinstance(raised.value.__cause__, core.KernelConfigError)
    _run_failed(run_case(G3, _services()), "KernelConfigError", f"'{physics}'")


@pytest.mark.req("TST-023")
@pytest.mark.req("SIG-037")
def test_a_shaft_inertia_off_the_rotor_inertia_is_refused_by_the_signal_layer() -> None:
    driver = _prepared_driver(G3, _off_inertia)
    with pytest.raises(DriverError, match="SIG-037") as raised:
        driver.run(EndSpec(max_tick=TICKS, at_tick=TICKS))
    assert isinstance(raised.value.__cause__, ValueError)
    _run_failed(run_case(G3, _services(_off_inertia)), "x_motor", "shaft body 'x_motor_shaft'", "SIG-037")


# ===== Rotor inertia from the Python side (SIG-037, BLD-034) ===========


@pytest.mark.req("SIG-037")
@pytest.mark.req("BLD-034")
def test_every_stepper_gets_its_shaft_body_and_axial_inertia() -> None:
    """The builder sets the shaft body's axial inertia to the rotor inertia (BLD-034), so the value passed is the
    catalog's 5.4e-6 kg m^2 to the round-off of a^T I a over a z axis (one product and a few zero sums)."""
    built = core_builder(prepare(G3).design)
    assert built.physics is not None
    places, _ = core_wiring.joint_places(built.physics)
    steppers = [item for item in built.wiring.instances if item.type == "stepper_behavioral"]
    assert sorted(item.name for item in steppers) == ["lift_motor", "x_motor", "y_motor"]
    for instance in steppers:
        params = core_wiring.component_parameters(instance, places, built.physics)
        assert params["shaft_body"] == f"{instance.name}_shaft"
        print(f"{instance.name}: rotor {params['rotor_inertia']!r}, shaft {params['shaft_axial_inertia']!r}")
        assert params["shaft_axial_inertia"] == pytest.approx(params["rotor_inertia"], rel=1e-12)
        assert params["rotor_inertia"] == pytest.approx(5.4e-6, rel=1e-12)


# ===== Approximations and control rate (SCO-013, TST-033) ==============


@pytest.mark.req("SCO-013")
def test_bridge_approximations_reach_the_report_the_score_and_the_suite_text(tmp_path: Path) -> None:
    """ADR 0003: the gantry's belts and screw are finite-stiffness transmissions the bridge maps rigid, so its
    translation report holds lines; each reaches the run report, the score and the printed text."""
    services = Services(ShortDriver, core_builder, RecordArchiver(DESIGNS_ROOT))
    result = run_case(G3, services, out_dir=tmp_path)
    assert result.report is not None
    assert not result.report.errored, result.report.error
    lines = result.report.approximations
    assert len(lines) > 0
    assert any("stiffness" in line for line in lines)
    assert result.score.approximations == lines
    text = report_text(SuiteScore("G3", result.score.design, (result.score,)))
    for line in lines:
        assert f"  approximation: {line}\n" in text
    assert result.archive is not None
    read, _ = RecordArchiveReader().open(result.archive)
    assert read.report.approximations == lines
    pendulum = run_case(P1, _services())
    assert pendulum.score.approximations == ()
    assert "approximation:" not in report_text(SuiteScore("P1", pendulum.score.design, (pendulum.score,)))


@pytest.mark.req("TST-033")
def test_the_archive_records_the_run_configuration_control_rate(tmp_path: Path) -> None:
    services = Services(ShortDriver, core_builder, RecordArchiver(DESIGNS_ROOT))
    result = run_case(G3, services, out_dir=tmp_path)
    assert result.metadata is not None
    assert result.config is not None
    assert result.metadata.control_rate == result.config.control_rate == Fraction(1000)
    driver = ShortDriver()
    prepared = prepare(G3)
    driver.load(core_builder(prepared.design), prepared.case)
    driver.configure(result.config)
    driver.set_initial_state(prepared.case.initial_state)
    driver.run(EndSpec(max_tick=TICKS))
    metadata = replace(result.metadata, control_rate=Fraction(500))
    path = tmp_path / "half.h5"
    RecordArchiver(DESIGNS_ROOT).write(path, driver.recording(), metadata)
    with RunArchive(path) as archive:
        assert archive.identity.control_rate == Fraction(500)
        assert archive.identity.physics_rate == Fraction(1000)
    read, _ = RecordArchiveReader().open(path)
    assert read.control_rate == Fraction(500)
    plain = run_case(P1, Services(ShortDriver, core_builder, RecordArchiver(DESIGNS_ROOT)), out_dir=tmp_path)
    assert plain.metadata is not None
    assert plain.metadata.control_rate is None
    assert plain.archive is not None
    with RunArchive(plain.archive) as archive:
        assert archive.identity.control_rate == Fraction(1000)
    read, _ = RecordArchiveReader().open(plain.archive)
    assert read.control_rate is None
