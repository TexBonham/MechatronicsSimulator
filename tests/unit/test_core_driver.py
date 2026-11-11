"""The core driver, record view and archive pair (TST-012, TST-021, TST-033, TST-038, REC-022, invariant 7)."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import replace
from fractions import Fraction
from pathlib import Path
from typing import Any

import pytest

from mechsim import _core as core
from mechsim.physics.mujoco.prepare import StartStateError, prepare_model
from mechsim.physics.mujoco.translate import UnmappableElementError
from mechsim.physics.neutral import LeadScrew
from mechsim.record import Recorder, RunArchive, archives_identical
from mechsim.record.tick import CouplingSample, RecordError, TickRecord
from mechsim.testing.core_driver import (
    CoreDriver,
    DriverError,
    RecordArchiver,
    RecordArchiveReader,
    core_builder,
    core_services,
    payload_digest,
)
from mechsim.testing.driver import EndSpec, RunConfig, RunMetadata, ScheduledCommand, ScheduledFault
from mechsim.testing.runner import prepare
from mechsim.testing.schema import DESIGNS_ROOT, ROOT, InitialState, JointState
from tests.unit.test_builder_support import gantry, pendulum
from tests.unit.test_record_support import BASE_RATE, finished, identity, synthetic_rows

P1 = ROOT / "test_cases" / "pendulum" / "P1_period.yaml"
G3 = ROOT / "test_cases" / "gantry" / "G3.yaml"
CONFIG = RunConfig(Fraction(1000), None, Fraction(1000), 0, (), None)
H = 0.001
# Pendulum closed form (P1_period.yaml): omega0^2 = m g d / I = 1 * 9.81 * 0.25 / (1/12) = 29.43 s^-2.
OMEGA0_SQUARED = 29.43


def _driver(ticks: int, *subscribers: Any) -> CoreDriver:
    prepared = prepare(P1)
    driver = CoreDriver(subscribers)
    driver.load(core_builder(prepared.design), prepared.case)
    driver.configure(CONFIG)
    driver.set_initial_state(prepared.case.initial_state)
    driver.report = driver.run(EndSpec(max_tick=ticks, at_tick=ticks))  # type: ignore[attr-defined]
    return driver


@pytest.mark.req("TST-012")
@pytest.mark.req("TST-036")
def test_firmware_designs_load_and_refuse_what_milestone_1_does_not_run() -> None:
    prepared = prepare(G3)
    driver = CoreDriver()
    driver.load(core_builder(prepared.design), prepared.case)
    native = RunConfig(Fraction(1000), Fraction(1000), Fraction(1000), 0, (), "native")
    with pytest.raises(DriverError, match="native only"):
        driver.configure(replace(native, controller_mode="emulated"))
    with pytest.raises(DriverError, match="equal the physics rate"):
        driver.configure(replace(native, control_rate=Fraction(3000), base_rate=Fraction(3000)))
    driver.configure(native)
    for kind, owner in (("controller_freeze", "controller"), ("slip", "physics"), ("power_loss", "signals")):
        with pytest.raises(DriverError, match="milestone 2"):
            driver.schedule_fault(ScheduledFault(owner, kind, "x", 1, 2))  # type: ignore[arg-type]
    pendulum_driver = CoreDriver()
    pendulum_driver.load(core_builder(prepare(P1).design), prepare(P1).case)
    with pytest.raises(DriverError, match="no operator link"):
        pendulum_driver.schedule_command(ScheduledCommand(1, "home"))
    with pytest.raises(DriverError, match="no signal layer"):
        pendulum_driver.schedule_fault(ScheduledFault("signals", "sensor_stuck", "x", 1, 2))
    with pytest.raises(DriverError, match="no controller"):
        pendulum_driver.configure(native)


@pytest.mark.req("TST-038")
def test_start_angle_is_placed_exactly_and_read_back_as_the_joint_coordinate() -> None:
    """Semi-implicit Euler from rest: v1 = -h omega0^2 theta0 (damping only shrinks it), theta1 = theta0 + h v1, so
    the first recorded row is within h^2 omega0^2 theta0 = 1.5e-6 rad of the 0.05 rad start; the bound below doubles
    it for the implicit damping term. Joint coordinate and the body's yaw about the world z hinge axis agree to
    round-off (both read the same quaternion)."""
    driver = _driver(5)
    position = driver.recording().series("joint:pivot.position")
    print(f"first row: start 0.05 rad, measured {position.values[0]!r} rad")
    assert abs(position.values[0] - 0.05) <= 2.0 * H * H * OMEGA0_SQUARED * 0.05
    assert position.values[1] < position.values[0]
    velocity = driver.recording().series("joint:pivot.velocity")
    assert velocity.values[0] < 0.0
    assert len(position.times) == 5
    assert position.times[0] == pytest.approx(H)
    body = core_builder(pendulum()).physics.bodies[0].name  # type: ignore[union-attr]
    yaw = driver.recording().series(f"pose:{body}.yaw")
    assert yaw.values[0] == pytest.approx(position.values[0], abs=1e-12)
    with pytest.raises(KeyError):
        driver.recording().series("joint:nothing.position")
    with pytest.raises(KeyError):
        driver.recording().series("actuator:x_motor.position")
    assert driver.recording().controller_states() == ("",) * 5
    assert driver.recording().events() == ()


@pytest.mark.req("TST-012")
def test_unsupported_inputs_stop_before_t0() -> None:
    prepared = prepare(P1)
    driver = CoreDriver()
    driver.load(core_builder(prepared.design), prepared.case)
    with pytest.raises(DriverError, match="no controller"):
        driver.configure(replace(CONFIG, control_rate=Fraction(1000)))
    with pytest.raises(DriverError, match="base rate"):
        driver.configure(replace(CONFIG, base_rate=Fraction(2000)))
    driver.configure(CONFIG)
    with pytest.raises(DriverError, match="start velocity"):
        driver.set_initial_state(InitialState(joints=[JointState(joint="pivot", position="0 rad", velocity="1 rad/s")]))
    with pytest.raises(DriverError, match="operator link"):
        driver.schedule_command(ScheduledCommand(1, "home"))
    with pytest.raises(DriverError, match="KER-023"):
        driver.schedule_fault(ScheduledFault("physics", "slip", "x_belt", 1, 2))
    driver.set_initial_state(InitialState(joints=[JointState(joint="nothing", position="0 rad", velocity="0 rad/s")]))
    with pytest.raises(DriverError, match="no such joint"):
        driver.run(EndSpec(max_tick=1, at_tick=1))
    with pytest.raises(DriverError, match="controller"):
        driver.run(EndSpec(max_tick=1, message="ready_fault"))
    with pytest.raises(DriverError, match="before run"):
        CoreDriver().recording()


@pytest.mark.req("TST-033")
@pytest.mark.req("REC-024")
def test_archive_round_trip_without_the_engine(tmp_path: Path) -> None:
    driver = _driver(20)
    metadata = RunMetadata(
        "P1-seed0",
        "P1",
        "pendulum",
        "0.1.0",
        "unset",
        "unset",
        0,
        None,
        None,
        driver.report,
        (("a", "1 m"),),  # type: ignore[attr-defined]
    )
    path = tmp_path / "p1.h5"
    digest = RecordArchiver(DESIGNS_ROOT).write(path, driver.recording(), metadata)
    assert len(digest) == 64
    read, view = RecordArchiveReader().open(path)
    assert read == metadata
    assert view.times() == driver.recording().times()
    assert view.series("joint:pivot.position") == driver.recording().series("joint:pivot.position")
    # The pendulum maps exactly, so the bridge reports no approximation; the list still rides in the archive.
    assert view.approximations == driver.recording().approximations == ()
    with RunArchive(path) as archive:
        assert archive.identity.controller_mode == "none"
        assert archive.identity.design_file == (DESIGNS_ROOT / "pendulum" / "design.yaml").read_bytes()
        assert archive.extra["end_reason"] == "end_time"


@pytest.mark.req("REC-019")
def test_a_raising_subscriber_reaches_the_run_report_and_the_archive(tmp_path: Path) -> None:
    """Invariant 7 (audit note): a subscriber error is not only logged; the run is scored as a run error."""

    def boom(row: Any) -> None:
        raise ValueError("subscriber boom")

    driver = _driver(3, boom)
    report = driver.report  # type: ignore[attr-defined]
    assert report.errored
    assert "recorder" in report.error
    assert "subscriber boom" in report.error
    assert any("subscriber boom" in error for error in driver.subscriber_errors)
    metadata = RunMetadata("r", "P1", "pendulum", "0.1.0", "unset", "unset", 0, None, None, report)
    path = tmp_path / "e.h5"
    RecordArchiver(DESIGNS_ROOT).write(path, driver.recording(), metadata)
    read, _ = RecordArchiveReader().open(path)
    assert read.report.error is not None
    assert "subscriber boom" in read.report.error


@pytest.mark.req("REC-028")
def test_payload_digest_is_canonical() -> None:
    first = core.PhysicsInputs()
    first.generalized_forces = [1.0, 2.0]
    second = core.PhysicsInputs()
    second.generalized_forces = [1.0, 2.0]
    third = core.PhysicsInputs()
    third.generalized_forces = [1.0, math.nextafter(2.0, 3.0)]
    assert payload_digest(first) == payload_digest(second)
    assert payload_digest(first) != payload_digest(third)
    assert len(payload_digest(first)) == 64


def _with_coupling(rows: list[TickRecord], samples: list[tuple[CouplingSample, ...]]) -> list[TickRecord]:
    return [replace(row, coupling=coupling) for row, coupling in zip(rows, samples, strict=True)]


@pytest.mark.req("REC-028")
@pytest.mark.req("REC-022")
@pytest.mark.req("REC-024")
def test_coupling_series_are_archived_replayed_and_compared(tmp_path: Path) -> None:
    rows = synthetic_rows(7, 4)
    samples = [
        (
            CouplingSample("a.level", "none"),
            CouplingSample("b.payload", "payload", float("nan"), payload_digest([index])),
        )
        if index == 0
        else (
            CouplingSample("a.level", "real", 0.5 * index),
            CouplingSample("b.payload", "payload", float("nan"), payload_digest([index])),
        )
        for index in range(4)
    ]
    rows = _with_coupling(rows, samples)

    def flush(path: Path, written: list[TickRecord]) -> Path:
        recorder = Recorder(BASE_RATE)
        for row in written:
            recorder.write_row(row)
        recorder.flush(path, identity(), finished(len(written)))
        return path

    first = flush(tmp_path / "a.h5", rows)
    with RunArchive(first) as archive:
        assert archive.coupling() == (("a.level", "real"), ("b.payload", "payload"))
        replayed = list(archive.records())
    assert replayed[0].coupling[0].kind == "none"
    assert replayed[2].coupling[0] == CouplingSample("a.level", "real", 1.0)
    assert replayed[3].coupling[1].digest == samples[3][1].digest
    assert replayed[3].coupling[1].kind == "payload"
    assert math.isnan(replayed[3].coupling[1].value)
    changed = list(rows)
    changed[2] = replace(
        rows[2],
        coupling=(rows[2].coupling[0], CouplingSample("b.payload", "payload", float("nan"), payload_digest(["x"]))),
    )
    report = archives_identical(first, flush(tmp_path / "b.h5", changed))
    assert not report.content_identical
    assert report.first_difference == "dataset coupling/digest/b.payload: first difference at index (2,)"


@pytest.mark.req("REC-028")
def test_coupling_rows_must_keep_names_and_kinds() -> None:
    rows = synthetic_rows(7, 3)
    recorder = Recorder(BASE_RATE)
    recorder.write_row(replace(rows[0], coupling=(CouplingSample("a.level", "real", 1.0),)))
    with pytest.raises(RecordError, match="changed kind"):
        recorder.write_row(replace(rows[1], coupling=(CouplingSample("a.level", "flag", 1.0),)))
    other = Recorder(BASE_RATE)
    with pytest.raises(RecordError, match="name order"):
        other.write_row(replace(rows[0], coupling=(CouplingSample("b", "real", 1.0), CouplingSample("a", "real", 1.0))))
    with pytest.raises(RecordError, match="digest"):
        Recorder(BASE_RATE).write_row(replace(rows[0], coupling=(CouplingSample("a", "payload", 0.0, "short"),)))
    with pytest.raises(RecordError, match="unknown kind"):
        Recorder(BASE_RATE).write_row(replace(rows[0], coupling=(CouplingSample("a", "text", 0.0),)))
    with pytest.raises(RecordError, match="no digest"):
        Recorder(BASE_RATE).write_row(replace(rows[0], coupling=(CouplingSample("a", "real", 0.0, "x"),)))


def test_services_wire_the_real_driver() -> None:
    services = core_services()
    assert isinstance(services.driver(), CoreDriver)
    assert isinstance(services.archiver, RecordArchiver)


@pytest.mark.req("TST-038")
def test_prepared_model_keeps_the_report_and_places_starts_exactly() -> None:
    """ADR 0003: the gantry's belts and screw are finite-stiffness transmissions the bridge maps rigid, so the report
    is not empty. A slide start shifts the child body along its axis by the start and sets `ref`; the shift is
    exact (one product of SI floats)."""
    built = core_builder(gantry()).physics
    assert built is not None
    assert [item.backlash for item in built.transmissions] == [0.0, 0.0, 0.0]
    locked = tuple(
        item.model_copy(update={"self_locking": True}) if isinstance(item, LeadScrew) else item
        for item in built.transmissions
    )
    with pytest.raises(UnmappableElementError, match="self-locking"):
        prepare_model(built.model_copy(update={"transmissions": locked}))
    physics = built
    plain = prepare_model(physics)
    assert len(plain.report) > 0
    placed = prepare_model(physics, {"x_axis": 0.3})
    assert placed.report == plain.report
    root = ET.fromstring(placed.mjcf)
    body = next(element for element in root.iter("body") if element.get("name") == "rail_y")
    joint = next(element for element in body.findall("joint") if element.get("name") == "x_axis")
    assert joint.get("ref") == "0.3"
    position = [float(value) for value in (body.get("pos") or "").split()]
    original = next(element for element in ET.fromstring(plain.mjcf).iter("body") if element.get("name") == "rail_y")
    before = [float(value) for value in (original.get("pos") or "").split()]
    assert position[0] - before[0] == pytest.approx(0.3, abs=1e-15)
    assert position[1:] == before[1:]
    with pytest.raises(StartStateError, match="reflected shaft"):
        prepare_model(physics, {"x_motor_spin": 0.1})
    with pytest.raises(StartStateError, match="spherical"):
        prepare_model(physics, {"hook_swing": 0.1})
