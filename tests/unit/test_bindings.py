"""The Python binding of the core runtime, mechsim._core_ext (ADR 0005), through the mechsim._core shim.

Each test cites the interface row it exercises through the binding; the C++ behaviour behind it has its own Catch2
tests under core/.
"""

import gc
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mechsim import _core as core
from mechsim.physics.native import _engine as eng


class RecordingParty(core.Party):  # type: ignore[misc]
    """Appends (name, tick, dt) to a shared log on every step, and the fault it is told about."""

    def __init__(self, name: str, log: list[Any], rate: Any = None, fail_at: int | None = None) -> None:
        super().__init__()
        self._name = name
        self._log = log
        self._rate = core.Rate(1000)
        if rate is not None:
            self._rate = rate
        self._fail_at = fail_at

    def name(self) -> str:
        return self._name

    def nominal_rate(self) -> Any:
        return self._rate

    def published(self) -> list[str]:
        return [f"{self._name}.out"]

    def consumed(self) -> list[str]:
        return []

    def start(self, base_rate: Any) -> None:
        self._log.append((self._name, "start", base_rate))

    def step(self, t: Any, dt: Any) -> None:
        if self._fail_at is not None and t.count() == self._fail_at:
            raise ValueError("boom")
        self._log.append((self._name, t.count(), dt.count()))

    def on_fault(self, fault: Any) -> None:
        self._log.append((self._name, "fault", fault.party, fault.time.count()))


def run_kernel(parties: list[Any], end: Any, sink: Any | None = None, config: Any | None = None) -> Any:
    if config is None:
        config = core.RunConfig()
    if sink is None:
        sink = core.BufferedRecorderSink()
    kernel = core.make_kernel(config)
    for party in parties:
        kernel.register_party(party)
    return kernel.run(core.EndCondition(end), sink)


def test_import_exposes_extension() -> None:
    assert core.extension.__name__ == "mechsim._core_ext"
    assert core.Kernel is core.extension.Kernel
    assert core.PythonSlowPathComponent is core.SignalComponent


def test_shim_names_the_build_step_when_extension_missing() -> None:
    with pytest.raises(core.CoreNotBuiltError, match="uv sync"):
        core.load_extension("mechsim._no_such_extension")


@pytest.mark.req("KER-019")
@pytest.mark.req("KER-013")
@pytest.mark.req("KER-018")
def test_python_parties_step_in_registration_order() -> None:
    log: list[Any] = []
    parties = [RecordingParty(name, log) for name in ("physics", "sensors", "controller")]
    report = run_kernel(parties, core.Seconds(3, 1000))
    assert report.outcome == core.RunOutcome.end_condition_met
    assert report.ticks == 3
    assert report.fault is None
    starts = [entry for entry in log if entry[1] == "start"]
    assert [entry[0] for entry in starts] == ["physics", "sensors", "controller"]
    assert all(entry[2] == core.Rate(1000) for entry in starts)
    steps = [entry for entry in log if entry[1] != "start"]
    expected = [(name, tick, 1) for tick in range(3) for name in ("physics", "sensors", "controller")]
    assert steps == expected


@pytest.mark.req("KER-019")
@pytest.mark.req("KER-003")
def test_python_party_runs_at_its_own_rate() -> None:
    # Physics 1 kHz and a 500 Hz party: base rate 1 kHz, the slow party has period 2 ticks.
    log: list[Any] = []
    report = run_kernel([RecordingParty("slow", log, rate=core.Rate(500))], core.Seconds(4, 1000))
    assert report.base_rate == core.Rate(1000)
    assert [entry for entry in log if entry[1] != "start"] == [("slow", 0, 2), ("slow", 2, 2)]


@pytest.mark.req("KER-015")
@pytest.mark.req("KER-017")
def test_python_exception_becomes_party_fault() -> None:
    log: list[Any] = []
    parties = [RecordingParty("first", log), RecordingParty("bad", log, fail_at=2), RecordingParty("last", log)]
    report = run_kernel(parties, core.Seconds(5, 1000))
    assert report.outcome == core.RunOutcome.run_error
    fault = report.fault
    assert isinstance(fault, core.PartyFault)
    assert isinstance(fault, RuntimeError)
    assert fault.party == "bad"
    assert fault.time == core.SimTime(2)
    assert fault.cause == "ValueError: boom"
    assert report.end_time == core.SimTime(2)
    faults = [entry for entry in log if entry[1] == "fault"]
    assert faults == [("first", "fault", "bad", 2), ("bad", "fault", "bad", 2), ("last", "fault", "bad", 2)]


@pytest.mark.req("KER-015")
def test_python_party_fault_keeps_its_cause() -> None:
    class Faulting(RecordingParty):
        def step(self, t: Any, dt: Any) -> None:
            raise core.PartyFault("ignored message")

    report = run_kernel([Faulting("p", [])], core.Seconds(1, 1000))
    assert report.fault.party == "p"
    assert report.fault.time == core.SimTime(0)
    assert report.fault.cause == "ignored message"


@pytest.mark.req("KER-018")
def test_party_missing_a_method_faults_the_run() -> None:
    class Nameless(core.Party):  # type: ignore[misc]
        pass

    kernel = core.make_kernel(core.RunConfig())
    with pytest.raises(RuntimeError, match="name"):
        kernel.register_party(Nameless())


def test_kernel_config_errors_map_to_value_error() -> None:
    with pytest.raises(core.KernelConfigError) as caught:
        core.Rate(0)
    assert isinstance(caught.value, ValueError)
    log: list[Any] = []
    kernel = core.make_kernel(core.RunConfig())
    kernel.register_party(RecordingParty("same", log))
    with pytest.raises(core.KernelConfigError, match="duplicate"):
        kernel.register_party(RecordingParty("same", log))
    with pytest.raises(core.KernelConfigError):
        kernel.run(core.EndCondition(core.Seconds(1, 3000)), core.BufferedRecorderSink())


@pytest.mark.req("KER-017")
def test_stop_callback_ends_the_run_early() -> None:
    kernel = core.make_kernel(core.RunConfig())
    kernel.register_party(RecordingParty("p", []))
    end = core.EndCondition(core.Seconds(1), stop=lambda t: t.count() >= 3)
    report = kernel.run(end, core.BufferedRecorderSink())
    assert report.outcome == core.RunOutcome.end_condition_met
    assert report.end_time == core.SimTime(3)
    assert kernel.now() == core.SimTime(3)


def test_time_types() -> None:
    assert core.Rate(3000, 3) == core.Rate(1000)
    assert core.lcm(core.Rate(1000), core.Rate(3000)) == core.Rate(3000)
    assert core.period_in_ticks(core.Rate(500), core.Rate(1000)) == core.Duration(2)
    assert core.ticks_at(core.Seconds(1, 100), core.Rate(1000)) == core.SimTime(10)
    assert core.to_seconds(core.SimTime(5), core.Rate(1000)) == pytest.approx(5e-3, rel=0, abs=1e-15)
    assert core.SimTime(2) + core.Duration(3) == core.SimTime(5)
    assert core.SimTime(5) - core.SimTime(2) == core.Duration(3)
    config = core.RunConfig()
    config.party_rates = {"slow": core.Rate(1, 10)}
    assert config.party_rates["slow"].denominator == 10


@pytest.mark.req("KER-022")
@pytest.mark.req("REC-019")
@pytest.mark.req("REC-020")
def test_recorder_sink_drains_rows_written_by_the_kernel() -> None:
    sink = core.BufferedRecorderSink()
    seen: list[int] = []

    def watch(row: Any) -> None:
        seen.append(row.time.count())
        row.time = core.SimTime(-1)

    subscription = sink.subscribe(watch)
    run_kernel([RecordingParty("p", [])], core.Seconds(5, 1000), sink=sink)
    assert seen == [1, 2, 3, 4, 5]
    assert sink.pending == 5
    rows = sink.drain_dicts()
    assert [row["time"] for row in rows] == [1, 2, 3, 4, 5]
    assert rows[0]["physics"]["bodies"] == []
    assert rows[0]["events"] == []
    assert sink.pending == 0
    sink.unsubscribe(subscription)
    assert sink.subscriber_count == 0


@pytest.mark.req("KER-022")
def test_recorder_sink_drains_numpy_columns() -> None:
    sink = core.BufferedRecorderSink()
    run_kernel([RecordingParty("p", [])], core.Seconds(4, 1000), sink=sink)
    arrays = sink.drain_arrays()
    assert arrays["time"].dtype == np.int64
    assert arrays["time"].tolist() == [1, 2, 3, 4]
    assert arrays["body_position"].shape == (4, 0, 3)
    assert arrays["body_orientation"].shape == (4, 0, 4)
    assert arrays["rope_tension"].shape == (4, 0)
    assert arrays["pin_signals"] == {}
    empty = sink.drain_arrays()
    assert empty["time"].shape == (0,)


def test_recorder_series_columns_fill_missing_with_nan() -> None:
    sink = core.BufferedRecorderSink()
    first = core.TickRecord()
    sample = core.SeriesSample()
    sample.name = "limit.level"
    sample.unit = "1"
    sample.value = 1.0
    first.pin_signals = [sample]
    sink.write_row(first)
    sink.write_row(core.TickRecord())
    column = sink.drain_arrays()["pin_signals"]["limit.level"]
    assert column[0] == 1.0
    assert math.isnan(column[1])


def test_recorder_rejects_arrays_when_body_count_changes() -> None:
    sink = core.BufferedRecorderSink()
    row = core.TickRecord()
    physics = core.PhysicsOutputs()
    physics.bodies = [core.BodyState()]
    row.physics = physics
    sink.write_row(row)
    sink.write_row(core.TickRecord())
    with pytest.raises(ValueError, match="drain as dicts"):
        sink.drain_arrays()


@pytest.mark.req("KER-022")
def test_python_recorder_sink_receives_rows() -> None:
    class ListSink(core.RecorderSink):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__()
            self.times: list[int] = []

        def write_row(self, row: Any) -> None:
            self.times.append(row.time.count())

        def subscribe(self, callback: Any) -> Any:
            return core.SubscriptionId(1)

        def unsubscribe(self, subscription: Any) -> None:
            pass

    sink = ListSink()
    run_kernel([RecordingParty("p", [])], core.Seconds(3, 1000), sink=sink)
    assert sink.times == [1, 2, 3]


@pytest.mark.req("INV-004")
def test_run_twice_gives_identical_rows() -> None:
    def once() -> list[Any]:
        sink = core.BufferedRecorderSink()
        parties = [RecordingParty("a", []), RecordingParty("b", [], rate=core.Rate(250))]
        run_kernel(parties, core.Seconds(8, 1000), sink=sink)
        return list(sink.drain_dicts())

    assert once() == once()


@pytest.mark.req("PHY-001")
@pytest.mark.req("PHY-002")
def test_null_adapter_loads_and_returns_zero_outputs() -> None:
    assert "null" in core.physics_adapter_names()
    adapter = core.make_physics_adapter("null")
    assert adapter.supported_elements() == []
    adapter.load(core.PreparedModel(neutral_model_json="{}", design_version="1.2.3"), core.Rate(1000))
    assert adapter.design_version == "1.2.3"
    adapter.step()
    adapter.step()
    outputs = adapter.outputs()
    assert outputs.steps == 2
    assert outputs.bodies == []
    assert outputs.joints == []
    assert outputs.members == []
    health = adapter.health()
    assert health.steps == 2
    assert health.physics_rate == core.Rate(1000)
    assert health.max_constraint_drift == 0.0


@pytest.mark.req("PHY-009")
def test_adapter_misuse_raises_adapter_error() -> None:
    adapter = core.make_physics_adapter("null")
    with pytest.raises(core.AdapterError) as caught:
        adapter.step()
    assert caught.value.step == 0
    assert "before load" in caught.value.cause
    adapter.load(core.PreparedModel(), core.Rate(1000))
    with pytest.raises(core.AdapterError, match="twice"):
        adapter.load(core.PreparedModel(), core.Rate(1000))
    with pytest.raises(core.AdapterError, match="spring 3"):
        adapter.set_spring_offset(3, 0.1)
    with pytest.raises(ValueError, match="unknown physics adapter 'nope'; known: mujoco, native, null"):
        core.make_physics_adapter("nope")


@pytest.mark.req("PHY-001")
def test_python_adapter_subclass_implements_the_interface() -> None:
    class StubAdapter(core.PhysicsAdapter):  # type: ignore[misc]
        def supported_elements(self) -> list[str]:
            return ["body"]

        def step(self) -> None:
            raise core.AdapterError("drift")

    adapter = StubAdapter()
    assert adapter.supported_elements() == ["body"]


class Doubler(core.SignalComponent):  # type: ignore[misc]
    """A Python plugin: output = 2 * input + completed physics steps, latched."""

    def __init__(self) -> None:
        super().__init__()
        self._steps = 0
        self._inputs: list[float] = [0.0]
        self._latched: list[float] = [0.0]
        self.injections: list[Any] = []

    def instance_name(self) -> str:
        return "doubler"

    def inputs(self) -> list[str]:
        return ["doubler.in"]

    def outputs(self) -> list[str]:
        return ["doubler.out"]

    def sample(self, state: Any) -> None:
        self._steps = state.steps

    def set_inputs(self, values: list[float]) -> None:
        self._inputs = list(values)

    def latch_outputs(self) -> None:
        self._latched = [2.0 * self._inputs[0] + self._steps]

    def output_values(self) -> list[float]:
        return self._latched

    def inject(self, injection: Any) -> None:
        self.injections.append(injection.mode)


@pytest.mark.req("SIG-030")
@pytest.mark.req("SIG-023")
def test_python_signal_component_round_trips_a_sample() -> None:
    plugin = Doubler()
    state = core.PhysicsOutputs()
    state.steps = 7
    assert core.signal_component_tick(plugin, state, [1.5]) == [10.0]
    table = core.SignalTable()
    table.add(plugin)
    assert "doubler.in" in table
    assert "doubler.out" in table
    assert len(table) == 2
    with pytest.raises(core.SignalNameError, match=r"doubler\.in"):
        table.add(plugin)


@pytest.mark.req("SIG-030")
def test_null_component_runs_through_the_same_path() -> None:
    component = core.NullSignalComponent("limit", ["limit.in"], ["limit.level"])
    state = core.PhysicsOutputs()
    state.steps = 4
    assert core.signal_component_tick(component, state, [1.0]) == [0.0]
    assert component.samples == 1
    assert component.latches == 1
    assert component.last_sample_step == 4
    with pytest.raises(ValueError):  # noqa: PT011
        core.signal_component_tick(component, state, [1.0, 2.0])
    with pytest.raises(core.SignalNameError):
        core.NullSignalComponent("limit", ["other.in"], [])


@pytest.mark.req("SIG-023")
@pytest.mark.req("SIG-024")
def test_pin_binding_table() -> None:
    table = core.PinBindingTable()
    table.add(core.PinBinding("limit.level", "PA0", core.PeripheralMode.digital_input))
    table.add(core.PinBinding("encoder.count", "PB3", core.PeripheralMode.counter_input))
    found = table.find_signal("encoder.count")
    assert found is not None
    assert found.pin == "PB3"
    assert table.find_pin("PC9") is None
    assert [entry.signal for entry in table.entries()] == ["limit.level", "encoder.count"]
    with pytest.raises(core.PinBindingError):
        table.add(core.PinBinding("other.level", "PA0"))
    with pytest.raises(core.PinBindingError):
        table.add(core.PinBinding("undotted", "PA1"))
    assert core.is_dotted_signal_name("a.b")
    assert core.signal_instance("a.b") == "a"


def test_failure_injection_validation() -> None:
    injection = core.FailureInjection()
    injection.mode = core.FailureMode.noisy
    injection.start = core.SimTime(5)
    injection.end = core.SimTime(5)
    with pytest.raises(ValueError):  # noqa: PT011
        core.validate_injection(injection)
    injection.end = core.SimTime(6)
    core.validate_injection(injection)
    plugin = Doubler()
    plugin.inject(injection)
    assert plugin.injections == [core.FailureMode.noisy]


@pytest.mark.req("CTL-001")
def test_null_controller_runtime_from_factory() -> None:
    assert "null" in core.controller_runtime_names()
    runtime = core.make_controller_runtime("null", core.Rate(1000))
    with pytest.raises(core.ControllerConfigError):
        runtime.load(core.FirmwareImage("fw.so", 500))
    runtime.load(core.FirmwareImage("fw.so", 1000))
    runtime.reset(core.PowerRail("main", 24.0, True))
    inputs = core.PinInputs()
    outputs = core.PinOutputs()
    outputs.step_count = [5] * core.stepper_channels
    runtime.tick(inputs, outputs)
    assert outputs.step_count == [0] * core.stepper_channels
    assert runtime.ticks_run == 1
    assert runtime.resets == 1
    assert runtime.firmware_path == "fw.so"
    assert len(inputs.digital_in) == core.digital_in_channels


def test_engine_invalid_model_maps_to_engine_error() -> None:
    model = eng.MultibodyModel()
    body = eng.Body()
    body.mass = -1.0
    model.bodies = [body]
    with pytest.raises(core.InvalidModelError) as caught:
        eng.MultibodyProvider(model)
    assert isinstance(caught.value, core.EngineError)
    assert caught.value.time == 0.0
    assert caught.value.cause


# Pendulum fixture from core/engine/tests/test_reference_pendulum.cpp and multibody_fixtures.hpp: a uniform rod of
# length L = 0.5 m and mass m = 1 kg (radius 0.01 m) on a revolute pivot at its end, g = 9.81 m/s^2 along -y.
ROD_LENGTH = 0.5
ROD_MASS = 1.0
ROD_RADIUS = 0.01
GRAVITY = 9.81


def hanging_rod() -> Any:
    body = eng.Body()
    body.name = "rod"
    body.parent = -1
    joint = eng.Joint()
    joint.type = eng.JointType.Revolute
    joint.position = np.zeros(3)
    joint.axis = np.array([0.0, 0.0, 1.0])
    joint.orientation = [math.cos(-math.pi / 4.0), 0.0, 0.0, math.sin(-math.pi / 4.0)]
    body.joint = joint
    body.mass = ROD_MASS
    body.com = np.array([0.5 * ROD_LENGTH, 0.0, 0.0])
    transverse = ROD_MASS * ROD_LENGTH**2 / 12.0
    body.inertia = np.diag([0.5 * ROD_MASS * ROD_RADIUS**2, transverse, transverse])
    model = eng.MultibodyModel()
    model.gravity = np.array([0.0, -GRAVITY, 0.0])
    model.bodies = [body]
    return model


@pytest.mark.req("ENG-036")
def test_engine_pendulum_period_from_python() -> None:
    # Closed form for the compound pendulum: T0 = 2 pi sqrt(I / (m g d)), I = m L^2 / 3 about the pivot, d = L / 2.
    # Finite amplitude theta0 = 0.05: T = T0 (1 + theta0^2 / 16 + O(theta0^4)), the O(theta0^4) term 2e-8. The scheme
    # (semi-implicit Euler, gravity explicit) gives T_num / T = Omega / (2 asin(Omega / 2)), Omega = omega h, about
    # 1 - Omega^2 / 24. ENG-036 holds the raw period to 0.1 % of T0; the corrected residual is held to 10 % of the
    # amplitude correction, the same bounds as the C++ reference test, so the binding must not change the numbers.
    theta0 = 0.05
    inertia = ROD_MASS * ROD_LENGTH**2 / 3.0
    weight_arm = ROD_MASS * GRAVITY * 0.5 * ROD_LENGTH
    omega = math.sqrt(weight_arm / inertia)
    closed_form = 2.0 * math.pi * math.sqrt(inertia / weight_arm)
    provider = eng.MultibodyProvider(hanging_rod())
    integrator = eng.SemiImplicitEuler(eng.StepConfig())
    h = integrator.config.h
    state = eng.State(np.array([-theta0]), np.zeros(1), 0.0)
    crossings: list[float] = []
    for _ in range(math.ceil(11.0 * closed_form / h)):
        before = float(state.q[0])
        t0 = state.t
        provider.step(integrator, state)
        after = float(state.q[0])
        if before < 0.0 <= after:
            crossings.append(t0 + h * before / (before - after))
    assert len(crossings) >= 10
    measured = (crossings[-1] - crossings[0]) / (len(crossings) - 1)
    omega_h = omega * h
    amplitude = theta0**2 / 16.0
    scheme = omega_h / (2.0 * math.asin(0.5 * omega_h))
    corrected = measured / (closed_form * (1.0 + amplitude) * scheme) - 1.0
    raw = measured / closed_form - 1.0
    print(f"ENG-036 via binding: closed form {closed_form:.9f} s, measured {measured:.9f} s, raw {raw:.3e}")
    assert abs(raw) <= 1e-3
    assert abs(corrected) <= 0.1 * amplitude
    assert len(provider.outputs.bodies) == 1


@pytest.mark.req("ENG-022")
def test_lumped_cantilever_from_python() -> None:
    # Header closed forms (lumped_beam.hpp): N segments of l = L / N, node spring k = E I / l at (j + 1/2) l, one
    # rigid body per spring node, a clamped stub [0, l / 2] in the parent; x = 0 is a point of the parent (world).
    spec = eng.BeamSpec()
    spec.youngs_modulus = 2.0e11
    spec.area_moment = 1.0e-8
    spec.density = 7850.0
    spec.area = 1.0e-4
    spec.length = 1.0
    spec.segments = 4
    model = eng.MultibodyModel()
    beam = eng.add_lumped_beam(model, spec)
    segment = spec.length / spec.segments
    assert beam.node_stiffness == pytest.approx(spec.youngs_modulus * spec.area_moment / segment, rel=1e-15)
    assert len(beam.bodies) == spec.segments
    assert len(model.bodies) == spec.segments
    assert beam.starts == pytest.approx([(j + 0.5) * segment for j in range(spec.segments)], rel=1e-15)
    tip = eng.beam_point(beam, spec.length)
    assert tip.body == beam.bodies[-1]
    assert tip.point == pytest.approx([spec.length - beam.starts[-1], 0.0, 0.0], abs=1e-15)
    root = eng.beam_point(beam, 0.0)
    assert root.body == -1
    with pytest.raises(core.InvalidModelError):
        eng.beam_point(beam, 2.0 * spec.length)
    eng.MultibodyProvider(model)


def test_engine_is_not_reexported_by_the_shim() -> None:
    with pytest.raises(AttributeError, match="invariant 5"):
        _ = core.engine
    assert "MultibodyProvider" not in dir(core)
    assert eng.engine.__name__ == "mechsim._core_ext.engine"


@pytest.mark.req("KER-019")
def test_party_owned_only_by_the_kernel_still_steps() -> None:
    log: list[Any] = []
    kernel = core.make_kernel(core.RunConfig())
    party = RecordingParty("orphan", log)
    kernel.register_party(party)
    del party
    gc.collect()
    report = kernel.run(core.EndCondition(core.Seconds(2, 1000)), core.BufferedRecorderSink())
    assert report.ticks == 2
    assert [entry for entry in log if entry[1] != "start"] == [("orphan", 0, 1), ("orphan", 1, 1)]


def test_results_outlive_kernel_and_sink() -> None:
    kernel = core.make_kernel(core.RunConfig())
    kernel.register_party(RecordingParty("bad", [], fail_at=1))
    sink = core.BufferedRecorderSink()
    report = kernel.run(core.EndCondition(core.Seconds(3, 1000)), sink)
    config = kernel.config
    rows = sink.drain_dicts()
    del kernel, sink
    gc.collect()
    assert report.fault.party == "bad"
    assert report.fault.time == core.SimTime(1)
    assert [row["time"] for row in rows] == [1]
    assert config.base_rate == core.Rate(1000)


@pytest.mark.req("KER-017")
def test_zero_tick_run_starts_parties_and_writes_nothing() -> None:
    log: list[Any] = []
    sink = core.BufferedRecorderSink()
    report = run_kernel([RecordingParty("p", log)], core.Seconds(0), sink=sink)
    assert report.outcome == core.RunOutcome.end_condition_met
    assert report.ticks == 0
    assert report.end_time == core.SimTime(0)
    assert [entry[1] for entry in log] == ["start"]
    assert sink.pending == 0


def test_wrong_types_raise() -> None:
    kernel = core.make_kernel(core.RunConfig())
    with pytest.raises(TypeError):
        kernel.register_party(object())
    with pytest.raises(TypeError):
        eng.State("x", np.zeros(1))
    with pytest.raises(core.KernelConfigError):
        core.Rate(1, 0)


@pytest.mark.req("PHY-009")
def test_python_adapter_error_crosses_the_binding() -> None:
    class StubAdapter(core.PhysicsAdapter):  # type: ignore[misc]
        def step(self) -> None:
            error = core.AdapterError("physics at step 3: drift")
            error.step = 3
            error.cause = "drift"
            raise error

    with pytest.raises(core.AdapterError) as caught:
        core.physics_adapter_step(StubAdapter())
    assert caught.value.step == 3
    assert caught.value.cause == "drift"


@pytest.mark.req("KER-015")
def test_raising_on_fault_is_reported_and_the_run_still_ends(monkeypatch: pytest.MonkeyPatch) -> None:
    unraisable: list[str] = []
    monkeypatch.setattr(sys, "unraisablehook", lambda info: unraisable.append(str(info.exc_value)))

    class LoudFault(RecordingParty):
        def on_fault(self, fault: Any) -> None:
            raise RuntimeError("on_fault failed")

    log: list[Any] = []
    parties = [LoudFault("loud", log), RecordingParty("bad", log, fail_at=0)]
    report = run_kernel(parties, core.Seconds(2, 1000))
    assert report.outcome == core.RunOutcome.run_error
    assert report.fault.party == "bad"
    assert unraisable == ["on_fault failed"]
    assert ("bad", "fault", "bad", 0) in log


@pytest.mark.req("KER-018")
def test_on_fault_is_pure_like_the_other_hooks(monkeypatch: pytest.MonkeyPatch) -> None:
    unraisable: list[str] = []
    monkeypatch.setattr(sys, "unraisablehook", lambda info: unraisable.append(str(info.exc_value)))

    class Silent(core.Party):  # type: ignore[misc]
        def name(self) -> str:
            return "silent"

        def nominal_rate(self) -> Any:
            return core.Rate(1000)

        def published(self) -> list[str]:
            return []

        def consumed(self) -> list[str]:
            return []

        def start(self, base_rate: Any) -> None:
            pass

        def step(self, t: Any, dt: Any) -> None:
            raise ValueError("stop")

    with pytest.raises(RuntimeError, match="on_fault"):
        Silent().on_fault(None)
    report = run_kernel([Silent()], core.Seconds(1, 1000))
    assert report.fault.cause == "ValueError: stop"
    assert unraisable == ["Python subclass does not implement on_fault()"]


@pytest.mark.req("KER-022")
def test_drain_arrays_with_bodies_and_ropes() -> None:
    sink = core.BufferedRecorderSink()
    for k in range(2):
        body = core.BodyState()
        body.position = [1.0 + k, 2.0, 3.0]
        rope = core.RopeTension()
        rope.tension = 10.0 * (k + 1)
        physics = core.PhysicsOutputs()
        physics.steps = k + 1
        physics.bodies = [body]
        physics.ropes = [rope]
        row = core.TickRecord()
        row.time = core.SimTime(k + 1)
        row.physics = physics
        sink.write_row(row)
    arrays = sink.drain_arrays()
    assert arrays["steps"].tolist() == [1, 2]
    assert arrays["body_position"].shape == (2, 1, 3)
    assert arrays["body_position"][:, 0, :].tolist() == [[1.0, 2.0, 3.0], [2.0, 2.0, 3.0]]
    assert arrays["body_orientation"][:, 0, :].tolist() == [[1.0, 0.0, 0.0, 0.0]] * 2
    assert arrays["rope_tension"].tolist() == [[10.0], [20.0]]


@pytest.mark.req("KER-015")
def test_python_party_fault_attributes_are_replaced_by_the_kernel() -> None:
    # KER-015: the kernel names the failing party and the step's start time itself; only the cause is the party's.
    class Explicit(RecordingParty):
        def step(self, t: Any, dt: Any) -> None:
            fault = core.PartyFault("message")
            fault.party = "someone_else"
            fault.time = core.SimTime(7)
            fault.cause = "explicit cause"
            raise fault

    report = run_kernel([Explicit("p", [])], core.Seconds(1, 1000))
    assert report.fault.party == "p"
    assert report.fault.time == core.SimTime(0)
    assert report.fault.cause == "explicit cause"


# ===== M1-X1: co-simulation surface ==================================================================================

COUPLING_NAMES = [
    "controller.pin_outputs",
    "physics.outputs",
    "signals.operator_status",
    "signals.physics_inputs",
    "signals.pin_inputs",
    "signals.rails",
]


class ShaftAdapter(core.PhysicsAdapter):  # type: ignore[misc]
    """One body at x = 1 m turning 0.01 rad per step; transmission 0 carries a 0.02 N m load torque."""

    def __init__(self) -> None:
        super().__init__()
        self.received: list[list[float]] = []
        self.drives: list[list[Any]] = []
        self._out = core.PhysicsOutputs()
        self._out.bodies = [core.BodyState()]
        self._out.joints = [core.JointForce()]
        state = core.TransmissionState()
        state.shaft_load_torque = 0.02
        state.power_flow = core.PowerFlow.Forward
        self._out.transmissions = [state]
        self._move()

    def _move(self) -> None:
        body = core.BodyState()
        body.position = [1.0, 0.0, 0.0]
        angle = 0.01 * self._out.steps
        body.orientation = [math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0)]
        self._out.bodies = [body]
        self._out.joint_positions = [angle]

    def supported_elements(self) -> list[str]:
        return []

    def load(self, model: Any, rate: Any) -> None:
        pass

    def reset(self) -> None:
        pass

    def step(self) -> None:
        self._out.steps = self._out.steps + 1
        self._move()

    def outputs(self) -> Any:
        return self._out

    def health(self) -> Any:
        return core.PhysicsHealth()

    def set_inputs(self, inputs: Any) -> None:
        self.received.append(list(inputs.generalized_forces))
        self.drives.append(list(inputs.joint_drives))

    def set_spring_offset(self, spring: int, offset: float) -> None:
        pass

    def set_spring_stiffness(self, spring: int, stiffness: float) -> None:
        pass

    def disable_constraint(self, constraint: int) -> None:
        pass

    def release_load(self, load: int) -> None:
        pass

    def set_payload_mass(self, payload: int, mass: float) -> None:
        pass

    def set_payload_attachment(self, payload: int, body: int) -> None:
        pass


def chain_layer() -> Any:
    config = core.SignalLayerConfig()
    table = core.PinBindingTable()
    for signal, pin, mode in [
        ("x_driver.step", "PA0", core.PeripheralMode.pulse_output),
        ("x_driver.dir", "PA1", core.PeripheralMode.digital_output),
        ("x_driver.enable", "PA2", core.PeripheralMode.digital_output),
        ("x_max_switch.out", "PB1", core.PeripheralMode.digital_input),
        ("x_encoder.channel_a", "PC0", core.PeripheralMode.counter_input),
        ("x_encoder.channel_b", "PC1", core.PeripheralMode.counter_input),
    ]:
        table.add(core.PinBinding(signal, pin, mode))
    config.bindings = table
    config.slots = [
        core.ChannelSlot("x_driver.step", core.PinField.step_count, 0),
        core.ChannelSlot("x_driver.dir", core.PinField.direction, 0),
        core.ChannelSlot("x_driver.enable", core.PinField.enable, 0),
        core.ChannelSlot("x_max_switch.out", core.PinField.digital_in, 1),
        core.ChannelSlot("x_encoder.channel_a", core.PinField.counter_delta, 0),
        core.ChannelSlot("x_encoder.channel_b", core.PinField.counter_delta, 0),
    ]
    config.connections = [
        core.Connection("x_driver.full_steps", "x_motor.full_steps"),
        core.Connection("x_driver.direction", "x_motor.direction"),
        core.Connection("x_driver.enabled", "x_motor.enabled"),
        core.Connection("x_driver.supply", "x_motor.supply"),
    ]
    config.rails = [
        core.RailSpec("motor_power", "motor_supply.voltage", [], ["x_driver.vmot"]),
        core.RailSpec("logic", "logic_supply.voltage"),
    ]
    config.controller_rail = "logic"
    config.actuators = [core.ActuatorOutput("x_motor.torque", 0, applied=False)]
    config.velocity_dofs = 1
    layer = core.SignalLayer(config)
    layer.add_component(core.make_signal_component("power_supply", "motor_supply", {"voltage": 24.0}), core.Stage.power)
    layer.add_component(core.make_signal_component("power_supply", "logic_supply", {"voltage": 3.3}), core.Stage.power)
    angular = core.JointCoordinate(core.CoordinateKind.angular, 0, None, [0.0, 0.0, 1.0])
    encoder = {"coordinate": angular, "resolution": 2.0 * math.pi / 2400.0}
    layer.add_component(core.make_signal_component("incremental_encoder", "x_encoder", encoder), core.Stage.sensor)
    linear = core.JointCoordinate(core.CoordinateKind.linear, 0)
    switch = {"coordinate": linear, "position": 0.5}
    layer.add_component(core.make_signal_component("limit_switch", "x_max_switch", switch), core.Stage.sensor)
    driver = core.make_signal_component("step_dir_driver", "x_driver", {"microsteps": 16})
    layer.add_component(driver, core.Stage.driver)
    motor = {"step_angle": math.pi / 100.0, "rotor_inertia": 0.0, "torque_speed": [(0.0, 0.4), (100.0, 0.3)]}
    layer.add_component(core.make_signal_component("stepper_behavioral", "x_motor", motor), core.Stage.actuator)
    return layer


@pytest.mark.req("KER-019")
@pytest.mark.req("KER-022")
@pytest.mark.req("KER-008")
@pytest.mark.req("KER-027")
def test_cosim_chain_runs_100_ticks_and_records_every_coupling_name() -> None:
    kernel = core.make_kernel(core.RunConfig())
    assert isinstance(kernel, core.CoSimKernel)
    adapter = ShaftAdapter()
    kernel.register_party(core.PhysicsParty(adapter, core.PhysicsPartyConfig()))
    kernel.register_party(chain_layer())
    config = core.ControllerPartyConfig()
    config.rail = "logic"
    runtime = core.make_controller_runtime("null", core.Rate(1000))
    firmware = core.FirmwareImage()
    firmware.path = "firmware/app"
    firmware.tick_hz = 1000
    runtime.load(firmware)
    kernel.register_party(core.ControllerParty(runtime, config))
    sink = core.InMemoryRecorderSink()
    report = kernel.run(core.EndCondition(core.Seconds(100, 1000)), sink)
    assert report.outcome == core.RunOutcome.end_condition_met
    assert [(r.party, r.source) for r in report.party_rates] == [
        ("physics", core.RateSource.nominal),
        ("signals", core.RateSource.nominal),
        ("controller", core.RateSource.control_rate),
    ]
    assert report.sink_errors == []
    assert sorted(kernel.coupling.names()) == COUPLING_NAMES
    rows = sink.drain()
    assert len(rows) == 100
    assert sink.drained() == 100
    last = rows[-1].to_dict()
    assert sorted(last["coupling"]) == COUPLING_NAMES
    coupling = last["coupling"]
    assert isinstance(coupling["physics.outputs"], core.PhysicsOutputs)
    assert coupling["physics.outputs"].steps == 100
    assert coupling["physics.outputs"].transmissions[0].shaft_load_torque == 0.02
    assert isinstance(coupling["signals.physics_inputs"], core.PhysicsInputs)
    assert isinstance(coupling["signals.pin_inputs"], core.PinInputs)
    assert coupling["signals.pin_inputs"].digital_in[1] == 1
    assert isinstance(coupling["controller.pin_outputs"], core.PinOutputs)
    assert coupling["signals.operator_status"] == []
    rails = {rail.name: rail.voltage for rail in coupling["signals.rails"]}
    assert rails == {"motor_power": 24.0, "logic": 3.3}
    units = {sample["name"]: sample["unit"] for sample in last["pin_signals"] + last["rails"]}
    assert units["logic.voltage"] == "V"
    assert units["x_driver.step"] != ""
    assert last["controller_state"] == "0"
    assert last["physics"]["transmissions"][0]["shaft_load_torque"] == 0.02
    # The null runtime leaves the driver disabled: the stepper's drive on its shaft joint is off (SIG-033, KER-027).
    assert adapter.received[-1] == [0.0]
    assert len(adapter.drives[-1]) == 1
    drive = adapter.drives[-1][0]
    assert drive.joint == 0
    assert drive.enabled is False
    assert drive.rest == pytest.approx(0.01 * 99, abs=1e-12)


@pytest.mark.req("KER-019")
@pytest.mark.req("KER-008")
def test_python_party_reads_and_writes_the_coupling_table() -> None:
    seen: list[Any] = []

    class Writer(RecordingParty):
        def published(self) -> list[str]:
            return ["writer.level", "writer.forces", "writer.count", "writer.flag"]

        def write_outputs(self, outputs: Any) -> None:
            forces = core.PhysicsInputs()
            forces.generalized_forces = [1.5]
            outputs.set("writer.level", 0.25)
            outputs.set("writer.forces", forces)
            outputs.set("writer.count", 7)
            outputs.set("writer.flag", True)

    class Reader(RecordingParty):
        def consumed(self) -> list[str]:
            return ["writer.level", "writer.forces", "writer.count", "writer.flag"]

        def read_inputs(self, inputs: Any) -> None:
            seen.append(
                (
                    inputs.real("writer.level"),
                    inputs.value("writer.forces").generalized_forces,
                    inputs.integer("writer.count"),
                    inputs.flag("writer.flag"),
                )
            )

    log: list[Any] = []
    report = run_kernel([Writer("writer", log), Reader("reader", log)], core.Seconds(2, 1000))
    assert report.outcome == core.RunOutcome.end_condition_met
    assert seen == [(0.25, [1.5], 7, True), (0.25, [1.5], 7, True)]

    class BadWriter(RecordingParty):
        def write_outputs(self, outputs: Any) -> None:
            outputs.set("bad.out", object())

    report = run_kernel([BadWriter("bad", log)], core.Seconds(1, 1000))
    assert report.outcome == core.RunOutcome.run_error
    assert "TypeError" in report.fault.cause


@pytest.mark.req("KER-017")
def test_end_triggers_end_on_a_published_signal() -> None:
    class Flag(RecordingParty):
        def published(self) -> list[str]:
            return ["flag.done"]

        def write_outputs(self, outputs: Any) -> None:
            outputs.set("flag.done", len(self._log) >= 3)

    log: list[Any] = []
    kernel = core.CoSimKernel(core.RunConfig())
    kernel.register_party(Flag("flag", log))
    kernel.set_end_triggers(core.EndTriggers(on_signal="flag.done"))
    report = kernel.run(core.EndCondition(core.Seconds(1)), core.InMemoryRecorderSink())
    assert report.end_reason == core.EndReason.signal
    assert [count.steps for count in report.step_counts] == [2]


@pytest.mark.req("KER-022")
def test_in_memory_sink_subscribes_and_unsubscribes() -> None:
    sink = core.InMemoryRecorderSink()
    times: list[int] = []
    subscription = sink.subscribe(lambda row: times.append(row.time.count()))
    assert sink.subscriber_count() == 1
    log: list[Any] = []
    kernel = core.CoSimKernel(core.RunConfig())
    kernel.register_party(RecordingParty("a", log))
    kernel.run(core.EndCondition(core.Seconds(2, 1000)), sink)
    sink.unsubscribe(subscription)
    assert sink.subscriber_count() == 0
    assert times == [1, 2]
    assert len(sink.rows()) == 2
    assert sink.subscriber_errors() == []


@pytest.mark.req("SIG-029")
@pytest.mark.req("SIG-034")
def test_builtin_components_by_catalog_type_name() -> None:
    coordinate = core.JointCoordinate(core.CoordinateKind.linear, 0)
    made = {
        "limit_switch": core.make_signal_component("limit_switch", "s", {"coordinate": coordinate}),
        "incremental_encoder": core.make_signal_component(
            "incremental_encoder",
            "e",
            {
                "coordinate": core.JointCoordinate(core.CoordinateKind.angular, 0, None, [0.0, 0.0, 1.0]),
                "resolution": 0.01,
            },
        ),
        "step_dir_driver": core.make_signal_component("step_dir_driver", "d", {"microsteps": 8}),
        "stepper_behavioral": core.make_signal_component(
            "stepper_behavioral", "m", {"step_angle": 0.01, "torque_speed": [(0.0, 0.4)]}
        ),
        "power_supply": core.make_signal_component("power_supply", "p", {"voltage": 24.0}),
        "estop_relay": core.make_signal_component("estop_relay", "r", {"cutoff_delay": 0.02}),
        "watchdog": core.make_signal_component("watchdog", "w", {"timeout": 0.05}),
    }
    assert made["stepper_behavioral"].outputs() == ["m.torque", "m.stalled", "m.missed_steps"]
    assert made["estop_relay"].inputs() == ["r.estop_loop", "r.reset"]
    assert made["power_supply"].outputs() == ["p.voltage"]
    with pytest.raises(ValueError, match="no parameter 'heartbeat'"):
        core.make_signal_component("estop_relay", "r", {"heartbeat": 1.0})
    with pytest.raises(ValueError, match="unknown built-in"):
        core.make_signal_component("hydraulic_ram", "h", {})
    with pytest.raises(ValueError, match="x_shaft"):
        core.make_signal_component(
            "stepper_behavioral",
            "m",
            {
                "step_angle": 0.01,
                "rotor_inertia": 1e-5,
                "torque_speed": [(0.0, 0.4)],
                "shaft_axial_inertia": 2e-5,
                "shaft_body": "x_shaft",
            },
        )
    injection = core.FailureInjection()
    injection.mode = core.FailureMode.slip
    injection.fraction = 0.5
    injection.end = core.SimTime(10)
    assert injection.fraction == 0.5
    assert core.FailureMode.stall != core.FailureMode.slip


@pytest.mark.req("SIG-030")
def test_component_clock_and_child_seed() -> None:
    clock = core.ComponentClock(core.Rate(1000), core.Duration(1), core.derive_child_seed(42, "x_encoder"))
    assert clock.seed == core.derive_child_seed(42, "x_encoder")
    assert core.derive_child_seed(42, "x_encoder") != core.derive_child_seed(42, "y_encoder")
    started: list[int] = []

    class Clocked(Doubler):
        def start(self, clock: Any) -> None:
            started.append(clock.seed)

        def begin_tick(self, t: Any) -> None:
            started.append(-t.count())

    component = Clocked()
    component.start(clock)
    component.begin_tick(core.SimTime(3))
    assert started == [clock.seed, -3]


@pytest.mark.req("CTL-006")
def test_channel_map_and_native_config_round_trip() -> None:
    channels = core.ChannelMap()
    channels.add(core.ChannelAssignment("x_max_switch.out", core.PinSlot.digital_in, 1))
    channels.add(core.ChannelAssignment("x_driver.dir", core.PinSlot.digital_out, 2))
    channels.add(core.ChannelAssignment("x_driver.enable", core.PinSlot.digital_out, 3))
    channels.assign_axis(0, core.AxisChannels(2, 3))
    assert channels.axis(0).enable_channel == 3
    assert channels.find("x_max_switch.out").channel == 1
    assert core.slot_capacity(core.PinSlot.digital_in) == 32
    config = core.NativeRuntimeConfig()
    config.control_rate = core.Rate(1000)
    config.channels = channels
    config.work_dir = "native_work"
    assert config.brownout_threshold_v == core.placeholder_brownout_threshold_v
    runtime = core.NativeControllerRuntime(config)
    assert runtime.ticks_run() == 0
    assert runtime.channel_map().entries()[0].signal == "x_max_switch.out"
    assert core.board_interface_version() > 0
    request = core.HostBuildRequest()
    request.tick_hz = 1000
    assert request.app_target == "mechsim_fw_app"


@pytest.mark.req("ENG-048")
@pytest.mark.req("PHY-033")
def test_engine_transmission_publishes_the_shaft_supply_sign() -> None:
    def body(name: str, joint_type: Any, axis: list[float], mass: float) -> Any:
        joint = eng.Joint()
        joint.type = joint_type
        joint.position = np.zeros(3)
        joint.axis = np.array(axis)
        made = eng.Body()
        made.name = name
        made.parent = -1
        made.joint = joint
        made.mass = mass
        made.com = np.zeros(3)
        made.inertia = np.diag([1e-5, 1e-5, 1e-5])
        return made

    model = eng.MultibodyModel()
    model.bodies = [
        body("shaft", eng.JointType.Revolute, [0.0, 0.0, 1.0], 0.1),
        body("carriage", eng.JointType.Prismatic, [1.0, 0.0, 0.0], 10.0),
    ]
    model.gravity = np.zeros(3)
    drive = eng.Transmission()
    drive.type = eng.TransmissionType.LeadScrew
    drive.input_body = 0
    drive.output_body = 1
    drive.ratio = 0.008
    model.transmissions = [drive]
    provider = eng.MultibodyProvider(model)
    integrator = eng.SemiImplicitEuler(eng.StepConfig())
    state = eng.State(provider.neutral_positions(), np.zeros(provider.velocity_size()), 0.0)
    provider.set_shaft_torque(0, 0.01)
    provider.step(integrator, state)
    out = provider.outputs.transmissions[0]
    # Driving the carriage, the shaft supplies torque along its + axis: positive, g F with g = lead / 2 pi.
    assert out.shaft_load_torque > 0.0
    assert out.shaft_load_torque == pytest.approx(0.008 / (2.0 * math.pi) * out.force, rel=1e-12)
    states = provider.transmission_states(state.q, state.v)
    assert states[0].shaft_load_torque == out.shaft_load_torque
    provider.set_slip(0, 0.25)
    assert provider.slip(0) == 0.25


PENDULUM_EXAMPLE = Path(__file__).resolve().parents[2] / "schemas" / "examples" / "physics_model.pendulum.yaml"


@pytest.mark.req("PHY-014")
@pytest.mark.req("KER-019")
@pytest.mark.req("KER-016")
def test_mujoco_pendulum_through_a_physics_party_matches_the_small_angle_closed_form() -> None:
    # Closed form: a 9 N m torque for the one step ending at 2 ms gives the rod v1 = h tau / (I + h D) (the damping is
    # implicit in the step), I = m L^2 / 3 about the pivot. Semi-implicit Euler then puts q = h v1 at 2 ms, which is
    # the free damped motion theta = v1 / omega_d exp(-zeta omega0 s) sin(omega_d s), s = t - 1 ms, started at 1 ms.
    # Amplitude 0.02 rad: the sin(theta) term departs from small angle by theta^2 / 6 ~ 7e-5; the scheme's phase error
    # over 100 steps is (omega0 h)^2 / 24 per radian, ~1e-6. Both are far inside the brief's 1 percent.
    from mechsim.physics import neutral
    from mechsim.physics.mujoco import translate as mj

    model = neutral.from_yaml(PENDULUM_EXAMPLE.read_text(encoding="utf-8"))
    translation = mj.translate_report(model)
    # The pendulum maps exactly: no approximation, no omission to report.
    assert translation.report == ()
    adapter = core.make_physics_adapter("mujoco")
    assert isinstance(adapter, core.MujocoAdapter)
    prepared = core.PreparedModel(model.model_dump_json(), translation.mjcf, model.design.version)
    adapter.load(prepared, core.Rate(1000))
    assert adapter.integrator() == "implicitfast"
    assert adapter.timestep() == 0.001

    h, tau, mass, length, damping, gravity = 0.001, 9.0, 1.0, 0.5, 0.04520785330006282, 9.81
    inertia = mass * length**2 / 3.0
    omega0 = math.sqrt(mass * gravity * 0.5 * length / inertia)
    zeta = damping / (2.0 * inertia * omega0)
    omega_d = omega0 * math.sqrt(1.0 - zeta * zeta)
    v1 = h * tau / (inertia + h * damping)

    class Kick(RecordingParty):
        def published(self) -> list[str]:
            return ["kick.forces"]

        def write_outputs(self, outputs: Any) -> None:
            forces = core.PhysicsInputs()
            forces.generalized_forces = [tau if len(self._log) == 2 else 0.0]
            outputs.set("kick.forces", forces)

    config = core.PhysicsPartyConfig()
    config.inputs = ["kick.forces"]
    kernel = core.CoSimKernel(core.RunConfig())
    kernel.register_party(core.PhysicsParty(adapter, config))
    kernel.register_party(Kick("kick", []))
    sink = core.InMemoryRecorderSink()
    report = kernel.run(core.EndCondition(core.Seconds(100, 1000)), sink)
    assert report.outcome == core.RunOutcome.end_condition_met
    rows = sink.drain()
    assert len(rows) == 100
    w, _, _, z = rows[-1].physics.bodies[0].orientation
    angle = 2.0 * math.atan2(z, w) + math.pi / 2.0
    s = 0.1 - h
    closed_form = v1 / omega_d * math.exp(-zeta * omega0 * s) * math.sin(omega_d * s)
    print(f"pendulum at 100 ms: closed form {closed_form} rad, measured {angle} rad")
    assert abs(angle - closed_form) <= 0.01 * abs(closed_form)
