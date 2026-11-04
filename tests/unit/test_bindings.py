"""The Python binding of the core runtime, mechsim._core_ext (ADR 0005), through the mechsim._core shim.

Each test cites the interface row it exercises through the binding; the C++ behaviour behind it has its own Catch2
tests under core/.
"""

import gc
import math
import sys
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
    with pytest.raises(ValueError, match="unknown physics adapter 'nope'; known: null"):
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
