"""Native adapter run-time approximation lines and node pose series (PHY-031, PHY-003, PHY-012, PHY-021; rulings 44,
45, 47, 48): the transmission creep line, the driven-joint mode line from the largest scheme decay a real run
published, and the member node poses named `<member>.node<k>` in a recorded archive."""

from __future__ import annotations

import gc
import math
from dataclasses import replace
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mechsim import _core as core
from mechsim.builder.structure import chain_layout
from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.native.translate import (
    MODE_RESOLUTION,
    SLIP_VELOCITY,
    driven_mode_lines,
    member_node_names,
    mode_text,
    translate,
)
from mechsim.physics.neutral import PhysicsModel, from_yaml
from mechsim.record.archive import RunArchive, write_archive
from mechsim.record.stream import Recorder
from mechsim.record.tick import BodyState
from mechsim.testing.core_driver import BRIDGE_NODE_POSES, CoreDriver, RecordArchiver, core_builder
from mechsim.testing.driver import EndSpec, RunReport
from mechsim.testing.runner import Services, prepare, run_case
from mechsim.testing.schema import DESIGNS_ROOT
from tests.reference.test_phy_native_beams import EI, beam_model, settle
from tests.unit.test_record_support import finished, identity, synthetic_rows

ROOT = Path(__file__).resolve().parents[2]
PENDULUM = (ROOT / "schemas/examples/physics_model.pendulum.yaml").read_text()
ORIGIN = ("0 m", "0 m", "0 m")
G = 9.81
H = 1e-3
G1 = ROOT / "test_cases" / "gantry" / "G1.yaml"
TICKS = 20


def _slider(name: str, axis: tuple[float, float, float], mass: float = 2.0) -> dict[str, Any]:
    return {
        "name": name,
        "mass": f"{mass} kg",
        "com": ORIGIN,
        "inertia": {"ixx": "0.01 kg*m**2", "iyy": "0.01 kg*m**2", "izz": "0.01 kg*m**2"},
        "material": "steel",
        "parent": "world",
        "joint": {"name": name + "_joint", "type": "prismatic", "position": ORIGIN, "axis": axis},
    }


def _driven_model(transmission: dict[str, Any] | None, axis: tuple[float, float, float]) -> PhysicsModel:
    """The pendulum example (gravity -y) plus a motor shaft about z and a 2 kg carriage on a prismatic `axis`."""
    data = from_yaml(PENDULUM).model_dump(mode="json")
    shaft = _slider("motor", (0.0, 0.0, 1.0), 0.1)
    shaft["joint"]["type"] = "revolute"
    data["bodies"] = [*data["bodies"], shaft, _slider("carriage", axis)]
    if transmission is not None:
        data["transmissions"] = [transmission]
    return PhysicsModel.model_validate(data)


def _belt(efficiency: float) -> dict[str, Any]:
    return {
        "type": "belt_pulley",
        "name": "belt",
        "shaft_joint": "motor_joint",
        "driven_joint": "carriage_joint",
        "efficiency": efficiency,
        "pulley_radius": "0.01 m",
        "stiffness": "1e5 N/m",
    }


def _creep_lines(model: PhysicsModel) -> list[str]:
    return [line for line in translate(model).report if "efficiency switch regularised" in line]


# ===== Creep line (PHY-031, rulings 44, 47) ==============================


@pytest.mark.req("PHY-031")
def test_a_lossy_belt_on_a_vertical_carriage_states_its_creep_with_the_static_load_values() -> None:
    # n = r = 0.01, eta = 0.8, F = m g = 19.62 N along the carriage axis (-y gravity, +y axis). Sag n F = 0.1962 / K;
    # loss coefficients n F (1 / eta - 1) / omega_s = 490.5 and n F (1 - eta) / omega_s = 392.4 N m s/rad.
    (line,) = _creep_lines(_driven_model(_belt(0.8), (0.0, 1.0, 0.0)))
    assert line.startswith("transmission 'belt': efficiency switch regularised below the shaft speed omega_s = ")
    assert f"omega_s = {SLIP_VELOCITY} rad/s" in line
    assert "tau = (n F (1 / eta - 1) / omega_s + D_d) / K driving the load" in line
    assert "(n F (1 - eta) / omega_s + D_d) / K back-driven" in line
    assert "n = 0.01, eta = 0.8" in line
    load = 2.0 * G
    assert f"at the static load F = {load:.6g} N: sag {0.01 * load:.6g} / K" in line
    assert f"tau = ({0.01 * load * 0.25 / 1e-4:.6g} + D_d) / K driving" in line
    assert f"({0.01 * load * 0.2 / 1e-4:.6g} + D_d) / K back-driven" in line
    assert "490.5" in line
    assert "392.4" in line
    assert "self-locking" not in line


@pytest.mark.req("PHY-031")
def test_a_horizontal_carriage_has_no_static_load_value_and_a_lossless_belt_no_line() -> None:
    (line,) = _creep_lines(_driven_model(_belt(0.8), (1.0, 0.0, 0.0)))
    assert "static load" not in line
    assert _creep_lines(_driven_model(_belt(1.0), (0.0, 1.0, 0.0))) == []
    assert _creep_lines(_driven_model(None, (0.0, 1.0, 0.0))) == []


@pytest.mark.req("PHY-031")
def test_a_self_locking_screw_states_its_path_dependent_sag_band() -> None:
    # Lead 8 mm: n = 0.008 / 2 pi; on a vertical carriage the band is [0, n m g / K].
    screw = {
        "type": "lead_screw",
        "name": "screw",
        "shaft_joint": "motor_joint",
        "driven_joint": "carriage_joint",
        "efficiency": 0.4,
        "lead": "0.008 m",
        "stiffness": "1e6 N/m",
        "self_locking": True,
    }
    (line,) = _creep_lines(_driven_model(screw, (0.0, 1.0, 0.0)))
    n = 0.008 / (2.0 * math.pi)
    assert f"path-dependent in [0, {n * 2.0 * G:.6g} / K]" in line
    assert "ruling 48" in line
    (horizontal,) = _creep_lines(_driven_model(screw, (1.0, 0.0, 0.0)))
    assert "path-dependent in [0, n F / K]" in horizontal
    screw["self_locking"] = False
    (plain,) = _creep_lines(_driven_model(screw, (0.0, 1.0, 0.0)))
    assert "self-locking" not in plain


# ===== Driven-joint mode line from a real run (PHY-031, rulings 44, 45) ====


def _run_decays(stiffness: float, ticks: int = 10) -> list[tuple[int, float]]:
    """A real native run of the driven model with a drive of stiffness K on the carriage (body 2), horizontal."""
    model = _driven_model(None, (1.0, 0.0, 0.0))
    prepared = prepare_model(model, None, H)
    adapter = core.make_physics_adapter("native")
    assert isinstance(adapter, core.NativeAdapter)
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(1000))
    for _ in range(ticks):
        inputs = core.PhysicsInputs()
        inputs.joint_drives = [core.JointDrive(2, 0.0, stiffness, 0.0, True)]
        adapter.set_inputs(inputs)
        adapter.step()
    return [(int(joint), float(decay)) for joint, decay in adapter.driven_scheme_decay()]


@pytest.mark.req("PHY-031", "PHY-034")
@pytest.mark.parametrize("x", [0.1, 0.3])
def test_the_driven_joint_line_carries_the_run_mode_and_flags_it_above_the_threshold(x: float) -> None:
    # The 2 kg carriage on a drive K = m (x / h)^2 has omega = sqrt(K / m) = x / h, so the engine publishes pi x
    # (rulings 44, 45; 1e-12): kept per period (1 + x^2)^(-pi / atan x), frequency ratio atan(x) / x; flagged above 0.2.
    decays = _run_decays(2.0 * (x / H) ** 2)
    assert len(decays) == 1
    assert decays[0][0] == 2
    assert decays[0][1] == pytest.approx(math.pi * x, rel=1e-12)
    model = _driven_model(None, (1.0, 0.0, 0.0))
    (line,) = driven_mode_lines(model, decays)
    assert line.startswith("driven joint 'carriage_joint': drive mode at run time, largest over the run x = omega h = ")
    assert mode_text(x) in line or mode_text(decays[0][1] / math.pi) in line
    kept = (1.0 + x * x) ** (-math.pi / math.atan(x))
    assert f"amplitude kept per period {kept:.3g}" in line
    assert f"frequency ratio {math.atan(x) / x:.3g}" in line
    assert ("unresolved at this step" in line) == (x > MODE_RESOLUTION)


@pytest.mark.req("PHY-031")
def test_the_decay_record_is_readable_after_the_kernel_is_released() -> None:
    """The record is taken before the party owns the adapter and read after the kernel, party and adapter are gone:
    a value snapshot, no handle to the adapter (x = 0.1, decay pi 0.1, 1e-12)."""
    model = _driven_model(None, (1.0, 0.0, 0.0))
    prepared = prepare_model(model, None, H)
    adapter = core.make_physics_adapter("native")
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(1000))
    record = adapter.scheme_decay_record()
    inputs = core.PhysicsInputs()
    inputs.joint_drives = [core.JointDrive(2, 0.0, 2.0 * (0.1 / H) ** 2, 0.0, True)]
    adapter.set_inputs(inputs)
    adapter.step()
    party_config = core.PhysicsPartyConfig()
    party_config.rate = core.Rate(1000)
    party_config.inputs = []
    kernel = core.make_kernel(core.RunConfig())
    kernel.register_party(core.PhysicsParty(adapter, party_config))
    kernel.run(core.EndCondition(core.Seconds(5, 1000)), core.InMemoryRecorderSink())
    del kernel
    del adapter
    gc.collect()
    ((joint, decay),) = record.values()
    assert joint == 2
    assert decay == pytest.approx(math.pi * 0.1, rel=1e-12)
    assert not hasattr(core.PhysicsParty, "adapter")


@pytest.mark.req("PHY-031")
def test_the_largest_decay_of_the_run_is_kept_and_an_undriven_run_keeps_none() -> None:
    assert _run_decays(0.0, 0) == []
    assert driven_mode_lines(_driven_model(None, (1.0, 0.0, 0.0)), []) == ()


# ===== Node pose series through CoreDriver (PHY-003, PHY-012, PHY-021) =====


class ShortDriver(CoreDriver):
    def run(self, end: EndSpec) -> RunReport:
        return super().run(EndSpec(max_tick=TICKS, at_tick=TICKS))


def _archive_run(tmp_path: Path, adapter: str) -> tuple[Any, PhysicsModel]:
    services = Services(ShortDriver, core_builder, RecordArchiver(DESIGNS_ROOT))
    result = run_case(G1, services, {"physics.adapter": adapter}, out_dir=tmp_path / adapter)
    assert result.report is not None
    assert not result.report.errored, result.report.error
    model = core_builder(prepare(G1, {"physics.adapter": adapter}).design, adapter).physics
    assert model is not None
    return result, model


@pytest.mark.req("PHY-003", "PHY-012", "REC-018")
def test_a_flexible_design_on_native_records_named_node_pose_series(tmp_path: Path) -> None:
    result, model = _archive_run(tmp_path, "native")
    nodes = member_node_names(model)
    assert len(nodes) > 0
    neutral = tuple(body.name for body in model.bodies)
    with RunArchive(result.archive) as archive:
        position = archive.series("physics/bodies/position")
        orientation = archive.series("physics/bodies/orientation")
    assert position.names == (*neutral, *nodes)
    assert orientation.names == position.names
    assert position.values.shape == (TICKS, len(neutral) + len(nodes), 3)
    first = position.names.index(nodes[0])
    node = np.asarray(position.values[:, first, :])
    assert np.all(np.isfinite(node))
    quaternion = np.asarray(orientation.values[:, first, :])
    assert np.allclose(np.linalg.norm(quaternion, axis=1), 1.0, atol=1e-12)
    # Undeformed spacing at the first sample: consecutive node origins of the first member sit the chain's start
    # spacing apart (rigid segments; a 1 ms deflection angle changes a length by its square, far below 1e-6).
    member = model.lumped_members[0]
    chain = chain_layout(member.root, member.end, int(member.segments), float(member.section.length))
    own = [name for name in nodes if name.startswith(f"{member.name}.node")]
    for k in range(1, len(own) - 2):
        a = np.asarray(position.values[0, position.names.index(own[k]), :])
        b = np.asarray(position.values[0, position.names.index(own[k + 1]), :])
        spacing = chain.starts[k + 1] - chain.starts[k]
        assert float(np.linalg.norm(b - a)) == pytest.approx(spacing, rel=1e-6)
    lines = [line for line in result.report.approximations if line.startswith("driven joint '")]
    assert len(lines) > 0
    assert all("x = omega h = " in line for line in lines)
    assert BRIDGE_NODE_POSES not in result.report.approximations


@pytest.mark.req("PHY-003", "PHY-012", "REC-018")
def test_a_recorded_mid_node_reads_back_at_the_chain_static_deflection(tmp_path: Path) -> None:
    """A 1 m steel cantilever (8 segments, tip load P = 1 N, settled as in the reference tier) recorded with named
    body series and read back by `beam.node4`, whose origin is its hinge at a = (4 + 1/2) l = 0.5625 m (`chain_layout`
    starts). Chain value at x = a for springs at x_k = (k + 1/2) l: P l / (E I) sum_{x_k < a} (L - x_k)(a - x_k)
    (1e-6, the settled backward Euler fixed point); the continuum P a^2 (3 L - a) / (6 E I) is 1.2 percent away,
    inside the 2 percent row (BLD-015 to BLD-017)."""
    beam = settle("free", 8, 1.0, 1.0)
    model_nodes = translate_names()
    bodies = tuple(
        BodyState(
            tuple(float(v) for v in b.position),
            tuple(float(v) for v in b.orientation),
            tuple(float(v) for v in b.angular_velocity),
            tuple(float(v) for v in b.linear_velocity),
        )
        for b in beam.adapter.outputs().bodies
    )
    row = synthetic_rows(1, 1)[0]
    row = replace(row, physics=replace(row.physics, bodies=bodies))
    recorder = Recorder(Fraction(1000), bodies=("base", *model_nodes))
    recorder.write_row(row)
    path = tmp_path / "beam.h5"
    write_archive(path, identity(), finished(1), recorder.recording())
    with RunArchive(path) as archive:
        series = archive.series("physics/bodies/position")
    mid = float(series.values[0, series.names.index("beam.node4"), 1])
    layout = chain_layout("clamped", "free", 8, 1.0)
    a = layout.starts[4]
    chain = sum((1.0 - x) * (a - x) for x in layout.hinges if x < a) * 0.125 / EI
    continuum = a * a * (3.0 - a) / (6.0 * EI)
    print(f"mid node deflection {mid:.9g} m, chain {chain:.9g}, continuum {continuum:.9g}")
    assert mid == pytest.approx(chain, rel=1e-6)
    assert mid == pytest.approx(continuum, rel=0.02)


def translate_names() -> tuple[str, ...]:
    return member_node_names(beam_model("free", 8, 0.0))


@pytest.mark.req("PHY-003", "PHY-021")
def test_the_same_design_on_the_bridge_records_no_node_series_and_says_so(tmp_path: Path) -> None:
    result, model = _archive_run(tmp_path, "mujoco")
    with RunArchive(result.archive) as archive:
        position = archive.series("physics/bodies/position")
    assert position.names == tuple(body.name for body in model.bodies)
    assert not any(".node" in name for name in position.names)
    assert BRIDGE_NODE_POSES in result.report.approximations
    assert not any(line.startswith("driven joint '") for line in result.report.approximations)
