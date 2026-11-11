"""Milestone 2 wiring of the gantry (M2-X1): payload body, brake, load cell chain, faults, labels and firmware events.

Brake bound (core_wiring module docstring): on a prismatic joint driven by a transmission of c metres per radian the
holding force is brake_torque / c and A0 = 1 / (m + sum J / c^2), m the mass below the joint, J the shaft's axial
inertia. Both are closed forms of the neutral model, so the test recomputes them from the model's own fields.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mechsim import _core as core
from mechsim.builder import BuildError
from mechsim.builder.payload import PAYLOAD_CLEARANCE, PAYLOAD_EDGE, with_payload
from mechsim.design.chip import shipped_chips
from mechsim.physics.mujoco.prepare import prepare_model
from mechsim.testing import core_wiring
from mechsim.testing.core_driver import CoreDriver, core_builder, model_columns
from mechsim.testing.runner import Services, prepare, run_case
from mechsim.testing.schema import ROOT

G3 = ROOT / "test_cases" / "gantry" / "G3.yaml"


@pytest.fixture(scope="module")
def built() -> object:
    return core_builder(prepare(G3).design)


def _instances(build: object) -> dict[str, object]:
    return {instance.name: instance for instance in build.wiring.instances}  # type: ignore[attr-defined]


@pytest.mark.req("BLD-036")
def test_payload_body_hangs_below_its_carrier_with_box_inertia(built: object) -> None:
    model = built.physics  # type: ignore[attr-defined]
    loaded = with_payload(model, "payload", "hook", 2.0)
    body = loaded.bodies[-1]
    assert body.joint is not None
    assert (body.name, body.parent, body.joint.type) == ("payload", "world", "free")
    assert body.mass == 2.0
    assert body.inertia.ixx == pytest.approx(2.0 * PAYLOAD_EDGE**2 / 6.0, rel=1e-15)
    # hook origin (0.1, 0.1, 0.34) at the reference configuration; hook box 0.04 m: extent sqrt(3) 0.02.
    drop = math.sqrt(3.0) * 0.02 + PAYLOAD_EDGE / 2.0 + PAYLOAD_CLEARANCE
    assert body.joint.position == pytest.approx((0.1, 0.1, 0.34 - drop), abs=1e-12)


@pytest.mark.req("BLD-036")
def test_payload_attached_at_the_start_is_a_fixed_child_of_its_carrier(built: object) -> None:
    # Ruling 24: rigid on a fixed joint, placed in the carrier's frame. The hook's reference orientation is the
    # identity, so the joint position is the world drop (0, 0, -drop) itself.
    model = built.physics  # type: ignore[attr-defined]
    loaded = with_payload(model, "payload", "hook", 2.0, fixed=True)
    body = loaded.bodies[-1]
    assert body.joint is not None
    assert (body.name, body.parent, body.joint.type, body.joint.name) == ("payload", "hook", "fixed", "payload_fixed")
    assert body.mass == 2.0
    drop = math.sqrt(3.0) * 0.02 + PAYLOAD_EDGE / 2.0 + PAYLOAD_CLEARANCE
    assert body.joint.position == pytest.approx((0.0, 0.0, -drop), abs=1e-12)
    places, _ = core_wiring.joint_places(loaded)
    cell = core_wiring.component_parameters(_instances(built)["load_cell"], places, loaded, [], None)  # type: ignore[arg-type]
    assert cell["joint_reaction"] == len(loaded.bodies) - 1
    assert "payload" not in cell


@pytest.mark.req("BLD-036")
def test_fixed_payload_is_placed_in_a_rotated_carriers_frame(built: object) -> None:
    # A carrier rotated a quarter turn about x (q = (cos 45, sin 45, 0, 0)) maps its local -y onto world -z, so the
    # world drop (0, 0, -d) is (0, -d, 0) in its frame.
    model = built.physics  # type: ignore[attr-defined]
    data = model.model_dump(mode="json")
    half = math.sqrt(0.5)
    for body in data["bodies"]:
        if body["name"] == "hook":
            body["joint"]["orientation"] = [half, half, 0.0, 0.0]
    rotated = type(model).model_validate(data)
    body = with_payload(rotated, "payload", "hook", 1.0, fixed=True).bodies[-1]
    assert body.joint is not None
    drop = math.sqrt(3.0) * 0.02 + PAYLOAD_EDGE / 2.0 + PAYLOAD_CLEARANCE
    assert body.joint.position == pytest.approx((0.0, -drop, 0.0), abs=1e-12)


@pytest.mark.req("BLD-036")
@pytest.mark.parametrize(
    ("name", "carrier", "mass", "message"),
    [
        ("hook", "hook", 1.0, "already has a body"),
        ("payload", "nowhere", 1.0, "unknown carrier"),
        ("payload", "hook", 0.0, "positive"),
    ],
)
def test_payload_body_refusals(built: object, name: str, carrier: str, mass: float, message: str) -> None:
    with pytest.raises(BuildError, match=message):
        with_payload(built.physics, name, carrier, mass)  # type: ignore[attr-defined]


@pytest.mark.req("BLD-036")
def test_payload_follows_its_carrier_to_the_start_configuration(built: object) -> None:
    model = with_payload(built.physics, "payload", "hook", 1.0)  # type: ignore[attr-defined]
    plain = prepare_model(model, {}, 0.001, {"payload": "hook"}).mjcf
    lifted = prepare_model(model, {"lift_axis": 0.1}, 0.001, {"payload": "hook"}).mjcf
    import xml.etree.ElementTree as ET

    def z(text: str) -> float:
        for body in ET.fromstring(text).iter("body"):
            if body.get("name") == "payload":
                return float(body.get("pos", "0 0 0").split()[2])
        raise KeyError("payload")

    assert z(lifted) - z(plain) == pytest.approx(0.1, abs=1e-12)


@pytest.mark.req("BLD-035")
@pytest.mark.req("SIG-020")
def test_brake_on_the_lift_motor_shaft_reflects_the_lifted_mass(built: object) -> None:
    # Ruling 23: the brake holds the shaft. Rigid screw x = c theta: kinetic energy (J + m c^2) omega^2 / 2, so the
    # shaft's inverse inertia is A0 = 1 / (J + m c^2), m every body below the lift joint, the torque as given.
    model = built.physics  # type: ignore[attr-defined]
    places, _ = core_wiring.joint_places(model)
    brake = _instances(built)["lift_brake"]
    params = core_wiring.brake_parameters(brake, places, model)  # type: ignore[arg-type]
    screw = next(item for item in model.transmissions if item.name == "lift_screw")
    c = screw.lead / (2.0 * math.pi)
    names = [body.name for body in model.bodies]
    mass = sum(model.bodies[names.index(name)].mass for name in ("lift_carriage", "hook"))
    shaft = model.bodies[names.index("lift_motor_shaft")]
    expected = 1.0 / (core_wiring.axial_inertia(shaft) + mass * c * c)
    print(f"brake torque {params['brake_torque']} N m, A0 {params['inverse_inertia']} (expected {expected}) 1/(kg m^2)")
    assert params["joint"] == places["lift_motor_spin"].body
    assert params["brake_torque"] == pytest.approx(0.5, rel=1e-12)
    assert params["inverse_inertia"] == pytest.approx(expected, rel=1e-12)
    assert core_wiring.undrivable_brake(brake, model) is None  # type: ignore[arg-type]


@pytest.mark.req("BLD-035")
@pytest.mark.req("SIG-020")
def test_a_brake_off_a_revolute_shaft_joint_is_a_wiring_error(built: object) -> None:
    """Ruling 29: a brake on a transmission's driven joint (the bridge holds no second drive there) or on any other
    non-revolute joint ends the build with an error naming the brake and the joint."""
    model = built.physics  # type: ignore[attr-defined]
    places, _ = core_wiring.joint_places(model)
    brake = _instances(built)["lift_brake"].model_copy(update={"acts_on": "lift_axis"})  # type: ignore[attr-defined]
    with pytest.raises(core_wiring.WiringError, match="brake 'lift_brake': cannot be applied on joint 'lift_axis'"):
        core_wiring.brake_parameters(brake, places, model)
    line = core_wiring.undrivable_brake(brake, model)
    assert line is not None
    assert "lift_screw" in line
    assert "the lift" not in line
    assert core_wiring.undrivable_brake(_instances(built)["x_motor"], model) is None  # type: ignore[arg-type]
    undriven = model.model_copy(update={"transmissions": ()})
    with pytest.raises(core_wiring.WiringError, match="acts on prismatic joint 'lift_axis'; a brake acts on"):
        core_wiring.brake_parameters(brake, places, undriven)
    swing = brake.model_copy(update={"acts_on": "hook_swing"})
    with pytest.raises(core_wiring.WiringError, match="acts on spherical joint 'hook_swing'; a brake acts on"):
        core_wiring.brake_parameters(swing, places, model)
    hinge = brake.model_copy(update={"acts_on": "x_motor_spin"})
    assert "inverse_inertia" not in core_wiring.brake_parameters(hinge, places, undriven)


@pytest.mark.req("BLD-035")
def test_load_cell_reads_its_payload_and_the_amplifier_the_chip_adc(built: object) -> None:
    model = built.physics  # type: ignore[attr-defined]
    places, _ = core_wiring.joint_places(model)
    chip = shipped_chips()["stm32_placeholder"]
    cell = core_wiring.component_parameters(_instances(built)["load_cell"], places, model, ["payload"], chip)  # type: ignore[arg-type]
    assert cell["payload"] == 0
    assert cell["capacity"] == pytest.approx(49.05)
    amp = core_wiring.component_parameters(_instances(built)["load_amp"], places, model, ["payload"], chip)  # type: ignore[arg-type]
    assert (amp["adc_bits"], amp["adc_reference"], amp["gain"]) == (12, pytest.approx(3.3), pytest.approx(403.0))
    original: Any = _instances(built)["load_cell"]
    reaction = original.model_copy(
        update={"senses": original.senses.model_copy(update={"joint_reaction": "hook_swing"})}
    )
    found = core_wiring.component_parameters(reaction, places, model, [], chip)
    assert found["joint_reaction"] == places["hook_swing"].body
    typo = original.model_copy(update={"senses": original.senses.model_copy(update={"joint_reaction": "hook_swnig"})})
    with pytest.raises(core_wiring.WiringError, match="senses 'hook_swnig', no joint or payload"):
        core_wiring.component_parameters(typo, places, model, ["payload"], chip, "payload")
    by_name = original.model_copy(
        update={"senses": original.senses.model_copy(update={"joint_reaction": None, "payload": "crate"})}
    )
    with pytest.raises(core_wiring.WiringError, match="load cell 'load_cell': senses payload 'crate'"):
        core_wiring.component_parameters(by_name, places, model, ["payload"], chip, "payload")
    with pytest.raises(core_wiring.WiringError, match="no chip"):
        core_wiring.component_parameters(_instances(built)["load_amp"], places, model, [], None)  # type: ignore[arg-type]


@pytest.mark.req("BLD-036")
@pytest.mark.req("SIG-014")
def test_load_cell_without_a_payload_in_the_run_has_no_target(built: object) -> None:
    """A case that attaches nothing builds no stand-in body; the cell then senses nothing and reads 0 N (A4)."""
    model = built.physics  # type: ignore[attr-defined]
    places, _ = core_wiring.joint_places(model)
    chip = shipped_chips()["stm32_placeholder"]
    original: Any = _instances(built)["load_cell"]
    cell = core_wiring.component_parameters(original, places, model, [], chip)
    assert not {"payload", "joint", "joint_reaction"} & set(cell)
    assert cell["capacity"] == pytest.approx(49.05)
    by_payload = original.model_copy(
        update={"senses": original.senses.model_copy(update={"joint_reaction": None, "payload": "payload"})}
    )
    assert not {"payload", "joint", "joint_reaction"} & set(
        core_wiring.component_parameters(by_payload, places, model, [], chip)
    )
    assert core_wiring.component_parameters(by_payload, places, model, ["payload"], chip)["payload"] == 0
    component = core.make_signal_component("load_cell", "load_cell", cell)
    assert component is not None


@pytest.mark.req("SIG-011")
@pytest.mark.req("LIB-025")
def test_frame_side_encoders_are_linear_on_the_axis_joints(built: object) -> None:
    """Ruling 32: each gantry encoder is a linear_encoder on its axis joint, m per count; the core type comes from the
    catalog's plugin.cpp and `linear` from the resolution's length dimension, no type name in code."""
    model = built.physics  # type: ignore[attr-defined]
    places, _ = core_wiring.joint_places(model)
    for axis in ("x", "y", "lift"):
        instance: Any = _instances(built)[f"{axis}_encoder"]
        assert instance.type == "linear_encoder"
        params = core_wiring.component_parameters(instance, places, model)
        assert params["linear"] is True
        assert params["resolution"] == pytest.approx(1.0 / 60000.0, rel=1e-15)
        assert params["coordinate"].kind == core.CoordinateKind.linear
        assert params["coordinate"].body == places[f"{axis}_axis"].body
        assert core_wiring.core_type(instance) == "incremental_encoder"
        assert instance.length_parameters == ("resolution", "noise")
        assert core.make_signal_component("incremental_encoder", instance.name, params) is not None
    rotary = _instances(built)["x_encoder"].model_copy(  # type: ignore[attr-defined]
        update={"type": "incremental_encoder", "length_parameters": ()}
    )
    assert "linear" not in core_wiring.component_parameters(rotary, places, model)


@pytest.mark.req("FW-028")
def test_event_axes_are_the_step_bindings(built: object) -> None:
    assert core_wiring.event_axes(built) == {0: "x_driver", 1: "y_driver", 2: "lift_driver"}  # type: ignore[arg-type]


@pytest.mark.req("BLD-034")
def test_two_actuators_on_one_shaft_are_refused_at_build() -> None:
    design = prepare(G3).design.model_copy(deep=True)
    motor = next(item for item in design.components if item.name == "lift_motor")
    design.components.append(motor.model_copy(update={"name": "lift_motor_b"}))
    with pytest.raises(Exception, match="driven by actuators 'lift_motor', 'lift_motor_b'"):
        core_builder(design)


@pytest.mark.req("REC-018")
def test_model_columns_label_joints_transmissions_and_payloads(built: object) -> None:
    model = with_payload(built.physics, "payload", "hook", 1.0)  # type: ignore[attr-defined]
    columns = model_columns(model, ["payload"])
    joints = columns["physics/joint_positions"]
    assert joints.names[-1] == "payload_free"
    assert joints.units[[body.joint.name for body in model.bodies if body.joint is not None].index("lift_axis")] == "m"
    assert columns["physics/transmissions/slip"].names == ("x_belt", "y_belt", "lift_screw")
    assert columns["physics/payloads/force"].names == ("payload",)


@pytest.mark.req("REC-018")
@pytest.mark.req("FW-028")
@pytest.mark.req("TST-020")
def test_g3_run_records_joints_payload_and_motion_events(tmp_path: Path) -> None:
    driver = CoreDriver()
    result = run_case(G3, Services(lambda: driver, core_builder), None, None)
    assert result.report is not None
    assert not result.report.errored, result.report
    view = driver.recording()
    positions = view.recording.series["physics/joint_positions"]
    assert positions.values.shape[1] == len(positions.names) > 0
    force = view.recording.series["physics/payloads/force"]
    assert force.values.shape[0] == len(view.times())
    attached = np.asarray(view.recording.series["physics/payloads/attached"].values)
    assert not attached[1:].any()
    kinds = [(event.kind, event.detail) for event in view.events() if event.source == "controller"]
    assert ("motion_start", "x_driver") in kinds
    assert ("motion_end", "x_driver") in kinds
    # Homing starts at the first controller step: its events have no row of that tick and go with the first row.
    starts = [event for event in view.events() if event.kind == "motion_start" and event.detail == "lift_driver"]
    assert len(starts) >= 1
    assert starts[0].time < 0.01
    assert ("state_change", "not_homed>homing") in kinds
    assert ("state_change", "homing>ready") in kinds
    # SCO-022: a controller_fault event only on a change; the first sample's "none" is no change.
    assert kinds.count(("controller_fault", "none")) == 0


@pytest.mark.req("DES-045", "FW-028")
@pytest.mark.parametrize(
    ("code", "arg", "detail"),
    [
        (6, 0, "x_driver"),
        (6, 9, "9"),
        (3, 3, "busy"),
        (3, 200, "200"),
        (1, 3, "ready"),
        (1, (2 << 8) | 3, "homing>ready"),
        (1, (9 << 8) | 3, "9>ready"),
        (2, 5, "5"),
        (8, 5, ""),
        (99, 7, "7"),
    ],
)
def test_event_arguments_decode_by_their_kind(code: int, arg: int, detail: str) -> None:
    """Round 4 ruling 28: axis, fault, state (a packed from-to pair above 255), number, none; unknown kinds as
    number."""
    state = core_wiring.RunnerState(
        event_arguments={6: "axis", 3: "fault", 1: "state", 2: "number", 8: "none"},
        axes={0: "x_driver"},
        fault_names={3: "busy"},
        state_names={2: "homing", 3: "ready"},
    )
    assert state.decode(code, arg) == detail
