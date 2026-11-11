"""Milestone 2 wiring of the gantry (M2-X1): payload body, brake, load cell chain, faults, labels and firmware events.

Brake bound (core_wiring module docstring): on a prismatic joint driven by a transmission of c metres per radian the
holding force is brake_torque / c and A0 = 1 / (m + sum J / c^2), m the mass below the joint, J the shaft's axial
inertia. Both are closed forms of the neutral model, so the test recomputes them from the model's own fields.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

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
    # hook origin (0.1, 0.1, 0.69) at the reference configuration; hook box 0.04 m: extent sqrt(3) 0.02.
    drop = math.sqrt(3.0) * 0.02 + PAYLOAD_EDGE / 2.0 + PAYLOAD_CLEARANCE
    assert body.joint.position == pytest.approx((0.1, 0.1, 0.69 - drop), abs=1e-12)


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
def test_brake_on_a_driven_prismatic_joint_reflects_the_rotor_and_is_reported(built: object) -> None:
    model = built.physics  # type: ignore[attr-defined]
    places, _ = core_wiring.joint_places(model)
    brake = _instances(built)["lift_brake"].model_copy(update={"acts_on": "lift_axis"})  # type: ignore[attr-defined]
    params = core_wiring.brake_parameters(brake, places, model)
    screw = next(item for item in model.transmissions if item.name == "lift_screw")
    c = screw.lead / (2.0 * math.pi)
    names = [body.name for body in model.bodies]
    mass = sum(model.bodies[names.index(name)].mass for name in ("lift_carriage", "hook"))
    shaft = model.bodies[names.index("lift_motor_shaft")]
    assert params["joint"] == places["lift_axis"].body
    assert params["brake_torque"] == pytest.approx(0.5 / c, rel=1e-12)
    assert params["inverse_inertia"] == pytest.approx(1.0 / (mass + core_wiring.axial_inertia(shaft) / c**2), rel=1e-12)
    line = core_wiring.undrivable_brake(brake, model)
    assert line is not None
    assert "not applied" in line
    assert "lift_screw" in line
    assert core_wiring.undrivable_brake(_instances(built)["x_motor"], model) is None  # type: ignore[arg-type]


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
    with pytest.raises(core_wiring.WiringError, match="no payload 'payload'"):
        core_wiring.component_parameters(_instances(built)["load_cell"], places, model, [], chip)  # type: ignore[arg-type]
    with pytest.raises(core_wiring.WiringError, match="no chip"):
        core_wiring.component_parameters(_instances(built)["load_amp"], places, model, [], None)  # type: ignore[arg-type]


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
