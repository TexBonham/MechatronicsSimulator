"""Unit tier tests and stubs for the physics adapter spec (PHY); stubs are strict xfail until converted."""

import math
import re
import time
from pathlib import Path
from typing import Any

import pytest

from mechsim import _core as core
from mechsim.physics.native.prepare import prepare_model as prepare_native
from mechsim.physics.neutral import PhysicsModel, from_yaml
from mechsim.testing.baseline import load_baseline
from mechsim.testing.schema import load_test_case
from tests.invariant.test_phy_stubs import _load, driven

ORIGIN = ("0 m", "0 m", "0 m")
PENDULUM = (Path(__file__).resolve().parents[2] / "schemas/examples/physics_model.pendulum.yaml").read_text()


def native_rig(friction: str = "0 N", lower: str | None = None) -> Any:
    """The pendulum example plus a slider (prismatic along z, optional friction and lower limit), a rope from the
    world to a free load, and a cantilever member on the link; loaded on the native adapter at 1 kHz."""
    data = from_yaml(PENDULUM).model_dump(mode="json")
    dof: dict[str, Any] = {"friction": friction}
    if lower is not None:
        dof["lower"] = lower
    inertia = {"ixx": "0.01 kg*m**2", "iyy": "0.01 kg*m**2", "izz": "0.01 kg*m**2"}
    slider = {"name": "slide", "type": "prismatic", "position": ORIGIN, "axis": (0, 1, 0), "linear": [dof]}
    hang = {"name": "hang", "type": "free", "position": ("2 m", "-1 m", "0 m")}
    data["bodies"] = [
        *data["bodies"],
        {
            "name": "slider",
            "mass": "1 kg",
            "com": ORIGIN,
            "inertia": inertia,
            "material": "steel",
            "parent": "world",
            "joint": slider,
        },
        {
            "name": "load",
            "mass": "1 kg",
            "com": ORIGIN,
            "inertia": inertia,
            "material": "steel",
            "parent": "world",
            "joint": hang,
        },
    ]
    data["ropes"] = [
        {
            "name": "line",
            "from_body": "world",
            "from_point": ("2 m", "0 m", "0 m"),
            "to_body": "load",
            "to_point": ORIGIN,
            "segments": 4,
            "axial_stiffness": "1e5 N",
            "damping": "20 N*s",
            "mass_per_length": "0.01 kg/m",
            "rest_length": "1 m",
        }
    ]
    section = {
        "elastic_modulus": "2e11 Pa",
        "second_moment_of_area": "1e-8 m**4",
        "density": "7800 kg/m**3",
        "area": "1e-4 m**2",
        "length": "0.2 m",
    }
    data["lumped_members"] = [
        {"name": "fin", "parent": "link", "position": ORIGIN, "section": section, "root": "clamped", "end": "free"}
    ]
    prepared = prepare_native(PhysicsModel.model_validate(data), None, 1e-3)
    adapter = core.make_physics_adapter("native")
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(1000))
    return adapter


@pytest.mark.req("PHY-003")
def test_phy_003() -> None:
    """The adapter interface shall report body poses, velocities, joint forces and contact forces after every step."""
    rig = native_rig()
    rig.step()
    out = rig.outputs()
    # The three neutral bodies, then the member's node bodies, root to tip: `fin` has the default 8 segments.
    assert len(out.bodies) == 3 + 8
    assert len(out.joints) == 3
    # Gravity -9.81 along y, the slider's axis: one semi-implicit step from rest gives v = -g h exactly; the hanging
    # link starts at equilibrium and its joint carries no torque.
    assert out.bodies[1].linear_velocity[1] == pytest.approx(-9.81e-3, rel=1e-12)
    assert out.joints[0].generalized_force[0] == pytest.approx(0.0, abs=1e-12)
    assert out.steps == 1


@pytest.mark.req("PHY-004")
def test_phy_004() -> None:
    """The adapter interface shall report rope tension after every step for every rope in the model."""
    rig = native_rig()
    for _ in range(2000):
        rig.step()
    rope = rig.outputs().ropes[0]
    # Engine lumping (ENG-047): 4 segments of 0.0025 kg, three full-mass inner nodes, a half segment mass on each end
    # body. At rest the load-end segment carries the load and its half mass, (1 + 0.00125) g (ruling 40); the top
    # segment carries that plus the three inner nodes, (1 + 0.00875) g, the maximum.
    assert not rope.slack
    assert rope.tension == pytest.approx((1.0 + 0.01 * 0.125) * 9.81, rel=1e-3)
    assert rope.max_tension == pytest.approx((1.0 + 0.01 * 0.875) * 9.81, rel=1e-3)
    assert len(rig.outputs().ropes) == 1


@pytest.mark.req("PHY-007")
def test_phy_007() -> None:
    """Every adapter that carries milestone 3 or later shall publish live member end-loads every tick."""
    rig = native_rig()
    rig.step()
    first = rig.outputs().members[0]
    rig.step()
    assert len(rig.outputs().members) == 1
    assert rig.outputs().members[0].start.force != first.start.force


@pytest.mark.req("PHY-009")
def test_phy_009() -> None:
    """An adapter shall report constraint drift beyond its threshold and solver failure as run errors that end the run
    with a report, never as returned numbers.
    """
    rig = native_rig()
    inputs = core.PhysicsInputs()
    wrench = core.BodyWrench()
    wrench.body = 0
    torque = core.Wrench()
    torque.moment = [math.inf, 0.0, 0.0]
    wrench.wrench = torque
    inputs.body_wrenches = [wrench]
    with pytest.raises(core.AdapterError, match="not finite"):
        rig.set_inputs(inputs)
    rig.release_load(3)
    rig.step()
    with pytest.raises(core.AdapterError, match="release"):
        rig.release_load(3)


def _neutral_values(value: Any, path: str, found: list[str]) -> None:
    """Every value reachable from an adapter's outputs is a builtin or a kernel type of `mechsim._core_ext`; an engine
    type (`mechsim._core_ext.engine`) or anything else is listed."""
    if isinstance(value, bool | int | float | str) or value is None:
        return
    if isinstance(value, list | tuple):
        for index, item in enumerate(value):
            _neutral_values(item, f"{path}[{index}]", found)
        return
    module = type(value).__module__
    if module != "mechsim._core_ext":
        found.append(f"{path}: {module}.{type(value).__name__}")
        return
    if hasattr(type(value), "__members__"):
        return
    for name in dir(value):
        if name.startswith("_") or name in ("name", "value"):
            continue
        attribute = getattr(value, name)
        if callable(attribute):
            continue
        _neutral_values(attribute, f"{path}.{name}", found)


@pytest.mark.req("PHY-012")
@pytest.mark.parametrize("adapter", ["native"])
def test_phy_012(adapter: str) -> None:
    """The only model type that crosses into an adapter shall be the engine-neutral physics model, and the only values
    that cross out shall be neutral types: load takes a PreparedModel (the neutral model's translation) and nothing
    else; every value under outputs() is a kernel type or a builtin, never an engine type."""
    physics = core.make_physics_adapter(adapter)
    model = from_yaml(PENDULUM)
    native = prepare_native(model, None, 1e-3)
    prepared = core.PreparedModel(native.model_json, "", native.version)
    with pytest.raises(TypeError):
        physics.load(model, core.Rate(1000))
    physics.load(prepared, core.Rate(1000))
    physics.step()
    found: list[str] = []
    _neutral_values(physics.outputs(), "outputs", found)
    _neutral_values(physics.health(), "health", found)
    assert found == []
    rig = native_rig()
    rig.step()
    _neutral_values(rig.outputs(), "rig", found)
    assert found == []


@pytest.mark.req("PHY-018")
def test_phy_018() -> None:
    """Every adapter shall map the neutral model's transmissions and lumped members onto its engine."""
    rig = native_rig()
    rig.step()
    assert len(rig.outputs().members) == 1
    assert "lumped_member" in rig.supported_elements()
    assert "transmission.belt_pulley" in rig.supported_elements()


@pytest.mark.req("PHY-020")
def test_phy_020() -> None:
    """The native adapter shall bind the own engine through the core runtime's single Python binding."""
    adapter = core.make_physics_adapter("native")
    assert "rope" in adapter.supported_elements()
    with pytest.raises(core.AdapterError):
        adapter.step()


ROOT = Path(__file__).resolve().parents[2]
BRIDGE_BASELINES = {
    "gantry": ("G1", "G2", "G3", "G4", "G5", "G6", "G7", "G8", "G9", "G10", "G11", "G13", "G14"),
    "pendulum": ("P1", "P2"),
}


@pytest.mark.req("PHY-025")
def test_phy_025() -> None:
    """The comparison recordings used for retirement shall be kept as fixed baselines, stored as YAML scoring summaries
    in git: one file per milestone 1 and 2 case under `test_cases/<suite>/bridge_baselines/`, each a valid baseline
    of that case alone, every entry from a run on the bridge (run id suffix `-mujoco`, TST-040), covering every
    criterion the case states; nothing under `mechsim/` names the folder, so no suite run writes it."""
    for suite, cases in BRIDGE_BASELINES.items():
        folder = ROOT / "test_cases" / suite / "bridge_baselines"
        assert sorted(path.stem for path in folder.glob("*.yaml")) == sorted(cases)
        for case_id in cases:
            baseline = load_baseline(folder / f"{case_id}.yaml")
            assert {entry.case for entry in baseline.entries} == {case_id}
            for entry in baseline.entries:
                assert entry.run is not None
                assert entry.run.run_id.endswith("-mujoco"), entry
            folder_cases = (ROOT / "test_cases" / suite).glob("*.yaml")
            case_file = next(path for path in folder_cases if path.stem.split("_")[0] == case_id)
            stated = {criterion.name for criterion in load_test_case(case_file).criteria}
            assert {entry.criterion for entry in baseline.entries} == stated
    for path in (ROOT / "mechsim").rglob("*.py"):
        assert "bridge_baselines" not in path.read_text(encoding="utf-8"), path


@pytest.mark.req("PHY-027")
def test_phy_027() -> None:
    """Every adapter shall apply the design's gravity, joint friction and joint limits."""
    rig = native_rig("2 N", "-0.001 m")
    # Gravity 9.81 down the slider's axis exceeds its friction F = 2 N, so the 1 kg slider slides onto its lower bound;
    # the engine's limit row (ENG-013, ruling 43) lands it on the bound with no penetration, and at rest the joint's
    # generalized force is the load it carries, m g - F at most and m g once the friction's frozen rate is the slip
    # threshold at zero speed (a body at rest on the bound: the friction damper sees v = 0 and carries nothing).
    for _ in range(2000):
        rig.step()
    out = rig.outputs()
    assert out.joint_positions[1] == pytest.approx(-0.001, abs=1e-12)
    assert out.bodies[1].linear_velocity[1] == pytest.approx(0.0, abs=1e-12)
    assert out.joints[1].generalized_force[0] == pytest.approx(9.81, rel=1e-9)
    assert out.bodies[0].angular_velocity[2] != 0.0


def _faller(adapter: str, hz: int) -> Any:
    data = from_yaml(PENDULUM).model_dump(mode="json")
    data["gravity"] = ("0 m/s**2", "0 m/s**2", "-9.81 m/s**2")
    data["signals"] = {}
    data["bodies"] = [
        {
            "name": "slider",
            "mass": "1 kg",
            "com": ORIGIN,
            "inertia": {"ixx": "0.01 kg*m**2", "iyy": "0.01 kg*m**2", "izz": "0.01 kg*m**2"},
            "material": "steel",
            "parent": "world",
            "joint": {"name": "drop", "type": "prismatic", "position": ORIGIN, "axis": (0, 0, 1)},
        }
    ]
    model = PhysicsModel.model_validate(data)
    physics = core.make_physics_adapter(adapter)
    native = prepare_native(model, None, 1.0 / hz)
    physics.load(core.PreparedModel(native.model_json, "", native.version), core.Rate(hz))
    return physics


@pytest.mark.req("PHY-031")
def test_phy_031() -> None:
    """Integrator choice and step size shall be per-adapter configuration, and no adapter shall change another
    adapter's configuration: three adapters loaded side by side at 1000, 400 and 250 Hz each keep their own rate and
    fall by their own step's closed form. Semi-implicit Euler under constant g: v_n = -g n h exactly; after n steps
    the speed names the step the adapter used."""
    adapters = [("native", 1000), ("native", 400), ("native", 250)]
    loaded = [(_faller(name, hz), hz) for name, hz in adapters]
    for _ in range(10):
        for physics, _hz in loaded:
            physics.step()
    for physics, hz in loaded:
        assert physics.health().physics_rate.numerator == hz
        assert physics.health().steps == 10
        assert physics.outputs().bodies[0].linear_velocity[2] == pytest.approx(-9.81 * 10 / hz, rel=1e-9)


@pytest.mark.req("PHY-032")
@pytest.mark.parametrize("adapter", ["native"])
def test_phy_032(adapter: str) -> None:
    """An adapter shall advance simulated time only on a step call and shall not read wall-clock time: outputs,
    health and inputs leave the step count and the state alone, and two runs with a pause between steps of one of
    them are bitwise equal; the native sources name no clock."""
    physics = _faller(adapter, 1000)
    before = physics.outputs().bodies[0].position[2]
    for _ in range(5):
        physics.outputs()
        physics.health()
        physics.set_inputs(core.PhysicsInputs())
    assert physics.health().steps == 0
    assert physics.outputs().steps == 0
    assert physics.outputs().bodies[0].position[2] == before
    paused = _faller(adapter, 1000)
    plain = _faller(adapter, 1000)
    for n in range(20):
        if n == 10:
            time.sleep(0.05)
        paused.step()
        plain.step()
    assert paused.outputs().bodies[0].position[2] == plain.outputs().bodies[0].position[2]
    assert paused.health().steps == 20
    clocks = re.compile(r"<chrono>|<ctime>|std::chrono|\bclock\(|\btime\(|gettimeofday|import time|datetime")
    root = Path(__file__).resolve().parents[2]
    sources = [
        *sorted((root / "core/physics/native").rglob("*.[ch]pp")),
        *sorted((root / "mechsim/physics/native").rglob("*.py")),
    ]
    assert len(sources) > 3
    assert [path.name for path in sources if "tests" not in path.parts and clocks.search(path.read_text())] == []


@pytest.mark.req("PHY-034")
def test_phy_034_native_joint_drive_on_a_belt_shaft_moves_the_driven_joint() -> None:
    """A drive toward 0.5 rad on the rotor of `driven()` (pulley radius 0.02 m, no load along the slide) must carry
    the carriage to 0.5 x 0.02 = 0.01 m. Reflected inertia J = 0.001 + 1 x 0.02^2 = 0.0014 kg m^2, K = 1 N m/rad,
    D = 0.05 N m s/rad: omega = 26.7 rad/s, zeta = D / (2 sqrt(K J)) = 0.67, decay time 0.056 s; after 1 s the
    transient is below e^-17 of 0.01 m, and the static belt stretch is zero (no load), so 1e-6 m holds."""
    physics = _load(driven(), "native")
    for _ in range(1000):
        inputs = core.PhysicsInputs()
        inputs.joint_drives = [core.JointDrive(0, 0.5, 1.0, 0.05, True)]
        physics.set_inputs(inputs)
        physics.step()
    assert physics.outputs().bodies[1].position[0] == pytest.approx(0.5 * 0.02, abs=1e-6)
