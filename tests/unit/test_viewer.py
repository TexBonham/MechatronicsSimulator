"""The viewer module (VIE-002 to VIE-011, VIE-014 to VIE-017): probes, timeline, geometry, overlays, replay cursor."""

from __future__ import annotations

import ast
import math
import os
import subprocess
import sys
from fractions import Fraction
from pathlib import Path

import numpy as np
import pytest

from mechsim.design.schema import DesignError
from mechsim.record.stream import Recorder
from mechsim.scoring.score import CriterionResult
from mechsim.testing.driver import MemberSection, RecordedEvent
from mechsim.viewer import (
    OverlayError,
    ProbeError,
    ReplayCursor,
    ReplayError,
    body_geometry,
    category_of,
    down_from_design,
    member_stress,
    probe,
    swing_angle,
    timeline,
)
from tests.unit.test_record_support import synthetic_rows
from tests.unit.test_runner_fakes import FakeRecording, recording_from

ROOT = Path(__file__).resolve().parents[2]
TIMES = [0.0, 0.001, 0.002, 0.003]
ONE_STEP_BOUND = 9.81 * 0.25 * math.sin(0.05) * 12.0 * 0.001**2


# ===== Timeline (VIE-002) ===============================================


@pytest.mark.req("VIE-002")
def test_timeline_marks_commands_faults_safety_events_and_criterion_failures_verbatim() -> None:
    events = [
        RecordedEvent(0.001, "operator", "operator_message", "Move to target"),
        RecordedEvent(0.002, "hoist_motor", "motor_stall", "demand above available torque"),
        RecordedEvent(0.002, "supply", "rail_off", "0 V"),
        RecordedEvent(0.003, "estop_relay", "estop_pressed", ""),
        RecordedEvent(0.003, "firmware", "motion_start", "trolley"),
        RecordedEvent(0.004, "board", "fault_injected", "sensor_stuck"),
    ]
    criteria = [
        CriterionResult("period", "fail", "limit", 1.0, "s", 0.0025),
        CriterionResult("final", "pass", "limit", 1.0, "s", 0.001),
        CriterionResult("settle", "fail", "limit", None, "", None),
    ]
    entries = timeline(events, criteria, end_time=0.005)
    assert [(e.t, e.category, e.label, e.source) for e in entries] == [
        (0.001, "command", "Move to target", "operator"),
        (0.002, "fault", "demand above available torque", "hoist_motor"),
        (0.002, "fault", "0 V", "supply"),
        (0.0025, "criterion_failure", "period", "scoring"),
        (0.003, "safety_event", "estop_pressed", "estop_relay"),
        (0.004, "fault", "sensor_stuck", "board"),
        (0.005, "criterion_failure", "settle", "scoring"),
    ]


@pytest.mark.req("VIE-002")
def test_event_kinds_map_to_timeline_categories() -> None:
    assert category_of("operator_command") == "command"
    assert category_of("operator_status") == "command"
    assert category_of("rail_on") == "fault"
    assert category_of("safety_trip") == "safety_event"
    assert category_of("motion_end") is None
    assert timeline([], []) == ()


# ===== Probes (VIE-003 to VIE-006) ======================================


def _view() -> FakeRecording:
    series = {
        "pose:link.x": [0.0, 0.1, 0.2, 0.3],
        "pose:link.yaw": [0.0, 0.01, 0.02, 0.03],
        "joint:pivot.position": [0.05, 0.04, 0.03, 0.02],
        "joint:pivot.velocity": [0.0, -10.0, -10.0, -10.0],
        "signal:mcu.pa0": [0.0, 3.3, 3.3, 0.0],
    }
    return recording_from(TIMES, series, "u")


@pytest.mark.req("VIE-003")
def test_body_probe_shows_its_recorded_pose_over_time() -> None:
    found = probe(_view(), "body", "link")
    assert found.times == tuple(TIMES)
    assert [column.name for column in found.columns] == ["x", "yaw"]
    assert found.value_at(2) == {"x": 0.2, "yaw": 0.02}
    with pytest.raises(ProbeError):
        probe(_view(), "body", "nothing")


@pytest.mark.req("VIE-004")
def test_joint_probe_shows_position_and_velocity_over_time() -> None:
    found = probe(_view(), "joint", "pivot")
    assert [(c.name, c.values) for c in found.columns] == [
        ("position", (0.05, 0.04, 0.03, 0.02)),
        ("velocity", (0.0, -10.0, -10.0, -10.0)),
    ]
    with pytest.raises(ProbeError):
        probe(_view(), "joint", "nothing")


@pytest.mark.req("VIE-005", "VIE-006")
def test_pin_and_signal_probes_show_the_recorded_signal() -> None:
    for kind in ("pin", "signal"):
        found = probe(_view(), kind, "mcu.pa0")  # type: ignore[arg-type]
        assert [(c.name, c.unit, c.values) for c in found.columns] == [("value", "u", (0.0, 3.3, 3.3, 0.0))]
    with pytest.raises(ProbeError):
        probe(_view(), "signal", "nothing")


class _RawView:
    """A record view carrying the raw recording and kinematics, as the core record view does."""

    def __init__(self) -> None:
        recorder = Recorder(Fraction(1000))
        self.rows = synthetic_rows(5, 4, events=False, contacts=False)
        for row in self.rows:
            recorder.write_row(row)
        self.recording = recorder.recording()
        joints = (_Joint("a"), _Joint("b"))
        self.kinematics = _Kinematics(("base", "arm"), joints)

    def times(self) -> list[float]:
        return list(TIMES)

    def series(self, reference: str) -> object:
        known = {"pose:arm.x", "joint:b.position"}
        if reference in known:
            return recording_from(TIMES, {reference: [1.0, 2.0, 3.0, 4.0]}, "m").series(reference)
        if reference == "signal:mcu.pa0" or reference == "signal:supply.24v":
            name = reference.removeprefix("signal:")
            group = "signals/pins"
            if name == "supply.24v":
                group = "signals/rails"
            stored = self.recording.series[f"{group}/{name}"]
            return recording_from(TIMES, {reference: list(stored.values)}, stored.unit).series(reference)
        raise KeyError(reference)


class _Joint:
    def __init__(self, name: str) -> None:
        self.name = name


class _Kinematics:
    def __init__(self, bodies: tuple[str, ...], joints: tuple[_Joint, ...]) -> None:
        self.bodies = bodies
        self.joints = joints


@pytest.mark.req("VIE-003", "VIE-004", "VIE-005")
def test_probes_add_raw_velocities_and_joint_forces_and_tell_pins_from_rails() -> None:
    view = _RawView()
    body = probe(view, "body", "arm")
    assert [c.name for c in body.columns] == ["x", "vx", "vy", "vz", "wx", "wy", "wz"]
    assert body.columns[1].values == tuple(row.physics.bodies[1].linear_velocity[0] for row in view.rows)
    assert body.columns[4].values == tuple(row.physics.bodies[1].angular_velocity[0] for row in view.rows)
    joint = probe(view, "joint", "b")
    # JOINT_DOFS = (1, 3) in the synthetic rows: joint b's generalized force is the second to fourth column.
    assert [c.name for c in joint.columns] == ["position", "force[0]", "force[1]", "force[2]"]
    forces = np.asarray(view.recording.series["physics/joints/generalized_force"].values)
    assert joint.columns[3].values == tuple(float(v) for v in forces[:, 3])
    assert probe(view, "pin", "mcu.pa0").columns[0].unit == "V"
    with pytest.raises(ProbeError):
        probe(view, "pin", "supply.24v")
    assert probe(view, "signal", "supply.24v").columns[0].unit == "V"


# ===== Replay cursor (VIE-007 to VIE-009) ===============================


@pytest.mark.req("VIE-007")
def test_scrub_lands_on_the_latest_recorded_time_at_or_before() -> None:
    cursor = ReplayCursor(TIMES)
    assert cursor.scrub(0.0025).t == 0.002
    assert cursor.scrub(0.002).index == 2
    assert cursor.scrub(9.0).index == 3
    assert cursor.scrub(0.0).index == 0
    late = ReplayCursor([0.5, 1.0])
    assert late.scrub(0.1).t == 0.5
    for bad in (-1.0, float("nan"), float("inf")):
        with pytest.raises(ReplayError):
            cursor.scrub(bad)
    with pytest.raises(ReplayError):
        ReplayCursor([])


@pytest.mark.req("VIE-008")
def test_pause_and_resume_gate_the_play_loop() -> None:
    cursor = ReplayCursor(TIMES)
    assert cursor.state().paused
    assert cursor.advance(0.002).index == 0
    assert not cursor.pause(False).paused
    assert cursor.advance(0.002).t == 0.002
    assert cursor.advance(0.0005).t == 0.002
    assert cursor.pause(True).paused
    assert cursor.advance(1.0).t == 0.002
    with pytest.raises(ReplayError):
        cursor.advance(-0.001)


@pytest.mark.req("VIE-009")
def test_step_moves_one_recorded_sample_and_stops_at_the_ends() -> None:
    cursor = ReplayCursor(TIMES)
    assert cursor.step("backward").index == 0
    assert [cursor.step("forward").index for _ in range(4)] == [1, 2, 3, 3]
    assert cursor.step("backward").t == 0.002


# ===== Served through the facade (VIE-014), geometry (VIE-015) ==========


@pytest.mark.req("VIE-014")
def test_the_facade_serves_what_the_viewer_computes(tmp_path: Path) -> None:
    from tests.unit.test_facade_service import run_machine

    facade, summary = run_machine(tmp_path)
    facade.replay_open(Path(summary.archive))
    served = facade.probe("M1-seed7", "joint", "pivot")
    _, view = facade.services.reader.open(Path(summary.archive))  # type: ignore[union-attr]
    computed = probe(view, "joint", "pivot")
    assert served.times == list(computed.times)
    assert [(c.name, c.values) for c in served.columns] == [(c.name, list(c.values)) for c in computed.columns]
    served_timeline = facade.timeline("M1-seed7").entries
    computed_timeline = timeline(view.events(), [], 0.01)
    assert [(e.t, e.category, e.label) for e in served_timeline][:2] == [
        (e.t, e.category, e.label) for e in computed_timeline
    ]
    assert [(e.category, e.label) for e in served_timeline][2:] == [("criterion_failure", "final_angle")]
    assert facade.scrub("M1-seed7", 0.0045).t == 0.004
    assert facade.step("M1-seed7", "forward").t == 0.005
    assert facade.pause("M1-seed7", False).paused is False
    assert facade.advance("M1-seed7", 0.002).t == 0.007


@pytest.mark.req("VIE-015")
def test_body_geometry_comes_from_the_design_file_bytes() -> None:
    data = (ROOT / "designs" / "pendulum" / "design.yaml").read_bytes()
    bodies = body_geometry(data, ROOT / "designs" / "pendulum")
    assert [(b.body, b.kind, b.file) for b in bodies] == [("link", "primitives", None)]
    shape = bodies[0].shapes[0]
    assert (shape.shape, shape.position, shape.sizes) == ("cylinder", (0.25, 0.0, 0.0), {"radius": 0.01, "length": 0.5})
    assert shape.orientation == (0.7071067811865476, 0.0, 0.7071067811865476, 0.0)
    for broken in (b"", b"a: [", b"name: x\n"):
        with pytest.raises(DesignError, match="design file"):
            body_geometry(broken)


@pytest.mark.req("VIE-015")
def test_box_sphere_capsule_and_open_sizes() -> None:
    text = (ROOT / "designs" / "pendulum" / "design.yaml").read_text(encoding="utf-8")
    shapes = """        - shape: box
          position: [0 m, 0 m, 0 m]
          size: [0.1 m, 0.2 m, 0.3 m]
        - shape: sphere
          position: [0 m, 0 m, 0 m]
          radius: 0.05 m
        - shape: capsule
          position: [0 m, 0 m, 0 m]
          radius: 0.02 m
          length: 0.1 m
"""
    edited = text.replace("      shapes:\n", "      shapes:\n" + shapes, 1)
    bodies = body_geometry(edited.encode())
    sizes = [(s.shape, s.sizes) for s in bodies[0].shapes]
    assert sizes[:3] == [
        ("box", {"size": (0.1, 0.2, 0.3)}),
        ("sphere", {"radius": 0.05}),
        ("capsule", {"radius": 0.02, "length": 0.1}),
    ]


# ===== Without the engine (VIE-016), no files (VIE-017) =================


@pytest.mark.req("VIE-016")
def test_replay_works_without_the_engine(tmp_path: Path) -> None:
    """A real P1 archive replays through the shipped reader with the core extension and MuJoCo unimportable."""
    from mechsim.facade.service import Facade, default_services

    case = ROOT / "test_cases" / "pendulum" / "P1_period.yaml"
    summary = Facade(default_services()).run_case(case, out_dir=tmp_path)
    assert summary.archive is not None
    # Released at rest from 0.05 rad, one 1 ms semi-implicit Euler step moves the angle by at most alpha dt^2 with
    # alpha = m g d sin(0.05) / I = 9.81 * 0.25 * sin(0.05) * 12 rad/s^2 (m = 1 kg, d = 0.25 m, I = 1/12 kg m^2).
    script = f"""
import sys

class Block:
    def find_spec(self, name, path=None, target=None):
        if name.startswith("mechsim._core") or name == "mujoco":
            raise ImportError("engine blocked: " + name, name=name)
        return None

sys.meta_path.insert(0, Block())
from pathlib import Path
from mechsim.facade.service import Facade, default_services

services = default_services()
assert services.runner is None and services.live is None
facade = Facade(services)
handle = facade.replay_open(Path({summary.archive!r}))
assert facade.scrub(handle.run_id, 12.0005).index == 11999
assert facade.step(handle.run_id, "backward").index == 11998
angle = facade.value_at(handle.run_id, "joint:pivot.position", 0.0)
assert angle.t == 0.001 and abs(angle.value - 0.05) <= {ONE_STEP_BOUND!r}
assert facade.probe(handle.run_id, "body", "link").columns[6].name == "vx"
assert facade.geometry(handle.run_id).bodies[0]["body"] == "link"
assert not any(name.startswith("mechsim._core") for name in sys.modules) and "mujoco" not in sys.modules
print("replayed", handle.samples)
"""
    result = subprocess.run(
        [sys.executable, "-c", script], cwd=ROOT, capture_output=True, text=True, check=False, env=dict(os.environ)
    )
    assert result.returncode == 0, result.stderr
    # P1 records one row per 1 ms tick from 0.001 s to its end at 24.4 s.
    assert result.stdout == "replayed 24400\n"


@pytest.mark.req("VIE-017")
def test_the_viewer_writes_no_files(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    for source in sorted((ROOT / "mechsim" / "viewer").glob("*.py")):
        tree = ast.parse(source.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                name = ""
                if isinstance(node.func, ast.Attribute):
                    name = node.func.attr
                elif isinstance(node.func, ast.Name):
                    name = node.func.id
                assert name not in ("open", "write_text", "write_bytes", "mkdir", "touch"), f"{source}: {name}"
    monkeypatch.chdir(tmp_path)
    probe(_view(), "joint", "pivot")
    timeline([RecordedEvent(0.0, "operator", "operator_message", "go")], [])
    body_geometry((ROOT / "designs" / "pendulum" / "design.yaml").read_bytes())
    ReplayCursor(TIMES).scrub(0.001)
    assert list(tmp_path.iterdir()) == []


# ===== Fractional play time (VIE-009), overlays (VIE-010, VIE-011) =======


@pytest.mark.req("VIE-009")
def test_advance_carries_the_time_left_over_past_the_shown_sample() -> None:
    # Binary-exact times so the carried sums are exact: two advances of 0.125 s reach the 0.25 s sample (dropping the
    # leftover would leave the cursor on 0 s forever).
    cursor = ReplayCursor([0.0, 0.25, 0.5, 0.75])
    cursor.pause(False)
    assert cursor.advance(0.125).index == 0
    assert cursor.advance(0.125).index == 1
    assert cursor.step("forward").t == 0.5
    assert cursor.advance(0.125).index == 2
    assert cursor.advance(0.125).index == 3
    assert cursor.advance(10.0).index == 3
    assert cursor.step("backward").t == 0.5
    assert cursor.advance(0.125).index == 2
    assert cursor.scrub(0.3).index == 1
    assert cursor.advance(0.2).index == 2
    assert cursor.scrub(-0.0).index == 0
    assert cursor.advance(0.25).index == 1


def _structural_view() -> object:
    from tests.unit.test_record_structural import TICKS, _columns, _row

    recorder = Recorder(Fraction(1000), _columns())
    for tick in range(TICKS):
        recorder.write_row(_row(tick))

    class _View:
        recording = recorder.recording()

        def times(self) -> list[float]:
            return [tick / 1000.0 for tick in range(TICKS)]

        def series(self, reference: str) -> object:
            raise KeyError(reference)

    return _View()


# Rectangle, width w = 0.02 m across (z), height h = 0.04 m along the bending direction (y): A = w h = 8e-4 m^2,
# I_z = w h^3 / 12 = 1.0667e-7 m^4, I_y = h w^3 / 12 = 2.6667e-8 m^4, c_y = h / 2 = 0.02 m, c_z = w / 2 = 0.01 m.
W, H = 0.02, 0.04
A, I_Z, I_Y, C_Y, C_Z = W * H, W * H**3 / 12.0, H * W**3 / 12.0, H / 2.0, W / 2.0
SECTIONS = (
    MemberSection("rail_x", "petg_printed", A, I_Z, 0.5, I_Y, C_Y, C_Z),
    MemberSection("rail_y", "petg_printed", A, I_Z, 0.5, I_Y, C_Y, C_Z),
)


@pytest.mark.req("VIE-010")
def test_member_stress_is_axial_plus_both_bending_terms_at_each_end() -> None:
    # Member k at tick t: start force (10 k, t, 0.5), start moment (t + 0.1 k, 2 t, -1 - k); end force (1, -2, t + k),
    # end moment (-t, 3 + k, 4 t), member frame. sigma = |N| / A + |M_y| c_z / I_y + |M_z| c_y / I_z (ruling 25):
    # start 10 k / A + 2 t c_z / I_y + (1 + k) c_y / I_z; end 1 / A + (3 + k) c_z / I_y + 4 t c_y / I_z.
    found = member_stress(_structural_view(), SECTIONS)
    assert [entry.member for entry in found] == ["rail_x", "rail_y"]
    for k, entry in enumerate(found):
        assert (entry.area, entry.second_moment_y, entry.second_moment_z, entry.fibre_y, entry.fibre_z) == (
            A,
            I_Y,
            I_Z,
            C_Y,
            C_Z,
        )
        start = [10.0 * k / A + 2.0 * t * C_Z / I_Y + (1.0 + k) * C_Y / I_Z for t in range(6)]
        end = [1.0 / A + (3.0 + k) * C_Z / I_Y + 4.0 * t * C_Y / I_Z for t in range(6)]
        assert entry.start == pytest.approx(start, rel=1e-12)
        assert entry.end == pytest.approx(end, rel=1e-12)


@pytest.mark.req("VIE-010")
def test_member_stress_refuses_a_section_without_y_axis_properties() -> None:
    bare = (SECTIONS[0], MemberSection("rail_y", "petg_printed", A, I_Z, 0.5))
    with pytest.raises(OverlayError, match="'rail_y': the section states no y-axis second moment"):
        member_stress(_structural_view(), bare)


@pytest.mark.req("VIE-010")
@pytest.mark.parametrize(
    ("shape", "expected"),
    [
        # Rectangle w = 0.02 (z), h = 0.04 (y): A = 8e-4, I_y = h w^3 / 12, I_z = w h^3 / 12, c_y = 0.02, c_z = 0.01;
        # Roark's J = a b^3 (1/3 - 0.21 (b/a)(1 - b^4 / (12 a^4))) with a = 0.04, b = 0.02:
        # 3.2e-7 (1/3 - 0.105 (1 - 1/192)) = 7.32417e-8; shear areas 5/6 A.
        (
            {"kind": "rectangle", "width": "0.02 m", "height": "0.04 m"},
            (
                8e-4,
                2.6666666666666667e-8,
                1.0666666666666667e-7,
                0.02,
                0.01,
                3.2e-7 * (1.0 / 3.0 - 0.105 * (1.0 - 1.0 / 192.0)),
                6.6666666666666667e-4,
            ),
        ),
        # Circle d = 0.02: A = pi d^2 / 4 = 3.14159e-4, I = pi d^4 / 64 = 7.85398e-9, c = 0.01, J = 2 I, shear 6/7 A.
        (
            {"kind": "circle", "diameter": "0.02 m"},
            (
                math.pi * 1e-4,
                math.pi * 1.6e-7 / 64.0,
                math.pi * 1.6e-7 / 64.0,
                0.01,
                0.01,
                math.pi * 1.6e-7 / 32.0,
                6.0 / 7.0 * math.pi * 1e-4,
            ),
        ),
        # Tube 0.02 / 0.016: A = pi (4e-4 - 2.56e-4) / 4, I = pi (1.6e-7 - 6.5536e-8) / 64, c = 0.01, J = 2 I;
        # m^2 = 0.64: Cowper (nu = 0) 6 (1.64)^2 / (7 (1.64)^2 + 20 (0.64)) = 16.1376 / 31.6272 = 0.510244.
        (
            {"kind": "tube", "outer_diameter": "0.02 m", "inner_diameter": "0.016 m"},
            (
                math.pi * 1.44e-4 / 4.0,
                math.pi * 9.4464e-8 / 64.0,
                math.pi * 9.4464e-8 / 64.0,
                0.01,
                0.01,
                math.pi * 9.4464e-8 / 32.0,
                16.1376 / 31.6272 * math.pi * 1.44e-4 / 4.0,
            ),
        ),
    ],
)
def test_section_shapes_give_every_property_by_hand(shape: dict[str, str], expected: tuple[float, ...]) -> None:
    from mechsim.design.schema import Section

    section = Section.model_validate({"shape": shape, "length": "0.5 m"})
    found = section.properties()
    assert found is not None
    measured = (
        found.area,
        found.second_moment_y,
        found.second_moment_z,
        found.fibre_y,
        found.fibre_z,
        found.torsion_constant,
        found.shear_area_y,
    )
    assert measured == pytest.approx(expected, rel=1e-12)
    assert found.shear_area_z == found.shear_area_y
    assert Section.model_validate({"area": "1 m**2", "second_moment": "1 m**4", "length": "1 m"}).properties() is None


@pytest.mark.req("VIE-010")
def test_member_stress_needs_the_recording_named_columns_and_every_section() -> None:
    with pytest.raises(OverlayError, match="needs the archived recording"):
        member_stress(recording_from(TIMES, {}), SECTIONS)
    with pytest.raises(OverlayError, match="'rail_y' is not in the run's member list"):
        member_stress(_structural_view(), SECTIONS[:1])
    view = _structural_view()
    unnamed = Recorder(Fraction(1000))
    from tests.unit.test_record_structural import _row

    unnamed.write_row(_row(0))
    view.recording = unnamed.recording()  # type: ignore[attr-defined]
    with pytest.raises(OverlayError, match="carry no names"):
        member_stress(view, SECTIONS)
    empty = Recorder(Fraction(1000))
    empty.write_row(synthetic_rows(1, 1)[0].__class__(time=0))
    view.recording = empty.recording()  # type: ignore[attr-defined]
    assert member_stress(view, SECTIONS) == ()


def _hanging(angles: list[float], length: float = 1.5) -> FakeRecording:
    # Trolley at (0.1 t, 0.2, 2) moving along x; the load hangs `length` below it, swung by theta in the x-z plane.
    times = [float(n) for n in range(len(angles))]
    trolley = [(0.1 * t, 0.2, 2.0) for t in times]
    load = [
        (x + length * math.sin(a), y, z - length * math.cos(a)) for (x, y, z), a in zip(trolley, angles, strict=True)
    ]
    series: dict[str, list[float]] = {}
    for body, points in (("trolley", trolley), ("hook_load", load)):
        for axis, column in zip("xyz", zip(*points, strict=True), strict=True):
            series[f"pose:{body}.{axis}"] = list(column)
    return recording_from(times, series, "m")


@pytest.mark.req("VIE-011")
def test_swing_angle_from_the_vertical_through_the_attachment() -> None:
    angles = [0.0, 0.1, -0.2, 0.05, 1.2, math.pi / 2, 2.0]
    found = swing_angle(_hanging(angles), "hook_load", "trolley")
    # The angle is unsigned (distance from the vertical): |theta| for |theta| <= pi.
    assert found.values == pytest.approx([abs(a) for a in angles], abs=1e-12)
    assert (found.load, found.attachment) == ("hook_load", "trolley")
    fixed = swing_angle(_hanging([0.3, 0.3]), "trolley", "world", (0.0, 0.2, 3.0))
    # The trolley 1 m below a world point straight above its start, then 0.1 m along x: atan2(0.1, 1).
    assert fixed.values == pytest.approx([0.0, math.atan2(0.1, 1.0)], abs=1e-12)
    with pytest.raises(OverlayError, match="no body 'crane'"):
        swing_angle(_hanging([0.0]), "hook_load", "crane")


@pytest.mark.req("VIE-011")
def test_the_vertical_comes_from_the_design_gravity() -> None:
    pendulum = (Path(__file__).resolve().parents[2] / "designs" / "pendulum" / "design.yaml").read_bytes()
    assert down_from_design(pendulum) == (0.0, -1.0, 0.0)
    assert down_from_design(b"") == (0.0, 0.0, -1.0)
    assert down_from_design(b"\xff\xfe") == (0.0, 0.0, -1.0)
    weightless = b"environment:\n  gravity: [0 m/s**2, 0 m/s**2, 0 m/s**2]\n"
    assert down_from_design(weightless) == (0.0, 0.0, -1.0)
    # A load 1 m from a world attachment at the origin, swung by 0.2 rad from -y in the x-y plane (the pendulum's
    # gravity): r = (sin 0.2, -cos 0.2, 0), so the angle from down = (0, -1, 0) is 0.2 rad.
    series = {"pose:bob.x": [math.sin(0.2)], "pose:bob.y": [-math.cos(0.2)], "pose:bob.z": [0.0]}
    found = swing_angle(recording_from([0.0], series, "m"), "bob", "world", down=(0.0, -1.0, 0.0))
    assert found.values == pytest.approx((0.2,), abs=1e-12)
