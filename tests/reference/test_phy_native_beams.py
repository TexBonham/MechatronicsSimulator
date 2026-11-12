"""Lumped beams through the native adapter against their closed forms (BLD-017, PHY-022, PHY-007; rulings 16, 24, 31).

Steel-like member: E I = 2e11 Pa x 1e-8 m^4 = 2000 N m^2, rho A = 7800 x 1e-4 = 0.78 kg/m, L = 1 m, no gravity,
along world x on a fixed base (member frame = world). The load acts along y (bending about the root z axis).
Member node poses come after the neutral bodies (`member_node_names`); a point x on node k lies at the node pose
plus R (x - start_k) along the node's x axis.

Statics: node damping D = (2 / omega_c) E I / l per node gives every mode zeta_i = omega_i / omega_c >= 1 for
omega_c the closed-form first mode, so 3000 steps of 1 ms leave below exp(-75) of the transient; the backward Euler
fixed point is the chain's static solution. The load gives a deflection of 1e-4 L, so the geometric terms (theta^2)
are below 1e-7 relative.

Chain values (the lumped error inside the 2 percent of rulings 20 and 32):
- cantilever, tip load P, springs at segment midpoints: delta = P l^3 / (E I) sum_{k=0}^{N-1} (k + 1/2)^2
  = P L^3 / (3 E I) (1 - 1 / (4 N^2)), -0.39 percent at N = 8;
- clamped-clamped, midspan load: P L^3 / (192 E I) (1 - 4 / N^2), -1.56 percent at N = 16 (engine contract).
First modes: lumped +2.49 percent clamped-clamped at N = 8 and +0.61 percent at 16 (engine contract); backward Euler
runs (omega h)^2 / 3 slow: cantilever omega = 178 rad/s at 10 kHz is 1e-4, clamped-clamped 1133 rad/s at 50 kHz is
1.7e-4. Higher modes contract by (1 + (omega_k h)^2)^(-1/2) per step and are gone before the counted crossings.
"""

from __future__ import annotations

import math
from typing import Any

import pytest

from mechsim import _core as core
from mechsim.builder.structure import chain_layout
from mechsim.physics.native import translate
from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.neutral import PhysicsModel
from tests.reference.test_phy_stubs import ORIGIN, base, block

E = 2e11
I_Z = 1e-8
RHO = 7800.0
AREA = 1e-4
LENGTH = 1.0
EI = E * I_Z
MU = RHO * AREA
TOLERANCE = 0.02


def beam_model(end: str, segments: int, node_damping: float) -> PhysicsModel:
    data = base(("0 m/s**2", "0 m/s**2", "0 m/s**2"))
    data["bodies"] = [block("base", 1.0, {"name": "mount", "type": "fixed", "position": ORIGIN})]
    data["lumped_members"] = [
        {
            "name": "beam",
            "parent": "base",
            "position": ORIGIN,
            "section": {
                "elastic_modulus": f"{E} Pa",
                "second_moment_of_area": f"{I_Z} m**4",
                "density": f"{RHO} kg/m**3",
                "area": f"{AREA} m**2",
                "length": f"{LENGTH} m",
            },
            "root": "clamped",
            "end": end,
            "segments": segments,
            "node_damping": f"{node_damping} N*m*s/rad",
        }
    ]
    return PhysicsModel.model_validate(data)


def _rotate(q: Any, v: tuple[float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


class Beam:
    """The member on the native adapter with a point load along y held at x_load; `point_y` reads that point."""

    def __init__(self, end: str, segments: int, hz: int, node_damping: float, x_load: float, load: float) -> None:
        model = beam_model(end, segments, node_damping)
        prepared = prepare_model(model, None, 1.0 / hz)
        self.adapter = core.make_physics_adapter("native")
        self.adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(hz))
        names = translate.member_node_names(model)
        assert len(names) == segments
        assert len(self.adapter.outputs().bodies) == 1 + len(names)
        chain = chain_layout("clamped", end, segments, LENGTH)
        node = max(k for k, start in enumerate(chain.starts) if start <= x_load + 1e-12)
        self.body = 1 + node
        self.offset = x_load - chain.starts[node]
        self.arm = x_load - (chain.starts[node] + chain.spans[node] / 2.0)
        self.load = load

    def point_y(self) -> float:
        pose = self.adapter.outputs().bodies[self.body]
        return float(pose.position[1] + _rotate(pose.orientation, (self.offset, 0.0, 0.0))[1])

    def step(self) -> None:
        """The point force as a wrench at the segment's centre of mass: F and (point - com) x F, world axes."""
        pose = self.adapter.outputs().bodies[self.body]
        force = (0.0, self.load, 0.0)
        moment = _cross(_rotate(pose.orientation, (self.arm, 0.0, 0.0)), force)
        wrench = core.BodyWrench()
        wrench.body = self.body
        applied = core.Wrench()
        applied.moment = list(moment)
        applied.force = list(force)
        wrench.wrench = applied
        inputs = core.PhysicsInputs()
        inputs.body_wrenches = [wrench]
        self.adapter.set_inputs(inputs)
        self.adapter.step()


def omega_closed(end: str) -> float:
    beta_l_squared = 3.516015
    if end == "clamped":
        beta_l_squared = 22.37329
    return beta_l_squared * math.sqrt(EI / (MU * LENGTH**4))


def settle(end: str, segments: int, x_load: float, load: float) -> Beam:
    stiffness = EI / (LENGTH / segments)
    beam = Beam(end, segments, 1000, 2.0 / omega_closed(end) * stiffness, x_load, load)
    for _ in range(3000):
        beam.step()
    return beam


def first_frequency(
    end: str, segments: int, hz: int, x_load: float, load: float, centre: float, seconds: float
) -> float:
    """Undamped, under the held load from rest: the point oscillates about the chain's static value `centre`; the
    period is the mean interval of upward crossings after the first two."""
    beam = Beam(end, segments, hz, 0.0, x_load, load)
    h = 1.0 / hz
    times: list[float] = []
    previous = beam.point_y() - centre
    for n in range(1, round(seconds * hz) + 1):
        beam.step()
        value = beam.point_y() - centre
        if previous < 0.0 <= value:
            times.append((n - 1 + (-previous) / (value - previous)) * h)
        previous = value
    crossings = times[2:]
    assert len(crossings) >= 4
    period = (crossings[-1] - crossings[0]) / (len(crossings) - 1)
    return 2.0 * math.pi / period


@pytest.mark.req("BLD-017", "PHY-022")
def test_cantilever_tip_deflection_at_eight_segments() -> None:
    load = 3.0 * EI * 1e-4 * LENGTH / LENGTH**3
    target = load * LENGTH**3 / (3.0 * EI)
    beam = settle("free", 8, LENGTH, load)
    measured = beam.point_y()
    print(f"cantilever tip: target {target:.9e} m measured {measured:.9e} m chain {target * (1 - 1 / 256):.9e} m")
    assert measured == pytest.approx(target, rel=TOLERANCE)


@pytest.mark.req("BLD-017", "PHY-022")
def test_cantilever_first_frequency_at_eight_segments() -> None:
    load = 3.0 * EI * 1e-4 * LENGTH / LENGTH**3
    centre = settle("free", 8, LENGTH, load).point_y()
    target = omega_closed("free") / (2.0 * math.pi)
    measured = first_frequency("free", 8, 10000, LENGTH, load, centre, 0.25) / (2.0 * math.pi)
    print(f"cantilever first mode: target {target:.6f} Hz measured {measured:.6f} Hz")
    assert measured == pytest.approx(target, rel=TOLERANCE)


@pytest.mark.req("BLD-017", "PHY-022")
def test_clamped_clamped_midspan_deflection_at_sixteen_segments() -> None:
    load = 192.0 * EI * 1e-4 / LENGTH**2
    target = load * LENGTH**3 / (192.0 * EI)
    beam = settle("clamped", 16, LENGTH / 2.0, load)
    measured = beam.point_y()
    print(
        f"clamped-clamped midspan: target {target:.9e} m measured {measured:.9e} m chain {target * (1 - 4 / 256):.9e} m"
    )
    assert measured == pytest.approx(target, rel=TOLERANCE)


@pytest.mark.req("BLD-017", "PHY-022")
def test_clamped_clamped_first_frequency_at_sixteen_segments() -> None:
    load = 192.0 * EI * 1e-4 / LENGTH**2
    centre = settle("clamped", 16, LENGTH / 2.0, load).point_y()
    target = omega_closed("clamped") / (2.0 * math.pi)
    measured = first_frequency("clamped", 16, 50000, LENGTH / 2.0, load, centre, 0.04) / (2.0 * math.pi)
    print(f"clamped-clamped first mode: target {target:.6f} Hz measured {measured:.6f} Hz")
    assert measured == pytest.approx(target, rel=TOLERANCE)


@pytest.mark.req("PHY-007", "REC-004")
def test_member_end_loads_balance_the_load_in_the_member_frame() -> None:
    """Statics of the settled members (support on member, moments about each end point, member frame = world here):
    cantilever: start force -P y, start moment -P L z, free end zero; clamped-clamped under a midspan load: by the
    chain's symmetry each end carries -P / 2 y, and the moments balance, M0 + M1 + L x (end force) + (L / 2) x P = 0,
    so M0_z = -M1_z (the fixed-end moment P L / 8 is printed beside it). Residuals: exp(-75) transient and theta^2
    geometry, both below 1e-7 relative."""
    load = 3.0 * EI * 1e-4 * LENGTH / LENGTH**3
    loads = settle("free", 8, LENGTH, load).adapter.outputs().members[0]
    assert loads.start.force[1] == pytest.approx(-load, rel=1e-6)
    assert loads.start.moment[2] == pytest.approx(-load * LENGTH, rel=1e-6)
    assert list(loads.end.force) == [0.0, 0.0, 0.0]
    assert list(loads.end.moment) == [0.0, 0.0, 0.0]
    load = 192.0 * EI * 1e-4 / LENGTH**2
    loads = settle("clamped", 16, LENGTH / 2.0, load).adapter.outputs().members[0]
    print(
        f"clamped-clamped end moments: {loads.start.moment[2]:.9f} and {loads.end.moment[2]:.9f} N m, "
        f"continuum P L / 8 = {load * LENGTH / 8.0:.9f} N m"
    )
    assert loads.start.force[1] == pytest.approx(-load / 2.0, rel=1e-6)
    assert loads.end.force[1] == pytest.approx(-load / 2.0, rel=1e-6)
    assert loads.start.moment[2] + loads.end.moment[2] == pytest.approx(0.0, abs=1e-6 * load * LENGTH)
    assert abs(loads.start.moment[2]) > 0.0
