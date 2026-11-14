"""Structural damping of a lumped member through the native adapter (BLD-040, DES-047, ruling 78).

Member as `test_phy_native_beams`: E I = 2000 N m^2, rho A = 0.78 kg/m, L = 1 m, eight segments (sixteen
clamped-clamped, its default, BLD-018), no gravity. Node
damping is the builder's conversion c = 2 zeta k / omega_1 (k = E I / l, omega_1 the chain's first mode for its
supports), which makes the first mode's modal damping ratio exactly zeta (`structure.node_damping`).

Release: a point load P held from rest at the tip (cantilever) or midspan (pinned-roller, clamped-clamped).
Relative to the loaded static shape the chain starts displaced by minus that shape at rest, which is the unloaded
chain released from the static shape (linearity), so the peaks of u = y - y_static decay at the first mode's
logarithmic decrement Lambda = 2 pi zeta / sqrt(1 - zeta^2). The deflection is 1e-4 L, so the geometric terms are below 1e-7 relative.

Bounds (BLD-040, T4): the plain closed form within 2 percent, and the scheme-corrected value Lambda (1 + x / (2
zeta)), x = omega h, within 0.2 percent. Budget:
- the scheme. Backward Euler on the first mode multiplies by z = 1 / (1 - h lambda) per step, lambda =
  omega (-zeta + i sqrt(1 - zeta^2)), so it adds about pi x per period, x = omega h. The step is chosen with
  x <= zeta / 100, so pi x is at most 1/200 of 2 pi zeta: a 0.5 percent raise inside the 2 percent, and the factor
  1 + x / (2 zeta) removes it to second order, (x / zeta)^2 <= 1e-4 relative. The exact scheme decrement
  2 pi ln(1 / abs(z)) / arg(z) is printed beside it.
- higher modes: zeta_n = zeta omega_n / omega_1 (cantilever omega_2 = 6.27 omega_1, pinned-roller's second mode is
  antisymmetric and unloaded, its third at 9 omega_1); the first two positive lobes are skipped, after which a
  higher mode keeps below exp(-2 pi 6.27 zeta 6.27 2) of its share, under 1e-2 of a share of a few percent at
  zeta = 0.01, so under 0.1 percent of a peak. Clamped-clamped: the second mode is antisymmetric and unloaded by the
  midspan load, the third at (10.996 / 4.730)^2 = 5.40 omega_1, below exp(-2 pi 5.40 zeta 5.40 2) of its share.
- peaks: each positive lobe's largest sample refined by a parabola; 350 or more samples per period put the error
  below 1e-6 relative.
"""

from __future__ import annotations

import math
from typing import Any

import pytest

from mechsim import _core as core
from mechsim.builder.structure import chain_layout, loaded_first_frequency, member_deflection, node_damping
from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.neutral import PhysicsModel
from tests.reference.test_phy_native_beams import EI, LENGTH, MU, beam_model
from tests.reference.test_phy_stubs import block

SEGMENTS = 8
TOLERANCE = 0.02
CORRECTED_TOLERANCE = 0.002
HZ = 10000
DEFLECTION = 1e-4 * LENGTH


def closed_decrement(zeta: float) -> float:
    return 2.0 * math.pi * zeta / math.sqrt(1.0 - zeta * zeta)


def scheme_decrement(zeta: float, omega: float, h: float) -> float:
    """Per-period log decrement of backward Euler on the damped first mode: z = 1 / (1 - h lambda)."""
    pole = complex(-zeta * omega, omega * math.sqrt(1.0 - zeta * zeta))
    multiplier = 1.0 / (1.0 - h * pole)
    return 2.0 * math.pi * -math.log(abs(multiplier)) / math.atan2(multiplier.imag, multiplier.real)


def corrected_decrement(zeta: float, x: float) -> float:
    """The closed form raised by backward Euler's pi x per period, to first order in x / zeta."""
    return closed_decrement(zeta) * (1.0 + x / (2.0 * zeta))


def substeps_for(omega: float, zeta: float) -> int:
    """Engine steps per adapter step so that x = omega h <= zeta / 100."""
    return math.ceil(omega / (HZ * zeta / 100.0))


def lobe_peaks(samples: list[float]) -> list[float]:
    """The largest value of each positive lobe of u, refined by the parabola through its sample and neighbours."""
    peaks: list[float] = []
    best = -1
    for index in range(1, len(samples) - 1):
        if samples[index] > 0.0 and (best < 0 or samples[index] > samples[best]):
            best = index
        if samples[index] > 0.0 and samples[index + 1] <= 0.0 and best > 0:
            low, mid, high = samples[best - 1], samples[best], samples[best + 1]
            curvature = high - 2.0 * mid + low
            peaks.append(mid - (high - low) ** 2 / (8.0 * curvature))
            best = -1
    return peaks


def measured_decrement(peaks: list[float], periods: int) -> float:
    assert len(peaks) >= 2 + periods + 1
    return math.log(peaks[2] / peaks[2 + periods]) / periods


class Point:
    """The load point (the tip, else midspan) on node `node`: `offset` from the node's frame origin along the member,
    `arm` from the node's centre of mass, `load` the force giving the static deflection DEFLECTION there."""

    def __init__(self, root: str, end: str, segments: int = SEGMENTS) -> None:
        layout = end
        if end == "clamped":
            layout = "roller"
        chain = chain_layout(root, layout, segments, LENGTH)
        x = LENGTH
        if end != "free":
            x = 0.5 * LENGTH
        self.node = max(k for k, start in enumerate(chain.starts) if start <= x + 1e-12)
        self.offset = x - chain.starts[self.node]
        self.arm = x - (chain.starts[self.node] + chain.spans[self.node] / 2.0)
        self.load = DEFLECTION / member_deflection(root, end, segments, LENGTH, EI, 1.0, x)


def run(model: PhysicsModel, body: int, point: Point, omega: float, zeta: float, periods: int) -> tuple[float, float]:
    """Hold the load along y at the point on output body `body` (a force at the node's centre of mass with the
    moment of its arm, small angles) from rest; return (measured decrement of u = y - DEFLECTION, engine step)."""
    substeps = substeps_for(omega, zeta)
    prepared = prepare_model(model, None, 1.0 / HZ, substeps=substeps)
    adapter = core.make_physics_adapter("native")
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(HZ))
    samples: list[float] = []
    steps = math.ceil((periods + 4) * 2.0 * math.pi / omega * HZ)
    for _ in range(steps):
        wrench = core.BodyWrench()
        wrench.body = body
        applied = core.Wrench()
        applied.force = [0.0, point.load, 0.0]
        applied.moment = [0.0, 0.0, point.arm * point.load]
        wrench.wrench = applied
        inputs = core.PhysicsInputs()
        inputs.body_wrenches = [wrench]
        adapter.set_inputs(inputs)
        adapter.step()
        pose = adapter.outputs().bodies[body]
        samples.append(float(pose.position[1]) + point.offset * math.sin(_angle(pose.orientation)) - DEFLECTION)
    return measured_decrement(lobe_peaks(samples), periods), 1.0 / (HZ * substeps)


def _angle(q: Any) -> float:
    w, _x, _y, z = q
    return 2.0 * math.atan2(z, w)


@pytest.mark.req("BLD-040", "DES-047")
@pytest.mark.parametrize(
    ("root", "end", "segments"), [("clamped", "free", 8), ("pinned", "roller", 8), ("clamped", "clamped", 16)]
)
@pytest.mark.parametrize(("zeta", "periods"), [(0.01, 4), (0.05, 3)])
def test_first_mode_decrement_is_the_closed_form(root: str, end: str, segments: int, zeta: float, periods: int) -> None:
    omega = loaded_first_frequency(root, end, segments, LENGTH, EI, MU)
    damping = node_damping(zeta, omega, EI / (LENGTH / segments))
    point = Point(root, end, segments)
    measured, h = run(beam_model(end, segments, damping, root), 1 + point.node, point, omega, zeta, periods)
    target = closed_decrement(zeta)
    corrected = corrected_decrement(zeta, omega * h)
    print(
        f"{root}-{end} N {segments} zeta {zeta}: closed form {target:.6g}, measured {measured:.6g} "
        f"({measured / target - 1:+.3%}), scheme-corrected {corrected:.6g} ({measured / corrected - 1:+.3%}), exact "
        f"scheme {scheme_decrement(zeta, omega, h):.6g}, x = {omega * h:.3g}"
    )
    assert abs(measured - target) <= TOLERANCE * target
    assert abs(measured - corrected) <= CORRECTED_TOLERANCE * corrected


CARRIED_MASS = 0.2


def _carried_model(damping: float, at: float) -> PhysicsModel:
    data = beam_model("free", SEGMENTS, damping).model_dump(mode="json")
    data["bodies"].append(
        block(
            "carriage", CARRIED_MASS, {"name": "carriage_mount", "type": "fixed", "position": [f"{at} m", "0 m", "0 m"]}
        )
    )
    data["bodies"][-1]["parent"] = "base"
    data["lumped_members"][0]["carries"] = ["carriage"]
    return PhysicsModel.model_validate(data)


@pytest.mark.req("BLD-040")
def test_a_carried_mass_lowers_the_realised_ratio_by_the_loaded_frequency() -> None:
    """Cantilever carrying 0.2 kg at the tip (about the beam's own modal mass 0.25 rho A L): C = beta K is unchanged
    and M gains m a a^T, so the loaded first mode omega_c carries zeta_c = zeta omega_c / omega_1; its decrement is
    2 pi zeta_c / sqrt(1 - zeta_c^2), within the same 2 percent (0.2 percent scheme-corrected) with x = omega_c h <=
    zeta_c / 100."""
    zeta = 0.05
    omega = loaded_first_frequency("clamped", "free", SEGMENTS, LENGTH, EI, MU)
    loaded = loaded_first_frequency("clamped", "free", SEGMENTS, LENGTH, EI, MU, CARRIED_MASS, LENGTH)
    realised = zeta * loaded / omega
    damping = node_damping(zeta, omega, EI / (LENGTH / SEGMENTS))
    point = Point("clamped", "free")
    measured, h = run(_carried_model(damping, LENGTH), 2 + point.node, point, loaded, realised, 3)
    target = closed_decrement(realised)
    corrected = corrected_decrement(realised, loaded * h)
    print(
        f"carried {CARRIED_MASS} kg at the tip: omega_1 {omega:.6g}, omega_c {loaded:.6g} rad/s, "
        f"zeta_c {realised:.6g}; "
        f"closed form {target:.6g}, measured {measured:.6g} ({measured / target - 1:+.3%}), scheme-corrected "
        f"{corrected:.6g}"
    )
    assert loaded < omega
    assert abs(measured - target) <= TOLERANCE * target
    assert abs(measured - corrected) <= CORRECTED_TOLERANCE * corrected
