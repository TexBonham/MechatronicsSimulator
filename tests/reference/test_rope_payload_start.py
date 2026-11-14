"""A case payload on a design rope starts statically consistent (BLD-032, ENG-047, brief M3-B3): no gantry, a body
on a rope from a fixed hook, built through the driver's path (`with_payload`, then `prepare_model` and the native
adapter).

Why the start is the static stretch and not the unstretched rope: a hanging payload at rest is a statically
consistent start, and an unstretched release is not a load case: under ENG-047's taut-set rule (brief M3-E8) it gives
the scheme's step response, a peak of at most 2 M g; under the retired start-of-step rule it read 3750 N for 1 kg on
this rope (the defect brief M3-B3 traced, ruling 87). At the static stretch every segment starts taut and the release
is the static load from the first step.

Closed forms. Segment k of N (from the top, l = L / N) carries T_k = (m + mu l / 2 + (N - k) mu l) g: the load, the
load end's half-segment share and the N - k nodes below it. The stretches T_k l / EA sum to the payload's drop
beyond the rest length, (m + mu L / 2) g L / EA (`static_stretch`). The load end settles at (m + mu l / 2) g; the
fitting's fixed joint carries the fitting, the whole rope and the load, (m_f + mu L + m) g.

Bound on the start-up. The engine places the nodes evenly on the line between the ends, so every segment starts at
the mean stretch: its tension is the mean (m + mu L / 2) g, off its static T_k by dT_k = mu l g (k - (N + 1) / 2).
That offset is the only departure from statics (the payload starts at rest, the hook is fixed), and its strain
energy is E0 = sum dT_k^2 / (2 k_s) = (mu l g)^2 N (N^2 - 1) / 12 / (2 k_s), k_s = N EA / L. The scheme does not add
energy (stiffness and damping implicit: backward Euler on the K and D modes, ruling 21), so afterwards a segment's
spring force departs from static by at most sqrt(2 k_s E0) = mu l g sqrt(N (N^2 - 1) / 12), and any body's speed is
at most sqrt(2 E0 / m_min) with m_min = mu l (the lightest, a node), so a segment's strain rate is at most twice that
and its damping force c_s times it, c_s = N D / L. The reported tension is spring plus damper, so it stays within
EXCESS = mu l g sqrt(N (N^2 - 1) / 12) + 2 c_s sqrt(2 E0 / (mu l)) of its static value, at every substep count
(1.77 N here, nearly all the damper term). The excess is below every T_k (the smallest, the load end's, is 9.8 N), so
no segment goes slack and the linear argument holds throughout.

Settling. The load's axial mode (stiffness EA / L, damping D / L) decays at (D / L) / (2 m): 16.7 1/s for 5 kg, the
slower case. The start's relative departure is at most EXCESS / ((m + mu l / 2) g) = 0.18 (1 kg); under 1e-9 of the
tension needs ln(0.18 / 1e-9) / 16.7 = 1.14 s, so the settled reading is taken at 1.5 s (backward Euler only damps
further).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import mechsim._core_ext as core
import pytest

from mechsim.builder.payload import PAYLOAD_EDGE, static_stretch, with_payload
from mechsim.physics.native.prepare import prepare_model
from mechsim.physics.neutral import PhysicsModel, Rope, from_yaml

ROOT = Path(__file__).resolve().parents[2]
GRAVITY = 9.81
SEGMENTS = 8
EA = 5e5
DAMPING = 50.0
MU = 0.035
LENGTH = 0.3
FITTING = 0.001
HOOK_Z = 1.0
HOOK_POINT = -0.02
EXACT = 1e-9
STEPS_FAST = 100
STEPS_SETTLE = 1500


def _rope() -> Rope:
    return Rope.model_validate(
        {
            "name": "line",
            "from_body": "hook",
            "from_point": ["0 m", "0 m", f"{HOOK_POINT!r} m"],
            "to_body": "payload",
            "to_point": ["0 m", "0 m", "0 m"],
            "segments": SEGMENTS,
            "axial_stiffness": f"{EA!r} N",
            "damping": f"{DAMPING!r} N*s",
            "mass_per_length": f"{MU!r} kg/m",
            "rest_length": f"{LENGTH!r} m",
        }
    )


def hung_model(mass: float) -> PhysicsModel:
    data: dict[str, Any] = from_yaml((ROOT / "schemas/examples/physics_model.pendulum.yaml").read_text()).model_dump(
        mode="json"
    )
    hook = dict(data["bodies"][0])
    hook["name"] = "hook"
    hook["com"] = ["0 m", "0 m", "0 m"]
    hook["joint"] = {"name": "hook_mount", "type": "fixed", "position": ["0 m", "0 m", f"{HOOK_Z!r} m"]}
    data["bodies"] = [hook]
    data["gravity"] = ["0 m/s**2", "0 m/s**2", f"{-GRAVITY!r} m/s**2"]
    data.pop("signals", None)
    model = PhysicsModel.model_validate(data)
    return with_payload(model, "payload", "hook", mass, True, _rope(), {}, FITTING)


def excess() -> float:
    segment = LENGTH / SEGMENTS
    node = MU * segment
    spread = math.sqrt(SEGMENTS * (SEGMENTS**2 - 1) / 12.0)
    k_s = SEGMENTS * EA / LENGTH
    c_s = SEGMENTS * DAMPING / LENGTH
    energy = (node * GRAVITY * spread) ** 2 / (2.0 * k_s)
    return node * GRAVITY * spread + 2.0 * c_s * math.sqrt(2.0 * energy / node)


def run(mass: float, substeps: int, steps: int) -> list[tuple[float, float, float]]:
    model = hung_model(mass)
    prepared = prepare_model(model, {}, 1e-3, {}, substeps)
    adapter = core.make_physics_adapter("native")
    adapter.load(core.PreparedModel(prepared.model_json, "", prepared.version), core.Rate(1000))
    rows: list[tuple[float, float, float]] = []
    for _ in range(steps):
        adapter.step()
        out = adapter.outputs()
        rope = out.ropes[0]
        rows.append((rope.max_tension, rope.tension, out.joints[1].reaction.force[2]))
    return rows


@pytest.mark.req("BLD-032")
def test_the_payload_hangs_at_the_ropes_static_stretch_below_the_hook() -> None:
    for mass in (1.0, 5.0):
        model = hung_model(mass)
        payload = model.bodies[-1]
        assert payload.joint is not None
        top = payload.joint.position[2] + PAYLOAD_EDGE / 2.0
        drop = HOOK_Z + HOOK_POINT - top
        stretch = (mass + MU * LENGTH / 2.0) * GRAVITY * LENGTH / EA
        print(f"{mass} kg: drop {drop!r}, closed form {LENGTH + stretch!r}")
        assert static_stretch(_rope(), mass, GRAVITY) == pytest.approx(stretch, rel=1e-15)
        # A handful of additions of order-one lengths: a few ulps of 1 m.
        assert drop == pytest.approx(LENGTH + stretch, abs=1e-14)
        assert payload.joint.position[:2] == (0.0, 0.0)


@pytest.mark.req("BLD-032")
@pytest.mark.req("ENG-047")
@pytest.mark.parametrize("substeps", [1, 10])
@pytest.mark.parametrize("mass", [1.0, 5.0])
def test_a_hanging_payload_starts_at_its_static_load(mass: float, substeps: int) -> None:
    segment = LENGTH / SEGMENTS
    top = (mass + MU * segment / 2.0 + (SEGMENTS - 1) * MU * segment) * GRAVITY
    load_end = (mass + MU * segment / 2.0) * GRAVITY
    held = (FITTING + MU * LENGTH + mass) * GRAVITY
    bound = excess()
    rows = run(mass, substeps, STEPS_FAST)
    peak = max(row[0] for row in rows)
    reactions = [row[2] for row in rows]
    print(
        f"{mass} kg, {substeps} substeps: peak tension {peak!r} against static top {top!r} + {bound!r}; "
        f"hook reaction {min(reactions)!r} to {max(reactions)!r} against static {held!r}"
    )
    assert bound < load_end
    assert peak <= top + bound
    assert max(reactions) <= held + bound
    assert min(reactions) >= held - bound
    assert min(row[1] for row in rows) >= load_end - bound


@pytest.mark.req("BLD-032")
@pytest.mark.req("ENG-047")
@pytest.mark.parametrize("substeps", [1, 10])
@pytest.mark.parametrize("mass", [1.0, 5.0])
def test_a_hanging_payload_settles_at_the_closed_form_and_runs_twice_identical(mass: float, substeps: int) -> None:
    load_end = (mass + MU * LENGTH / SEGMENTS / 2.0) * GRAVITY
    held = (FITTING + MU * LENGTH + mass) * GRAVITY
    first = run(mass, substeps, STEPS_SETTLE)
    settled = first[-1]
    print(f"{mass} kg, {substeps} substeps: settled load end {settled[1]!r} target {load_end!r}")
    assert settled[1] == pytest.approx(load_end, rel=EXACT)
    assert settled[2] == pytest.approx(held, rel=EXACT)
    assert run(mass, substeps, STEPS_SETTLE) == first
