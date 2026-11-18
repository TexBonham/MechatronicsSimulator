"""Rope table reference (INT-022, ADR 0016): a mass hanging still on one rope carries tension m g."""

from __future__ import annotations

from fractions import Fraction
from types import SimpleNamespace

import pytest

from mechsim.integrity.config import IntegrityConfig
from mechsim.integrity.inputs import run_inputs
from mechsim.integrity.table import evaluate
from mechsim.record.layout import ROPE_SERIES, Columns
from mechsim.record.stream import Recorder
from mechsim.record.tick import PhysicsOutputs, RopeTension, TickRecord
from tests.unit.test_integrity import _design, _View

# Static equilibrium of the hanging mass: T = m g exactly; the table reads the recorded largest segment tension, so the
# proposed tolerance is 1e-9 relative (no arithmetic beyond a maximum).
EXACT = 1e-9
MASS = 2.5
GRAVITY = 9.81


def hanging(tensions: list[float], breaking: float | None = None, allowance: str | None = "0") -> SimpleNamespace:
    recorder = Recorder(Fraction(1000), {path: Columns(("hoist",)) for path in ROPE_SERIES})
    for tick, tension in enumerate(tensions):
        rope = RopeTension(tension, False, tension)
        recorder.write_row(TickRecord(time=tick, physics=PhysicsOutputs(steps=tick + 1, ropes=(rope,))))
    view = _View(recorder.recording())
    physics = SimpleNamespace(
        bodies=(), fastenings=(), ropes=(SimpleNamespace(name="hoist", breaking_tension=breaking),)
    )
    build = SimpleNamespace(design=_design(allowance), members=(), physics=physics)
    return SimpleNamespace(build=build, view=view)


@pytest.mark.req("INT-022")
def test_a_hanging_mass_rope_peaks_at_m_g() -> None:
    target = MASS * GRAVITY
    run = hanging([target] * 4, allowance=None)
    table = evaluate(IntegrityConfig(schema_version=1), run_inputs(run.build, run.view, {}))
    (rope,) = table.ropes
    print(f"hanging rope tension: closed form {target!r}, measured {rope.peak_tension!r}")
    assert rope.rope == "hoist"
    assert rope.peak_tension == pytest.approx(target, rel=EXACT)
    assert rope.peak_tension_time == 0.0
    assert rope.flags == ()
