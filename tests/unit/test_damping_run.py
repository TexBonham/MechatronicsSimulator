"""Per-run damping lines and the evaluation start's boundary step (BLD-040, BLD-041, PHY-036, PHY-037, ruling 80)."""

from __future__ import annotations

from fractions import Fraction
from pathlib import Path

import pytest

from mechsim.design.schema import yaml_loader
from mechsim.integrity.tier1 import Tier1Service
from mechsim.physics.native.translate import steps_before
from mechsim.testing.core_driver import CoreDriver, core_builder, core_services
from mechsim.testing.runner import Services, run_case
from tests.unit.test_integrity_tier1 import SNAP_CASE, SPRING, config

ROOT = Path(__file__).resolve().parents[2]


def _old_rule(start: float) -> int:
    """D2's float test on the row's end time at 1 kHz: a row counted when t - step < start."""
    return sum(1 for tick in range(1, int(start * 1000) + 3) if float(Fraction(tick) / 1000) - 0.001 < start)


@pytest.mark.req("PHY-036")
@pytest.mark.parametrize(("start", "expected"), [(0.01, 10), (8.0, 8000)])
def test_the_driver_counts_the_engines_steps_before_the_start(start: float, expected: int) -> None:
    """At one substep the engine's accumulated time after ten sums of 0.001 is 0.010000000000000002 and after 8000 is
    8.000000000001005, so the step begun at the start is not before it; `steps_before` repeats that sum. D2's row-time
    test t - step < start counted one more."""
    assert steps_before(start, Fraction(1000), 1) == expected
    assert _old_rule(start) == expected + 1
    assert steps_before(0.0, Fraction(1000), 1) == 0


HEAVY_BOB = 1.0e6


def _boundary_run(tmp_path: Path, start: float, substeps: int) -> tuple[list[float], list[str]]:
    """The rope_spring bob made 1e6 kg and placed 0.1 m below the rope's rest length, rope snap on, evaluation start
    `start`, 1 kHz with `substeps` engine steps per step, run to start + 5 ms; returns the snap events' times and the
    held-rope lines."""
    root = tmp_path / "designs"
    (root / "rope_spring").mkdir(parents=True)
    text = SPRING.read_text(encoding="utf-8").replace('default: "-1 m"', 'default: "-1.1 m"')
    text = text.replace("      mass: 1 kg", f"      mass: {HEAVY_BOB} kg")
    (root / "rope_spring" / "design.yaml").write_text(text, encoding="utf-8")
    case = tmp_path / "S9.yaml"
    body = SNAP_CASE.replace("physics:\n", f"integrity:\n  evaluate_from: {start} s\nphysics:\n")
    body = body.replace("  adapter: native\n", f"  adapter: native\n  substeps: {substeps}\n")
    body = body.replace("  at: 0.02 s\n  max_duration: 1 s", f"  at: {start + 0.005} s\n  max_duration: 10 s")
    case.write_text(body, encoding="utf-8")
    drivers: list[CoreDriver] = []

    def factory() -> CoreDriver:
        driver = CoreDriver(designs_root=root, tier1=Tier1Service(config=config({"check": "rope_snap"})))
        drivers.append(driver)
        return driver

    result = run_case(case, Services(factory, core_builder, None, designs_root=root), None)
    assert result.report is not None, result.score.error
    assert not result.report.errored, result.report.error
    snaps = [event.time for event in drivers[0].recording().events() if event.detail.startswith("rope_snapped")]
    held = [line for line in result.report.approximations if "before the evaluation start" in line]
    return snaps, held


BOUNDARY_FOLLOW_UP = "engine evaluation start by step count, brief E7g follow-up"


@pytest.mark.req("PHY-036", "INT-031")
@pytest.mark.parametrize(
    ("start", "substeps"),
    [
        (0.01, 1),
        (8.0, 1),
        pytest.param(0.01, 10, marks=pytest.mark.xfail(strict=True, reason=BOUNDARY_FOLLOW_UP)),
        pytest.param(8.0, 10, marks=pytest.mark.xfail(strict=True, reason=BOUNDARY_FOLLOW_UP)),
    ],
)
def test_the_step_begun_at_the_start_is_not_before_it(tmp_path: Path, start: float, substeps: int) -> None:
    """Fixture: one rope segment k = 500 kN/m from the world to a 1e6 kg bob 0.1 m past the rest length, so it starts
    at T0 = 50 kN against the 5 kN breaking tension. omega = sqrt(k / M) = 0.707 rad/s, a period of 8.9 s: the
    tension starts at its lowest (released at rest, gravity adds M g = 9.8 MN of pull towards a 19.6 m stretch) and
    stays above T0 > 5 kN for the first 8.9 s, so every step to 8.005 s exceeds. The engine holds each exceedance in
    a step begun before the start and snaps in the first step that is not: the step begun at the start, whose row is
    stamped start + 1 ms. The held line is printed because the earlier steps exceeded, and the driver's count of
    steps before the start (whose rows the held line reads) must be the start's step count, start x 1 kHz.

    At ten substeps the engine already snaps in the step begun at the start (its gate tests each engine step, and the
    second engine step of that step begins at 0.0101 s), but the driver's `steps_before` repeats the engine's summed
    time per adapter step and counts that step as before (11 at 0.01 s, 8001 at 8 s), so its held line would read a
    row in which the rope snapped: the cases fail until the driver and the engine count steps the same way."""
    snaps, held = _boundary_run(tmp_path, start, substeps)
    assert len(snaps) == 1
    assert round(snaps[0] * 1000) == round(start * 1000) + 1
    assert len(held) == 1
    assert steps_before(start, Fraction(1000), substeps) == round(start * 1000)


@pytest.fixture(scope="module")
def gantry_lines(tmp_path_factory: pytest.TempPathFactory) -> tuple[str, ...]:
    """G15 cut to 5 ms (native, the design load on payload_rope): the run's lines, through `run_case`."""
    loader = yaml_loader()
    data = loader.load((ROOT / "test_cases" / "gantry" / "G15.yaml").read_text(encoding="utf-8"))
    data["end"] = {"at": "0.005 s", "max_duration": "0.005 s"}
    data["commands"] = []
    data["faults"] = []
    data.pop("integrity", None)
    directory = tmp_path_factory.mktemp("g15_short")
    case = directory / "G15.yaml"
    with case.open("w", encoding="utf-8") as stream:
        loader.dump(data, stream)
    services = core_services()
    result = run_case(case, Services(CoreDriver, services.builder, None, services.designs_root), None)
    assert result.report is not None, result.score.error
    assert not result.report.errored, result.report.error
    return tuple(result.report.approximations)


def _figure(line: str, label: str) -> float:
    return float(line.split(label)[1].split(" ")[0].rstrip(";,"))


@pytest.mark.req("BLD-040", "BLD-041", "PHY-037")
def test_every_damped_member_and_belt_prints_stated_built_and_loaded_ratios(gantry_lines: tuple[str, ...]) -> None:
    """Ruling 80: each line prints the data's ratio exactly as stated (0.01), the ratio realised at build time and the
    ratio realised with the run's bodies and payload. The design load hangs on payload_rope below the hook, parented to
    the world; the rope's end fitting 'payload_shackle' (the rope part's end_fitting_mass) is a run-time body fixed to
    the hook, inside the hoist and trolley belts' driven subtrees.

    A belt's build-time figure is its anchor: D = 2 zeta sqrt(k m) with m the build-time subtree mass, so D / (2 sqrt(k
    m)) returns zeta to rounding (1e-12 relative). A member's build-time figure is c omega_c / (2 k) with omega_c the
    first mode with its build-time carried mass, below omega_1 because a mass only lowers it (Rayleigh), so below 0.01.
    The run adds positive mass to each belt's and member's carried subtree, so each run figure is lower again."""
    members = [line for line in gantry_lines if "damping ratio stated" in line and line.startswith("flexible body")]
    belts = [line for line in gantry_lines if "damping ratio stated" in line and line.startswith("transmission")]
    assert len(members) == 2
    assert len(belts) >= 3
    for line in (*members, *belts):
        assert "damping ratio stated 0.01 " in line
        assert "case payload 'payload' 5.2164 kg on 'hook'" in line
    for line in members:
        built = _figure(line, "realised at build time ")
        loaded = _figure(line, "realised in this run ")
        assert loaded < built < 0.01
    for line in belts:
        built = _figure(line, "realised at build time ")
        loaded = _figure(line, "realised in this run ")
        assert built == pytest.approx(0.01, rel=1e-12)
        assert loaded < built
    x_belt = next(line for line in belts if line.startswith("transmission 'x_belt'"))
    assert "run-time bodies 'payload_shackle'" in x_belt
