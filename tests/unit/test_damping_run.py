"""Per-run damping lines and the evaluation start's boundary step (BLD-040, BLD-041, PHY-036, PHY-037, ruling 80)."""

from __future__ import annotations

import math
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
    """At one substep a whole-step start is start / h steps (ENG-047: n = ceil(start / h - 1e-9)), so the step begun at
    the start is not before it. D2's row-time test t - step < start counted one more."""
    assert steps_before(start, Fraction(1000), 1) == expected
    assert _old_rule(start) == expected + 1
    assert steps_before(0.0, Fraction(1000), 1) == 0


@pytest.mark.req("PHY-036", "ENG-047", "ENG-052")
@pytest.mark.parametrize(
    ("start", "rate", "substeps", "expected"),
    [
        (0.01, 1000, 10, 10),
        (8.0, 1000, 10, 8000),
        (0.0105, 1000, 1, 11),
        (0.0105, 1000, 10, 10),
        (0.01005, 1000, 10, 10),
        (0.0109999, 1000, 10, 11),
        (0.0100999, 1000, 10, 10),
        (0.011, 1000, 10, 11),
        (0.00015, 10000, 1, 2),
        (0.00015, 1000, 10, 0),
        (0.0, 1000, 10, 0),
        (-0.5, 1000, 1, 0),
    ],
)
def test_steps_before_is_the_engines_step_rule_in_whole_adapter_steps(
    start: float, rate: int, substeps: int, expected: int
) -> None:
    """The engine's first judged step is n = max(0, ceil(start / h - 1e-9)) with h = 1 / (rate substeps) (ENG-047,
    ENG-052); adapter step j holds engine steps j s to j s + s - 1, so floor(n / s) adapter steps are wholly before.
    Whole-step starts: 0.01 / 1e-4 and 8 / 1e-4 divide to 100 and 80000 within 1e-14 of a step, far inside the 1e-9
    slack, so n = 100, 80000 and the counts 10, 8000 (the old summed time counted 11 and 8001). Between steps n rounds
    up: 0.0105 s is 10.5 steps at 1 ms (n = 11) and 105 at 0.1 ms (n = 105, 10 whole adapter steps); 0.01005 s is
    100.5 engine steps (n = 101: the row of adapter step 10 holds judged engine steps, so 10); 0.0100999 s is 100.999
    (n = 101 again, 10); 0.0109999 s is 109.999 (n = 110: adapter step 10's last engine step begins at 0.0109 s,
    before the start, so all 11 are held); 0.011 s is 110 (11). 0.15 ms is 1.5 steps at 0.1 ms
    (n = 2, the engine's own case) and inside the first 1 ms adapter step at ten substeps (0). A start at or below 0
    judges from the first step (0)."""
    assert steps_before(start, Fraction(rate), substeps) == expected


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
    end_ms = math.ceil(Fraction(repr(start)) * 1000) + 5
    body = body.replace("  at: 0.02 s\n  max_duration: 1 s", f"  at: {end_ms} ms\n  max_duration: 10 s")
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


@pytest.mark.req("PHY-036", "INT-031", "ENG-047")
@pytest.mark.parametrize(
    ("start", "substeps", "held_rows"),
    [
        (0.01, 1, 10),
        (8.0, 1, 8000),
        (0.01, 10, 10),
        (8.0, 10, 8000),
        (0.0105, 1, 11),
        (0.01005, 10, 10),
    ],
)
def test_the_step_begun_at_the_start_is_not_before_it(
    tmp_path: Path, start: float, substeps: int, held_rows: int
) -> None:
    """Fixture: one rope segment k = 500 kN/m from the world to a 1e6 kg bob 0.1 m past the rest length, so it starts
    at T0 = 50 kN against the 5 kN breaking tension. omega = sqrt(k / M) = 0.707 rad/s, a period of 8.9 s: the
    tension starts at its lowest (released at rest, gravity adds M g = 9.8 MN of pull towards a 19.6 m stretch) and
    stays above T0 > 5 kN for the first 8.9 s, so every step to 8.005 s exceeds. The engine holds each exceedance in
    a step begun before the start and snaps in the first step that is not: the step begun at the start, whose row is
    stamped start + 1 ms. The held line is printed because the earlier steps exceeded, and the driver's count of
    steps before the start (whose rows the held line reads) must be the start's step count, start x 1 kHz.

    At ten substeps the engine snaps in its first judged engine step n = ceil(start / 0.1 ms - 1e-9) (ENG-047), inside
    adapter step floor(n / 10), whose row is stamped one step later; the rows before it are the held rows the line
    reads, and `steps_before` counts exactly those. Between steps: 0.0105 s at one substep judges from step 11 (snap
    row 12 ms, 11 held rows); 0.01005 s at ten judges from engine step 101, inside adapter step 10 (snap row 11 ms, 10
    held rows)."""
    snaps, held = _boundary_run(tmp_path, start, substeps)
    assert len(snaps) == 1
    assert round(snaps[0] * 1000) == held_rows + 1
    assert len(held) == 1
    assert steps_before(start, Fraction(1000), substeps) == held_rows


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


def _mass(line: str, label: str) -> float:
    return float(line.split(label)[1].split(" kg")[0])


SHACKLE = 0.001
RAIL_Y_CHAIN = 2.025991  # rail_y_beam: rho A L = 26986.275 x 1.5e-4 x 0.5 = 2.023967, times 1.001 for its roller body


@pytest.mark.req("BLD-040", "BLD-041", "PHY-037")
def test_every_damped_member_and_belt_prints_stated_built_and_loaded_ratios(gantry_lines: tuple[str, ...]) -> None:
    """Rulings 80, 82: each line prints the data's ratio exactly as stated (0.01), the ratio realised at build time and
    the ratio realised with the run's bodies. The mass is what the engine carries (ENG-052's m_c): the joint tree below
    the root and the member chains rooted in it. The design load hangs on payload_rope below the hook, parented to the
    world: not in any tree, never counted (a pendulum to the member's and the belt's mode). The rope's end fitting
    'payload_shackle' (1 g) is a run-time body fixed to the hook, inside every subtree here, so each run mass is the
    build mass plus 1 g exactly (to the line's six figures).

    A belt's build-time figure: the builder anchors D = 2 zeta sqrt(k m_b), m_b the subtree's bodies without member
    chains; the line divides by sqrt(k m) with m = m_b + chains, so it prints 0.01 sqrt(m_b / m): 0.01 to rounding for
    the belts that carry no member, below it for x_belt, whose carriage carries rail_y_beam. A member's build-time
    figure is c omega_c / (2 k) with omega_c the first mode with its build-time carried mass, below omega_1 because a
    mass only lowers it (Rayleigh), so below 0.01. The shackle adds positive mass, so each run figure is lower again."""
    members = [line for line in gantry_lines if "damping ratio stated" in line and line.startswith("flexible body")]
    belts = [line for line in gantry_lines if "damping ratio stated" in line and line.startswith("transmission")]
    assert len(members) == 2
    assert len(belts) >= 3
    for line in (*members, *belts):
        assert "damping ratio stated 0.01 " in line
        assert "case payload 'payload' 5.2164 kg on 'hook' not counted" in line
    for line in members:
        built = _figure(line, "realised at build time ")
        loaded = _figure(line, "realised in this run ")
        assert loaded < built < 0.01
        assert _mass(line, "(carried mass ") + SHACKLE == pytest.approx(
            _mass(line.split("realised in this run")[1], "(carried mass "), abs=2e-5
        )
    for line in belts:
        built = _figure(line, "realised at build time ")
        loaded = _figure(line, "realised in this run ")
        mass = _mass(line, "(m = ")
        chains = _mass(line, "member chains on them ")
        assert built == pytest.approx(0.01 * math.sqrt((mass - chains) / mass), rel=1e-5)
        assert loaded < built
        assert _mass(line.split("realised in this run")[1], "(m = ") == pytest.approx(mass + SHACKLE, abs=2e-5)
    x_belt = next(line for line in belts if line.startswith("transmission 'x_belt'"))
    assert "run-time bodies 'payload_shackle'" in x_belt
    assert _mass(x_belt, "member chains on them ") == pytest.approx(RAIL_Y_CHAIN, rel=1e-5)
    rail_x = next(line for line in members if line.startswith("flexible body 'rail_x'"))
    assert _mass(rail_x, "member chains on them ") == pytest.approx(RAIL_Y_CHAIN, rel=1e-5)
    for line in belts:
        if not line.startswith("transmission 'x_belt'"):
            assert _mass(line, "member chains on them ") == 0.0
            assert _figure(line, "realised at build time ") == pytest.approx(0.01, rel=1e-12)
