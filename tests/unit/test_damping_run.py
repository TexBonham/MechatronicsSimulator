"""Per-run damping lines and the evaluation start's boundary step (BLD-040, BLD-041, PHY-036, PHY-037, ruling 80)."""

from __future__ import annotations

import math
from fractions import Fraction
from pathlib import Path

import pytest

from mechsim.physics.native.translate import steps_before
from mechsim.testing.core_driver import CoreDriver, core_services
from mechsim.testing.runner import Services, run_case, yaml_loader

ROOT = Path(__file__).resolve().parents[2]


def _old_rule(start: float) -> int:
    """D2's float test on the row's end time at 1 kHz: a row counted when t - step < start."""
    return sum(1 for tick in range(1, int(start * 1000) + 3) if float(Fraction(tick) / 1000) - 0.001 < start)


@pytest.mark.req("PHY-036")
@pytest.mark.parametrize(
    ("start", "substeps", "expected"), [(0.01, 1, 10), (8.0, 1, 8000), (0.01, 10, 11), (8.0, 10, 8001)]
)
def test_the_boundary_step_follows_the_engines_own_time(start: float, substeps: int, expected: int) -> None:
    """The engine gates on its accumulated time t + h (h = 1 / (1000 substeps)) against the start. At one substep ten
    sums of 0.001 give 0.010000000000000002 and 8000 give 8.000000000001005, so the step begun at the start is not
    before it; at ten substeps the sums fall just short (0.010999999999999989 after eleven steps' worth, 8.000999...
    after 8001), so the engine itself counts the step begun at 0.01 s (and at 8 s) as before. The driver now counts
    exactly the engine's steps; D2's row-time test counted one more at one substep."""
    time = 0.0
    count = 0
    while time < start:
        count += 1
        for _ in range(substeps):
            time = time + 1.0 / (1000.0 * substeps)
    assert count == expected
    assert steps_before(start, Fraction(1000), substeps) == expected
    assert _old_rule(start) == math.floor(start * 1000) + 1
    assert steps_before(0.0, Fraction(1000), substeps) == 0


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
