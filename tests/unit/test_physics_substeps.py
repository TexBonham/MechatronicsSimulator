"""Physics substeps (TST-042, rulings 21, 49): `physics.substeps` sets the native adapter's engine steps per physics
step, leaving the physics and control rates unchanged; above one on the bridge the run fails."""

from __future__ import annotations

from fractions import Fraction
from pathlib import Path

import pytest

from mechsim.testing.core_driver import CoreDriver, DriverError, core_builder
from mechsim.testing.driver import EndSpec, RunConfig, RunReport
from mechsim.testing.runner import RunnerError, prepare, run_config
from mechsim.testing.schema import DESIGNS_ROOT, ROOT

P1 = ROOT / "test_cases" / "pendulum" / "P1_period.yaml"
TICKS = 5


class ShortDriver(CoreDriver):
    def run(self, end: EndSpec) -> RunReport:
        return super().run(EndSpec(max_tick=TICKS, at_tick=TICKS))


def _case_with(tmp_path: Path, physics: str) -> Path:
    text = P1.read_text(encoding="utf-8")
    path = tmp_path / "P1_period.yaml"
    path.write_text(f"{text.rstrip()}\n{physics}\n", encoding="utf-8")
    return path


def _short_run(adapter: str, substeps: int) -> RunReport:
    prepared = prepare(P1)
    driver = ShortDriver()
    driver.load(core_builder(prepared.design, adapter), prepared.case)
    driver.configure(
        RunConfig(Fraction(1000), None, Fraction(1000), 0, (), None, physics_adapter=adapter, physics_substeps=substeps)
    )
    driver.set_initial_state(prepared.case.initial_state)
    return driver.run(EndSpec(max_tick=TICKS, at_tick=TICKS))


@pytest.mark.req("TST-042")
def test_the_default_is_one_substep_and_the_rates_ignore_it(tmp_path: Path) -> None:
    prepared = prepare(P1)
    assert prepared.case.physics.substeps == 1
    build = core_builder(prepared.design)
    assert run_config(prepared.case, prepared.design, build).physics_substeps == 1
    written = _case_with(tmp_path, "physics:\n  adapter: native\n  substeps: 10")
    loaded = prepare(written, designs_root=DESIGNS_ROOT)
    config = run_config(loaded.case, loaded.design, build)
    assert config.physics_substeps == 10
    assert config.physics_rate == run_config(prepared.case, prepared.design, build).physics_rate
    assert config.base_rate == config.physics_rate


@pytest.mark.req("TST-042")
@pytest.mark.parametrize("value", ["0", "-1", "1.5", "'2'", "true"])
def test_a_substep_count_below_one_or_not_an_integer_is_a_validation_error_naming_the_field(
    tmp_path: Path, value: str
) -> None:
    written = _case_with(tmp_path, f"physics:\n  adapter: native\n  substeps: {value}")
    with pytest.raises(RunnerError) as raised:
        prepare(written, designs_root=DESIGNS_ROOT)
    assert "physics.substeps" in str(raised.value)


@pytest.mark.req("TST-042")
def test_native_lines_state_the_substep_and_the_bridge_refuses_substeps() -> None:
    lines = _short_run("native", 4).approximations
    assert any("at 0.00025 s" in line for line in lines), lines
    assert not any("at 0.001 s" in line for line in lines)
    assert any("at 0.001 s" in line for line in _short_run("native", 1).approximations)
    with pytest.raises(DriverError) as raised:
        _short_run("mujoco", 2)
    assert str(raised.value) == "physics.substeps 2: only the native adapter substeps (TST-042)"
    assert _short_run("mujoco", 1).reason == "end_time"
