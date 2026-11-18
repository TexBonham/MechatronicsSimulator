"""Physics adapter selection (TST-040, ruling 41): `physics.adapter` is a run-configuration field of the case,
overridden by `--set physics.adapter=<name>`, part of the run identity and of the archive; no environment variable."""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
from pathlib import Path

import h5py
import pytest

from mechsim.record.archive import RunArchive
from mechsim.testing import core_driver
from mechsim.testing.core_driver import CoreDriver, DriverError, RecordArchiver, RecordArchiveReader, core_builder
from mechsim.testing.driver import EndSpec, RunConfig, RunReport
from mechsim.testing.runner import RunnerError, Services, prepare, run_case, run_id, variation_run_id
from mechsim.testing.schema import DESIGNS_ROOT, ROOT, Case, load_test_case

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


def _config(adapter: str) -> RunConfig:
    return RunConfig(Fraction(1000), None, Fraction(1000), 0, (), None, physics_adapter=adapter)


@pytest.mark.req("TST-040")
def test_the_default_adapter_is_native() -> None:
    prepared = prepare(P1)
    assert prepared.case.physics.adapter == "native"
    assert CoreDriver().adapter == "native"


@pytest.mark.req("TST-040")
def test_the_one_value_from_the_case_and_from_set(tmp_path: Path) -> None:
    adapter = "native"
    written = _case_with(tmp_path, f"physics:\n  adapter: {adapter}")
    assert prepare(written, designs_root=DESIGNS_ROOT).case.physics.adapter == adapter
    assert prepare(P1, {"physics.adapter": adapter}).case.physics.adapter == adapter
    driver = CoreDriver()
    driver.load(core_builder(prepare(P1).design), prepare(P1).case)
    driver.configure(_config(adapter))
    assert driver.adapter == adapter


@pytest.mark.req("TST-040")
def test_an_unknown_adapter_is_a_validation_error_naming_the_field(tmp_path: Path) -> None:
    with pytest.raises(RunnerError, match=r"physics\.adapter"):
        prepare(P1, {"physics.adapter": "bullet"})
    with pytest.raises(RunnerError, match=r"physics\.adapter"):
        prepare(_case_with(tmp_path, "physics:\n  adapter: bullet"))
    driver = CoreDriver()
    driver.load(core_builder(prepare(P1).design), prepare(P1).case)
    with pytest.raises(DriverError, match=r"physics\.adapter 'bullet'"):
        driver.configure(_config("bullet"))


@pytest.mark.req("TST-040", "PHY-026")
def test_the_retired_bridge_is_refused_by_name_from_the_case_from_set_and_at_the_driver(tmp_path: Path) -> None:
    """Invariant 7: a document or override naming `mujoco` is an error naming the retired adapter, never a silent
    run on another adapter."""
    retired = r"physics\.adapter 'mujoco': the MuJoCo bridge was retired \(ADR 0004, milestone 3\); use 'native'"
    with pytest.raises(RunnerError, match=retired):
        prepare(P1, {"physics.adapter": "mujoco"})
    with pytest.raises(RunnerError, match=retired):
        prepare(_case_with(tmp_path, "physics:\n  adapter: mujoco"))
    driver = CoreDriver()
    driver.load(core_builder(prepare(P1).design), prepare(P1).case)
    with pytest.raises(DriverError, match=retired):
        driver.configure(_config("mujoco"))


@pytest.mark.req("TST-040")
def test_the_environment_does_not_select_the_adapter(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("MECHSIM_PHYSICS_ADAPTER", "mujoco")
    assert prepare(P1).case.physics.adapter == "native"
    assert not hasattr(core_driver, "ADAPTER_VARIABLE")
    assert not hasattr(core_driver, "physics_adapter_name")


@pytest.mark.req("TST-040")
def test_the_run_id_carries_a_non_default_adapter_only() -> None:
    """The identity rule (TST-040): a run on the default adapter keeps the id it had before the field existed; a
    run on any other adapter appends `-<adapter>`, so the two runs of one case never share an archive name."""
    case: Case = load_test_case(P1, DESIGNS_ROOT)
    plain = run_id(case, None)
    assert plain == f"{case.id}-seed{case.seed}"
    assert run_id(case, None, None, "native") == plain
    assert run_id(case, None, None, "chrono") == f"{plain}-chrono"
    assert run_id(case, (0, 1), "2", "chrono") == f"{case.id}-seed{case.seed}-sweep0-1-2-chrono"
    assert variation_run_id(case.id, 9) == f"{case.id}-variation-seed9"
    assert variation_run_id(case.id, 9, "chrono") == f"{case.id}-variation-seed9-chrono"


@pytest.mark.req("TST-040", "REC-013")
def test_the_adapter_round_trips_through_the_archive(tmp_path: Path) -> None:
    adapter = "native"
    services = Services(ShortDriver, core_builder, RecordArchiver(DESIGNS_ROOT))
    result = run_case(P1, services, {"physics.adapter": adapter}, out_dir=tmp_path)
    assert result.report is not None
    assert not result.report.errored, result.report.error
    assert result.config is not None
    assert result.config.physics_adapter == adapter
    assert result.metadata is not None
    assert result.metadata.physics_adapter == adapter
    assert result.archive is not None
    assert result.archive.name == f"{result.metadata.run_id}.h5"
    with RunArchive(result.archive) as archive:
        assert archive.identity.physics_adapter == adapter
    read, _ = RecordArchiveReader().open(result.archive)
    assert read.physics_adapter == adapter
    assert read.run_id == result.metadata.run_id
    assert "physics adapter: native" in result.report.approximations


@pytest.mark.req("TST-040")
def test_an_archive_without_the_field_reads_as_the_bridge(tmp_path: Path) -> None:
    services = Services(ShortDriver, core_builder, RecordArchiver(DESIGNS_ROOT))
    result = run_case(P1, services, out_dir=tmp_path)
    assert result.archive is not None
    with h5py.File(result.archive, "a") as handle:
        del handle.attrs["physics_adapter"]
    with RunArchive(result.archive) as archive:
        assert archive.identity.physics_adapter == "mujoco"
    with RunArchive(result.archive) as archive, pytest.raises(Exception, match="physics adapter 'bullet'"):
        replace(archive.identity, physics_adapter="bullet")
