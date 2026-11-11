"""The in-process facade (FAC-001, FAC-006 to FAC-012, FAC-014, FAC-015, FAC-017, FAC-018; TST-034)."""

from __future__ import annotations

import ast
import hashlib
from collections.abc import Callable, Sequence
from fractions import Fraction
from pathlib import Path
from typing import Any

import pytest

from mechsim.facade.decimate import bucket_select, decimate
from mechsim.facade.live import Batch, LiveDriver, Subscription
from mechsim.facade.service import Facade, FacadeError, FacadeServices
from mechsim.record.layout import PINS, Recording
from mechsim.record.tick import TickRecord
from mechsim.testing.driver import EndSpec, RunConfig, RunReport, Series
from mechsim.testing.runner import Services
from tests.unit.test_record_support import synthetic_rows
from tests.unit.test_runner_fakes import (
    MACHINE_CASE,
    FakeArchiver,
    FakeArchiveReader,
    FakeBuilder,
    InMemoryFakeDriver,
    Row,
    unrun_baseline,
    write_case,
    write_designs,
)

ROOT = Path(__file__).resolve().parents[2]
ANGLE = "joint:pivot.position"
SETS = {"link_mass": "1 kg"}


def fake_services(tmp_path: Path, error_tick: int | None = None) -> FacadeServices:
    designs = write_designs(tmp_path)

    def driver() -> InMemoryFakeDriver:
        return InMemoryFakeDriver(
            lambda n, t: Row({ANGLE: 0.05 + n, "signal:mcu.pa0": 3.3}),
            {ANGLE: "rad", "signal:mcu.pa0": "V"},
            {"homed": False},
            error_tick=error_tick,
        )

    runner = Services(driver, FakeBuilder(), FakeArchiver(), designs)
    return FacadeServices(runner, FakeArchiveReader(), designs, files_root=tmp_path)


def run_machine(tmp_path: Path, error_tick: int | None = None) -> tuple[Facade, Any]:
    facade = Facade(fake_services(tmp_path, error_tick))
    case = write_case(tmp_path / "cases", MACHINE_CASE)
    return facade, facade.run_case(case, sets=SETS, out_dir=tmp_path / "runs")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


# ===== Clients reach the simulator only through the facade =============


def _imported_modules(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module is not None:
            names.add(node.module)
    return names


@pytest.mark.req("FAC-001", "TST-034")
def test_the_cli_and_web_clients_import_no_runner_recorder_or_scoring() -> None:
    clients = [ROOT / "mechsim" / "cli.py", *sorted((ROOT / "web").rglob("*.py"))]
    forbidden = ("mechsim.testing", "mechsim.record", "mechsim.scoring")
    for client in clients:
        for module in _imported_modules(client):
            assert not module.startswith(forbidden), f"{client} imports {module}"
    mechsim_modules = {m for m in _imported_modules(ROOT / "mechsim" / "cli.py") if m.startswith("mechsim")}
    assert mechsim_modules == {"mechsim.facade.files", "mechsim.facade.server", "mechsim.facade.service"}


@pytest.mark.req("FAC-009")
def test_the_facade_is_python_in_mechsim_facade_and_runs_on_the_core_binding() -> None:
    from mechsim.facade.core_wiring import CoreLiveWiring
    from mechsim.facade.service import default_services
    from mechsim.testing.core_driver import CoreDriver

    services = default_services()
    assert Facade.__module__ == "mechsim.facade.service"
    assert services.runner is not None
    assert services.runner.driver is CoreDriver
    assert isinstance(services.live, CoreLiveWiring)
    assert "mechsim._core" in CoreDriver.run.__globals__["core"].__name__


# ===== validate, run, suite, replay (FAC-010) ===========================


@pytest.mark.req("FAC-010")
def test_validate_reports_each_file_and_a_missing_path_is_not_found(tmp_path: Path) -> None:
    facade = Facade(fake_services(tmp_path))
    result = facade.validate(ROOT / "designs" / "pendulum")
    assert [(Path(f.path).name, f.kind, f.errors) for f in result.files] == [("design.yaml", "design", [])]
    with pytest.raises(FacadeError) as raised:
        facade.validate(tmp_path / "missing")
    assert (raised.value.code, raised.value.message) == ("not_found", f"{tmp_path / 'missing'}: no such file or folder")


@pytest.mark.req("FAC-010")
def test_list_designs_names_each_design_folder_with_its_version(tmp_path: Path) -> None:
    facade = Facade(fake_services(tmp_path))
    listed = facade.list_designs()
    assert [(d.name, d.version, d.errors) for d in listed.designs] == [
        ("machine", "0.1.0", []),
        ("pendulum", "0.1.0", []),
    ]


@pytest.mark.req("FAC-010")
def test_run_case_returns_the_scored_summary_and_archive(tmp_path: Path) -> None:
    _, summary = run_machine(tmp_path)
    assert (summary.run_id, summary.case_id, summary.status, summary.end_reason) == (
        "M1-seed7",
        "M1",
        "fail",
        "end_time",
    )
    assert summary.archive == str(tmp_path / "runs" / "M1-seed7.h5")
    assert summary.report_text.startswith("Suite M1 (machine 0.1.0): FAIL\n")
    assert summary.error is None
    assert summary.dry is None


@pytest.mark.req("FAC-010")
def test_dry_run_returns_the_schedule_and_no_archive(tmp_path: Path) -> None:
    facade = Facade(fake_services(tmp_path))
    case = write_case(tmp_path / "cases", MACHINE_CASE)
    summary = facade.run_case(case, sets=SETS, out_dir=tmp_path / "runs", dry=True)
    assert summary.dry is not None
    assert (summary.dry.base_rate, summary.dry.commands, summary.dry.faults) == ("1000", 2, 0)
    assert summary.archive is None


@pytest.mark.req("FAC-010")
def test_run_without_a_driver_or_with_a_wrong_design_file_is_refused(tmp_path: Path) -> None:
    case = write_case(tmp_path / "cases", MACHINE_CASE)
    with pytest.raises(FacadeError) as raised:
        Facade(FacadeServices()).run_case(case)
    assert raised.value.message == "no simulation driver is wired into this build"
    with pytest.raises(FacadeError) as raised:
        Facade(fake_services(tmp_path)).run_case(case, design=case)
    assert (raised.value.field, raised.value.message) == ("design", f"{case}: not a <designs>/<name>/design.yaml file")


@pytest.mark.req("FAC-010")
def test_a_requested_seed_or_mode_must_be_the_cases_own(tmp_path: Path) -> None:
    facade = Facade(fake_services(tmp_path))
    case = write_case(tmp_path / "cases", MACHINE_CASE)
    with pytest.raises(FacadeError) as raised:
        facade.run_case(case, sets=SETS, seed=8)
    assert raised.value.code == "validation_failed"
    with pytest.raises(FacadeError) as raised:
        facade.run_case(case, sets=SETS, mode="emulated")
    assert "mode emulated requested, the case runs mode native" in raised.value.message
    assert facade.run_id_of(case) == "M1-seed7"


@pytest.mark.req("FAC-010", "FAC-012")
def test_suite_report_and_diff_are_served_by_suite_id(tmp_path: Path) -> None:
    facade = Facade(fake_services(tmp_path))
    suite = tmp_path / "demo"
    write_case(suite, MACHINE_CASE + "baseline: baseline.yaml\n")
    (suite / "baseline.yaml").write_text(unrun_baseline("demo", ["M1"]), encoding="utf-8")
    markdown = tmp_path / "report.md"
    summary = facade.run_suite(suite, sets=SETS, out_dir=tmp_path / "runs", report_path=markdown)
    assert (summary.name, summary.status, summary.errors) == ("demo", "fail", [])
    assert Path(summary.report_path).read_text(encoding="utf-8") == summary.report_text
    report = facade.report(summary.suite_id)
    assert report.markdown == markdown.read_text(encoding="utf-8")
    assert report.text == summary.report_text
    diff = facade.diff(summary.suite_id)
    assert [(e.section, e.case, e.criterion, e.old, e.new) for e in diff.entries] == [
        ("unchanged", "M1", "final_angle", "not_run", "fail")
    ]
    assert diff.text == "Regression diff against the baseline:\n  unchanged: M1/final_angle (not_run -> fail)\n"
    with pytest.raises(FacadeError) as raised:
        facade.report("nope")
    assert raised.value.code == "not_found"


# ===== Run errors (FAC-014) =============================================


@pytest.mark.req("FAC-014")
def test_a_run_error_is_a_structured_report_not_an_exception(tmp_path: Path) -> None:
    _, summary = run_machine(tmp_path, error_tick=4)
    assert summary.status == "error"
    assert summary.error is not None
    assert (summary.error.party, summary.error.cause, summary.error.requirement_id) == (
        "physics",
        "solver did not converge",
        "INV-007",
    )
    assert summary.error.t == pytest.approx(0.004, abs=0.0)


@pytest.mark.req("FAC-014")
def test_a_case_stopped_before_t0_reports_the_runner_and_its_reason(tmp_path: Path) -> None:
    facade = Facade(fake_services(tmp_path))
    case = write_case(tmp_path / "cases", MACHINE_CASE.replace("version: 0.1.0", "version: 9.9.9"))
    summary = facade.run_case(case, sets=SETS, out_dir=tmp_path / "runs")
    assert summary.status == "error"
    assert summary.run_id is None
    assert summary.error is not None
    assert (summary.error.party, summary.error.t, summary.error.requirement_id) == ("runner", 0.0, "INV-007")
    assert "9.9.9" in summary.error.cause


# ===== Replay: series, events, values (FAC-008, FAC-011) ================


@pytest.mark.req("FAC-008", "FAC-011")
def test_replay_reads_the_archive_series_events_and_values(tmp_path: Path) -> None:
    facade, summary = run_machine(tmp_path)
    handle = facade.replay_open(Path(summary.archive))
    assert (handle.run_id, handle.samples, handle.t_first, handle.t_last) == ("M1-seed7", 11, 0.0, 0.01)
    events = facade.events(handle.run_id).events
    assert [(e.t, e.kind, e.detail) for e in events] == [
        (0.002, "operator_message", "home"),
        (0.003, "operator_message", "target"),
    ]
    value = facade.value_at(handle.run_id, ANGLE, 0.0035)
    assert (value.t, value.value, value.unit) == (0.003, 3.05, "rad")
    assert facade.value_at(handle.run_id, "signal:mcu.pa0", 0.0).value == 3.3
    with pytest.raises(FacadeError) as raised:
        facade.value_at(handle.run_id, "joint:nothing.position", 0.0)
    assert raised.value.code == "not_found"


@pytest.mark.req("FAC-008")
def test_replay_needs_a_reader_and_an_archive(tmp_path: Path) -> None:
    with pytest.raises(FacadeError) as raised:
        Facade(FacadeServices()).replay_open(tmp_path / "a.h5")
    assert raised.value.message == "no archive reader is wired into this build"
    with pytest.raises(FacadeError) as raised:
        Facade(fake_services(tmp_path / "one")).replay_open(tmp_path / "a.h5")
    assert raised.value.message == f"{tmp_path / 'a.h5'}: no such archive"
    with pytest.raises(FacadeError) as raised:
        Facade(fake_services(tmp_path / "two")).events("M1-seed7")
    assert raised.value.code == "not_found"


@pytest.mark.req("FAC-007", "FAC-011", "FAC-018")
def test_series_are_decimated_in_the_service_and_the_archive_is_untouched(tmp_path: Path) -> None:
    facade, summary = run_machine(tmp_path)
    archive = Path(summary.archive)
    before = digest(archive)
    facade.replay_open(archive)
    # Samples at t_i = i ms, i = 0..10. Window [0, 0.01] in 3 buckets of 10/3 ms: centres 1.667, 5, 8.333 ms; nearest
    # samples 2, 5 and 8 ms. The angle row script is 0.05 + i.
    window = facade.series("M1-seed7", [ANGLE], 0.0, 0.01, 3)
    assert window.times == [0.002, 0.005, 0.008]
    assert window.values == [[2.05, 5.05, 8.05]]
    assert window.units == ["rad"]
    everything = facade.series("M1-seed7", [ANGLE, "signal:mcu.pa0"], 0.0, 0.01, 11)
    assert len(everything.times) == 11
    assert everything.values[1] == [3.3] * 11
    assert digest(archive) == before
    for names, start, end, points in (([ANGLE, ANGLE], 0.0, 0.01, 3), ([ANGLE], 0.01, 0.0, 3), ([ANGLE], 0.0, 0.01, 0)):
        with pytest.raises(FacadeError) as raised:
            facade.series("M1-seed7", names, start, end, points)
        assert (raised.value.code, raised.value.requirement_id) == ("validation_failed", "FAC-018")


# ===== Decimation rule (FAC-007, FAC-018) ===============================


@pytest.mark.req("FAC-007", "FAC-018")
def test_decimation_keeps_every_sample_when_points_cover_the_window() -> None:
    times = [0.0, 1.0, 2.0, 3.0, 4.0]
    assert decimate(times, 0.0, 4.0, 5) == [0, 1, 2, 3, 4]
    assert decimate(times, 0.0, 4.0, 50) == [0, 1, 2, 3, 4]
    assert decimate(times, 1.0, 3.0, 3) == [1, 2, 3]
    assert decimate(times, 5.0, 6.0, 3) == []


@pytest.mark.req("FAC-007", "FAC-018")
def test_decimation_takes_the_sample_nearest_each_bucket_centre() -> None:
    # t_i = i s, i = 0..9, window [0, 9], 3 buckets of 3 s: [0, 3), [3, 6), [6, 9]; centres 1.5, 4.5, 7.5. Each centre
    # is equidistant from two samples, so the earlier one wins: 1, 4, 7.
    times = [float(i) for i in range(10)]
    assert decimate(times, 0.0, 9.0, 3) == [1, 4, 7]
    # One bucket over [0, 9]: centre 4.5, samples 4 and 5 tie, the earlier wins.
    assert decimate(times, 0.0, 9.0, 1) == [4]
    # Irregular times: buckets [0, 5), [5, 10]; centres 2.5 and 7.5; 3.0 is nearer 2.5 than 0.0; the second bucket
    # holds only 9.0.
    assert decimate([0.0, 3.0, 9.0, 10.5], 0.0, 10.0, 2) == [1, 2]
    # An empty bucket contributes nothing: buckets [0, 4.5) and [4.5, 9] with centres 2.25 and 6.75; every sample is
    # in the first, where 2.0 is nearest 2.25.
    assert bucket_select([0.0, 1.0, 2.0], 0.0, 9.0, 2) == [2]


@pytest.mark.req("FAC-018")
def test_decimation_rejects_an_empty_window_or_zero_points() -> None:
    with pytest.raises(ValueError, match="FAC-018"):
        decimate([0.0], 1.0, 1.0, 1)
    with pytest.raises(ValueError, match="FAC-018"):
        decimate([0.0], 0.0, 1.0, 0)


# ===== Live streaming (FAC-006, FAC-007) ================================


class _RecordingView:
    def __init__(self, recording: Recording) -> None:
        self._recording = recording

    def times(self) -> Sequence[float]:
        return [float(t) for t in self._recording.series["time/seconds"].values]

    def series(self, reference: str) -> Series:
        name = reference.removeprefix("signal:")
        stored = self._recording.series[f"{PINS}/{name}"]
        return Series(tuple(self.times()), tuple(float(v) for v in stored.values), stored.unit)

    def controller_states(self) -> Sequence[str]:
        return []

    def events(self) -> Sequence[Any]:
        return []


class _RowDriver:
    """Hands each synthetic row to the live recorder as a kernel subscriber would (REC-019)."""

    def __init__(self, on_row: Callable[[TickRecord], None], rows: Sequence[TickRecord]) -> None:
        self.on_row = on_row
        self.rows = rows

    def load(self, build: Any, case: Any) -> None:
        pass

    def reported_controller_state(self) -> dict[str, bool | str]:
        return {}

    def configure(self, config: RunConfig) -> None:
        pass

    def set_initial_state(self, state: Any) -> None:
        pass

    def schedule_command(self, command: Any) -> None:
        pass

    def schedule_fault(self, fault: Any) -> None:
        pass

    def run(self, end: EndSpec) -> RunReport:
        for row in self.rows:
            self.on_row(row)
        return RunReport("end_time", len(self.rows) - 1, Fraction(1000))

    def recording(self) -> Any:
        return None


class _Wiring:
    def __init__(self, rows: Sequence[TickRecord]) -> None:
        self.rows = rows

    def driver(self, on_row: Callable[[TickRecord], None]) -> _RowDriver:
        return _RowDriver(on_row, self.rows)

    def view(self, recording: Recording, build: Any) -> _RecordingView:
        return _RecordingView(recording)


class _Listener:
    def __init__(self) -> None:
        self.progress_times: list[float] = []
        self.batches: list[Batch] = []
        self.failures: list[str] = []

    def progress(self, t: float) -> None:
        self.progress_times.append(t)

    def batch(self, batch: Batch) -> None:
        self.batches.append(batch)

    def failed(self, message: str) -> None:
        self.failures.append(message)


def _live(rows: Sequence[TickRecord], subscriptions: Sequence[Subscription]) -> _Listener:
    listener = _Listener()
    driver = LiveDriver(_Wiring(rows), listener, subscriptions)
    driver.load(object(), None)  # type: ignore[arg-type]
    driver.configure(RunConfig(Fraction(1000), None, Fraction(1000), 0, (), None))
    driver.run(EndSpec(max_tick=len(rows)))
    return listener


@pytest.mark.req("FAC-006", "FAC-007")
def test_live_batches_come_from_the_recorder_subscription_and_match_the_offline_decimation() -> None:
    rows = synthetic_rows(3, 5000, events=False, contacts=False)
    times = [row.time / 1000 for row in rows]
    subscription = Subscription(("signal:mcu.pa0",), 0.5, 4.5, 40)
    listener = _live(rows, [subscription])
    # CHUNK_ROWS = 2048: chunks end at rows 2048 and 4096, then the run's end, each a progress report.
    assert listener.progress_times == [2.047, 4.095, 4.999]
    assert [batch.sequence for batch in listener.batches] == list(range(len(listener.batches)))
    assert len(listener.batches) == 3
    streamed = [t for batch in listener.batches for t in batch.times]
    expected = bucket_select(times, 0.5, 4.5, 40)
    assert streamed == [times[index] for index in expected]
    assert len(streamed) == 40
    values = [v for batch in listener.batches for v in batch.columns[0]]
    assert values == [rows[index].pin_signals[0].value for index in expected]
    assert listener.failures == []


@pytest.mark.req("FAC-006")
def test_a_live_subscription_to_a_missing_series_fails_once_and_stops() -> None:
    rows = synthetic_rows(4, 300, events=False, contacts=False)
    listener = _live(rows, [Subscription(("signal:nothing",), 0.0, 0.2, 5)])
    assert listener.batches == []
    assert len(listener.failures) == 1


# ===== Facade run equals runner run (FAC-015, FAC-007) ==================


@pytest.mark.req("FAC-015", "FAC-007", "FAC-006")
def test_a_streamed_facade_run_archives_the_same_bytes_as_the_runner(tmp_path: Path) -> None:
    from mechsim.facade.service import default_services
    from mechsim.record.compare import archives_identical
    from mechsim.testing.runner import run_case

    services = default_services()
    assert services.runner is not None
    case = ROOT / "test_cases" / "pendulum" / "P1_period.yaml"
    listener = _Listener()
    subscription = Subscription(("joint:pivot.position",), 0.0, 24.4, 100)
    facade = Facade(services)
    summary = facade.run_case(case, out_dir=tmp_path / "facade", listener=listener, subscriptions=[subscription])
    direct = run_case(case, services.runner, None, tmp_path / "runner")
    assert summary.status == "pass"
    assert direct.archive is not None
    report = archives_identical(Path(summary.archive), direct.archive)
    assert report.identical, report.first_difference
    assert 0 < sum(len(batch.times) for batch in listener.batches) <= 100
    assert listener.failures == []


# ===== Files (FAC-017) ==================================================


@pytest.mark.req("FAC-017")
def test_files_are_read_and_written_only_as_valid_designs_and_cases(tmp_path: Path) -> None:
    facade = Facade(fake_services(tmp_path))
    design = Path("designs/pendulum/design.yaml")
    content = facade.read_file(design)
    assert content.kind == "design"
    text = content.text
    edited = text.replace("description: Single rigid link", "description: Edited rigid link")
    assert facade.write_file(design, edited).kind == "design"
    assert (tmp_path / design).read_text(encoding="utf-8") == edited
    before = (tmp_path / design).read_bytes()
    with pytest.raises(FacadeError) as raised:
        facade.write_file(design, edited.replace("mass: 1 kg", "mass: -1 kg"))
    assert (raised.value.code, raised.value.requirement_id) == ("validation_failed", "FAC-017")
    assert (tmp_path / design).read_bytes() == before
    new_case = Path("cases/M1.yaml")
    with pytest.raises(FacadeError):
        facade.write_file(new_case, MACHINE_CASE.replace("seed: 7", "seed: -7"))
    assert not (tmp_path / new_case).exists()
    assert facade.write_file(new_case, MACHINE_CASE).kind == "case"
    for path, refused in (
        (Path("designs/pendulum/notes.txt"), text),
        (Path("baseline.yaml"), unrun_baseline("demo", ["M1"])),
        (Path("../outside.yaml"), text),
        (Path("bad.yaml"), "a: [unclosed"),
    ):
        with pytest.raises(FacadeError) as raised:
            facade.write_file(path, refused)
        assert raised.value.code == "validation_failed"
    with pytest.raises(FacadeError) as raised:
        facade.read_file(Path("designs/none.yaml"))
    assert raised.value.code == "not_found"
