"""The in-process facade (ADR 0006, FAC-001, FAC-009): the one entry point every client drives the simulator through.
Plain synchronous methods returning pydantic result models; the CLI and the WebSocket service both call only this.
Run errors come back as structured reports, never as exceptions (FAC-014); request errors raise `FacadeError` with a
protocol error code (FAC-020)."""

from __future__ import annotations

import bisect
import threading
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Literal

from pydantic import Field, JsonValue

from mechsim.design.schema import DesignError, validate_design_file, yaml_loader
from mechsim.facade.decimate import decimate
from mechsim.facade.files import WRITABLE_KINDS, kind_of_text, validate_file
from mechsim.facade.live import LiveWiring, RunListener, Subscription, live_factory
from mechsim.facade.protocol import ErrorCode
from mechsim.record.archive import RunArchive
from mechsim.schema_base import StrictModel
from mechsim.scoring.score import (
    CriterionResult,
    SuiteScore,
    diff_sections,
    report_markdown,
    report_text,
    suite_baseline,
)
from mechsim.testing.baseline import BaselineDiff, baseline_text
from mechsim.testing.driver import ArchiveReader, RecordView, RunMetadata
from mechsim.testing.runner import RunResult, Services, run_case, run_id, run_suite
from mechsim.testing.schema import DESIGNS_ROOT, ROOT, CaseError, load_test_case
from mechsim.viewer import (
    ANGLE_UNIT,
    STRESS_UNIT,
    OverlayError,
    ProbeError,
    ReplayCursor,
    ReplayError,
    body_geometry,
    down_from_design,
    member_stress,
    probe,
    swing_angle,
    timeline,
)

RUN_ERROR_REQUIREMENT = "INV-007"
CORE_MODULE = "mechsim._core"
ProbeKind = Literal["body", "joint", "pin", "signal"]


class FacadeError(Exception):
    """A request the facade refuses; `code` is the protocol error code, `field` names the offending input."""

    def __init__(
        self, code: ErrorCode, message: str, requirement_id: str | None = None, field: str | None = None
    ) -> None:
        super().__init__(message)
        self.code: ErrorCode = code
        self.message = message
        self.requirement_id = requirement_id
        self.field = field


@dataclass(frozen=True)
class FacadeServices:
    """The runner's services, the archive reader, the designs folder, live-row wiring and the folder files are read
    from and written to (FAC-017). The CLI's `CliServices` is this class."""

    runner: Services | None = None
    reader: ArchiveReader | None = None
    designs_root: Path = DESIGNS_ROOT
    live: LiveWiring | None = None
    files_root: Path = ROOT


def default_services() -> FacadeServices:
    """The archive reader always (replay needs no engine, VIE-016); the runner and live wiring over the core binding
    when the extension is built, else unwired (runs refused, validate and replay still work)."""
    from mechsim.record.reader import RecordArchiveReader

    reader = RecordArchiveReader()
    try:
        from mechsim.facade.core_wiring import CoreLiveWiring
        from mechsim.testing.core_driver import core_services
    except ImportError as error:
        if error.name is None or not error.name.startswith(CORE_MODULE):
            raise
        return FacadeServices(reader=reader)
    return FacadeServices(runner=core_services(), reader=reader, live=CoreLiveWiring())


# ===== Result models ===================================================


class DesignEntry(StrictModel):
    name: str
    path: str
    version: str | None
    errors: list[str]


class DesignList(StrictModel):
    designs: list[DesignEntry]


class RunEntry(StrictModel):
    run_id: str
    archive: str | None
    open: bool


class RunList(StrictModel):
    runs: list[RunEntry]


class FileValidation(StrictModel):
    path: str
    kind: str | None
    errors: list[str]


class ValidationResult(StrictModel):
    files: list[FileValidation]


class RunErrorReport(StrictModel):
    """Who failed, when (simulated s), why, and the requirement a run error answers to (FAC-014, INV-007)."""

    party: str
    t: float
    cause: str
    requirement_id: str


class DrySummary(StrictModel):
    base_rate: str
    commands: int
    faults: int


class RunResultSummary(StrictModel):
    run_id: str | None
    case_id: str
    status: str
    archive: str | None
    end_reason: str | None
    end_time: float | None
    report_text: str
    error: RunErrorReport | None
    dry: DrySummary | None


class DiffEntry(StrictModel):
    section: str
    case: str
    criterion: str
    old: str | None
    new: str | None


class SuiteResultSummary(StrictModel):
    suite_id: str
    name: str
    status: str
    report_text: str
    diff_text: str | None
    diff: list[DiffEntry]
    report_path: str
    markdown_path: str | None
    baseline_path: str | None
    errors: list[str]
    runs: list[RunResultSummary]


class ReportResult(StrictModel):
    suite_id: str
    text: str
    markdown: str


class DiffResult(StrictModel):
    suite_id: str
    entries: list[DiffEntry]
    text: str | None


class ReplayHandle(StrictModel):
    run_id: str
    case_id: str
    design_name: str
    design_version: str
    seed: int
    controller_mode: str
    end_reason: str
    end_time: float
    error: str | None
    samples: int
    t_first: float
    t_last: float


class EventEntry(StrictModel):
    t: float
    source: str
    kind: str
    detail: str


class EventList(StrictModel):
    run_id: str
    events: list[EventEntry]


class SeriesWindow(StrictModel):
    """Decimated samples: `values[i]` is the column of `series[i]`, aligned with `times` (FAC-007, FAC-018)."""

    run_id: str
    series: list[str]
    units: list[str]
    times: list[float]
    values: list[list[float]]


class ValueAt(StrictModel):
    run_id: str
    ref: str
    t: float
    value: float
    unit: str


class ProbeColumnResult(StrictModel):
    name: str
    unit: str
    values: list[float]


class ProbeResult(StrictModel):
    run_id: str
    kind: str
    name: str
    times: list[float]
    columns: list[ProbeColumnResult]


class TimelineEntryResult(StrictModel):
    t: float
    category: str
    label: str
    source: str


class TimelineResult(StrictModel):
    run_id: str
    entries: list[TimelineEntryResult]


class MemberStressEntry(StrictModel):
    member: str
    fibre: float = Field(description="Extreme-fibre distance c, m (the member list's section).")
    second_moment: float = Field(description="Second moment of area I, m^4 (the member list's section).")
    start: list[float]
    end: list[float]


class MemberStressResult(StrictModel):
    """Bending stress M c / I at each member end per recorded sample, Pa (VIE-010)."""

    run_id: str
    unit: str
    times: list[float]
    members: list[MemberStressEntry]


class SwingAngleResult(StrictModel):
    """Swing angle of a suspended load from the vertical through its attachment per recorded sample, rad (VIE-011)."""

    run_id: str
    load: str
    attachment: str
    unit: str
    times: list[float]
    values: list[float]


class GeometryResult(StrictModel):
    run_id: str
    bodies: list[dict[str, JsonValue]]


class CursorResult(StrictModel):
    run_id: str
    index: int
    t: float
    paused: bool
    samples: int


class FileContent(StrictModel):
    path: str
    kind: str
    text: str


class WriteResult(StrictModel):
    path: str
    kind: str


# ===== Helpers ==========================================================


@dataclass
class _Replay:
    archive: Path
    metadata: RunMetadata
    view: RecordView
    cursor: ReplayCursor
    criteria: tuple[CriterionResult, ...]


def _error_report(result: RunResult) -> RunErrorReport | None:
    score = result.score
    if score.status != "error":
        return None
    t = 0.0
    if score.end_time is not None:
        t = float(score.end_time)
    text = score.error or "run error"
    party = "runner"
    if result.report is not None and result.report.error is not None:
        text = result.report.error
        party = "run"
    elif score.ran or score.end_reason == "run_error":
        party = "driver"
    name, separator, rest = text.partition(": ")
    if separator != "" and " " not in name and party != "runner":
        party = name
        text = rest
    return RunErrorReport(party=party, t=t, cause=text, requirement_id=RUN_ERROR_REQUIREMENT)


def _summary(result: RunResult, name: str, dry_run: bool = False) -> RunResultSummary:
    score = result.score
    dry: DrySummary | None = None
    if dry_run and result.config is not None:
        dry = DrySummary(
            base_rate=str(result.config.base_rate), commands=len(result.commands), faults=len(result.faults)
        )
    archive: str | None = None
    if result.archive is not None:
        archive = str(result.archive)
    identifier: str | None = None
    if result.metadata is not None:
        identifier = result.metadata.run_id
    return RunResultSummary(
        run_id=identifier,
        case_id=score.case_id,
        status=score.status,
        archive=archive,
        end_reason=score.end_reason,
        end_time=score.end_time,
        report_text=report_text(SuiteScore(name, score.design, (score,))),
        error=_error_report(result),
        dry=dry,
    )


def _diff_entries(diff: BaselineDiff | None) -> list[DiffEntry]:
    if diff is None:
        return []
    entries: list[DiffEntry] = []
    for title, changes in diff_sections(diff):
        for change in changes:
            entries.append(
                DiffEntry(section=title, case=change.case, criterion=change.criterion, old=change.old, new=change.new)
            )
    return entries


def _diff_text(entries: Sequence[DiffEntry]) -> str:
    """Newly passing and newly failing first, then unchanged, missing, added (SCO-010)."""
    lines = ["Regression diff against the baseline:"]
    for entry in entries:
        lines.append(f"  {entry.section}: {entry.case}/{entry.criterion} ({entry.old} -> {entry.new})")
    return "\n".join(lines) + "\n"


def _plain(value: Any) -> JsonValue:
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, list | tuple):
        return [_plain(item) for item in value]
    if value is None or isinstance(value, str | int | float | bool):
        return value
    return str(value)


# ===== Facade ===========================================================


class Facade:
    """One instance per service. Replay handles and suite results live for the instance (keyed by run id and suite
    id); every method is safe to call from one thread at a time per handle."""

    def __init__(self, services: FacadeServices | None = None, runs_dir: Path = Path("runs")) -> None:
        self._services = services or FacadeServices()
        self.runs_dir = runs_dir
        self._replays: dict[str, _Replay] = {}
        self._suites: dict[str, tuple[SuiteResultSummary, str]] = {}
        self._scores: dict[Path, tuple[CriterionResult, ...]] = {}
        self._write_lock = threading.Lock()

    @property
    def services(self) -> FacadeServices:
        return self._services

    # ----- designs and files ------------------------------------------

    def list_designs(self, root: Path | None = None) -> DesignList:
        folder = root or self._services.designs_root
        entries: list[DesignEntry] = []
        for path in sorted(folder.glob("*/design.yaml")):
            errors = validate_design_file(path)
            version: str | None = None
            try:
                data = yaml_loader().load(path.read_text(encoding="utf-8"))
            except Exception:
                data = None
            if isinstance(data, dict) and isinstance(data.get("version"), str):
                version = data["version"]
            entries.append(DesignEntry(name=path.parent.name, path=str(path), version=version, errors=errors))
        return DesignList(designs=entries)

    def list_runs(self) -> RunList:
        """Archives `<runs_dir>/*.h5` (run id = file stem) and every open replay, by run id (ruling 22)."""
        found: dict[str, RunEntry] = {}
        if self.runs_dir.is_dir():
            for path in sorted(self.runs_dir.glob("*.h5")):
                found[path.stem] = RunEntry(run_id=path.stem, archive=str(path), open=False)
        for name, replay in self._replays.items():
            found[name] = RunEntry(run_id=name, archive=str(replay.archive), open=True)
        return RunList(runs=[found[key] for key in sorted(found)])

    def validate(self, path: Path) -> ValidationResult:
        """Every YAML file under a folder, or the one file (DES-036); a missing path is `not_found`."""
        files: list[Path] = []
        if path.is_dir():
            files.extend(sorted(path.rglob("*.yaml")))
        elif path.is_file():
            files.append(path)
        else:
            raise FacadeError("not_found", f"{path}: no such file or folder", "FAC-010", "path")
        results: list[FileValidation] = []
        for file in files:
            kind, errors = validate_file(file, self._services.designs_root)
            results.append(FileValidation(path=str(file), kind=kind, errors=errors))
        return ValidationResult(files=results)

    def _inside(self, path: Path) -> Path:
        root = self._services.files_root.resolve()
        resolved = path
        if not path.is_absolute():
            resolved = root / path
        resolved = resolved.resolve()
        if not resolved.is_relative_to(root):
            raise FacadeError("validation_failed", f"{path}: outside the files root {root}", "FAC-017", "path")
        return resolved

    def read_file(self, path: Path) -> FileContent:
        """A design or test case file's text (FAC-017); any other file is refused."""
        target = self._inside(path)
        if not target.is_file():
            raise FacadeError("not_found", f"{path}: no such file", "FAC-017", "path")
        text = target.read_bytes().decode("utf-8")
        kind = self._writable_kind(target, text, path)
        return FileContent(path=path.as_posix(), kind=kind, text=text)

    def _writable_kind(self, target: Path, text: str, shown: Path) -> str:
        try:
            kind = kind_of_text(target, text)
        except Exception as error:
            raise FacadeError("validation_failed", f"{shown}: YAML parse error: {error}", "FAC-017", "text") from error
        if kind not in WRITABLE_KINDS:
            raise FacadeError("validation_failed", f"{shown}: not a design or test case file", "FAC-017", "path")
        return kind

    def write_file(self, path: Path, text: str) -> WriteResult:
        """Writes a design or test case file only when the new text validates against its schema (FAC-017, DES-036);
        the file is checked in place and restored byte for byte when it does not validate."""
        target = self._inside(path)
        if target.suffix != ".yaml":
            raise FacadeError("validation_failed", f"{path}: not a .yaml file", "FAC-017", "path")
        kind = self._writable_kind(target, text, path)
        with self._write_lock:
            previous: bytes | None = None
            if target.is_file():
                previous = target.read_bytes()
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(text.encode("utf-8"))
            _, errors = validate_file(target, self._services.designs_root)
            if len(errors) > 0:
                if previous is None:
                    target.unlink()
                else:
                    target.write_bytes(previous)
                raise FacadeError("validation_failed", "\n".join(errors), "FAC-017", "text")
        return WriteResult(path=path.as_posix(), kind=kind)

    # ----- runs --------------------------------------------------------

    def _runner(self, design: Path | None) -> Services:
        """`design` names `<designs>/<name>/design.yaml`; its folder's parent is the designs root of the run."""
        runner = self._services.runner
        if runner is None:
            raise FacadeError("not_found", "no simulation driver is wired into this build", "FAC-009")
        if design is None:
            return runner
        if design.name != "design.yaml" or not design.is_file():
            raise FacadeError("not_found", f"{design}: not a <designs>/<name>/design.yaml file", "FAC-010", "design")
        return replace(runner, designs_root=design.resolve().parent.parent)

    def run_id_of(self, case: Path, design: Path | None = None) -> str:
        """The run id a nominal run of the case gets (`<case id>-seed<seed>`), for subscribing before it starts."""
        runner = self._runner(design)
        try:
            return run_id(load_test_case(case, runner.designs_root), None)
        except CaseError as error:
            raise FacadeError("validation_failed", str(error), "TST-009", "case") from error

    def _check_identity(self, case: Path, designs_root: Path, seed: int | None, mode: str | None) -> None:
        """A requested seed or mode must be the case's own; a case that does not load is left to the runner, which
        scores it as not run with the reason (TST-023)."""
        if seed is None and mode is None:
            return
        try:
            loaded = load_test_case(case, designs_root)
        except (CaseError, OSError):
            return
        if seed is not None and seed != loaded.seed:
            raise FacadeError(
                "validation_failed", f"{case}: seed {seed} requested, the case runs seed {loaded.seed}", "TST-009"
            )
        written = loaded.controller_mode or "none"
        if mode is not None and mode != written:
            raise FacadeError(
                "validation_failed", f"{case}: mode {mode} requested, the case runs mode {written}", "TST-036"
            )

    def run_case(
        self,
        case: Path,
        design: Path | None = None,
        sets: Mapping[str, str] | None = None,
        out_dir: Path | None = None,
        dry: bool = False,
        seed: int | None = None,
        mode: str | None = None,
        listener: RunListener | None = None,
        subscriptions: Sequence[Subscription] = (),
    ) -> RunResultSummary:
        """One run through the runner, scored and archived under `out_dir` (the facade's runs folder by default).
        A run error is the summary's `error`, never an exception (FAC-014). With a listener and live wiring, rows
        stream as progress and decimated batches while the run goes (FAC-006, FAC-007)."""
        runner = self._runner(design)
        self._check_identity(case, runner.designs_root, seed, mode)
        live = self._services.live
        if listener is not None and live is not None:
            runner = replace(runner, driver=live_factory(live, listener, subscriptions))
        target = out_dir or self.runs_dir
        result = run_case(case, runner, sets, target, dry)
        if result.archive is not None:
            self._scores[result.archive.resolve()] = result.score.criteria
        return _summary(result, case.stem, dry)

    def run_suite(
        self,
        suite: Path,
        design: Path | None = None,
        sets: Mapping[str, str] | None = None,
        out_dir: Path | None = None,
        report_path: Path | None = None,
        baseline_path: Path | None = None,
    ) -> SuiteResultSummary:
        """Every case of the suite, the regression diff, `<out>/<name>_report.txt`, and on request the shareable
        Markdown report (SCO-012) and the summary as a baseline file."""
        target = out_dir or self.runs_dir
        run = run_suite(suite, self._runner(design), sets, target)
        text = report_text(run.score)
        entries = _diff_entries(run.diff)
        diff_text: str | None = None
        if run.diff is not None:
            diff_text = _diff_text(entries)
        target.mkdir(parents=True, exist_ok=True)
        report = target / f"{run.score.name}_report.txt"
        report.write_text(text, encoding="utf-8")
        markdown = report_markdown(run.score, run.diff)
        markdown_path: str | None = None
        if report_path is not None:
            report_path.write_text(markdown, encoding="utf-8")
            markdown_path = str(report_path)
        written_baseline: str | None = None
        if baseline_path is not None:
            baseline_path.write_text(baseline_text(suite_baseline(run.score)), encoding="utf-8")
            written_baseline = str(baseline_path)
        for case_run in run.runs:
            if case_run.archive is not None:
                self._scores[case_run.archive.resolve()] = case_run.score.criteria
        suite_id = f"{run.score.name}-{len(self._suites) + 1}"
        summary = SuiteResultSummary(
            suite_id=suite_id,
            name=run.score.name,
            status=run.score.status,
            report_text=text,
            diff_text=diff_text,
            diff=entries,
            report_path=str(report),
            markdown_path=markdown_path,
            baseline_path=written_baseline,
            errors=list(run.errors),
            runs=[_summary(case_run, case_run.score.case_id) for case_run in run.runs],
        )
        self._suites[suite_id] = (summary, markdown)
        return summary

    def _suite(self, suite_id: str) -> tuple[SuiteResultSummary, str]:
        if suite_id not in self._suites:
            raise FacadeError("not_found", f"no suite result {suite_id!r}", "FAC-012", "suite_id")
        return self._suites[suite_id]

    def report(self, suite_id: str) -> ReportResult:
        """The pass/fail report of a suite this facade ran: text and shareable Markdown (FAC-012, SCO-012)."""
        summary, markdown = self._suite(suite_id)
        return ReportResult(suite_id=suite_id, text=summary.report_text, markdown=markdown)

    def diff(self, suite_id: str) -> DiffResult:
        """The regression diff of a suite this facade ran, in report order (FAC-012, SCO-010)."""
        summary, _ = self._suite(suite_id)
        return DiffResult(suite_id=suite_id, entries=summary.diff, text=summary.diff_text)

    # ----- replay ------------------------------------------------------

    def replay_open(self, archive: Path) -> ReplayHandle:
        """Opens a run archive through the archive reader alone (FAC-008, VIE-016); the handle is the run id."""
        reader = self._services.reader
        if reader is None:
            raise FacadeError("not_found", "no archive reader is wired into this build", "FAC-008")
        if not archive.is_file():
            raise FacadeError("not_found", f"{archive}: no such archive", "FAC-008", "archive")
        try:
            metadata, view = reader.open(archive)
        except (OSError, ValueError, KeyError) as error:
            raise FacadeError("not_found", f"{archive}: not a run archive: {error}", "FAC-008", "archive") from error
        times = list(view.times())
        try:
            cursor = ReplayCursor(times)
        except ReplayError as error:
            raise FacadeError("not_found", f"{archive}: {error}", "VIE-016", "archive") from error
        criteria = self._scores.get(archive.resolve(), ())
        self._replays[metadata.run_id] = _Replay(archive, metadata, view, cursor, criteria)
        report = metadata.report
        return ReplayHandle(
            run_id=metadata.run_id,
            case_id=metadata.case_id,
            design_name=metadata.design_name,
            design_version=metadata.design_version,
            seed=metadata.seed,
            controller_mode=metadata.controller_mode or "none",
            end_reason=report.reason,
            end_time=report.end_time,
            error=report.error,
            samples=len(times),
            t_first=times[0],
            t_last=times[-1],
        )

    def is_open(self, run_id: str) -> bool:
        return run_id in self._replays

    def _replay(self, run_id: str) -> _Replay:
        if run_id not in self._replays:
            raise FacadeError("not_found", f"no open replay {run_id!r}", "FAC-008", "run_id")
        return self._replays[run_id]

    def events(self, run_id: str) -> EventList:
        replay = self._replay(run_id)
        events = [
            EventEntry(t=event.time, source=event.source, kind=event.kind, detail=event.detail)
            for event in replay.view.events()
        ]
        return EventList(run_id=run_id, events=events)

    def timeline(self, run_id: str) -> TimelineResult:
        """Commands, faults, safety events and criterion failures at their simulated times (VIE-002); criterion
        failures for runs this facade scored."""
        replay = self._replay(run_id)
        entries = timeline(replay.view.events(), replay.criteria, replay.metadata.report.end_time)
        return TimelineResult(
            run_id=run_id,
            entries=[
                TimelineEntryResult(t=entry.t, category=entry.category, label=entry.label, source=entry.source)
                for entry in entries
            ],
        )

    def series(self, run_id: str, names: Sequence[str], t_start: float, t_end: float, points: int) -> SeriesWindow:
        """Recorded series decimated to at most `points` samples over [t_start, t_end] (FAC-007, FAC-018)."""
        replay = self._replay(run_id)
        if len(names) == 0 or len(set(names)) != len(names):
            raise FacadeError("validation_failed", "series names must be one or more and distinct", "FAC-018")
        if points < 1 or not t_start < t_end:
            raise FacadeError("validation_failed", "need points >= 1 and t_start < t_end", "FAC-018")
        times = list(replay.view.times())
        kept = decimate(times, t_start, t_end, points)
        units: list[str] = []
        columns: list[list[float]] = []
        for name in names:
            try:
                recorded = replay.view.series(name)
            except KeyError as error:
                raise FacadeError("not_found", f"no series {name!r} in run {run_id}", "FAC-011", "series") from error
            units.append(recorded.unit)
            columns.append([float(recorded.values[index]) for index in kept])
        return SeriesWindow(
            run_id=run_id, series=list(names), units=units, times=[times[index] for index in kept], values=columns
        )

    def value_at(self, run_id: str, ref: str, t: float) -> ValueAt:
        """A body (`pose:`), joint (`joint:`), pin or signal (`signal:`) value at the latest recorded time at or
        before t (FAC-011)."""
        replay = self._replay(run_id)
        times = list(replay.view.times())
        index = max(bisect.bisect_right(times, t) - 1, 0)
        try:
            recorded = replay.view.series(ref)
        except KeyError as error:
            raise FacadeError("not_found", f"no series {ref!r} in run {run_id}", "FAC-011", "ref") from error
        return ValueAt(run_id=run_id, ref=ref, t=times[index], value=float(recorded.values[index]), unit=recorded.unit)

    def probe(self, run_id: str, kind: ProbeKind, name: str) -> ProbeResult:
        """VIE-003 to VIE-006, computed in `mechsim.viewer` (VIE-014)."""
        replay = self._replay(run_id)
        try:
            found = probe(replay.view, kind, name)
        except ProbeError as error:
            raise FacadeError("not_found", str(error.args[0]), "FAC-011", "name") from error
        return ProbeResult(
            run_id=run_id,
            kind=kind,
            name=name,
            times=list(found.times),
            columns=[
                ProbeColumnResult(name=column.name, unit=column.unit, values=list(column.values))
                for column in found.columns
            ],
        )

    def member_stress(self, run_id: str) -> MemberStressResult:
        """VIE-010 overlay, computed in `mechsim.viewer` from the recorded end-loads and the archived member list."""
        replay = self._replay(run_id)
        try:
            found = member_stress(replay.view, replay.metadata.members)
        except OverlayError as error:
            raise FacadeError("not_found", f"run {run_id}: {error.args[0]}", "VIE-010", "run_id") from error
        return MemberStressResult(
            run_id=run_id,
            unit=STRESS_UNIT,
            times=[float(t) for t in replay.view.times()],
            members=[
                MemberStressEntry(
                    member=entry.member,
                    fibre=entry.fibre,
                    second_moment=entry.second_moment,
                    start=list(entry.start),
                    end=list(entry.end),
                )
                for entry in found
            ],
        )

    def swing_angle(self, run_id: str, load: str, attachment: str) -> SwingAngleResult:
        """VIE-011 overlay: the load body's swing from the vertical (the archived design's gravity) through the
        attachment body's origin, or through the world origin for `world`."""
        replay = self._replay(run_id)
        design_file = b""
        try:
            with RunArchive(replay.archive) as archive:
                design_file = archive.identity.design_file
        except (OSError, ValueError, KeyError):
            design_file = b""
        try:
            found = swing_angle(replay.view, load, attachment, down=down_from_design(design_file))
        except OverlayError as error:
            raise FacadeError("not_found", f"run {run_id}: {error.args[0]}", "VIE-011", "load") from error
        return SwingAngleResult(
            run_id=run_id,
            load=load,
            attachment=attachment,
            unit=ANGLE_UNIT,
            times=[float(t) for t in replay.view.times()],
            values=list(found.values),
        )

    def geometry(self, run_id: str) -> GeometryResult:
        """Body geometry from the design file the archive carries (VIE-015); reads only (VIE-017)."""
        replay = self._replay(run_id)
        design_file = b""
        try:
            with RunArchive(replay.archive) as archive:
                design_file = archive.identity.design_file
        except (OSError, ValueError, KeyError):
            design_file = b""
        try:
            bodies = body_geometry(design_file)
        except DesignError as error:
            raise FacadeError("not_found", f"run {run_id}: {error}", "VIE-015", "run_id") from error
        plain: list[dict[str, JsonValue]] = []
        for body in bodies:
            plain.append(
                {
                    "body": body.body,
                    "kind": body.kind,
                    "file": body.file,
                    "shapes": [
                        {
                            "shape": shape.shape,
                            "position": _plain(shape.position),
                            "orientation": _plain(shape.orientation),
                            "sizes": _plain(shape.sizes),
                        }
                        for shape in body.shapes
                    ],
                }
            )
        return GeometryResult(run_id=run_id, bodies=plain)

    def _cursor(self, run_id: str, action: str, value: Any) -> CursorResult:
        replay = self._replay(run_id)
        try:
            if action == "scrub":
                state = replay.cursor.scrub(value)
            elif action == "pause":
                state = replay.cursor.pause(value)
            elif action == "advance":
                state = replay.cursor.advance(value)
            else:
                state = replay.cursor.step(value)
        except ReplayError as error:
            raise FacadeError("validation_failed", str(error), "VIE-007", "t") from error
        return CursorResult(run_id=run_id, index=state.index, t=state.t, paused=state.paused, samples=state.samples)

    def scrub(self, run_id: str, t: float) -> CursorResult:
        return self._cursor(run_id, "scrub", t)

    def pause(self, run_id: str, paused: bool) -> CursorResult:
        return self._cursor(run_id, "pause", paused)

    def step(self, run_id: str, direction: Literal["forward", "backward"]) -> CursorResult:
        return self._cursor(run_id, "step", direction)

    def advance(self, run_id: str, dt: float) -> CursorResult:
        return self._cursor(run_id, "advance", dt)
