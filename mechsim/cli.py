"""The command-line client (ADR 0006 first client, ADR 0012 L9): validate, run, suite, replay. Headless (TST-035).

Exit status: 0 passed (or every file valid), 1 failed (a criterion failed, or a file is invalid), 2 error (a run error
or a case that could not run, an unknown file kind, a missing path, a usage error, or no simulation driver wired).
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

from pydantic import ValidationError

from mechsim.design.chip import validate_chip_file
from mechsim.design.schema import validate_design_file, yaml_loader
from mechsim.library.schema import CatalogError, load_catalog_file
from mechsim.materials.schema import MaterialError, load_material_file
from mechsim.physics.neutral import from_yaml
from mechsim.schema_base import format_errors
from mechsim.scoring.score import SuiteScore, report_text, suite_baseline
from mechsim.testing.baseline import BaselineDiff, baseline_text, validate_baseline_file
from mechsim.testing.driver import ArchiveReader
from mechsim.testing.runner import Services, run_case, run_suite
from mechsim.testing.schema import DESIGNS_ROOT, ROOT, validate_test_case_file

MATERIALS_DIR = (ROOT / "mechsim" / "materials" / "data").resolve()
EXIT_PASS = 0
EXIT_FAIL = 1
EXIT_ERROR = 2
_STATUS_EXIT = {"pass": EXIT_PASS, "fail": EXIT_FAIL, "error": EXIT_ERROR}


class CliError(RuntimeError):
    """Something the CLI needs is not available (for example no simulation driver wired)."""


@dataclass(frozen=True)
class CliServices:
    """The runner's services and the archive reader; integration wires the real ones over the core binding."""

    runner: Services | None = None
    reader: ArchiveReader | None = None
    designs_root: Path = DESIGNS_ROOT


def default_services() -> CliServices:
    return CliServices()


# ===== validate ========================================================


def file_kind(path: Path) -> str | None:
    """design, case, catalog, chip, baseline, material or physics model; None when it is none of them. Chips,
    materials and physics models by location and name, the others by their top-level keys."""
    if path.name == "chip.yaml":
        return "chip"
    if path.name.startswith("physics_model.") and path.suffix == ".yaml":
        return "physics_model"
    if path.resolve().parent == MATERIALS_DIR:
        return "material"
    data = yaml_loader().load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        return None
    keys = set(data)
    if "criteria" in keys:
        return "case"
    if {"suite", "entries"} <= keys:
        return "baseline"
    if {"types", "parts"} <= keys:
        return "catalog"
    if {"bodies", "joints"} <= keys:
        return "design"
    return None


def validate_file(path: Path, designs_root: Path) -> tuple[str | None, list[str]]:
    try:
        kind = file_kind(path)
    except Exception as error:
        return None, [f"{path}: YAML parse error: {error}"]
    if kind == "chip":
        return kind, validate_chip_file(path)
    if kind == "case":
        return kind, validate_test_case_file(path, designs_root)
    if kind == "baseline":
        return kind, validate_baseline_file(path)
    if kind == "design":
        return kind, validate_design_file(path)
    if kind == "material":
        try:
            load_material_file(path)
        except MaterialError as error:
            return kind, [str(error)]
        return kind, []
    if kind == "physics_model":
        try:
            from_yaml(path.read_text(encoding="utf-8"))
        except ValidationError as error:
            return kind, [f"{path}: {message}" for message in format_errors(error)]
        return kind, []
    if kind == "catalog":
        try:
            load_catalog_file(path)
        except CatalogError as error:
            return kind, [str(error)]
        return kind, []
    return None, [f"{path}: not a design, test case, catalog, chip, baseline, material or physics model file"]


def _validate(paths: Sequence[Path], services: CliServices, out: TextIO, err: TextIO) -> int:
    status = EXIT_PASS
    files: list[Path] = []
    for path in paths:
        if path.is_dir():
            files.extend(sorted(path.rglob("*.yaml")))
        elif path.is_file():
            files.append(path)
        else:
            err.write(f"{path}: no such file or folder\n")
            status = EXIT_ERROR
    for path in files:
        kind, errors = validate_file(path, services.designs_root)
        if kind is None:
            err.write("\n".join(errors) + "\n")
            status = EXIT_ERROR
        elif len(errors) > 0:
            err.write("\n".join(errors) + "\n")
            status = max(status, EXIT_FAIL)
        else:
            out.write(f"{path}: valid {kind}\n")
    return status


# ===== run, suite, replay ==============================================


def _bindings(pairs: Sequence[str]) -> dict[str, str]:
    bindings: dict[str, str] = {}
    for pair in pairs:
        name, separator, value = pair.partition("=")
        if separator == "" or name.strip() == "":
            raise CliError(f"--set takes name=value, got {pair!r}")
        bindings[name.strip()] = value.strip()
    return bindings


def _runner(services: CliServices) -> Services:
    if services.runner is None:
        raise CliError("no simulation driver is wired into this build")
    return services.runner


def _run(arguments: argparse.Namespace, services: CliServices, out: TextIO) -> int:
    result = run_case(arguments.case, _runner(services), _bindings(arguments.set), arguments.out, arguments.dry)
    if arguments.dry and result.config is not None:
        out.write(
            f"dry run {result.score.case_id}: base rate {result.config.base_rate} Hz, "
            f"{len(result.commands)} commands, {len(result.faults)} faults scheduled\n"
        )
        return EXIT_PASS
    score = SuiteScore(arguments.case.stem, result.score.design, (result.score,))
    out.write(report_text(score))
    if result.archive is not None:
        out.write(f"archive: {result.archive}\n")
    return _STATUS_EXIT[result.score.status]


def _suite(arguments: argparse.Namespace, services: CliServices, out: TextIO, err: TextIO) -> int:
    run = run_suite(arguments.directory, _runner(services), _bindings(arguments.set), arguments.out)
    text = report_text(run.score)
    out.write(text)
    if run.diff is not None:
        out.write(_diff_text(run.diff))
    arguments.out.mkdir(parents=True, exist_ok=True)
    report = arguments.out / f"{run.score.name}_report.txt"
    report.write_text(text, encoding="utf-8")
    out.write(f"report: {report}\n")
    if arguments.write_baseline is not None:
        arguments.write_baseline.write_text(baseline_text(suite_baseline(run.score)), encoding="utf-8")
        out.write(f"baseline: {arguments.write_baseline}\n")
    for message in run.errors:
        err.write(message + "\n")
    if len(run.errors) > 0:
        return EXIT_ERROR
    return _STATUS_EXIT[run.score.status]


def _diff_text(diff: BaselineDiff) -> str:
    lines = ["Regression diff against the baseline:"]
    for title, changes in (
        ("newly passing", diff.newly_passing),
        ("newly failing", diff.newly_failing),
        ("missing", diff.missing),
        ("added", diff.added),
    ):
        for change in changes:
            lines.append(f"  {title}: {change.case}/{change.criterion} ({change.old} -> {change.new})")
    lines.append(f"  unchanged: {len(diff.unchanged)}")
    return "\n".join(lines) + "\n"


def _replay(arguments: argparse.Namespace, services: CliServices, out: TextIO) -> int:
    """The event timeline from the archive alone (TST-033); the viewer comes later."""
    if services.reader is None:
        raise CliError("no archive reader is wired into this build")
    if not arguments.archive.is_file():
        raise CliError(f"{arguments.archive}: no such archive")
    metadata, recording = services.reader.open(arguments.archive)
    mode = "none"
    if metadata.controller_mode is not None:
        mode = metadata.controller_mode
    out.write(
        f"Run {metadata.run_id}: case {metadata.case_id}, design {metadata.design_name} {metadata.design_version}, "
        f"seed {metadata.seed}, controller mode {mode}\n"
    )
    for event in recording.events():
        out.write(f"{event.time:12.6f} s  {event.source}  {event.kind}  {event.detail}\n")
    report = metadata.report
    out.write(f"ended: {report.reason} at {report.end_time:.12g} s\n")
    if report.error is not None:
        out.write(f"error: {report.error}\n")
    return EXIT_PASS


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="mechsim", description="Mechatronic simulator command line")
    commands = parser.add_subparsers(dest="command", required=True)
    validate = commands.add_parser(
        "validate", help="validate design, case, catalog, chip, baseline, material and physics model files"
    )
    validate.add_argument("paths", type=Path, nargs="+")
    run = commands.add_parser("run", help="run one test case and write its archive")
    run.add_argument("case", type=Path)
    run.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="bind an open parameter")
    run.add_argument("--out", type=Path, default=Path("runs"), help="archive folder (default ./runs)")
    run.add_argument("--dry", action="store_true", help="configure and schedule without running")
    suite = commands.add_parser("suite", help="run every case of a suite folder and write its report")
    suite.add_argument("directory", type=Path)
    suite.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="bind an open parameter")
    suite.add_argument("--out", type=Path, default=Path("runs"), help="archive and report folder (default ./runs)")
    suite.add_argument("--write-baseline", type=Path, default=None, help="write the summary as a baseline file")
    replay = commands.add_parser("replay", help="print a run's event timeline from its archive")
    replay.add_argument("archive", type=Path)
    return parser


def main(
    argv: Sequence[str] | None = None,
    services: CliServices | None = None,
    out: TextIO | None = None,
    err: TextIO | None = None,
) -> int:
    stdout = out or sys.stdout
    stderr = err or sys.stderr
    try:
        arguments = build_parser().parse_args(argv)
    except SystemExit as error:
        if error.code == 0:
            return EXIT_PASS
        return EXIT_ERROR
    chosen = services or default_services()
    handlers: dict[str, Callable[[], int]] = {
        "validate": lambda: _validate(arguments.paths, chosen, stdout, stderr),
        "run": lambda: _run(arguments, chosen, stdout),
        "suite": lambda: _suite(arguments, chosen, stdout, stderr),
        "replay": lambda: _replay(arguments, chosen, stdout),
    }
    try:
        return handlers[arguments.command]()
    except CliError as error:
        stderr.write(f"error: {error}\n")
        return EXIT_ERROR


if __name__ == "__main__":
    sys.exit(main())
