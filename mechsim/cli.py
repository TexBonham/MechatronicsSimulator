"""The command-line client (ADR 0006 first client, ADR 0012 L9): validate, run, suite, replay, every one through
the facade and nothing else (TST-034, FAC-001). Headless (TST-035).

Exit status: 0 passed (or every file valid), 1 failed (a criterion failed, or a file is invalid), 2 error (a run error
or a case that could not run, an unknown file kind, a missing path, a usage error, or no simulation driver wired).
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import TextIO

from mechsim.facade.files import file_kind
from mechsim.facade.service import Facade, FacadeError, FacadeServices, default_services

EXIT_PASS = 0
EXIT_FAIL = 1
EXIT_ERROR = 2
SET_HELP = "bind an open parameter, or override instance:<name>.<parameter> or instance:<name>.part"
_STATUS_EXIT = {"pass": EXIT_PASS, "fail": EXIT_FAIL, "error": EXIT_ERROR}

CliServices = FacadeServices

__all__ = ["CliError", "CliServices", "default_services", "file_kind", "main", "main_exit"]


class CliError(RuntimeError):
    """A command-line input the CLI cannot use (for example a malformed --set)."""


# ===== validate ========================================================


def _validate(paths: Sequence[Path], facade: Facade, out: TextIO, err: TextIO) -> int:
    status = EXIT_PASS
    results = []
    for path in paths:
        try:
            results.append(facade.validate(path))
        except FacadeError as error:
            err.write(f"{error.message}\n")
            status = EXIT_ERROR
    for result in results:
        for file in result.files:
            if file.kind is None:
                err.write("\n".join(file.errors) + "\n")
                status = EXIT_ERROR
            elif len(file.errors) > 0:
                err.write("\n".join(file.errors) + "\n")
                status = max(status, EXIT_FAIL)
            else:
                out.write(f"{file.path}: valid {file.kind}\n")
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


def _run(arguments: argparse.Namespace, facade: Facade, out: TextIO) -> int:
    bindings = _bindings(arguments.set)
    result = facade.run_case(arguments.case, arguments.design, bindings, arguments.out, arguments.dry)
    if result.dry is not None:
        out.write(
            f"dry run {result.case_id}: base rate {result.dry.base_rate} Hz, "
            f"{result.dry.commands} commands, {result.dry.faults} faults scheduled\n"
        )
        return EXIT_PASS
    out.write(result.report_text)
    if result.archive is not None:
        out.write(f"archive: {result.archive}\n")
    return _STATUS_EXIT[result.status]


def _suite(arguments: argparse.Namespace, facade: Facade, out: TextIO, err: TextIO) -> int:
    bindings = _bindings(arguments.set)
    run = facade.run_suite(
        arguments.suite, arguments.design, bindings, arguments.out, arguments.report, arguments.write_baseline
    )
    out.write(run.report_text)
    if run.diff_text is not None:
        out.write(run.diff_text)
    out.write(f"report: {run.report_path}\n")
    if run.markdown_path is not None:
        out.write(f"shareable report: {run.markdown_path}\n")
    if run.baseline_path is not None:
        out.write(f"baseline: {run.baseline_path}\n")
    for message in run.errors:
        err.write(message + "\n")
    if len(run.errors) > 0:
        return EXIT_ERROR
    return _STATUS_EXIT[run.status]


def _replay(arguments: argparse.Namespace, facade: Facade, out: TextIO) -> int:
    """The event timeline from the archive alone (TST-033), through the facade's replay (FAC-008)."""
    handle = facade.replay_open(arguments.archive)
    out.write(
        f"Run {handle.run_id}: case {handle.case_id}, design {handle.design_name} {handle.design_version}, "
        f"seed {handle.seed}, controller mode {handle.controller_mode}\n"
    )
    for event in facade.events(handle.run_id).events:
        out.write(f"{event.t:12.6f} s  {event.source}  {event.kind}  {event.detail}\n")
    out.write(f"ended: {handle.end_reason} at {handle.end_time:.12g} s\n")
    if handle.error is not None:
        out.write(f"error: {handle.error}\n")
    return EXIT_PASS


def _error_text(error: FacadeError) -> str:
    if error.field == "design":
        return f"--design {error.message}"
    return error.message


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="mechsim", description="Mechatronic simulator command line")
    commands = parser.add_subparsers(dest="command", required=True)
    validate = commands.add_parser(
        "validate", help="validate design, case, catalog, chip, baseline, material and physics model files"
    )
    validate.add_argument("paths", type=Path, nargs="+")
    run = commands.add_parser("run", help="run one test case and write its archive")
    run.add_argument("case", type=Path)
    run.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help=SET_HELP)
    run.add_argument("--out", type=Path, default=Path("runs"), help="archive folder (default ./runs)")
    run.add_argument("--dry", action="store_true", help="configure and schedule without running")
    run.add_argument("--design", type=Path, default=None, help="the case's design file, <designs>/<name>/design.yaml")
    suite = commands.add_parser("suite", help="run every case of a suite file or folder and write its report")
    suite.add_argument("suite", type=Path, help="a suite.yaml, or a suite folder")
    suite.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help=SET_HELP)
    suite.add_argument("--out", type=Path, default=Path("runs"), help="archive and report folder (default ./runs)")
    suite.add_argument("--write-baseline", type=Path, default=None, help="write the summary as a baseline file")
    suite.add_argument("--report", type=Path, default=None, help="write the shareable Markdown report here")
    suite.add_argument("--design", type=Path, default=None, help="the cases' design file, <designs>/<name>/design.yaml")
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
    facade = Facade(services or default_services())
    handlers: dict[str, Callable[[], int]] = {
        "validate": lambda: _validate(arguments.paths, facade, stdout, stderr),
        "run": lambda: _run(arguments, facade, stdout),
        "suite": lambda: _suite(arguments, facade, stdout, stderr),
        "replay": lambda: _replay(arguments, facade, stdout),
    }
    try:
        return handlers[arguments.command]()
    except CliError as error:
        stderr.write(f"error: {error}\n")
        return EXIT_ERROR
    except FacadeError as error:
        stderr.write(f"error: {_error_text(error)}\n")
        return EXIT_ERROR


def main_exit() -> None:
    """Console-script entry `mechsim` (pyproject `[project.scripts]`): the exit status is `main`'s."""
    sys.exit(main())


if __name__ == "__main__":
    main_exit()
