"""The CLI (TST-030 to TST-033, TST-035): validate, run, suite, replay; exit 0 pass, 1 fail, 2 error."""

from __future__ import annotations

import io
import os
import re
import socket
import subprocess
import sys
from pathlib import Path

import pytest

from mechsim.cli import EXIT_ERROR, EXIT_FAIL, EXIT_PASS, CliServices, file_kind, main
from mechsim.testing.runner import Services
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
SHIPPED = sorted(
    [
        *ROOT.glob("designs/*/design.yaml"),
        *ROOT.glob("test_cases/**/*.yaml"),
        *ROOT.glob("chips/*/chip.yaml"),
        *ROOT.glob("mechsim/library/catalog/*.yaml"),
        *ROOT.glob("mechsim/materials/data/*.yaml"),
        *ROOT.glob("schemas/examples/*.yaml"),
    ]
)
ANGLE = "joint:pivot.position"


def call(argv: list[str], services: CliServices | None = None) -> tuple[int, str, str]:
    out = io.StringIO()
    err = io.StringIO()
    code = main(argv, services, out, err)
    return code, out.getvalue(), err.getvalue()


def cli_services(tmp_path: Path, angle: float = 0.05, error_tick: int | None = None) -> CliServices:
    designs = write_designs(tmp_path)

    def driver() -> InMemoryFakeDriver:
        return InMemoryFakeDriver(
            lambda n, t: Row({ANGLE: angle}), {ANGLE: "rad"}, {"homed": False}, error_tick=error_tick
        )

    runner = Services(driver, FakeBuilder(), FakeArchiver(), designs)
    return CliServices(runner, FakeArchiveReader(), designs)


# ===== validate ========================================================


@pytest.mark.req("TST-030")
@pytest.mark.parametrize("path", SHIPPED, ids=[str(path.relative_to(ROOT)) for path in SHIPPED])
def test_validate_accepts_every_shipped_file(path: Path) -> None:
    code, out, err = call(["validate", str(path)])
    assert (code, err) == (EXIT_PASS, "")
    assert out.startswith(f"{path}: valid ")


@pytest.mark.req("TST-030")
def test_validate_tells_the_kinds_apart() -> None:
    kinds = {path.relative_to(ROOT).as_posix(): file_kind(path) for path in SHIPPED}
    assert kinds["designs/pendulum/design.yaml"] == "design"
    assert kinds["test_cases/gantry/G3.yaml"] == "case"
    assert kinds["test_cases/gantry/baseline.yaml"] == "baseline"
    assert kinds["test_cases/gantry/suite.yaml"] == "suite"
    assert kinds["chips/stm32_placeholder/chip.yaml"] == "chip"
    assert kinds["mechsim/library/catalog/limit_switch.yaml"] == "catalog"
    assert kinds["mechsim/materials/data/abs_printed.yaml"] == "material"
    assert kinds["schemas/examples/physics_model.pendulum.yaml"] == "physics_model"
    assert kinds["schemas/examples/material_overrides.example.yaml"] == "material_overrides"
    assert None not in kinds.values()


@pytest.mark.req("TST-030")
def test_every_shipped_yaml_file_is_covered() -> None:
    every = {
        path
        for path in ROOT.rglob("*.yaml")
        if not any(
            part.startswith(".") or part in ("tests", "build", "node_modules") for part in path.relative_to(ROOT).parts
        )
    }
    assert every == set(SHIPPED)


def test_validate_an_invalid_physics_model_fails(tmp_path: Path) -> None:
    path = tmp_path / "physics_model.bad.yaml"
    path.write_text("schema_version: 1\n", encoding="utf-8")
    code, _, err = call(["validate", str(path)])
    assert code == EXIT_FAIL
    assert f"{path}: " in err


@pytest.mark.req("TST-030")
def test_validate_a_folder_runs_nothing() -> None:
    code, out, _ = call(["validate", str(ROOT / "test_cases")])
    assert code == EXIT_PASS
    # G1 to G11, G13, G14, P1, P2, the gantry baseline and suite: 17 files.
    assert out.count(": valid ") == 17


@pytest.mark.req("TST-030")
def test_validate_reports_an_invalid_file_as_failed(tmp_path: Path) -> None:
    bad = tmp_path / "X1.yaml"
    bad.write_text("id: X1\ncriteria: []\n", encoding="utf-8")
    code, _, err = call(["validate", str(bad)])
    assert code == EXIT_FAIL
    assert f"{bad}: " in err


@pytest.mark.parametrize(
    ("text", "message"),
    [("just: notes\n", "not a design, test case"), ("[1, 2]\n", "not a design"), ("a: [\n", "YAML parse error")],
)
def test_validate_an_unknown_or_unreadable_file_is_an_error(tmp_path: Path, text: str, message: str) -> None:
    path = tmp_path / "other.yaml"
    path.write_text(text, encoding="utf-8")
    code, _, err = call(["validate", str(path)])
    assert code == EXIT_ERROR
    assert message in err


def test_validate_a_missing_path_is_an_error(tmp_path: Path) -> None:
    code, _, err = call(["validate", str(tmp_path / "none.yaml")])
    assert code == EXIT_ERROR
    assert "no such file or folder" in err


def test_a_usage_error_exits_2_and_help_exits_0(capsys: pytest.CaptureFixture[str]) -> None:
    assert call(["frobnicate"])[0] == EXIT_ERROR
    assert call([])[0] == EXIT_ERROR
    assert call(["--help"])[0] == EXIT_PASS


# ===== run =============================================================


@pytest.mark.req("TST-031")
def test_run_runs_one_case_and_writes_its_archive(tmp_path: Path) -> None:
    services = cli_services(tmp_path)
    case = write_case(tmp_path / "cases", MACHINE_CASE)
    code, out, _ = call(["run", str(case), "--set", "link_mass=1 kg", "--out", str(tmp_path / "runs")], services)
    assert code == EXIT_PASS
    assert (tmp_path / "runs" / "M1-seed7.h5").is_file()
    assert f"archive: {tmp_path / 'runs' / 'M1-seed7.h5'}" in out
    assert "Case M1: PASS" in out


@pytest.mark.parametrize(
    ("angle", "error_tick", "bindings", "code"),
    [
        (0.07, None, ["link_mass=1 kg"], EXIT_FAIL),
        (0.05, 3, ["link_mass=1 kg"], EXIT_ERROR),
        (0.05, None, [], EXIT_ERROR),
    ],
)
def test_run_exit_codes_for_fail_run_error_and_unresolved(
    tmp_path: Path, angle: float, error_tick: int | None, bindings: list[str], code: int
) -> None:
    services = cli_services(tmp_path, angle, error_tick)
    case = write_case(tmp_path / "cases", MACHINE_CASE)
    arguments = ["run", str(case), "--out", str(tmp_path / "runs")]
    for binding in bindings:
        arguments.extend(["--set", binding])
    assert call(arguments, services)[0] == code


def test_run_dry_schedules_without_running(tmp_path: Path) -> None:
    services = cli_services(tmp_path)
    case = write_case(tmp_path / "cases", MACHINE_CASE)
    code, out, _ = call(["run", str(case), "--set", "link_mass=1 kg", "--dry", "--out", str(tmp_path / "r")], services)
    assert code == EXIT_PASS
    assert out == "dry run M1: base rate 1000 Hz, 2 commands, 0 faults scheduled\n"
    assert not (tmp_path / "r").exists()


def test_run_with_a_malformed_binding_or_no_driver_is_an_error(tmp_path: Path) -> None:
    case = write_case(tmp_path / "cases", MACHINE_CASE)
    code, _, err = call(["run", str(case), "--set", "link_mass"], cli_services(tmp_path))
    assert (code, err) == (EXIT_ERROR, "error: --set takes name=value, got 'link_mass'\n")
    code, _, err = call(["run", str(case)], CliServices())
    assert (code, err) == (EXIT_ERROR, "error: no simulation driver is wired into this build\n")


# ===== suite ===========================================================


@pytest.mark.req("TST-032")
def test_suite_runs_in_batch_and_writes_its_report(tmp_path: Path) -> None:
    services = cli_services(tmp_path)
    suite = tmp_path / "demo"
    write_case(suite, MACHINE_CASE)
    write_case(suite, MACHINE_CASE.replace("id: M1", "id: M2"), "M2.yaml")
    out_dir = tmp_path / "runs"
    baseline = tmp_path / "next_baseline.yaml"
    code, out, _ = call(
        ["suite", str(suite), "--set", "link_mass=1 kg", "--out", str(out_dir), "--write-baseline", str(baseline)],
        services,
    )
    assert code == EXIT_PASS
    report = (out_dir / "demo_report.txt").read_text(encoding="utf-8")
    assert report.startswith("Suite demo (machine 0.1.0): PASS\n2 runs: 2 passed, 0 failed, 0 errored\n")
    assert report in out
    code, out, _ = call(["validate", str(baseline)])
    assert code == EXIT_PASS


def test_suite_prints_the_regression_diff_and_fails_on_a_failing_case(tmp_path: Path) -> None:
    services = cli_services(tmp_path, angle=0.07)
    suite = tmp_path / "demo"
    write_case(suite, MACHINE_CASE + "baseline: baseline.yaml\n")
    (suite / "baseline.yaml").write_text(
        unrun_baseline("demo", ["M1"]),
        encoding="utf-8",
    )
    code, out, _ = call(["suite", str(suite), "--set", "link_mass=1 kg", "--out", str(tmp_path / "runs")], services)
    assert code == EXIT_FAIL
    assert "Regression diff against the baseline:\n  unchanged: M1/final_angle (not_run -> fail)\n" in out


@pytest.mark.req("SCO-009", "SCO-010", "SCO-011", "SCO-012", "TST-008")
def test_suite_file_run_prints_the_diff_and_writes_the_report_and_next_baseline(tmp_path: Path) -> None:
    services = cli_services(tmp_path)
    suite = tmp_path / "demo"
    write_case(suite, MACHINE_CASE)
    write_case(suite, MACHINE_CASE.replace("id: M1", "id: M2"), "M2.yaml")
    (suite / "baseline.yaml").write_text(unrun_baseline("nightly", ["M1", "M3"]), encoding="utf-8")
    (suite / "suite.yaml").write_text(
        "name: nightly\ndesign:\n  name: machine\n  version: 0.1.0\ncases: [M1.yaml, M2.yaml]\n"
        "baseline: baseline.yaml\n",
        encoding="utf-8",
    )
    code, _, err = call(["validate", str(suite / "suite.yaml")])
    assert (code, err) == (EXIT_PASS, "")
    report = tmp_path / "report.md"
    baseline = tmp_path / "next.yaml"
    argv = ["suite", str(suite / "suite.yaml"), "--set", "link_mass=1 kg", "--out", str(tmp_path / "runs")]
    argv += ["--report", str(report), "--write-baseline", str(baseline)]
    code, out, _ = call(argv, services)
    assert code == EXIT_PASS
    assert (
        "Regression diff against the baseline:\n"
        "  newly passing: M1/final_angle (not_run -> pass)\n"
        "  missing: M3/final_angle (not_run -> None)\n"
        "  added: M2/final_angle (None -> pass)\n"
    ) in out
    text = report.read_text(encoding="utf-8")
    assert text.startswith("# Suite nightly: PASS\n")
    assert "### Newly passing (1)\n\n- M1 / final_angle: not_run -> pass\n" in text
    first = baseline.read_text(encoding="utf-8")
    code, _, _ = call(["validate", str(baseline)])
    assert code == EXIT_PASS
    call(argv, services)
    assert baseline.read_text(encoding="utf-8") == first
    assert report.read_text(encoding="utf-8") == text


def test_suite_with_no_cases_is_an_error(tmp_path: Path) -> None:
    (tmp_path / "empty").mkdir()
    code, _, err = call(["suite", str(tmp_path / "empty"), "--out", str(tmp_path / "runs")], cli_services(tmp_path))
    assert (code, err) == (EXIT_ERROR, "suite empty: no test case files\n")


# ===== replay ==========================================================


@pytest.mark.req("TST-033")
def test_replay_prints_the_event_timeline_from_the_archive_alone(tmp_path: Path) -> None:
    services = cli_services(tmp_path)
    case = write_case(tmp_path / "cases", MACHINE_CASE)
    call(["run", str(case), "--set", "link_mass=1 kg", "--out", str(tmp_path / "runs")], services)
    no_engine = CliServices(None, FakeArchiveReader())
    code, out, _ = call(["replay", str(tmp_path / "runs" / "M1-seed7.h5")], no_engine)
    assert code == EXIT_PASS
    assert out == (
        "Run M1-seed7: case M1, design machine 0.1.0, seed 7, controller mode native\n"
        "    0.002000 s  operator  operator_message  home\n"
        "    0.003000 s  operator  operator_message  target\n"
        "ended: end_time at 0.01 s\n"
    )


def test_replay_of_a_missing_archive_or_without_a_reader_is_an_error(tmp_path: Path) -> None:
    code, _, err = call(["replay", str(tmp_path / "none.h5")], CliServices(None, FakeArchiveReader()))
    assert (code, err) == (EXIT_ERROR, f"error: {tmp_path / 'none.h5'}: no such archive\n")
    assert call(["replay", str(tmp_path / "none.h5")])[0] == EXIT_ERROR


# ===== serve ===========================================================


SERVE_URL = r"http://127\.0\.0\.1:(\d+)/\?token=([A-Za-z0-9_-]+)\n"


@pytest.mark.req("VIE-012")
def test_serve_check_prints_the_page_url_and_exits_0(tmp_path: Path) -> None:
    """`serve --check` starts the facade with the browser client on 127.0.0.1, prints the one URL line and stops."""
    services = CliServices(files_root=tmp_path)
    code, out, err = call(
        ["serve", "--check", "--files-root", str(tmp_path), "--runs", str(tmp_path / "runs")], services
    )
    assert (code, err) == (EXIT_PASS, "")
    match = re.fullmatch(SERVE_URL, out)
    assert match is not None, out
    assert int(match.group(1)) > 0
    assert len(match.group(2)) >= 32
    again = call(["serve", "--check"], services)[1]
    assert re.fullmatch(SERVE_URL, again) is not None
    assert again != out


@pytest.mark.req("VIE-012")
def test_serve_on_a_port_in_use_exits_2(tmp_path: Path) -> None:
    with socket.socket() as taken:
        taken.bind(("127.0.0.1", 0))
        taken.listen()
        port = taken.getsockname()[1]
        code, out, err = call(["serve", "--check", "--port", str(port)], CliServices(files_root=tmp_path))
    assert (code, out) == (EXIT_ERROR, "")
    assert err.startswith("error: ")


@pytest.mark.req("VIE-012")
def test_serve_rejects_a_bad_port_as_a_usage_error(tmp_path: Path) -> None:
    assert call(["serve", "--port", "http"], CliServices(files_root=tmp_path))[0] == EXIT_ERROR


@pytest.mark.req("VIE-012")
def test_serve_check_runs_from_the_console_entry() -> None:
    result = subprocess.run(
        [sys.executable, "-m", "mechsim.cli", "serve", "--check"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
        timeout=60,
    )
    assert result.returncode == EXIT_PASS, result.stderr
    assert re.fullmatch(SERVE_URL, result.stdout) is not None


# ===== headless ========================================================


@pytest.mark.req("TST-035")
def test_the_cli_runs_with_no_display_and_loads_no_display_module() -> None:
    environment = {key: value for key, value in os.environ.items() if key not in ("DISPLAY", "WAYLAND_DISPLAY")}
    script = (
        "import sys\nfrom mechsim.cli import main\n"
        "code = main(['validate', 'designs/pendulum/design.yaml'])\n"
        "display = ('PySide6', 'PySide2', 'PyQt5', 'PyQt6', 'tkinter', 'matplotlib', 'OpenGL', 'pyqtgraph')\n"
        "print(code, sorted(name for name in sys.modules if name.split('.')[0] in display))\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", script], cwd=ROOT, env=environment, capture_output=True, text=True, check=False
    )
    assert result.stdout.splitlines()[-1] == "0 []"
    module = subprocess.run(
        [sys.executable, "-m", "mechsim.cli", "validate", "designs/pendulum/design.yaml"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert module.returncode == EXIT_PASS
