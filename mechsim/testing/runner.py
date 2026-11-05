"""The test runner (TST): load a case and its design, resolve `${}` values, check the controller precondition, build
the command and fault timelines in base ticks, run through a `SimulationDriver`, archive, and hand every run to scoring.

Timing rule (TST open parameter, ruled form): a scheduled time is taken to 12 significant digits, which removes the
float error of unit conversion from the decimal the file states, and an operator command or a fault edge between base
ticks is delivered at the next base tick, the first tick n with n / base_rate >= t. A fault is active for
start_tick <= n < end_tick; a window that rounds to no tick is an error. The end time and `max_duration` must fall on a
base tick, as the kernel requires (KER, invariant 7).
"""

from __future__ import annotations

import json
import math
import re
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np
from pydantic import ValidationError

from mechsim.design.schema import (
    CHOICE,
    Design,
    DesignError,
    OpenParameter,
    blob_hash,
    iter_open_values,
    open_name,
    yaml_loader,
)
from mechsim.schema_base import format_errors
from mechsim.scoring.score import (
    CaseScore,
    ScoringError,
    SuiteScore,
    not_run,
    run_failed,
    score_run,
    suite_baseline,
)
from mechsim.testing.baseline import Baseline, BaselineDiff, RunIdentity, diff_baselines, load_baseline
from mechsim.testing.driver import (
    Archiver,
    Builder,
    BuildResult,
    EndSpec,
    FaultOwner,
    RunConfig,
    RunMetadata,
    RunReport,
    ScheduledCommand,
    ScheduledFault,
    SimulationDriver,
)
from mechsim.testing.schema import (
    DESIGNS_ROOT,
    Case,
    DesignRef,
    Quantity,
    cross_check,
)
from mechsim.units import UnitError, parse_quantity

DEFAULT_PHYSICS_STEP = 0.001
SIGNIFICANT_DIGITS = 12
FAULT_OWNERS: dict[str, FaultOwner] = {
    "sensor_stuck": "signals",
    "sensor_noisy": "signals",
    "sensor_dropped": "signals",
    "sensor_forced": "signals",
    "motor_stall": "signals",
    "power_loss": "signals",
    "estop_press": "signals",
    "slip": "physics",
    "payload_change": "physics",
    "controller_freeze": "controller",
}
_SAFE = re.compile(r"[^A-Za-z0-9_.\-]+")

DriverFactory = Callable[[], SimulationDriver]


class RunnerError(ValueError):
    """A case cannot run: invalid file, design mismatch, unresolved value, precondition mismatch, off-tick time."""


# ===== Timing ==========================================================


def seconds_fraction(seconds: float) -> Fraction:
    return Fraction(f"{seconds:.{SIGNIFICANT_DIGITS}g}")


def rate_fraction(hertz: float) -> Fraction:
    return Fraction(f"{hertz:.{SIGNIFICANT_DIGITS}g}")


def rational_lcm(first: Fraction, second: Fraction) -> Fraction:
    """lcm of two positive rationals p/q, r/s in lowest terms: lcm(p, r) / gcd(q, s)."""
    return Fraction(math.lcm(first.numerator, second.numerator), math.gcd(first.denominator, second.denominator))


def next_tick(seconds: float, base_rate: Fraction) -> int:
    """The first base tick at or after `seconds` (the delivery rule for a time between ticks)."""
    return math.ceil(seconds_fraction(seconds) * base_rate)


def exact_tick(seconds: float, base_rate: Fraction, what: str) -> int:
    ticks = seconds_fraction(seconds) * base_rate
    if ticks.denominator != 1:
        raise RunnerError(f"{what}: {seconds:g} s is not on the base tick of 1/{base_rate} s")
    return ticks.numerator


# ===== Loading and resolution ==========================================


@dataclass(frozen=True)
class Prepared:
    """A case ready to run: the case as written, the resolved case and design, the limit texts as written."""

    case_path: Path
    design_path: Path
    written: Case
    case: Case
    design: Design
    limits: dict[str, str]
    bindings: tuple[tuple[str, str], ...]


def _plain(path: Path, what: str) -> Any:
    try:
        data = yaml_loader().load(path.read_text(encoding="utf-8"))
    except Exception as error:
        raise RunnerError(f"{path}: cannot read {what}: {error}") from error
    return json.loads(json.dumps(data))


def _validate_case(data: Any, path: Path) -> Case:
    try:
        return Case.model_validate(data)
    except ValidationError as error:
        raise RunnerError("\n".join(f"{path}: {message}" for message in format_errors(error))) from error


def _validate_design(data: Any, path: Path) -> Design:
    try:
        return Design.model_validate(data, context={"base": path.parent})
    except ValidationError as error:
        raise RunnerError("\n".join(f"{path}: {message}" for message in format_errors(error))) from error


def _substitute(data: Any, values: Mapping[str, Any], skip: frozenset[str]) -> Any:
    if isinstance(data, dict):
        out: dict[str, Any] = {}
        for key, item in data.items():
            if key in skip:
                out[key] = item
            else:
                out[key] = _substitute(item, values, frozenset())
        return out
    if isinstance(data, list):
        return [_substitute(item, values, frozenset()) for item in data]
    if isinstance(data, str):
        name = open_name(data)
        if name is not None and name in values:
            return values[name]
    return data


def set_design_path(data: Any, parameter: str, value: str) -> Any:
    """Set `<bodies|joints|components>.<item>.<field>[.<field>]` in a design's plain data (sweep addressing)."""
    section, item, *fields = parameter.split(".")
    entries = data.get(section)
    if not isinstance(entries, list):
        raise RunnerError(f"sweep {parameter}: the design has no {section}")
    target: Any = None
    for entry in entries:
        if isinstance(entry, dict) and entry.get("name") == item:
            target = entry
    if target is None:
        raise RunnerError(f"sweep {parameter}: no {section} item {item!r}")
    for name in fields[:-1]:
        if not isinstance(target, dict) or not isinstance(target.get(name), dict):
            raise RunnerError(f"sweep {parameter}: no field {name!r}")
        target = target[name]
    if not isinstance(target, dict) or fields[-1] not in target:
        raise RunnerError(f"sweep {parameter}: no field {fields[-1]!r}")
    target[fields[-1]] = value
    return data


def resolve_open_values(
    declared: Mapping[str, OpenParameter], case_values: Mapping[str, str], given: Mapping[str, str]
) -> dict[str, str | dict[str, str]]:
    """Each declared open parameter's value, in order (DES-044): `--set`, the case's `parameters`, the parameter's
    `default`. A parameter with none of the three stays unresolved and is reported where it is referenced."""
    values: dict[str, str | dict[str, str]] = {}
    for name in sorted(declared):
        default = declared[name].default
        if name in given:
            values[name] = given[name]
        elif name in case_values:
            values[name] = case_values[name]
        elif default is not None:
            values[name] = default
    return values


def _binding_text(value: str | dict[str, str]) -> str:
    if isinstance(value, str):
        return value
    return json.dumps(value, sort_keys=True)


def prepare(
    case_path: Path,
    bindings: Mapping[str, str] | None = None,
    designs_root: Path = DESIGNS_ROOT,
    design_values: Mapping[str, str] | None = None,
) -> Prepared:
    """Load and validate the case (TST-009), check the design version (TST-011) and the case against the design,
    then resolve every `${}` in both from `bindings`; anything left unresolved is an error."""
    given: dict[str, str] = dict(bindings or {})
    raw_case = _plain(case_path, "test case")
    written = _validate_case(raw_case, case_path)
    design_path = designs_root / written.design.name / "design.yaml"
    if not design_path.is_file():
        raise RunnerError(f"{case_path}: design.name: no design file {design_path}")
    raw_design = _plain(design_path, "design")
    try:
        design_written = _validate_design(raw_design, design_path)
    except RunnerError as error:
        raise RunnerError(f"{case_path}: design: the design does not load: {error}") from error
    if written.design.version != design_written.version:
        raise RunnerError(
            f"{case_path}: design.version: case was written against {written.design.version}, "
            f"design is {design_written.version}"
        )
    errors = cross_check(written, design_written)
    if len(errors) > 0:
        raise RunnerError("\n".join(f"{case_path}: {message}" for message in errors))
    declared = {parameter.name: parameter for parameter in design_written.open_parameters}
    for parameter in written.open_parameters:
        declared[parameter.name] = parameter
    problems: list[str] = []
    for source, values in (("parameters", written.parameters), ("binding", given)):
        for name in sorted(values):
            if name not in declared:
                problems.append(f"{source} {name!r}: not an open parameter of the design or the case")
                continue
            parameter = declared[name]
            if parameter.dimension != CHOICE:
                try:
                    parse_quantity(values[name], parameter.dimension)
                except UnitError as error:
                    problems.append(f"{source} {name!r}: {error}")
    if len(problems) > 0:
        raise RunnerError("\n".join(f"{case_path}: {message}" for message in problems))
    resolved = resolve_open_values(declared, written.parameters, given)
    given = {name: _binding_text(value) for name, value in resolved.items()}
    for path, value in (design_values or {}).items():
        set_design_path(raw_design, path, value)
    resolved_design_data = _substitute(raw_design, resolved, frozenset({"open_parameters"}))
    resolved_case_data = _substitute(
        raw_case, resolved, frozenset({"open_parameters", "parameters", "sweeps", "variations"})
    )
    resolved_case_data["open_parameters"] = []
    resolved_case_data["parameters"] = {}
    design = _validate_design(resolved_design_data, design_path)
    case = _validate_case(resolved_case_data, case_path)
    unresolved = [
        f"{path}: unresolved {value}"
        for path, value in [*iter_open_values(design, "design"), *iter_open_values(case, "case")]
    ]
    if len(unresolved) > 0:
        raise RunnerError("\n".join(f"{case_path}: {message}" for message in unresolved))
    errors = cross_check(case, design)
    if len(errors) > 0:
        raise RunnerError("\n".join(f"{case_path}: {message}" for message in errors))
    limits = {criterion.name: criterion.limit_text() for criterion in written.criteria}
    return Prepared(case_path, design_path, written, case, design, limits, tuple(sorted(given.items())))


# ===== Timelines =======================================================


def _seconds(value: Any, what: str) -> float:
    if isinstance(value, float):
        return value
    raise RunnerError(f"{what}: unresolved {value}")


def _si(value: Any, what: str) -> float:
    if isinstance(value, Quantity):
        return value.value
    if isinstance(value, float):
        return value
    raise RunnerError(f"{what}: unresolved {value}")


def run_config(case: Case, design: Design, build: BuildResult, dry: bool = False) -> RunConfig:
    """Rates from the case, else the design (firmware tick) and the 1 ms default step; seeds per D1 (TST-028)."""
    step = DEFAULT_PHYSICS_STEP
    control: Fraction | None = None
    if design.firmware is not None:
        control = rate_fraction(_seconds(design.firmware.tick_rate, "firmware.tick_rate"))
    if case.rates is not None:
        if case.rates.physics_step is not None:
            step = _seconds(case.rates.physics_step, "rates.physics_step")
        if case.rates.control_rate is not None:
            control = rate_fraction(_seconds(case.rates.control_rate, "rates.control_rate"))
    physics = 1 / seconds_fraction(step)
    base = physics
    if control is not None:
        base = rational_lcm(physics, control)
    return RunConfig(
        physics_rate=physics,
        control_rate=control,
        base_rate=base,
        seed=case.seed,
        module_seeds=module_seeds(case.seed, build.module_instances),
        controller_mode=case.controller_mode,
        dry=dry,
    )


def module_seeds(seed: int, instances: Sequence[str]) -> tuple[tuple[str, int], ...]:
    """One child of SeedSequence(seed) per module instance, spawned in registration order (D1, TST-028)."""
    if len(set(instances)) != len(instances):
        raise RunnerError("module instances must be unique to seed them")
    children = np.random.SeedSequence(seed).spawn(len(instances))
    return tuple(
        (name, int(child.generate_state(1, dtype=np.uint64)[0]))
        for name, child in zip(instances, children, strict=True)
    )


def command_timeline(case: Case, base_rate: Fraction) -> list[ScheduledCommand]:
    commands: list[ScheduledCommand] = []
    for index, command in enumerate(case.commands):
        at = _seconds(command.at, f"commands[{index}].at")
        payload = tuple(
            (name, _si(value, f"commands[{index}].payload.{name}")) for name, value in command.payload.items()
        )
        commands.append(ScheduledCommand(next_tick(at, base_rate), command.message, payload))
    return commands


def fault_timeline(case: Case, base_rate: Fraction) -> list[ScheduledFault]:
    """Each fault routed to the module that owns the thing it breaks (TST-014 to TST-020, TST-037)."""
    faults: list[ScheduledFault] = []
    for index, fault in enumerate(case.faults):
        path = f"faults[{index}]"
        start = _seconds(fault.start, f"{path}.start")
        start_tick = next_tick(start, base_rate)
        end_tick: int | None = None
        parameters: list[tuple[str, float | str]] = []
        target: str | None = None
        if fault.kind == "payload_change":
            target = fault.target
            parameters.append(("mass", _si(fault.mass, f"{path}.mass")))
        else:
            if fault.end is not None:
                end = seconds_fraction(_seconds(fault.end, f"{path}.end"))
            else:
                end = seconds_fraction(start) + seconds_fraction(_seconds(fault.duration, f"{path}.duration"))
            end_tick = math.ceil(end * base_rate)
            if end_tick <= start_tick:
                raise RunnerError(f"{path}: the fault window covers no base tick of 1/{base_rate} s")
            if fault.kind == "power_loss":
                target = fault.rail
            elif fault.kind != "controller_freeze":
                target = fault.target
            if fault.kind == "sensor_noisy":
                parameters.append(("standard_deviation", _si(fault.standard_deviation, f"{path}.standard_deviation")))
            elif fault.kind == "sensor_forced":
                parameters.append(("level", fault.level))
            elif fault.kind == "slip":
                parameters.append(("fraction", _si(fault.fraction, f"{path}.fraction")))
            elif fault.kind == "motor_stall":
                parameters.append(("available_torque", _si(fault.available_torque, f"{path}.available_torque")))
        faults.append(
            ScheduledFault(FAULT_OWNERS[fault.kind], fault.kind, target, start_tick, end_tick, tuple(parameters))
        )
    return faults


def end_spec(case: Case, base_rate: Fraction) -> EndSpec:
    max_tick = exact_tick(_seconds(case.end.max_duration, "end.max_duration"), base_rate, "end.max_duration")
    at_tick: int | None = None
    if case.end.at is not None:
        at_tick = exact_tick(_seconds(case.end.at, "end.at"), base_rate, "end.at")
    return EndSpec(max_tick, at_tick, case.end.message, case.end.controller_state)


def check_precondition(case: Case, reported: Mapping[str, bool | str]) -> None:
    """`initial_state.controller` is a precondition on the firmware's reported state before t = 0, not a state the
    runner sets; any mismatch is an error (owner ruling 2026-11-04)."""
    problems: list[str] = []
    for name, expected in case.initial_state.controller.items():
        if name not in reported:
            problems.append(f"initial_state.controller.{name}: the firmware reports no {name!r}")
        elif reported[name] != expected or type(reported[name]) is not type(expected):
            problems.append(
                f"initial_state.controller.{name}: expected {expected!r}, firmware reports {reported[name]!r}"
            )
    if len(problems) > 0:
        raise RunnerError("controller precondition not met: " + "; ".join(problems))


# ===== Runs ============================================================


@dataclass(frozen=True)
class RunResult:
    score: CaseScore
    report: RunReport | None = None
    metadata: RunMetadata | None = None
    archive: Path | None = None
    config: RunConfig | None = None
    commands: tuple[ScheduledCommand, ...] = ()
    faults: tuple[ScheduledFault, ...] = ()


@dataclass(frozen=True)
class Services:
    """What a run needs from outside the runner: a fresh driver per run, the builder, the archive writer."""

    driver: DriverFactory
    builder: Builder
    archiver: Archiver | None = None
    designs_root: Path = DESIGNS_ROOT


def run_id(case: Case, sweep_index: tuple[int, int] | None) -> str:
    text = f"{case.id}-seed{case.seed}"
    if sweep_index is not None:
        text = f"{text}-sweep{sweep_index[0]}-{sweep_index[1]}"
    return _SAFE.sub("_", text)


def run_case(
    case_path: Path,
    services: Services,
    bindings: Mapping[str, str] | None = None,
    out_dir: Path | None = None,
    dry: bool = False,
    sweep: tuple[str, str] | None = None,
    sweep_index: tuple[int, int] | None = None,
) -> RunResult:
    """One run, always scored (TST-023): a case stopped before t = 0 is scored as errored and not run; a run that
    ends in a run error is scored as errored with its end reason and time (TST-022)."""
    values = dict(bindings or {})
    design_values: dict[str, str] = {}
    if sweep is not None:
        name = open_name(sweep[0])
        if name is not None:
            values[name] = sweep[1]
        else:
            design_values[sweep[0]] = sweep[1]
    try:
        prepared = prepare(case_path, values, services.designs_root, design_values)
    except (RunnerError, DesignError) as error:
        return RunResult(_unrun(case_path, str(error), sweep))
    case = prepared.case
    try:
        build = services.builder(prepared.design)
        config = run_config(case, prepared.design, build, dry)
        commands = command_timeline(case, config.base_rate)
        faults = fault_timeline(case, config.base_rate)
        end = end_spec(case, config.base_rate)
        driver = services.driver()
        driver.load(build, case)
        driver.configure(config)
        driver.set_initial_state(case.initial_state)
        if prepared.design.firmware is not None:
            check_precondition(case, driver.reported_controller_state())
        for command in commands:
            driver.schedule_command(command)
        for fault in faults:
            driver.schedule_fault(fault)
    except RunnerError as error:
        return RunResult(not_run(case, case.id, case.design, case.seed, str(error), sweep))
    if dry:
        score = not_run(case, case.id, case.design, case.seed, "dry run: configured, not run", sweep)
        return RunResult(score, config=config, commands=tuple(commands), faults=tuple(faults))
    try:
        report = driver.run(end)
        recording = driver.recording()
    except RunnerError as error:
        return RunResult(
            run_failed(case, str(error), sweep), config=config, commands=tuple(commands), faults=tuple(faults)
        )
    identifier = run_id(case, sweep_index)
    metadata = RunMetadata(
        run_id=identifier,
        case_id=case.id,
        design_name=case.design.name,
        design_version=case.design.version,
        design_hash=blob_hash(prepared.design_path),
        case_hash=blob_hash(case_path),
        seed=case.seed,
        controller_mode=case.controller_mode,
        sweep=sweep,
        report=report,
        bindings=prepared.bindings,
        control_rate=config.control_rate,
    )
    archive: Path | None = None
    identity: RunIdentity | None = None
    if services.archiver is not None and out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        archive = out_dir / f"{identifier}.h5"
        digest = services.archiver.write(archive, recording, metadata)
        identity = RunIdentity(
            run_id=identifier,
            archive_sha256=digest,
            design_hash=metadata.design_hash,
            case_hash=metadata.case_hash,
        )
    score = score_run(case, recording, report, prepared.limits, sweep, identity)
    return RunResult(score, report, metadata, archive, config, tuple(commands), tuple(faults))


def _unrun(case_path: Path, error: str, sweep: tuple[str, str] | None) -> CaseScore:
    """Score a case that did not load: the identity it states, when it parses at all."""
    try:
        written = Case.model_validate(_plain(case_path, "test case"))
    except (RunnerError, ValidationError):
        return not_run(
            None, case_path.stem.partition("_")[0], DesignRef(name="unknown", version="0.0.0"), 0, error, sweep
        )
    return not_run(written, written.id, written.design, written.seed, error, sweep)


def sweep_runs(
    case_path: Path, services: Services, bindings: Mapping[str, str] | None, out_dir: Path | None
) -> list[RunResult]:
    """One run per value of each sweep, each recorded with the value it used (TST-025); sweeps are separate, not a
    product."""
    try:
        written = Case.model_validate(_plain(case_path, "test case"))
    except (RunnerError, ValidationError):
        return []
    results: list[RunResult] = []
    for index, sweep in enumerate(written.sweeps):
        for position, value in enumerate(sweep.values):
            results.append(
                run_case(
                    case_path,
                    services,
                    bindings,
                    out_dir,
                    sweep=(sweep.parameter, value),
                    sweep_index=(index, position),
                )
            )
    return results


# ===== Suites ==========================================================


@dataclass(frozen=True)
class SuiteRun:
    score: SuiteScore
    runs: tuple[RunResult, ...]
    baseline_path: Path | None = None
    baseline: Baseline | None = None
    diff: BaselineDiff | None = None
    errors: tuple[str, ...] = field(default_factory=tuple)


def case_files(directory: Path) -> list[Path]:
    """The test case files of a suite folder: YAML mappings with `criteria`, in natural case-id order."""
    found: list[tuple[tuple[str, int, str], Path]] = []
    for path in sorted(directory.glob("*.yaml")):
        try:
            data = yaml_loader().load(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        if isinstance(data, dict) and "criteria" in data:
            match = re.match(r"^(.*?)([0-9]+)", path.stem)
            key = (path.stem, 0, path.name)
            if match is not None:
                key = (match.group(1), int(match.group(2)), path.name)
            found.append((key, path))
    return [path for _, path in sorted(found)]


def run_suite(
    directory: Path,
    services: Services,
    bindings: Mapping[str, str] | None = None,
    out_dir: Path | None = None,
) -> SuiteRun:
    """Every case of the suite in one batch (TST-024), plus each case's sweep runs; the summary goes to scoring with
    the suite's baseline for the regression diff (TST-027)."""
    runs: list[RunResult] = []
    for path in case_files(directory):
        runs.append(run_case(path, services, bindings, out_dir))
        runs.extend(sweep_runs(path, services, bindings, out_dir))
    errors: list[str] = []
    designs = sorted({(run.score.design.name, run.score.design.version) for run in runs})
    design = DesignRef(name="unknown", version="0.0.0")
    if len(designs) == 1:
        design = DesignRef(name=designs[0][0], version=designs[0][1])
    elif len(designs) > 1:
        errors.append(f"suite {directory.name}: cases name more than one design: {designs}")
    else:
        errors.append(f"suite {directory.name}: no test case files")
    score = SuiteScore(directory.name, design, tuple(run.score for run in runs))
    baseline_path, baseline, diff = _baseline(directory, score, errors)
    return SuiteRun(score, tuple(runs), baseline_path, baseline, diff, tuple(errors))


def _baseline(
    directory: Path, score: SuiteScore, errors: list[str]
) -> tuple[Path | None, Baseline | None, BaselineDiff | None]:
    named: set[Path] = set()
    for path in case_files(directory):
        try:
            written = Case.model_validate(_plain(path, "test case"))
        except (RunnerError, ValidationError):
            continue
        if written.baseline is not None:
            named.add((path.parent / written.baseline).resolve())
    if len(named) == 0:
        return None, None, None
    if len(named) > 1:
        errors.append(f"suite {directory.name}: cases name more than one baseline")
        return None, None, None
    path = named.pop()
    old = load_baseline(path)
    try:
        new = suite_baseline(score)
    except ScoringError as error:
        errors.append(f"baseline: {error}")
        return path, None, None
    try:
        diff = diff_baselines(old, new)
    except ValueError as error:
        errors.append(f"baseline: {error}")
        return path, new, None
    return path, new, diff
