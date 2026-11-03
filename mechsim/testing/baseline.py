"""Regression baselines (SCO, ADR 0012 D4). Generates `schemas/baseline.json`.

A baseline is a suite's scoring summary as YAML in git: one entry per case and criterion with its status, measured
value, limit, time of the worst case and the identity of the run archive that produced it, never the archive itself.
Entries are sorted by case id (natural order, G3 before G10) then criterion name.
"""

from __future__ import annotations

import argparse
import io
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Annotated, Any, Literal

from pydantic import ConfigDict, Field, ValidationError, model_validator

from mechsim.design.schema import Name, _format_errors, _Strict, yaml_loader
from mechsim.testing.schema import ROOT, Case, CaseId, DesignRef, Measured
from mechsim.units import Time

SCHEMA_PATH = ROOT / "schemas" / "baseline.json"

Status = Literal["pass", "fail", "not_run", "error"]
GitHash = Annotated[str, Field(pattern=r"^[0-9a-f]{40}$")]
_CASE_ID = re.compile(r"^(.*?)([0-9]+)$")


class BaselineError(ValueError):
    """A baseline file failed to load; the message names the file, the path within it and the reason."""


class RunIdentity(_Strict):
    """Which archive produced an entry: its run id and hashes (L8, D3)."""

    run_id: Annotated[str, Field(pattern=r"^[A-Za-z0-9][A-Za-z0-9_.\-]*$")]
    archive_sha256: Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]
    design_hash: GitHash
    case_hash: GitHash


class Entry(_Strict):
    case: CaseId
    criterion: Name
    status: Status
    measured: Measured | None = None
    limit: str = Field(min_length=1, description="the criterion's limit as the case states it")
    worst_time: Time | None = None
    run: RunIdentity | None = None

    @model_validator(mode="after")
    def _status_fields(self) -> Entry:
        if self.status in ("pass", "fail"):
            if self.measured is None or self.run is None:
                raise ValueError(f"a {self.status} entry needs measured and run")
        elif self.measured is not None or self.worst_time is not None:
            raise ValueError(f"a {self.status} entry takes no measured or worst_time")
        if self.status == "error" and self.run is None:
            raise ValueError("an error entry needs the run that errored")
        if self.status == "not_run" and self.run is not None:
            raise ValueError("a not_run entry takes no run")
        if self.worst_time is not None and self.worst_time < 0.0:
            raise ValueError(f"worst_time must not be negative, got {self.worst_time}")
        return self


def entry_key(entry: Entry) -> tuple[str, int, str]:
    match = _CASE_ID.match(entry.case)
    if match is None:
        return entry.case, 0, entry.criterion
    return match.group(1), int(match.group(2)), entry.criterion


class Baseline(_Strict):
    """Root of a suite's `baseline.yaml`."""

    model_config = ConfigDict(extra="forbid", strict=True, title="BaselineFile")

    design: DesignRef
    suite: Name
    entries: list[Entry] = Field(min_length=1)

    @model_validator(mode="after")
    def _sorted_and_unique(self) -> Baseline:
        errors: list[str] = []
        for index in range(1, len(self.entries)):
            previous = entry_key(self.entries[index - 1])
            current = entry_key(self.entries[index])
            if current == previous:
                errors.append(f"entries[{index}]: duplicate entry {self.entries[index].case}/{current[2]}")
            elif current < previous:
                errors.append(f"entries[{index}]: entries must be sorted by case then criterion")
        if len(errors) > 0:
            raise ValueError("\n".join(errors))
        return self


def sorted_baseline(baseline: Baseline) -> Baseline:
    return baseline.model_copy(update={"entries": sorted(baseline.entries, key=entry_key)})


def baseline_text(baseline: Baseline) -> str:
    """The deterministic YAML form scoring writes (SCO-011): sorted entries, unset fields left out."""
    data = sorted_baseline(baseline).model_dump(mode="json", exclude_none=True)
    stream = io.StringIO()
    yaml_loader().dump(data, stream)
    return stream.getvalue()


# ===== Diff ============================================================


@dataclass(frozen=True)
class Change:
    case: str
    criterion: str
    old: Status | None
    new: Status | None


@dataclass(frozen=True)
class BaselineDiff:
    """Newly passing: passes now, did not before. Newly failing: passed before, does not now (fail, error or not
    run). Unchanged: in both otherwise. Missing: only in the old baseline. Added: only in the new one (SCO-010)."""

    newly_passing: tuple[Change, ...]
    newly_failing: tuple[Change, ...]
    unchanged: tuple[Change, ...]
    missing: tuple[Change, ...]
    added: tuple[Change, ...]


def diff_baselines(old: Baseline, new: Baseline) -> BaselineDiff:
    """Diff a suite's new results against its baseline (SCO-009, SCO-010); both must be for the same suite."""
    if old.suite != new.suite:
        raise ValueError(f"cannot diff suite {new.suite!r} against a baseline of suite {old.suite!r}")
    before = {entry_key(entry): entry for entry in old.entries}
    after = {entry_key(entry): entry for entry in new.entries}
    passing: list[Change] = []
    failing: list[Change] = []
    unchanged: list[Change] = []
    missing: list[Change] = []
    added: list[Change] = []
    for key in sorted(set(before) | set(after)):
        old_entry = before.get(key)
        new_entry = after.get(key)
        if new_entry is None and old_entry is not None:
            missing.append(Change(old_entry.case, old_entry.criterion, old_entry.status, None))
        elif old_entry is None and new_entry is not None:
            added.append(Change(new_entry.case, new_entry.criterion, None, new_entry.status))
        elif old_entry is not None and new_entry is not None:
            change = Change(new_entry.case, new_entry.criterion, old_entry.status, new_entry.status)
            if new_entry.status == "pass" and old_entry.status != "pass":
                passing.append(change)
            elif old_entry.status == "pass" and new_entry.status != "pass":
                failing.append(change)
            else:
                unchanged.append(change)
    return BaselineDiff(tuple(passing), tuple(failing), tuple(unchanged), tuple(missing), tuple(added))


# ===== Files and schema ================================================


def check_against_cases(baseline: Baseline, cases: list[Case]) -> list[str]:
    """Every entry names a case of the suite and one of its criteria, with that criterion's limit text."""
    errors: list[str] = []
    by_id = {case.id: case for case in cases}
    for index, entry in enumerate(baseline.entries):
        case = by_id.get(entry.case)
        if case is None:
            errors.append(f"entries[{index}].case: no case {entry.case!r} in the suite")
            continue
        if case.design != baseline.design:
            errors.append(f"entries[{index}].case: {entry.case} is written against another design version")
        criteria = {criterion.name: criterion for criterion in case.criteria}
        criterion = criteria.get(entry.criterion)
        if criterion is None:
            errors.append(f"entries[{index}].criterion: {entry.case} has no criterion {entry.criterion!r}")
        elif criterion.limit_text() != entry.limit:
            errors.append(f"entries[{index}].limit: {entry.limit!r} differs from the case's {criterion.limit_text()!r}")
    return errors


def _load(path: Path) -> tuple[Baseline | None, list[str]]:
    try:
        data = yaml_loader().load(path.read_text(encoding="utf-8"))
    except Exception as error:
        return None, [f"{path}: YAML parse error: {error}"]
    try:
        baseline = Baseline.model_validate(data)
    except ValidationError as error:
        return None, [f"{path}: {message}" for message in _format_errors(error)]
    return baseline, []


def validate_baseline_file(path: Path) -> list[str]:
    return _load(path)[1]


def load_baseline(path: Path) -> Baseline:
    baseline, errors = _load(path)
    if baseline is None:
        raise BaselineError("\n".join(errors))
    return baseline


def baseline_json_schema() -> dict[str, Any]:
    schema = Baseline.model_json_schema()
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["$id"] = SCHEMA_PATH.name
    return schema


def schema_text() -> str:
    return json.dumps(baseline_json_schema(), indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Regression baseline schema")
    parser.add_argument("--write-schema", action="store_true", help=f"write {SCHEMA_PATH}")
    parser.add_argument("--validate", type=Path, nargs="*", default=[], help="baseline files to validate")
    arguments = parser.parse_args(argv)
    if arguments.write_schema:
        SCHEMA_PATH.write_text(schema_text(), encoding="utf-8")
        return 0
    if len(arguments.validate) > 0:
        status = 0
        for path in arguments.validate:
            for message in validate_baseline_file(path):
                sys.stderr.write(message + "\n")
                status = 1
        return status
    sys.stdout.write(schema_text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
