"""Facade protocol message models (ADR 0015, proposed; FAC spec).

Messages travel as JSON text frames; time series travel as a ``tick_batch`` header message followed by one binary
frame of little-endian float64 values. These models carry shape only: no domain logic.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, JsonValue, TypeAdapter, field_validator, model_validator

PROTOCOL_VERSION = 1
FLOAT64_SIZE = 8
SCHEMA_PATH = Path(__file__).resolve().parents[2] / "schemas" / "facade_protocol.json"
SCHEMA_ID = "https://mechsim.local/schemas/facade_protocol.json"

NonEmptyText = Annotated[str, Field(min_length=1)]
RunId = Annotated[str, Field(min_length=1, description="Run identifier assigned by the service.")]
SimTime = Annotated[
    float,
    Field(ge=0.0, allow_inf_nan=False, description="Simulated time in seconds."),
]
RequirementId = Annotated[str, Field(pattern=r"^[A-Z]{2,5}-\d{3}$")]
SeriesNames = Annotated[list[NonEmptyText], Field(min_length=1, description="Dotted series names, unique.")]

ErrorCode = Literal[
    "unsupported_version",
    "invalid_message",
    "unknown_kind",
    "validation_failed",
    "not_found",
    "busy",
    "run_error",
    "internal",
]
RunEndReason = Literal["end_condition", "solver_failure", "drift", "emulator_fault", "cancelled"]
TimelineCategory = Literal["command", "fault", "safety_event", "criterion_failure"]


def _require_unique(names: list[str]) -> list[str]:
    if len(set(names)) != len(names):
        raise ValueError("series names must be unique")
    return names


# ===== Base models ==========================================================


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)


class _Envelope(_Strict):
    version: int = Field(
        json_schema_extra={"const": PROTOCOL_VERSION},
        description="Protocol version; the service accepts only its own version.",
    )
    message_id: NonEmptyText

    @field_validator("version")
    @classmethod
    def _supported_version(cls, value: int) -> int:
        if value != PROTOCOL_VERSION:
            raise ValueError(f"unsupported protocol version {value}; this service speaks version {PROTOCOL_VERSION}")
        return value


# ===== Request payloads =====================================================


class ListDesignsPayload(_Strict):
    pass


class ValidateDesignPayload(_Strict):
    design: NonEmptyText = Field(description="Design file path relative to the repository root.")


class RunCasePayload(_Strict):
    design: NonEmptyText
    test_case: NonEmptyText
    seed: int = Field(ge=0)
    mode: NonEmptyText = Field(description="Controller mode name.")


class RunSuitePayload(_Strict):
    design: NonEmptyText
    suite: NonEmptyText
    seed: int = Field(ge=0)
    mode: NonEmptyText


class ReplayOpenPayload(_Strict):
    archive: NonEmptyText = Field(description="Run archive path.")


class SubscribeSeriesPayload(_Strict):
    run_id: RunId
    series: SeriesNames
    t_start: SimTime
    t_end: SimTime
    points: int = Field(ge=1, description="Points per series the service decimates to over the window.")

    _unique = field_validator("series")(_require_unique)

    @model_validator(mode="after")
    def _window_ordered(self) -> SubscribeSeriesPayload:
        if self.t_start >= self.t_end:
            raise ValueError("t_start must be before t_end")
        return self


class ScrubPayload(_Strict):
    run_id: RunId
    t: SimTime


class PausePayload(_Strict):
    run_id: RunId
    paused: bool = Field(description="True pauses, false resumes.")


class StepPayload(_Strict):
    run_id: RunId
    direction: Literal["forward", "backward"]


# ===== Response payloads ====================================================


class OkPayload(_Strict):
    request_id: NonEmptyText
    result: dict[str, JsonValue]


class ErrorPayload(_Strict):
    request_id: NonEmptyText | None = Field(description="Null when the request could not be parsed.")
    code: ErrorCode
    message: NonEmptyText
    requirement_id: RequirementId | None = Field(default=None, description="ID of the violated requirement.")


# ===== Event payloads =======================================================


class RunProgressPayload(_Strict):
    run_id: RunId
    t: SimTime
    t_end: SimTime | None = Field(default=None, description="Planned end time, null when open-ended.")

    @model_validator(mode="after")
    def _within_end(self) -> RunProgressPayload:
        if self.t_end is not None and self.t > self.t_end:
            raise ValueError("t must not exceed t_end")
        return self


class RunEndedPayload(_Strict):
    run_id: RunId
    t: SimTime
    reason: RunEndReason
    message: NonEmptyText
    requirement_id: RequirementId | None = None


class TickBatchPayload(_Strict):
    run_id: RunId
    sequence: int = Field(ge=0)
    t_start: SimTime
    t_end: SimTime
    series: SeriesNames
    dtype: Literal["float64"]
    byte_order: Literal["little"]
    sample_count: int = Field(ge=1)
    byte_length: int = Field(
        ge=FLOAT64_SIZE * 2,
        description="Binary frame length: (1 + len(series)) * sample_count * 8; the time column comes first.",
    )

    _unique = field_validator("series")(_require_unique)

    @model_validator(mode="after")
    def _consistent(self) -> TickBatchPayload:
        if self.t_start > self.t_end:
            raise ValueError("t_start must not be after t_end")
        expected = (1 + len(self.series)) * self.sample_count * FLOAT64_SIZE
        if self.byte_length != expected:
            raise ValueError(f"byte_length {self.byte_length} does not match {expected} for the series and samples")
        if self.sample_count == 1 and self.t_start != self.t_end:
            raise ValueError("a one-sample batch must have t_start equal to t_end")
        return self


class TimelineEventPayload(_Strict):
    run_id: RunId
    t: SimTime
    category: TimelineCategory
    label: NonEmptyText = Field(description="Message label, verbatim.")


# ===== Messages =============================================================


class ListDesignsRequest(_Envelope):
    kind: Literal["list_designs"]
    payload: ListDesignsPayload


class ValidateDesignRequest(_Envelope):
    kind: Literal["validate_design"]
    payload: ValidateDesignPayload


class RunCaseRequest(_Envelope):
    kind: Literal["run_case"]
    payload: RunCasePayload


class RunSuiteRequest(_Envelope):
    kind: Literal["run_suite"]
    payload: RunSuitePayload


class ReplayOpenRequest(_Envelope):
    kind: Literal["replay_open"]
    payload: ReplayOpenPayload


class SubscribeSeriesRequest(_Envelope):
    kind: Literal["subscribe_series"]
    payload: SubscribeSeriesPayload


class ScrubRequest(_Envelope):
    kind: Literal["scrub"]
    payload: ScrubPayload


class PauseRequest(_Envelope):
    kind: Literal["pause"]
    payload: PausePayload


class StepRequest(_Envelope):
    kind: Literal["step"]
    payload: StepPayload


class OkResponse(_Envelope):
    kind: Literal["ok"]
    payload: OkPayload


class ErrorResponse(_Envelope):
    kind: Literal["error"]
    payload: ErrorPayload


class RunProgressEvent(_Envelope):
    kind: Literal["run_progress"]
    payload: RunProgressPayload


class RunEndedEvent(_Envelope):
    kind: Literal["run_ended"]
    payload: RunEndedPayload


class TickBatchEvent(_Envelope):
    kind: Literal["tick_batch"]
    payload: TickBatchPayload


class TimelineEventMessage(_Envelope):
    kind: Literal["timeline_event"]
    payload: TimelineEventPayload


Message = Annotated[
    ListDesignsRequest
    | ValidateDesignRequest
    | RunCaseRequest
    | RunSuiteRequest
    | ReplayOpenRequest
    | SubscribeSeriesRequest
    | ScrubRequest
    | PauseRequest
    | StepRequest
    | OkResponse
    | ErrorResponse
    | RunProgressEvent
    | RunEndedEvent
    | TickBatchEvent
    | TimelineEventMessage,
    Field(discriminator="kind"),
]

MESSAGE_ADAPTER: TypeAdapter[Message] = TypeAdapter(Message)


def parse_message(text: str | bytes) -> Message:
    return MESSAGE_ADAPTER.validate_json(text)


def dump_message(message: Message) -> str:
    return MESSAGE_ADAPTER.dump_json(message, exclude_none=False).decode("utf-8")


# ===== Schema ===============================================================


def render_schema() -> str:
    schema = MESSAGE_ADAPTER.json_schema()
    document: dict[str, object] = {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": SCHEMA_ID,
        "title": "FacadeMessage",
        "description": f"Facade protocol message, version {PROTOCOL_VERSION} (ADR 0015). Generated; do not edit.",
    }
    document.update(schema)
    return json.dumps(document, indent=2) + "\n"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m mechsim.facade.protocol")
    parser.add_argument("--write-schema", action="store_true", help=f"write {SCHEMA_PATH.name} into schemas/")
    arguments = parser.parse_args(argv)
    if not arguments.write_schema:
        parser.print_help()
        return 2
    SCHEMA_PATH.write_text(render_schema(), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
