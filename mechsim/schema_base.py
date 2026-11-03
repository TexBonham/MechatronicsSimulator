"""Shared base for data-file models (L4, invariant 3).

Every data-file model derives from `StrictModel`: unknown keys are rejected and values are never coerced, so a file
either matches its schema exactly or fails to load. `format_errors` turns a pydantic `ValidationError` into one line
per error for the loaders' reports.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, ValidationError


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)


def _location_part(part: str | int) -> str | None:
    if isinstance(part, int):
        return str(part)
    if part[:1].isupper() or "[" in part or "-" in part:
        return None
    return part


def format_errors(error: ValidationError) -> list[str]:
    """One line per error, union branch names dropped from the path; the `${name}` branch's own error is dropped."""
    messages: list[str] = []
    for item in error.errors():
        if any(isinstance(part, str) and part.startswith("is-instance[OpenValue]") for part in item["loc"]):
            continue
        parts = [_location_part(part) for part in item["loc"]]
        location = ".".join(part for part in parts if part is not None)
        text = str(item["msg"]).removeprefix("Value error, ")
        for line in text.split("\n"):
            message = line
            if location != "":
                message = f"{location}: {line}"
            if message not in messages:
                messages.append(message)
    return messages
