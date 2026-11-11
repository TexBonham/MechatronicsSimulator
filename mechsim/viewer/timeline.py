"""Event timeline of a run (VIE-002): recorded events and criterion failures at their simulated times."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Literal, Protocol

Category = Literal["command", "fault", "safety_event", "criterion_failure"]

COMMAND_KINDS = frozenset({"operator_message", "operator_command", "operator_status"})
FAULT_KINDS = frozenset({"motor_stall", "rail_off", "rail_on"})
SAFETY_MARKERS = ("estop", "safety")
FAULT_MARKERS = ("fault",)


class EventLike(Protocol):
    @property
    def time(self) -> float: ...

    @property
    def source(self) -> str: ...

    @property
    def kind(self) -> str: ...

    @property
    def detail(self) -> str: ...


class CriterionLike(Protocol):
    @property
    def name(self) -> str: ...

    @property
    def status(self) -> str: ...

    @property
    def worst_time(self) -> float | None: ...


@dataclass(frozen=True)
class TimelineEntry:
    t: float
    category: Category
    label: str
    source: str


def category_of(kind: str) -> Category | None:
    """Timeline category of a recorded event kind; None for an event the timeline does not mark."""
    if kind in COMMAND_KINDS:
        return "command"
    if kind in FAULT_KINDS or any(marker in kind for marker in FAULT_MARKERS):
        return "fault"
    if any(marker in kind for marker in SAFETY_MARKERS):
        return "safety_event"
    return None


def _event_label(event: EventLike) -> str:
    if event.detail == "":
        return event.kind
    return event.detail


def timeline(
    events: Sequence[EventLike], criteria: Sequence[CriterionLike] = (), end_time: float = 0.0
) -> tuple[TimelineEntry, ...]:
    """Every command, fault and safety event in record order, then each failed criterion at its worst time (the run's
    end time when the criterion has none), ordered by time; labels verbatim (D2)."""
    entries: list[TimelineEntry] = []
    for event in events:
        category = category_of(event.kind)
        if category is not None:
            entries.append(TimelineEntry(float(event.time), category, _event_label(event), event.source))
    for criterion in criteria:
        if criterion.status == "fail":
            t = end_time
            if criterion.worst_time is not None:
                t = float(criterion.worst_time)
            entries.append(TimelineEntry(t, "criterion_failure", criterion.name, "scoring"))
    return tuple(sorted(entries, key=lambda entry: entry.t))
