"""The engine's per-member carried-load validity record (ENG-052, ruling 69, brief M3-I5): what a run stores in its
archive metadata (`carried_validity`) and what integrity and scoring read to mark a flagged member's results not
valid from its flag time (rulings 69, 72). Engine-free: plain values from `CarriedRecord.values()`."""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from dataclasses import dataclass

VALIDITY_KEY = "carried_validity"
NOT_VALID_KIND = "integrity_not_valid"
NOT_VALID_FLAG = "not_valid"


@dataclass(frozen=True)
class CarriedValidity:
    """One carrying member: `flagged` when the engine found its dynamic ratio above the limit, `since` the start time
    (s) of the first flagged step (rows stamped after it are not valid; 0 while unflagged), `ratio` the largest dynamic
    ratio, `work` the pair's net work on the member (J)."""

    member: str
    flagged: bool
    since: float
    ratio: float
    work: float

    def reason(self) -> str:
        """The not-valid reason scoring states (rulings 69, 72)."""
        return (
            f"one-way carried-load result not valid for {self.member} from {self.since:.12g} s "
            f"(dynamic ratio {self.ratio:.6g})"
        )

    def covers(self, time: float) -> bool:
        """True for a sample inside the not-valid interval: flagged and stamped after the flagged step's start."""
        return self.flagged and time > self.since


def flagged(records: Sequence[CarriedValidity]) -> dict[str, CarriedValidity]:
    """The flagged members by name."""
    return {record.member: record for record in records if record.flagged}


def validity_json(records: Sequence[CarriedValidity]) -> str:
    """Canonical JSON (member order kept, keys sorted); a non-finite number is a `ValueError`, as its reader refuses
    it."""
    for record in records:
        if not all(math.isfinite(value) for value in (record.since, record.ratio, record.work)):
            raise ValueError(f"carried validity of {record.member!r}: a value is not finite")
    data = [
        {
            "flagged": record.flagged,
            "member": record.member,
            "ratio": record.ratio,
            "since": record.since,
            "work": record.work,
        }
        for record in records
    ]
    return json.dumps(data, sort_keys=True)


def validity_from_json(text: str) -> tuple[CarriedValidity, ...]:
    """The records back; a non-finite number or a missing key is a `ValueError`."""
    out: list[CarriedValidity] = []
    for item in json.loads(text):
        values = (float(item["since"]), float(item["ratio"]), float(item["work"]))
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"carried validity of {item['member']!r}: a value is not finite")
        out.append(CarriedValidity(str(item["member"]), bool(item["flagged"]), *values))
    return tuple(out)
