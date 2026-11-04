"""Series layout of a recording: names, units and axes shared by the recorder, archive, exports and comparison."""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from typing import Any

import numpy as np
from numpy.typing import NDArray

from mechsim.record.tick import RecordedEvent, RecordError

TICK_AXIS = "tick"
CONTACT_AXIS = "contact"

PINS = "signals/pins"
RAILS = "signals/rails"
EFFORTS = "signals/actuator_efforts"
NAMED_GROUPS = (PINS, RAILS, EFFORTS)

CONTROLLER_STATE = "controller/state"
CONTACT_ROW = "physics/contacts/row"
JOINT_FORCE = "physics/joints/generalized_force"
CONSTRAINT_FORCE = "physics/constraints/force"

AXIS_TIME = {
    TICK_AXIS: ("time/tick", "time/seconds"),
    CONTACT_AXIS: ("physics/contacts/tick", "physics/contacts/seconds"),
}

# (path, unit, axis, dtype) in canonical order; named signal series follow, then the controller state.
FIXED_SERIES: tuple[tuple[str, str, str, str], ...] = (
    ("time/tick", "1", TICK_AXIS, "int64"),
    ("time/seconds", "s", TICK_AXIS, "float64"),
    ("physics/steps", "1", TICK_AXIS, "int64"),
    ("physics/bodies/position", "m", TICK_AXIS, "float64"),
    ("physics/bodies/orientation", "1", TICK_AXIS, "float64"),
    ("physics/bodies/angular_velocity", "rad/s", TICK_AXIS, "float64"),
    ("physics/bodies/linear_velocity", "m/s", TICK_AXIS, "float64"),
    (JOINT_FORCE, "N or N m", TICK_AXIS, "float64"),
    ("physics/joints/reaction_moment", "N m", TICK_AXIS, "float64"),
    ("physics/joints/reaction_force", "N", TICK_AXIS, "float64"),
    (CONSTRAINT_FORCE, "N or N m", TICK_AXIS, "float64"),
    ("physics/constraints/translational_error", "m", TICK_AXIS, "float64"),
    ("physics/constraints/rotational_error", "rad", TICK_AXIS, "float64"),
    (CONTACT_ROW, "1", CONTACT_AXIS, "int64"),
    ("physics/contacts/tick", "1", CONTACT_AXIS, "int64"),
    ("physics/contacts/seconds", "s", CONTACT_AXIS, "float64"),
    ("physics/contacts/body_a", "1", CONTACT_AXIS, "int64"),
    ("physics/contacts/body_b", "1", CONTACT_AXIS, "int64"),
    ("physics/contacts/point", "m", CONTACT_AXIS, "float64"),
    ("physics/contacts/force", "N", CONTACT_AXIS, "float64"),
    ("physics/members/start_moment", "N m", TICK_AXIS, "float64"),
    ("physics/members/start_force", "N", TICK_AXIS, "float64"),
    ("physics/members/end_moment", "N m", TICK_AXIS, "float64"),
    ("physics/members/end_force", "N", TICK_AXIS, "float64"),
    ("physics/ropes/tension", "N", TICK_AXIS, "float64"),
    ("physics/ropes/slack", "1", TICK_AXIS, "bool"),
    ("physics/transmissions/slip", "m or rad", TICK_AXIS, "float64"),
    ("physics/transmissions/slipping", "1", TICK_AXIS, "bool"),
)


@dataclass(frozen=True)
class Series:
    """One recorded series: values with the axis as first dimension, its unit (REC-018), and for the flattened
    per-DOF series the number of columns each joint or constraint contributes."""

    values: NDArray[Any]
    unit: str
    axis: str
    counts: tuple[int, ...] = ()


@dataclass(frozen=True)
class Recording:
    """Everything one run recorded, in canonical series order."""

    base_rate: Fraction
    series: dict[str, Series]
    named: dict[str, tuple[str, ...]]
    controller_states: tuple[str, ...]
    events: tuple[tuple[int, RecordedEvent], ...]


def seconds(ticks: NDArray[np.int64], base_rate: Fraction) -> NDArray[np.float64]:
    """Simulated seconds of base-tick counts: tick * den / num, one rounding."""
    scaled = ticks * np.int64(base_rate.denominator)
    result: NDArray[np.float64] = np.true_divide(scaled, float(base_rate.numerator))
    return result


def check_rate(rate: Fraction, what: str) -> None:
    if rate <= 0:
        raise RecordError(f"{what} must be positive, got {rate}")


def check_text(text: str, what: str) -> None:
    """Fixed-length HDF5 strings drop trailing NULs, so a NUL would not survive the round trip verbatim (REC-009)."""
    if "\0" in text:
        raise RecordError(f"{what} {text!r} contains a NUL character")


def check_name(name: str, group: str) -> None:
    if name == "":
        raise RecordError(f"{group}: empty series name")
    if "/" in name:
        raise RecordError(f"{group}: series name {name!r} contains '/'")
    check_text(name, f"{group} name")
