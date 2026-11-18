"""Series layout of a recording: names, units and axes shared by the recorder, archive, exports and comparison."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, replace
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
COUPLING = "coupling"
COUPLING_VALUE = "coupling/value"
COUPLING_DIGEST = "coupling/digest"
DIGEST_LENGTH = 64
CONTACT_ROW = "physics/contacts/row"
JOINT_FORCE = "physics/joints/generalized_force"
CONSTRAINT_FORCE = "physics/constraints/force"
JOINT_POSITIONS = "physics/joint_positions"
DRIVE_TORQUE = "physics/joint_drive_torques/torque"
DRIVE_DRIVEN = "physics/joint_drive_torques/driven"
MEMBER_SERIES = (
    "physics/members/start_moment",
    "physics/members/start_force",
    "physics/members/end_moment",
    "physics/members/end_force",
)
ROPE_TENSION = "physics/ropes/tension"
ROPE_MAX_TENSION = "physics/ropes/max_tension"
ROPE_SERIES = (ROPE_TENSION, "physics/ropes/slack", ROPE_MAX_TENSION)

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
    (ROPE_MAX_TENSION, "N", TICK_AXIS, "float64"),
    ("physics/transmissions/slip", "m or rad", TICK_AXIS, "float64"),
    ("physics/transmissions/slipping", "1", TICK_AXIS, "bool"),
    ("physics/transmissions/shaft_load_torque", "N m", TICK_AXIS, "float64"),
    ("physics/transmissions/force", "N or N m", TICK_AXIS, "float64"),
    ("physics/transmissions/stretch", "m or rad", TICK_AXIS, "float64"),
    ("physics/transmissions/power_flow", "1", TICK_AXIS, "int64"),
    (JOINT_POSITIONS, "m or rad", TICK_AXIS, "float64"),
    (DRIVE_TORQUE, "N or N m", TICK_AXIS, "float64"),
    (DRIVE_DRIVEN, "1", TICK_AXIS, "bool"),
    ("physics/payloads/body", "1", TICK_AXIS, "int64"),
    ("physics/payloads/carrier", "1", TICK_AXIS, "int64"),
    ("physics/payloads/attached", "1", TICK_AXIS, "bool"),
    ("physics/payloads/force", "N", TICK_AXIS, "float64"),
)


# Body series (one column per published body): the driver may name their columns, the neutral bodies then the
# native adapter's member node bodies `<member>.node<k>` (PHY-003, REC-018); unnamed when it does not.
BODY_SERIES: tuple[str, ...] = (
    "physics/bodies/position",
    "physics/bodies/orientation",
    "physics/bodies/angular_velocity",
    "physics/bodies/linear_velocity",
)


# Index-column series (one column per joint coordinate, transmission, payload, structural member or rope, model order)
# that take per-column names and units from the driver (REC-018, REC-004, REC-026).
COLUMN_SERIES: tuple[str, ...] = (
    *MEMBER_SERIES,
    *ROPE_SERIES,
    "physics/transmissions/slip",
    "physics/transmissions/slipping",
    "physics/transmissions/shaft_load_torque",
    "physics/transmissions/force",
    "physics/transmissions/stretch",
    "physics/transmissions/power_flow",
    JOINT_POSITIONS,
    DRIVE_TORQUE,
    DRIVE_DRIVEN,
    "physics/payloads/body",
    "physics/payloads/carrier",
    "physics/payloads/attached",
    "physics/payloads/force",
)


@dataclass(frozen=True)
class Columns:
    """Per-column labels of one index-column series, from the neutral model: entity names in column order and,
    when given, each column's unit (REC-018)."""

    names: tuple[str, ...]
    units: tuple[str, ...] = ()


@dataclass(frozen=True)
class Series:
    """One recorded series: values with the axis as first dimension, its unit (REC-018), for the flattened
    per-DOF series the number of columns each joint or constraint contributes, and for an index-column series the
    column names and per-column units (empty when the caller gave none; `unit` then is the conditional string)."""

    values: NDArray[Any]
    unit: str
    axis: str
    counts: tuple[int, ...] = ()
    names: tuple[str, ...] = ()
    units: tuple[str, ...] = ()


def check_columns(columns: Mapping[str, Columns]) -> None:
    """Shape-free checks of driver column labels: known series, valid unique names, one unit per name or none."""
    for path, labels in columns.items():
        if path not in COLUMN_SERIES:
            raise RecordError(f"columns: {path!r} is not an index-column series")
        for name in labels.names:
            check_name(name, path)
        if len(set(labels.names)) != len(labels.names):
            raise RecordError(f"{path}: duplicate column names in {list(labels.names)}")
        if labels.units and len(labels.units) != len(labels.names):
            raise RecordError(f"{path}: {len(labels.units)} units for {len(labels.names)} columns")
        for unit in labels.units:
            check_text(unit, f"{path} unit")
            if unit == "":
                raise RecordError(f"{path}: empty column unit")


def label_series(series: dict[str, Series], columns: Mapping[str, Columns]) -> dict[str, Series]:
    """`series` with the given column labels applied; each must match its series' column count."""
    check_columns(columns)
    labelled = dict(series)
    for path, labels in columns.items():
        found = labelled[path]
        width = 0
        if found.values.ndim > 1:
            width = int(found.values.shape[1])
        if len(labels.names) != width:
            raise RecordError(f"{path}: {len(labels.names)} column names for {width} columns")
        labelled[path] = replace(found, names=labels.names, units=labels.units)
    return labelled


@dataclass(frozen=True)
class Recording:
    """Everything one run recorded, in canonical series order. `coupling` lists (name, kind) of the coupling
    variables in name order; their series are `coupling/value/<name>` and `coupling/digest/<name>` (KER-022)."""

    base_rate: Fraction
    series: dict[str, Series]
    named: dict[str, tuple[str, ...]]
    controller_states: tuple[str, ...]
    events: tuple[tuple[int, RecordedEvent], ...]
    coupling: tuple[tuple[str, str], ...] = ()


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
