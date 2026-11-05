"""Synthetic, seeded tick records for the recorder tests; no tests here.

Stands in for the C++ sink until the binding lands: every TickRecord field is populated, entity counts are fixed per
run, contact counts vary per tick, and events of every kind arrive on some ticks.
"""

from __future__ import annotations

from fractions import Fraction

import numpy as np

from mechsim.record import RunEnd, RunIdentity, TickRecord
from mechsim.record.tick import (
    BodyState,
    ConstraintForce,
    ContactForce,
    JointForce,
    MemberEndLoads,
    PhysicsOutputs,
    RecordedEvent,
    RopeTension,
    SeriesSample,
    TransmissionState,
    Wrench,
)

BASE_RATE = Fraction(1000)
PINS = (("mcu.pa0", "V"), ("mcu.pb3", "V"), ("encoder.a", "1"))
RAILS = (("supply.24v", "V"),)
EFFORTS = (("motor.torque", "N m"),)
STATES = ("idle", "homing", "run")
EVENTS = (
    ("fault_injector", "injected_fault", "encoder.a stuck high"),
    ("safety", "safety_event", "E-stop pressed"),
    ("operator", "operator_message", "Not-Aus ausgelöst: Hubwerk gestoppt"),
    ("controller", "controller_fault", "watchdog reset"),
)
JOINT_DOFS = (1, 3)
CONSTRAINT_ROWS = (2,)


def _vec(values: np.ndarray) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


def synthetic_rows(seed: int, ticks: int, *, events: bool = True, contacts: bool = True) -> list[TickRecord]:
    """`ticks` rows at base ticks 0, 1, ...; values drawn from one seeded generator (D1)."""
    rng = np.random.default_rng(seed)
    draws = rng.standard_normal((ticks, 64))
    contact_counts = rng.integers(0, 3, size=ticks)
    rows: list[TickRecord] = []
    for tick in range(ticks):
        d = draws[tick]
        quat = d[0:4] / np.linalg.norm(d[0:4])
        bodies = (
            BodyState(
                _vec(d[4:7]),
                (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
                _vec(d[7:10]),
                _vec(d[10:13]),
            ),
            BodyState(_vec(d[13:16]), (1.0, 0.0, 0.0, 0.0), _vec(d[16:19]), _vec(d[19:22])),
        )
        joints = (
            JointForce((float(d[22]),), Wrench(_vec(d[23:26]), _vec(d[26:29]))),
            JointForce((float(d[29]), float(d[30]), float(d[31])), Wrench(_vec(d[32:35]), _vec(d[35:38]))),
        )
        constraints = (ConstraintForce((float(d[38]), float(d[39])), abs(float(d[40])), abs(float(d[41]))),)
        contact_list: tuple[ContactForce, ...] = ()
        if contacts:
            contact_list = tuple(
                ContactForce(index, 1 - index, _vec(d[42 + 3 * index : 45 + 3 * index]), _vec(d[48:51]))
                for index in range(int(contact_counts[tick]))
            )
        members = (MemberEndLoads(Wrench(_vec(d[51:54]), _vec(d[54:57])), Wrench(_vec(d[57:60]), _vec(d[60:63]))),)
        ropes = (RopeTension(abs(float(d[63])), tick % 5 == 0),)
        transmissions = (TransmissionState(float(d[0]) * 1e-3, tick % 3 == 0),)
        physics = PhysicsOutputs(tick + 1, bodies, joints, constraints, contact_list, members, ropes, transmissions)
        event_list: tuple[RecordedEvent, ...] = ()
        if events and tick % 7 == 3:
            source, kind, detail = EVENTS[(tick // 7) % len(EVENTS)]
            event_list = (
                RecordedEvent(tick, source, kind, detail),
                RecordedEvent(tick, "operator", "status", "Bereit"),
            )
        rows.append(
            TickRecord(
                time=tick,
                physics=physics,
                pin_signals=tuple(SeriesSample(name, unit, float(d[index])) for index, (name, unit) in enumerate(PINS)),
                rails=tuple(SeriesSample(name, unit, 24.0 + float(d[5]) * 0.01) for name, unit in RAILS),
                actuator_efforts=tuple(SeriesSample(name, unit, float(d[6])) for name, unit in EFFORTS),
                controller_state=STATES[(tick // 10) % len(STATES)],
                events=event_list,
            )
        )
    return rows


def identity(**overrides: object) -> RunIdentity:
    fields: dict[str, object] = {
        "design_name": "synthetic",
        "design_version": "1.2.3",
        "design_source_hash": "0123456789abcdef0123456789abcdef01234567",
        "test_case_id": "S1",
        "test_case_hash": "89abcdef0123456789abcdef0123456789abcdef",
        "seed": 42,
        "controller_mode": "native",
        "physics_rate": Fraction(1000),
        "control_rate": Fraction(1000),
        "repository_commit": "unset",
        "tool_version": "0.0.0",
        "design_file": b"name: synthetic\nversion: 1.2.3\n",
        "test_case_file": b"id: S1\n",
    }
    fields.update(overrides)
    return RunIdentity(**fields)  # type: ignore[arg-type]


def finished(ticks: int) -> RunEnd:
    return RunEnd("end_condition_met", ticks, ticks)
