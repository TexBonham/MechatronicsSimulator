"""Facade protocol message models: shape, bounds and rejections (FAC spec, ADR 0015 proposed)."""

from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

from mechsim.facade import protocol

EXAMPLES = Path(__file__).resolve().parents[2] / "schemas" / "examples"
EXAMPLE_NAMES = ["facade_request.json", "facade_response.json", "facade_event.json", "facade_tick_batch.json"]


def _load(name: str) -> dict[str, Any]:
    data: dict[str, Any] = json.loads((EXAMPLES / name).read_text(encoding="utf-8"))
    return data


def _parse(data: dict[str, Any]) -> protocol.Message:
    return protocol.parse_message(json.dumps(data))


def _envelope(kind: str, payload: dict[str, Any]) -> dict[str, Any]:
    return {"version": protocol.PROTOCOL_VERSION, "message_id": "m-1", "kind": kind, "payload": payload}


VALID: dict[str, dict[str, Any]] = {
    "list_designs": {},
    "validate_design": {"design": "designs/example/design.yaml"},
    "run_case": {"design": "d.yaml", "test_case": "t.yaml", "seed": 0, "mode": "native"},
    "run_suite": {"design": "d.yaml", "suite": "s.yaml", "seed": 7, "mode": "emulated"},
    "replay_open": {"archive": "runs/run-0001.h5"},
    "subscribe_series": {"run_id": "r", "series": ["a.b"], "t_start": 0.0, "t_end": 1.0, "points": 1},
    "scrub": {"run_id": "r", "t": 0.5},
    "pause": {"run_id": "r", "paused": True},
    "step": {"run_id": "r", "direction": "backward"},
    "ok": {"request_id": "c-1", "result": {"designs": ["a", "b"], "count": 2, "nested": {"x": None}}},
    "error": {"request_id": None, "code": "invalid_message", "message": "not JSON"},
    "run_progress": {"run_id": "r", "t": 1.0, "t_end": 2.0},
    "run_ended": {"run_id": "r", "t": 2.0, "reason": "drift", "message": "energy drift", "requirement_id": "KER-015"},
    "tick_batch": {
        "run_id": "r",
        "sequence": 0,
        "t_start": 0.0,
        "t_end": 0.001,
        "series": ["a.b"],
        "dtype": "float64",
        "byte_order": "little",
        "sample_count": 2,
        "byte_length": 32,
    },
    "timeline_event": {"run_id": "r", "t": 0.0, "category": "criterion_failure", "label": "position error"},
}

OPTIONAL: dict[str, set[str]] = {
    "error": {"requirement_id"},
    "run_progress": {"t_end"},
    "run_ended": {"requirement_id"},
}

WRONG_TYPES: dict[str, dict[str, Any]] = {
    "validate_design": {"design": 3},
    "run_case": {"seed": "0"},
    "run_suite": {"seed": 1.5},
    "replay_open": {"archive": ["a"]},
    "subscribe_series": {"points": "10"},
    "scrub": {"t": "0.5 s"},
    "pause": {"paused": "yes"},
    "step": {"direction": 1},
    "ok": {"result": [1, 2]},
    "error": {"code": 404},
    "run_progress": {"t": None},
    "run_ended": {"reason": True},
    "tick_batch": {"sample_count": 2.0},
    "timeline_event": {"label": 5},
}


def _expect_invalid(data: dict[str, Any], fragment: str | None = None) -> None:
    with pytest.raises(ValidationError) as caught:
        _parse(data)
    if fragment is not None:
        assert fragment in str(caught.value)


# ===== Examples and round trip ==============================================


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize("name", EXAMPLE_NAMES)
def test_example_validates_and_round_trips(name: str) -> None:
    data = _load(name)
    message = _parse(data)
    assert json.loads(protocol.dump_message(message)) == data


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize("kind", sorted(VALID))
def test_every_kind_round_trips(kind: str) -> None:
    data = _envelope(kind, VALID[kind])
    message = _parse(data)
    assert message.kind == kind
    dumped = json.loads(protocol.dump_message(message))
    for field in OPTIONAL.get(kind, set()):
        if field not in data["payload"]:
            assert dumped["payload"].pop(field) is None
    assert dumped == data
    assert protocol.parse_message(protocol.dump_message(message)) == message


# ===== Required fields, types, unknown keys =================================


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize("field", ["version", "message_id", "kind", "payload"])
def test_envelope_field_missing_rejected(field: str) -> None:
    data = _envelope("scrub", VALID["scrub"])
    del data[field]
    _expect_invalid(data)


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize(
    ("kind", "field"),
    [
        (kind, field)
        for kind, payload in sorted(VALID.items())
        for field in payload
        if field not in OPTIONAL.get(kind, set())
    ],
)
def test_payload_field_missing_rejected(kind: str, field: str) -> None:
    payload = copy.deepcopy(VALID[kind])
    del payload[field]
    _expect_invalid(_envelope(kind, payload), field)


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize("kind", sorted(WRONG_TYPES))
def test_payload_wrong_type_rejected(kind: str) -> None:
    payload = copy.deepcopy(VALID[kind])
    payload.update(WRONG_TYPES[kind])
    _expect_invalid(_envelope(kind, payload))


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize(
    ("field", "value"),
    [("version", "1"), ("version", 1.0), ("message_id", 1), ("kind", 3), ("payload", [])],
)
def test_envelope_wrong_type_rejected(field: str, value: Any) -> None:
    data = _envelope("scrub", VALID["scrub"])
    data[field] = value
    _expect_invalid(data)


@pytest.mark.req("FAC-002")
def test_envelope_unknown_key_rejected() -> None:
    data = _envelope("scrub", VALID["scrub"])
    data["extra"] = 1
    _expect_invalid(data, "extra")


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize("kind", sorted(VALID))
def test_payload_unknown_key_rejected(kind: str) -> None:
    payload = copy.deepcopy(VALID[kind])
    payload["unexpected"] = 1
    _expect_invalid(_envelope(kind, payload), "unexpected")


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize("kind", ["nope", "", "Scrub", "resume"])
def test_unknown_kind_rejected(kind: str) -> None:
    _expect_invalid(_envelope(kind, {}), "does not match any of the expected tags")


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize("text", ["", "not json", "[]", "1", "null"])
def test_non_object_rejected(text: str) -> None:
    with pytest.raises(ValidationError):
        protocol.parse_message(text)


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize(
    ("kind", "field"),
    [
        ("scrub", "run_id"),
        ("validate_design", "design"),
        ("run_case", "test_case"),
        ("run_case", "mode"),
        ("run_suite", "suite"),
        ("replay_open", "archive"),
        ("ok", "request_id"),
        ("error", "message"),
        ("timeline_event", "label"),
        ("run_ended", "message"),
    ],
)
def test_empty_text_rejected(kind: str, field: str) -> None:
    payload = copy.deepcopy(VALID[kind])
    payload[field] = ""
    _expect_invalid(_envelope(kind, payload), field)


@pytest.mark.req("FAC-002")
def test_empty_message_id_rejected() -> None:
    data = _envelope("scrub", VALID["scrub"])
    data["message_id"] = ""
    _expect_invalid(data, "message_id")


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize(("seed", "valid"), [(-1, False), (0, True), (2**40, True)])
def test_seed_bound(seed: int, valid: bool) -> None:
    payload = copy.deepcopy(VALID["run_case"])
    payload["seed"] = seed
    if valid:
        _parse(_envelope("run_case", payload))
    else:
        _expect_invalid(_envelope("run_case", payload), "seed")


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize("value", ["sideways", "Forward", ""])
def test_step_direction_enumerated(value: str) -> None:
    payload = copy.deepcopy(VALID["step"])
    payload["direction"] = value
    _expect_invalid(_envelope("step", payload), "direction")


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize(
    ("kind", "field", "value"),
    [
        ("run_ended", "reason", "crashed"),
        ("timeline_event", "category", "status"),
        ("error", "code", "teapot"),
    ],
)
def test_enumerated_value_rejected(kind: str, field: str, value: str) -> None:
    payload = copy.deepcopy(VALID[kind])
    payload[field] = value
    _expect_invalid(_envelope(kind, payload), field)


@pytest.mark.req("FAC-002")
def test_ok_result_empty_object_accepted() -> None:
    payload = copy.deepcopy(VALID["ok"])
    payload["result"] = {}
    _parse(_envelope("ok", payload))


# ===== Version ===============================================================


@pytest.mark.req("FAC-003")
@pytest.mark.parametrize("version", [0, 2, -1, 99])
def test_version_mismatch_rejected_naming_both(version: int) -> None:
    data = _envelope("pause", VALID["pause"])
    data["version"] = version
    _expect_invalid(data, f"unsupported protocol version {version}; this service speaks version 1")


@pytest.mark.req("FAC-003")
@pytest.mark.parametrize("kind", sorted(VALID))
def test_every_kind_carries_version(kind: str) -> None:
    data = _envelope(kind, VALID[kind])
    del data["version"]
    _expect_invalid(data, "version")


# ===== Simulated time ========================================================


TIME_FIELDS: list[tuple[str, str]] = [
    ("scrub", "t"),
    ("subscribe_series", "t_start"),
    ("run_progress", "t"),
    ("run_ended", "t"),
    ("tick_batch", "t_start"),
    ("timeline_event", "t"),
]


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize(("kind", "field"), TIME_FIELDS)
@pytest.mark.parametrize(("value", "valid"), [(-1e-9, False), (0.0, True), (0, True)])
def test_time_lower_bound(kind: str, field: str, value: float, valid: bool) -> None:
    payload = copy.deepcopy(VALID[kind])
    payload[field] = value
    if valid:
        _parse(_envelope(kind, payload))
    else:
        _expect_invalid(_envelope(kind, payload), field)


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize(("kind", "field"), TIME_FIELDS)
@pytest.mark.parametrize("text", ["Infinity", "NaN", "-Infinity"])
def test_time_non_finite_rejected(kind: str, field: str, text: str) -> None:
    payload = copy.deepcopy(VALID[kind])
    payload[field] = 0.0
    raw = json.dumps(_envelope(kind, payload)).replace(f'"{field}": 0.0', f'"{field}": {text}')
    with pytest.raises(ValidationError):
        protocol.parse_message(raw)


@pytest.mark.req("FAC-002")
@pytest.mark.parametrize(("t", "t_end", "valid"), [(2.0, 2.0, True), (1.0, 2.0, True), (2.5, 2.0, False)])
def test_run_progress_within_end(t: float, t_end: float, valid: bool) -> None:
    payload = {"run_id": "r", "t": t, "t_end": t_end}
    if valid:
        _parse(_envelope("run_progress", payload))
    else:
        _expect_invalid(_envelope("run_progress", payload), "t_end")


@pytest.mark.req("FAC-002")
def test_run_progress_open_ended_accepted() -> None:
    _parse(_envelope("run_progress", {"run_id": "r", "t": 1e6, "t_end": None}))


# ===== Decimation request ====================================================


@pytest.mark.req("FAC-018")
@pytest.mark.parametrize(("points", "valid"), [(-1, False), (0, False), (1, True), (2000, True)])
def test_decimation_points_bound(points: int, valid: bool) -> None:
    payload = copy.deepcopy(VALID["subscribe_series"])
    payload["points"] = points
    if valid:
        _parse(_envelope("subscribe_series", payload))
    else:
        _expect_invalid(_envelope("subscribe_series", payload), "points")


@pytest.mark.req("FAC-018")
@pytest.mark.parametrize(("t_start", "t_end", "valid"), [(2.0, 1.0, False), (1.0, 1.0, False), (1.0, 1.0001, True)])
def test_decimation_window_ordered(t_start: float, t_end: float, valid: bool) -> None:
    payload = copy.deepcopy(VALID["subscribe_series"])
    payload["t_start"] = t_start
    payload["t_end"] = t_end
    if valid:
        _parse(_envelope("subscribe_series", payload))
    else:
        _expect_invalid(_envelope("subscribe_series", payload), "t_start must be before t_end")


@pytest.mark.req("FAC-018")
@pytest.mark.parametrize("series", [[], ["a.b", "a.b"], [""]])
def test_decimation_series_rejected(series: list[str]) -> None:
    payload = copy.deepcopy(VALID["subscribe_series"])
    payload["series"] = series
    _expect_invalid(_envelope("subscribe_series", payload), "series")


# ===== Binary frame header ===================================================


def _batch(**changes: Any) -> dict[str, Any]:
    payload = copy.deepcopy(VALID["tick_batch"])
    payload.update(changes)
    return _envelope("tick_batch", payload)


@pytest.mark.req("FAC-019")
@pytest.mark.parametrize(("byte_length", "valid"), [(24, False), (31, False), (32, True), (33, False), (40, False)])
def test_byte_length_matches_samples_and_dtype(byte_length: int, valid: bool) -> None:
    if valid:
        _parse(_batch(byte_length=byte_length))
    else:
        _expect_invalid(_batch(byte_length=byte_length), "does not match")


@pytest.mark.req("FAC-019")
def test_byte_length_counts_every_series_plus_time() -> None:
    _parse(_batch(series=["a.b", "c.d", "e.f"], byte_length=4 * 2 * 8))
    _expect_invalid(_batch(series=["a.b", "c.d", "e.f"], byte_length=3 * 2 * 8), "does not match")


@pytest.mark.req("FAC-019")
@pytest.mark.parametrize(("sample_count", "valid"), [(-1, False), (0, False), (1, True)])
def test_sample_count_bound(sample_count: int, valid: bool) -> None:
    data = _batch(sample_count=sample_count, byte_length=2 * sample_count * 8, t_start=0.0, t_end=0.0)
    if valid:
        _parse(data)
    else:
        _expect_invalid(data, "sample_count")


@pytest.mark.req("FAC-019")
@pytest.mark.parametrize(("t_end", "valid"), [(0.0, True), (0.001, False)])
def test_one_sample_batch_has_one_time(t_end: float, valid: bool) -> None:
    data = _batch(sample_count=1, byte_length=16, t_end=t_end)
    if valid:
        _parse(data)
    else:
        _expect_invalid(data, "one-sample batch")


@pytest.mark.req("FAC-019")
@pytest.mark.parametrize(("t_start", "t_end", "valid"), [(1.0, 0.5, False), (1.0, 1.0, True), (0.5, 1.0, True)])
def test_batch_window_ordered(t_start: float, t_end: float, valid: bool) -> None:
    data = _batch(t_start=t_start, t_end=t_end)
    if valid:
        _parse(data)
    else:
        _expect_invalid(data, "t_start must not be after t_end")


@pytest.mark.req("FAC-019")
@pytest.mark.parametrize(
    ("field", "value"),
    [("dtype", "float32"), ("dtype", "int64"), ("byte_order", "big"), ("sequence", -1), ("series", [])],
)
def test_batch_header_field_rejected(field: str, value: Any) -> None:
    _expect_invalid(_batch(**{field: value}), field)


@pytest.mark.req("FAC-019")
def test_batch_sequence_zero_accepted() -> None:
    _parse(_batch(sequence=0))


# ===== Error response ========================================================


@pytest.mark.req("FAC-020")
def test_error_requires_code() -> None:
    payload = copy.deepcopy(VALID["error"])
    del payload["code"]
    _expect_invalid(_envelope("error", payload), "code")


@pytest.mark.req("FAC-020")
@pytest.mark.parametrize("code", ["", None])
def test_error_code_not_empty(code: str | None) -> None:
    payload = copy.deepcopy(VALID["error"])
    payload["code"] = code
    _expect_invalid(_envelope("error", payload), "code")


@pytest.mark.req("FAC-020")
@pytest.mark.parametrize(
    ("requirement_id", "valid"),
    [("FAC-003", True), ("KERN-001", True), (None, True), ("FAC-3", False), ("fac-003", False), ("", False)],
)
def test_error_requirement_id_pattern(requirement_id: str | None, valid: bool) -> None:
    payload = copy.deepcopy(VALID["error"])
    payload["requirement_id"] = requirement_id
    if valid:
        _parse(_envelope("error", payload))
    else:
        _expect_invalid(_envelope("error", payload), "requirement_id")


@pytest.mark.req("FAC-020")
def test_error_request_id_may_be_null_but_present() -> None:
    payload = copy.deepcopy(VALID["error"])
    del payload["request_id"]
    _expect_invalid(_envelope("error", payload), "request_id")
