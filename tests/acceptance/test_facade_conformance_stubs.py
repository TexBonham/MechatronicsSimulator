"""Facade conformance suite (FAC-005, ADR 0007, ADR 0015 proposed): a client-independent suite that drives the
service over the protocol only. A fixture starts the WebSocket service in-process on a free port with a token; every
test speaks protocol messages through the `websockets` client and never calls the facade. The pendulum P1 run is
the suite's one simulated run (module fixture); FAC-015 adds the CLI's run of the same case to compare archives.
"""

from __future__ import annotations

import asyncio
import hashlib
import io
import json
import shutil
import threading
import uuid
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from websockets.exceptions import ConnectionClosed, InvalidStatus
from websockets.sync.client import ClientConnection, connect

from mechsim.facade.protocol import PROTOCOL_VERSION, TickBatchPayload

ROOT = Path(__file__).resolve().parents[2]
TOKEN = "conformance-token"
ANGLE = "joint:pivot.position"
RUN_ID = "P1-seed0"
POINTS = 60
P1_END = 24.4
MAX_FRAME = 2**24


@dataclass
class Exchange:
    """Every frame the service sent for one request, up to and including its `ok` or `error` answer."""

    messages: list[dict[str, Any]] = field(default_factory=list)
    batches: list[tuple[dict[str, Any], bytes]] = field(default_factory=list)

    @property
    def answer(self) -> dict[str, Any]:
        return self.messages[-1]

    def kinds(self) -> list[str]:
        return [message["kind"] for message in self.messages]


def send(ws: ClientConnection, kind: str, payload: dict[str, Any], version: int = PROTOCOL_VERSION) -> str:
    message_id = uuid.uuid4().hex
    ws.send(json.dumps({"version": version, "message_id": message_id, "kind": kind, "payload": payload}))
    return message_id


def receive(ws: ClientConnection, request_id: str | None, timeout: float = 120.0) -> Exchange:
    exchange = Exchange()
    while True:
        frame = ws.recv(timeout=timeout)
        assert isinstance(frame, str), "a binary frame must follow its tick_batch header"
        message = json.loads(frame)
        exchange.messages.append(message)
        if message["kind"] == "tick_batch":
            body = ws.recv(timeout=timeout)
            assert isinstance(body, bytes)
            exchange.batches.append((message["payload"], body))
            continue
        if message["kind"] in ("ok", "error") and message["payload"]["request_id"] == request_id:
            return exchange


def request(ws: ClientConnection, kind: str, payload: dict[str, Any], version: int = PROTOCOL_VERSION) -> Exchange:
    return receive(ws, send(ws, kind, payload, version))


def decode(header: dict[str, Any], body: bytes) -> np.ndarray:
    """(1 + series) x samples float64 little-endian, time column first (FAC-019)."""
    rows = 1 + len(header["series"])
    return np.frombuffer(body, dtype="<f8").reshape(rows, header["sample_count"])


def next_batch(ws: ClientConnection) -> tuple[dict[str, Any], bytes]:
    message = json.loads(ws.recv(timeout=30))
    assert message["kind"] == "tick_batch"
    body = ws.recv(timeout=30)
    assert isinstance(body, bytes)
    return message["payload"], body


class Service:
    """The service on its own event loop thread, as a separate process would run it."""

    def __init__(self, files_root: Path) -> None:
        from mechsim.facade.core_wiring import CoreLiveWiring
        from mechsim.facade.server import serve
        from mechsim.facade.service import Facade, FacadeServices
        from mechsim.testing.core_driver import RecordArchiveReader, core_services

        services = FacadeServices(
            core_services(files_root / "designs"),
            RecordArchiveReader(),
            files_root / "designs",
            CoreLiveWiring(),
            files_root,
        )
        self.facade = Facade(services, files_root / "runs")
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        self.server = asyncio.run_coroutine_threadsafe(serve(self.facade, port=0, token=TOKEN), self.loop).result(30)

    def url(self, token: str = TOKEN) -> str:
        return f"ws://127.0.0.1:{self.server.port}/?token={token}"

    def close(self) -> None:
        asyncio.run_coroutine_threadsafe(self.server.close(), self.loop).result(30)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(30)


@dataclass
class Session:
    service: Service
    root: Path
    run: Exchange
    subscribed: Exchange


def _copy_inputs(root: Path) -> None:
    (root / "designs" / "pendulum").mkdir(parents=True)
    shutil.copy(ROOT / "designs" / "pendulum" / "design.yaml", root / "designs" / "pendulum" / "design.yaml")
    (root / "test_cases" / "pendulum").mkdir(parents=True)
    shutil.copy(ROOT / "test_cases" / "pendulum" / "P1_period.yaml", root / "test_cases" / "pendulum")


@pytest.fixture(scope="module")
def session(tmp_path_factory: pytest.TempPathFactory) -> Iterator[Session]:
    root = tmp_path_factory.mktemp("conformance")
    _copy_inputs(root)
    service = Service(root)
    with connect(service.url()) as ws:
        window = {"run_id": RUN_ID, "series": [ANGLE], "t_start": 0.0, "t_end": P1_END, "points": POINTS}
        subscribed = request(ws, "subscribe_series", window)
        run = request(
            ws,
            "run_case",
            {
                "design": "designs/pendulum/design.yaml",
                "test_case": "test_cases/pendulum/P1_period.yaml",
                "seed": 0,
                "mode": "none",
            },
        )
    yield Session(service, root, run, subscribed)
    service.close()


@pytest.fixture
def ws(session: Session) -> Iterator[ClientConnection]:
    with connect(session.service.url(), max_size=MAX_FRAME) as client:
        yield client


def archive(session: Session) -> Path:
    return Path(session.run.answer["payload"]["result"]["archive"])


def open_replay(ws: ClientConnection, session: Session) -> dict[str, Any]:
    exchange = request(ws, "replay_open", {"archive": str(archive(session))})
    assert exchange.answer["kind"] == "ok"
    result: dict[str, Any] = exchange.answer["payload"]["result"]
    return result


def assert_error(exchange: Exchange, code: str, requirement: str | None) -> dict[str, Any]:
    """Every error carries a code, a message and, where one applies, the violated requirement (FAC-020)."""
    answer = exchange.answer
    assert answer["kind"] == "error"
    payload: dict[str, Any] = answer["payload"]
    assert (payload["code"], payload["requirement_id"]) == (code, requirement)
    assert payload["message"] != ""
    return payload


# ===== Connection and envelope (FAC-003, FAC-005, FAC-020) ==============


@pytest.mark.req("FAC-005")
def test_conformance_fac_005(session: Session, tmp_path: Path) -> None:
    """A client with only the URL and token is served; a missing or wrong token is refused at the handshake; shutdown
    closes every connection with 1001 going away."""
    for bad in ("", "wrong"):
        with pytest.raises(InvalidStatus) as raised, connect(session.service.url(bad)):
            pass
        assert raised.value.response.status_code == 401
    other = tmp_path / "other"
    _copy_inputs(other)
    service = Service(other)
    with connect(service.url()) as client:
        assert request(client, "list_designs", {}).answer["kind"] == "ok"
        service.close()
        with pytest.raises(ConnectionClosed) as closed:
            client.recv(timeout=10)
    assert closed.value.rcvd is not None
    assert closed.value.rcvd.code == 1001


@pytest.mark.req("FAC-003", "FAC-020")
def test_conformance_version_mismatch_names_both_versions(ws: ClientConnection) -> None:
    message_id = send(ws, "list_designs", {}, version=PROTOCOL_VERSION + 1)
    exchange = receive(ws, message_id)
    payload = assert_error(exchange, "unsupported_version", "FAC-003")
    assert payload["request_id"] == message_id
    assert f"version {PROTOCOL_VERSION + 1}" in payload["message"]
    assert f"version {PROTOCOL_VERSION}" in payload["message"]


@pytest.mark.req("FAC-020")
def test_conformance_malformed_and_unknown_requests(ws: ClientConnection) -> None:
    ws.send("{not json")
    assert assert_error(receive(ws, None), "invalid_message", "FAC-002")["request_id"] is None
    ws.send(b"\x00\x01")
    assert assert_error(receive(ws, None), "invalid_message", "FAC-019")["request_id"] is None
    unknown = send(ws, "launch_rockets", {})
    assert assert_error(receive(ws, unknown), "unknown_kind", "FAC-020")["request_id"] == unknown
    service_kind = send(ws, "ok", {"request_id": "x", "result": {}})
    assert_error(receive(ws, service_kind), "unknown_kind", "FAC-020")
    window = {"run_id": RUN_ID, "series": [ANGLE, ANGLE], "t_start": 1.0, "t_end": 0.5, "points": 0}
    bad = send(ws, "subscribe_series", window)
    assert assert_error(receive(ws, bad), "invalid_message", "FAC-018")["request_id"] == bad


# ===== Operations (FAC-010) =============================================


@pytest.mark.req("FAC-010")
def test_conformance_fac_010(ws: ClientConnection, session: Session) -> None:
    """validate, run, suite and replay answer with ok or error responses."""
    listed = request(ws, "list_designs", {}).answer
    assert [d["name"] for d in listed["payload"]["result"]["designs"]] == ["pendulum"]
    valid = request(ws, "validate_design", {"design": "designs/pendulum/design.yaml"}).answer
    assert valid["kind"] == "ok"
    assert [(f["kind"], f["errors"]) for f in valid["payload"]["result"]["files"]] == [("design", [])]
    assert_error(request(ws, "validate_design", {"design": "designs/none/design.yaml"}), "not_found", "FAC-010")
    result = session.run.answer["payload"]["result"]
    assert session.run.answer["kind"] == "ok"
    assert (result["run_id"], result["status"], result["end_reason"], result["error"]) == (
        RUN_ID,
        "pass",
        "end_time",
        None,
    )
    assert open_replay(ws, session)["run_id"] == RUN_ID
    assert_error(request(ws, "replay_open", {"archive": "runs/none.h5"}), "not_found", "FAC-008")


# ===== Streaming (FAC-006, FAC-007, FAC-019) ============================


@pytest.mark.req("FAC-006")
def test_conformance_fac_006(session: Session) -> None:
    """A subscribed client receives progress and tick batches while the run goes, then run_ended, then ok."""
    assert session.subscribed.answer["payload"]["result"] == {"run_id": RUN_ID, "live": True, "sample_count": 0}
    kinds = session.run.kinds()
    assert kinds[-2:] == ["run_ended", "ok"]
    assert kinds.count("run_progress") >= 2
    assert kinds.index("tick_batch") < kinds.index("run_ended")
    progress = [m["payload"]["t"] for m in session.run.messages if m["kind"] == "run_progress"]
    assert progress == sorted(progress)
    # P1 ends at 24.4 s, the last 1 kHz sample; the last progress report is that sample's time.
    assert progress[-1] == pytest.approx(P1_END, abs=1e-9)
    ended = session.run.messages[-2]["payload"]
    assert (ended["run_id"], ended["reason"], ended["requirement_id"]) == (RUN_ID, "end_condition", None)
    sequences = [header["sequence"] for header, _ in session.run.batches]
    assert sequences == list(range(len(sequences)))


@pytest.mark.req("FAC-007", "FAC-018")
def test_conformance_fac_007(session: Session, ws: ClientConnection) -> None:
    """At most N points per series over the window, decimated in the service; the archive is untouched by replay
    subscriptions."""
    times = np.concatenate([decode(header, body)[0] for header, body in session.run.batches])
    # 24400 samples over [0.001, 24.4] in 60 buckets of 0.40667 s: every bucket holds samples, so exactly 60 arrive.
    assert len(times) == POINTS
    assert np.all(np.diff(times) > 0)
    width = P1_END / POINTS
    centres = (np.arange(POINTS) + 0.5) * width
    # Each kept sample is the recorded tick nearest its bucket centre: within half a 1 ms tick of it.
    assert np.max(np.abs(times - centres)) <= 0.0005 + 1e-9
    before = hashlib.sha256(archive(session).read_bytes()).hexdigest()
    open_replay(ws, session)
    window = {"run_id": RUN_ID, "series": [ANGLE], "t_start": 0.0, "t_end": P1_END, "points": 25}
    replayed = request(ws, "subscribe_series", window)
    assert replayed.answer["payload"]["result"]["sample_count"] == 25
    assert next_batch(ws)[0]["sample_count"] == 25
    assert hashlib.sha256(archive(session).read_bytes()).hexdigest() == before


@pytest.mark.req("FAC-019")
def test_conformance_tick_batch_byte_length(session: Session) -> None:
    """Each header is followed by one binary frame of exactly (1 + series) x samples x 8 bytes; a header whose byte
    length disagrees is rejected by the protocol model."""
    assert len(session.run.batches) >= 2
    for header, body in session.run.batches:
        TickBatchPayload.model_validate(header)
        assert header["byte_length"] == len(body) == (1 + len(header["series"])) * header["sample_count"] * 8
        assert (header["dtype"], header["byte_order"], header["series"]) == ("float64", "little", [ANGLE])
        columns = decode(header, body)
        assert (columns[0][0], columns[0][-1]) == (header["t_start"], header["t_end"])
    header = dict(session.run.batches[0][0])
    header["byte_length"] += 8
    with pytest.raises(ValueError, match="byte_length"):
        TickBatchPayload.model_validate(header)


# ===== Replay (FAC-008, FAC-011) ========================================


@pytest.mark.req("FAC-008")
def test_conformance_fac_008(ws: ClientConnection, session: Session) -> None:
    """Replay serves the archive's data: the handle, a decimated window, and the cursor (scrub, pause, step)."""
    handle = open_replay(ws, session)
    # One row per 1 ms tick after each step, t = 0.001 s to 24.4 s: 24400 samples.
    assert (handle["samples"], handle["t_first"], handle["end_reason"]) == (24400, 0.001, "end_time")
    assert handle["t_last"] == pytest.approx(P1_END, abs=1e-9)
    window = {"run_id": RUN_ID, "series": [ANGLE], "t_start": 1.0, "t_end": 2.0, "points": 10}
    assert request(ws, "subscribe_series", window).kinds() == ["ok"]
    header, body = next_batch(ws)
    assert header["sample_count"] == 10
    times = decode(header, body)[0]
    assert np.all((times >= 1.0) & (times <= 2.0))
    scrub = request(ws, "scrub", {"run_id": RUN_ID, "t": 12.0005}).answer["payload"]["result"]
    assert (scrub["index"], scrub["paused"]) == (11999, True)
    assert scrub["t"] == pytest.approx(12.0, abs=1e-12)
    stepped = request(ws, "step", {"run_id": RUN_ID, "direction": "backward"}).answer["payload"]["result"]
    assert stepped["index"] == 11998
    resumed = request(ws, "pause", {"run_id": RUN_ID, "paused": False}).answer["payload"]["result"]
    assert resumed["paused"] is False
    assert_error(request(ws, "scrub", {"run_id": "nothing", "t": 0.0}), "not_found", "FAC-008")


@pytest.mark.req("FAC-011")
def test_conformance_fac_011(ws: ClientConnection, session: Session) -> None:
    """Recorded series, events, probes and the value of a body, joint or signal at a recorded time."""
    open_replay(ws, session)
    value = request(ws, "value_at", {"run_id": RUN_ID, "ref": ANGLE, "t": 0.0}).answer["payload"]["result"]
    # A time before the first recorded sample (t1 = 1 ms) reads that sample. The start 0.05 rad is placed exactly
    # (TST-038) at rest, so after one step the angle has moved by at most alpha dt^2 (semi-implicit Euler; the exact
    # motion is half that): alpha = m g d sin(0.05) / I = 1 * 9.81 * 0.25 * 0.04998 / (1/12) = 1.4710 rad/s^2,
    # dt = 1 ms, bound 1.471e-6 rad.
    assert (value["t"], value["unit"]) == (0.001, "rad")
    bound = 9.81 * 0.25 * np.sin(0.05) * 12.0 * 0.001**2
    assert abs(value["value"] - 0.05) <= bound
    # The link turns about world z, so its yaw from the reference configuration is the joint angle.
    pose = request(ws, "value_at", {"run_id": RUN_ID, "ref": "pose:link.yaw", "t": 0.0}).answer
    assert abs(pose["payload"]["result"]["value"] - 0.05) <= bound
    later = request(ws, "value_at", {"run_id": RUN_ID, "ref": ANGLE, "t": 0.0025}).answer["payload"]["result"]
    assert later["t"] == 0.002
    events = request(ws, "events", {"run_id": RUN_ID}).answer
    assert events["payload"]["result"] == {"run_id": RUN_ID, "events": []}
    joint = request(ws, "probe", {"run_id": RUN_ID, "probe": "joint", "name": "pivot"}).answer["payload"]["result"]
    assert [c["name"] for c in joint["columns"]][:2] == ["position", "velocity"]
    assert len(joint["times"]) == 24400
    body = request(ws, "probe", {"run_id": RUN_ID, "probe": "body", "name": "link"}).answer["payload"]["result"]
    assert [c["name"] for c in body["columns"]][:6] == ["x", "y", "z", "roll", "pitch", "yaw"]
    timeline = request(ws, "timeline", {"run_id": RUN_ID}).answer["payload"]["result"]
    assert timeline == {"run_id": RUN_ID, "entries": []}
    geometry = request(ws, "geometry", {"run_id": RUN_ID}).answer["payload"]["result"]
    assert [(b["body"], b["kind"]) for b in geometry["bodies"]] == [("link", "primitives")]
    assert geometry["bodies"][0]["shapes"][0]["sizes"] == {"radius": 0.01, "length": 0.5}
    missing = {"run_id": RUN_ID, "ref": "joint:nothing.position", "t": 0.0}
    assert_error(request(ws, "value_at", missing), "not_found", "FAC-011")


# ===== Reports, errors, files (FAC-012, FAC-014, FAC-015, FAC-017) ======

UNRUNNABLE = (ROOT / "test_cases" / "pendulum" / "P1_period.yaml").read_text(encoding="utf-8")
UNRUNNABLE = UNRUNNABLE.replace("version: 0.1.0", "version: 9.9.9")


@pytest.mark.req("FAC-012")
def test_conformance_fac_012(ws: ClientConnection, tmp_path: Path) -> None:
    """Pass/fail reports and regression diffs by suite id (the suite's one case stops before t = 0: no run)."""
    suite = tmp_path / "suite"
    suite.mkdir()
    (suite / "P1_period.yaml").write_text(UNRUNNABLE, encoding="utf-8")
    payload = {"design": "designs/pendulum/design.yaml", "suite": str(suite), "seed": 0, "mode": "none"}
    ran = request(ws, "run_suite", payload).answer["payload"]["result"]
    assert (ran["name"], ran["status"]) == ("suite", "error")
    report = request(ws, "report", {"suite_id": ran["suite_id"]}).answer["payload"]["result"]
    assert report["text"] == ran["report_text"]
    assert report["markdown"].startswith("#")
    diff = request(ws, "diff", {"suite_id": ran["suite_id"]}).answer["payload"]["result"]
    assert (diff["entries"], diff["text"]) == ([], None)
    assert_error(request(ws, "report", {"suite_id": "nothing"}), "not_found", "FAC-012")


@pytest.mark.req("FAC-014")
def test_conformance_fac_014(ws: ClientConnection, tmp_path: Path) -> None:
    """A run error reaches the client as a structured report: party, time, cause, requirement."""
    case = tmp_path / "P1_period.yaml"
    case.write_text(UNRUNNABLE, encoding="utf-8")
    payload = {"design": "designs/pendulum/design.yaml", "test_case": str(case), "seed": 0, "mode": "none"}
    exchange = request(ws, "run_case", payload)
    assert exchange.kinds() == ["ok"]
    result = exchange.answer["payload"]["result"]
    assert (result["status"], result["run_id"], result["archive"]) == ("error", None, None)
    error = result["error"]
    assert (error["party"], error["t"], error["requirement_id"]) == ("runner", 0.0, "INV-007")
    assert "9.9.9" in error["cause"]


@pytest.mark.req("FAC-017")
def test_conformance_fac_017(ws: ClientConnection, session: Session) -> None:
    """Design and test case files are read and written through the protocol; an invalid write is refused and the
    file is left as it was."""
    path = "designs/pendulum/design.yaml"
    read = request(ws, "read_file", {"path": path}).answer["payload"]["result"]
    assert read["kind"] == "design"
    before = (session.root / path).read_bytes()
    assert read["text"].encode() == before
    invalid = {"path": path, "text": read["text"].replace("mass: 1 kg", "mass: -1 kg")}
    assert_error(request(ws, "write_file", invalid), "validation_failed", "FAC-017")
    assert (session.root / path).read_bytes() == before
    written = request(ws, "write_file", {"path": path, "text": read["text"]}).answer
    assert written["payload"]["result"] == {"path": path, "kind": "design"}
    assert (session.root / path).read_bytes() == before
    assert_error(request(ws, "read_file", {"path": "../escape.yaml"}), "validation_failed", "FAC-017")


@pytest.mark.req("FAC-015")
def test_conformance_fac_015(session: Session, tmp_path: Path) -> None:
    """The run started through the protocol archives the same bytes as the same run from the CLI."""
    from mechsim.cli import main
    from mechsim.record.compare import archives_identical

    case = ROOT / "test_cases" / "pendulum" / "P1_period.yaml"
    code = main(["run", str(case), "--out", str(tmp_path)], None, io.StringIO(), io.StringIO())
    assert code == 0
    report = archives_identical(archive(session), tmp_path / f"{RUN_ID}.h5")
    assert report.identical, report.first_difference


@pytest.mark.req("FAC-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_conformance_fac_013() -> None:
    """A client reads integrity results and the load-case export through protocol requests."""
    raise NotImplementedError
