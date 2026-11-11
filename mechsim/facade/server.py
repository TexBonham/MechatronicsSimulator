"""The facade over WebSocket (ADR 0015 proposed; FAC-003, FAC-005, FAC-006, FAC-019, FAC-020). One `Facade` behind
every connection; a client connects to `ws://host:port/?token=<token>` and speaks only protocol messages. Requests map
one to one onto `Facade` methods; a run goes to a worker thread and streams `run_progress` and `tick_batch` events
(the header text frame, then one binary frame) before `run_ended` and the `ok` answer."""

from __future__ import annotations

import asyncio
import hmac
import json
import threading
import uuid
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from http import HTTPStatus
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, unquote, urlsplit

import numpy as np
from pydantic import BaseModel, ValidationError
from websockets.asyncio.server import Server, ServerConnection
from websockets.asyncio.server import serve as ws_serve
from websockets.datastructures import Headers
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Request, Response

from mechsim.facade.live import Batch, Subscription
from mechsim.facade.protocol import (
    MESSAGE_ADAPTER,
    PROTOCOL_VERSION,
    ErrorCode,
    ErrorPayload,
    ErrorResponse,
    Message,
    OkPayload,
    OkResponse,
    RunEndedEvent,
    RunEndedPayload,
    RunEndReason,
    RunProgressEvent,
    RunProgressPayload,
    TickBatchEvent,
    TickBatchPayload,
    dump_message,
)
from mechsim.facade.service import Facade, FacadeError, RunResultSummary

REQUEST_KINDS = frozenset(
    {
        "list_designs",
        "validate_design",
        "run_case",
        "run_suite",
        "replay_open",
        "subscribe_series",
        "scrub",
        "pause",
        "step",
        "advance",
        "list_runs",
        "events",
        "timeline",
        "geometry",
        "value_at",
        "probe",
        "report",
        "diff",
        "read_file",
        "write_file",
    }
)
Frame = str | bytes
WEB_ROOT = Path(__file__).resolve().parents[2] / "web"
STATIC_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
}


def _message_id() -> str:
    return uuid.uuid4().hex


def error_message(
    request_id: str | None,
    code: ErrorCode,
    message: str,
    requirement_id: str | None = None,
    run_id: str | None = None,
    report: dict[str, Any] | None = None,
) -> str:
    payload = ErrorPayload(
        request_id=request_id,
        code=code,
        message=message,
        requirement_id=requirement_id,
        run_id=run_id,
        report=report,
    )
    return dump_message(
        ErrorResponse(version=PROTOCOL_VERSION, message_id=_message_id(), kind="error", payload=payload)
    )


def ok_message(request_id: str, result: BaseModel | dict[str, Any]) -> str:
    if isinstance(result, BaseModel):
        return ok_message(request_id, result.model_dump(mode="json"))
    payload = OkPayload(request_id=request_id, result=result)
    return dump_message(OkResponse(version=PROTOCOL_VERSION, message_id=_message_id(), kind="ok", payload=payload))


def tick_batch_frames(run_id: str, batch: Batch) -> tuple[str, bytes]:
    """The header and its binary frame: (1 + series) x samples float64 little-endian, time column first (FAC-019)."""
    columns = [list(batch.times), *[list(column) for column in batch.columns]]
    body = np.asarray(columns, dtype="<f8").tobytes()
    header = TickBatchPayload(
        run_id=run_id,
        sequence=batch.sequence,
        t_start=batch.times[0],
        t_end=batch.times[-1],
        series=list(batch.series),
        dtype="float64",
        byte_order="little",
        sample_count=len(batch.times),
        byte_length=len(body),
    )
    text = dump_message(
        TickBatchEvent(version=PROTOCOL_VERSION, message_id=_message_id(), kind="tick_batch", payload=header)
    )
    return text, body


def classify(text: str | bytes) -> Message | str:
    """The parsed request, or the `error` frame answering it: unparseable JSON or a binary frame is
    `invalid_message` with a null request id; a wrong version `unsupported_version` naming both versions (FAC-003);
    a kind the protocol does not have, or a service-side kind, `unknown_kind`; a failed validation
    `invalid_message` (FAC-020)."""
    if isinstance(text, bytes):
        return error_message(None, "invalid_message", "requests are JSON text frames", "FAC-019")
    try:
        data = json.loads(text)
    except ValueError as error:
        return error_message(None, "invalid_message", f"not JSON: {error}", "FAC-002")
    if not isinstance(data, dict):
        return error_message(None, "invalid_message", "a message is a JSON object", "FAC-002")
    request_id: str | None = None
    identifier = data.get("message_id")
    if isinstance(identifier, str) and identifier != "":
        request_id = identifier
    version = data.get("version")
    if type(version) is not int or version != PROTOCOL_VERSION:
        return error_message(
            request_id,
            "unsupported_version",
            f"unsupported protocol version {version}; this service speaks version {PROTOCOL_VERSION}",
            "FAC-003",
        )
    kind = data.get("kind")
    if not isinstance(kind, str) or kind not in REQUEST_KINDS:
        return error_message(request_id, "unknown_kind", f"unknown request kind {kind!r}", "FAC-020")
    try:
        return MESSAGE_ADAPTER.validate_json(text)
    except ValidationError as error:
        requirement = "FAC-002"
        if kind == "subscribe_series":
            requirement = "FAC-018"
        detail = "; ".join(f"{'.'.join(str(p) for p in item['loc'])}: {item['msg']}" for item in error.errors())
        return error_message(request_id, "invalid_message", f"{kind}: {detail}", requirement)


def run_end_reason(summary: RunResultSummary) -> RunEndReason:
    """The runner's structured end reason (`RunReport.reason`, scored as the case's end reason)."""
    reasons: dict[str, RunEndReason] = {
        "end_time": "end_time",
        "message": "message",
        "controller_state": "controller_state",
        "max_duration": "max_duration",
        "run_error": "run_error",
    }
    if summary.end_reason is None or summary.end_reason not in reasons:
        return "run_error"
    return reasons[summary.end_reason]


class _Listener:
    """Runs on the worker thread; every frame is handed to the connection's loop in order."""

    def __init__(self, push: Callable[[Frame], None], run_id: str, request_id: str) -> None:
        self._push = push
        self._run_id = run_id
        self._request_id = request_id
        self.stream_errors: list[str] = []

    def progress(self, t: float) -> None:
        payload = RunProgressPayload(run_id=self._run_id, t=t, t_end=None)
        self._push(
            dump_message(
                RunProgressEvent(
                    version=PROTOCOL_VERSION, message_id=_message_id(), kind="run_progress", payload=payload
                )
            )
        )

    def batch(self, batch: Batch) -> None:
        header, body = tick_batch_frames(self._run_id, batch)
        self._push(header)
        self._push(body)

    def failed(self, message: str) -> None:
        """A stream that cannot be served is reported in the run's one answer, never as its own answer."""
        self.stream_errors.append(message)


@dataclass
class _Connection:
    socket: ServerConnection
    queue: asyncio.Queue[Frame | None]
    live: dict[str, list[Subscription]] = field(default_factory=dict)


class FacadeServer:
    """A running service: `port`, `url`, `close()` (every connection closed with 1001 going away)."""

    def __init__(self, facade: Facade, token: str, web_root: Path = WEB_ROOT) -> None:
        self.facade = facade
        self._token = token
        self.web_root = web_root
        self._server: Server | None = None
        self._run_lock = threading.Lock()
        self._facade_lock = threading.Lock()
        self.host = "127.0.0.1"

    @property
    def port(self) -> int:
        if self._server is None:
            raise RuntimeError("the server is not started")
        sockets = list(self._server.sockets)
        port: int = sockets[0].getsockname()[1]
        return port

    def url(self) -> str:
        return f"ws://{self.host}:{self.port}/?token={self._token}"

    def page_url(self) -> str:
        """The browser client's address; the page reads the token and opens `/ws?token=` on the same origin."""
        return f"http://{self.host}:{self.port}/?token={self._token}"

    def _authorize(self, connection: ServerConnection, request: Request) -> Response | None:
        """A WebSocket upgrade needs the token (FAC-005); a plain GET is the static route of `web/` (VIE-012)."""
        if request.headers.get("Upgrade", "").lower() != "websocket":
            return static_response(self.web_root, request.path)
        offered = parse_qs(urlsplit(request.path).query).get("token", [""])[0]
        if hmac.compare_digest(offered.encode(), self._token.encode()):
            return None
        return connection.respond(HTTPStatus.UNAUTHORIZED, "a valid ?token= is required\n")

    async def start(self, host: str, port: int) -> None:
        self.host = host
        self._server = await ws_serve(self._handle, host, port, process_request=self._authorize, max_size=2**24)

    async def close(self) -> None:
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()

    async def _handle(self, socket: ServerConnection) -> None:
        connection = _Connection(socket, asyncio.Queue())
        writer = asyncio.create_task(self._write(connection))
        tasks: set[asyncio.Task[None]] = set()
        try:
            async for frame in socket:
                task = asyncio.create_task(self._answer(connection, frame))
                tasks.add(task)
                task.add_done_callback(tasks.discard)
        except ConnectionClosed:
            pass
        finally:
            for task in list(tasks):
                await asyncio.gather(task, return_exceptions=True)
            await connection.queue.put(None)
            await writer

    async def _write(self, connection: _Connection) -> None:
        while True:
            frame = await connection.queue.get()
            if frame is None:
                return
            try:
                await connection.socket.send(frame)
            except ConnectionClosed:
                return

    async def _answer(self, connection: _Connection, frame: Frame) -> None:
        parsed = classify(frame)
        if isinstance(parsed, str):
            await connection.queue.put(parsed)
            return
        request_id = parsed.message_id
        try:
            await self._dispatch(connection, parsed)
        except FacadeError as error:
            await connection.queue.put(error_message(request_id, error.code, error.message, error.requirement_id))
        except Exception as error:
            await connection.queue.put(error_message(request_id, "internal", f"{type(error).__name__}: {error}"))

    async def _call(self, function: Callable[[], Any]) -> Any:
        def locked() -> Any:
            with self._facade_lock:
                return function()

        return await asyncio.to_thread(locked)

    def _path(self, text: str) -> Path:
        path = Path(text)
        if path.is_absolute():
            return path
        return self.facade.services.files_root / path

    async def _dispatch(self, connection: _Connection, message: Message) -> None:
        request_id = message.message_id
        facade = self.facade
        kind = message.kind
        payload: Any = message.payload
        handlers: dict[str, Callable[[], Awaitable[BaseModel | dict[str, Any] | str | None]]] = {
            "list_designs": lambda: self._call(lambda: facade.list_designs()),
            "validate_design": lambda: self._call(lambda: facade.validate(self._path(payload.design))),
            "run_case": lambda: self._run_case(connection, request_id, payload),
            "run_suite": lambda: self._run_suite(payload),
            "replay_open": lambda: self._call(lambda: facade.replay_open(self._path(payload.archive))),
            "subscribe_series": lambda: self._subscribe(connection, request_id, payload),
            "scrub": lambda: self._call(lambda: facade.scrub(payload.run_id, payload.t)),
            "pause": lambda: self._call(lambda: facade.pause(payload.run_id, payload.paused)),
            "step": lambda: self._call(lambda: facade.step(payload.run_id, payload.direction)),
            "advance": lambda: self._call(lambda: facade.advance(payload.run_id, payload.dt)),
            "list_runs": lambda: self._call(lambda: facade.list_runs()),
            "events": lambda: self._call(lambda: facade.events(payload.run_id)),
            "timeline": lambda: self._call(lambda: facade.timeline(payload.run_id)),
            "geometry": lambda: self._call(lambda: facade.geometry(payload.run_id)),
            "value_at": lambda: self._call(lambda: facade.value_at(payload.run_id, payload.ref, payload.t)),
            "probe": lambda: self._call(lambda: facade.probe(payload.run_id, payload.probe, payload.name)),
            "report": lambda: self._call(lambda: facade.report(payload.suite_id)),
            "diff": lambda: self._call(lambda: facade.diff(payload.suite_id)),
            "read_file": lambda: self._call(lambda: facade.read_file(Path(payload.path))),
            "write_file": lambda: self._call(lambda: facade.write_file(Path(payload.path), payload.text)),
        }
        result = await handlers[kind]()
        if isinstance(result, str):
            await connection.queue.put(result)
        elif result is not None:
            await connection.queue.put(ok_message(request_id, result))

    async def _subscribe(self, connection: _Connection, request_id: str, payload: Any) -> dict[str, Any] | None:
        """On an open replay: the decimated window as one batch, after the `ok`. Otherwise a live subscription held
        for the run of that id (`<case id>-seed<seed>`) on this connection."""
        subscription = Subscription(tuple(payload.series), payload.t_start, payload.t_end, payload.points)
        if not self.facade.is_open(payload.run_id):
            connection.live.setdefault(payload.run_id, []).append(subscription)
            return {"run_id": payload.run_id, "live": True, "sample_count": 0}
        window = await self._call(
            lambda: self.facade.series(
                payload.run_id, list(payload.series), payload.t_start, payload.t_end, payload.points
            )
        )
        result = {"run_id": payload.run_id, "live": False, "sample_count": len(window.times), "units": window.units}
        await connection.queue.put(ok_message(request_id, result))
        if len(window.times) > 0:
            batch = Batch(0, tuple(window.series), tuple(window.times), tuple(tuple(c) for c in window.values))
            header, body = tick_batch_frames(payload.run_id, batch)
            await connection.queue.put(header)
            await connection.queue.put(body)
        return None

    async def _run_case(self, connection: _Connection, request_id: str, payload: Any) -> str:
        if not self._run_lock.acquire(blocking=False):
            raise FacadeError("busy", "a run is already in progress on this service", "FAC-006")
        try:
            loop = asyncio.get_running_loop()
            case = self._path(payload.test_case)
            design = self._path(payload.design)

            def push(frame: Frame) -> None:
                loop.call_soon_threadsafe(connection.queue.put_nowait, frame)

            run_id = case.stem
            subscriptions: list[Subscription] = []
            try:
                run_id = self.facade.run_id_of(case, design)
                subscriptions = connection.live.pop(run_id, [])
            except FacadeError as error:
                if error.field != "case":
                    raise
            listener = _Listener(push, run_id, request_id)
            summary: RunResultSummary = await asyncio.to_thread(
                lambda: self.facade.run_case(
                    case,
                    design,
                    None,
                    None,
                    False,
                    payload.seed,
                    payload.mode,
                    listener,
                    subscriptions,
                )
            )
        finally:
            self._run_lock.release()
        if summary.run_id is not None:
            message = "run ended"
            requirement: str | None = None
            if summary.error is not None:
                message = f"{summary.error.party}: {summary.error.cause}"
                requirement = summary.error.requirement_id
            end = RunEndedPayload(
                run_id=summary.run_id,
                t=summary.end_time or 0.0,
                reason=run_end_reason(summary),
                message=message,
                requirement_id=requirement,
            )
            await connection.queue.put(
                dump_message(
                    RunEndedEvent(version=PROTOCOL_VERSION, message_id=_message_id(), kind="run_ended", payload=end)
                )
            )
        return self._run_answer(request_id, summary, listener.stream_errors)

    def _run_answer(self, request_id: str, summary: RunResultSummary, stream_errors: list[str]) -> str:
        """The run's one terminal answer: `error` (run_error, with the run id and the structured report) when the run
        ended in error or never started, else `ok` with the summary and any stream errors (FAC-014, FAC-020)."""
        error = summary.error
        if error is not None:
            name = summary.run_id or summary.case_id
            return error_message(
                request_id,
                "run_error",
                f"run {name}: {error.party} at {error.t:.12g} s: {error.cause}",
                error.requirement_id,
                summary.run_id,
                error.model_dump(mode="json"),
            )
        result = summary.model_dump(mode="json")
        result["stream_errors"] = list(stream_errors)
        return ok_message(request_id, result)

    async def _run_suite(self, payload: Any) -> BaseModel:
        if not self._run_lock.acquire(blocking=False):
            raise FacadeError("busy", "a run is already in progress on this service", "FAC-006")
        try:
            suite = self._path(payload.suite)
            design = self._path(payload.design)
            summary: BaseModel = await asyncio.to_thread(lambda: self.facade.run_suite(suite, design))
        finally:
            self._run_lock.release()
        return summary


def static_file(web_root: Path, request_path: str) -> Path | None:
    """The file under `web_root` a GET path names: `/` is `index.html`; top-level `*.html`, `*.js`, `*.css` and
    `vendor/*` only. None for anything else, `..`, an absolute or encoded-absolute path, or a path resolving outside
    `web_root` (VIE-012)."""
    path = unquote(urlsplit(request_path).path)
    if not path.startswith("/") or path.startswith("//") or "\\" in path or "\x00" in path:
        return None
    relative = path[1:]
    if relative == "":
        relative = "index.html"
    parts = relative.split("/")
    for part in parts:
        if part in ("", ".", "..") or ":" in part:
            return None
    allowed = False
    if len(parts) == 1 and Path(parts[0]).suffix in STATIC_TYPES:
        allowed = True
    if len(parts) == 2 and parts[0] == "vendor":
        allowed = True
    if not allowed:
        return None
    root = web_root.resolve()
    candidate = (root / Path(*parts)).resolve()
    if not candidate.is_relative_to(root) or not candidate.is_file():
        return None
    return candidate


def static_response(web_root: Path, request_path: str) -> Response:
    """200 with the file and its content type, or 404; never a file outside `web_root`."""
    found = static_file(web_root, request_path)
    if found is None:
        body = b"not found\n"
        headers = Headers([("Content-Type", "text/plain; charset=utf-8"), ("Content-Length", str(len(body)))])
        return Response(HTTPStatus.NOT_FOUND.value, HTTPStatus.NOT_FOUND.phrase, headers, body)
    body = found.read_bytes()
    content_type = STATIC_TYPES.get(found.suffix, "application/octet-stream")
    headers = Headers(
        [
            ("Content-Type", content_type),
            ("Content-Length", str(len(body))),
            ("Cache-Control", "no-store"),
            ("X-Content-Type-Options", "nosniff"),
        ]
    )
    return Response(HTTPStatus.OK.value, HTTPStatus.OK.phrase, headers, body)


async def serve(
    facade: Facade, host: str = "127.0.0.1", port: int = 0, *, token: str, web_root: Path = WEB_ROOT
) -> FacadeServer:
    """Starts the service; `port=0` picks a free port (`FacadeServer.port`). The token is required (FAC-005). Plain
    GETs on the same port serve the browser client from `web_root` (VIE-012)."""
    if token == "":
        raise ValueError("the facade service needs a non-empty token")
    server = FacadeServer(facade, token, web_root)
    await server.start(host, port)
    return server


__all__ = [
    "FacadeServer",
    "classify",
    "error_message",
    "ok_message",
    "serve",
    "static_file",
    "static_response",
    "tick_batch_frames",
]
