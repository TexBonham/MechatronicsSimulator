"""The browser client in `web/` (VIE-001, VIE-012, FAC-016, FAC-019; ruling 21): checked without a browser. The
client's protocol kinds against the schema, same-origin only, the vendored three.js against its recorded hash, a
coarse domain-logic screen, and the static route of the facade service. A browser-driven test is deferred (ruling
21); `web/README.md` carries the manual checklist."""

from __future__ import annotations

import asyncio
import hashlib
import http.client
import json
import re
import threading
from collections.abc import Iterator
from pathlib import Path

import pytest
from websockets.exceptions import InvalidStatus
from websockets.sync.client import connect

from mechsim.facade.server import FacadeServer, serve, static_file
from mechsim.facade.service import Facade, FacadeServices

ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "web"
VENDOR = WEB / "vendor" / "three.module.js"
SCHEMA = ROOT / "schemas" / "facade_protocol.json"
TOKEN = "web-client-token"
SENT = re.compile(r"""\bkind:\s*["']([A-Za-z_]+)["']""")
HANDLED = re.compile(r"""\bkind\s*===\s*["']([A-Za-z_]+)["']""")
URL_LITERAL = re.compile(r"""(?i)\b(?:https?|wss?):\/\/""")
IMPORT = re.compile(r"""\bimport\b[^;]*?\bfrom\s*["']([^"']+)["']|\bimport\s*\(\s*["']([^"']+)["']""")
LINK = re.compile(r"""\b(?:src|href)\s*=\s*["']([^"']+)["']""")
DOMAIN_TOKENS = ("Math.sqrt", "Math.atan2", "9.81")
WRITE_APIS = ("localStorage", "sessionStorage", "indexedDB", "showSaveFilePicker", "createObjectURL", "download")


def own_files(*suffixes: str) -> list[Path]:
    """The client's own files: everything under `web/` but the vendored third-party library."""
    return sorted(
        path
        for path in WEB.rglob("*")
        if path.is_file() and path.suffix in suffixes and (WEB / "vendor") not in path.parents
    )


def schema_kinds() -> tuple[set[str], set[str]]:
    """(request kinds, response and event kinds) from the generated protocol schema."""
    definitions = json.loads(SCHEMA.read_text(encoding="utf-8"))["$defs"]
    requests: set[str] = set()
    others: set[str] = set()
    for name, definition in definitions.items():
        kind = definition.get("properties", {}).get("kind", {}).get("const")
        if kind is None:
            continue
        if name.endswith("Request"):
            requests.add(kind)
        else:
            others.add(kind)
    return requests, others


def scan(pattern: re.Pattern[str], text: str) -> set[str]:
    return {match.group(1) for match in pattern.finditer(text)}


def client_text() -> str:
    return "\n".join(path.read_text(encoding="utf-8") for path in own_files(".js"))


# ===== (a) protocol kinds ==============================================


@pytest.mark.req("VIE-012")
@pytest.mark.req("FAC-005")
def test_every_kind_the_client_sends_is_a_request_kind_of_the_schema() -> None:
    requests, _ = schema_kinds()
    sent = scan(SENT, client_text())
    assert sent <= requests, f"not request kinds: {sorted(sent - requests)}"
    needed = {
        "list_designs",
        "run_case",
        "replay_open",
        "geometry",
        "timeline",
        "subscribe_series",
        "scrub",
        "pause",
        "step",
        "probe",
        "value_at",
        "list_runs",
        "advance",
    }
    assert needed <= sent, f"the client never sends {sorted(needed - sent)}"


@pytest.mark.req("VIE-012")
@pytest.mark.req("FAC-019")
def test_every_kind_the_client_handles_is_a_response_or_event_kind_of_the_schema() -> None:
    _, others = schema_kinds()
    handled = scan(HANDLED, client_text())
    assert handled <= others, f"not response or event kinds: {sorted(handled - others)}"
    assert {"ok", "error", "tick_batch", "run_progress", "run_ended", "timeline_event"} <= handled


@pytest.mark.req("VIE-012")
def test_the_kind_scan_catches_an_unknown_or_misplaced_kind() -> None:
    requests, others = schema_kinds()
    text = 'send({ kind: "teleport" }); send({ kind: \'ok\' }); if (m.kind === "replay_open") {}'
    assert scan(SENT, text) - requests == {"teleport", "ok"}
    assert scan(HANDLED, text) - others == {"replay_open"}


@pytest.mark.req("FAC-019")
def test_the_tick_batch_decoder_checks_the_frame_length_from_the_header() -> None:
    """The decoder rejects a header whose byte length disagrees with (1 + series) x samples x 8 and reads the frame
    little-endian (FAC-019); checked in the source because the client runs only in a browser (ruling 21)."""
    text = (WEB / "protocol.js").read_text(encoding="utf-8")
    assert "1 + header.series.length" in text
    assert "header.sample_count * 8" in text
    assert "header.byte_length !== expected || buffer.byteLength !== expected" in text
    assert "getFloat64(" in text
    assert ", true)" in text
    assert 'binaryType = "arraybuffer"' in text


def app_function(name: str) -> str:
    """The source of one top-level function of `web/app.js`, up to the next top-level declaration."""
    text = (WEB / "app.js").read_text(encoding="utf-8")
    found = re.search(
        rf"^(?:async )?function {name}\(.*?(?=^(?://|(?:async function|function|const|element|client)\b))",
        text,
        re.M | re.S,
    )
    assert found is not None, f"app.js has no function {name}"
    return found.group(0)


@pytest.mark.req("VIE-009")
@pytest.mark.req("FAC-016")
def test_playback_advances_the_facade_cursor_and_keeps_no_clock() -> None:
    """Ruling 22: a playing frame sends `advance` with the frame interval and draws the cursor the facade answers;
    the play loop neither scrubs nor adds to a simulated time of its own."""
    tick = app_function("tick")
    assert scan(SENT, tick) == {"advance"}
    assert "showCursor(cursor)" in tick
    assert "cursor.index + 1 >= cursor.samples" in tick
    play = app_function("play") + app_function("frameInterval") + tick
    assert "cursorTo(" not in play
    assert "clock" not in (WEB / "app.js").read_text(encoding="utf-8")
    assert "cursor.t +" not in play
    assert "t_last" not in play


@pytest.mark.req("VIE-012")
def test_the_run_list_comes_from_list_runs() -> None:
    """Ruling 22: the runs shown are the facade's `list_runs`, refreshed after a run and an opened replay."""
    assert scan(SENT, app_function("listRuns")) == {"list_runs"}
    assert "listRuns()" in app_function("runCase")
    assert "listRuns()" in app_function("openReplay")
    assert "addRun" not in (WEB / "app.js").read_text(encoding="utf-8")


@pytest.mark.req("VIE-009")
@pytest.mark.req("FAC-016")
def test_the_drawn_pose_is_the_cursor_sample() -> None:
    """Ruling 22: a decimated window can drop the cursor's sample; the pose is drawn only from a window sample whose
    time equals the cursor's, the window is narrowed until it holds one, and `value_at` reads it otherwise."""
    exact = app_function("cursorSample")
    assert "poses.times[index] !== t" in exact
    pose = app_function("poseAt")
    assert "cursorSample(poses, t)" in pose
    assert "state.span / 2" in pose
    assert scan(SENT, pose) == {"value_at"}
    place = app_function("placeBodies")
    assert "poseAt(body, t)" in place
    assert "sampleAt(" not in place


# ===== (b) own origin only =============================================


@pytest.mark.req("VIE-012")
def test_the_client_references_no_host_but_its_own_origin() -> None:
    files = own_files(".js", ".html", ".css")
    assert {path.name for path in files} >= {"index.html", "app.js", "protocol.js", "replay.js", "timeline.js"}
    for path in files:
        text = path.read_text(encoding="utf-8")
        assert URL_LITERAL.search(text) is None, f"{path.name} names a host"
        targets = [m.group(1) or m.group(2) for m in IMPORT.finditer(text)] + [m.group(1) for m in LINK.finditer(text)]
        for target in targets:
            assert target.startswith("./"), f"{path.name} loads {target!r}, not a relative path"


@pytest.mark.req("VIE-012")
def test_the_origin_scan_catches_a_remote_host() -> None:
    assert URL_LITERAL.search('import * as T from "https://cdn.example/three.js";') is not None
    assert URL_LITERAL.search('new WebSocket("ws://10.0.0.1:9000/ws")') is not None
    assert URL_LITERAL.search('new URL("/ws", page)') is None
    imports = [m.group(1) or m.group(2) for m in IMPORT.finditer('import { a } from "three";\nimport("/x.js");')]
    assert imports == ["three", "/x.js"]


# ===== (c) vendored three.js ===========================================


@pytest.mark.req("VIE-001")
def test_the_vendored_three_js_matches_its_recorded_hash() -> None:
    readme = (WEB / "README.md").read_text(encoding="utf-8")
    recorded = re.search(r"SHA-256[^0-9a-f]*([0-9a-f]{64})", readme)
    assert recorded is not None, "web/README.md records no SHA-256 for three.js"
    assert hashlib.sha256(VENDOR.read_bytes()).hexdigest() == recorded.group(1)
    version = re.search(r"three\.js\D*(\d+\.\d+\.\d+)", readme)
    assert version is not None, "web/README.md records no three.js version"
    revision = re.search(r"const REVISION = '(\d+)'", VENDOR.read_text(encoding="utf-8"))
    assert revision is not None
    assert version.group(1).split(".")[1] == revision.group(1)


@pytest.mark.req("VIE-001")
@pytest.mark.req("VIE-015")
def test_the_replay_draws_bodies_from_the_facade_geometry_with_three_js() -> None:
    """VIE-001 without a browser: the replay imports only the vendored three.js, draws every shape kind the facade's
    `geometry` names by its sizes, and poses bodies from the recorded `pose:` series."""
    text = (WEB / "replay.js").read_text(encoding="utf-8")
    assert [m.group(1) or m.group(2) for m in IMPORT.finditer(text)] == ["./vendor/three.module.js"]
    for shape in ('"box"', '"sphere"', '"cylinder"', '"capsule"', '"plane"', "vertices"):
        assert shape in text
    assert "pose:${body}.${field}" in text
    assert '"x", "y", "z", "roll", "pitch", "yaw"' in text
    app = (WEB / "app.js").read_text(encoding="utf-8")
    assert 'kind: "geometry"' in app
    assert "client.window(" in app


# ===== (d) domain logic, files, design names ===========================


@pytest.mark.req("FAC-016")
def test_coarse_domain_logic_screen() -> None:
    """A coarse domain-logic screen, not a proof: no client file uses a square root, an atan2 or standard gravity.
    The vendored three.js is excluded: it is a pinned third-party rendering library checked by its hash."""
    for path in own_files(".js", ".html", ".css"):
        text = path.read_text(encoding="utf-8")
        for token in DOMAIN_TOKENS:
            assert token not in text, f"{path.name} contains {token}"


@pytest.mark.req("VIE-017")
def test_the_client_writes_no_files() -> None:
    text = client_text() + (WEB / "index.html").read_text(encoding="utf-8")
    for api in WRITE_APIS:
        assert api not in text, f"the client uses {api}"
    assert "write_file" not in scan(SENT, text)


@pytest.mark.req("VIE-012")
def test_the_client_names_no_design() -> None:
    """Invariant 2: no design or test case folder name appears in the client."""
    names = {path.name for path in (ROOT / "designs").iterdir() if path.is_dir()}
    names |= {path.name for path in (ROOT / "test_cases").iterdir() if path.is_dir()}
    assert len(names) > 0
    for path in own_files(".js", ".html", ".css"):
        text = path.read_text(encoding="utf-8").lower()
        for name in names:
            assert name.lower() not in text, f"{path.name} names {name}"


# ===== static route ====================================================


@pytest.mark.req("VIE-012")
def test_the_route_maps_paths_to_files_under_web() -> None:
    assert static_file(WEB, "/") == (WEB / "index.html").resolve()
    assert static_file(WEB, "/?token=abc") == (WEB / "index.html").resolve()
    assert static_file(WEB, "/index.html") == (WEB / "index.html").resolve()
    assert static_file(WEB, "/app.js") == (WEB / "app.js").resolve()
    assert static_file(WEB, "/style.css") == (WEB / "style.css").resolve()
    assert static_file(WEB, "/vendor/three.module.js") == VENDOR.resolve()


@pytest.mark.req("VIE-012")
@pytest.mark.parametrize(
    "path",
    [
        "/../pyproject.toml",
        "/vendor/../app.js",
        "/%2e%2e/pyproject.toml",
        "/vendor/%2e%2e/%2e%2e/pyproject.toml",
        "/..%2fpyproject.toml",
        "//etc/passwd",
        "/etc/passwd",
        "/" + str(ROOT / "pyproject.toml"),
        "/C:/Windows/win.ini",
        "/vendor\\..\\..\\pyproject.toml",
        "/app.js%00.css",
        "/./app.js",
        "/vendor//three.module.js",
        "/vendor/",
        "/vendor",
        "/README.md",
        "/missing.js",
        "/vendor/missing.js",
        "/vendor/sub/file.js",
        "/protocol.py",
        "relative.js",
        "",
    ],
)
def test_the_route_refuses_traversal_absolute_and_unknown_paths(path: str) -> None:
    assert static_file(WEB, path) is None


@pytest.mark.req("VIE-012")
def test_the_route_refuses_a_link_out_of_the_web_root(tmp_path: Path) -> None:
    web = tmp_path / "web"
    (web / "vendor").mkdir(parents=True)
    secret = tmp_path / "secret.js"
    secret.write_text("secret", encoding="utf-8")
    (web / "index.html").write_text("page", encoding="utf-8")
    (web / "leak.js").symlink_to(secret)
    (web / "vendor" / "leak.js").symlink_to(secret)
    (web / "folder.js").mkdir()
    assert static_file(web, "/") == (web / "index.html").resolve()
    assert static_file(web, "/leak.js") is None
    assert static_file(web, "/vendor/leak.js") is None
    assert static_file(web, "/folder.js") is None


class Running:
    def __init__(self, files_root: Path) -> None:
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        facade = Facade(FacadeServices(files_root=files_root), files_root / "runs")
        future = asyncio.run_coroutine_threadsafe(serve(facade, port=0, token=TOKEN), self.loop)
        self.server: FacadeServer = future.result(30)

    def get(self, path: str) -> tuple[int, str, bytes]:
        connection = http.client.HTTPConnection("127.0.0.1", self.server.port, timeout=10)
        try:
            connection.request("GET", path)
            response = connection.getresponse()
            return response.status, response.getheader("Content-Type") or "", response.read()
        finally:
            connection.close()

    def close(self) -> None:
        asyncio.run_coroutine_threadsafe(self.server.close(), self.loop).result(30)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(30)


@pytest.fixture
def running(tmp_path: Path) -> Iterator[Running]:
    service = Running(tmp_path)
    try:
        yield service
    finally:
        service.close()


@pytest.mark.req("VIE-012")
def test_the_service_serves_the_client_and_the_socket_on_one_port(running: Running) -> None:
    status, content_type, body = running.get(f"/?token={TOKEN}")
    assert (status, content_type) == (200, "text/html; charset=utf-8")
    assert body == (WEB / "index.html").read_bytes()
    status, content_type, body = running.get("/vendor/three.module.js")
    assert (status, content_type) == (200, "text/javascript; charset=utf-8")
    assert hashlib.sha256(body).digest() == hashlib.sha256(VENDOR.read_bytes()).digest()
    status, content_type, _ = running.get("/style.css")
    assert (status, content_type) == (200, "text/css; charset=utf-8")
    for refused in ("/missing.js", "/README.md", "/../pyproject.toml", "/%2e%2e/pyproject.toml", "//etc/passwd"):
        assert running.get(refused)[0] == 404, refused
    assert running.server.page_url() == f"http://127.0.0.1:{running.server.port}/?token={TOKEN}"
    with connect(f"ws://127.0.0.1:{running.server.port}/ws?token={TOKEN}") as socket:
        socket.send(json.dumps({"version": 1, "message_id": "m1", "kind": "list_designs", "payload": {}}))
        answer = json.loads(socket.recv(timeout=10))
    assert (answer["kind"], answer["payload"]["request_id"]) == ("ok", "m1")
    with pytest.raises(InvalidStatus) as raised, connect(f"ws://127.0.0.1:{running.server.port}/ws?token=wrong"):
        pass
    assert raised.value.response.status_code == 401
