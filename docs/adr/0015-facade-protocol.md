# ADR 0015: Facade protocol, JSON over WebSocket with binary series frames

Date: 2026-11-03
Status: Accepted (owner ruling 2026-11-04: option A, JSON over WebSocket with binary series frames)
Closes rulings: none (closes the protocol open items of ADR 0006 and ADR 0007)
Supersedes: none

## Context
ADR 0006 makes one serializable facade the only way any client drives the simulator; ADR 0007 puts a browser client in
`web/` over a local Python service, replaceable later by a standalone app, with a versioned protocol, a schema, a
client-independent conformance suite, streaming and decimation in the service, and clients holding no domain logic.
Replay reads run archives, never live engine state (ADR 0006). Whether designs and results are shared across a team is
an open question in the requirements doc, so the protocol must not assume a single machine forever, but nothing today
needs more than localhost. Schemas are generated from pydantic models (ADR 0012, L4).

## Options

| Option | Shape |
|---|---|
| A | JSON messages over WebSocket for control and events; time series as a JSON header frame (series names, dtype, sample count) followed by one little-endian float64 binary frame |
| B | gRPC with protobuf |
| C | HTTP REST for requests plus server-sent events for streams |
| D | A Python-only API with no wire protocol |

| Criterion | A | B | C | D |
|---|---|---|---|---|
| Browser client without a build step | Native `WebSocket`, `JSON.parse`, `Float64Array` on an `ArrayBuffer` | Needs grpc-web, a proxy and generated stubs | Native `fetch` and `EventSource` | None; the browser cannot call Python |
| Binary efficiency for series | Raw float64, 8 bytes per sample, no encoding | Packed repeated doubles, good | Text only (SSE); base64 or JSON numbers, 2x to 3x larger | In-process, best |
| Schema from pydantic (L4) | Direct: `TypeAdapter.json_schema()` | Second schema language (`.proto`), kept in step by hand or a generator | Direct for bodies; OpenAPI for routes | Type hints only; no wire schema |
| Versioning story | Integer version in every message, checked per message | Field numbers and package versions; strong but tooling-bound | URL or header versioning; events unversioned unless added | Python package version; no wire contract |
| Local-only security posture | Bind to localhost, token in the connection URL | Same binding; TLS and metadata tokens add setup | Same binding; token per request; CORS to manage | No socket, nothing to expose |
| Testability by a client-independent conformance suite | One socket, scripted message sequences, schema-checked | Needs generated stubs in the suite | Two channels to correlate (requests and events) | Not client-independent; a standalone app could not run it |

## Decision
Recommended: option A.

| Element | Decision |
|---|---|
| Transport | One WebSocket per client to the local service |
| Text frames | One JSON message each, validated against `schemas/facade_protocol.json` |
| Envelope | `version`, `message_id`, `kind`, `payload`; unknown keys and unknown kinds rejected |
| Version | Integer, currently 1, in every message; exact match; a mismatch is rejected with an error naming both versions (FAC-003) |
| Requests | `list_designs`, `validate_design`, `run_case`, `run_suite`, `replay_open`, `subscribe_series`, `scrub`, `pause` (with `paused` true or false, so resume is the same kind), `step` (forward or backward) |
| Responses | `ok` with the request id and a JSON result object; `error` with the request id (null when unparseable), a code, a message and the violated requirement ID where one applies (FAC-020) |
| Events | `run_progress`, `run_ended` with its reason (the runner's end reasons: end time, message, controller state, max duration, run error; a run error carries the structured report with party, time and cause; amended 2026-11-10 at the F1 review), `tick_batch`, `timeline_event` (command, fault, safety event, criterion failure, label verbatim) |
| Time | Simulated time in seconds as a finite float at or above zero, always with the run id; no wall-clock time in any message |
| Decimation request | The client sends `subscribe_series` with series names, a window `t_start` before `t_end`, and `points` of at least one per series; the service decimates and never changes the recording (FAC-007, FAC-018) |
| Binary frames | A `tick_batch` text message (run id, sequence, window, series, `dtype` float64, `byte_order` little, `sample_count`, `byte_length`) is followed by exactly one binary frame of (1 + series count) x sample count x 8 bytes: the time column, then one column per series in header order (FAC-019) |
| Security | Service binds to localhost only; a per-session token in the connection URL |
| Models | `mechsim/facade/protocol.py`, pydantic, `extra="forbid"`, strict types, no domain logic |
| Conformance | `tests/acceptance/test_facade_conformance_stubs.py`, run against the service as a client would; every client must pass it (FAC-005) |

## Consequences
- The browser client needs no build step and no protocol library; it parses JSON and wraps binary frames as
  `Float64Array` views.
- The schema is generated from the models and staleness-tested, so the protocol and the Python service cannot drift.
- Every client sees the same messages; a standalone app either embeds the browser client or speaks the same socket and
  passes the same suite (VIE-013).
- A header and its binary frame must stay paired on one connection; the service sends them back to back.
- Any later remote or team mode needs authentication and transport security this ADR does not provide.
- Series values cross the wire as float64 in SI units; units are fixed by the series name's recorded unit (REC-018),
  not repeated per sample.

## Open items
- Transport library (for example the WebSocket server package): not in this ADR's scope; FAC module plan, milestone 1.
- Authentication beyond localhost and the token: owner, tied to the team-sharing question.
- Whether the standalone app wraps the browser client or speaks the protocol natively: owner (ADR 0007 open item).
- Decimation algorithm (stride, min-max per bucket) and back-pressure (REC open parameter): FAC module plan, milestone 1.
- Probe value and archive-query request kinds (FAC-011): added with the replay service, milestone 1.
- Report and regression-diff request kinds (FAC-012): the protocol must add them, for example `get_report` (run or suite
  id) and `get_regression_diff` (two run ids), answered by `ok` with the report or diff as the result; milestone 2.
- Integrity-result and load-case-export request kinds (FAC-013): the protocol must add them, for example
  `get_integrity_results` (run id) and `export_load_cases` (run id, format), answered by `ok`; milestone 3.
- Design and test case file request kinds (FAC-017): the protocol must add them, for example `read_file` (path) and
  `write_file` (path, content), where a write that fails its file's schema is refused by `error` with code
  `validation_failed`; milestone 1. The owner rules whether writes are allowed before team sharing is decided.
