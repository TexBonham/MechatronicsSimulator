# Spec: Facade service and protocol (FAC)

## Purpose

The facade (`mechsim/facade/`) is the one serializable surface every client uses to drive the simulator: a Python
service over the core binding, speaking a versioned protocol with a schema in `schemas/` and a client-independent
conformance suite. Streaming and decimation live here, not in clients; replay reads archives only. The protocol and
transport are proposed in ADR 0015; FAC-018 to FAC-020 carry its message rules.

"Architecture doc" below is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| TST | FAC-001, FAC-010, FAC-015 | TST-030 to TST-035 |
| REC | FAC-006, FAC-007, FAC-008, FAC-011 | REC-019, REC-020, REC-024 |
| SCO | FAC-012 | SCO-008, SCO-010, SCO-012 |
| INT | FAC-013 | INT-022, INT-032 |
| VIE | FAC-005, FAC-011, FAC-016, FAC-018, FAC-019 | VIE-012, VIE-013, VIE-014 |
| DES | FAC-017 | DES-008, DES-011, DES-015, DES-036 |
| KER, PHY, SIG, CTL | FAC-009, FAC-014 | KER-015, KER-016, PHY-009, CTL-019, KER-026 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| FAC-001 | Every client shall drive the simulator through the one facade and through no other entry point. | ADR 0006, Decision, Facade; Architecture doc, Repository starting point, invariant 9; Ruling R-15 | M1 | unit | n/a |
| FAC-002 | Every facade request and response shall serialize to and from the protocol schema without loss. | ADR 0006, Decision, Facade; ADR 0007, Decision, Facade | phase 1 | unit | n/a |
| FAC-003 | Every facade exchange shall carry the protocol version, and the service shall reject a request of an unsupported version with an error naming both versions. | ADR 0007, Decision, Facade; Ruling R-16 | phase 1 | unit | n/a |
| FAC-004 | The protocol schema shall live in `schemas/`, generated from its pydantic models and checked by the schema staleness test. | ADR 0007, Decision, Facade; ADR 0012 (L4) | phase 1 | unit | n/a |
| FAC-005 | The facade shall ship a client-independent conformance suite that every client, the browser client and any standalone app alike, must pass. | ADR 0007, Decision, Facade; ADR 0007, Consequences; Ruling R-16 | phase 1 | unit | n/a |
| FAC-006 | The service shall stream run data to clients from the recorder's streaming interface. | ADR 0006, Decision, Recorder; ADR 0007, Decision, Streaming and decimation | M1 | unit | n/a |
| FAC-007 | The service shall decide what reaches a client and at what rate, decimating in the service, and decimation shall never change the recording. | ADR 0007, Decision, Streaming and decimation; ADR 0007, Consequences; Ruling R-16 | M1 | unit | n/a |
| FAC-008 | Replay through the facade shall read run archives only, never live engine state. | ADR 0006, Decision, Replay; Ruling R-15 | M1 | unit | n/a |
| FAC-009 | The facade service shall be Python in `mechsim/facade/`, calling the core runtime through its pybind11 binding. | ADR 0006, Decision; Architecture doc, Repository starting point, layout (`core/bindings/`, `mechsim/facade/`); Ruling R-14 | M1 | unit | n/a |
| FAC-010 | The facade shall expose the validate, run, suite and replay operations the CLI commands use. | ADR 0006, Decision, First client; ADR 0012 (L9) | M1 | unit | n/a |
| FAC-011 | The facade shall expose an archive's recorded series and events, and the value of any body, joint, pin or signal at any recorded time. | ADR 0006, Decision, Replay; Architecture doc, Modules: outputs, Viewer and replay row | M1 | unit | n/a |
| FAC-012 | The facade shall expose pass/fail reports and regression diffs. | Architecture doc, Modules: outputs, Scoring row; ADR 0007, Consequences | M2 | unit | n/a |
| FAC-013 | The facade shall expose integrity results and the load-case export. | Architecture doc, Modules: outputs, Structural analysis and Exports rows; ADR 0007, Consequences | M3 | unit | n/a |
| FAC-014 | The facade shall return every run error (solver failure, drift, emulator fault) to the client as a structured error report. | Architecture doc, Repository starting point, invariant 7 | M1 | unit | n/a |
| FAC-015 | A run started through the facade by any client shall produce the same archive as the same run started from the CLI. | Architecture doc, Repository starting point, invariant 4; ADR 0007, Consequences | M1 | unit | n/a |
| FAC-016 | Client code shall hold no domain logic: every value a client shows comes from the facade. | ADR 0007, Decision, Client role; Ruling R-16 | M1 | invariant | n/a |
| FAC-017 | The facade shall expose reading and writing design and test case files, validating every write against the file's schema. | Architecture doc, Repository starting point, invariant 3; Seams reserved for the roadmap, GUI design and test editors row | M1 | unit | n/a |
| FAC-018 | A series subscription shall name one or more distinct series, a simulated-time window whose start is before its end, and a point count of at least one per series, and the service shall decimate to that count: every window sample when the count covers them, otherwise the count's equal buckets over the window (half-open, the last closed), each contributing its sample nearest the bucket centre, the earlier on a tie and none when empty. | ADR 0015 (proposed), Decision, Decimation request; ADR 0007, Decision, Streaming and decimation | phase 1 | unit | n/a |
| FAC-019 | Time series shall cross the protocol as a header message naming the run, the series, the dtype float64, little-endian byte order and the sample count, followed by one binary frame of (1 + series count) x sample count x 8 bytes, time column first; a header whose byte length disagrees shall be rejected. | ADR 0015 (proposed), Decision, Binary frames | phase 1 | unit | n/a |
| FAC-020 | Every error response shall carry an error code and a message, and the ID of the violated requirement where one applies. | ADR 0015 (proposed), Decision, Messages; Architecture doc, Repository starting point, invariant 7 | phase 1 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Facade protocol and transport | proposed in ADR 0015: JSON over WebSocket, binary float64 frames for series; service on `websockets` (asyncio, >= 13), token as `?token=` on the URL, 401 at the handshake otherwise (M2-F1) | owner (ADR 0015 acceptance) |
| Version compatibility rule (exact match or semver range) | proposed in ADR 0015: exact match on an integer version, currently 1 | owner (ADR 0015 acceptance) |
| Default decimation policy and client rate requests | client request proposed in ADR 0015 (FAC-018); algorithm (M2-F1): every window sample when `points` >= the window's samples, otherwise `points` equal buckets over [t_start, t_end] (half-open, the last closed), each contributing its sample nearest the bucket centre (the earlier on a tie, none when empty); live streams emit each bucket once the recording has passed it | owner (ADR 0015 acceptance) |
| Whether designs and results are shared across a team | not set | owner (Requirements doc, Assumptions and open questions) |

## Service rules (M2-F1)

| Rule | Value |
|---|---|
| Requests beyond the phase 1 set (additive kinds) | `events`, `timeline`, `geometry`, `value_at`, `probe` (open replay, FAC-011, VIE-002 to VIE-006, VIE-015); `report`, `diff` (by the `suite_id` a `run_suite` result names, FAC-012); `read_file`, `write_file` (FAC-017); `advance` (`run_id`, `dt`: the playing cursor by dt, held while paused) and `list_runs` (archives under the service's runs folder by run id, plus open replays marked `open`) (ruling 22, brief M2-X5; the browser client's switch to them is owed, no client sources in this tree) |
| Error codes (the protocol's `ErrorCode`, all emitted) | `unsupported_version` (FAC-003; a version that is not the integer itself, booleans and floats included), `invalid_message` (unparseable, binary or failing validation; request id null when unparseable), `unknown_kind`, `validation_failed` (refused file or request), `not_found`, `busy` (a run or suite already going), `run_error` (a run that ended in error or never started), `internal` |
| Run errors (FAC-014) and answers | one terminal answer per request id: a run that ended in error, or a case stopped before t = 0, answers `error` with code `run_error`, the run id (null when it never got one) and `report` (party, simulated time, cause, requirement `INV-007`); otherwise `ok` with the summary and `stream_errors` (subscriptions that could not be served). A run that started sends `run_ended` before the answer, its `reason` the runner's end reason: `end_time`, `message`, `controller_state`, `max_duration` or `run_error` |
| Live subscriptions | `subscribe_series` on a run id that is not an open replay is held for the next run of that id on the connection; a run's id is `<case id>-seed<seed>` |
| Static route (M2-F2, VIE-012) | a request without a WebSocket upgrade is a plain GET of the browser client under `web/`: `/` is `index.html`, then top-level `*.html`, `*.js`, `*.css` and `vendor/*` only, never a path outside `web/`; anything else 404. A WebSocket upgrade on any path needs the token (FAC-005); the browser client uses `/ws?token=` |
| Files (FAC-017) | read and write only design and test case files under the service's files root; a write is checked in place and the previous bytes restored when it does not validate |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Rendering, scrubbing and overlays | VIE |
| Archive writing | REC |
| CLI argument parsing | TST |
| Engine debug viewer | ENG (`core/viewer_debug/`, ADR 0007) |
