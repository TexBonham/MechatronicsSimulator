# mechsim/facade/: contract

Facade service and protocol (ADR 0006, ADR 0007, ADR 0015 proposed). Spec: `docs/spec/fac.md` (service rules
section). Every client (CLI, `web/`) imports only `mechsim.facade` (FAC-001, FAC-016); the facade alone reaches the
runner, recorder, scoring and viewer.

| File | Holds |
|---|---|
| `protocol.py` | Message models (below) |
| `service.py` | `Facade(services, runs_dir)`, `FacadeServices(runner, reader, designs_root, live, files_root)` (= the CLI's `CliServices`), `default_services()` (archive reader always; runner and live wiring when the core extension imports), `FacadeError(code, message, requirement_id, field)`, result models |
| `files.py` | `file_kind`, `kind_of_text`, `validate_file` (moved from the CLI), `WRITABLE_KINDS` = design, case |
| `decimate.py` | `decimate(times, t_start, t_end, points) -> indices`, `bucket_select`, `bucket_of`, `window` |
| `live.py` | `LiveWiring` protocol (`driver(on_row)`, `view(recording, build)`), `LiveDriver` (wraps the run's driver, mirrors rows into its own `Recorder`, subscription emits progress and batches every `CHUNK_ROWS` = 2048 rows and at the end), `Subscription`, `Batch`, `RunListener` |
| `core_wiring.py` | `CoreLiveWiring`: `CoreDriver(subscribers=...)` rows as `TickRecord`, `record.reader.RecordingView` + `Kinematics.of` for partial recordings; imported only when the core is built |
| `server.py` | `serve(facade, host, port, *, token, web_root=WEB_ROOT) -> FacadeServer` (`port`, `url()`, `page_url()`, `close()`), `classify(frame)`, `ok_message`, `error_message`, `tick_batch_frames`, `static_file(web_root, path)`, `static_response(web_root, path)` |

## Facade methods (all synchronous; results are pydantic models)

| Method | Returns / rule |
|---|---|
| `list_designs(root)` | `DesignList`: name, path, version, errors per `<root>/*/design.yaml` |
| `validate(path)` | `ValidationResult` per YAML file of a folder or the file; missing path `not_found` |
| `run_case(case, design, sets, out_dir, dry, seed, mode, listener, subscriptions)` | `RunResultSummary`: run id, case id, status, archive, end reason/time, `report_text`, `error: RunErrorReport(party, t, cause, requirement_id=INV-007)`, `dry`; never raises for a run error (FAC-014); seed/mode must equal the case's |
| `run_id_of(case, design)` | `<case id>-seed<seed>` |
| `run_suite(suite, design, sets, out_dir, report_path, baseline_path)` | `SuiteResultSummary` (suite_id, text, diff entries and text, written paths, errors, runs); writes `<out>/<name>_report.txt` |
| `report(suite_id)`, `diff(suite_id)` | text + Markdown; diff entries in report order (FAC-012) |
| `replay_open(archive)` | `ReplayHandle` keyed by run id; archive reader only (FAC-008) |
| `events`, `timeline`, `probe(run_id, kind, name)`, `geometry` | viewer results (VIE-002 to VIE-006, VIE-015) |
| `member_stress(run_id)`, `swing_angle(run_id, load, attachment)` | `MemberStressResult` (unit Pa, times, members with area, second moments y and z, fibres y and z, start, end), `SwingAngleResult` (unit rad, times, values; down from the archived design's gravity, an unreadable archive `not_found`); VIE-010, VIE-011 |
| `series(run_id, names, t_start, t_end, points)` | `SeriesWindow`, decimated; archive untouched (FAC-007) |
| `value_at(run_id, ref, t)` | latest sample at or before t; refs `pose:`, `joint:`, `signal:`, `actuator:` |
| `scrub`, `pause`, `step`, `advance` | `CursorResult` (index, t, paused, samples) |
| `read_file(path)`, `write_file(path, text)` | design and test case files under `files_root` only; a failed write restores the previous bytes |

## Server

| Request kind | Facade call |
|---|---|
| `list_designs`, `validate_design` | `list_designs()`, `validate(path)` |
| `run_case` | worker thread; `run_progress`, `tick_batch` (header + one binary frame) while running, `run_ended`, then the one answer; one run at a time (`busy`) |
| `run_suite`, `report`, `diff` | `run_suite(suite, design)`, `report`, `diff` |
| `replay_open`, `scrub`, `pause`, `step`, `advance` (`dt`, ruling 22) | the replay and its cursor |
| `list_runs` | `list_runs()`: `RunList` of `<runs_dir>/*.h5` by run id plus open replays (`open`), sorted (ruling 22) |
| `subscribe_series` | open replay: `ok` then one batch; otherwise held for the next run of that run id on the connection |
| `events`, `timeline`, `geometry`, `value_at`, `probe`, `member_stress`, `swing_angle`, `read_file`, `write_file` | same-named methods |

Static route (M2-F2, VIE-012): `process_request` answers a request without `Upgrade: websocket` from `web/` (`/` is
`index.html`; top-level `*.html`, `*.js`, `*.css`; `vendor/*`; else 404; `..`, `.`, empty segments, `:`, backslash,
NUL and links resolving out of the root refused); an upgrade on any path needs the token. `mechsim serve` (CLI)
prints `page_url()`; the browser client opens `/ws?token=` on its own origin.

Paths in requests are relative to `files_root` unless absolute. Error codes: `unsupported_version` (FAC-003; the
version must be the int itself, `true` and `1.0` included as unsupported), `invalid_message` (bad JSON, binary, failed
validation; request id null when unparseable), `unknown_kind`, `validation_failed`, `not_found`, `busy`, `run_error`,
`internal`. Shutdown closes connections with 1001.

One terminal answer per request id. `run_case` answers `error` (`run_error`, `run_id`, `report` = the structured
run error) when the run ended in error or never started, else `ok` with the summary plus `stream_errors`; a listener
failure mid-run only lands in `stream_errors`. `run_ended.reason` is the runner's end reason (`end_time`, `message`,
`controller_state`, `max_duration`, `run_error`).

Locks: `_run_lock` admits one run or suite per service (a second gets `busy`), and every other facade call runs on a
worker thread under `_facade_lock`, while the run itself holds only `_run_lock` so replay requests stay served during
it. `close()` closes the listening socket and every connection, then waits for in-flight request tasks, so a running
run's worker thread finishes before `close()` returns.

Decimation: every window sample when `points` >= samples in [t_start, t_end]; otherwise `points` equal buckets
(half-open, last closed), each giving its sample nearest the centre (earlier on a tie, none when empty).

## protocol.py

Pydantic message models, `extra="forbid"`, strict types, no domain logic.

| Name | Role |
|---|---|
| `PROTOCOL_VERSION` | Integer, 1; exact match required (FAC-003) |
| `Message` | Discriminated union on `kind` of every message class |
| `parse_message(text)` | JSON text or bytes to a `Message`; raises `ValidationError` |
| `dump_message(msg)` | `Message` to JSON text |
| `render_schema()` | Generated JSON Schema text |
| `main(argv)` | `--write-schema` writes `schemas/facade_protocol.json` |

Envelope: `version`, `message_id`, `kind`, `payload`.

| Group | Kinds |
|---|---|
| Requests | `list_designs`, `validate_design`, `run_case`, `run_suite`, `replay_open`, `subscribe_series`, `scrub`, `pause` (`paused` bool), `step` (`forward`/`backward`), `advance` (`run_id`, `dt`), `list_runs` (M2-X5, ruling 22); M2-F1: `events`, `timeline`, `geometry` (`run_id`), `value_at` (`run_id`, `ref`, `t`), `probe` (`run_id`, `probe`, `name`), `member_stress` (`run_id`), `swing_angle` (`run_id`, `load`, `attachment`) (brief R), `report`, `diff` (`suite_id`), `read_file` (`path`), `write_file` (`path`, `text`) |
| Responses | `ok` (`request_id`, `result`), `error` (`request_id` or null, `code`, `message`, `requirement_id`) |
| Events | `run_progress`, `run_ended` (`reason`), `tick_batch`, `timeline_event` (`category`, `label`) |

Rules: times are simulated seconds, finite, >= 0, with `run_id`. `subscribe_series`: `t_start < t_end`,
`points >= 1`, unique series (FAC-018). `tick_batch`: `byte_length == (1 + len(series)) * sample_count * 8`, time
column first, float64 little-endian; one sample means `t_start == t_end` (FAC-019).

## Regenerate the schema

`uv run python -m mechsim.facade.protocol --write-schema` after every model change; staleness test:
`tests/invariant/test_schema_facade.py`. Examples: `schemas/examples/facade_*.json`.

## Tests

- Unit: `tests/unit/test_facade_protocol.py`, `tests/unit/test_facade_service.py` (fakes; FAC-015 runs P1 on the core)
- Conformance suite: `tests/acceptance/test_facade_conformance_stubs.py` (protocol only, one P1 run plus the CLI's
  for FAC-015, busy and one-answer checks; FAC-013 still a stub)
- Invariant: `tests/invariant/test_fac_stubs.py` (clients import only the facade)
- Browser client and static route: `tests/unit/test_web_client.py` (ruling 21: no browser)

## Section index

| Section | File | Line |
|---|---|---|
| Frames and messages | server.py | 84 |
| Listener and connections | server.py | 190 |
| Facade server | server.py | 227 |
| Static files | server.py | 469 |
| Serve entry point | server.py | 518 |
| Result models | service.py | 97 |
| Helpers | service.py | 315 |
| Facade | service.py | 404 |
