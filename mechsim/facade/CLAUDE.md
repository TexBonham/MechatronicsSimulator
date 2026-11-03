# mechsim/facade/: contract

Facade service and protocol (ADR 0006, ADR 0007, ADR 0015 proposed). Spec: `docs/spec/fac.md`.

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
| Requests | `list_designs`, `validate_design`, `run_case`, `run_suite`, `replay_open`, `subscribe_series`, `scrub`, `pause` (`paused` bool), `step` (`forward`/`backward`) |
| Responses | `ok` (`request_id`, `result`), `error` (`request_id` or null, `code`, `message`, `requirement_id`) |
| Events | `run_progress`, `run_ended` (`reason`), `tick_batch`, `timeline_event` (`category`, `label`) |

Rules: times are simulated seconds, finite, >= 0, with `run_id`. `subscribe_series`: `t_start < t_end`,
`points >= 1`, unique series (FAC-018). `tick_batch`: `byte_length == (1 + len(series)) * sample_count * 8`, time
column first, float64 little-endian; one sample means `t_start == t_end` (FAC-019).

## Regenerate the schema

`uv run python -m mechsim.facade.protocol --write-schema` after every model change; staleness test:
`tests/invariant/test_schema_facade.py`. Examples: `schemas/examples/facade_*.json`.

## Tests

- Unit: `tests/unit/test_facade_protocol.py`
- Conformance stubs (strict xfail): `tests/acceptance/test_facade_conformance_stubs.py`
