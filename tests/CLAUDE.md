# tests/: contract

Python test tiers (T1); C++ tests under `core/<module>/tests/`. Detail: `tests/README.md`.

## Tiers

| Directory | Tier |
|---|---|
| `unit/` | Per-module |
| `reference/` | Analytical reference |
| `acceptance/` | G1-G16 plus pendulum |
| `invariant/` | Import boundaries, schemas, determinism |

## Markers

- `@pytest.mark.req("ID")`: cites a requirement (registered in `pyproject.toml`)
- `@pytest.mark.xfail(strict=True, reason="stub")`: marks a stub

## Stub conventions

- Python: `req` plus strict xfail; body `raise NotImplementedError`.
- C++: `[<ID>][!shouldfail][stub]` tags with `FAIL("stub");`.

## Section index, `unit/test_traceability_tool.py`

| Section | File | Line |
|---|---|---|
| Helpers | `tests/unit/test_traceability_tool.py` | 25 |
| Spec table parsing | `tests/unit/test_traceability_tool.py` | 62 |
| Python citations | `tests/unit/test_traceability_tool.py` | 168 |
| C++ citations | `tests/unit/test_traceability_tool.py` | 324 |
| Repository collection | `tests/unit/test_traceability_tool.py` | 431 |
| Matrix | `tests/unit/test_traceability_tool.py` | 571 |
| CLI | `tests/unit/test_traceability_tool.py` | 642 |

## Section index, `unit/test_component_schema.py`

| Section | File | Line |
|---|---|---|
| Worked examples | `tests/unit/test_component_schema.py` | 97 |
| Facets | `tests/unit/test_component_schema.py` | 153 |
| Tiers | `tests/unit/test_component_schema.py` | 355 |
| Parameters | `tests/unit/test_component_schema.py` | 410 |
| Part resolution | `tests/unit/test_component_schema.py` | 485 |
| Parts | `tests/unit/test_component_schema.py` | 662 |
| Category rules | `tests/unit/test_component_schema.py` | 735 |
| Plugin reference | `tests/unit/test_component_schema.py` | 837 |
| Structure | `tests/unit/test_component_schema.py` | 860 |

## Section index, `unit/test_neutral_model.py`

| Section | File | Line |
|---|---|---|
| Worked example | `tests/unit/test_neutral_model.py` | 222 |
| Run configuration defaults | `tests/unit/test_neutral_model.py` | 284 |
| Required fields | `tests/unit/test_neutral_model.py` | 314 |
| Bounds | `tests/unit/test_neutral_model.py` | 467 |
| Types and units | `tests/unit/test_neutral_model.py` | 602 |
| Unknown keys and empty collections | `tests/unit/test_neutral_model.py` | 698 |
| Names and references | `tests/unit/test_neutral_model.py` | 778 |
| Collision shapes | `tests/unit/test_neutral_model.py` | 857 |
| Joint types | `tests/unit/test_neutral_model.py` | 877 |
| Determinism and SI content | `tests/unit/test_neutral_model.py` | 932 |
| Joint tiers, contacts, names | `tests/unit/test_neutral_model.py` | 985 |
