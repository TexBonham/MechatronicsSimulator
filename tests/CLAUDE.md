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
| Matrix | `tests/unit/test_traceability_tool.py` | 563 |
| CLI | `tests/unit/test_traceability_tool.py` | 634 |
