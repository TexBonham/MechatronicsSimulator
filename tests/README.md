# Tests

The simulator's own tests, organized in the four tiers of ADR 0008 (ruling T1). Every tier runs headless in CI.

| Tier | Folder | Content |
|---|---|---|
| Unit | `tests/unit/` | Per-module unit tests |
| Analytical reference | `tests/reference/` | Results checked against closed-form references within spec tolerances |
| Acceptance | `tests/acceptance/` | G1 to G16 plus the pendulum |
| Invariant | `tests/invariant/` | Import boundaries, schema validity of every shipped YAML file, run-twice determinism |

C++ tests for the core runtime live next to their code under `core/<module>/tests/` and run through ctest.

## Running

```sh
uv run pytest                               # every Python tier
uv run pytest tests/unit -q                 # one tier
uv run pytest -m req                        # only tests that cite a requirement
ctest --preset debug-sanitize               # C++ tests, after configuring and building that preset
```

## Citing requirements

Each spec in `docs/spec/` numbers its requirements (`ENG-001`, `INV-005`, ...). A test names the requirement it
verifies; the traceability test (ADR 0008) fails on a requirement with no test and on a test citing an unknown ID.

- Python: the `req` marker, registered in `pyproject.toml`. Apply it once per requirement; a test may carry several.

  ```python
  @pytest.mark.req("INV-005")
  @pytest.mark.req("INV-009")
  def test_repository_has_no_import_boundary_violations() -> None: ...
  ```

- C++ (Catch2 v3): a tag holding the requirement ID, e.g. `TEST_CASE("pendulum period", "[ENG-001]")`. Tags are
  free-form in Catch2, so nothing is registered; the traceability test reads them. `[smoke]` marks build smoke tests
  that verify no requirement.

## Stubs before code (ruling T3)

Every requirement gets a test stub committed as a strict expected failure before any code is written for it.

## Stub conventions

- Python: a stub carries the requirement marker and a strict xfail marker, and its body raises `NotImplementedError`.

  ```python
  @pytest.mark.req("ENG-001")
  @pytest.mark.xfail(strict=True, reason="stub")
  def test_pendulum_period_matches_closed_form() -> None:
      raise NotImplementedError
  ```

  Converting a stub means removing the xfail marker and writing the real test; the `req` marker stays.

- C++ (Catch2 v3): a stub is `TEST_CASE("<ID> <short title>", "[<ID>][!shouldfail][stub]")` with the body
  `FAIL("stub");`.

  ```cpp
  TEST_CASE("ENG-001 pendulum period", "[ENG-001][!shouldfail][stub]") {
      FAIL("stub");
  }
  ```

  Converting a stub removes the `[!shouldfail]` and `[stub]` tags and writes the real test; the ID tag stays.

Both conventions share one property: a stub that starts passing fails the run (pytest strict xfail; Catch2
`[!shouldfail]` fails a test whose body passes). A stub must therefore be converted in the commit that implements its
requirement.

Tolerances come from the spec, never from the test (ruling T4); a tolerance change is a spec edit.

`pytest` runs with `--strict-markers`, so a misspelled marker is an error rather than a silently ignored tag.
