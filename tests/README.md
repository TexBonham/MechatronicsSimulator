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
  free-form in Catch2, so nothing is registered; the traceability test will read them. `[smoke]` marks build smoke tests
  that verify no requirement.

## Stubs before code (ruling T3)

Every requirement gets a test stub committed as a strict expected failure before any code is written for it.

- Python: `@pytest.mark.xfail(reason="stub: <ID> not implemented")`. `xfail_strict = true` in `pyproject.toml` turns
  a stub that starts passing into a failure, so it must be converted (marker removed) in the commit that implements
  the requirement.
- C++: not yet ruled (ADR 0008 open item). Catch2's `[!shouldfail]` tag has the same strict semantics (the test
  fails if its body passes) and is the candidate convention.

Tolerances come from the spec, never from the test (ruling T4); a tolerance change is a spec edit.

`pytest` runs with `--strict-markers`, so a misspelled marker is an error rather than a silently ignored tag.
