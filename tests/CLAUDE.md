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

## Section index, round 4 large test files

| Section | File | Line |
|---|---|---|
| Pendulum and the neutral model | unit/test_builder.py | 69 |
| Mass properties | unit/test_builder.py | 174 |
| Shapes and contacts | unit/test_builder.py | 197 |
| Lumped and structural members | unit/test_builder.py | 269 |
| Transmissions | unit/test_builder.py | 361 |
| STEP bodies | unit/test_builder.py | 411 |
| Shipped gantry, rigid members, module order | unit/test_builder.py | 478 |
| Loud failures | unit/test_builder.py | 584 |
| Shipped examples | unit/test_design_schema.py | 99 |
| Header and version | unit/test_design_schema.py | 190 |
| Open parameters | unit/test_design_schema.py | 301 |
| Bodies | unit/test_design_schema.py | 397 |
| Joints | unit/test_design_schema.py | 771 |
| Components | unit/test_design_schema.py | 947 |
| Signals and pin bindings | unit/test_design_schema.py | 1054 |
| Power | unit/test_design_schema.py | 1179 |
| Environment, firmware, structural | unit/test_design_schema.py | 1216 |
| Wiring and rail consumers | unit/test_design_schema.py | 1317 |
| Contact environment | unit/test_design_schema.py | 1518 |
| Chip pin map | unit/test_design_schema.py | 1763 |
| Operator messages | unit/test_design_schema.py | 1843 |
| Errors | unit/test_design_schema.py | 1923 |
| CAD mass cross-check | unit/test_design_schema.py | 1980 |
| Milestone 2 senses, acts_on, event codes (DES-043, DES-045) | unit/test_design_schema.py | 2062 |
| Model level | unit/test_mjcf_translation.py | 392 |
| Joints | unit/test_mjcf_translation.py | 521 |
| Shapes and contacts | unit/test_mjcf_translation.py | 652 |
| Constraints, members, transmissions | unit/test_mjcf_translation.py | 717 |
| Payloads | unit/test_mjcf_translation.py | 1081 |
| Loading, design version, resolution | unit/test_runner.py | 90 |
| Initial state and controller precondition | unit/test_runner.py | 154 |
| Command timeline | unit/test_runner.py | 197 |
| Fault timeline | unit/test_runner.py | 252 |
| End conditions and run errors | unit/test_runner.py | 368 |
| Configuration, seeds, mode, dry | unit/test_runner.py | 444 |
| Suites and sweeps | unit/test_runner.py | 520 |
| Suite files | unit/test_runner.py | 624 |
| Fault routing through the driver | unit/test_runner.py | 733 |
| Overrides and part substitution | unit/test_runner.py | 777 |
| Statistics | unit/test_scoring.py | 73 |
| Criteria | unit/test_scoring.py | 173 |
| Cases and errored runs | unit/test_scoring.py | 595 |
| Suites, report, baselines | unit/test_scoring.py | 631 |
| Shipped examples | unit/test_test_case_schema.py | 120 |
| Header | unit/test_test_case_schema.py | 182 |
| Open parameters | unit/test_test_case_schema.py | 332 |
| Initial state | unit/test_test_case_schema.py | 393 |
| Commands | unit/test_test_case_schema.py | 528 |
| Faults | unit/test_test_case_schema.py | 599 |
| End condition | unit/test_test_case_schema.py | 763 |
| Criteria | unit/test_test_case_schema.py | 813 |
| Sweeps, variations, baseline | unit/test_test_case_schema.py | 1139 |
| Suite file | unit/test_test_case_schema.py | 1261 |
| Files | unit/test_test_case_schema.py | 1332 |
| Fixtures and helpers | unit/test_record.py | 57 |
| Body, joint and contact channels | unit/test_record.py | 83 |
| Named signal checks | unit/test_record.py | 145 |
| Actuators, joints and payloads | unit/test_record.py | 174 |
| Index columns and writer labels | unit/test_record.py | 264 |
| Archive format version | unit/test_record.py | 347 |
| Controller state and fault log | unit/test_record.py | 384 |
| Stepper and estop catalog entries | unit/test_library_catalog.py | 26 |
| Milestone 2 catalog types (LIB-019, LIB-025, LIB-026, LIB-027) | unit/test_library_catalog.py | 47 |
| Parametrized case builders | unit/test_library_catalog.py | 183 |
| Shipped files and placeholders | unit/test_library_catalog.py | 267 |
| Sensor types | unit/test_library_catalog.py | 317 |
| Clients reach the simulator only through the facade | unit/test_facade_service.py | 64 |
| validate, run, suite, replay (FAC-010) | unit/test_facade_service.py | 103 |
| Run errors (FAC-014) | unit/test_facade_service.py | 198 |
| Replay: series, events, values (FAC-008, FAC-011) | unit/test_facade_service.py | 226 |
| Decimation rule (FAC-007, FAC-018) | unit/test_facade_service.py | 307 |
| Live streaming (FAC-006, FAC-007) | unit/test_facade_service.py | 343 |
| Facade run equals runner run (FAC-015, FAC-007) | unit/test_facade_service.py | 462 |
| Files (FAC-017) | unit/test_facade_service.py | 487 |
| Fixtures and helpers | acceptance/test_acc_stubs.py | 21 |
| ACC-001 to ACC-008 | acceptance/test_acc_stubs.py | 67 |
| Refusal helpers | acceptance/test_acc_stubs.py | 192 |
| ACC-009 to ACC-024 | acceptance/test_acc_stubs.py | 214 |
| ACC-025 to ACC-028 | acceptance/test_acc_stubs.py | 460 |
| ACC-029 to ACC-032 | acceptance/test_acc_stubs.py | 532 |
| ACC-033 to ACC-034 | acceptance/test_acc_stubs.py | 567 |
| ACC-035 to ACC-045 | acceptance/test_acc_stubs.py | 620 |
| PHY-016 | acceptance/test_acc_stubs.py | 718 |

Filled gantry: `test_builder_support.py`.
