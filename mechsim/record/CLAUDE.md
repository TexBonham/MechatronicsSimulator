# mechsim/record: contract

Recorder, HDF5 run archive, exports, run-twice comparison (REC). h5py and scipy only here; no engine types.

| File | Holds |
|---|---|
| `tick.py` | `TickRecord` and physics structs, frozen mirror of `core/kernel/.../tick_record.hpp`; `RecordError` |
| `layout.py` | `FIXED_SERIES` (path, unit, axis, dtype) canonical order; `Series`, `Recording`; `seconds()` |
| `stream.py` | `Recorder(base_rate)`: `write_row`, `subscribe(cb) -> int`, `unsubscribe`, `recording()`, `flush(path, identity, end)` |
| `archive.py` | `RunIdentity`, `RunEnd`, `RunFault`, `write_archive`, `RunArchive(path)` reader, `records()` replay, `repository_commit`, `tool_version` |
| `export.py` | `export_csv`, `export_json`, `export_mat(archive, path, names)` |
| `compare.py` | `archives_identical(a, b) -> Report(identical, content_identical, first_difference)` |

## Archive layout (format version 1)

- Root attrs: format, format_version, design_name/version/source_hash, test_case_id/hash, seed (uint64),
  controller_mode, base/physics/control_rate (int64 [num, den]), repository_commit, tool_version, end_outcome,
  end_time, end_ticks, fault_party/time/cause on run_error, `series` (canonical names).
- `time/tick`, `time/seconds`, `physics/...`, `signals/{pins,rails,actuator_efforts}/<name>` (group attr `names`),
  `controller/state` (index into `controller/state_names`), `events` (row, tick, seconds, source, kind, detail),
  `inputs/design_file`, `inputs/test_case_file` (bytes).
- Every series: attrs `unit`, `axis` (`tick` or `contact`); `counts` on the flattened per-DOF joint and constraint
  forces. Contacts are ragged: one entry per contact with `row`, `tick`, `seconds`.

## Conventions

- First row fixes entity counts and named series; later rows must match. Rows strictly advance in time.
- Determinism: fixed write order, chunks of `CHUNK_ROWS` = 1024 rows, no compression, `track_times=False`,
  fixed-length strings (NUL rejected), no wall clock.
- Exports prepend the axis time series; CSV headers `name[i,j] [unit]`; MAT v5 cells `series_names`,
  `series_units`, `series_values`; JSON rejects NaN.
- Synthetic rows for tests: `tests/unit/test_record_support.py`.
