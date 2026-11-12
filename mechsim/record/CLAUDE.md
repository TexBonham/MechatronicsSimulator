# mechsim/record: contract

Recorder, HDF5 run archive, archive reader, exports, run-twice comparison (REC). h5py and scipy only here; no engine
types. `reader.py` imports `mechsim.testing.driver` and `.runner` types (no cycle: testing imports record only from
`core_driver.py`).

| File | Holds |
|---|---|
| `tick.py` | `TickRecord` and physics structs, frozen mirror of `core/kernel/.../tick_record.hpp` (incl. `coupling`: `CouplingSample(name, kind, value, digest)`, kinds none/real/integer/flag/payload; `PhysicsOutputs.joint_positions`, `joint_drive_torques` (`JointDriveTorque(joint, torque)`), `payloads` (`PayloadState(body, carrier, attached, force)`, carrier `WORLD_BODY` = -1 once released); `RopeTension(tension = load end, slack, max_tension)`); `RecordError` |
| `layout.py` | `FIXED_SERIES` (path, unit, axis, dtype) canonical order; `MEMBER_SERIES`, `ROPE_SERIES` (`ROPE_TENSION`, slack, `ROPE_MAX_TENSION`); `COLUMN_SERIES`, `Columns(names, units)`, `label_series`; `Series` (+ `names`, `units`), `Recording`; `seconds()` |
| `stream.py` | `Recorder(base_rate, columns=None)`: `write_row`, `subscribe(cb) -> int`, `unsubscribe`, `recording()`, `flush(path, identity, end)` |
| `archive.py` | `RunIdentity` (controller_mode native, emulated, none), `RunEnd`, `RunFault`, `write_archive(path, identity, end, recording, extra, columns)` (`extra` -> root attrs `meta_<key>`, read back as `RunArchive.extra`), `RunArchive(path)` reader, `coupling()`, `records()` replay, `repository_commit`, `tool_version` |
| `export.py` | `export_csv`, `export_json`, `export_mat(archive, path, names)`; index-column series exported with per-column names and units (`series[name] [unit]`, JSON `names`/`units`, MAT `series_columns`/`series_column_units`) |
| `reader.py` | Engine-free read side (ruling 17; nothing from `mechsim._core`): `Kinematics`/`JointKinematics` (`of(model)`, `to_json`, `from_json`), `RecordingView` (the `RecordView` scoring and replay read: `pose:`, `joint:`, `actuator:`, `signal:` refs, events by the label table), `RecordArchiveReader.open(path) -> (RunMetadata, RecordingView)` (metadata `members`, `material_overrides`, `variation`, `integrity` from the extras), `DriverError`; re-exported by `testing/core_driver.py` |
| `compare.py` | `archives_identical(a, b) -> Report(identical, content_identical, first_difference)` |

## Archive layout (format version 2; reader accepts 1 and 2)

- Root attrs: format, format_version, design_name/version/source_hash, test_case_id/hash, seed (uint64),
  controller_mode, base/physics/control_rate (int64 [num, den]), repository_commit, tool_version, end_outcome,
  end_time, end_ticks, fault_party/time/cause on run_error, `series` (canonical names).
- `time/tick`, `time/seconds`, `physics/...`, `signals/{pins,rails,actuator_efforts}/<name>` (group attr `names`),
  `controller/state` (index into `controller/state_names`), `events` (row, tick, seconds, source, kind, detail),
  `inputs/design_file`, `inputs/test_case_file` (bytes).
- `coupling` group attrs `names`, `kinds` (name order); series `coupling/value/<name>` (float64, NaN for none and
  payload) and `coupling/digest/<name>` (S64, SHA-256 of a payload's canonical content, empty otherwise) (REC-028).
  Transmissions also record `shaft_load_torque`, `force`, `stretch`, `power_flow` (0 static, 1 forward, 2 backward).
- Per joint coordinate (one column per `joint_positions` entry, neutral order): `physics/joint_positions` (m or rad),
  `physics/joint_drive_torques/torque` (N or N m, 0 where undriven) and `physics/joint_drive_torques/driven` (bool);
  a drive on a joint outside the positions, or two on one joint, is a `RecordError`; replay returns drives in joint
  order (REC-027). Transmission `slip` (C's field) is `physics/transmissions/slip` (m or rad, one column per
  transmission). Per payload: `physics/payloads/{body, carrier (int64, -1 world), attached, force (N, 3)}`.
- Structural (REC-004, REC-026): `physics/members/{start,end}_{moment,force}` ([rows, members, 3]) and
  `physics/ropes/{tension (load end), slack, max_tension}` ([rows, ropes]); columns named by `model_columns` (lumped
  member's `structural_member` or name; rope name). A format 2 archive without `max_tension` replays `tension`.
- Integrity events (REC-010): source and kind `integrity`, detail `IntegrityCrossing.detail()` (canonical JSON
  `limit, member, quantity, tier, value`). Extras written by `RecordArchiver`: `members` (`members_json`),
  `variation` (JSON or null), `material_overrides` + `material_overrides_sha256` (when the design names a file),
  `integrity` (`IntegrityResults.to_json`, when attached).
- Index-column series (`layout.COLUMN_SERIES`: members, ropes, transmissions, joint positions, drive torques, payloads) carry
  dataset attrs `names` and `units` (per column, REC-018) from `Columns(names, units)` the driver builds from the
  neutral model: `Recorder(base_rate, columns)` (counts checked at the first row) or
  `write_archive(..., columns=)` (overrides). Without them `names` and `units` are empty and `unit` is the
  conditional string (`m or rad`, `N or N m`). `Series.names`, `Series.units` read them back.
- Version 1 archives lack the joint, drive and payload series and the labels: `records()` replays those fields empty;
  `RunArchive.format_version`; any other version is a `RecordError`.
- Every series: attrs `unit`, `axis` (`tick` or `contact`); `counts` on the flattened per-DOF joint and constraint
  forces. Contacts are ragged: one entry per contact with `row`, `tick`, `seconds`.

## Conventions

- First row fixes entity counts and named series; later rows must match. Rows strictly advance in time.
- Determinism: fixed write order, chunks of `CHUNK_ROWS` = 1024 rows, no compression, `track_times=False`,
  fixed-length strings (NUL rejected), no wall clock.
- Exports prepend the axis time series; CSV headers `name[i,j] [unit]`; MAT v5 cells `series_names`,
  `series_units`, `series_values`; JSON rejects NaN.
- Synthetic rows for tests: `tests/unit/test_record_support.py`.

## Section index

| Section | File | Line |
|---|---|---|
| Run records | archive.py | 85 |
| Repository and tool identity | archive.py | 157 |
| HDF5 helpers | archive.py | 179 |
| Writer | archive.py | 233 |
| Value conversions | archive.py | 294 |
| Reader | archive.py | 322 |
