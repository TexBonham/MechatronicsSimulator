# mechsim/viewer/: contract

Viewer module (VIE). Everything a client shows is computed here and served through `mechsim.facade` (VIE-014).
Headless: no Qt or display import in this milestone (invariant 9; `check_imports` allows them only here). Imported
only by `mechsim/facade/` and tests. Writes no files (VIE-017); needs no engine (VIE-016). Spec: `docs/spec/vie.md`.

| File | Holds |
|---|---|
| `replay.py` | `ReplayCursor(times)`: `scrub(t)` latest sample at or before t, `pause(bool)`, `step("forward" or "backward")` one sample held at the ends, `advance(dt)` only while playing, `state() -> CursorState(index, t, paused, samples)`; starts paused at sample 0; `ReplayError` on no samples or a negative or non-finite time |
| `timeline.py` | `timeline(events, criteria, end_time) -> TimelineEntry(t, category, label, source)` sorted by time; `category_of(kind)`; categories `command`, `fault`, `safety_event`, `criterion_failure` (failed criteria at their worst time) |
| `probes.py` | `probe(view, kind, name) -> Probe(kind, name, times, columns)`; body `pose:` six components + raw velocities, joint `joint:` position, velocity + generalized force, pin (pins group) and signal `signal:`; raw columns only when the view exposes `recording` and `kinematics` (the core record view); `ProbeError` |
| `geometry.py` | `body_geometry(design_file_bytes, base) -> BodyGeometry(body, kind, shapes, file)`; shapes with position, orientation and sizes in m (None while open) |

`__init__.py` exports exactly these names.

## Tests

`tests/unit/test_viewer.py` (VIE-002 to VIE-009, VIE-014 to VIE-017; VIE-016 runs a subprocess with `mechsim._core`
and `mujoco` blocked). VIE-001 and VIE-012 stay stubs for the browser client (`tests/unit/test_vie_stubs.py`).
