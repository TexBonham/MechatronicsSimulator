# mechsim/physics/native/: contract

Translation half of the native adapter (PHY-020): neutral model to the engine model document, build time only.
Stepping is `core/physics/native/`. The only Python importer of `mechsim._core_ext.engine` (`_engine.py`, invariant 5).

| Name | Does |
|---|---|
| `translate(model, step=None, payloads=None, starts=None) -> Translation(document, report)` | `.text` is the JSON (sorted keys, deterministic); `report` one line per approximation (SCO-013) |
| `prepare.prepare_model(model, starts, step, payloads) -> NativeArtifacts(model_json, version, report)` | `PreparedModel(model_json, "", version)` |
| `supported_elements()`, `SUPPORTED_ELEMENTS` | Mirrors `supported_kinds` in `core/physics/native/src/native_adapter.cpp` (test-checked) |
| `element_kinds(model)` | Kinds present, written as `elements` |
| `member_node_names(model)` | Names of the member node poses after the neutral bodies: `<member>.node<k>`, chain root to tip (pinned root: pin segment first) |
| `output_per_input(transmission)` | n: r, lead / 2 pi, 1 / ratio (signed) |
| `UnmappableElementError(element, reason)`, `StartStateError` | Refusals (PHY-019): also friction, stiffness or limits on a spherical or free joint |
| `adapter.py` | Engine type re-exports for tests |

## Document (`mechsim-native-1`)

| Key | Content |
|---|---|
| `bodies` | Neutral order (index = neutral body = joint = payload index), then `world.anchor` (fixed) when a constraint names the world; joint `type`, placement, axis, per-DOF `stiffness`/`rest` (coordinate joints), `damping`, `friction`, `lower`/`upper` (null = none) in engine order (free: angular then linear); inertia 9 numbers row major |
| `neutral_bodies` | Count of neutral bodies |
| `constraints` | Point/hinge, body indices, anchors, axes |
| `members` | `ends`: cantilever (clamped, free), simply_supported (pinned, roller), clamped (clamped, clamped); others refused |
| `transmissions` | Engine fields; efficiency both directions; damping 0 |
| `ropes` | Engine fields; world ends -1; attachment damping and friction 0 (neutral rope has none) |
| `contact` | null without `contacts`; geometries (world -1), material pairs, exclusions + parent-child pairs on non-free joints (as the bridge), slip velocity 1e-4 |
| `payloads`, `starts` | Body indices; a shaft starts at its driven start / n |

## Section index, `translate.py`

| Section | Line |
|---|---|
| Joints | 122 |
| Shapes and contacts | 208 |
| Members, ropes and transmissions | 290 |
| Model | 450 |
