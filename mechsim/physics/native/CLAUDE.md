# mechsim/physics/native/: contract

Translation half of the native adapter (PHY-020): neutral model to the engine model document, build time only.
Stepping is `core/physics/native/`. The only Python importer of `mechsim._core_ext.engine` (`_engine.py`, invariant 5).

| Name | Does |
|---|---|
| `translate(model, step=None, payloads=None, starts=None, substeps=1) -> Translation(document, report)` | `.text` is the JSON (sorted keys, deterministic); `report` one line per approximation (SCO-013), at h / substeps; `substeps` written when above 1 |
| `scheme_mode(x) -> (kept, ratio)`, `mode_text(x)` | Ruling 45: amplitude kept per period (1 + x^2)^(-pi / atan x), frequency ratio atan(x) / x; the text flags x above 0.2 unresolved; every member, rope swing and belt mode line uses it; drive and gear lines state the form with x from the engine's scheme decay |
| `prepare.prepare_model(model, starts, step, payloads, substeps=1) -> NativeArtifacts(model_json, version, report)` | `PreparedModel(model_json, "", version)` |
| `supported_elements()`, `SUPPORTED_ELEMENTS` | Mirrors `supported_kinds` in `core/physics/native/src/native_adapter.cpp` (test-checked) |
| `element_kinds(model)` | Kinds present, written as `elements` |
| `driven_mode_lines(model, decays)` | One line per driven joint from the run's largest scheme decay pi x (`NativeAdapter.driven_scheme_decay()`), `mode_text(x)`; `CoreDriver` appends them at the end of a native run (rulings 44, 45) |
| `carried_member_lines(model, values)`, `carried_validity_lines(model, values)` | Per carrying member from `CarriedRecord.values()` (10-tuples): the deflection and lag line; the work, peak force, quasi-static deflection and ratio line against `CARRIED_RATIO_LIMIT` (2.04, = `core.CARRIED_RATIO_LIMIT`), plus the not-valid line from the flagged time when unresolved (ENG-052, PHY-035, ruling 69) |
| Creep line | Per transmission with efficiency below one: omega_s = `SLIP_VELOCITY`, tau forward and backward as formulas in F, K, D_d, values at the static load (prismatic driven joint: subtree mass times gravity along the world axis) when nonzero; self-locking screw: sag band [0, n F / K] (rulings 47, 48) |
| `member_node_names(model)` | Names of the member node poses after the neutral bodies: `<member>.node<k>`, chain root to tip (pinned root: pin segment first) |
| `output_per_input(transmission)` | n: r, lead / 2 pi, 1 / ratio (signed) |
| `UnmappableElementError(element, reason)`, `StartStateError` | Refusals (PHY-019): also friction, stiffness or limits on a spherical or free joint, limits on a planar joint (ENG-013) |
| `adapter.py` | Engine type re-exports for tests |

## Document (`mechsim-native-1`)

| Key | Content |
|---|---|
| `bodies` | Neutral order (index = neutral body = joint = payload index), then `world.anchor` (fixed) when a constraint names the world; joint `type`, placement, axis, per-DOF `stiffness`/`rest` (coordinate joints), `damping`, `friction`, `lower`/`upper` (null = none) in engine order (free: angular then linear); inertia 9 numbers row major |
| `neutral_bodies` | Count of neutral bodies |
| `constraints` | Point/hinge, body indices, anchors, axes |
| `members` | `ends`: cantilever (clamped, free), simply_supported (pinned, roller), clamped (clamped, clamped); others refused; `carried` (neutral body indices of `LumpedMember.carries`) only when present; report: per member the ruling 65 out-of-plane line, per carrying member the ENG-052 approximation line |
| `transmissions` | Engine fields; efficiency both directions; damping 0 |
| `ropes` | Engine fields; world ends -1; attachment damping and friction 0 (neutral rope has none) |
| `contact` | null without `contacts`; geometries (world -1), material pairs, exclusions + parent-child pairs on non-free joints (as the bridge), slip velocity 1e-4 |
| `payloads`, `starts` | Body indices; a shaft starts at its driven start / n |

## Section index, `translate.py`

| Section | Line |
|---|---|
| Joints | 141 |
| Shapes and contacts | 242 |
| Members, ropes and transmissions | 324 |
| Model | 598 |
