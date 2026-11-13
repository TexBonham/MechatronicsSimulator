# core/physics/native/: contract

`mechsim::physics::native::NativeAdapter`, target `mechsim_physics_native` (-> `mechsim_kernel`, private
`mechsim_engine`), tests `mechsim_physics_native_tests` (ctest names `native.*`, reference `reference.native.*`).
Implements `kernel::PhysicsAdapter` over `engine::MultibodyProvider` + `SemiImplicitEuler` (PHY-020, ADR 0013, ADR
0014). The public header names no engine type (pimpl); registered as `"native"` in `core/bindings/src/factories.cpp`.

| Call | Does |
|---|---|
| `load(PreparedModel, rate)` | Parses `neutral_model_json` = the engine model document (`src/json.hpp` reader, format `mechsim-native-1`, written by `mechsim/physics/native/translate.py`); `engine_artifact` must be empty; refuses element kinds outside `supported_elements()` (21: the bridge's 20 + `rope`); every body index range-checked (`body_index`, ParseError naming the field); optional `substeps` (engine steps per adapter step, h = 1 / (rate substeps), default 1; written by `translate(..., substeps)`); joint `friction` (empty when all zero), `lower`/`upper` (null = open) into the engine joint (ENG-013, ENG-014); a start coordinate past a bound by more than the drift threshold is refused (ruling 43); builds bodies, constraints, `add_lumped_beam`, transmissions, `add_rope` (end points at the start configuration), contact model, starts, payload welds (a payload follows its carrier's start); h = 1 / rate; any failure `AdapterError(0, ...)` |
| `step()` | `substeps` engine steps (the tick's inputs re-applied before each after the first, then cleared with the engine drives, so a tick without `set_inputs` applies none; substep s of S drives toward the rest interpolated from the previous tick's end rest to this tick's, the last at this tick's exactly, a joint undriven last tick at this tick's throughout (ruling 49); health: max drift and iterations); `EngineError` -> `AdapterError(steps, "engine: ...")` (PHY-009); gathers every engine step's events into `outputs().model_changes` and `events()`, each once (ruling 40) |
| `set_inputs()` | Body wrenches on neutral bodies or member nodes (index in the published body list); generalized forces in neutral DOF order (free joints linear first; engine angular first), shaft DOFs through `set_shaft_torque`; body wrenches; joint drives (revolute/prismatic, one per joint, finite, >= 0): the previous tick's engine drives cleared (`clear_joint_drive`), each enabled one set with `set_joint_drive(joint, 0, rest, K, D)` (additive on the model spring, rest = end-of-step rest, ruling 42); engine refusals -> `AdapterError` |
| `outputs()` | Neutral bodies, then member node bodies (member node poses follow the neutral bodies, PHY-003) (`node_bodies`: each member's chain root to tip; X1 names `<member>.node<k>`); shaft joint force = `shaft_load_torque` (PHY-033); constraints neutral only; contact bodies outside the neutral list -> `kWorldBody`; members in the member root frame (ruling 24); ropes `tension` = load end, `max_tension` = largest segment, `slack` = load end zero; `model_changes` = pending spring changes, then the engine's (spring events skipped: listed by the entry with the spring index); transmissions `slip` = slip offset / n; joint drive torque K (rest - q') - D v' |
| `set_spring_offset/_stiffness(i)` | Spring order: each revolute, prismatic, planar DOF, then member nodes root to tip; joint springs through the extras, member nodes through the engine; each listed once in the next tick's `model_changes` |
| `disable_constraint(c)` | Neutral constraints only |
| `release_load(i)` | Payload body index: weld released; body count + r: rope r; second release ends the run |
| `set_payload_mass/attachment` | Payload bodies only; `kWorldBody` releases |
| `set_transmission_slip(t, s)` | s in [0, 1] -> `set_slip` |
| `integrator()`, `timestep()`, `events()`, `rope_segment_tensions()`, `rope_load_end_tensions()` | Extras for tests and later briefs (PHY-004, I2) |
| `driven_scheme_decay()` | `DrivenSchemeDecay{joint, decay}` per joint that carried an enabled drive since load, joint order: the largest engine `JointOutput::scheme_decay` (pi omega h) of any engine step (rulings 44, 45; bound as `NativeAdapter.driven_scheme_decay()`, `scheme_decay_record()` the same values as a shared `DrivenSchemeDecayRecord` that outlives the adapter, reset by `load`, read by `CoreDriver` after the run) |

## Extras (`ExtendedProvider`, implicit through K and D)

| Term | Form |
|---|---|
| Joint spring change | -(K1 - K0)(q - rest0) + K1 offset, stiffness K1 - K0 |

Joint limits, joint friction and joint drives are the engine's (ENG-013, ENG-014, ENG-051; ruling 37): no adapter term.

## JSON reader (`src/json.hpp`)

JSON only: duplicate keys, a leading plus, leading zeros, a bare dot, hex, NaN and Infinity, bad escapes and control
characters in strings are ParseErrors; nesting capped at 64 levels; empty or truncated input refused.

## Section index

| Section | File | Line |
|---|---|---|
| Supported elements | src/native_adapter.cpp | 38 |
| Helpers | src/native_adapter.cpp | 69 |
| Joint extras | src/native_adapter.cpp | 238 |
| Engine | src/native_adapter.cpp | 325 |
| Adapter (load) | src/native_adapter.cpp | 415 |
| Inputs | src/native_adapter.cpp | 829 |
| Step and outputs | src/native_adapter.cpp | 922 |
| Mid-run changes | src/native_adapter.cpp | 1162 |
| Fixtures and helpers | tests/test_native_adapter.cpp | 26 |
| Joint drives | tests/test_native_adapter.cpp | 193 |
| Friction and limits | tests/test_native_adapter.cpp | 250 |
| Transmissions | tests/test_native_adapter.cpp | 390 |
| Drives through transmissions | tests/test_native_adapter.cpp | 456 |
| Payloads and mid-run changes | tests/test_native_adapter.cpp | 637 |
| Members and ropes | tests/test_native_adapter.cpp | 709 |
| Model change events | tests/test_native_adapter.cpp | 766 |
| JSON reader | tests/test_native_adapter.cpp | 924 |
