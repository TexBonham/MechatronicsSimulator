# core/physics/native/: contract

`mechsim::physics::native::NativeAdapter`, target `mechsim_physics_native` (-> `mechsim_kernel`, private
`mechsim_engine`), tests `mechsim_physics_native_tests` (ctest names `native.*`, reference `reference.native.*`).
Implements `kernel::PhysicsAdapter` over `engine::MultibodyProvider` + `SemiImplicitEuler` (PHY-020, ADR 0013, ADR
0014). The public header names no engine type (pimpl); registered as `"native"` in `core/bindings/src/factories.cpp`.

| Call | Does |
|---|---|
| `load(PreparedModel, rate)` | Parses `neutral_model_json` = the engine model document (`src/json.hpp` reader, format `mechsim-native-1`, written by `mechsim/physics/native/translate.py`); `engine_artifact` must be empty; refuses element kinds outside `supported_elements()` (21: the bridge's 20 + `rope`); builds bodies, constraints, `add_lumped_beam`, transmissions, `add_rope` (end points at the start configuration), contact model, starts, payload welds (a payload follows its carrier's start); h = 1 / rate; any failure `AdapterError(0, ...)` |
| `step()` | `provider.step(ExtendedIntegrator)`; `EngineError` -> `AdapterError(steps, "engine: ...")` (PHY-009); collects the step's `events()` |
| `set_inputs()` | Generalized forces in neutral DOF order (free joints linear first; engine angular first), shaft DOFs through `set_shaft_torque`; body wrenches; joint drives (revolute/prismatic, one per joint, finite, >= 0); engine refusals -> `AdapterError` |
| `outputs()` | Neutral bodies only; shaft joint force = `shaft_load_torque` (PHY-033); constraints neutral only; contact bodies outside the neutral list -> `kWorldBody`; members in the member root frame (ruling 24); ropes `tension` = max segment, `slack` = load end zero; transmissions `slip` = slip offset / n; joint drive torque K (rest - q') - D v' |
| `set_spring_offset/_stiffness(i)` | Spring order: each revolute, prismatic, planar DOF, then member nodes root to tip; joint springs through the extras, member nodes through the engine |
| `disable_constraint(c)` | Neutral constraints only |
| `release_load(i)` | Payload body index: weld released; body count + r: rope r; second release ends the run |
| `set_payload_mass/attachment` | Payload bodies only; `kWorldBody` releases |
| `set_transmission_slip(t, s)` | s in [0, 1] -> `set_slip` |
| `integrator()`, `timestep()`, `events()`, `rope_segment_tensions()`, `rope_load_end_tensions()` | Extras for tests and later briefs (PHY-004, I2) |

## Extras (`ExtendedProvider`, implicit through K and D)

| Term | Form |
|---|---|
| Joint drive (PHY-034) | f -= K (q - rest) + D v, K, D on the diagonal: backward Euler, static sag and lag D u / K exact |
| Joint spring change | -(K1 - K0)(q - rest0) + K1 offset, stiffness K1 - K0 |
| Coulomb friction (PHY-027, PHY-030) | -F sign(v) sliding; below h F / m the damper F / max(v_s, abs(v)); v_s = engine `friction_slip_threshold` |
| Limits (PHY-027) | One-sided K = m w^2, D = 2 m w, w = 1 / (2 h), m = H_kk; active when q or q + h v is past the limit |

## Section index

| Section | File | Line |
|---|---|---|
| Supported elements | src/native_adapter.cpp | 38 |
| Helpers | src/native_adapter.cpp | 69 |
| Joint extras | src/native_adapter.cpp | 212 |
| Engine | src/native_adapter.cpp | 351 |
| Adapter (load) | src/native_adapter.cpp | 429 |
| Inputs | src/native_adapter.cpp | 824 |
| Step and outputs | src/native_adapter.cpp | 900 |
| Mid-run changes | src/native_adapter.cpp | 1072 |
| Fixtures and helpers | tests/test_native_adapter.cpp | 25 |
| Joint drives | tests/test_native_adapter.cpp | 183 |
| Transmissions | tests/test_native_adapter.cpp | 280 |
| Payloads and mid-run changes | tests/test_native_adapter.cpp | 346 |
| Members and ropes | tests/test_native_adapter.cpp | 418 |
