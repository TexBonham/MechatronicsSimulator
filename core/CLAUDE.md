# core/: C++ core runtime

One compiled library, one pybind11 binding (R-08, ADR 0005). Own CMake tree.

| Dir | Holds |
|---|---|
| `engine/` | Own physics engine: math core, integrators, multibody, collision, contact (ADR 0001) |
| `kernel/` | Co-simulation kernel: time, rates, sync points, seeds |
| `signals/` | Built-in signal components: sensors, front-ends, rails, safety hardware, operator link, pin bindings |
| `controller/` | Controller runtimes: native shim, Renode bridge |
| `physics/` | `PhysicsAdapter` implementations: `mujoco/` (MuJoCo C API), `native/` (own engine) (ADR 0014); stubs |
| `bindings/` | pybind11 binding exposing the core runtime to Python |
| `viewer_debug/` | Minimal immediate-mode C++ window for engine development (ADR 0007) |

`engine/` is its own CMake subproject and builds alone (R-07, ADR 0001).

## Build and test

| Step | Command |
|---|---|
| Configure | `cmake --preset debug-sanitize` (or `release`) |
| Build | `cmake --build --preset debug-sanitize` |
| Test | `ctest --preset debug-sanitize` |

## Rules

- C++20, Eigen, pybind11; ASan, UBSan, clang-tidy, warnings as errors on every target here (ADR 0005).
- No raw owning pointers across any API.
- Curly braces on all control flow; no ternary operators; no comments unless the why is non-obvious.
- Catch2 tests tag requirement IDs: `[ENG-001]`, `[KER-001]`, `[SIG-001]`, `[CTL-001]`, `[PHY-001]`, `[REC-001]`.

## Plugin tiers (ADR 0005)

| Tier | Language | Path |
|---|---|---|
| Built-in types | C++ | Inside the core runtime |
| User-added, first | Python | Per-tick slow path through the binding |
| User-added, stable | C++ | Promoted here |

Both paths implement one interface and pass the same reference tests.

## Physics interface requirement

Physics publishes live member end-loads every tick and accepts mid-run model changes: spring offsets, stiffness changes, constraint removal, load release (ADR 0010).

## Never imported here

| Excluded | Why |
|---|---|
| Python | Python enters only through `bindings/` |
| Qt | Invariant 9; display code only in `viewer_debug/` |
| MuJoCo | Only in `core/physics/mujoco/` (stepping) and `mechsim/physics/mujoco/` (translation) (ADR 0014) |
| Reference collision library | Test-only differential oracle, never linked into the core runtime (ADR 0002) |

## Contracts

| Package | Contract | Target |
|---|---|---|
| Engine | `engine/CLAUDE.md` | `mechsim_engine` |
| Kernel: time, `Party`, `Kernel`, `PhysicsAdapter`, `RecorderSink` | `kernel/CLAUDE.md` | `mechsim_kernel` |
| Signals: `SignalComponent`, `PinBindingTable`, `PowerRail` | `signals/CLAUDE.md` | `mechsim_signals` -> kernel |
| Controller: `ControllerRuntime`, `PinInputs`/`PinOutputs` | `controller/CLAUDE.md` | `mechsim_controller` -> signals |
