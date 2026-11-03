# core/engine: contract

Namespace `mechsim::engine`; q (nq), v (nv). Eigen only, dense (ENG-001); LLT checks H, PartialPivLU solves.

## Headers (`include/mechsim/engine/`)

| Header | Holds |
|---|---|
| `step_config.hpp` | `StepConfig { double h = default_step_s; }`, `default_step_s = 1e-3` s, the only step literal (ENG-002) |
| `state.hpp` | `State { VectorXd q; VectorXd v; double t; }` |
| `dynamics_provider.hpp` | `DynamicsTerms { mass, bias, force, stiffness, damping }`, abstract `DynamicsProvider` |
| `integrator.hpp` | abstract `Integrator`, `SemiImplicitEuler` (ADR 0013 option A, ENG-003) |
| `step_health.hpp` | `StepHealth` (ENG-026) |
| `engine_error.hpp` | `EngineError`, `NonFiniteStateError`, `SolverNonConvergenceError` (ENG-025), `InvalidModelError` (ENG-049), `InvalidConfigError` (ENG-050) |

## Signatures

| Type | Member |
|---|---|
| `DynamicsProvider` | `Index position_size() const`, `Index velocity_size() const` |
| | `void evaluate(double t, const VectorXd& q, const VectorXd& v, DynamicsTerms&) const` |
| | `VectorXd integrate_positions(const VectorXd& q, const VectorXd& v, double h) const` (pure) |
| | `double project(VectorXd& q, VectorXd& v) const`: no-op hook, returns drift (0) |
| `Integrator` | `const StepConfig& config() const`, `StepHealth step(const DynamicsProvider&, State&)` |
| `SemiImplicitEuler` | `explicit SemiImplicitEuler(StepConfig)`; throws `InvalidConfigError` unless h finite, > 0 |
| `StepHealth` | `max_constraint_drift` (m or rad), `converged`, `iterations`, `time` (end, s), `step` (s) |
| `EngineError` | `EngineError(double time, std::string cause)`, `time()`, `cause()`; subtypes share it |

## Step

`(H + h D + h^2 K) dv = h (f - c) - h^2 K v`; `v += dv`; `q = integrate_positions(q, v, h)`; `project(q, v)`; `t += h`.
K = -df/dq, D = -df/dv; f applied+spring+damper; c Coriolis+gravity+passive.

## Conventions

| Topic | Rule |
|---|---|
| Units | SI everywhere, units per field in headers (ENG-004) |
| Errors | Non-finite dv, q or v: `NonFiniteStateError` (also drift); H not SPD (exact symmetry) or size mismatch: `InvalidModelError`; `time()` is the failing step start; state untouched on throw |
| Determinism | No threads, unordered containers or fast-math in the step path (ENG-023) |

## Package 2a adds

Featherstone provider (CRBA H, RNEA c, quaternion `integrate_positions`); real `project` (ENG-024 drift);
PGS solver with the ENG-025 cap (50), throwing `SolverNonConvergenceError`, filling `iterations`.
