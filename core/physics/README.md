Implementations of the core `PhysicsAdapter` interface (`core/kernel/include/mechsim/kernel/physics_adapter.hpp`,
ADR 0014). Engine types stay inside each subdirectory (invariant 5, PHY-011).

| Dir | Adapter |
|---|---|
| `mujoco/` | Steps MuJoCo through its C API; MJCF comes from `mechsim/physics/mujoco/` |
| `native/` | Steps the own engine in `core/engine/` |
