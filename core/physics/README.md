Implementations of the core `PhysicsAdapter` interface (`core/kernel/include/mechsim/kernel/physics_adapter.hpp`,
ADR 0014). Engine types stay inside each subdirectory (invariant 5, PHY-011). The MuJoCo bridge was retired (ADR 0004); its
history is in `HISTORY.md`.

| Dir | Adapter |
|---|---|
| `native/` | Steps the own engine in `core/engine/` |
