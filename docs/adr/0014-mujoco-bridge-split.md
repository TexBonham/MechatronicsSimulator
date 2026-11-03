# ADR 0014: MuJoCo bridge split between Python translation and C++ stepping

Date: 2026-11-03
Status: Accepted (owner ruling 2026-11-03)
Closes rulings: refines R-11 and ADR 0004 for the C++ core runtime (ADR 0005)
Supersedes: ADR 0004, Decision table "Role 1: bridge" as to where the adapter runs; architecture doc layout entry
`mechsim/physics/mujoco/ (bridge)` as the only location

## Context
ADR 0005 makes the kernel part of the compiled core runtime, stepping physics every tick. ADR 0004 placed the MuJoCo
bridge adapter in Python under `mechsim/physics/mujoco/`. A C++ kernel calling a Python adapter once per tick puts an
interpreter crossing on the hot path for the whole bridge period, the very cost ADR 0005 removed for the signal layer.
MuJoCo ships a C library, so stepping it does not need Python.

## Decision

| Part | Location | Language | Content |
|---|---|---|---|
| Translation | `mechsim/physics/mujoco/` | Python | Neutral physics model to MJCF, element by element (PHY-013, PHY-017); build-time only |
| Stepping | `core/physics/mujoco/` | C++ | Implements the core `PhysicsAdapter` interface (defined in `core/kernel/`, phase 1 brief C5) over MuJoCo's C API: load the MJCF, step with implicitfast at the configured step, publish poses, velocities, forces, member end-loads and health, accept inputs and mid-run changes |
| Own engine | `core/engine/` with its adapter in `core/physics/native/` | C++ | Implements the same `PhysicsAdapter` |
| Python view | `mechsim/physics/native/` | Python | Builder-side mapping for the own engine (neutral model passes through unchanged) |

MuJoCo's C library arrives through CMake at a pinned release when stepping is built (milestone 1), never before. MuJoCo
headers and types are allowed only under `core/physics/mujoco/`; the Python `mujoco` module only under
`mechsim/physics/mujoco/`. The import-boundary checker covers the Python side today; a CI include check for the C++
side lands with the stepping adapter.

## Consequences
- No interpreter crossing per tick in either engine mode; the kernel sees one C++ interface.
- Retirement (ADR 0004) deletes `core/physics/mujoco/` and `mechsim/physics/mujoco/`; nothing else changes.
- Invariant 5's boundary is now two directories per engine, both named here; PHY-011 is read accordingly.
- The `PhysicsAdapter` C++ interface is a phase 1 contract (brief C5) and must carry every operation ENG-027, ENG-028,
  ENG-030 to ENG-035 and the PHY interface rows require, so both adapters implement one surface.
- The differential oracle compares two C++ adapters on the same neutral model; the Python translation is tested on its
  own by validating the MJCF it emits.

## Open items
- MuJoCo release to pin and the fetch mechanism: milestone 1 brief for the stepping adapter.
- C++ include-boundary check in CI: same brief.
