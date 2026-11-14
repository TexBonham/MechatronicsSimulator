# mechsim/physics/: contract

Adapters `mujoco/` (MJCF, ADR 0014), `native/` (`adapter.py`: re-export skeleton until PHY-020; the only Python importer of `_core_ext.engine`); `neutral.py` is the engine-neutral model (ADR 0003).

## neutral.py

Frozen pydantic v2, `extra="forbid"`. Files: unit strings; models: SI floats; dumps write SI unit strings.
Names `^[a-z][a-z0-9_]*$` (catalog pattern); "world" reserved.

| Name | Does |
|---|---|
| `PhysicsModel` | schema_version 1, design (name, semver, blob), gravity, run, world_shapes, contacts, bodies (min 1), constraints, lumped_members, transmissions, signals, structural_members, ropes, fastenings; `fastening_index(name)` |
| `RunConfig` | step 1 ms, iteration_cap 50, projection 3, drift 1e-6 m/rad, symmetry and unit-quaternion 1e-12, velocity tolerance 1e-9 m/s and rad/s (engine takes one) |
| `Body` | mass > 0, com, PSD `InertiaTensor` about com, material, parent, `Joint`, shapes (no plane), catalog_part |
| `Joint` | six types; position, orientation (w-first), axis; `linear`/`angular` empty or exact per `DOF_COUNTS`; spherical, free: no stiffness, rest, limits |
| Shapes (`shape`) | sphere, capsule, box, cylinder, convex_hull (spans a volume), plane (world only) |
| `ContactModel` | default stiffness, damping, friction; material-pair overrides; body-pair exclusions |
| Constraints (`type`) | point, hinge; extend the `Constraint` union |
| `LumpedMember` | `Section` (E, I, rho, A, L), clamped/pinned root, free/roller end, segments 8 (ruling 17), roller 1e-3, node_damping |
| Transmissions (`type`) | belt_pulley, lead_screw, gear_pair; shaft revolute, driven per `DRIVEN_JOINT_TYPES`; slip 0; `damping` at the output, default 0 (N s/m prismatic, N m s/rad revolute; PHY-037) |
| `SignalInterface` | load_torque_joints, pose_bodies, drive_joints (revolute/prismatic joints the signal layer may drive besides shaft joints, PHY-034) |
| `StructuralMember` | section, material, supports (min 1, within L), fasteners (opaque), `mounts` (fixed joints on the member, BLD-008) |
| `Rope` | ADR 0016 lumped chain (BLD-032): name, from_body/from_point, to_body/to_point ("world" allowed, not the same body), segments >= 1, axial_stiffness EA (N), damping (strain-rate, N s), mass_per_length (kg/m), rest_length, optional breaking_tension (absent: never snaps, ruling 19); shares the element namespace; element kind `rope` |
| `Fastening` | BLD-031: name, kind `mount` (a fixed joint, named after it) or `fastener` (a body's fastener entry), body_a, body_b, the held `joint`; own namespace; addressed by index (`fastening_index`), not an element kind |
| `from_yaml`, `to_yaml`, `schema_text` | load, deterministic dump, JSON Schema with `$schema`/`$id` |

`PhysicsModel` cross-checks names and references and lists every failure in one error.

Regenerate after any model change: `uv run python -m mechsim.physics.neutral --write-schema` (staleness test
`tests/invariant/test_schema_physics_model.py`). Example: `schemas/examples/physics_model.pendulum.yaml`.

## Section index, `neutral.py`

| Section | Line |
|---|---|
| Field types | 32 |
| Run configuration | 174 |
| Joints | 204 |
| Bodies and collision shapes | 292 |
| Constraints | 419 |
| Contacts | 449 |
| Lumped members | 474 |
| Transmissions | 518 |
| Signals and structural members | 562 |
| Physics model | 605 |
| Files and schema | 732 |
