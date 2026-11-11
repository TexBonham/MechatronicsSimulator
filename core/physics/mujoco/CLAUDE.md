# core/physics/mujoco/: contract

`mechsim::physics::mujoco::MujocoAdapter`, target `mechsim_physics_mujoco` (-> `mechsim_kernel`, private
`mujoco::mujoco`), tests `mechsim_physics_mujoco_tests`. Implements `kernel::PhysicsAdapter` (PHY-014, ADR 0014).
MuJoCo 3.9.0, prebuilt Linux x86_64 release fetched by `core/cmake/MuJoCo.cmake` (URL + SHA-256). MuJoCo headers
only here (CI grep, `tests/invariant/test_phy_stubs.py`); the public header names no MuJoCo type (pimpl).

| Call | Does |
|---|---|
| `MujocoAdapter()` | Unloaded; `supported_elements()` = the 20 kinds of `mechsim/physics/mujoco/translate.py` |
| `load(PreparedModel, rate)` | Parses and compiles `engine_artifact` (MJCF from the translation); timestep 1 / rate, implicitfast, autoreset off; refuses kinds outside the list, missing metadata, parse or compile errors |
| `step()` | One `mj_step`; inputs applied for that step, then cleared; BADQPOS/BADQVEL/BADQACC warnings or a non-finite state throw `AdapterError` |
| `outputs()` | Forces of the step taken (`mj_rnePostConstraint`), poses and velocities of the state it produced |
| `health()` | Drift = max active connect / hinge-axis error; iterations = max island `solver_niter`; converged = below the cap |
| `integrator()`, `timestep()` | For PHY-014 checks |

## Outputs

| Field | Source |
|---|---|
| bodies | `xpos`, `xquat` (w first), `mj_objectVelocity` world frame at the body origin |
| joints | per DOF `cdof . cfrc_int`; reaction = `cfrc_int` (parent on child) moved to the body origin; reflected shaft: load torque (below), reaction zero |
| constraints | equality `efc_force` rows (3 per connect; empty when disabled); errors from site positions |
| contacts | `mj_contactForce` in world, on body_a (geom1); world and member bodies are `kernel::kWorldBody` |
| members | start: `cfrc_int` of the first segment moved to the root point; end: roller connect force on the tip, else zero |
| ropes | empty (milestone 3) |
| joint_positions | per neutral joint: revolute/prismatic q - qpos0; reflected shaft (q_driven - qpos0) / c + (r_cmd - r_eff) (slip and play, below); others 0 (PHY-034) |
| joint_drive_torques | one per input drive, in order: the drive equality's `efc_force` (times c on a shaft), 0 when disabled |
| transmissions | per shaft, neutral order: `shaft_load_torque` (PHY-033, the joint's value, explicit torque plus drive torque), `force` = gain tau_load / c (gain eta driving, 1 / eta back-driven), `stretch` 0 (rigid), `power_flow` Static (tau qd = 0), Forward, Backward; `slip` = shaft travel lost to slip (rad), `slipping` = s > 0 |
| payloads | per `mechsim:payloads` entry: body, carrier (`kWorldBody` released), attached = weld active, force = weld `efc_force` rows 0-2 (world, on the payload) |

## Transmissions (reflection, PHY-018, PHY-033)

The shaft body is not in the MJCF; the driven joint carries armature J / c^2 (driven = c * shaft; c = r,
lead / 2 pi, 1 / ratio). Each step the shaft joint's input torque tau becomes a driven-joint force:
F = eta tau / c when driving (tau qd_driven c >= 0, standing still included), F = tau / (eta c) when back-driven.
Outputs: the shaft body's pose and velocity turn q_driven / c, qd_driven / c about its axis on its parent; the shaft
joint's `generalized_force` is the load torque tau_load = tau - J (qd_driven' - qd_driven) / (h c). Body wrenches,
springs and payload changes on a shaft body throw `AdapterError`.

## Faults (PHY-006, milestone 2)

| Call | Does |
|---|---|
| `set_transmission_slip(t, s)` | s in [0, 1] (0 clears), else `AdapterError`; drive rest gets (1 - s) of each commanded increment, `slip_accum += s Delta r_cmd` (kept when cleared); explicit shaft torque scaled by (1 - s), drive force unscaled, `force` = gain (tau_load - s tau_explicit) / c (approximation, ruling 9) |
| backlash | numeric [13] = b / abs(c), shaft rad; play with memory on the drive rest (after slip), band centred on the first rest; torque-driven shafts pass torque without play (approximation) |
| `release_load(p)`, `set_payload_attachment(p, kWorldBody)` | weld `<p>.attach` off; `p` must be a `mechsim:payloads` body |
| `set_payload_attachment(p, carrier)` | weld on at the translated relative pose; any other carrier throws (no re-parenting) |

Shaft output under slip or play: angle + (r_cmd - r_eff), rate + (Delta r_cmd - Delta r_eff) / h (`ShaftMap::receive`).
Steady slip (approximation, recorded): the spring sees K (rest - theta') with theta' the reported end-state shaft
angle, so the stepper's theta + omega h prediction (omega the reported rate v) is exact and leaves no spring offset;
the drive's damper acts on the driven rate, (1 - s) v, so the drive carries a constant offset D s v against the
stepper's damper term (bounded by D v; the stepper's tangent absorbs it in its lead).
A step with no enabled drive on the shaft ends tracking; the next enabled rest restarts there, no increment counted,
band re-centred on rest - slip_accum, `slip_accum` kept (ruling 10).

## Joint drives (PHY-034)

Every shaft joint and every `signals.drive_joints` entry has an inactive one-joint equality `<joint>.drive` (listed
in `mechsim:drives`) on the DOF it moves. `apply_drives` each step: all off, then per enabled drive K_d = K / c^2,
D_d = D / c^2, `eq_data[0]` = rest c - qvel h, d = clamp(Z / (1 + Z), 1e-4, 400 / 401) with Z = (K_d h^2 + D_d h) A0
(A0 = `dof_invweight0`), solref = (-(1 - d) K_d A0, -(1 - d) D_d A0), solimp (d, d, 0.001, 0.5, 2);
envelope Z <= 400, beyond it `AdapterError` (PHY-034; past the clamp backward Euler is lost and from Z ~ 535 the
shifted-rest map diverges). The result is backward Euler, the spring and damper at the step's end state,
(m + D_d h + K_d h^2) a = F - D_d v - K_d (q + v h - rest_d); the read-back `efc_force` is K (rest - q') - D v'. Static sag and steady lag (behind the same step's rest)
exact. Brief M2-C2: the old start-state spring (eq_data = rest c, d = 1 - 1 / (K_d A0 h^2)) left a stepper's rest at
theta + omega h standing omega h ahead of the joint, so its braking drive drove G10's x axis to 0.78 m/s under slip.
Derivation above `apply_drives`.

## Index spaces (neutral file order)

Velocity DOFs: joint order, linear then angular (free: MuJoCo order, translation world, rotation local). Springs:
each revolute, prismatic and planar DOF, then lumped-member nodes root to tip. Transmissions: neutral order. Payloads
and loads are bodies; mass change rescales inertia and reruns `mj_setConst` on scratch data; only `mechsim:payloads`
bodies release or re-attach (no ropes).

## Section index, `src/mujoco_adapter.cpp`

| Section | Line |
|---|---|
| Supported elements | 35 |
| Helpers | 61 |
| Engine | 310 |
| Element maps | 354 |
| Shafts | 532 |
| Outputs | 583 |
| Inputs | 807 |
| Adapter | 940 |
| Mid-run changes | 1085 |

## Build notes

`mujoco_adapter.cpp` defines `MUJOCO_INCLUDE_MJSAN_H_` before `mujoco.h`: the release binary has no ASan stack hooks
and GCC rejects mjsan.h's attributes under `-fsanitize=address`. `.clang-tidy` here adds this directory to the
header filter. Fixtures `tests/fixtures/*.xml` are generated by `tests/unit/test_mjcf_translation.py::fixtures()`
and checked against it.
